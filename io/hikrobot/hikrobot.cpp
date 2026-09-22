#include "hikrobot.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include "MvCameraControl.h"

namespace io
{
namespace
{
// [9.9-6] 必要设置检查返回码，错误中保留节点名称，便于 Pi 上定位不支持的功能。
void check(int status, const std::string & operation)
{
  if (status != MV_OK)
    throw std::runtime_error(fmt::format("{} failed: 0x{:08x}", operation, static_cast<unsigned>(status)));
}

void set_enum(void * handle, const char * key, const std::string & value)
{
  check(MV_CC_SetEnumValueByString(handle, key, value.c_str()), std::string(key) + "=" + value);
}

std::int64_t get_int(void * handle, const char * key)
{
  MVCC_INTVALUE_EX result{};
  check(MV_CC_GetIntValueEx(handle, key, &result), std::string("read ") + key);
  return result.nCurValue;
}

// [Pi相机B-抽取] 青大真机驱动通过数值枚举访问 DecimationHorizontal/Vertical。
// 原整数 API 在 Pi 返回 0x80000109；改用对应类型，并继续严格检查写入和回读。
void set_enum_number(void * handle, const char * key, unsigned int value)
{
  check(MV_CC_SetEnumValue(handle, key, value), fmt::format("{}={}", key, value));
  MVCC_ENUMVALUE result{};
  check(MV_CC_GetEnumValue(handle, key, &result), std::string("read ") + key);
  if (result.nCurValue != value)
    throw std::runtime_error(fmt::format("{} requested {}, read back {}", key, value, result.nCurValue));
}

float set_float(void * handle, const char * key, double value)
{
  check(MV_CC_SetFloatValue(handle, key, static_cast<float>(value)), key);
  MVCC_FLOATVALUE result{};
  check(MV_CC_GetFloatValue(handle, key, &result), std::string("read ") + key);
  // 浮点节点可能按设备步进量化，容许 1%；更大的偏差不能当成设置成功。
  if (!std::isfinite(result.fCurValue) ||
      std::abs(result.fCurValue - value) > std::max(0.001, std::abs(value) * 0.01))
    throw std::runtime_error(fmt::format("{} requested {}, read back {}", key, value, result.fCurValue));
  return result.fCurValue;
}

std::string serial_of(const MV_CC_DEVICE_INFO & device)
{
  const auto & bytes = device.SpecialInfo.stUsb3VInfo.chSerialNumber;
  const auto end = std::find(std::begin(bytes), std::end(bytes), 0);
  return std::string(reinterpret_cast<const char *>(bytes), static_cast<std::size_t>(end - bytes));
}

// [9.9-6] 按成功阶段配对 Stop / Close / Destroy，清理失败也继续释放下一层。
struct Session
{
  void * handle = nullptr;
  bool opened = false;
  bool grabbing = false;
  LatestFrameBuffer & frames;
  std::shared_ptr<spdlog::logger> logger;
  Session(LatestFrameBuffer & buffer, std::shared_ptr<spdlog::logger> log)
  : frames(buffer), logger(std::move(log)) {}
  ~Session()
  {
    frames.invalidate();  // 先撤销旧帧，再执行可能耗时的设备清理。
    const auto cleanup = [&](int status, const char * operation) {
      if (status != MV_OK)
        logger->error("[HikRobot] {} cleanup failed: 0x{:08x}", operation, static_cast<unsigned>(status));
    };
    if (grabbing) cleanup(MV_CC_StopGrabbing(handle), "StopGrabbing");
    if (opened) cleanup(MV_CC_CloseDevice(handle), "CloseDevice");
    if (handle) cleanup(MV_CC_DestroyHandle(handle), "DestroyHandle");
  }
};

struct ImageLease
{
  void * handle;
  MV_FRAME_OUT & raw;
  bool held = true;
  ~ImageLease() { if (held) MV_CC_FreeImageBuffer(handle, &raw); }
  void release()
  {
    held = false;  // Free 失败也不重复释放同一缓冲，交给 Session 清理并重开。
    check(MV_CC_FreeImageBuffer(handle, &raw), "FreeImageBuffer");
  }
};

cv::Size configure(void * handle, const HikRobotOptions & options,
                   const std::shared_ptr<spdlog::logger> & logger)
{
  // [Pi相机B] Pi 自由运行在此设置上返回 MV_E_PARAMETER(0x80000004)。
  // 只在外触发模式选择 FrameStart，避免用无关节点阻挡连续取图；
  // 必需的 TriggerMode=Off 及外触发设置仍严格检查返回码，不吞掉配置错误。
  if (options.external_trigger) set_enum(handle, "TriggerSelector", "FrameStart");
  set_enum(handle, "TriggerMode", "Off");
  set_enum(handle, "AcquisitionMode", "Continuous");
  set_enum(handle, "ExposureAuto", "Off");
  set_enum(handle, "GainAuto", "Off");
  set_enum(handle, "BalanceWhiteAuto", "Continuous");  // 保留 SP25 彩色相机白平衡策略。
  const auto exposure = set_float(handle, "ExposureTime", options.exposure_ms * 1000.0);
  const auto gain = set_float(handle, "Gain", options.gain);
  // [Pi相机B-抽取] 配置倍率不变，仅纠正节点访问类型；Width/Height 仍走整数回读。
  set_enum_number(handle, "DecimationHorizontal", options.decimation_horizontal);
  set_enum_number(handle, "DecimationVertical", options.decimation_vertical);

  check(MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", !options.external_trigger),
        "AcquisitionFrameRateEnable");
  float configured_rate = 0;
  if (options.external_trigger) {
    set_enum(handle, "TriggerSource", options.trigger_source);
    set_enum(handle, "TriggerActivation", options.trigger_activation);
    set_enum(handle, "TriggerMode", "On");
  } else {
    configured_rate = set_float(handle, "AcquisitionFrameRate", options.acquisition_frame_rate);
  }

  const auto width = get_int(handle, "Width");
  const auto height = get_int(handle, "Height");
  if (width <= 0 || height <= 0 || width > std::numeric_limits<int>::max() ||
      height > std::numeric_limits<int>::max())
    throw std::runtime_error("invalid camera Width/Height readback");
  if (options.expected_width && (width != options.expected_width || height != options.expected_height))
    throw std::runtime_error(fmt::format("camera output {}x{}, expected {}x{}; verify decimation/ROI",
      width, height, options.expected_width, options.expected_height));

  // Linux SDK 不支持 SetGrabStrategy/SetOutputQueueSize；限制 SDK 缓存并持续取流。
  check(MV_CC_SetImageNodeNum(handle, 2), "SetImageNodeNum(2)");
  logger->info("[HikRobot] output={}x{}, BGR8, exposure_us={}, gain={}, external_trigger={}, "
               "configured_rate={} (not measured FPS), rotate_180={} (software)", width, height, exposure, gain,
               options.external_trigger, configured_rate, options.rotate_180);  // [Pi方向] 明示实际软件方向设置。
  return {static_cast<int>(width), static_cast<int>(height)};
}

// [9.9-6] 转换写入预分配的 BGR8 槽位；实际像素格式交给 SDK，不猜 Bayer 排列。
void convert_bgr(void * handle, const MV_FRAME_OUT & raw, cv::Mat & image)
{
  const auto bytes = image.total() * image.elemSize();
  if (raw.stFrameInfo.enPixelType == PixelType_Gvsp_BGR8_Packed && raw.stFrameInfo.nFrameLen == bytes) {
    std::memcpy(image.data, raw.pBufAddr, bytes);
    return;
  }
  MV_CC_PIXEL_CONVERT_PARAM_EX conversion{};
  conversion.nWidth = image.cols;
  conversion.nHeight = image.rows;
  conversion.enSrcPixelType = raw.stFrameInfo.enPixelType;
  conversion.pSrcData = raw.pBufAddr;
  conversion.nSrcDataLen = raw.stFrameInfo.nFrameLen;
  conversion.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
  conversion.pDstBuffer = image.data;
  conversion.nDstBufferSize = static_cast<unsigned>(bytes);
  check(MV_CC_ConvertPixelTypeEx(handle, &conversion), "ConvertPixelTypeEx -> BGR8");
  if (conversion.nDstLen != bytes) throw std::runtime_error("SDK BGR output byte count mismatch");
}
}  // namespace

HikRobotOptions HikRobotOptions::from_yaml(const YAML::Node & yaml)
{
  // [9.9-6] 老配置缺省采用 1x1 抽取；Pi 的显式参数放在 configs/camera.yaml。
  HikRobotOptions result;
  const auto read = [&](const char * key, auto & value) {
    if (yaml[key]) value = yaml[key].as<std::decay_t<decltype(value)>>();
  };
  read("exposure_ms", result.exposure_ms);
  read("gain", result.gain);
  read("rotate_180", result.rotate_180);  // [Pi方向] 旧配置缺省不旋转；不推断相机安装朝向。
  read("serial_number", result.serial_number);
  read("external_trigger", result.external_trigger);
  read("trigger_source", result.trigger_source);
  read("trigger_activation", result.trigger_activation);
  read("acquisition_frame_rate", result.acquisition_frame_rate);
  read("decimation_horizontal", result.decimation_horizontal);
  read("decimation_vertical", result.decimation_vertical);
  read("expected_width", result.expected_width);
  read("expected_height", result.expected_height);
  read("pool_size", result.pool_size);
  read("grab_timeout_ms", result.grab_timeout_ms);
  read("read_timeout_ms", result.read_timeout_ms);
  read("reconnect_delay_ms", result.reconnect_delay_ms);
  read("reconnect_after_timeouts", result.reconnect_after_timeouts);
  result.validate();
  return result;
}

void HikRobotOptions::validate() const
{
  if (!std::isfinite(exposure_ms) || exposure_ms <= 0 || exposure_ms > 1000 ||
      !std::isfinite(gain) || gain < 0 || gain > 1000 ||
      !std::isfinite(acquisition_frame_rate) || acquisition_frame_rate <= 0 || acquisition_frame_rate > 1000)
    throw std::invalid_argument("invalid HikRobot exposure_ms / gain / acquisition_frame_rate");
  if (decimation_horizontal < 1 || decimation_vertical < 1 ||
      decimation_horizontal > 16 || decimation_vertical > 16)
    throw std::invalid_argument("camera decimation must be in [1, 16]; device support also required");
  if (expected_width < 0 || expected_height < 0 || ((expected_width == 0) != (expected_height == 0)))
    throw std::invalid_argument("expected_width and expected_height must both be zero or positive");
  if (pool_size < 2 || pool_size > 16 || grab_timeout_ms < 1 || grab_timeout_ms > 1000 ||
      read_timeout_ms < 1 || read_timeout_ms > 5000 || reconnect_delay_ms < 1 ||
      reconnect_delay_ms > 60000 || reconnect_after_timeouts < 1 || reconnect_after_timeouts > 1000)
    throw std::invalid_argument("invalid camera pool size or timeout/reconnect configuration");
  if (external_trigger && (trigger_source.empty() || trigger_source == "Software" || trigger_activation.empty()))
    throw std::invalid_argument("external_trigger requires a physical trigger_source and trigger_activation");
}

HikRobot::HikRobot(HikRobotOptions options)
: options_(std::move(options)), frames_(options_.pool_size), logger_(tools::logger())
{
  options_.validate();
  worker_ = std::thread(&HikRobot::run, this);
}

HikRobot::~HikRobot()
{
  {
    std::lock_guard<std::mutex> lock(retry_mutex_);
    quit_ = true;
  }
  frames_.stop();
  retry_.notify_all();
  if (worker_.joinable()) worker_.join();
}

CameraFrame HikRobot::read_frame()
{
  return frames_.read(std::chrono::milliseconds(options_.read_timeout_ms));
}

void HikRobot::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  // [9.9-5] 兼容调用方不持有帧引用，必须深拷贝；超时不能留下上一帧继续瞄准。
  const auto frame = read_frame();
  if (!frame) {
    img.release();
    timestamp = {};
    throw std::runtime_error("HikRobot read timeout; use read_frame() for recoverable reads");
  }
  img = frame->image.clone();
  timestamp = frame->timestamp;
}

CameraStats HikRobot::stats() const { return frames_.stats(); }

void HikRobot::run()
{
  // [9.9-6] 首次选中后固定串号，重连不误连另一台同 VID/PID 相机，也不复位 USB 设备。
  std::string selected_serial = options_.serial_number;
  bool streamed_before = false;
  while (!quit_) {
    try {
      Session session(frames_, logger_);
      MV_CC_DEVICE_INFO_LIST devices{};
      check(MV_CC_EnumDevices(MV_USB_DEVICE, &devices), "EnumDevices(USB)");
      MV_CC_DEVICE_INFO * selected = nullptr;
      for (unsigned i = 0; i < devices.nDeviceNum; ++i) {
        auto * device = devices.pDeviceInfo[i];
        if (!device || device->nTLayerType != MV_USB_DEVICE) continue;
        if (!selected_serial.empty() && serial_of(*device) != selected_serial) continue;
        if (selected) throw std::runtime_error("multiple matching USB cameras; set unique serial_number");
        selected = device;
      }
      if (!selected) throw std::runtime_error("configured USB camera not found");
      selected_serial = serial_of(*selected);
      if (selected_serial.empty()) throw std::runtime_error("USB camera serial_number is empty");
      check(MV_CC_CreateHandle(&session.handle, selected), "CreateHandle");
      check(MV_CC_OpenDevice(session.handle), "OpenDevice");
      session.opened = true;
      const auto size = configure(session.handle, options_, logger_);
      frames_.configure(size);
      check(MV_CC_StartGrabbing(session.handle), "StartGrabbing");
      session.grabbing = true;
      if (streamed_before) frames_.reconnected();
      streamed_before = true;
      logger_->info("[HikRobot] grabbing serial={}", selected_serial);
      int consecutive_timeouts = 0;
      while (!quit_) {
        MV_FRAME_OUT raw{};
        const auto status = MV_CC_GetImageBuffer(session.handle, &raw, options_.grab_timeout_ms);
        // [9.9-5] 主机收帧时间和像素一起发布；设备帧号仅诊断，重连不重置 sequence。
        const auto timestamp = std::chrono::steady_clock::now();
        if (status == static_cast<int>(MV_E_NODATA)) {
          frames_.capture_timeout();
          if (!MV_CC_IsDeviceConnected(session.handle)) throw std::runtime_error("camera disconnected");
          if (!options_.external_trigger && ++consecutive_timeouts >= options_.reconnect_after_timeouts)
            throw std::runtime_error("free-run camera repeatedly timed out");
          continue;  // 外触发未到脉冲是正常等待，不循环重置相机。
        }
        check(status, "GetImageBuffer");
        ImageLease lease{session.handle, raw};
        consecutive_timeouts = 0;
        if (quit_) break;
        const auto & info = raw.stFrameInfo;
        const unsigned width = info.nExtendWidth ? info.nExtendWidth : info.nWidth;
        const unsigned height = info.nExtendHeight ? info.nExtendHeight : info.nHeight;
        if (!raw.pBufAddr || !info.nFrameLen || info.nLostPacket ||
            width != static_cast<unsigned>(size.width) || height != static_cast<unsigned>(size.height))
          throw std::runtime_error("incomplete frame or actual image size differs from camera readback");
        auto frame = frames_.acquire(timestamp, info.nFrameNum);
        if (frame) convert_bgr(session.handle, raw, frame->image);
        lease.release();  // 槽位满同样立即归还 SDK 缓冲，不等待消费者。
        // [Pi方向] 对尚未发布的独占BGR槽位原地旋转；先归还SDK缓冲，再将同一正向帧交给所有消费者。
        // 不只旋转网页，也不修改已交付帧；尺寸、BGR通道、sequence和主机收帧时间戳保持原定义。
        if (frame && options_.rotate_180) cv::flip(frame->image, frame->image, -1);
        if (frame) frames_.publish(std::move(frame));
      }
    } catch (const std::exception & error) {
      if (!quit_) logger_->warn("[HikRobot] {}; retry in {} ms", error.what(), options_.reconnect_delay_ms);
    }
    std::unique_lock<std::mutex> lock(retry_mutex_);
    retry_.wait_for(lock, std::chrono::milliseconds(options_.reconnect_delay_ms), [&] { return quit_.load(); });
  }
}
}  // namespace io
