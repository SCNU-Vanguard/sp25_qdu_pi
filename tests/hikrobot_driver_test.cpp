// [9.9-6] 使用仓库 SDK 原始头文件提供测试替身，检验生产驱动的配置、异常与资源配对。
#include "io/hikrobot/hikrobot.hpp"
#include "MvCameraControl.h"

#include <algorithm>  // [Pi相机B] 核对故障场景中实际调用过的 SDK 节点。
#include <array>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <iostream>
#include <map>
#include <spdlog/sinks/null_sink.h>
#include <stdexcept>
#include <vector>

using namespace std::chrono_literals;
namespace
{
void require(bool value, const char * message)
{
  if (!value) throw std::runtime_error(message);
}
struct Event
{
  bool bgr = true;
  bool bad_size = false;
  bool bad_conversion_length = false;
  unsigned char value = 11;
  bool patterned = false;  // [Pi方向] 非均匀像素可区分180度旋转、单轴镜像和漏旋转。
};
struct Fake
{
  std::mutex mutex;
  std::condition_variable changed;
  std::deque<Event> events;
  std::vector<std::string> calls;
  std::map<std::string, std::string> enums;
  // [Pi相机B-抽取] 按真实节点类型保存数值枚举，不能再让整数 API 接受抽取节点。
  std::map<std::string, unsigned int> numeric_enums;
  std::map<std::string, std::int64_t> ints;
  std::map<std::string, float> floats;
  std::string fail_next;
  std::string rejected_enum;  // [Pi相机B] 持续拒绝指定枚举设置，复现 Pi 的参数错误。
  std::string rejected_enum_read;  // [Pi相机B-抽取] 回读失败与回读值不符分别验证。
  std::string mismatched_enum_read;
  std::thread::id owner;
  bool violation = false;
  bool connected = true;
  bool two_devices = false;
  bool created = false, opened = false, grabbing = false, held = false;
  bool bad_conversion_length = false;
  int gets = 0, frees = 0, starts = 0, stops = 0, opens = 0, closes = 0, creates = 0, destroys = 0;
  unsigned frame_number = 0;
  std::array<unsigned char, 8 * 6 * 3> pixels{};
  std::array<MV_CC_DEVICE_INFO, 2> devices{};

  void reset()
  {
    std::lock_guard<std::mutex> lock(mutex);
    events.clear(); calls.clear(); enums.clear(); ints.clear(); floats.clear(); fail_next.clear();
    rejected_enum.clear();  // [Pi相机B] 不让一个故障场景影响下一个场景。
    // [Pi相机B-抽取] 默认只通过枚举 API 提供横纵抽取节点，清除跨场景故障状态。
    numeric_enums = {{"DecimationHorizontal", 1U}, {"DecimationVertical", 1U}};
    rejected_enum_read.clear(); mismatched_enum_read.clear();
    owner = {}; violation = false; connected = true; two_devices = false;
    created = opened = grabbing = held = bad_conversion_length = false;
    gets = frees = starts = stops = opens = closes = creates = destroys = 0;
    frame_number = 0;
    ints["Width"] = 8; ints["Height"] = 6;
    for (int i = 0; i < 2; ++i) {
      devices[i] = {};
      devices[i].nTLayerType = MV_USB_DEVICE;
      std::strcpy(reinterpret_cast<char *>(devices[i].SpecialInfo.stUsb3VInfo.chSerialNumber),
                  i == 0 ? "camera-A" : "camera-B");
    }
  }
  int call(const std::string & operation)
  {
    if (owner == std::thread::id{}) owner = std::this_thread::get_id();
    if (owner != std::this_thread::get_id()) violation = true;
    calls.push_back(operation);
    changed.notify_all();
    if (operation == fail_next) { fail_next.clear(); return MV_E_PARAMETER; }
    return MV_OK;
  }
  void frame(Event event = {})
  {
    std::lock_guard<std::mutex> lock(mutex);
    events.push_back(event);
    changed.notify_all();
  }
  template <class F> void wait(F predicate)
  {
    std::unique_lock<std::mutex> lock(mutex);
    require(changed.wait_for(lock, 2s, predicate), "fake SDK event wait timed out");
  }
  void balanced()
  {
    std::lock_guard<std::mutex> lock(mutex);
    require(!violation && !created && !opened && !grabbing && !held, "SDK lifecycle/order/thread violation");
    require(creates == destroys && opens == closes && starts == stops && gets == frees,
            "SDK create/open/start/get not paired with cleanup");
  }
} fake;

io::HikRobotOptions options()
{
  io::HikRobotOptions result;
  result.expected_width = 8; result.expected_height = 6;
  result.grab_timeout_ms = 5; result.read_timeout_ms = 100;
  result.reconnect_delay_ms = 5; result.reconnect_after_timeouts = 1000;
  return result;
}
io::CameraFrame receive(io::HikRobot & camera, Event event = {})
{
  fake.frame(event);
  auto frame = camera.read_frame();
  require(bool(frame), "expected simulated camera frame");
  return frame;
}
}  // namespace

namespace tools
{
std::shared_ptr<spdlog::logger> logger()
{
  static auto log = std::make_shared<spdlog::logger>("camera-test", std::make_shared<spdlog::sinks::null_sink_mt>());
  return log;
}
}  // namespace tools

// 以下函数签名必须与真实 SDK 头文件一致；不用生产代码中的虚拟接口掩盖签名错误。
extern "C"
{
int MV_CC_EnumDevices(unsigned int, MV_CC_DEVICE_INFO_LIST * list)
{
  std::lock_guard<std::mutex> lock(fake.mutex);
  const auto status = fake.call("EnumDevices");
  list->nDeviceNum = fake.connected ? (fake.two_devices ? 2 : 1) : 0;
  for (unsigned i = 0; i < list->nDeviceNum; ++i) list->pDeviceInfo[i] = &fake.devices[i];
  return status;
}
int MV_CC_CreateHandle(void ** handle, const MV_CC_DEVICE_INFO * device)
{
  std::lock_guard<std::mutex> lock(fake.mutex);
  const auto status = fake.call("CreateHandle");
  if (status) return status;
  if (fake.created) fake.violation = true;
  fake.enums["selected_serial"] = reinterpret_cast<const char *>(device->SpecialInfo.stUsb3VInfo.chSerialNumber);
  *handle = &fake;
  fake.created = true; ++fake.creates;
  return MV_OK;
}
int MV_CC_OpenDevice(void *, unsigned int, unsigned short)
{
  std::lock_guard<std::mutex> lock(fake.mutex);
  const auto status = fake.call("OpenDevice");
  if (status) return status;
  if (!fake.created || fake.opened) fake.violation = true;
  fake.opened = true; ++fake.opens;
  return MV_OK;
}
int MV_CC_StartGrabbing(void *)
{
  std::lock_guard<std::mutex> lock(fake.mutex);
  const auto status = fake.call("StartGrabbing");
  if (status) return status;
  if (!fake.opened || fake.grabbing) fake.violation = true;
  fake.grabbing = true; ++fake.starts;
  return MV_OK;
}
int MV_CC_StopGrabbing(void *)
{
  std::lock_guard<std::mutex> lock(fake.mutex);
  const auto status = fake.call("StopGrabbing");
  if (!fake.grabbing || fake.held) fake.violation = true;
  fake.grabbing = false; ++fake.stops;
  return status;
}
int MV_CC_CloseDevice(void *)
{
  std::lock_guard<std::mutex> lock(fake.mutex);
  const auto status = fake.call("CloseDevice");
  if (!fake.opened || fake.grabbing || fake.held) fake.violation = true;
  fake.opened = false; ++fake.closes;
  return status;
}
int MV_CC_DestroyHandle(void *)
{
  std::lock_guard<std::mutex> lock(fake.mutex);
  const auto status = fake.call("DestroyHandle");
  if (!fake.created || fake.opened || fake.grabbing || fake.held) fake.violation = true;
  fake.created = false; ++fake.destroys;
  return status;
}
bool MV_CC_IsDeviceConnected(void *)
{
  std::lock_guard<std::mutex> lock(fake.mutex);
  fake.call("IsDeviceConnected");
  return fake.connected;
}
int MV_CC_SetEnumValueByString(void *, const char * key, const char * value)
{
  std::lock_guard<std::mutex> lock(fake.mutex);
  // [Pi相机B] 参数被拒绝时不假装节点已写入；保留调用记录供分支检查。
  const auto status = fake.call(key);
  if (status != MV_OK) return status;
  if (fake.rejected_enum == key) return MV_E_PARAMETER;
  fake.enums[key] = value;
  return MV_OK;
}
// [Pi相机B-抽取] 与真实 SDK 原型一致；设置/回读故障必须能阻止生产驱动启动。
int MV_CC_SetEnumValue(void *, const char * key, unsigned int value)
{
  std::lock_guard<std::mutex> lock(fake.mutex);
  const auto status = fake.call(key);
  if (status != MV_OK) return status;
  if (fake.rejected_enum == key || !fake.numeric_enums.count(key)) return MV_E_PARAMETER;
  fake.numeric_enums[key] = value;
  return MV_OK;
}
int MV_CC_GetEnumValue(void *, const char * key, MVCC_ENUMVALUE * value)
{
  std::lock_guard<std::mutex> lock(fake.mutex);
  const auto status = fake.call(std::string("read ") + key);
  if (status != MV_OK) return status;
  if (fake.rejected_enum_read == key || !fake.numeric_enums.count(key)) return MV_E_PARAMETER;
  value->nCurValue = fake.numeric_enums.at(key) + (fake.mismatched_enum_read == key ? 1U : 0U);
  return MV_OK;
}
int MV_CC_SetIntValueEx(void *, const char * key, int64_t value)
{
  std::lock_guard<std::mutex> lock(fake.mutex);
  // [Pi相机B-抽取] Pi 随库头文件定义 0x80000109 为节点不存在；模拟整数接口查不到枚举节点。
  const auto status = fake.call(key);
  if (status != MV_OK) return status;
  if (fake.numeric_enums.count(key)) return static_cast<int>(0x80000109U);
  fake.ints[key] = value;
  return MV_OK;
}
int MV_CC_GetIntValueEx(void *, const char * key, MVCC_INTVALUE_EX * value)
{
  std::lock_guard<std::mutex> lock(fake.mutex);
  // [Pi相机B-抽取] 读取也必须使用正确类型；Width/Height 继续通过整数接口提供。
  const auto status = fake.call(std::string("read ") + key);
  if (status != MV_OK) return status;
  if (fake.numeric_enums.count(key)) return static_cast<int>(0x80000109U);
  value->nCurValue = fake.ints[key];
  return MV_OK;
}
int MV_CC_SetFloatValue(void *, const char * key, float value)
{
  std::lock_guard<std::mutex> lock(fake.mutex);
  fake.floats[key] = value;
  return fake.call(key);
}
int MV_CC_GetFloatValue(void *, const char * key, MVCC_FLOATVALUE * value)
{
  std::lock_guard<std::mutex> lock(fake.mutex);
  value->fCurValue = fake.floats[key];
  return fake.call(std::string("read ") + key);
}
int MV_CC_SetBoolValue(void *, const char * key, bool value)
{
  std::lock_guard<std::mutex> lock(fake.mutex);
  fake.ints[key] = value;
  return fake.call(key);
}
int MV_CC_SetImageNodeNum(void *, unsigned int count)
{
  std::lock_guard<std::mutex> lock(fake.mutex);
  if (count != 2 || fake.grabbing) fake.violation = true;
  return fake.call("SetImageNodeNum");
}
int MV_CC_GetImageBuffer(void *, MV_FRAME_OUT * raw, unsigned int timeout)
{
  std::unique_lock<std::mutex> lock(fake.mutex);
  const auto status = fake.call("GetImageBuffer");
  if (!fake.grabbing || fake.held) fake.violation = true;
  if (status) return status;
  fake.changed.wait_for(lock, std::chrono::milliseconds(timeout), [&] { return !fake.events.empty(); });
  if (fake.events.empty()) return MV_E_NODATA;
  const auto event = fake.events.front(); fake.events.pop_front();
  for (std::size_t i = 0; i < fake.pixels.size(); i += 3) {
    const auto value = event.value + (event.patterned ? i / 3 : 0);  // [Pi方向] 原场景仍为均匀像素。
    fake.pixels[i] = value;
    fake.pixels[i + 1] = value + 11;
    fake.pixels[i + 2] = value + 22;
  }
  raw->pBufAddr = fake.pixels.data();
  raw->stFrameInfo.nWidth = event.bad_size ? 7 : 8;
  raw->stFrameInfo.nHeight = 6;
  raw->stFrameInfo.nFrameLen = fake.pixels.size();
  raw->stFrameInfo.nFrameNum = ++fake.frame_number;
  raw->stFrameInfo.enPixelType = event.bgr ? PixelType_Gvsp_BGR8_Packed : PixelType_Gvsp_RGB8_Packed;
  fake.bad_conversion_length = event.bad_conversion_length;
  fake.held = true; ++fake.gets;
  return MV_OK;
}
int MV_CC_FreeImageBuffer(void *, MV_FRAME_OUT *)
{
  std::lock_guard<std::mutex> lock(fake.mutex);
  const auto status = fake.call("FreeImageBuffer");
  if (!fake.held) fake.violation = true;
  fake.held = false; ++fake.frees;
  std::fill(fake.pixels.begin(), fake.pixels.end(), 0xdd);  // 归还后 SDK 可以立即覆盖。
  return status;
}
int MV_CC_ConvertPixelTypeEx(void *, MV_CC_PIXEL_CONVERT_PARAM_EX * conversion)
{
  std::lock_guard<std::mutex> lock(fake.mutex);
  const auto status = fake.call("ConvertPixelTypeEx");
  if (status) return status;
  if (!fake.held || conversion->enDstPixelType != PixelType_Gvsp_BGR8_Packed ||
      conversion->nDstBufferSize != fake.pixels.size()) fake.violation = true;
  for (unsigned i = 0; i < conversion->nDstBufferSize; i += 3) {
    conversion->pDstBuffer[i] = conversion->pSrcData[i + 2];
    conversion->pDstBuffer[i + 1] = conversion->pSrcData[i + 1];
    conversion->pDstBuffer[i + 2] = conversion->pSrcData[i];
  }
  conversion->nDstLen = conversion->nDstBufferSize - (fake.bad_conversion_length ? 1 : 0);
  return MV_OK;
}
}  // extern "C"

int main()
{
  try {
    const auto yaml_options = io::HikRobotOptions::from_yaml(YAML::Load(
      "exposure_ms: 2.5\ngain: 8\npool_size: 3\nexternal_trigger: true\ntrigger_source: Line2\n"
      "trigger_activation: FallingEdge\ndecimation_horizontal: 2\ndecimation_vertical: 2\n"));
    require(yaml_options.exposure_ms == 2.5 && yaml_options.pool_size == 3 &&
            yaml_options.trigger_source == "Line2", "YAML options not applied");
    // [Pi方向] 默认兼容未旋转配置；显式bool必须能读取，非法类型不能默默忽略。
    require(!yaml_options.rotate_180 &&
            io::HikRobotOptions::from_yaml(YAML::Load("rotate_180: true")).rotate_180 &&
            !io::HikRobotOptions::from_yaml(YAML::Load("rotate_180: false")).rotate_180,
            "rotate_180 YAML default/explicit option not applied");
    for (const auto * yaml : {"pool_size: 1", "gain: .nan", "grab_timeout_ms: 0", "expected_width: 720",
                             "external_trigger: true\ntrigger_source: Software",
                             "rotate_180: sideways"}) {  // [Pi方向] 拒绝非bool方向配置。
      bool rejected = false;
      try { io::HikRobotOptions::from_yaml(YAML::Load(yaml)); }
      catch (const std::exception &) { rejected = true; }
      require(rejected, "invalid YAML options accepted");
    }

    fake.reset();
    io::CameraFrame survivor;
    {
      io::HikRobot camera(options());
      survivor = receive(camera);
      require(survivor->image.at<cv::Vec3b>(0, 0) == cv::Vec3b(11, 22, 33), "BGR copy/lifetime wrong");
      auto converted = receive(camera, {false});
      require(converted->image.at<cv::Vec3b>(0, 0) == cv::Vec3b(33, 22, 11), "SDK RGB -> BGR contract wrong");
      require(converted->sequence > survivor->sequence, "sequence not increasing");
      fake.frame();
      cv::Mat owned;
      std::chrono::steady_clock::time_point stamp;
      camera.read(owned, stamp);
      for (int i = 0; i < 6; ++i) receive(camera, {true, false, false, 77});
      require(owned.at<cv::Vec3b>(0, 0) == cv::Vec3b(11, 22, 33), "legacy Mat not independently owned");
      bool timeout = false;
      try { camera.read(owned, stamp); } catch (const std::runtime_error &) { timeout = true; }
      require(timeout && owned.empty() && stamp == std::chrono::steady_clock::time_point{},
              "legacy timeout left a stale frame");
    }
    fake.balanced();
    require(survivor->image.at<cv::Vec3b>(0, 0)[0] == 11, "frame invalid after camera destructor");
    survivor.reset();

    // [Pi方向] 两种像素来源×旋转开关，逐像素核对方向和BGR通道；重连后仍应用同一设置，
    // 已交付帧在SDK归还/重连/新帧到来后保持不变，防止为画面转正破坏槽位引用契约。
    for (const bool rotated : {false, true}) {
      for (const bool bgr : {true, false}) {
        fake.reset();
        {
          auto config = options();
          config.rotate_180 = rotated;
          io::HikRobot camera(config);
          io::CameraFrame previous;
          cv::Mat previous_pixels;
          for (int pass = 0; pass < 2; ++pass) {
            const auto value = static_cast<unsigned char>(11 + 30 * pass);
            const auto frame = receive(camera, {bgr, false, false, value, true});
            require(frame->image.size() == cv::Size(8, 6) && frame->image.type() == CV_8UC3,
                    "rotation changed image size/type");
            for (int y = 0; y < 6; ++y) {
              for (int x = 0; x < 8; ++x) {
                const int source_x = rotated ? 7 - x : x;
                const int source_y = rotated ? 5 - y : y;
                const int first = value + source_y * 8 + source_x;
                const cv::Vec3b expected = bgr ? cv::Vec3b(first, first + 11, first + 22)
                                               : cv::Vec3b(first + 22, first + 11, first);
                require(frame->image.at<cv::Vec3b>(y, x) == expected,
                        "rotate_180 pixel mapping/channel order wrong");
              }
            }
            require(frame->timestamp != std::chrono::steady_clock::time_point{}, "rotation lost timestamp");
            if (previous) {
              require(frame->sequence > previous->sequence && frame->timestamp >= previous->timestamp &&
                      camera.stats().reconnects >= 1, "rotation reconnect lost metadata");
              require(cv::norm(previous->image, previous_pixels, cv::NORM_INF) == 0,
                      "rotation modified an already delivered frame");
            } else {
              previous = frame;
              previous_pixels = frame->image.clone();
              { std::lock_guard<std::mutex> lock(fake.mutex); fake.fail_next = "FreeImageBuffer"; }
              fake.frame();
              fake.wait([] { return fake.starts >= 2; });
            }
          }
        }
        fake.balanced();
      }
    }

    // [Pi相机B] 复现 TriggerSelector 持续返回 0x80000004：自由运行仍须能取到帧，
    // 且必须关闭触发、启用连续采集，不设置当前模式用不到的触发选择项/信号源。
    fake.reset(); fake.rejected_enum = "TriggerSelector";
    {
      io::HikRobot camera(options());
      receive(camera);
      std::lock_guard<std::mutex> lock(fake.mutex);
      require(std::find(fake.calls.begin(), fake.calls.end(), "TriggerSelector") == fake.calls.end() &&
              fake.enums["TriggerMode"] == "Off" && fake.enums["AcquisitionMode"] == "Continuous" &&
              !fake.enums.count("TriggerSource") && !fake.enums.count("TriggerActivation"),
              "free-run depends on trigger-only nodes or failed to disable triggering");
    }
    fake.balanced();

    // [Pi相机B] 不得把所有节点错误都吞掉：自由运行的 TriggerMode 和外触发的
    // TriggerSelector 仍是必要设置；持续失败时不准 StartGrabbing，并完整清理句柄。
    for (const auto * rejected : {"TriggerMode", "TriggerSelector"}) {
      fake.reset(); fake.rejected_enum = rejected;
      {
        auto config = options();
        config.external_trigger = fake.rejected_enum == "TriggerSelector";
        io::HikRobot camera(config);
        require(!camera.read_frame(), "required trigger setting failure fabricated a frame");
        std::lock_guard<std::mutex> lock(fake.mutex);
        require(fake.starts == 0 &&
                std::find(fake.calls.begin(), fake.calls.end(), rejected) != fake.calls.end(),
                "required trigger setting failure was ignored");
      }
      fake.balanced();
    }

    // [Pi相机B-抽取] 横纵使用不同值，确认两个枚举分别设置并回读；不证明真机支持 2x3。
    fake.reset();
    {
      auto config = options();
      config.decimation_horizontal = 2; config.decimation_vertical = 3;
      io::HikRobot camera(config);
      receive(camera);
      std::lock_guard<std::mutex> lock(fake.mutex);
      require(fake.numeric_enums.at("DecimationHorizontal") == 2 &&
              fake.numeric_enums.at("DecimationVertical") == 3 &&
              !fake.ints.count("DecimationHorizontal") && !fake.ints.count("DecimationVertical") &&
              std::find(fake.calls.begin(), fake.calls.end(), "read DecimationHorizontal") != fake.calls.end() &&
              std::find(fake.calls.begin(), fake.calls.end(), "read DecimationVertical") != fake.calls.end(),
              "decimation enum values/readback not applied independently");
    }
    fake.balanced();

    // [Pi相机B-抽取] 任一轴设置失败、回读失败或回读不符，均不得启动或泄漏设备资源。
    for (const auto * node : {"DecimationHorizontal", "DecimationVertical"}) {
      for (int failure = 0; failure < 3; ++failure) {
        fake.reset();
        if (failure == 0) fake.rejected_enum = node;
        if (failure == 1) fake.rejected_enum_read = node;
        if (failure == 2) fake.mismatched_enum_read = node;
        {
          auto config = options();
          config.decimation_horizontal = 2; config.decimation_vertical = 2;
          io::HikRobot camera(config);
          require(!camera.read_frame(), "failed decimation setting fabricated a frame");
          std::lock_guard<std::mutex> lock(fake.mutex);
          const auto operation = failure == 0 ? std::string(node) : std::string("read ") + node;
          require(fake.starts == 0 &&
                  std::find(fake.calls.begin(), fake.calls.end(), operation) != fake.calls.end(),
                  "decimation setting/readback failure was ignored or not exercised");
        }
        fake.balanced();
      }
    }

    // 各初始化阶段失败后必须清理并重新配置，不能 Destroy 未创建的句柄。
    for (const auto * operation : {"CreateHandle", "OpenDevice", "DecimationHorizontal",
                                  "read Width", "SetImageNodeNum", "StartGrabbing"}) {
      fake.reset(); fake.fail_next = operation;
      { io::HikRobot camera(options()); receive(camera); }
      fake.balanced();
    }

    // 成功取到 SDK 缓冲后，转换失败、长度错误、尺寸错误和 Free 失败均不能泄漏/重复释放。
    for (int failure = 0; failure < 4; ++failure) {
      fake.reset();
      {
        io::HikRobot camera(options());
        const auto before = receive(camera);
        {
          std::lock_guard<std::mutex> lock(fake.mutex);
          if (failure == 0) fake.fail_next = "ConvertPixelTypeEx";
          if (failure == 3) fake.fail_next = "FreeImageBuffer";
        }
        fake.frame({failure != 0 && failure != 1, failure == 2, failure == 1});
        fake.wait([] { return fake.starts >= 2; });
        const auto after = receive(camera);
        require(after->sequence > before->sequence && camera.stats().reconnects >= 1,
                "reconnect lost sequence or was not counted");
      }
      fake.balanced();
    }

    fake.reset();
    {
      auto config = options(); config.external_trigger = true;
      config.reconnect_after_timeouts = 1;
      config.trigger_source = "Line2"; config.trigger_activation = "FallingEdge";
      config.decimation_horizontal = 2; config.decimation_vertical = 2;
      io::HikRobot camera(config);
      require(!camera.read_frame(), "external trigger fabricated a frame");
      {
        std::lock_guard<std::mutex> lock(fake.mutex);
        // [Pi相机B] 外触发成功路径仍明确选择帧触发，不能因自由运行修复而删掉。
        require(fake.starts == 1 && fake.enums["TriggerSelector"] == "FrameStart" &&
                fake.enums["TriggerMode"] == "On" &&
                fake.enums["TriggerSource"] == "Line2" && fake.enums["TriggerActivation"] == "FallingEdge" &&
                // [Pi相机B-抽取] 外触发同样通过枚举接口配置两轴。
                fake.numeric_enums.at("DecimationHorizontal") == 2 &&
                fake.numeric_enums.at("DecimationVertical") == 2 &&
                fake.ints["AcquisitionFrameRateEnable"] == 0 &&
                !fake.floats.count("AcquisitionFrameRate"), "external trigger configuration/retry wrong");
        fake.connected = false;
      }
      fake.wait([] { return fake.destroys >= 1; });
      {
        std::lock_guard<std::mutex> lock(fake.mutex);
        fake.connected = true;
        fake.two_devices = true;  // 重连时多了一台设备，仍应锁定原串号。
      }
      receive(camera);
      std::lock_guard<std::mutex> lock(fake.mutex);
      require(fake.enums["selected_serial"] == "camera-A", "reconnect switched camera identity");
    }
    fake.balanced();

    fake.reset();
    {
      auto config = options(); config.reconnect_after_timeouts = 1;
      io::HikRobot camera(config);
      fake.wait([] { return fake.starts >= 2; });
      require(camera.stats().capture_timeouts > 0, "free-run timeout not counted");
    }
    fake.balanced();

    fake.reset(); fake.two_devices = true;
    {
      io::HikRobot camera(options());
      require(!camera.read_frame(), "ambiguous camera selection accepted");
      std::lock_guard<std::mutex> lock(fake.mutex);
      require(fake.creates == 0, "ambiguous device opened");
    }
    fake.balanced();
    fake.reset(); fake.two_devices = true;
    {
      auto config = options(); config.serial_number = "camera-B";
      io::HikRobot camera(config); receive(camera);
      std::lock_guard<std::mutex> lock(fake.mutex);
      require(fake.enums["selected_serial"] == "camera-B", "explicit serial not selected");
    }
    fake.balanced();

    fake.reset();
    {
      auto config = options(); config.expected_width = 720; config.expected_height = 540;
      io::HikRobot camera(config);
      require(!camera.read_frame(), "output size mismatch was hidden by resizing");
      std::lock_guard<std::mutex> lock(fake.mutex);
      require(fake.starts == 0, "started with incompatible output size");
    }
    fake.balanced();

    for (const auto * operation : {"StopGrabbing", "CloseDevice", "DestroyHandle"}) {
      fake.reset();
      {
        io::HikRobot camera(options()); receive(camera);
        std::lock_guard<std::mutex> lock(fake.mutex);
        fake.fail_next = operation;
      }
      fake.balanced();  // 清理某一层失败仍调用后续层，不早退。
    }

    // 退出打断较长重连等待，而不是每次析构额外卡住数秒。
    fake.reset(); fake.connected = false;
    const auto begin = std::chrono::steady_clock::now();
    {
      auto config = options(); config.reconnect_delay_ms = 5000;
      io::HikRobot camera(config);
      require(!camera.read_frame(), "disconnected camera returned frame");
    }
    require(std::chrono::steady_clock::now() - begin < 1s, "shutdown did not interrupt reconnect wait");
    fake.balanced();
    std::cout << "HikRobot configuration, BGR, timeout, reconnection and resource lifecycle passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
