#include "hailo_runtime.hpp"

#include <hailo/inference_pipeline.hpp>
#include <hailo/vdevice.hpp>

#include <array>
#include <limits>
#include <map>
#include <stdexcept>
#include <utility>

namespace auto_aim
{
namespace
{
// [9.9-2] 所有 SDK 失败携带操作名、状态码和错误描述，禁止静默返回上帧输出。
void check_status(hailo_status status, const char * operation)
{
  if (status == HAILO_SUCCESS) return;
  const char * message = hailo_get_status_message(status);
  throw std::runtime_error(
    std::string("HailoRT ") + operation + " failed (" + std::to_string(status) + "): " +
    (message ? message : "unknown status"));
}

template <typename T>
T take(hailort::Expected<T> result, const char * operation)
{
  check_status(result.status(), operation);
  return result.release();
}

cv::Size image_size(const hailo_vstream_info_t & info)
{
  if (info.shape.width == 0 || info.shape.height == 0 ||
      info.shape.width > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
      info.shape.height > static_cast<std::uint32_t>(std::numeric_limits<int>::max())) {
    throw std::runtime_error("Invalid Hailo stream dimensions: " + std::string(info.name));
  }
  return {static_cast<int>(info.shape.width), static_cast<int>(info.shape.height)};
}

HailoElementType element_type(hailo_format_type_t type)
{
  switch (type) {
    case HAILO_FORMAT_TYPE_UINT8: return HailoElementType::uint8;
    case HAILO_FORMAT_TYPE_UINT16: return HailoElementType::uint16;
    case HAILO_FORMAT_TYPE_FLOAT32: return HailoElementType::float32;
    default: throw std::runtime_error("Hailo output must be UINT8, UINT16 or FLOAT32");
  }
}

void check_nhwc(const hailo_format_t & format)
{
  if (format.order != HAILO_FORMAT_ORDER_NHWC ||
      (format.flags & HAILO_FORMAT_FLAGS_TRANSPOSED) != 0) {
    throw std::runtime_error("Expected untransposed NHWC host buffers from HailoRT");
  }
}
}  // namespace

struct HailoRuntime::Impl
{
  // [9.9-2] 声明顺序决定逆序析构：先停止激活和数据流，再释放主机缓冲、网络和设备。
  std::unique_ptr<hailort::Hef> hef;
  std::unique_ptr<hailort::VDevice> device;
  std::shared_ptr<hailort::ConfiguredNetworkGroup> network;
  std::unique_ptr<HailoOutput> outputs;
  std::map<std::string, hailort::MemoryView> input_views;
  std::map<std::string, hailort::MemoryView> output_views;
  std::unique_ptr<hailort::InferVStreams> pipeline;
  std::unique_ptr<hailort::ActivatedNetworkGroup> activation;
  hailort::InputVStream * input_stream = nullptr;
  std::array<hailort::OutputVStream *, 3> output_streams{};
  cv::Size size;
  std::string input_stream_name;
  std::size_t input_bytes = 0;
  bool failed = false;

  Impl(const std::string & hef_path, const HailoRuntimeOptions & options)
  {
    if (hef_path.empty() || options.timeout_ms == 0 ||
        options.expected_input_size.width <= 0 || options.expected_input_size.height <= 0 ||
        options.expected_input_size.width % 32 != 0 || options.expected_input_size.height % 32 != 0) {
      throw std::invalid_argument("Expected a HEF path, a positive timeout and input dimensions divisible by 32");
    }

    // [9.9-2][Pi-4.24] 与 CMake 实际采用的 SDK 版本一致，拒绝运行时加载另一版本的库。
    hailo_version_t version{};
    check_status(hailo_get_library_version(&version), "get_library_version");
    const auto actual_version = std::to_string(version.major) + "." +
      std::to_string(version.minor) + "." + std::to_string(version.revision);
    if (actual_version != SP_HAILORT_BUILD_VERSION) {
      throw std::runtime_error(
        std::string("Expected HailoRT ") + SP_HAILORT_BUILD_VERSION + ", found " + actual_version);
    }

    // [9.9-2] HEF 只在构造时解析一次。网络和设备身份按元数据校验，不按文件名推测。
    hef = std::make_unique<hailort::Hef>(take(hailort::Hef::create(hef_path), "load HEF"));
    if (take(hef->get_hef_device_arch(), "get HEF architecture") != HAILO_ARCH_HAILO8L) {
      throw std::runtime_error("This runtime requires a HEF compiled for HAILO8L");
    }
    const auto group_names = hef->get_network_groups_names();
    if (group_names.size() != 1 || take(hef->get_network_infos(), "get networks").size() != 1) {
      throw std::runtime_error("Expected exactly one network group containing one network");
    }
    const auto hef_inputs = take(hef->get_input_vstream_infos(), "get HEF inputs");
    const auto hef_outputs = take(hef->get_output_vstream_infos(), "get HEF outputs");
    if (hef_inputs.size() != 1 || hef_outputs.size() != 3) {
      throw std::runtime_error("Expected one RGB input and three raw detection outputs");
    }
    size = image_size(hef_inputs.front());
    if (size != options.expected_input_size || hef_inputs.front().shape.features != 3) {
      throw std::runtime_error(
        "HEF input mismatch: actual " + std::to_string(size.width) + "x" +
        std::to_string(size.height) + "x" + std::to_string(hef_inputs.front().shape.features) +
        ", expected " + std::to_string(options.expected_input_size.width) + "x" +
        std::to_string(options.expected_input_size.height) + "x3");
    }
    // UINT8 输入接口接收原始 RGB 像素；输入归一化/颜色语义仍需与 HEF 导出流程核对。
    if (hef_inputs.front().format.type != HAILO_FORMAT_TYPE_UINT8) {
      throw std::runtime_error("Expected a UINT8 HEF image input");
    }

    hailo_vdevice_params_t device_params{};
    check_status(hailo_init_vdevice_params(&device_params), "init VDevice parameters");
    device_params.device_count = 1;
    // [9.9-2] 单设备、单网络、单帧；显式激活一次，不在每帧加载或切换网络。
    device_params.scheduling_algorithm = HAILO_SCHEDULING_ALGORITHM_NONE;
    device = take(hailort::VDevice::create(device_params), "create VDevice");
    const auto physical_devices = take(device->get_physical_devices(), "get physical devices");
    if (physical_devices.size() != 1 ||
        take(physical_devices.front().get().get_architecture(), "get device architecture") !=
          HAILO_ARCH_HAILO8L) {
      throw std::runtime_error("Expected exactly one physical HAILO8L device");
    }

    auto config = take(hef->create_configure_params(HAILO_STREAM_INTERFACE_PCIE), "make PCIe configuration");
    for (auto & entry : config) {
      // [Pi-batch] SDK 禁止网络组和网络同时显式指定 batch；组层保持默认，仅网络层设单帧。
      // HAILO_DEFAULT_BATCH_SIZE 是“不在此层指定”的标记，不表示本网络处理零帧。
      entry.second.batch_size = HAILO_DEFAULT_BATCH_SIZE;
      for (auto & network_params : entry.second.network_params_by_name) {
        network_params.second.batch_size = 1;
      }
    }
    auto groups = take(device->configure(*hef, config), "configure network group");
    if (groups.size() != 1) throw std::runtime_error("Unexpected configured network group count");
    network = std::move(groups.front());

    constexpr std::uint32_t queue_size = 2;
    auto input_params = take(network->make_input_vstream_params(
      false, HAILO_FORMAT_TYPE_UINT8, options.timeout_ms, queue_size), "make input parameters");
    auto output_params = take(network->make_output_vstream_params(
      false, options.float32_output ? HAILO_FORMAT_TYPE_FLOAT32 : HAILO_FORMAT_TYPE_AUTO,
      options.timeout_ms, queue_size), "make output parameters");
    // [9.9-2][Pi-4.24] 4.24 同样弃用 quantized 布尔参数/标志；转换由数据类型决定。
    // 固定主机布局为 NHWC，原生整数由本工程反量化，FLOAT32 由 SDK 反量化。
    for (auto & entry : input_params) {
      entry.second.user_buffer_format.order = HAILO_FORMAT_ORDER_NHWC;
      entry.second.user_buffer_format.flags = HAILO_FORMAT_FLAGS_NONE;
    }
    for (auto & entry : output_params) {
      entry.second.user_buffer_format.order = HAILO_FORMAT_ORDER_NHWC;
      entry.second.user_buffer_format.flags = HAILO_FORMAT_FLAGS_NONE;
    }
    pipeline = std::make_unique<hailort::InferVStreams>(take(
      hailort::InferVStreams::create(*network, input_params, output_params), "create InferVStreams"));

    auto inputs = pipeline->get_input_vstreams();
    auto streams = pipeline->get_output_vstreams();
    if (inputs.size() != 1 || streams.size() != 3) {
      throw std::runtime_error("Unexpected runtime vstream count");
    }
    input_stream = &inputs.front().get();
    check_nhwc(input_stream->get_user_buffer_format());
    input_bytes = static_cast<std::size_t>(size.width) * size.height * 3;
    if (input_stream->get_user_buffer_format().type != HAILO_FORMAT_TYPE_UINT8 ||
        image_size(input_stream->get_info()) != size ||
        input_stream->get_info().shape.features != 3 || input_stream->get_frame_size() != input_bytes) {
      throw std::runtime_error("Hailo input buffer does not match packed UINT8 RGB geometry");
    }
    input_stream_name = input_stream->name();
    input_views.emplace(input_stream_name, hailort::MemoryView{});

    std::vector<HailoHeadInfo> heads;
    heads.reserve(streams.size());
    for (auto & reference : streams) {
      auto & stream = reference.get();
      check_nhwc(stream.get_user_buffer_format());
      const auto & info = stream.get_info();
      if (info.shape.features != HailoOutput::features_per_head) {
        throw std::runtime_error("Expected 66 features in raw output " + stream.name() +
          ", found " + std::to_string(info.shape.features));
      }
      HailoHeadInfo head{
        stream.name(), image_size(info), static_cast<int>(info.shape.features),
        element_type(stream.get_user_buffer_format().type), {}};
      for (const auto & q : stream.get_quant_infos()) {
        head.quantization.push_back({q.qp_zp, q.qp_scale});
      }
      heads.push_back(std::move(head));
    }
    outputs = std::make_unique<HailoOutput>(size, std::move(heads));

    // [9.9-2] 以真实名称绑定预分配缓冲，校验 SDK 字节数；map 和 MemoryView 跨帧复用。
    for (std::size_t i = 0; i < outputs->heads().size(); ++i) {
      const auto & name = outputs->heads()[i].name;
      auto & stream = take(pipeline->get_output_by_name(name), "get output vstream").get();
      if (stream.get_frame_size() != outputs->frame_bytes(i)) {
        throw std::runtime_error("Hailo output byte size mismatch: " + name);
      }
      output_streams[i] = &stream;
      output_views.emplace(name, hailort::MemoryView(outputs->data(i), outputs->frame_bytes(i)));
    }
    activation = take(network->activate(), "activate network group");
  }
};

HailoRuntime::HailoRuntime(const std::string & hef_path, const HailoRuntimeOptions & options)
: impl_(std::make_unique<Impl>(hef_path, options))
{
}

HailoRuntime::~HailoRuntime() = default;
cv::Size HailoRuntime::input_size() const { return impl_->size; }
const std::string & HailoRuntime::input_name() const { return impl_->input_stream_name; }
const std::vector<HailoHeadInfo> & HailoRuntime::output_info() const { return impl_->outputs->heads(); }

void HailoRuntime::infer(const cv::Mat & rgb, cv::Mat & output)
{
  // [9.9-2] 输入检查发生在 SDK 调用前；失败时不把之前的检测数据作为本帧交给上层。
  if (&rgb == &output) {
    output.release();
    throw std::invalid_argument("Hailo input and output must be different Mat objects");
  }
  output.release();
  if (impl_->failed) throw std::runtime_error("Hailo runtime has failed; recreate it before inference");
  if (rgb.empty() || rgb.dims != 2 || rgb.type() != CV_8UC3 ||
      rgb.size() != impl_->size || !rgb.isContinuous() ||
      rgb.total() * rgb.elemSize() != impl_->input_bytes) {
    throw std::invalid_argument("Hailo input must be contiguous CV_8UC3 RGB with the configured size");
  }
  impl_->input_views.begin()->second = hailort::MemoryView(rgb.data, impl_->input_bytes);
  try {
    check_status(impl_->pipeline->infer(impl_->input_views, impl_->output_views, 1), "infer");
    // [9.9-2][Pi-4.24] 继续额外检查各流是否中止，避免将不完整输出交给解码器。
    if (impl_->input_stream->is_aborted()) throw std::runtime_error("Hailo input stream was aborted");
    for (auto * stream : impl_->output_streams) {
      if (stream->is_aborted()) throw std::runtime_error("Hailo output stream was aborted");
    }
    output = impl_->outputs->fuse();
  } catch (...) {
    // 超时/中止后不同输出可能失去帧对齐，交给调用方重建，禁止原实例盲目重试。
    impl_->failed = true;
    throw;
  }
}
}  // namespace auto_aim
