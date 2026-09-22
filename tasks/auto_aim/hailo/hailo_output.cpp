#include "hailo_output.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <utility>

namespace auto_aim
{
namespace
{
// [9.9-2] 按真实数据类型访问，避免把 UINT16 缓冲区误当字节或 FLOAT32 读取。
int cv_type(HailoElementType type)
{
  switch (type) {
    case HailoElementType::uint8: return CV_8UC1;
    case HailoElementType::uint16: return CV_16UC1;
    case HailoElementType::float32: return CV_32FC1;
  }
  throw std::invalid_argument("Unsupported Hailo output element type");
}

template <typename T>
void dequantize(const cv::Mat & buffer, const HailoHeadInfo & head, float * destination)
{
  const auto * source = buffer.ptr<T>();
  const std::size_t count = buffer.total();
  if (head.quantization.size() == 1) {
    const auto q = head.quantization.front();
    for (std::size_t i = 0; i < count; ++i) {
      destination[i] = (static_cast<float>(source[i]) - q.zero_point) * q.scale;
    }
  } else {
    // [9.9-2] 支持 SDK 返回的逐特征通道量化参数，NHWC 的末维才是通道。
    for (std::size_t pixel = 0; pixel < count; pixel += head.features) {
      for (int channel = 0; channel < head.features; ++channel) {
        const auto q = head.quantization[channel];
        destination[pixel + channel] =
          (static_cast<float>(source[pixel + channel]) - q.zero_point) * q.scale;
      }
    }
  }
}
}  // namespace

HailoOutput::HailoOutput(cv::Size input_size, std::vector<HailoHeadInfo> heads)
: heads_(std::move(heads))
{
  // [9.9-2] 启动时校验全部布局后再申请内存；不依赖 HEF 输出名称或返回顺序。
  constexpr std::array<int, 3> strides{8, 16, 32};
  if (input_size.width <= 0 || input_size.height <= 0 ||
      input_size.width % strides.back() != 0 || input_size.height % strides.back() != 0) {
    throw std::invalid_argument("Hailo input width and height must be positive multiples of 32");
  }
  if (heads_.size() != strides.size()) {
    throw std::invalid_argument("Expected exactly three raw Hailo detection heads");
  }
  for (auto & head : heads_) {
    if (head.name.empty() || head.grid_size.width <= 0 || head.grid_size.height <= 0 ||
        head.features != features_per_head ||
        input_size.width % head.grid_size.width != 0 ||
        input_size.height % head.grid_size.height != 0) {
      throw std::invalid_argument("Invalid Hailo head " + head.name + ": expected a grid with 66 features");
    }
    head.stride = input_size.width / head.grid_size.width;
    if (head.stride != input_size.height / head.grid_size.height) {
      throw std::invalid_argument("Inconsistent horizontal/vertical stride: " + head.name);
    }
    cv_type(head.type);
    if (head.type != HailoElementType::float32) {
      if (head.quantization.size() != 1 &&
          head.quantization.size() != static_cast<std::size_t>(head.features)) {
        throw std::invalid_argument("Expected one or 66 quantization entries: " + head.name);
      }
      const float maximum = head.type == HailoElementType::uint8 ? 255.0F : 65535.0F;
      for (const auto q : head.quantization) {
        if (!std::isfinite(q.zero_point) || !std::isfinite(q.scale) || q.scale <= 0 ||
            !std::isfinite((0.0F - q.zero_point) * q.scale) ||
            !std::isfinite((maximum - q.zero_point) * q.scale)) {
          throw std::invalid_argument("Invalid quantization parameters: " + head.name);
        }
      }
    }
  }
  std::sort(heads_.begin(), heads_.end(), [](const auto & a, const auto & b) {
    return a.stride < b.stride;
  });
  std::int64_t total_rows = 0;
  for (std::size_t i = 0; i < heads_.size(); ++i) {
    auto & head = heads_[i];
    if (head.stride != strides[i]) {
      throw std::invalid_argument("Hailo heads must cover strides 8, 16 and 32 exactly once");
    }
    for (std::size_t j = 0; j < i; ++j) {
      if (head.name == heads_[j].name) {
        throw std::invalid_argument("Duplicate Hailo output name: " + head.name);
      }
    }
    head.first_row = static_cast<int>(total_rows);
    total_rows += static_cast<std::int64_t>(head.grid_size.width) *
                  head.grid_size.height * anchors_per_cell;
    if (total_rows > std::numeric_limits<int>::max() ||
        head.grid_size.width > std::numeric_limits<int>::max() / head.features) {
      throw std::invalid_argument("Hailo output dimensions exceed OpenCV limits");
    }
  }

  // [9.9-2] 三个原始输出和融合矩阵一次性分配，后续帧直接复用。
  buffers_.reserve(heads_.size());
  for (const auto & head : heads_) {
    buffers_.emplace_back(
      head.grid_size.height, head.grid_size.width * head.features, cv_type(head.type));
  }
  fused_.create(static_cast<int>(total_rows), values_per_candidate, CV_32FC1);
}

std::size_t HailoOutput::frame_bytes(std::size_t head) const
{
  const auto & buffer = buffers_.at(head);
  return buffer.total() * buffer.elemSize();
}

const cv::Mat & HailoOutput::fuse()
{
  for (std::size_t i = 0; i < heads_.size(); ++i) {
    const auto & head = heads_[i];
    const auto & buffer = buffers_[i];
    auto * destination = fused_.ptr<float>(head.first_row);
    switch (head.type) {
      case HailoElementType::uint8:
        dequantize<std::uint8_t>(buffer, head, destination);
        break;
      case HailoElementType::uint16:
        dequantize<std::uint16_t>(buffer, head, destination);
        break;
      case HailoElementType::float32:
        // [9.9-2] FLOAT32 已由 HailoRT 转换，不再反量化；阻止 NaN/Inf 进入后续解码。
        if (!cv::checkRange(buffer, true)) {
          throw std::runtime_error("Non-finite FLOAT32 Hailo output: " + head.name);
        }
        std::memcpy(destination, buffer.data, frame_bytes(i));
        break;
    }
  }
  return fused_;
}
}  // namespace auto_aim
