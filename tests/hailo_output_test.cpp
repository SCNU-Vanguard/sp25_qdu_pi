// [9.9-2] 无硬件合同测试：检查真实尺寸下的布局、量化、边界和缓冲复用，不模拟 NPU 推理。
#include "tasks/auto_aim/hailo/hailo_output.hpp"

#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
using auto_aim::HailoElementType;
using auto_aim::HailoHeadInfo;
using auto_aim::HailoOutput;

void require(bool condition, const std::string & message)
{
  if (!condition) throw std::runtime_error(message);
}

void expect_failure(const std::function<void()> & operation, const std::string & label)
{
  try {
    operation();
  } catch (const std::exception &) {
    return;
  }
  throw std::runtime_error("Expected rejection: " + label);
}

std::vector<HailoHeadInfo> make_heads()
{
  HailoHeadInfo fine{"z_fine", {80, 64}, 66, HailoElementType::uint8, {{120.5F, 0.25F}}};
  HailoHeadInfo middle{"a_middle", {40, 32}, 66, HailoElementType::uint16, {}};
  for (int c = 0; c < 66; ++c) {
    middle.quantization.push_back({33000.0F + c, 0.01F * (c + 1)});
  }
  // FLOAT32 的量化参数故意无效，验证不会发生二次反量化。
  HailoHeadInfo coarse{"m_coarse", {20, 16}, 66, HailoElementType::float32,
    {{std::numeric_limits<float>::quiet_NaN(), 0.0F}}};
  return {coarse, fine, middle};  // 故意打乱输出顺序和名称排序。
}

float source_value(HailoElementType type, int pixel, int channel)
{
  if (type == HailoElementType::uint8) return static_cast<float>((pixel + channel) % 256);
  if (type == HailoElementType::uint16) return static_cast<float>((pixel * 71 + channel * 257) % 65536);
  return (pixel * 66 + channel - 10000) * 0.125F;
}

void fill(HailoOutput & output)
{
  for (std::size_t h = 0; h < output.heads().size(); ++h) {
    const auto & head = output.heads()[h];
    for (int p = 0; p < head.grid_size.area(); ++p) {
      for (int c = 0; c < head.features; ++c) {
        const int index = p * head.features + c;
        const float value = source_value(head.type, p, c);
        switch (head.type) {
          case HailoElementType::uint8:
            static_cast<std::uint8_t *>(output.data(h))[index] = static_cast<std::uint8_t>(value);
            break;
          case HailoElementType::uint16:
            static_cast<std::uint16_t *>(output.data(h))[index] = static_cast<std::uint16_t>(value);
            break;
          case HailoElementType::float32:
            static_cast<float *>(output.data(h))[index] = value;
            break;
        }
      }
    }
  }
}

void check_values(const HailoOutput & output, const cv::Mat & fused)
{
  require(fused.rows == 20160 && fused.cols == 22 && fused.type() == CV_32FC1,
    "Expected 20160 x 22 FLOAT32 matrix");
  require(output.heads()[0].first_row == 0 && output.heads()[1].first_row == 15360 &&
    output.heads()[2].first_row == 19200, "Wrong scale row offsets");
  for (const auto & head : output.heads()) {
    for (int y = 0; y < head.grid_size.height; ++y) {
      for (int x = 0; x < head.grid_size.width; ++x) {
        const int pixel = y * head.grid_size.width + x;
        for (int anchor = 0; anchor < 3; ++anchor) {
          for (int field = 0; field < 22; ++field) {
            const int channel = anchor * 22 + field;
            float expected = source_value(head.type, pixel, channel);
            if (head.type != HailoElementType::float32) {
              const auto q = head.quantization.at(head.quantization.size() == 1 ? 0 : channel);
              expected = (expected - q.zero_point) * q.scale;
            }
            const int row = head.first_row + pixel * 3 + anchor;
            const float actual = fused.at<float>(row, field);
            require(std::abs(actual - expected) <= 0.00001F * (1.0F + std::abs(expected)),
              "Wrong scale/grid/anchor/channel mapping or quantization");
          }
        }
      }
    }
  }
}

void test_fusion()
{
  HailoOutput output({640, 512}, make_heads());
  require(output.heads()[0].name == "z_fine" && output.heads()[1].name == "a_middle" &&
    output.heads()[2].name == "m_coarse", "Outputs were ordered by name or SDK position");
  require(output.frame_bytes(0) == 337920 && output.frame_bytes(1) == 168960 &&
    output.frame_bytes(2) == 84480, "Incorrect native buffer byte counts");
  fill(output);
  const auto & fused = output.fuse();
  check_values(output, fused);

  const auto * fused_data = fused.data;
  const std::vector<void *> original_buffers{output.data(0), output.data(1), output.data(2)};
  for (int frame = 0; frame < 100; ++frame) {
    require(output.fuse().data == fused_data, "Fused output was reallocated");
    for (std::size_t h = 0; h < original_buffers.size(); ++h) {
      require(output.data(h) == original_buffers[h], "Native output was reallocated");
    }
  }
  static_cast<std::uint8_t *>(output.data(0))[0] = 255;
  require(output.fuse().at<float>(0, 0) == (255.0F - 120.5F) * 0.25F,
    "New frame data was not used");
  auto * floats = static_cast<float *>(output.data(2));
  for (const float invalid : {std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity()}) {
    floats[0] = invalid;
    expect_failure([&] { output.fuse(); }, "non-finite FLOAT32 output");
  }
}

void test_rejected_contracts()
{
  expect_failure([] { HailoOutput output({0, 512}, make_heads()); }, "empty input");
  expect_failure([] { HailoOutput output({641, 512}, make_heads()); }, "non-strided input");
  const auto reject = [](const std::function<void(std::vector<HailoHeadInfo> &)> & mutate) {
    auto heads = make_heads();
    mutate(heads);
    expect_failure([&] { HailoOutput output({640, 512}, heads); }, "invalid output metadata");
  };
  reject([](auto & h) { h.pop_back(); });
  reject([](auto & h) { h[0].features = 22; });
  reject([](auto & h) { h[0].name.clear(); });
  reject([](auto & h) { h[0].name = h[1].name; });
  reject([](auto & h) { h[0].grid_size = {0, 16}; });
  reject([](auto & h) { h[0].grid_size = {20, -16}; });
  reject([](auto & h) { h[0].grid_size = {21, 16}; });
  reject([](auto & h) { h[0].grid_size = {20, 32}; });
  reject([](auto & h) { h[0].grid_size = h[1].grid_size; });
  reject([](auto & h) { h[0].grid_size = {10, 8}; });
  reject([](auto & h) { h[1].quantization.clear(); });
  reject([](auto & h) { h[1].quantization.resize(2); });
  reject([](auto & h) { h[1].quantization[0].scale = 0; });
  reject([](auto & h) { h[1].quantization[0].scale = -1; });
  reject([](auto & h) { h[1].quantization[0].scale = std::numeric_limits<float>::infinity(); });
  reject([](auto & h) { h[1].quantization[0].zero_point = std::numeric_limits<float>::quiet_NaN(); });
  reject([](auto & h) { h[1].quantization[0].scale = std::numeric_limits<float>::max(); });
  reject([](auto & h) { h[0].type = static_cast<HailoElementType>(99); });
}
}  // namespace

int main()
{
  try {
    test_fusion();
    test_rejected_contracts();
    std::cout << "PASS: three-scale mapping, UINT8/UINT16/FLOAT32, channel quantization, "
                 "100-frame buffer reuse, non-finite values and invalid metadata\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}
