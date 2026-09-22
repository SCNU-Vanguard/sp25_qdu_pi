// [9.9-3][Pi接入] 手算坐标与混合整数夹具验证模型 tail；不把合成张量视为真机识别证明。
#include "tasks/auto_aim/hailo/szu_int16_model.hpp"
#include "tasks/auto_aim/hailo/hailo_output.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace
{
using namespace auto_aim;
void require(bool ok, const char * message)
{
  if (!ok) throw std::runtime_error(message);
}
void near(float actual, float expected)
{
  require(std::abs(actual - expected) < 1e-4F, "Model tail coordinate/precision mismatch");
}

void test_all_anchors_and_order()
{
  SzuInt16Model model;
  cv::Mat padded(20160, 25, CV_32F, cv::Scalar(-100));
  auto raw = padded.colRange(1, 23);  // 同时检验有行步长的张量。
  // 每头选 x=2,y=3，原始四点 (0,0),(0,1),(1,1),(1,0)。期望值手算。
  const int cells[] = {242, 122, 62};
  const int starts[] = {0, 15360, 19200};
  const int sizes[] = {5120, 1280, 320};
  const float right_bottom[9][2] = {
    {26, 37}, {32, 54}, {49, 47},
    {62, 109}, {94, 93}, {91, 167},
    {180, 186}, {220, 294}, {437, 422}};
  const float origins[3][2] = {{16, 24}, {32, 48}, {64, 96}};
  for (int h = 0; h < 3; ++h) {
    for (int a = 0; a < 3; ++a) {
      auto * p = raw.ptr<float>(starts[h] + cells[h] * 3 + a);
      const float values[8] = {0, 0, 0, 1, 1, 1, 1, 0};
      std::copy(values, values + 8, p);
      p[8] = 2.0006F;  // ARM FP16 应舍入为 2.0。
      p[21] = static_cast<float>(h * 3 + a);
    }
  }
  const auto & decoded = model.decode_heads(raw);
  const auto * allocation = decoded.data;
  for (int h = 0; h < 3; ++h) {
    for (int a = 0; a < 3; ++a) {
      const auto * p = decoded.ptr<float>(starts[h] + a * sizes[h] + cells[h]);
      const auto & rb = right_bottom[h * 3 + a];
      near(p[0], origins[h][0]); near(p[1], origins[h][1]);
      near(p[2], origins[h][0]); near(p[3], rb[1]);
      near(p[4], rb[0]); near(p[5], rb[1]);
      near(p[6], rb[0]); near(p[7], origins[h][1]);
      near(p[8], 2); near(p[21], h * 3 + a);
    }
  }
  raw.at<float>(0, 0) = 1.0003F;
  near(model.decode_heads(raw).at<float>(0, 0), 10);
  require(allocation == model.decode_heads(raw).data, "Tail buffer reallocated");
  bool rejected = false;
  try { model.decode_heads(decoded); } catch (const std::invalid_argument &) { rejected = true; }
  require(rejected, "Tail must not decode its own pixel output twice");
  rejected = false;
  try { model.decode_heads(raw.rowRange(0, 20159)); }
  catch (const std::invalid_argument &) { rejected = true; }
  require(rejected, "Wrong raw shape accepted");
}

void test_integer_heads_to_armor()
{
  HailoOutput output({640, 512}, {
    {"conv60", {20, 16}, 66, HailoElementType::uint8, {{128, 0.125F}}},
    {"conv47", {80, 64}, 66, HailoElementType::uint16, {{40000, 0.125F}}},
    {"conv54", {40, 32}, 66, HailoElementType::uint8, {{128, 0.125F}}}});
  for (std::size_t i = 0; i < 3; ++i) {
    std::fill_n(static_cast<unsigned char *>(output.data(i)), output.frame_bytes(i), 0);
  }
  // fine head，x=10,y=10,anchor=1。模型点还原为 (80,80),(80,110),(112,110),(112,80)。
  auto * candidate = static_cast<std::uint16_t *>(output.data(0)) + 810 * 66 + 22;
  const int point_values[] = {0, 0, 0, 1, 2, 1, 2, 0};
  std::fill_n(candidate, 22, 39920);  // 其余 logits=-10。
  for (int i = 0; i < 8; ++i) candidate[i] = 40000 + point_values[i] * 8;
  candidate[8] = 40016;  // 2 -> sigmoid(2)，不可直接当概率或再次 sigmoid。
  SzuInt16Model model;
  const ArmorName expected[] = {sentry, one, two, three, four, five, outpost, base, base};
  for (int color = 0; color < 4; ++color) {
    for (int name = 0; name < 9; ++name) {
      std::fill(candidate + 9, candidate + 22, 39920);
      candidate[9 + color] = 40016;
      candidate[13 + name] = 40016;
      const auto armors = model.decode(output.fuse(), {720, 540});
      if (color >= 2) {
        require(armors.empty(), "Auxiliary color reached Armor");
        continue;
      }
      require(armors.size() == 1, "Quantized candidate lost");
      const auto & armor = armors.front();
      require(armor.name == expected[name] && armor.color == (color == 0 ? blue : red),
        "Real INT16 class/alias/color mapping mismatch");
      near(armor.confidence, 0.880797078F);
      const float points[4][2] = {{90, 84.375F}, {126, 84.375F},
        {126, 116.015625F}, {90, 116.015625F}};
      for (int i = 0; i < 4; ++i) {
        near(armor.points[i].x, points[i][0]); near(armor.points[i].y, points[i][1]);
      }
    }
  }
  auto raw = output.fuse().clone();
  const int row = 810 * 3 + 1;
  raw.at<float>(row, 11) = -10;
  raw.at<float>(row, 12) = -10;
  raw.at<float>(row, 9) = 2;
  auto held = model.decode(raw, {720, 540});
  require(held.size() == 1, "Valid frame missing before error test");
  for (float invalid : {std::numeric_limits<float>::quiet_NaN(),
                       std::numeric_limits<float>::infinity(), 70000.0F}) {
    raw.at<float>(row, 0) = invalid;
    require(model.decode(raw, {720, 540}).empty(), "Bad/FP16-overflow corner accepted");
  }
  require(held.front().name == base && held.front().points[0].x == 90,
    "Previous Armor changed when tail buffer reused");
  raw.setTo(-100);
  require(model.decode(raw, {720, 540}).empty(), "Empty frame retained old detection");
}
}  // namespace

int main()
{
  try {
    test_all_anchors_and_order();
    test_integer_heads_to_armor();
    std::cout << "PASS: nine anchors, grid/order, FP16, mixed integer heads, aliases, colors, corners\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}
