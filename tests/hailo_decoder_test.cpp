// [9.9-3] 使用人工可核算张量检查实际 Decoder/Armor；不把合成数据当作 HEF 兼容证明。
#include "tasks/auto_aim/hailo/hailo_decoder.hpp"
#include "tasks/auto_aim/hailo/hailo_output.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace
{
using namespace auto_aim;
using Quad = std::array<cv::Point2f, 4>;

void require(bool condition, const std::string & message)
{
  if (!condition) throw std::runtime_error(message);
}

void near(double actual, double expected, const std::string & label)
{
  require(std::abs(actual - expected) < 1e-4, label);
}

void reject(const std::function<void()> & operation, const std::string & label)
{
  try {
    operation();
  } catch (const std::invalid_argument &) {
    return;
  }
  throw std::runtime_error("Expected invalid_argument: " + label);
}

HailoDecoderContract model_contract()
{
  // [9.9-3] 仅为测试夹具定义语义，绝不是 szu_int16_head_l.hef 的已验证配置。
  HailoDecoderContract model;
  model.corners_are_input_pixels = true;
  model.colors = {blue, red, extinguish, purple};
  model.names = {sentry, one, two, three, four, five, outpost, base, not_armor};
  model.corner_indices = {0, 3, 2, 1};  // 原始 LT/LB/RB/RT → SP25 LT/RT/RB/LB。
  return model;
}

cv::Mat empty_frame()
{
  return cv::Mat(20160, 22, CV_32FC1, cv::Scalar(-100));
}

Quad rectangle(float x, float y, float width, float height)
{
  return {{{x, y}, {x + width, y}, {x + width, y + height}, {x, y + height}}};
}

void write_candidate(float * row, const Quad & points, float score = 4,
  int color_id = 0, int name_id = 3, bool probabilities = false)
{
  std::fill(row, row + 22, probabilities ? 0.0F : -10.0F);
  // 手写模型顺序，避免测试调用生产重排算法而掩盖同样的顺序错误。
  row[0] = points[0].x; row[1] = points[0].y;
  row[2] = points[3].x; row[3] = points[3].y;
  row[4] = points[2].x; row[5] = points[2].y;
  row[6] = points[1].x; row[7] = points[1].y;
  row[8] = score;
  row[9 + color_id] = probabilities ? 0.6F : 2.0F;
  row[13 + name_id] = probabilities ? 0.6F : 2.0F;
}

void test_geometry_and_armor()
{
  HailoDecoder decoder(model_contract());
  auto frame = empty_frame();
  const Quad points{{{80, 100}, {180, 120}, {170, 160}, {70, 140}}};
  write_candidate(frame.ptr<float>(0), points, 2);
  const auto armors = decoder.decode(frame, {720, 540});
  require(armors.size() == 1, "Tilted valid quad was lost");
  const auto & armor = armors.front();
  const Quad expected{{{90, 105.46875F}, {202.5F, 126.5625F},
    {191.25F, 168.75F}, {78.75F, 147.65625F}}};
  for (int i = 0; i < 4; ++i) {
    near(armor.points[i].x, expected[i].x, "PnP corner order / x resize mapping");
    near(armor.points[i].y, expected[i].y, "PnP corner order / y resize mapping");
  }
  near(armor.confidence, 0.880797078, "Objectness confidence should not multiply class scores");
  near(armor.center.x, 140.625, "Armor center x");
  near(armor.center.y, 137.109375, "Armor center y");
  near(armor.center_norm.x, 0.1953125, "Normalized center x");
  near(armor.center_norm.y, 0.25390625, "Normalized center y");
  require(armor.box == cv::Rect(78, 105, 125, 64), "Outward-rounded original image bbox");
  require(armor.color == blue && armor.name == three && armor.type == small,
    "Semantic Armor constructor changed model mapping");
  require(armor.priority == fifth && !armor.duplicated && armor.class_id == -1,
    "Indeterminate priority/duplicated/class_id");
  require(armor.xyz_in_gimbal.isZero() && armor.xyz_in_world.isZero() &&
    armor.ypr_in_gimbal.isZero() && armor.ypr_in_world.isZero() && armor.ypd_in_world.isZero() &&
    armor.yaw_raw == 0 && armor.left.length == 0 && armor.right.id == 0,
    "Indeterminate pose or default lightbar");
  near(armor.side_ratio, 1, "Lightbar side ratio");

  // [9.9-4] 使用正确 bbox 单独检查入口错误；新构造不再委托旧网络构造。
  reject([&] { Armor a(blue, three, small, 0.9, {70, 100, 110, 60}, {}, {640, 512}); },
    "Semantic constructor must validate point count before indexing");
  auto p = std::vector<cv::Point2f>(points.begin(), points.end());
  reject([&] { Armor a(blue, not_armor, small, 0.9, {70, 100, 110, 60}, p, {640, 512}); },
    "Invalid Armor semantic name");
  reject([&] { Armor a(blue, three, small, -1, {70, 100, 110, 60}, p, {640, 512}); },
    "Invalid Armor confidence");
}

void test_semantic_maps_and_types()
{
  auto model = model_contract();
  HailoDecoder decoder(model);
  auto frame = empty_frame();
  for (int color = 0; color < 4; ++color) {
    for (int name = 0; name < 9; ++name) {
      write_candidate(frame.ptr<float>(0), rectangle(100, 100, 60, 30), 4, color, name);
      const auto result = decoder.decode(frame, {640, 512});
      if (name == 8) {
        require(result.empty(), "not_armor must be rejected");
        continue;
      }
      require(result.size() == 1 && result.front().color == model.colors[color] &&
        result.front().name == model.names[name], "4 x 9 semantic mapping");
      require(result.front().type == (name == 1 ? big : small), "Narrow armor type");
      write_candidate(frame.ptr<float>(0), rectangle(100, 100, 120, 30), 4, color, name);
      const auto wide = decoder.decode(frame, {640, 512});
      const bool expected_big = name == 1 || (name >= 3 && name <= 5) || name == 7;
      require(wide.front().type == (expected_big ? big : small), "Wide armor type");
    }
  }
  // 映射可以更换，不能偷偷直接 cast 网络编号为 SP25 枚举。
  std::swap(model.colors[0], model.colors[1]);
  std::swap(model.names[0], model.names[3]);
  HailoDecoder remapped(model);
  write_candidate(frame.ptr<float>(0), rectangle(100, 100, 60, 30), 4, 0, 0);
  const auto changed = remapped.decode(frame, {640, 512});
  require(changed.front().color == red && changed.front().name == three, "Explicit remapping ignored");

  // 高度非等比 resize 后，大小判断必须使用还原后的原图几何。
  write_candidate(frame.ptr<float>(0), rectangle(100, 100, 100, 30));
  require(decoder.decode(frame, {320, 512}).front().type == small, "Type used network-space ratio");
}

void test_scores_and_invalid_rows()
{
  HailoDecoder decoder(model_contract());
  auto frame = empty_frame();
  const auto quad = rectangle(100, 100, 60, 30);
  for (const float logit : {-10000.0F, 0.0F, 1.0F, 10000.0F}) {
    write_candidate(frame.ptr<float>(0), quad, logit);
    const auto result = decoder.decode(frame, {640, 512});
    require(result.empty() == (logit < 2), "Stable sigmoid / confidence threshold");
  }
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  for (const int field : {0, 7, 8, 9, 12, 13, 21}) {
    for (const float value : {nan, inf, -inf}) {
      write_candidate(frame.ptr<float>(0), quad);
      frame.at<float>(0, field) = value;
      require(decoder.decode(frame, {640, 512}).empty(), "Non-finite candidate was accepted");
    }
  }
  // 无效候选不能影响另一条有效观测。
  write_candidate(frame.ptr<float>(20159), rectangle(300, 100, 60, 30));
  require(decoder.decode(frame, {640, 512}).size() == 1, "Bad row discarded whole frame");
  frame = empty_frame();
  auto model = model_contract();
  model.scores = HailoScoreEncoding::probabilities;
  HailoDecoder probabilities(model);
  write_candidate(frame.ptr<float>(0), quad, 0.85F, 0, 3, true);
  near(probabilities.decode(frame, {640, 512}).front().confidence, 0.85, "Double sigmoid");
  frame.at<float>(0, 8) = 0.8F;
  require(probabilities.decode(frame, {640, 512}).size() == 1, "Threshold equality");
  for (const int field : {8, 9, 21}) {
    write_candidate(frame.ptr<float>(0), quad, 0.85F, 0, 3, true);
    frame.at<float>(0, field) = 1.1F;
    require(probabilities.decode(frame, {640, 512}).empty(), "Out-of-range probability");
  }
}

void test_quad_rejection()
{
  HailoDecoder decoder(model_contract());
  auto frame = empty_frame();
  const auto normal = rectangle(100, 100, 60, 30);
  const auto rejected = [&](const Quad & points, const std::string & label) {
    write_candidate(frame.ptr<float>(0), points);
    require(decoder.decode(frame, {640, 512}).empty(), label);
  };
  auto points = normal;
  std::swap(points[1], points[3]);
  rejected(points, "Reversed winding");
  points = normal;
  std::swap(points[1], points[2]);
  rejected(points, "Self-intersection");
  points = normal;
  points[2] = {110, 110};
  rejected(points, "Concave quad");
  points = normal;
  points[1] = points[0];
  rejected(points, "Repeated point");
  rejected(rectangle(100, 100, 1, 1), "Too small quad");
  rejected(rectangle(-1, 100, 60, 30), "Outside image left");
  rejected(rectangle(600, 100, 60, 30), "Outside image right");
  rejected(rectangle(100, 500, 60, 30), "Outside image bottom");
  points = {{{100, 100}, {120, 100}, {140, 100}, {160, 100}}};
  rejected(points, "Collinear quad");
  // 保留已知语义点在大角度旋转后的对应关系，不按 x/y 重新排序。
  const float angle = 1.2F;
  points = rectangle(-30, -15, 60, 30);
  for (auto & p : points) {
    p = {200 + p.x * std::cos(angle) - p.y * std::sin(angle),
      200 + p.x * std::sin(angle) + p.y * std::cos(angle)};
  }
  write_candidate(frame.ptr<float>(0), points);
  const auto result = decoder.decode(frame, {640, 512});
  require(result.size() == 1, "Strongly tilted valid quad was rejected");
  near(cv::norm(result.front().points[0] - points[0]), 0, "Semantic corners were geometrically sorted");
}

void test_suppression_and_limits()
{
  auto model = model_contract();
  HailoDecoder decoder(model);
  auto frame = empty_frame();
  write_candidate(frame.ptr<float>(0), rectangle(100, 100, 60, 30), 2);
  write_candidate(frame.ptr<float>(15360), rectangle(101, 101, 60, 30), 4, 1, 4);
  write_candidate(frame.ptr<float>(20159), rectangle(300, 100, 60, 30), 3);
  auto result = decoder.decode(frame, {640, 512});
  require(result.size() == 2 && result.front().color == red && result.front().name == four,
    "Cross-class NMS must keep highest score");

  frame = empty_frame();
  write_candidate(frame.ptr<float>(0), rectangle(100, 140, 100, 20), 4);
  write_candidate(frame.ptr<float>(1), rectangle(140, 100, 20, 100), 3);
  require(decoder.decode(frame, {640, 512}).size() == 1, "Near-center duplicate below NMS IoU");
  HailoDecoderOptions options;
  options.duplicate_center_ratio = 0;
  HailoDecoder no_duplicate(model, options);
  require(no_duplicate.decode(frame, {640, 512}).size() == 2, "Disabled duplicate suppression");
  write_candidate(frame.ptr<float>(1), rectangle(140, 100, 20, 100), 3, 1);
  require(decoder.decode(frame, {640, 512}).size() == 2, "Different-color near centers merged");
  write_candidate(frame.ptr<float>(1), rectangle(140, 100, 20, 100), 3, 0, 4);
  require(decoder.decode(frame, {640, 512}).size() == 2, "Different-name near centers merged");

  options.max_candidates = 2;
  options.max_detections = 2;
  HailoDecoder capped(model, options);
  frame = empty_frame();
  write_candidate(frame.ptr<float>(0), rectangle(20, 100, 60, 30), 2);
  write_candidate(frame.ptr<float>(15360), rectangle(200, 100, 60, 30), 4);
  write_candidate(frame.ptr<float>(20159), rectangle(400, 100, 60, 30), 3);
  result = capped.decode(frame, {640, 512});
  require(result.size() == 2 && result.front().box.x == 200 && result.back().box.x == 400,
    "Top-K must include better candidates from later scales");
  frame.at<float>(0, 8) = 4;
  frame.at<float>(20159, 8) = 4;
  result = capped.decode(frame, {640, 512});
  require(result.front().box.x == 20 && result.back().box.x == 200, "Deterministic equal-score top-K");
  options.max_detections = 1;
  HailoDecoder single(model, options);
  require(single.decode(frame, {640, 512}).size() == 1, "Output cap");

  // [9.9-3] 稠密输入只留下 K 个最高分，跨 20160 行访问不越界。
  for (int r = 0; r < frame.rows; ++r) {
    write_candidate(frame.ptr<float>(r), rectangle(100, 100, 60, 30), 2);
  }
  write_candidate(frame.ptr<float>(20159), rectangle(300, 100, 60, 30), 4);
  result = capped.decode(frame, {640, 512});
  require(result.size() == 2 && result.front().box.x == 300, "Dense candidate top-K");
}

void test_fusion_lifetime_and_strided_input()
{
  // 接通第二部分真实融合实现与第三部分，所有检测头均参与，测试不链接 Hailo SDK。
  HailoOutput output({640, 512}, {
    {"coarse", {20, 16}, 66, HailoElementType::float32, {}},
    {"fine", {80, 64}, 66, HailoElementType::float32, {}},
    {"middle", {40, 32}, 66, HailoElementType::float32, {}}});
  for (std::size_t h = 0; h < 3; ++h) {
    auto * values = static_cast<float *>(output.data(h));
    std::fill(values, values + output.frame_bytes(h) / sizeof(float), -100.0F);
    const int last = output.heads()[h].grid_size.area() * 66 - 22;
    write_candidate(values + last, rectangle(20.0F + h * 180, 100, 60, 30), 4);
  }
  HailoDecoder decoder(model_contract());
  const auto held = decoder.decode(output.fuse(), {640, 512});
  require(held.size() == 3, "Fusion -> Decoder lost a scale or the final anchor");
  near(held.front().center.x, 50, "Sorted scale order");
  auto frame = empty_frame();
  cv::Mat padded(frame.rows, 25, CV_32F);
  cv::Mat strided = padded.colRange(1, 23);
  frame.copyTo(strided);
  require(!strided.isContinuous(), "Test requires non-contiguous input");
  for (int i = 0; i < 100; ++i) {
    write_candidate(strided.ptr<float>(20159), rectangle(100.0F + i, 100, 60, 30));
    const auto result = decoder.decode(strided, {640, 512});
    require(result.size() == 1 && result.front().box.x == 100 + i, "Stale results / row stride");
  }
  near(held.front().center.x, 50, "Earlier Armor shared reused candidate storage");
  require(decoder.decode(empty_frame(), {640, 512}).empty(), "Empty scene retained previous Armor");
  reject([&] { decoder.decode(cv::Mat(), {640, 512}); }, "Empty tensor is not a valid empty scene");
  reject([&] { decoder.decode(frame.colRange(0, 21), {640, 512}); }, "Wrong columns");
  reject([&] { decoder.decode(frame.rowRange(0, 20159), {640, 512}); }, "Wrong row count");
  reject([&] { decoder.decode(cv::Mat(20160, 22, CV_16U), {640, 512}); }, "Not dequantized");
  reject([&] { decoder.decode(frame, {0, 512}); }, "Invalid image size");
  require(decoder.decode(frame, {640, 512}).empty(), "State after rejected input");
}

void test_rejected_contracts()
{
  reject([] { HailoDecoder decoder(HailoDecoderContract{}); }, "Unconfirmed model");
  const auto bad_model = [](const std::function<void(HailoDecoderContract &)> & change) {
    auto model = model_contract();
    change(model);
    reject([&] { HailoDecoder decoder(model); }, "Invalid model contract");
  };
  bad_model([](auto & m) { m.corners_are_input_pixels = false; });
  bad_model([](auto & m) { m.input_size = {641, 512}; });
  bad_model([](auto & m) { m.colors[0] = m.colors[1]; });
  // [9.9-3][Pi接入] 重复语义合法（基地别名）；仍拒绝非法枚举。
  bad_model([](auto & m) { m.names[0] = static_cast<ArmorName>(9); }); // 可表示，但不是合法编号。
  bad_model([](auto & m) { m.corner_indices = {0, 3, 2, 4}; });
  bad_model([](auto & m) { m.scores = static_cast<HailoScoreEncoding>(99); });
  const auto bad_options = [](const std::function<void(HailoDecoderOptions &)> & change) {
    HailoDecoderOptions options;
    change(options);
    reject([&] { HailoDecoder decoder(model_contract(), options); }, "Invalid decoder options");
  };
  bad_options([](auto & o) { o.min_confidence = -1; });
  bad_options([](auto & o) { o.nms_iou = 1.1F; });
  bad_options([](auto & o) { o.min_quad_area = 0; });
  bad_options([](auto & o) { o.big_armor_ratio = std::numeric_limits<float>::quiet_NaN(); });
  bad_options([](auto & o) { o.duplicate_center_ratio = -1; });
  bad_options([](auto & o) { o.max_candidates = 0; });
  bad_options([](auto & o) { o.max_detections = o.max_candidates + 1; });
}

// [9.9-4] 复查第二、三部分的实际衔接：混合整数/浮点输出，反量化后直接构造 Armor。
void test_mixed_precision_to_armor()
{
  HailoOutput output({640, 512}, {
    {"coarse", {20, 16}, 66, HailoElementType::float32, {}},
    {"fine", {80, 64}, 66, HailoElementType::uint8, {{128, 2}}},
    {"middle", {40, 32}, 66, HailoElementType::uint16, {{40000, 0.5F}}}});
  for (std::size_t h = 0; h < 3; ++h) {
    const auto & head = output.heads()[h];
    const int count = head.grid_size.area() * 66;
    float row[22];
    write_candidate(row, rectangle(20.0F + h * 80, 100, 40, 20), 4,
      h == 2 ? 3 : static_cast<int>(h), h == 0 ? 3 : h == 1 ? 1 : 7);
    for (int i = 0; i < count; ++i) {
      const float value = i >= count - 22 ? row[i - (count - 22)] : -100.0F;
      if (h == 0) {
        static_cast<std::uint8_t *>(output.data(h))[i] = static_cast<std::uint8_t>(value / 2 + 128);
      } else if (h == 1) {
        static_cast<std::uint16_t *>(output.data(h))[i] = static_cast<std::uint16_t>(value * 2 + 40000);
      } else {
        static_cast<float *>(output.data(h))[i] = value;
      }
    }
  }
  HailoDecoder decoder(model_contract());
  const auto armors = decoder.decode(output.fuse(), {640, 512});
  require(armors.size() == 3, "Mixed precision fusion lost an Armor");
  auto it = armors.begin();
  require(it->color == blue && it->name == three && it->box.x == 20 && it->type == small,
    "UINT8 fusion -> semantic Armor");
  ++it;
  require(it->color == red && it->name == one && it->box.x == 100 && it->type == big,
    "UINT16 values above 32767 -> semantic Armor");
  ++it;
  require(it->color == purple && it->name == base && it->box.x == 180 && it->type == small,
    "FLOAT32 fusion -> semantic Armor");
  for (const auto & armor : armors) {
    near(armor.confidence, 0.98201379, "Mixed precision objectness");
    require(armor.points.size() == 4 && armor.priority == fifth && !armor.duplicated,
      "Mixed precision Armor defaults");
  }
}

// [9.9-4] min+(max-min) 的 FLOAT32 舍入曾把 3.000000238 还原成 3，导致 bbox 漏包角点。
void test_fractional_bbox_enclosure()
{
  HailoDecoder decoder(model_contract());
  auto frame = empty_frame();
  const float minimum = 0.3075F;
  const float maximum = std::nextafter(3.0F, 4.0F);
  const Quad p{{{minimum, minimum}, {maximum, minimum},
    {maximum, maximum}, {minimum, maximum}}};
  write_candidate(frame.ptr<float>(0), p);
  const auto armors = decoder.decode(frame, {640, 512});
  require(armors.size() == 1 && armors.front().box == cv::Rect(0, 0, 4, 4),
    "Outward bbox rounding lost a subpixel corner");
  require(armors.front().points[2] == p[2], "BBox correction must not clamp PnP corners");
}
}  // namespace

int main()
{
  try {
    test_geometry_and_armor();
    test_semantic_maps_and_types();
    test_scores_and_invalid_rows();
    test_quad_rejection();
    test_suppression_and_limits();
    test_fusion_lifetime_and_strided_input();
    test_rejected_contracts();
    test_mixed_precision_to_armor();  // [9.9-4] 第二至四部分的无硬件回归。
    test_fractional_bbox_enclosure();
    std::cout << "PASS: mappings, sigmoid, PnP corner order, resize, Armor initialization, "
      "geometry, NMS, duplicate suppression, bounded top-K, fusion and 100-frame lifetime\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}
