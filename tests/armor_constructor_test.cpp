// [9.9-4] 直接验证 Armor 网络入口，不依赖 Decoder 先行过滤，更不需要模型或设备。
#include "tasks/auto_aim/armor.hpp"

#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <utility>

namespace
{
using namespace auto_aim;

// [9.9-4] 编译期防止重新引入裸 ID 网络入口；传统灯条入口继续可用。
using Points = std::vector<cv::Point2f>;
static_assert(!std::is_constructible_v<Armor, int, float, cv::Rect, Points>);
static_assert(!std::is_constructible_v<Armor, int, float, cv::Rect, Points, cv::Point2f>);
static_assert(!std::is_constructible_v<Armor, int, int, float, cv::Rect, Points>);
static_assert(!std::is_constructible_v<Armor, int, int, float, cv::Rect, Points, cv::Point2f>);
static_assert(std::is_constructible_v<Armor, const Lightbar &, const Lightbar &>);

void require(bool condition, const char * message)
{
  if (!condition) throw std::runtime_error(message);
}

void near(double actual, double expected, const char * message)
{
  require(std::abs(actual - expected) < 1e-6, message);
}

void reject(const std::function<void()> & operation, const char * message)
{
  try {
    operation();
  } catch (const std::invalid_argument &) {
    return;
  }
  throw std::runtime_error(message);
}

std::vector<cv::Point2f> points()
{
  return {{10, 10}, {110, 10}, {110, 50}, {10, 30}};
}

void check_defaults(const Armor & armor)
{
  require(armor.priority == fifth && armor.class_id == -1 && !armor.duplicated,
    "Priority/class_id/duplicated must have defined defaults");
  require(armor.xyz_in_gimbal.isZero() && armor.xyz_in_world.isZero() &&
    armor.ypr_in_gimbal.isZero() && armor.ypr_in_world.isZero() && armor.ypd_in_world.isZero() &&
    armor.yaw_raw == 0 && armor.pattern.empty(), "Pose and pattern defaults");
  for (const auto * light : {&armor.left, &armor.right}) {
    require(light->id == 0 && light->color == extinguish && light->points.empty() &&
      light->center == cv::Point2f() && light->top == cv::Point2f() &&
      light->bottom == cv::Point2f() && light->top2bottom == cv::Point2f() &&
      light->angle == 0 && light->angle_error == 0 && light->length == 0 &&
      light->width == 0 && light->ratio == 0 && light->rotated_rect.size == cv::Size2f() &&
      light->rotated_rect.center == cv::Point2f() && light->rotated_rect.angle == 0,
      "Default lightbar state must be safe to copy");
  }
}

void test_geometry_and_semantics()
{
  // 不对称梯形：灯条 20/40，最长横边 sqrt(100^2+20^2)，中点连线斜率 0.1。
  const double confidence = 0.912345678901234;
  for (const auto color : {red, blue, extinguish, purple}) {
    for (const auto name : {one, two, three, four, five, sentry, outpost, base}) {
      for (const auto type : {small, big}) {
        Armor armor(color, name, type, confidence, {10, 10, 100, 40}, points(), {640, 512});
        require(armor.color == color && armor.name == name && armor.type == type &&
          armor.confidence == confidence, "Constructor must preserve semantics and double confidence");
        require(armor.points == points() && armor.box == cv::Rect(10, 10, 100, 40),
          "Constructor must preserve point correspondence and bbox");
        near(armor.center.x, 60, "Center x");
        near(armor.center.y, 25, "Center y");
        near(armor.center_norm.x, 0.09375, "Normalized center x");
        near(armor.center_norm.y, 0.048828125, "Normalized center y");
        near(armor.ratio, 2.5495097567963922, "SP25 network geometry ratio changed");
        near(armor.side_ratio, 2, "Lightbar length ratio");
        near(armor.rectangular_error, 0.099668652491162, "SP25 rectangular error changed");
        check_defaults(armor);
      }
    }
  }
}

void test_invalid_entry()
{
  const auto invalid_points = [](std::vector<cv::Point2f> p, const char * label) {
    reject([&] { Armor a(blue, three, small, 0.9, {0, 0, 640, 512}, p, {640, 512}); }, label);
  };
  invalid_points({}, "Accepted empty points");
  invalid_points({{10, 10}, {20, 10}, {20, 20}}, "Accepted three points");
  invalid_points({{10, 10}, {20, 10}, {20, 20}, {10, 20}, {5, 5}}, "Accepted five points");
  invalid_points({{10, 10}, {20, 10}, {30, 10}, {40, 10}}, "Accepted collinear points");
  invalid_points({{10, 10}, {30, 30}, {30, 10}, {10, 30}}, "Accepted self-intersection");
  invalid_points({{10, 10}, {30, 10}, {15, 15}, {10, 30}}, "Accepted concave quad");
  invalid_points({{10, 10}, {10, 30}, {30, 30}, {30, 10}}, "Accepted reversed winding");
  auto p = points();
  p[1] = p[0];
  invalid_points(p, "Accepted repeated point");
  for (const float v : {std::numeric_limits<float>::quiet_NaN(),
       std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity(),
       -1.0F, 640.0F}) {
    p = points();
    p[0].x = v;
    invalid_points(p, "Accepted non-finite or outside-image point");
  }
  for (const cv::Rect & box : {cv::Rect(), cv::Rect(0, 0, -1, 10), cv::Rect(-1, 0, 120, 60),
       cv::Rect(0, 0, 641, 60), cv::Rect(0, 0, 120, 513), cv::Rect(0, 0, 5, 5),
       cv::Rect(1, 0, std::numeric_limits<int>::max(), 60)}) {
    reject([&] { Armor a(blue, three, small, 0.9, box, points(), {640, 512}); },
      "Accepted invalid bbox or bbox that does not enclose keypoints");
  }
  for (const double confidence : {-0.1, 1.1, std::numeric_limits<double>::quiet_NaN(),
       std::numeric_limits<double>::infinity()}) {
    reject([&] { Armor a(blue, three, small, confidence, {10, 10, 100, 40}, points(), {640, 512}); },
      "Accepted invalid confidence");
  }
  reject([] { Armor a(blue, not_armor, small, 0.9, {10, 10, 100, 40}, points(), {640, 512}); },
    "Accepted not_armor");
  reject([] { Armor a(blue, three, small, 0.9, {10, 10, 100, 40}, points(), {0, 512}); },
    "Accepted invalid image dimensions");

  // bbox 为几何包络，右/下角点可落在 x+width、y+height 上；不能用半开 contains 误删。
  Armor border(red, base, big, 1, {600, 480, 39, 31},
    {{600, 480}, {639, 480}, {639, 511}, {600, 511}}, {640, 512});
  require(border.points[2] == cv::Point2f(639, 511), "Valid boundary point was changed");
}

void test_ownership()
{
  auto p = points();
  Armor copied(blue, three, small, 0.9, {10, 10, 100, 40}, p, {640, 512});
  p[0] = {0, 0};
  require(copied.points[0] == cv::Point2f(10, 10), "Caller mutation changed Armor points");
  p = points();
  const auto * storage = p.data();
  Armor moved(blue, three, small, 0.9, {10, 10, 100, 40}, std::move(p), {640, 512});
  require(moved.points.data() == storage, "Network constructor made a redundant vector copy");
  const Armor retained = moved;
  moved.points[0] = {0, 0};
  require(retained.points[0] == cv::Point2f(10, 10), "Armor copies share mutable keypoints");
  check_defaults(retained);
}

// [9.9-4] 传统灯条几何保留，避免网络入口整理误伤其他 SP25 数据构造方式。
void test_traditional_geometry()
{
  Lightbar left(cv::RotatedRect({20, 30}, {4, 20}, 0), 1);
  Lightbar right(cv::RotatedRect({100, 30}, {4, 20}, 0), 2);
  left.color = right.color = blue;
  Armor armor(left, right);
  const Points expected{{20, 20}, {100, 20}, {100, 40}, {20, 40}};
  require(armor.points == expected && armor.color == blue, "Traditional lightbar geometry changed");
  near(armor.ratio, 4, "Traditional ratio");
  near(armor.side_ratio, 1, "Traditional side ratio");
  near(armor.rectangular_error, 0, "Traditional rectangular error");
  require(armor.center_norm == cv::Point2f() && armor.confidence == 0 &&
    armor.priority == fifth && !armor.duplicated && armor.class_id == -1 &&
    armor.xyz_in_world.isZero(), "Traditional entry lost default initialization");
}
}  // namespace

int main()
{
  try {
    test_geometry_and_semantics();
    test_invalid_entry();
    test_ownership();
    test_traditional_geometry();
    std::cout << "PASS: semantic Armor entry, preserved SP25 geometry, defaults, invalid inputs and ownership\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}
