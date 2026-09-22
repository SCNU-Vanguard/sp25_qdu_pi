#include "lightbar_refiner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <utility>

namespace auto_aim
{
namespace
{
// [Pi灯条] 沿用原SP25灯条/装甲几何检查的含义；当前只接近直立板，允许±45度倾斜。
constexpr double max_light_angle = 45.0 * CV_PI / 180;
constexpr double max_rectangular_error = 25.0 * CV_PI / 180;
constexpr double max_parallel_error = 20.0 * CV_PI / 180;
constexpr std::size_t max_lights = 128, max_side_matches = 8;

bool inside(const cv::Point2f & p, cv::Size size)
{
  return std::isfinite(p.x) && std::isfinite(p.y) && p.x >= 0 && p.y >= 0 &&
         p.x <= size.width - 1 && p.y <= size.height - 1;
}

bool valid_quad(const std::vector<cv::Point2f> & points, cv::Size size)
{
  for (std::size_t i = 0; i < 4; ++i) {
    if (!inside(points[i], size)) return false;
    const auto a = points[(i + 1) % 4] - points[i];
    const auto b = points[(i + 2) % 4] - points[(i + 1) % 4];
    if (static_cast<double>(a.x) * b.y - static_cast<double>(a.y) * b.x <= 1e-6)
      return false;
  }
  return true;
}

cv::Rect enclosing_box(const std::vector<cv::Point2f> & points)
{
  float x0 = points[0].x, x1 = x0, y0 = points[0].y, y1 = y0;
  for (const auto & p : points) {
    x0 = std::min(x0, p.x); x1 = std::max(x1, p.x);
    y0 = std::min(y0, p.y); y1 = std::max(y1, p.y);
  }
  const int x = static_cast<int>(std::floor(x0)), y = static_cast<int>(std::floor(y0));
  return {x, y, static_cast<int>(std::ceil(x1)) - x, static_cast<int>(std::ceil(y1)) - y};
}

double side_cost(const cv::Point2f & top, const cv::Point2f & bottom,
                 const Lightbar & light, double limit)
{
  // [Pi灯条] 以实测长度归一化，替代原版固定15像素总误差；仍限制移动和尺度变化。
  const double predicted_length = cv::norm(bottom - top);
  if (predicted_length < light.length * 0.4 || predicted_length > light.length * 2.0)
    return std::numeric_limits<double>::infinity();
  const double d0 = cv::norm(top - light.top) / light.length;
  const double d1 = cv::norm(bottom - light.bottom) / light.length;
  return std::max(d0, d1) <= limit ? (d0 + d1) * 0.5 : std::numeric_limits<double>::infinity();
}
}  // namespace

LightbarRefiner::LightbarRefiner(LightbarRefineOptions options) : options_(options)
{
  // [Pi灯条] 配置先校验再开NPU；不接受会无界放宽匹配的参数。
  if (options_.threshold < 1 || options_.threshold > 254 ||
      !std::isfinite(options_.min_length) || options_.min_length < 2 ||
      !std::isfinite(options_.max_corner_shift_ratio) ||
      options_.max_corner_shift_ratio <= 0 || options_.max_corner_shift_ratio > 1 ||
      !std::isfinite(options_.big_armor_ratio) || options_.big_armor_ratio <= 0)
    throw std::invalid_argument("LightbarRefiner: invalid threshold, length or corner-shift ratio");
}

std::list<Armor> LightbarRefiner::refine(const cv::Mat & bgr, std::list<Armor> candidates)
{
  stats_ = {};
  stats_.input = candidates.size();
  if (!options_.enabled) { stats_.output = candidates.size(); return candidates; }
  if (bgr.empty() || bgr.type() != CV_8UC3 || bgr.dims != 2)
    throw std::invalid_argument("LightbarRefiner requires CV_8UC3 BGR");
  if (candidates.empty()) return {};
  // [Pi灯条] 原SP25中Detector::detect返回false时，YOLOv5仍保留网络候选。
  // 保存未改写的副本：当前帧没有可靠灯条时不把“修正失败”变成“检测掉框”。
  const auto network_fallback = candidates;

  // [Pi灯条] 每帧只提取一次灯条，避免三个候选重复找同一灯条却拿到不同ID；不修改输入图。
  // [Pi灯条·实拍] 红/蓝灯条的单色通道可能已饱和，但灰度仍低于150；灰度阈值会截短端点。
  // 分别读取红/蓝通道并取较亮者，只改变找灯条的像素掩码；后续颜色、几何和网络候选检查保持原样。
  cv::extractChannel(bgr, blue_, 0);
  cv::extractChannel(bgr, red_, 2);
  cv::max(blue_, red_, bright_);
  cv::threshold(bright_, binary_, options_.threshold, 255, cv::THRESH_BINARY);
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary_, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  std::vector<Lightbar> lights;
  for (const auto & contour : contours) {
    if (contour.size() < 4) continue;
    const auto bounds = cv::boundingRect(contour);
    // [Pi灯条] 触边的亮斑可能是被截断的灯条，不拿它补出看似完整的角点。
    if (bounds.x <= 0 || bounds.y <= 0 || bounds.br().x >= bgr.cols || bounds.br().y >= bgr.rows)
      continue;
    Lightbar light(cv::minAreaRect(contour), lights.size());
    if (!std::isfinite(light.ratio) || light.length < options_.min_length ||
        light.width < 1 || light.ratio < 1.5 || light.ratio > 20 ||
        light.angle_error > max_light_angle || !inside(light.top, bgr.size()) ||
        !inside(light.bottom, bgr.size())) continue;
    // [Pi灯条] 白亮芯外通常有蓝/红光晕；局部扩2..8像素读色，纯白反光不指定为敌方颜色。
    const int pad = std::clamp(static_cast<int>(std::ceil(light.width * 0.5)), 2, 8);
    const auto color_box = cv::Rect(bounds.x - pad, bounds.y - pad,
      bounds.width + 2 * pad, bounds.height + 2 * pad) & cv::Rect(0, 0, bgr.cols, bgr.rows);
    const auto mean = cv::mean(bgr(color_box));
    const double difference = mean[0] - mean[2];
    if (std::abs(difference) < 5.0) continue;
    light.color = difference > 0 ? blue : red;
    lights.push_back(std::move(light));
  }
  stats_.lights = lights.size();
  // [Pi灯条] 高噪声场景跳过修正，避免候选×灯条配对耗时失控；仍保留网络框。
  if (lights.size() > max_lights) {
    stats_.ambiguous = candidates.size();
    stats_.fallback = network_fallback.size();
    stats_.output = network_fallback.size();
    return network_fallback;
  }

  // [Pi灯条] 相同物理灯条对只输出一次；颜色/编号取匹配该灯条对的最高分候选。
  candidates.sort([](const Armor & a, const Armor & b) { return a.confidence > b.confidence; });
  std::set<std::pair<std::size_t, std::size_t>> used_pairs;
  std::list<Armor> result;
  for (const auto & candidate : candidates) {
    std::vector<std::pair<std::size_t, double>> left, right;
    if (candidate.points.size() != 4) { ++stats_.unmatched; continue; }
    for (std::size_t i = 0; i < lights.size(); ++i) {
      if (lights[i].color != candidate.color) continue;
      const double lc = side_cost(candidate.points[0], candidate.points[3], lights[i],
                                  options_.max_corner_shift_ratio);
      const double rc = side_cost(candidate.points[1], candidate.points[2], lights[i],
                                  options_.max_corner_shift_ratio);
      if (std::isfinite(lc)) left.emplace_back(i, lc);
      if (std::isfinite(rc)) right.emplace_back(i, rc);
    }
    if (left.size() > max_side_matches || right.size() > max_side_matches) {
      ++stats_.ambiguous;
      continue;
    }
    double best = std::numeric_limits<double>::infinity(), second = best;
    std::pair<std::size_t, std::size_t> chosen{};
    for (const auto & [li, lc] : left) for (const auto & [ri, rc] : right) {
      if (li == ri || lights[li].center.x >= lights[ri].center.x) continue;
      const Armor geometry(lights[li], lights[ri]);
      if (geometry.ratio < 1.0 || geometry.ratio > 5.0 || geometry.side_ratio > 1.5 ||
          geometry.rectangular_error > max_rectangular_error ||
          std::abs(lights[li].angle - lights[ri].angle) > max_parallel_error ||
          !valid_quad(geometry.points, bgr.size())) continue;
      const double cost = (lc + rc) * 0.5;
      if (cost < best) { second = best; best = cost; chosen = {li, ri}; }
      else if (cost < second) second = cost;
    }
    if (!std::isfinite(best)) { ++stats_.unmatched; continue; }
    // [Pi灯条] 两种灯条组合几乎同样合理时不“挑一个凑数”；若全帧都失败则回退网络框。
    if (second - best < 0.15) { ++stats_.ambiguous; continue; }
    if (!used_pairs.insert(chosen).second) { ++stats_.duplicates; continue; }
    const auto & l = lights[chosen.first];
    const auto & r = lights[chosen.second];
    std::vector<cv::Point2f> points{l.top, r.top, r.bottom, l.bottom};
    // [Pi灯条] 重建Armor以同步所有派生几何；仅改points会让Tracker/PnP看到矛盾数据。
    Armor corrected(candidate.color, candidate.name, small, candidate.confidence,
                    enclosing_box(points), points, bgr.size());
    corrected.left = l;
    corrected.right = r;
    corrected.priority = candidate.priority;
    corrected.class_id = candidate.class_id;
    const auto name = corrected.name;
    corrected.type = name == one ? big : small;
    if (name == three || name == four || name == five || name == base)
      corrected.type = corrected.ratio >= options_.big_armor_ratio ? big : small;
    result.push_back(std::move(corrected));
  }
  if (result.empty()) {
    // [Pi灯条] 仅当这一帧没有任何可靠修正结果时回退，避免已修正的单板旁又出现
    // 未修正的重复框。跨帧连续目标由后续Tracker处理，本模块不伪造历史状态。
    stats_.fallback = network_fallback.size();
    stats_.output = network_fallback.size();
    return network_fallback;
  }
  stats_.output = result.size();
  return result;
}
}  // namespace auto_aim
