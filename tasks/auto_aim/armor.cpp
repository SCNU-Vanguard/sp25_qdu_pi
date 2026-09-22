#include "armor.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <utility>

namespace auto_aim
{
// [9.9-4] 语义入口独立初始化，不再借用旧 class_id 构造，也不重复复制四角点 vector。
Armor::Armor(
  Color color, ArmorName name, ArmorType type, double confidence, const cv::Rect & box,
  std::vector<cv::Point2f> keypoints, cv::Size image_size)
: color(color), points(std::move(keypoints)), type(type), name(name), box(box), confidence(confidence)
{
  if (color < red || color > purple || name < one || name >= not_armor ||
      (type != big && type != small) || !std::isfinite(confidence) || confidence < 0 ||
      confidence > 1 || image_size.width <= 0 || image_size.height <= 0 || points.size() != 4 ||
      box.x < 0 || box.y < 0 || box.width <= 0 || box.height <= 0 ||
      static_cast<double>(box.x) + box.width > image_size.width ||
      static_cast<double>(box.y) + box.height > image_size.height) {
    throw std::invalid_argument("Armor: invalid semantics, image, bounding box or point count");
  }

  // [9.9-4] 即使直接调用 Armor，也要拒绝坏几何；先校验全部点，再做差/除法/角度计算。
  const double right_edge = static_cast<double>(box.x) + box.width;
  const double bottom_edge = static_cast<double>(box.y) + box.height;
  for (const auto & p : points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || p.x < 0 || p.y < 0 ||
        static_cast<double>(p.x) > image_size.width - 1 ||
        static_cast<double>(p.y) > image_size.height - 1 ||
        p.x < box.x || p.y < box.y || p.x > right_edge || p.y > bottom_edge) {
      throw std::invalid_argument("Armor: non-finite/outside-image point or bbox does not enclose points");
    }
  }
  for (std::size_t i = 0; i < points.size(); ++i) {
    const auto a = points[(i + 1) % 4] - points[i];
    const auto b = points[(i + 2) % 4] - points[(i + 1) % 4];
    const double cross = static_cast<double>(a.x) * b.y - static_cast<double>(a.y) * b.x;
    if (cross <= 1e-6 || cv::norm(a) < 1e-3) {
      throw std::invalid_argument("Armor: expected a non-degenerate convex LT/RT/RB/LB quadrilateral");
    }
  }

  // [9.9-4] 保留 SP25 网络入口的中心、宽高比和矩形夹角公式，仅合并原来的重复实现。
  center = (points[0] + points[1] + points[2] + points[3]) / 4;
  center_norm = {center.x / image_size.width, center.y / image_size.height};
  const auto left_width = cv::norm(points[0] - points[3]);
  const auto right_width = cv::norm(points[1] - points[2]);
  const auto top_length = cv::norm(points[0] - points[1]);
  const auto bottom_length = cv::norm(points[3] - points[2]);
  ratio = std::max(top_length, bottom_length) / std::max(left_width, right_width);
  side_ratio = std::max(left_width, right_width) / std::min(left_width, right_width);
  const auto left_center = (points[0] + points[3]) / 2;
  const auto right_center = (points[1] + points[2]) / 2;
  const auto left2right = right_center - left_center;
  const auto roll = std::atan2(left2right.y, left2right.x);
  const auto left_rectangular_error = std::abs(
    std::atan2((points[3] - points[0]).y, (points[3] - points[0]).x) - roll - CV_PI / 2);
  const auto right_rectangular_error = std::abs(
    std::atan2((points[2] - points[1]).y, (points[2] - points[1]).x) - roll - CV_PI / 2);
  rectangular_error = std::max(left_rectangular_error, right_rectangular_error);
}

// [9.9-4] 以下 SP25 灯条及传统 Armor 几何实现保留。
Lightbar::Lightbar(const cv::RotatedRect & rotated_rect, std::size_t id)
: id(id), rotated_rect(rotated_rect)
{
  std::vector<cv::Point2f> corners(4);
  rotated_rect.points(&corners[0]);
  std::sort(corners.begin(), corners.end(), [](const cv::Point2f & a, const cv::Point2f & b) {
    return a.y < b.y;
  });

  center = rotated_rect.center;
  top = (corners[0] + corners[1]) / 2;
  bottom = (corners[2] + corners[3]) / 2;
  top2bottom = bottom - top;

  points.emplace_back(top);
  points.emplace_back(bottom);

  width = cv::norm(corners[0] - corners[1]);
  angle = std::atan2(top2bottom.y, top2bottom.x);
  angle_error = std::abs(angle - CV_PI / 2);
  length = cv::norm(top2bottom);
  ratio = length / width;
}

//传统构造函数
Armor::Armor(const Lightbar & left, const Lightbar & right)
: left(left), right(right), duplicated(false)
{
  color = left.color;
  center = (left.center + right.center) / 2;

  points.emplace_back(left.top);
  points.emplace_back(right.top);
  points.emplace_back(right.bottom);
  points.emplace_back(left.bottom);

  auto left2right = right.center - left.center;
  auto width = cv::norm(left2right);
  auto max_lightbar_length = std::max(left.length, right.length);
  auto min_lightbar_length = std::min(left.length, right.length);
  ratio = width / max_lightbar_length;
  side_ratio = max_lightbar_length / min_lightbar_length;

  auto roll = std::atan2(left2right.y, left2right.x);
  auto left_rectangular_error = std::abs(left.angle - roll - CV_PI / 2);
  auto right_rectangular_error = std::abs(right.angle - roll - CV_PI / 2);
  rectangular_error = std::max(left_rectangular_error, right_rectangular_error);
}

}  // namespace auto_aim
