#pragma once

#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

namespace calibration
{
// 采图和离线求解共用；每次只检测当前原图，不保留上一帧的圆心。
inline bool find_circle_grid(
  const cv::Mat & image, const cv::Size & pattern, std::vector<cv::Point2f> & centers)
{
  centers.clear();
  if (cv::findCirclesGrid(image, pattern, centers, cv::CALIB_CB_SYMMETRIC_GRID)) return true;

  // 保留原来的面积/形状筛选，只扩大较暗圆点的阈值搜索范围。
  static const auto detector = [] {
    cv::SimpleBlobDetector::Params params;
    params.minThreshold = 5;
    params.thresholdStep = 5;
    return cv::SimpleBlobDetector::create(params);
  }();
  centers.clear();
  const bool found = cv::findCirclesGrid(
    image, pattern, centers, cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, detector);
  if (!found) centers.clear();
  return found;
}

// 只调整显示：选更接近画面水平方向的一组行，不改变求解用的圆心顺序。
inline void draw_circle_grid(
  cv::Mat & drawing, const cv::Size & pattern,
  const std::vector<cv::Point2f> & centers, bool found)
{
  if (!found || pattern.width < 2 || pattern.height < 2 ||
      centers.size() != static_cast<size_t>(pattern.area())) return;

  const auto row = centers[pattern.width - 1] - centers[0];
  const auto col = centers[(pattern.height - 1) * pattern.width] - centers[0];
  const bool transpose = col.x * col.x * row.dot(row) > row.x * row.x * col.dot(col);
  const int rows = transpose ? pattern.width : pattern.height;
  const int cols = transpose ? pattern.height : pattern.width;
  for (int r = 0; r < rows; ++r) {
    for (int c = 1; c < cols; ++c) {
      const auto & a = centers[transpose ? (c - 1) * pattern.width + r : r * pattern.width + c - 1];
      const auto & b = centers[transpose ? c * pattern.width + r : r * pattern.width + c];
      // 统一从左向右绘制。
      cv::line(drawing, a.x <= b.x ? a : b, a.x <= b.x ? b : a, {0, 220, 0}, 1, cv::LINE_AA);
    }
  }
  for (const auto & center : centers)
    cv::circle(drawing, center, 3, {0, 220, 255}, 1, cv::LINE_AA);
}
}  // namespace calibration
