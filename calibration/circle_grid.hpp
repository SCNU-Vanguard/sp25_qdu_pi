#pragma once

#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
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

// 仅整理显示副本的顺序：横向逐行、从上到下；仍使用原 OpenCV 彩色标记和连线。
inline void draw_circle_grid(
  cv::Mat & drawing, const cv::Size & pattern,
  const std::vector<cv::Point2f> & centers, bool found)
{
  if (!found || pattern.width < 2 || pattern.height < 2 ||
      centers.size() != static_cast<size_t>(pattern.area())) return;

  const auto row = centers[pattern.width - 1] - centers[0] +
                   centers.back() - centers[(pattern.height - 1) * pattern.width];
  const auto col = centers[(pattern.height - 1) * pattern.width] - centers[0] +
                   centers.back() - centers[pattern.width - 1];
  const bool transpose = col.x * col.x * row.dot(row) > row.x * row.x * col.dot(col);
  const int rows = transpose ? pattern.width : pattern.height;
  const int cols = transpose ? pattern.height : pattern.width;
  const auto point = [&](int r, int c) -> const cv::Point2f & {
    return centers[transpose ? c * pattern.width + r : r * pattern.width + c];
  };
  const bool flip_rows = point(0, 0).y + point(0, cols - 1).y >
                         point(rows - 1, 0).y + point(rows - 1, cols - 1).y;
  const bool flip_cols = point(0, 0).x + point(rows - 1, 0).x >
                         point(0, cols - 1).x + point(rows - 1, cols - 1).x;
  std::vector<cv::Point2f> display_centers;
  display_centers.reserve(centers.size());
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c)
      display_centers.push_back(point(flip_rows ? rows - 1 - r : r, flip_cols ? cols - 1 - c : c));
  }
  cv::drawChessboardCorners(drawing, {cols, rows}, display_centers, true);
}
}  // namespace calibration
