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
}  // namespace calibration
