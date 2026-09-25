#pragma once

#include <yaml-cpp/yaml.h>
#include <cmath>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <string>
#include <vector>

namespace calibration
{
// This workflow deliberately accepts aligned (symmetric) circles only. A staggered board needs
// different object coordinates; silently using the rectangular grid would produce wrong results.
struct CircleGrid
{
  cv::Size size;
  double spacing_mm;

  explicit CircleGrid(const std::string & path)
  {
    const auto yaml = YAML::LoadFile(path);
    size = {yaml["pattern_cols"].as<int>(), yaml["pattern_rows"].as<int>()};
    spacing_mm = yaml["center_distance_mm"].as<double>();
    if (size.width < 2 || size.height < 2 || size.width > 100 || size.height > 100 ||
        !std::isfinite(spacing_mm) || spacing_mm <= 0)
      throw std::runtime_error("invalid circle count or center_distance_mm");
    if (yaml["pattern_asymmetric"] && yaml["pattern_asymmetric"].as<bool>())
      throw std::runtime_error("this workflow requires an aligned symmetric circle grid");
  }

  bool find(const cv::Mat & image, std::vector<cv::Point2f> & centers) const
  {
    return cv::findCirclesGrid(image, size, centers, cv::CALIB_CB_SYMMETRIC_GRID);
  }

  std::vector<cv::Point3f> points() const
  {
    std::vector<cv::Point3f> result;
    for (int row = 0; row < size.height; ++row)
      for (int col = 0; col < size.width; ++col)
        result.emplace_back(col * spacing_mm, row * spacing_mm, 0);
    return result;
  }
};
}  // namespace calibration
