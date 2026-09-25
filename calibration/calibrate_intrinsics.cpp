// Offline, headless calibration of numbered original images captured by capture_intrinsics.
// The result is a review artifact; this program never updates a production configuration.
#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <regex>

#include "calibration/circle_grid.hpp"

namespace
{
const std::string keys =
  "{help h         |      | show help}"
  "{@input-folder  |      | directory containing numbered original PNG/JPG files}"
  "{pattern-config |      | default: input-folder/pattern-used.yaml}"
  "{output-file    |      | default: input-folder/intrinsics.yaml; must not exist}"
  "{review-folder  |      | default: input-folder/review; must not exist}"
  "{min-views      | 20   | minimum successfully detected views}";

bool finite_matrix(const cv::Mat & matrix)
{
  return cv::checkRange(matrix);
}
}

int main(int argc, char ** argv)
{
  try {
    cv::CommandLineParser cli(argc, argv, keys);
    if (cli.has("help")) { cli.printMessage(); return 0; }
    const std::filesystem::path input(cli.get<std::string>(0));
    auto pattern = cli.get<std::string>("pattern-config");
    auto output = cli.get<std::string>("output-file");
    auto review = cli.get<std::string>("review-folder");
    const int min_views = cli.get<int>("min-views");
    if (!cli.check() || input.empty() || !std::filesystem::is_directory(input) || min_views < 3) {
      cli.printErrors();
      throw std::runtime_error("provide an input directory and --min-views >= 3");
    }
    if (pattern.empty()) pattern = (input / "pattern-used.yaml").string();
    if (output.empty()) output = (input / "intrinsics.yaml").string();
    if (review.empty()) review = (input / "review").string();
    const calibration::CircleGrid grid(pattern);
    if (std::filesystem::exists(output) || std::filesystem::exists(review))
      throw std::runtime_error("result/review already exists; choose new output paths");

    std::vector<std::filesystem::path> files;
    const std::regex original_name("^[0-9]+\\.(png|jpg|jpeg)$", std::regex::icase);
    for (const auto & entry : std::filesystem::directory_iterator(input))
      if (entry.is_regular_file() && std::regex_match(entry.path().filename().string(), original_name))
        files.push_back(entry.path());
    std::sort(files.begin(), files.end());
    if (files.empty()) throw std::runtime_error("no numbered original PNG/JPG images found");
    std::filesystem::create_directories(review);
    std::vector<std::vector<cv::Point2f>> image_points;
    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<std::string> accepted;
    cv::Size size;
    for (const auto & path : files) {
      const auto image = cv::imread(path.string());
      if (image.empty()) throw std::runtime_error("cannot read " + path.string());
      if (size.empty()) size = image.size();
      if (image.size() != size)
        throw std::runtime_error("mixed image sizes: " + path.string());
      std::vector<cv::Point2f> centers;
      const bool found = grid.find(image, centers);
      auto drawing = image.clone();
      cv::drawChessboardCorners(drawing, grid.size, centers, found);
      if (!cv::imwrite((std::filesystem::path(review) / (path.stem().string() + ".png")).string(), drawing))
        throw std::runtime_error("cannot save circle review image");
      std::cout << fmt::format("[{}] {}\n", found ? "accepted" : "not found", path.filename().string())
                << std::flush;
      if (!found) continue;
      accepted.push_back(path.filename().string());
      image_points.push_back(centers);
      object_points.push_back(grid.points());
    }
    if (accepted.size() < static_cast<std::size_t>(min_views))
      throw std::runtime_error(fmt::format("only {} valid views; require {}", accepted.size(), min_views));

    cv::Mat camera_matrix, distort_coeffs;
    std::vector<cv::Mat> rotations, translations;
    // Retain the existing SP25 five-coefficient model with k3 fixed to zero.
    const double rms = cv::calibrateCamera(object_points, image_points, size, camera_matrix,
      distort_coeffs, rotations, translations, cv::CALIB_FIX_K3,
      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-12));
    if (!std::isfinite(rms) || !finite_matrix(camera_matrix) || !finite_matrix(distort_coeffs) ||
        camera_matrix.at<double>(0, 0) <= 0 || camera_matrix.at<double>(1, 1) <= 0)
      throw std::runtime_error("calibration produced invalid parameters");
    std::vector<double> per_view_rms;
    double distance_sum = 0;
    std::size_t point_count = 0;
    for (std::size_t i = 0; i < image_points.size(); ++i) {
      std::vector<cv::Point2f> projected;
      cv::projectPoints(object_points[i], rotations[i], translations[i], camera_matrix,
                        distort_coeffs, projected);
      double squared_sum = 0;
      for (std::size_t j = 0; j < projected.size(); ++j) {
        const double distance = cv::norm(projected[j] - image_points[i][j]);
        distance_sum += distance;
        squared_sum += distance * distance;
      }
      point_count += projected.size();
      const double error = std::sqrt(squared_sum / projected.size());
      per_view_rms.push_back(error);
      std::cout << fmt::format("view_rms_px={:.4f} file={}\n", error, accepted[i]);
    }
    YAML::Emitter result;
    const std::vector<double> matrix(camera_matrix.begin<double>(), camera_matrix.end<double>());
    const std::vector<double> distortion(distort_coeffs.begin<double>(), distort_coeffs.end<double>());
    result.SetDoublePrecision(17);
    result << YAML::BeginMap
      << YAML::Key << "image_width" << YAML::Value << size.width
      << YAML::Key << "image_height" << YAML::Value << size.height
      << YAML::Key << "pattern_cols" << YAML::Value << grid.size.width
      << YAML::Key << "pattern_rows" << YAML::Value << grid.size.height
      << YAML::Key << "center_distance_mm" << YAML::Value << grid.spacing_mm
      << YAML::Key << "accepted_views" << YAML::Value << accepted.size()
      << YAML::Key << "calibration_flags" << YAML::Value << "CALIB_FIX_K3"
      << YAML::Key << "rms_px" << YAML::Value << rms
      << YAML::Key << "mean_point_error_px" << YAML::Value << distance_sum / point_count
      << YAML::Key << "camera_matrix" << YAML::Value << YAML::Flow << matrix
      << YAML::Key << "distort_coeffs" << YAML::Value << YAML::Flow << distortion
      << YAML::Key << "per_view" << YAML::Value << YAML::BeginSeq;
    for (std::size_t i = 0; i < accepted.size(); ++i)
      result << YAML::BeginMap << YAML::Key << "file" << YAML::Value << accepted[i]
        << YAML::Key << "rms_px" << YAML::Value << per_view_rms[i] << YAML::EndMap;
    result << YAML::EndSeq << YAML::EndMap;
    if (!result.good()) throw std::runtime_error("cannot serialize calibration");
    std::ofstream out(output);
    out.exceptions(std::ios::badbit | std::ios::failbit);
    out << result.c_str() << '\n';
    out.close();
    std::cout << fmt::format(
      "Calibration result: {}\nReview images: {}\nRMS={:.4f}px; mean point error={:.4f}px\n"
      "REVIEW REQUIRED: image coverage, pose diversity and independent measurements matter too.\n"
      "Production YAML was not modified.\n", output, review, rms, distance_sum / point_count);
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "calibrate_intrinsics failed: " << error.what() << '\n';
    return 1;
  }
}
