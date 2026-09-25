/* Standalone build:
g++ -std=c++17 -O2 -I. tests/calibration/circle_grid_test.cpp \
  $(pkg-config --cflags --libs opencv4) -o build/circle_grid_test
*/
#include <opencv2/opencv.hpp>

#include <iostream>
#include <set>
#include <stdexcept>

#include "calibration/circle_grid.hpp"

void require(bool condition, const char * message)
{
  if (!condition) throw std::runtime_error(message);
}

cv::Mat board(double tilt, int light, double blur, std::vector<cv::Point2f> & truth)
{
  const cv::Matx33d camera(900, 0, 359.5, 0, 900, 269.5, 0, 0, 1);
  const cv::Vec3d rotation(tilt, tilt * 0.3, 0.1), translation(0, 0, 850);
  std::vector<cv::Point3f> centers;
  cv::Mat image(1080, 1440, CV_8UC1, cv::Scalar(light));
  for (int row = 0; row < 7; ++row) {
    for (int col = 0; col < 7; ++col) {
      const cv::Point3f center((col - 3) * 30, (row - 3) * 30, 0);
      centers.push_back(center);
      std::vector<cv::Point3f> circle;
      for (int i = 0; i < 64; ++i) {
        const double angle = 2 * CV_PI * i / 64;
        circle.emplace_back(center.x + 7.5 * std::cos(angle), center.y + 7.5 * std::sin(angle), 0);
      }
      std::vector<cv::Point2f> projected;
      cv::projectPoints(circle, rotation, translation, camera, cv::noArray(), projected);
      std::vector<cv::Point> polygon;
      for (const auto & point : projected) polygon.emplace_back(cvRound(2 * point.x), cvRound(2 * point.y));
      cv::fillConvexPoly(image, polygon, cv::Scalar(light * 0.15), cv::LINE_AA);
    }
  }
  cv::projectPoints(centers, rotation, translation, camera, cv::noArray(), truth);
  cv::resize(image, image, {720, 540}, 0, 0, cv::INTER_AREA);
  if (blur > 0) cv::GaussianBlur(image, image, {}, blur);
  return image;
}

int main(int argc, char ** argv) try
{
  int baseline = 0, passed = 0;
  double max_error = 0;
  for (double tilt : {0.0, 0.4, 0.8, 1.0}) {
    for (int light : {40, 70, 130, 220}) {
      for (double blur : {0.0, 0.8, 1.4}) {
        std::vector<cv::Point2f> truth, detected, original;
        const auto image = board(tilt, light, blur, truth);
        baseline += cv::findCirclesGrid(image, {7, 7}, original);
        require(calibration::find_circle_grid(image, {7, 7}, detected), "missed complete grid");
        require(detected.size() == truth.size(), "wrong circle count");
        std::set<int> matched;
        for (const auto & point : detected) {
          double error = 1e9;
          int index = -1;
          for (int i = 0; i < static_cast<int>(truth.size()); ++i) {
            const double distance = cv::norm(point - truth[i]);
            if (distance < error) { error = distance; index = i; }
          }
          require(error < 0.8 && matched.insert(index).second, "wrong or inaccurate circle center");
          max_error = std::max(max_error, error);
        }
        ++passed;
      }
    }
  }
  std::vector<cv::Point2f> truth, detected;
  auto missing = board(0, 200, 0, truth);
  cv::circle(missing, truth[24], 14, cv::Scalar(200), cv::FILLED);
  require(!calibration::find_circle_grid(missing, {7, 7}, detected), "accepted incomplete board");
  require(detected.empty(), "retained partial or old centers");
  require(!calibration::find_circle_grid(cv::Mat(540, 720, CV_8UC1, cv::Scalar(80)), {7, 7}, detected),
          "accepted blank frame");
  for (int i = 1; i < argc; ++i) {
    const auto image = cv::imread(argv[i]);
    require(!image.empty(), "could not read negative fixture");
    require(!calibration::find_circle_grid(image, {7, 7}, detected), "accepted board-free frame");
  }
  std::cout << "PASS: original=" << baseline << "/48, updated=" << passed
            << "/48, max center error=" << max_error << " px; incomplete, blank and "
            << argc - 1 << " board-free frames rejected\n";
  return 0;
}
catch (const std::exception & error) {
  std::cerr << error.what() << '\n';
  return 1;
}
