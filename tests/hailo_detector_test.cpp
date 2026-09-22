// [9.9-2][9.9-3][Pi接入] 真实 YOLO 图片验收；不初始化相机、C 板或控制线程。
#include "tasks/auto_aim/yolo.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <stdexcept>

int main(int argc, char ** argv)
{
  if (argc < 3 || argc > 5) {
    std::cerr << "Usage: hailo_detector_test config.yaml image.jpg [frames=100] [result.jpg]\n";
    return 1;
  }
  try {
    std::size_t parsed = 0;
    const int frames = argc >= 4 ? std::stoi(argv[3], &parsed) : 100;
    if (frames < 1 || frames > 100000 || (argc >= 4 && parsed != std::string(argv[3]).size())) {
      throw std::invalid_argument("frames must be an integer in [1, 100000]");
    }
    if (argc == 5 && std::filesystem::weakly_canonical(argv[2]) ==
                     std::filesystem::weakly_canonical(argv[4])) {
      throw std::invalid_argument("Result image must not overwrite the source image");
    }
    const auto bgr = cv::imread(argv[2], cv::IMREAD_COLOR);
    if (bgr.empty()) throw std::runtime_error("Cannot read input image");
    auto_aim::YOLO detector(argv[1], true);
    for (int i = 0; i < 10; ++i) detector.detect(bgr);
    std::vector<double> times;
    times.reserve(frames);
    std::list<auto_aim::Armor> armors;
    // [Pi接入] 每帧完整执行前处理+推理+反量化+tail+Decoder；图片读盘、画框不计时。
    const auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < frames; ++i) {
      const auto before = std::chrono::steady_clock::now();
      armors = detector.detect(bgr);
      times.push_back(std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - before).count());
    }
    const double seconds = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start).count();
    const double average = std::accumulate(times.begin(), times.end(), 0.0) / frames;
    std::sort(times.begin(), times.end());
    const auto p95 = static_cast<std::size_t>(std::ceil(frames * 0.95)) - 1;
    std::cout << std::fixed << std::setprecision(3)
      << "static_image_detector_fps=" << frames / seconds
      << " ms(avg/p95/max)=" << average << '/' << times[p95] << '/' << times.back()
      << " last_detections=" << armors.size() << '\n';
    cv::Mat drawing;
    if (argc == 5) drawing = bgr.clone();
    for (const auto & armor : armors) {
      const auto label = auto_aim::COLORS[armor.color] + " " + auto_aim::ARMOR_NAMES[armor.name];
      std::cout << label << ' ' << auto_aim::ARMOR_TYPES[armor.type]
        << " confidence=" << armor.confidence << " corners(LT,RT,RB,LB)=";
      for (int i = 0; i < 4; ++i) {
        std::cout << armor.points[i] << ' ';
        if (!drawing.empty()) {
          cv::line(drawing, armor.points[i], armor.points[(i + 1) % 4], {0, 255, 0}, 2);
          cv::putText(drawing, std::to_string(i), armor.points[i], cv::FONT_HERSHEY_SIMPLEX,
            0.5, {0, 255, 255}, 1);
        }
      }
      std::cout << '\n';
      if (!drawing.empty()) cv::putText(drawing, label, armor.points[0] + cv::Point2f(0, -8),
        cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 255, 0}, 1);
    }
    if (!drawing.empty() && !cv::imwrite(argv[4], drawing)) throw std::runtime_error("Cannot save result image");
    std::cout << "Includes preprocessing and all detection stages; excludes camera, tracking, aiming and control.\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "Hailo detector test failed: " << error.what() << '\n';
    return 1;
  }
}
