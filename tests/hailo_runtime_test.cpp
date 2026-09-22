// [9.9-2] HEF + 静态图片验证运行时；不使用相机或控制接口，不代表整链路 FPS。
#include "tasks/auto_aim/hailo/hailo_runtime.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

int main(int argc, char ** argv)
{
  if (argc < 3 || argc > 5) {
    std::cerr << "Usage: hailo_runtime_test model.hef image.jpg [frames=100] [native|float32]\n"
                 "Contract: 640x512 UINT8 RGB, three NHWC grids with 66 features (3x22).\n";
    return 1;
  }
  try {
    std::size_t parsed = 0;
    const int frames = argc >= 4 ? std::stoi(argv[3], &parsed) : 100;
    if (frames <= 0 || frames > 100000 || (argc >= 4 && parsed != std::string(argv[3]).size())) {
      throw std::invalid_argument("frames must be an integer in [1, 100000]");
    }
    auto_aim::HailoRuntimeOptions options;
    if (argc == 5) {
      const std::string format = argv[4];
      if (format != "native" && format != "float32") {
        throw std::invalid_argument("output format must be native or float32");
      }
      options.float32_output = format == "float32";
    }
    cv::Mat bgr = cv::imread(argv[2], cv::IMREAD_COLOR);
    if (bgr.empty()) throw std::runtime_error("Cannot read input image");
    auto_aim::HailoRuntime runtime(argv[1], options);
    std::cout << "Input: " << runtime.input_name() << " " << runtime.input_size() << " RGB UINT8\n";
    for (const auto & head : runtime.output_info()) {
      const char * type = head.type == auto_aim::HailoElementType::uint8 ? "UINT8" :
        head.type == auto_aim::HailoElementType::uint16 ? "UINT16" : "FLOAT32";
      std::cout << "Output: " << head.name << " grid=" << head.grid_size << " features=" <<
        head.features << " type=" << type << " stride=" << head.stride << " first_row=" <<
        head.first_row << " quant_entries=" << head.quantization.size() << '\n';
      if (!head.quantization.empty()) {
        std::cout << "  first_quant: zp=" << head.quantization.front().zero_point <<
          " scale=" << head.quantization.front().scale << '\n';
      }
    }

    // [9.9-2] 此测试重复同一张图：只预处理一次，计时明确排除相机和图像预处理。
    cv::Mat resized, rgb, fused;
    cv::resize(bgr, resized, runtime.input_size());
    cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);
    for (int warmup = 0; warmup < 10; ++warmup) runtime.infer(rgb, fused);
    std::vector<double> times;
    times.reserve(frames);
    const auto start = std::chrono::steady_clock::now();
    for (int frame = 0; frame < frames; ++frame) {
      const auto before = std::chrono::steady_clock::now();
      runtime.infer(rgb, fused);
      times.push_back(std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - before).count());
    }
    const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    const double average = std::accumulate(times.begin(), times.end(), 0.0) / times.size();
    std::sort(times.begin(), times.end());
    const auto p95 = static_cast<std::size_t>(std::ceil(times.size() * 0.95)) - 1;
    double minimum, maximum;
    cv::minMaxLoc(fused, &minimum, &maximum);
    std::cout << std::fixed << std::setprecision(3) <<
      "raw_infer_and_fuse_fps=" << frames / seconds <<
      " ms(avg/p95/max)=" << average << '/' << times[p95] << '/' << times.back() << '\n' <<
      "Fused=" << fused.rows << 'x' << fused.cols << " min=" << minimum << " max=" << maximum << '\n';
    std::cout << "Static-image runtime test only; excludes camera, preprocessing, decoding and aiming.\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "Hailo runtime test failed: " << error.what() << '\n';
    return 1;
  }
}
