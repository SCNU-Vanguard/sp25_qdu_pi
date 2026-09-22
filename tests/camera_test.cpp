#include "io/camera.hpp"

#include <iostream>
#include <opencv2/opencv.hpp>
#include <thread>

// [9.9-6] 独立海康单测复用本程序，避免为了测相机而要求 Hailo/JSON/云台依赖。
#ifdef SP_HIK_ONLY
#include "io/hikrobot/hikrobot.hpp"
#endif

#include "tools/exiter.hpp"
#include "tools/logger.hpp"

const std::string keys =
  "{help h usage ? |                     | 输出命令行参数说明}"
  "{config-path c  | configs/camera.yaml | yaml配置文件路径 }"
  "{d display      |                     | 显示视频流       }"
  "{seconds        | 10                  | 测试秒数，0 表示直到 Ctrl-C }"
  "{slow-ms        | 0                   | 模拟消费者每帧额外耗时毫秒 }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;

  auto config_path = cli.get<std::string>("config-path");
  auto display = cli.has("display");
  const auto seconds = cli.get<int>("seconds");
  const auto slow_ms = cli.get<int>("slow-ms");
  if (!cli.check() || seconds < 0 || slow_ms < 0 || slow_ms > 1000) {
    cli.printErrors();
    std::cerr << "seconds must be nonnegative; slow-ms must be in [0, 1000]\n";
    return 1;
  }
  try {
#ifdef SP_HIK_ONLY
    const auto yaml = YAML::LoadFile(config_path);
    if (yaml["camera_name"].as<std::string>() != "hikrobot")
      throw std::invalid_argument("hik_camera_test requires camera_name: hikrobot");
    io::HikRobot camera(io::HikRobotOptions::from_yaml(yaml));
#else
    io::Camera camera(config_path);
#endif

    // [9.9-5] 无预览默认运行；每秒汇总，避免逐帧日志影响测试吞吐。
    const auto begin = std::chrono::steady_clock::now();
    auto last_report = begin;
    io::CameraStats previous;
    std::uint64_t received = 0, previous_received = 0;
    double host_age_ms = 0;
    while (!exiter.exit() && (seconds == 0 ||
           std::chrono::steady_clock::now() - begin < std::chrono::seconds(seconds))) {
      const auto frame = camera.read_frame();
      const auto now = std::chrono::steady_clock::now();
      if (frame) {
        if (frame->image.empty() || frame->image.type() != CV_8UC3)
          throw std::runtime_error("camera violated CV_8UC3 BGR contract");
        if (++received == 1)
          tools::logger()->info("first image: {}x{}, CV_8UC3 BGR", frame->image.cols, frame->image.rows);
        host_age_ms = std::chrono::duration<double, std::milli>(now - frame->timestamp).count();
        if (display) {
          cv::imshow("img", frame->image);
          if (cv::waitKey(1) == 'q') break;
        }
        if (slow_ms) std::this_thread::sleep_for(std::chrono::milliseconds(slow_ms));
      }
      // [9.9-5] FPS 窗口包含模拟处理/显示耗时；交付数直接取本程序实际读取量。
      const auto report_time = std::chrono::steady_clock::now();
      const auto elapsed = std::chrono::duration<double>(report_time - last_report).count();
      if (elapsed < 1.0) continue;
      const auto stats = camera.stats();
      tools::logger()->info(
        "capture={:.1f} FPS, deliver={:.1f} FPS, captured={}, delivered={}, consumer_skipped={}, "
        "pool_dropped={}, read_timeouts={}, capture_timeouts={}, reconnects={}, last_host_age={:.2f} ms",
        (stats.captured - previous.captured) / elapsed, (received - previous_received) / elapsed,
        stats.captured, received, stats.consumer_skipped, stats.pool_dropped, stats.read_timeouts,
        stats.capture_timeouts, stats.reconnects, host_age_ms);
      previous = stats;
      previous_received = received;
      last_report = report_time;
    }
    tools::logger()->info("received {} images; host age excludes exposure and SDK/USB buffering", received);
    return received ? 0 : 2;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
