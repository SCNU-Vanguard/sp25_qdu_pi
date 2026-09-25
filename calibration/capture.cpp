#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <opencv2/opencv.hpp>

#include "calibration/circle_grid.hpp"
#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tests/live_preview.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
  "{help h usage ?  |                          | 输出命令行参数说明}"
  "{@config-path c  | configs/calibration.yaml | yaml配置文件路径 }"
  "{output-folder o |      assets/img_with_q   | 输出文件夹路径   }"
  "{board-config    |                          | 圆点板配置，默认使用相机配置 }"
  "{camera-only     | false                    | 只采内参照片，不打开C板、不保存四元数 }"
  "{preview-port    | 0                        | 网页端口；0使用原来的窗口显示 }";

void write_q(const std::string q_path, const Eigen::Quaterniond & q)
{
  std::ofstream q_file(q_path);
  Eigen::Vector4d xyzw = q.coeffs();
  // 输出顺序为wxyz
  q_file << fmt::format("{} {} {} {}", xyzw[3], xyzw[0], xyzw[1], xyzw[2]);
  q_file.close();
  if (!q_file) throw std::runtime_error("Cannot save quaternion: " + q_path);
}

// SSH 的终端保持正常行输入：输入 s 或 q 后按回车，无需后台读键线程。
int terminal_key()
{
  pollfd input{STDIN_FILENO, POLLIN, 0};
  if (::poll(&input, 1, 0) <= 0) return -1;
  char key;
  return ::read(STDIN_FILENO, &key, 1) == 1 ? key : 'q';
}

void capture_loop(
  const std::string & config_path, const std::string & output_folder,
  const cv::Size & pattern_size, bool camera_only, int preview_port)
{
  tools::Exiter exiter;
  std::unique_ptr<live_preview::Server> preview;
  if (preview_port > 0) {
    preview = std::make_unique<live_preview::Server>(
      preview_port, "圆点标定采图：在 SSH 输入 s 并回车保存，输入 q 并回车退出。"
                    "found=yes 表示识别到完整圆点阵列；saved 表示已保存照片数。");
    tools::logger()->info("Browser preview: http://<Pi-IP>:{}/", preview_port);
  }
  std::unique_ptr<io::CBoard> cboard;
  if (!camera_only) cboard = std::make_unique<io::CBoard>(config_path, true);
  io::Camera camera(config_path);

  int count = 0;
  auto next_wait_log = std::chrono::steady_clock::time_point{};
  while (!exiter.exit()) {
    int key = preview ? terminal_key() : -1;
    if (key == 'q') break;
    if (preview) preview->check();
    // 持有帧引用直到本轮保存/预览结束；暂时无帧时交给现有驱动继续采集和重连。
    const auto frame = camera.read_frame();
    if (!frame || frame->image.empty()) {
      const auto now = std::chrono::steady_clock::now();
      if (now >= next_wait_log) {
        tools::logger()->warn("[capture] 等待相机图像，继续重试；按 q 回车或 Ctrl+C 退出");
        next_wait_log = now + std::chrono::seconds(1);
      }
      if (key == 's') tools::logger()->warn("Not saved: 当前没有图像，请画面恢复后重新按 s");
      if (!preview && cv::waitKey(1) == 'q') break;
      continue;
    }
    const auto & img = frame->image;
    const auto timestamp = frame->timestamp;
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    if (cboard) q = cboard->imu_at(timestamp);

    // 在图像上显示欧拉角，用来判断imuabs系的xyz正方向，同时判断imu是否存在零漂
    auto img_with_ypr = img.clone();
    if (cboard) {
      Eigen::Vector3d zyx = tools::eulers(q, 2, 1, 0) * 57.3;  // degree
      tools::draw_text(img_with_ypr, fmt::format("Z {:.2f}", zyx[0]), {40, 40}, {0, 0, 255});
      tools::draw_text(img_with_ypr, fmt::format("Y {:.2f}", zyx[1]), {40, 80}, {0, 0, 255});
      tools::draw_text(img_with_ypr, fmt::format("X {:.2f}", zyx[2]), {40, 120}, {0, 0, 255});
    }

    std::vector<cv::Point2f> centers_2d;
    auto success = calibration::find_circle_grid(img, pattern_size, centers_2d);
    cv::drawChessboardCorners(img_with_ypr, pattern_size, centers_2d, success);  // 显示识别结果

    // 按“s”保存图片和对应四元数，按“q”退出程序
    if (preview) {
      live_preview::Overlay overlay;
      overlay.status = fmt::format(
        "{}x{} | found={} | saved={} | {}", img.cols, img.rows,
        success ? "yes" : "no", count, camera_only ? "camera only" : "image + IMU");
      preview->publish(img_with_ypr, {}, frame->sequence, overlay);
    } else {
      cv::resize(img_with_ypr, img_with_ypr, {}, 0.5, 0.5);
      cv::imshow("Press s to save, q to quit", img_with_ypr);
      key = cv::waitKey(1);
    }
    if (key == 'q')
      break;
    else if (key != 's')
      continue;

    if (camera_only && !success) {
      tools::logger()->warn("Not saved: 未检测到完整圆点阵列，请调整标定板后重新按 s");
      continue;
    }
    if (cboard && !cboard->imu_fresh()) {
      tools::logger()->warn("Not saved: IMU 数据过期，不能保存有效的图片/四元数组合");
      continue;
    }

    // 保存图片和四元数
    auto img_path = fmt::format("{}/{}.jpg", output_folder, count + 1);
    auto q_path = fmt::format("{}/{}.txt", output_folder, count + 1);
    if (!cv::imwrite(img_path, img)) throw std::runtime_error("Cannot save image: " + img_path);
    if (cboard) write_q(q_path, q);
    count++;
    tools::logger()->info("[{}] Saved in {}", count, output_folder);
  }

  // 离开该作用域时，camera和cboard会自动关闭
}

int main(int argc, char * argv[]) try
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);
  auto output_folder = cli.get<std::string>("output-folder");
  auto board_config = cli.get<std::string>("board-config");
  auto camera_only = cli.get<bool>("camera-only");
  auto preview_port = cli.get<int>("preview-port");
  if (!cli.check()) { cli.printErrors(); return 1; }
  if (preview_port < 0 || preview_port > 65535)
    throw std::runtime_error("preview-port must be 0..65535");
  auto board = YAML::LoadFile(board_config.empty() ? config_path : board_config);
  cv::Size pattern_size(board["pattern_cols"].as<int>(), board["pattern_rows"].as<int>());
  if (pattern_size.width < 2 || pattern_size.height < 2)
    throw std::runtime_error("Circle grid must have at least 2 rows and 2 columns");

  // 新建输出文件夹
  std::filesystem::create_directories(output_folder);
  if (!std::filesystem::is_empty(output_folder))
    throw std::runtime_error("Output folder must be empty to avoid overwriting photos: " + output_folder);

  tools::logger()->info("Symmetric circle grid: {} columns, {} rows; camera_only={}",
                        pattern_size.width, pattern_size.height, camera_only);
  // 主循环，保存图片和对应四元数
  capture_loop(config_path, output_folder, pattern_size, camera_only, preview_port);

  if (!camera_only) tools::logger()->warn("注意四元数输出顺序为wxyz");

  return 0;
}
catch (const std::exception & e) {
  fmt::print(stderr, "capture failed: {}\n", e.what());
  return 1;
}
