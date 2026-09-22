#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/dm_imu/dm_imu.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/auto_buff/buff_aimer.hpp"
#include "tasks/auto_buff/buff_detector.hpp"
#include "tasks/auto_buff/buff_solver.hpp"
#include "tasks/auto_buff/buff_target.hpp"
#include "tasks/auto_buff/buff_type.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

const std::string keys =
  "{help h usage ? |                  | 输出命令行参数说明}"
  "{@config-path   | configs/uav.yaml | yaml配置文件路径 }";

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;

  io::Camera camera(config_path);
  io::CBoard cboard(config_path);

  // [9.9-1] 统一到保留的检测入口，移除旧 Detector；待 Hailo 接入后恢复构建。
  auto_aim::YOLO detector(config_path);
  auto_aim::Solver solver(config_path);
  // auto_aim::YOLO yolo(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  auto_buff::Buff_Detector buff_detector(config_path);
  auto_buff::Solver buff_solver(config_path);
  auto_buff::SmallTarget buff_small_target;
  auto_buff::BigTarget buff_big_target;
  auto_buff::Aimer buff_aimer(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  auto mode = io::Mode::idle;
  auto last_mode = io::Mode::idle;

  auto last_age_log = std::chrono::steady_clock::now();  // [9.9-10] 仅每秒报告主机帧龄。
  while (!exiter.exit()) {
    // [9.9-5] 本轮持有槽位引用，下一轮只取新 sequence；无新帧时停止控制。
    const auto frame = camera.read_frame();
    if (!frame) {
      cboard.send(io::Command{});
      continue;
    }
    img = frame->image;
    t = frame->timestamp;
    // [9.9-10] 主机收帧参考；去掉未经此相机验证的 1 ms 偏移。
    q = cboard.imu_at(t);
    mode = cboard.mode;
    // recorder.record(img, q, t);
    if (last_mode != mode) {
      tools::logger()->info("Switch to {}", io::MODES[mode]);
      last_mode = mode;
    }

    /// 自瞄
    if (mode == io::Mode::auto_aim || mode == io::Mode::outpost) {
      solver.set_R_gimbal2world(q);

      Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

      auto armors = detector.detect(img);

      auto targets = tracker.track(armors, t);
      // [9.9-10] 相机输出为本机 steady_clock 收帧时刻，显式传入处理延迟（秒）。
      const double pipeline_delay_s = tools::delta_time(std::chrono::steady_clock::now(), t);

      auto command = aimer.aim(targets, cboard.bullet_speed, pipeline_delay_s);
      // [9.9-10] 包含 Aimer 本次计算耗时；不是曝光到控制器执行的端到端延迟。
      const auto command_time = std::chrono::steady_clock::now();
      if (command_time - last_age_log >= std::chrono::seconds(1)) {
        tools::logger()->info("[Aimer] host_frame_age_ms={:.2f}", tools::delta_time(command_time, t) * 1e3);
        last_age_log = command_time;
      }

      command.shoot = shooter.shoot(command, aimer, targets, ypr);

      cboard.send(command);
    }

    /// 打符
    else if (mode == io::Mode::small_buff || mode == io::Mode::big_buff) {
      buff_solver.set_R_gimbal2world(q);

      // [9.9-5] 原打符检测含原地调试绘图，不能修改池中只读图像。
      auto buff_image = img.clone();
      auto power_runes = buff_detector.detect(buff_image);

      buff_solver.solve(power_runes);

      io::Command buff_command;
      if (mode == io::Mode::small_buff) {
        buff_small_target.get_target(power_runes, t);
        auto target_copy = buff_small_target;
        buff_command = buff_aimer.aim(target_copy, t, cboard.bullet_speed, true);
      } else if (mode == io::Mode::big_buff) {
        buff_big_target.get_target(power_runes, t);
        auto target_copy = buff_big_target;
        buff_command = buff_aimer.aim(target_copy, t, cboard.bullet_speed, true);
      }
      cboard.send(buff_command);
    }

    else
      continue;
  }

  return 0;
}
