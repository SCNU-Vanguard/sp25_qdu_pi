#include <fmt/core.h>

#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tasks/auto_aim/aimer.hpp"
// [9.9-1] 移除未使用的旧推理链头文件，保留原主循环。
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

using namespace std::chrono;

const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{seconds          | 0    | 自动结束秒数；0表示持续运行到Ctrl+C }"
  "{@config-path   | configs/standard3.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  const int run_seconds = cli.get<int>("seconds");
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }
  if (!cli.check() || run_seconds < 0) {
    cli.printErrors();
    tools::logger()->error("standard: --seconds must be non-negative");
    return 2;
  }

  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;

  // [9.21-PI-STARTUP] Match the already validated hailo_camera_test order: initialize logging and
  // finish Hailo/model construction before the HikRobot SDK starts USB enumeration.  Previously
  // standard started QDU + camera worker threads while Hailo was loading, and HikRobot repeatedly
  // returned MV_E_RESOURCE (0x80000006) even though the isolated camera test worked.
  const auto log = tools::logger();
  log->info("[standard] loading Hailo and SP25 auto-aim modules before camera startup");
  auto_aim::YOLO detector(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);
  log->info("[standard] Hailo and auto-aim modules ready; starting HikRobot camera");

  io::Camera camera(config_path);
  io::CameraFrame startup_frame;
  while (!exiter.exit() && !startup_frame) startup_frame = camera.read_frame();
  if (exiter.exit()) return 0;
  log->info(
    "[standard] camera ready: {}x{}, sequence={}; starting QDU link",
    startup_frame->image.cols, startup_frame->image.rows, startup_frame->sequence);
  startup_frame.reset();

  // [9.21-PI-STARTUP] Open QDU only after the camera owns its SDK resources.  Runtime RX/TX still
  // runs concurrently with capture and inference after this deterministic startup gate.
  io::CBoard cboard(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  auto mode = io::Mode::idle;
  auto last_mode = io::Mode::idle;

  auto last_age_log = std::chrono::steady_clock::now();  // [9.9-10] 仅每秒报告主机帧龄。
  // [9.21-QDU-NORMAL-TX] A bounded integration run exits through normal destructors, allowing the
  // QDU worker to disarm and clear its target before closing the serial port.
  const auto deadline = run_seconds == 0
                          ? std::chrono::steady_clock::time_point::max()
                          : std::chrono::steady_clock::now() + std::chrono::seconds(run_seconds);
  while (!exiter.exit() && std::chrono::steady_clock::now() < deadline) {
    // [9.9-5] 本轮持有槽位引用，下一轮只取新 sequence；无新帧时停止控制。
    const auto frame = camera.read_frame();
    if (!frame) {
      cboard.send(io::Command{});
      continue;
    }
    img = frame->image;
    t = frame->timestamp;
    // [9.21-QDU-NORMAL-TX] A live quaternion is required by the production Solver.  The QDU
    // backend will also inhibit TX, but skipping here prevents stale attitude from updating the
    // Tracker before the serial link has recovered.
    if (!cboard.imu_fresh()) {
      cboard.send(io::Command{});
      const auto now = std::chrono::steady_clock::now();
      if (now - last_age_log >= std::chrono::seconds(1)) {
        const auto stats = cboard.link_stats();
        tools::logger()->warn(
          "[standard/QDU] waiting for fresh IMU: frame={}, serial_open={}, rx_q={}, tx={}, "
          "suppressed={}",
          frame->sequence, stats.serial_open, stats.received_quaternions,
          stats.transmitted_commands, stats.suppressed_commands);
        last_age_log = now;
      }
      continue;
    }
    // [9.9-10] 主机收帧参考；去掉未经此相机验证的 1 ms 偏移。
    q = cboard.imu_at(t);
    mode = cboard.mode;

    if (last_mode != mode) {
      tools::logger()->info("Switch to {}", io::MODES[mode]);
      last_mode = mode;
    }

    // recorder.record(img, q, t);

    solver.set_R_gimbal2world(q);

    Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    auto armors = detector.detect(img);

    auto targets = tracker.track(armors, t);
    // [9.9-10] 相机输出为本机 steady_clock 收帧时刻，显式传入处理延迟（秒）。
    const double pipeline_delay_s = tools::delta_time(std::chrono::steady_clock::now(), t);

    auto command = aimer.aim(targets, cboard.bullet_speed, pipeline_delay_s);
    // [9.9-10] 包含 Aimer 本次计算耗时；不是曝光到控制器执行的端到端延迟。
    const auto command_time = std::chrono::steady_clock::now();
    cboard.send(command);
    if (command_time - last_age_log >= std::chrono::seconds(1)) {
      // [9.21-QDU-NORMAL-TX] Report the real production command and both RX/TX counters.  This is
      // observability only; target selection and Aimer output remain the SP25 implementation.
      const auto stats = cboard.link_stats();
      tools::logger()->info(
        "[standard/QDU] frame={} age_ms={:.2f} detections={} targets={} control={} fire={} "
        "yaw_rad={:.5f} pitch_rad={:.5f} serial_open={} imu_fresh={} rx_q={} tx={} suppressed={}",
        frame->sequence, tools::delta_time(command_time, t) * 1e3, armors.size(), targets.size(),
        command.control, command.shoot, command.yaw, command.pitch, stats.serial_open,
        stats.imu_fresh, stats.received_quaternions, stats.transmitted_commands,
        stats.suppressed_commands);
      last_age_log = command_time;
    }
  }

  const auto final_stats = cboard.link_stats();
  tools::logger()->info(
    "[standard/QDU] end: rx_q={}, tx={}, suppressed={}, opens={}, disconnects={}",
    final_stats.received_quaternions, final_stats.transmitted_commands,
    final_stats.suppressed_commands, final_stats.open_count, final_stats.disconnect_count);
  return 0;
}
