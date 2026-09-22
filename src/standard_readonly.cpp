// [9.21-QDU-READONLY] Read-only, zero-control full-chain acceptance entry.
//
// Why this exists: the real QDU-Future board currently returns zero application bytes, so the SP25
// upper layers (Tracker, Solver, Aimer) have never executed against live attitude.  This entry runs
// the *entire* production chain — camera, Hailo detector, Solver, Tracker, Aimer, Shooter — against
// a QDU link that is physically incapable of transmitting, so the whole pipeline can be exercised
// and measured before a gimbal or launcher is ever connected.
//
// What it guarantees, by construction rather than by discipline:
//   * io::CBoard is built with force_read_only=true, which overrides the YAML tx_enabled flag before
//     the serial worker thread starts.  Every write path in the worker is gated on the resulting
//     config, including the reconnect-neutral and shutdown-neutral frames.
//   * This file never calls CBoard::send() or CBoard::send_target().  There is no call site to
//     un-comment: the would-be command is measured, reported, and discarded.
//   * The read-only gate is self-checked at startup.  If it does not take effect the entry refuses
//     to run rather than continuing in an unknown state.
//
// What it does NOT claim: nothing here proves real C-board RX/TX, and nothing here is auto-aim
// acceptance.  It proves the SP25 chain runs end to end on live attitude and reports what it would
// command.  Use qdu_board_simulator to supply attitude while the real board is silent, and treat
// every number this entry prints as simulated until a real board delivers quaternions.
#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <list>
#include <memory>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <string>
#include <vector>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tests/live_preview.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

using namespace std::chrono;

namespace
{
const std::string keys =
  "{help h usage ?     |      | print command line arguments}"
  "{seconds          | 0    | seconds to run; 0 runs until Ctrl+C}"
  "{qdu-device        |      | override qdu_communication.device, e.g. /dev/pts/3}"
  "{preview-port      | 0    | optional browser preview port; 0 disables the preview}"
  "{@config-path      | configs/pi_standard3_qdu.yaml | positional yaml config path}";

// [9.21-QDU-READONLY] Apply the --qdu-device override by emitting a merged config next to the
// original.  Only the QDU device is touched; every other key is copied verbatim so the chain under
// test sees exactly the production configuration.
std::string merged_config_path(const std::string & source, const std::string & device_override)
{
  if (device_override.empty()) return source;

  YAML::Node root = YAML::LoadFile(source);
  if (!root["qdu_communication"] || !root["qdu_communication"].IsMap())
    throw std::runtime_error("missing qdu_communication map in " + source);
  root["qdu_communication"]["device"] = device_override;

  // A per-process suffix keeps concurrent runs from sharing one temporary file.
  static unsigned counter = 0;
  const auto suffix = std::to_string(steady_clock::now().time_since_epoch().count()) + "-" +
                      std::to_string(++counter);
  auto path = std::filesystem::temp_directory_path() / ("sp25-readonly-" + suffix + ".yaml");

  std::ofstream out(path, std::ios::trunc);
  if (!out) throw std::runtime_error("cannot write merged config " + path.string());
  YAML::Emitter emitter;
  emitter << root;
  out << emitter.c_str() << '\n';
  out.flush();
  if (!out) throw std::runtime_error("failed writing merged config " + path.string());
  return path.string();
}

// [9.21-QDU-READONLY] What the chain decided on one frame.  Reporting only; nothing is transmitted.
struct FrameSummary
{
  std::uint64_t sequence{};
  double frame_age_ms{};
  double attitude_age_ms{};
  std::size_t detections{};
  std::size_t targets{};
  std::string tracker_state{"-"};
  std::string target_name{"-"};
  int target_priority{};
  double pnp_distance_m{};
  Eigen::Vector3d world_xyz{Eigen::Vector3d::Zero()};
  bool would_control{false};
  bool would_fire{false};
  double would_yaw{};
  double would_pitch{};
};

FrameSummary summarize(
  std::uint64_t sequence, double frame_age_ms, double attitude_age_ms,
  const std::list<auto_aim::Armor> & armors, const std::list<auto_aim::Target> & targets,
  const io::Command & command, auto_aim::Aimer & aimer, auto_aim::Shooter & shooter,
  const Eigen::Vector3d & gimbal_ypr, const std::string & tracker_state)
{
  FrameSummary summary;
  summary.sequence = sequence;
  summary.frame_age_ms = frame_age_ms;
  summary.attitude_age_ms = attitude_age_ms;
  summary.detections = armors.size();
  summary.targets = targets.size();
  summary.tracker_state = tracker_state;
  summary.would_control = command.control;
  summary.would_yaw = command.yaw;
  summary.would_pitch = command.pitch;
  // [9.21-QDU-READONLY] The Shooter is built from the production config, where auto_fire stays
  // false, so this reports the decision the chain would make and never authorizes anything.  The
  // live gimbal attitude is passed honestly: a fabricated value would make would_fire meaningless.
  summary.would_fire = shooter.shoot(command, aimer, targets, gimbal_ypr);

  if (!targets.empty()) {
    const auto & target = targets.front();
    summary.target_name = auto_aim::ARMOR_NAMES.at(target.name);
    summary.target_priority = static_cast<int>(target.priority);
    // The EKF state holds the tracked center at x[0], x[2], x[4]; see Target::h_armor_xyz.
    const auto x = target.ekf_x();
    if (x.size() >= 9) summary.world_xyz = {x[0], x[2], x[4]};
  }
  // The best available single-armor PnP range.  The Tracker keeps the armors it solved, so this is
  // a real measurement rather than a derived one; it is not the same number as the tracked range.
  for (const auto & armor : armors) {
    const double range = armor.xyz_in_gimbal.norm();
    if (range > 1e-6) {
      summary.pnp_distance_m = range;
      break;
    }
  }
  return summary;
}

std::string format_summary(const FrameSummary & summary, const io::QduLinkStats & stats)
{
  return fmt::format(
    "[standard_readonly] frame={} age_ms={:.2f} att_age_ms={:.2f} det={} targets={} "
    "tracker_state={} target={} prio={} pnp_m={:.2f} world=[{:.2f},{:.2f},{:.2f}] "
    "would_control={} would_fire={} would_yaw={:.5f} would_pitch={:.5f} "
    "serial_open={} rx_q={} tx={} (NOT TRANSMITTED)",
    summary.sequence, summary.frame_age_ms, summary.attitude_age_ms, summary.detections,
    summary.targets, summary.tracker_state, summary.target_name, summary.target_priority,
    summary.pnp_distance_m, summary.world_xyz.x(), summary.world_xyz.y(), summary.world_xyz.z(),
    summary.would_control, summary.would_fire, summary.would_yaw, summary.would_pitch,
    stats.serial_open, stats.received_quaternions, stats.transmitted_commands);
}

// [Pi预览-OVERLAY] Convert Tracker and Aimer output into the pixel overlay.  Reprojection uses the
// production Solver, so the drawn aim point is the same geometry the Solver solves with.
live_preview::Overlay build_overlay(
  auto_aim::Solver & solver, const auto_aim::Aimer & aimer,
  const std::list<auto_aim::Target> & targets, const std::string & tracker_state,
  const FrameSummary & summary)
{
  live_preview::Overlay overlay;

  if (!targets.empty()) {
    const auto & target = targets.front();
    for (const auto & xyza : target.armor_xyza_list()) {
      const auto corners =
        solver.reproject_armor(xyza.head<3>(), xyza.w(), target.armor_type, target.name);
      if (corners.size() == 4) overlay.target_quads.push_back(corners);
    }
  }

  // debug_aim_point carries the world-coordinate point the Aimer actually selected.
  if (aimer.debug_aim_point.valid) {
    const auto & point = aimer.debug_aim_point.xyza;
    const auto pixels = solver.world2pixel(
      {cv::Point3f(static_cast<float>(point.x()), static_cast<float>(point.y()),
                   static_cast<float>(point.z()))});
    if (!pixels.empty()) {
      overlay.aim_point = pixels.front();
      overlay.has_aim_point = true;
    }
  }

  overlay.status = fmt::format(
    "{} · {} · prio {} · would_yaw {:.3f} would_pitch {:.3f} · NOT TRANSMITTED", tracker_state,
    summary.target_name, summary.target_priority, summary.would_yaw, summary.would_pitch);
  return overlay;
}
}  // namespace

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  const int run_seconds = cli.get<int>("seconds");
  const auto qdu_device = cli.get<std::string>("qdu-device");
  const int preview_port = cli.get<int>("preview-port");
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }
  if (!cli.check() || run_seconds < 0 || preview_port < 0 || preview_port > 65535) {
    cli.printErrors();
    tools::logger()->error("standard_readonly: --seconds or --preview-port is out of range");
    return 2;
  }

  tools::Exiter exiter;

  std::string effective_config;
  try {
    effective_config = merged_config_path(config_path, qdu_device);
  } catch (const std::exception & error) {
    tools::logger()->error("standard_readonly: cannot prepare config: {}", error.what());
    return 2;
  }
  if (!qdu_device.empty())
    tools::logger()->info(
      "[standard_readonly] overriding qdu_communication.device with {} (effective config {})",
      qdu_device, effective_config);

  const auto log = tools::logger();
  log->info(
    "[standard_readonly] loading Hailo and SP25 auto-aim modules; the QDU link will be opened "
    "read-only and no command will ever be transmitted");

  auto_aim::YOLO detector(effective_config, false);
  auto_aim::Solver solver(effective_config);
  auto_aim::Tracker tracker(effective_config, solver);
  auto_aim::Aimer aimer(effective_config);
  auto_aim::Shooter shooter(effective_config);
  log->info("[standard_readonly] Hailo and auto-aim modules ready; starting HikRobot camera");

  // [9.21-QDU-READONLY] The entry must never be part of a firing decision, so surface the gate
  // explicitly.  The entry stays safe either way because the TX gate below is hard-enforced.
  try {
    const auto root = YAML::LoadFile(effective_config);
    if (root["auto_fire"] && root["auto_fire"].as<bool>())
      log->warn(
        "[standard_readonly] config has auto_fire: true; this entry still transmits nothing, but "
        "the acceptance configuration is not the read-only one");
  } catch (const std::exception & error) {
    log->warn("[standard_readonly] cannot read auto_fire from config: {}", error.what());
  }

  io::Camera camera(effective_config);
  io::CameraFrame startup_frame;
  while (!exiter.exit() && !startup_frame) startup_frame = camera.read_frame();
  if (exiter.exit()) return 0;
  log->info(
    "[standard_readonly] camera ready: {}x{}, sequence={}; opening QDU link read-only",
    startup_frame->image.cols, startup_frame->image.rows, startup_frame->sequence);
  startup_frame.reset();

  // [9.21-QDU-READONLY] force_read_only is the whole safety argument of this entry.  It is passed
  // once, here, and the link can no longer write no matter what the YAML asked for.
  io::CBoard cboard(effective_config, /*force_read_only=*/true);
  if (!cboard.read_only()) {
    log->error("standard_readonly: read-only gate did not take effect; refusing to run");
    return 3;
  }
  if (cboard.tx_enabled()) {
    log->error("standard_readonly: TX is still enabled after the read-only gate; refusing to run");
    return 3;
  }
  log->info(
    "[standard_readonly] QDU link read-only confirmed: TX is forced off, reconnect-neutral and "
    "shutdown-neutral frames are suppressed too");

  // [Pi预览-OVERLAY] The preview is optional and purely observational; it draws the production
  // detections plus the Tracker selection and the aim point, and carries no control decision.
  std::unique_ptr<live_preview::Server> preview;
  if (preview_port > 0) {
    try {
      preview = std::make_unique<live_preview::Server>(preview_port);
      log->info("[standard_readonly] browser preview on port {}", preview_port);
    } catch (const std::exception & error) {
      log->warn("[standard_readonly] preview disabled: {}", error.what());
    }
  }

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;
  auto mode = io::Mode::idle;
  auto last_mode = io::Mode::idle;
  auto last_report = steady_clock::now();
  auto next_overlay = steady_clock::now();

  const auto deadline = run_seconds == 0
                          ? steady_clock::time_point::max()
                          : steady_clock::now() + std::chrono::seconds(run_seconds);

  while (!exiter.exit() && steady_clock::now() < deadline) {
    const auto frame = camera.read_frame();
    if (!frame) {
      // No new frame means no control this round.  The production entry clears its command here; a
      // read-only entry has nothing to clear because it never transmitted anything.
      continue;
    }
    img = frame->image;
    t = frame->timestamp;

    if (!cboard.imu_fresh()) {
      // [9.21-QDU-READONLY] Publish the raw frame anyway.  With the real board currently silent this
      // is the state the entry spends most of its time in, and the operator still needs to see that
      // the camera is alive and delivering frames.
      if (preview) preview->publish(img, std::list<auto_aim::Armor>{}, frame->sequence);
      const auto now = steady_clock::now();
      if (now - last_report >= std::chrono::seconds(1)) {
        const auto stats = cboard.link_stats();
        log->warn(
          "[standard_readonly] waiting for fresh IMU: frame={}, serial_open={}, rx_q={}, "
          "att_age_ms={:.1f}",
          frame->sequence, stats.serial_open, stats.received_quaternions, cboard.imu_age_ms());
        last_report = now;
      }
      continue;
    }

    q = cboard.imu_at(t);
    mode = cboard.mode;
    if (last_mode != mode) {
      log->info("Switch to {}", io::MODES[mode]);
      last_mode = mode;
    }

    solver.set_R_gimbal2world(q);
    const Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    auto armors = detector.detect(img);
    auto targets = tracker.track(armors, t);
    const double pipeline_delay_s = tools::delta_time(steady_clock::now(), t);
    // [9.21-QDU-READONLY] This is exactly the command `standard` would send.  It is measured here
    // and then discarded; CBoard::send() is never called from this entry.
    const auto command = aimer.aim(targets, cboard.bullet_speed, pipeline_delay_s);
    const auto command_time = steady_clock::now();

    // [Pi预览-OVERLAY] Build the overlay at the preview's own rate so reprojection is only paid for
    // frames that can actually be published.
    if (preview) {
      const auto now = steady_clock::now();
      if (now >= next_overlay) {
        next_overlay = now + std::chrono::milliseconds(100);
        const auto summary = summarize(
          frame->sequence, tools::delta_time(command_time, t) * 1e3, cboard.imu_age_ms(), armors,
          targets, command, aimer, shooter, ypr, tracker.state());
        preview->publish(
          img, armors, frame->sequence,
          build_overlay(solver, aimer, targets, tracker.state(), summary));
      }
      preview->check();
    }

    if (command_time - last_report >= std::chrono::seconds(1)) {
      const auto stats = cboard.link_stats();
      const auto summary = summarize(
        frame->sequence, tools::delta_time(command_time, t) * 1e3, cboard.imu_age_ms(), armors,
        targets, command, aimer, shooter, ypr, tracker.state());
      log->info(
        "{} gimbal_ypr=[{:.3f},{:.3f},{:.3f}]", format_summary(summary, stats), ypr.x(), ypr.y(),
        ypr.z());
      last_report = command_time;
    }
  }

  const auto final_stats = cboard.link_stats();
  log->info(
    "[standard_readonly] end: rx_q={}, ahrs_q={}, gimbal_q={}, invalid_q={}, tx={}, opens={}, "
    "disconnects={}. No command was ever transmitted by this entry.",
    final_stats.received_quaternions, final_stats.received_ahrs_quaternions,
    final_stats.received_gimbal_quaternions, final_stats.invalid_quaternions,
    final_stats.transmitted_commands, final_stats.open_count, final_stats.disconnect_count);
  return 0;
}
