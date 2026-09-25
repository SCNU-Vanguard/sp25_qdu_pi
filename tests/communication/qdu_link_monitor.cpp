//不用启动相机和整套自瞄，就能单独检查树莓派与 C 板之间有没有数据
#include "io/cboard.hpp"

#include <Eigen/Geometry>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

namespace
{
struct Options
{
  std::string config_path;
  int seconds{30};
  bool expect_reconnect{false};
  bool send_command{false};
  float yaw{};
  float pitch{};
};

Options parse_options(int argc, char ** argv)
{
  if (argc < 2)
    throw std::runtime_error(
      "usage: qdu_link_monitor CONFIG [--seconds N] [--expect-reconnect] "
      "[--send-command YAW_RAD PITCH_RAD]");
  Options result;
  result.config_path = argv[1];
  for (int i = 2; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--seconds" && i + 1 < argc) {
      result.seconds = std::stoi(argv[++i]);
    } else if (arg == "--expect-reconnect") {
      result.expect_reconnect = true;
    } else if (arg == "--send-command" && i + 2 < argc) {
      result.send_command = true;
      result.yaw = std::stof(argv[++i]);
      result.pitch = std::stof(argv[++i]);
    } else {
      throw std::runtime_error("unknown argument: " + arg);
    }
  }
  if (result.seconds <= 0) throw std::runtime_error("--seconds must be positive");
  return result;
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    const auto options = parse_options(argc, argv);
    std::filesystem::create_directories("logs");
    io::CBoard cboard(options.config_path);
    // [9.21-QDU-NORMAL-TX] TX is only available to the byte-level automated regression when both
    // its temporary config and explicit nonzero command option agree.  Normal hardware uses
    // `standard`; this monitor remains RX-only without the option.
    if (cboard.tx_enabled() != options.send_command) {
      std::cerr << "TX safety mismatch between config and --send-command\n";
      return 4;
    }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(options.seconds);
    while (std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
      const auto now = std::chrono::steady_clock::now();
      if (options.send_command) {
        cboard.send(io::Command{true, false, options.yaw, options.pitch});
      }
      const auto stats = cboard.link_stats();
      Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
      if (cboard.imu_fresh()) q = cboard.imu_at(now);
      std::cout << std::fixed << std::setprecision(4)
                << "serial_open=" << stats.serial_open << " imu_fresh=" << stats.imu_fresh
                << " att_age_ms=" << cboard.imu_age_ms() << " opens=" << stats.open_count
                << " disconnects=" << stats.disconnect_count
                << " bytes=" << stats.received_bytes << " packets=" << stats.received_packets
                << " quaternions=" << stats.received_quaternions
                << " ahrs_q=" << stats.received_ahrs_quaternions
                << " gimbal_q=" << stats.received_gimbal_quaternions
                << " invalid_q=" << stats.invalid_quaternions
                << " unknown_topics=" << stats.unknown_topics
                << " crc_header=" << stats.parser.header_crc_errors
                << " crc_payload=" << stats.parser.payload_crc_errors << " q_wxyz=[" << q.w()
                << ',' << q.x() << ',' << q.y() << ',' << q.z() << "]\n";
    }

    const auto final_stats = cboard.link_stats();
    // [9.21-QDU-SIM] Report the whole receive path, not just the headline count, so an automated
    // acceptance run can assert that packets decoded cleanly instead of merely arriving.
    std::cout << "result opens=" << final_stats.open_count
              << " disconnects=" << final_stats.disconnect_count
              << " bytes=" << final_stats.received_bytes
              << " packets=" << final_stats.received_packets
              << " quaternions=" << final_stats.received_quaternions
              << " ahrs_q=" << final_stats.received_ahrs_quaternions
              << " gimbal_q=" << final_stats.received_gimbal_quaternions
              << " invalid_q=" << final_stats.invalid_quaternions
              << " unknown_topics=" << final_stats.unknown_topics
              << " crc_header=" << final_stats.parser.header_crc_errors
              << " crc_payload=" << final_stats.parser.payload_crc_errors
              << " tx_commands=" << final_stats.transmitted_commands << '\n';
    if (final_stats.received_quaternions == 0) return 2;
    if (!options.send_command && final_stats.transmitted_commands != 0) return 3;
    if (options.send_command && final_stats.transmitted_commands == 0) return 6;
    if (options.expect_reconnect &&
        (final_stats.open_count < 2 || final_stats.disconnect_count < 1))
      return 5;
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "qdu_link_monitor failed: " << error.what() << '\n';
    return 1;
  }
}
