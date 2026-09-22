// [9.21-QDU-SIM] QDU C-board emulator for read-only full-chain acceptance.
//
// The real QDU-Future board currently returns zero application bytes to the Pi, so Tracker,
// Solver and Aimer have never run against live attitude.  This tool replaces the board with a
// pseudo-terminal that speaks the exact libxr SharedTopic wire format, using the *production*
// codec in io/qdu_shared_topic_protocol.cpp so the simulated bytes cannot drift from the real
// protocol.
//
// It does two jobs at once:
//   1. publish ahrs_quaternion / gimbal_quat at a configurable rate, so the SP25 upper layers
//      receive fresh attitude;
//   2. decode whatever the SP25 side writes back (target_euler / fire_notify), which is how the
//      read-only entry proves it transmitted nothing at all, and how a TX-enabled run proves the
//      Aimer result reaches the wire in the right byte order.
//
// Nothing here commands hardware.  The emulator is a test instrument, not a substitute for real
// C-board acceptance.
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

#include "io/qdu_shared_topic_protocol.hpp"
#include "tools/logger.hpp"

namespace
{
using Clock = std::chrono::steady_clock;

struct Options
{
  std::string device_file;
  std::string quaternion_topic{"ahrs_quaternion"};
  std::string motion{"identity"};
  double rate_hz{100.0};
  double motion_rate{0.5};  // rad/s for sweep modes
  int seconds{0};
  double stall_after_s{-1.0};  // stop publishing attitude, keep the port open
  bool expect_silent{false};   // assert the SP25 side writes nothing at all
};

enum class ExitCode
{
  kOk = 0,
  kUsage = 1,
  kFireObserved = 2,
  kUnexpectedRx = 3,
  kNoConsumer = 4,
};

Options parse_options(int argc, char ** argv)
{
  Options result;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto need_value = [&](const char * name) -> std::string {
      if (i + 1 >= argc) throw std::runtime_error(std::string(name) + " requires a value");
      return argv[++i];
    };
    if (arg == "--device-file") {
      result.device_file = need_value("--device-file");
    } else if (arg == "--quaternion-topic") {
      const auto value = need_value("--quaternion-topic");
      if (value != "ahrs_quaternion" && value != "gimbal_quat" && value != "both")
        throw std::runtime_error("--quaternion-topic must be ahrs_quaternion|gimbal_quat|both");
      result.quaternion_topic = value;
    } else if (arg == "--motion") {
      const auto value = need_value("--motion");
      if (value != "identity" && value != "yaw-sweep" && value != "pitch-sweep" &&
          value != "roll-sweep" && value != "tumble")
        throw std::runtime_error("--motion must be identity|yaw-sweep|pitch-sweep|roll-sweep|tumble");
      result.motion = value;
    } else if (arg == "--motion-rate") {
      result.motion_rate = std::stod(need_value("--motion-rate"));
    } else if (arg == "--rate") {
      result.rate_hz = std::stod(need_value("--rate"));
    } else if (arg == "--seconds") {
      result.seconds = std::stoi(need_value("--seconds"));
    } else if (arg == "--stall-after") {
      result.stall_after_s = std::stod(need_value("--stall-after"));
    } else if (arg == "--expect-silent") {
      result.expect_silent = true;
    } else if (arg == "-h" || arg == "--help") {
      throw std::runtime_error("usage");
    } else {
      throw std::runtime_error("unknown argument: " + arg);
    }
  }
  if (result.rate_hz <= 0.0 || result.rate_hz > 1000.0)
    throw std::runtime_error("--rate must be in (0, 1000] Hz");
  if (!(result.motion_rate > 0.0) || !std::isfinite(result.motion_rate))
    throw std::runtime_error("--motion-rate must be positive and finite");
  if (result.seconds < 0) throw std::runtime_error("--seconds must be non-negative");
  return result;
}

// [9.21-QDU-SIM] Only the emulator's own pose model; the SP25 side consumes it as real attitude.
Eigen::Quaterniond make_quaternion(const Options & options, double elapsed_s)
{
  if (options.motion == "identity") return Eigen::Quaterniond::Identity();
  const double angle = options.motion_rate * elapsed_s;
  if (options.motion == "yaw-sweep")
    return Eigen::Quaterniond(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()));
  if (options.motion == "pitch-sweep")
    return Eigen::Quaterniond(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY()));
  if (options.motion == "roll-sweep")
    return Eigen::Quaterniond(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX()));
  // tumble: three incommensurate rates so no single-axis assumption can pass by accident.
  const Eigen::Quaterniond yaw(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()));
  const Eigen::Quaterniond pitch(Eigen::AngleAxisd(angle * 0.37, Eigen::Vector3d::UnitY()));
  const Eigen::Quaterniond roll(Eigen::AngleAxisd(angle * 0.61, Eigen::Vector3d::UnitX()));
  return yaw * pitch * roll;
}

// [9.21-QDU-SIM] libxr writes a 48-bit microsecond timestamp; reuse the production encoder.
std::uint64_t steady_us_48()
{
  const auto value =
    std::chrono::duration_cast<std::chrono::microseconds>(Clock::now().time_since_epoch()).count();
  return static_cast<std::uint64_t>(value) & 0x0000FFFFFFFFFFFFULL;
}

std::vector<std::uint8_t> quaternion_payload(const Eigen::Quaterniond & q)
{
  // [9.21-QDU] LibXR Quaternion inherits Eigen; the raw ABI on the wire is x, y, z, w.
  std::vector<std::uint8_t> payload(sizeof(float) * 4U);
  const float values[4] = {
    static_cast<float>(q.x()), static_cast<float>(q.y()), static_cast<float>(q.z()),
    static_cast<float>(q.w())};
  for (int i = 0; i < 4; ++i)
    std::memcpy(payload.data() + i * sizeof(float), &values[i], sizeof(float));
  return payload;
}

// [9.21-QDU-SIM] A fresh pty has ECHO/ICANON/OPOST on, so bytes written to the master are echoed
// back into the master's read queue and would be miscounted as C-board traffic.  cfmakeraw on the
// master fd configures the shared line discipline (verified on Linux: ECHO/ICANON/OPOST all clear
// and no echo returns), which also matches how the SP25 serial layer configures the slave.
void set_raw_mode(int master_fd)
{
  struct termios settings{};
  if (::tcgetattr(master_fd, &settings) != 0)
    throw std::system_error(errno, std::generic_category(), "tcgetattr(pty master)");
  ::cfmakeraw(&settings);
  if (::tcsetattr(master_fd, TCSANOW, &settings) != 0)
    throw std::system_error(errno, std::generic_category(), "tcsetattr(pty master)");
}

// [9.21-QDU-SIM] A ptmx write succeeds even before any consumer opens the slave, so write success
// is not an attachment signal, and the kernel pty input queue is finite (~20 kB).  Once it fills a
// write returns a SHORT count rather than EAGAIN, and committing that short write would leave a
// truncated frame in the queue and desynchronize the consumer's parser.  So every frame is gated
// on POLLOUT first and only committed once all of its bytes fit; a frame that cannot be committed
// in full within the timeout is dropped whole and the next one is attempted.  Dropping is safe:
// the attitude stream is periodic, so the consumer simply sees one fewer sample.
bool write_packet(int master_fd, const std::vector<std::uint8_t> & packet, int timeout_ms)
{
  const auto deadline = Clock::now() + std::chrono::milliseconds(timeout_ms);
  std::size_t offset = 0;
  while (offset < packet.size()) {
    const auto remaining = deadline - Clock::now();
    if (remaining <= Clock::duration::zero()) return false;
    pollfd item{master_fd, POLLOUT, 0};
    const int ready = ::poll(
      &item, 1,
      static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(remaining).count()));
    if (ready < 0) {
      if (errno == EINTR) continue;
      throw std::system_error(errno, std::generic_category(), "poll(pty master, POLLOUT)");
    }
    if (ready == 0) continue;

    const ssize_t written = ::write(master_fd, packet.data() + offset, packet.size() - offset);
    if (written > 0) {
      offset += static_cast<std::size_t>(written);
      continue;
    }
    if (written == 0) continue;  // No progress; the deadline above bounds the retry.
    if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) continue;
    // EIO/ENXIO only mean the consumer closed its side; keep emulating a board that stays present.
    if (errno == EIO || errno == ENXIO) return false;
    throw std::system_error(errno, std::generic_category(), "write(pty master)");
  }
  return true;
}

struct RxCounters
{
  std::atomic<std::uint64_t> bytes{0};
  std::atomic<std::uint64_t> target_euler{0};
  std::atomic<std::uint64_t> fire_notify{0};
  std::atomic<std::uint64_t> fire_true{0};
  std::atomic<std::uint64_t> other_topics{0};
  std::atomic<std::uint64_t> bad_size{0};
};

float read_float_le(const std::uint8_t * data)
{
  float value{};
  std::memcpy(&value, data, sizeof(value));
  return value;
}

void handle_rx_packet(
  const io::qdu::Packet & packet, RxCounters & counters, const std::uint32_t target_euler_key,
  const std::uint32_t fire_notify_key, bool verbose)
{
  if (packet.topic_crc32 == fire_notify_key) {
    ++counters.fire_notify;
    if (packet.payload.size() != 1) {
      ++counters.bad_size;
      return;
    }
    const bool fire = packet.payload[0] != 0;
    if (fire) ++counters.fire_true;
    if (verbose)
      std::cout << "[sim/rx] fire_notify=" << (fire ? "TRUE" : "false") << std::endl;
    return;
  }
  if (packet.topic_crc32 == target_euler_key) {
    ++counters.target_euler;
    if (packet.payload.size() != sizeof(float) * 9U) {
      ++counters.bad_size;
      return;
    }
    // [9.21-QDU] target_euler order is rol, pit, yaw, rol_dot, pit_dot, yaw_dot, ...
    // SP25 fills rol and pit with the mechanical pitch axis, so yaw is the third float.
    const float yaw = read_float_le(packet.payload.data() + 8);
    const float pitch = read_float_le(packet.payload.data() + 4);
    if (verbose)
      std::cout << "[sim/rx] target_euler yaw_rad=" << yaw << " pitch_rad=" << pitch << std::endl;
    return;
  }
  ++counters.other_topics;
}
}  // namespace

int main(int argc, char ** argv)
{
  Options options;
  try {
    options = parse_options(argc, argv);
  } catch (const std::exception & error) {
    if (std::string(error.what()) != "usage")
      std::cerr << "qdu_board_simulator: " << error.what() << '\n';
    std::cerr << "usage: qdu_board_simulator [--device-file PATH] [--seconds N]\n"
                 "                              [--rate HZ] [--quaternion-topic NAME|both]\n"
                 "                              [--motion identity|yaw-sweep|pitch-sweep|roll-sweep|"
                 "tumble]\n"
                 "                              [--motion-rate RAD_S] [--stall-after SEC]\n"
                 "                              [--expect-silent]\n";
    return static_cast<int>(ExitCode::kUsage);
  }

  try {
    // tools::logger() writes logs/<timestamp>.log on first use.
    std::filesystem::create_directories("logs");
  } catch (const std::exception & error) {
    std::cerr << "qdu_board_simulator: cannot create logs directory: " << error.what() << '\n';
    return static_cast<int>(ExitCode::kUsage);
  }

  // [9.21-QDU-SIM] Create the pseudo-terminal that stands in for DevC-USB.
  const int master_fd = ::posix_openpt(O_RDWR | O_NOCTTY);
  if (master_fd < 0) throw std::system_error(errno, std::generic_category(), "posix_openpt");
  if (::grantpt(master_fd) != 0 || ::unlockpt(master_fd) != 0)
    throw std::system_error(errno, std::generic_category(), "grantpt/unlockpt");
  char slave_name[256] = {0};
  if (::ptsname_r(master_fd, slave_name, sizeof(slave_name)) != 0)
    throw std::system_error(errno, std::generic_category(), "ptsname_r");
  const std::string device = slave_name;

  struct FdGuard
  {
    int fd;
    ~FdGuard()
    {
      if (fd >= 0) ::close(fd);
    }
  } master_guard{master_fd};

  // [9.21-QDU-SIM] Disable terminal echo before publishing, and make the master non-blocking so a
  // full pty queue cannot stall the publish loop while the consumer is still starting up.
  set_raw_mode(master_fd);
  const int flags = ::fcntl(master_fd, F_GETFL, 0);
  if (flags < 0 || ::fcntl(master_fd, F_SETFL, flags | O_NONBLOCK) < 0)
    throw std::system_error(errno, std::generic_category(), "fcntl(pty master, O_NONBLOCK)");

  if (!options.device_file.empty()) {
    std::ofstream out(options.device_file, std::ios::trunc);
    if (!out) throw std::runtime_error("cannot write --device-file " + options.device_file);
    out << device << '\n';
  }

  // [9.21-QDU-SIM] Print the pseudo-terminal path before anything else: a launch script needs it
  // on the first stdout line, and the shared logger also writes to stdout.
  std::cout << "simulator_device=" << device << std::endl;
  tools::logger()->info(
    "[sim] QDU emulator on {} rate={:.1f}Hz quaternion_topic={} motion={} expect_silent={}",
    device, options.rate_hz, options.quaternion_topic, options.motion, options.expect_silent);
  std::cout << "point qdu_communication.device at the path above, then start the SP25 entry"
            << std::endl;

  const bool send_ahrs = options.quaternion_topic == "ahrs_quaternion" || options.quaternion_topic == "both";
  const bool send_gimbal = options.quaternion_topic == "gimbal_quat" || options.quaternion_topic == "both";
  const std::uint32_t target_euler_key = io::qdu::topic_crc32("target_euler");
  const std::uint32_t fire_notify_key = io::qdu::topic_crc32("fire_notify");
  io::qdu::Parser parser;
  RxCounters counters;

  const auto start = Clock::now();
  const auto deadline = options.seconds == 0 ? Clock::time_point::max()
                                            : start + std::chrono::seconds(options.seconds);
  const auto period = std::chrono::duration_cast<Clock::duration>(
    std::chrono::duration<double>(1.0 / options.rate_hz));
  // [9.21-QDU-SIM] A frame that cannot be committed in full within a fraction of the publish period
  // is dropped whole, so a slow or absent consumer can never stall the emulator.
  const int write_timeout_ms = std::max(
    1, std::min(5, static_cast<int>(
                      std::chrono::duration_cast<std::chrono::milliseconds>(period).count()) /
                      4));
  auto next_tick = Clock::now();
  auto next_report = start + std::chrono::seconds(1);
  std::uint64_t sent_quaternions = 0;
  std::uint64_t dropped_frames = 0;
  bool warned_no_consumer = false;
  bool stalled = false;

  std::vector<std::uint8_t> read_buffer(4096);
  while (Clock::now() < deadline) {
    const double elapsed = std::chrono::duration<double>(Clock::now() - start).count();

    // [9.21-QDU-SIM] Publishing keeps running at full rate; only the attitude stream stops, so
    // the consumer's stale gate is exercised without closing the serial device.
    if (!stalled && options.stall_after_s >= 0.0 && elapsed >= options.stall_after_s) {
      stalled = true;
      tools::logger()->warn("[sim] attitude stream stalled after {:.3f}s", elapsed);
      std::cout << "simulator_stalled=1" << std::endl;
    }

    if (!stalled) {
      const auto payload = quaternion_payload(make_quaternion(options, elapsed));
      // [9.21-QDU-SIM] Drop the frame whole rather than commit a partial one; a dropped attitude
      // sample is invisible to the consumer, a truncated frame would break its parser.
      const auto publish = [&](const std::uint32_t topic_key) {
        const auto packet =
          io::qdu::pack(topic_key, steady_us_48(), payload.data(), payload.size());
        if (write_packet(master_fd, packet, write_timeout_ms)) {
          ++sent_quaternions;
          return;
        }
        ++dropped_frames;
        if (!warned_no_consumer) {
          warned_no_consumer = true;
          tools::logger()->warn(
            "[sim] attitude frames are being dropped: the pty queue is full, so nothing is "
            "draining {} yet",
            device);
        }
      };
      if (send_ahrs) publish(io::qdu::topic_crc32("ahrs_quaternion"));
      if (send_gimbal) publish(io::qdu::topic_crc32("gimbal_quat"));
    }

    // [9.21-QDU-SIM] Drain whatever the SP25 entry wrote back into the same UART.
    pollfd item{master_fd, POLLIN, 0};
    const int ready = ::poll(&item, 1, 0);
    if (ready > 0 && (item.revents & POLLIN) != 0) {
      const ssize_t count = ::read(master_fd, read_buffer.data(), read_buffer.size());
      if (count > 0) {
        counters.bytes += static_cast<std::uint64_t>(count);
        parser.feed(read_buffer.data(), static_cast<std::size_t>(count), [&](const io::qdu::Packet &
                                                                             packet) {
          handle_rx_packet(packet, counters, target_euler_key, fire_notify_key, false);
        });
      } else if (count < 0 && errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK) {
        // EIO here only means the consumer closed its side; keep emulating a board reboot.
        tools::logger()->info("[sim] consumer side closed (errno={})", errno);
      }
    }

    const auto now = Clock::now();
    if (now >= next_report) {
      next_report = now + std::chrono::seconds(1);
      std::cout << "[sim] t=" << std::fixed << std::setprecision(1)
                << std::chrono::duration<double>(now - start).count() << "s sent_q="
                << sent_quaternions << " dropped=" << dropped_frames << " rx_bytes="
                << counters.bytes.load() << " rx_target=" << counters.target_euler.load()
                << " rx_fire=" << counters.fire_notify.load()
                << " rx_other=" << counters.other_topics.load() << std::endl;
    }

    next_tick += period;
    const auto remaining = next_tick - Clock::now();
    if (remaining > Clock::duration::zero()) std::this_thread::sleep_for(remaining);
    else next_tick = Clock::now();  // Fallen behind; resynchronize instead of bursting.
  }

  const auto parser_stats = parser.stats();
  std::cout << "result sent_quaternions=" << sent_quaternions
            << " rx_bytes=" << counters.bytes.load()
            << " rx_target_euler=" << counters.target_euler.load()
            << " rx_fire_notify=" << counters.fire_notify.load()
            << " rx_fire_true=" << counters.fire_true.load()
            << " rx_other_topics=" << counters.other_topics.load()
            << " rx_bad_size=" << counters.bad_size.load()
            << " accepted=" << parser_stats.accepted_packets
            << " discarded=" << parser_stats.discarded_bytes
            << " crc_header=" << parser_stats.header_crc_errors
            << " crc_payload=" << parser_stats.payload_crc_errors
            << " dropped=" << dropped_frames << '\n';

  // [9.21-QDU-SIM] fire_notify=true would mean a read-only entry authorized firing.  Treat it as a
  // hard failure even though this emulator has no actuator attached.
  if (counters.fire_true.load() != 0) {
    std::cerr << "qdu_board_simulator: SAFETY FAILURE, fire_notify was true "
              << counters.fire_true.load() << " times\n";
    return static_cast<int>(ExitCode::kFireObserved);
  }
  if (options.expect_silent && counters.bytes.load() != 0) {
    std::cerr << "qdu_board_simulator: expected a silent read-only consumer but received "
              << counters.bytes.load() << " bytes\n";
    return static_cast<int>(ExitCode::kUnexpectedRx);
  }
  return static_cast<int>(ExitCode::kOk);
}
