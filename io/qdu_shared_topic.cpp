#include "qdu_shared_topic.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>

#include "serial/serial.h"
#include "tools/logger.hpp"

namespace io
{
namespace
{
constexpr std::size_t kQuaternionPayloadSize = sizeof(float) * 4U;
constexpr std::size_t kTargetPayloadSize = sizeof(float) * 9U;
constexpr std::size_t kMaxImuSamples = 1024;

int64_t steady_ns(std::chrono::steady_clock::time_point timestamp)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp.time_since_epoch()).count();
}

uint64_t steady_us_48()
{
  const auto value = std::chrono::duration_cast<std::chrono::microseconds>(
                       std::chrono::steady_clock::now().time_since_epoch())
                       .count();
  return static_cast<uint64_t>(value) & 0x0000FFFFFFFFFFFFULL;
}

float read_float_le(const uint8_t * data)
{
  static_assert(sizeof(float) == 4, "QDU SharedTopic requires 32-bit float");
  static_assert(std::numeric_limits<float>::is_iec559, "QDU SharedTopic requires IEEE-754 float");
  uint32_t bits = static_cast<uint32_t>(data[0]) | (static_cast<uint32_t>(data[1]) << 8U) |
                  (static_cast<uint32_t>(data[2]) << 16U) |
                  (static_cast<uint32_t>(data[3]) << 24U);
  float result{};
  std::memcpy(&result, &bits, sizeof(result));
  return result;
}

std::string yaml_string(const YAML::Node & node, const char * key, const std::string & fallback)
{
  return node[key] ? node[key].as<std::string>() : fallback;
}

template<typename T>
T yaml_value(const YAML::Node & node, const char * key, const T & fallback)
{
  return node[key] ? node[key].as<T>() : fallback;
}
}  // namespace

QduSharedTopic::Config QduSharedTopic::read_config(const std::string & path)
{
  const auto root = YAML::LoadFile(path);
  const auto node = root["qdu_communication"];
  if (!node || !node.IsMap())
    throw std::runtime_error("missing qdu_communication map in " + path);

  Config result;
  result.device = yaml_string(node, "device", "");
  if (result.device.empty()) throw std::runtime_error("qdu_communication.device is empty");
  result.baud_rate = yaml_value<uint32_t>(node, "baud_rate", result.baud_rate);
  result.tx_enabled = yaml_value<bool>(node, "tx_enabled", result.tx_enabled);
  result.reconnect_interval_ms =
    yaml_value<int>(node, "reconnect_interval_ms", result.reconnect_interval_ms);
  result.read_timeout_ms = yaml_value<int>(node, "read_timeout_ms", result.read_timeout_ms);
  result.stale_timeout_ms = yaml_value<int>(node, "stale_timeout_ms", result.stale_timeout_ms);
  result.default_bullet_speed =
    yaml_value<double>(node, "default_bullet_speed", result.default_bullet_speed);
  result.default_mode = yaml_string(node, "default_mode", result.default_mode);
  if (node["quaternion_topics"]) {
    result.quaternion_topics = node["quaternion_topics"].as<std::vector<std::string>>();
  }

  if (result.baud_rate != 115200U)
    throw std::runtime_error("QDU DevC-USB protocol requires baud_rate=115200");
  if (result.reconnect_interval_ms < 10 || result.read_timeout_ms < 1 ||
      result.stale_timeout_ms < result.read_timeout_ms)
    throw std::runtime_error("invalid qdu_communication timeout configuration");
  if (result.default_bullet_speed <= 0.0 || !std::isfinite(result.default_bullet_speed))
    throw std::runtime_error("qdu_communication.default_bullet_speed must be positive");
  if (result.quaternion_topics.empty())
    throw std::runtime_error("qdu_communication.quaternion_topics is empty");
  return result;
}

QduSharedTopic::QduSharedTopic(const std::string & config_path, bool force_read_only)
: config_(read_config(config_path)), force_read_only_(force_read_only)
{
  // [9.21-QDU] QDU libxr hashes the topic name only; the "host" domain is not on the wire.
  for (const auto & name : config_.quaternion_topics)
    quaternion_topic_keys_.push_back(qdu::topic_crc32(name));
  target_euler_key_ = qdu::topic_crc32("target_euler");
  fire_notify_key_ = qdu::topic_crc32("fire_notify");

  // [9.21-QDU-READONLY] Override the YAML before the worker starts so no write path can be taken,
  // including the reconnect-neutral and shutdown-neutral frames.
  if (force_read_only_) config_.tx_enabled = false;

  tools::logger()->info(
    "[QDU] device={}, 115200 8N1, tx={}, stale_timeout={} ms{}", config_.device,
    config_.tx_enabled ? "enabled" : "disabled", config_.stale_timeout_ms,
    force_read_only_ ? " (forced read-only: no bytes will be written)" : "");
  worker_ = std::thread(&QduSharedTopic::worker_loop, this);
}

QduSharedTopic::~QduSharedTopic()
{
  stop_.store(true);
  if (worker_.joinable()) worker_.join();
}

bool QduSharedTopic::imu_at(
  std::chrono::steady_clock::time_point timestamp, Eigen::Quaterniond & q) const
{
  // [9.21-QDU-NORMAL-TX] Do not expose a cached quaternion after the receive stream is stale.
  // The normal Tracker/Solver path must stop instead of solving against an old gimbal pose.
  if (!imu_fresh()) return false;
  std::lock_guard<std::mutex> lock(imu_mutex_);
  if (imu_samples_.empty()) return false;

  // [9.21-QDU] Keep one sample before the requested host time, then interpolate when bracketed.
  while (imu_samples_.size() > 2 && imu_samples_[1].host_timestamp <= timestamp)
    imu_samples_.pop_front();

  if (imu_samples_.size() >= 2 && imu_samples_[0].host_timestamp <= timestamp &&
      timestamp <= imu_samples_[1].host_timestamp) {
    const auto span = std::chrono::duration<double>(
      imu_samples_[1].host_timestamp - imu_samples_[0].host_timestamp);
    const auto offset = std::chrono::duration<double>(timestamp - imu_samples_[0].host_timestamp);
    const double ratio = span.count() > 0.0 ? offset.count() / span.count() : 0.0;
    q = imu_samples_[0].q.slerp(std::clamp(ratio, 0.0, 1.0), imu_samples_[1].q).normalized();
  } else if (timestamp < imu_samples_.front().host_timestamp) {
    q = imu_samples_.front().q;
  } else {
    q = imu_samples_.back().q;
  }
  return true;
}

bool QduSharedTopic::imu_fresh() const
{
  const int64_t last = last_imu_ns_.load();
  if (last == 0 || !serial_open_.load()) return false;
  const int64_t age_ns = steady_ns(std::chrono::steady_clock::now()) - last;
  return age_ns >= 0 && age_ns <= static_cast<int64_t>(config_.stale_timeout_ms) * 1000000LL;
}

double QduSharedTopic::imu_age_ms() const
{
  const int64_t last = last_imu_ns_.load();
  // [9.21-QDU-READONLY] No quaternion has ever been decoded; -1 keeps this distinguishable from a
  // genuinely fresh sample and from a stalled stream that reports a large positive age.
  if (last == 0) return -1.0;
  const int64_t age_ns = steady_ns(std::chrono::steady_clock::now()) - last;
  if (age_ns < 0) return 0.0;  // Clock skew must not report a negative age.
  return static_cast<double>(age_ns) / 1e6;
}

bool QduSharedTopic::serial_open() const
{
  return serial_open_.load();
}

void QduSharedTopic::send(const QduTargetCommand & command)
{
  // [9.21-QDU] Never retain a command while disconnected/stale; this prevents replay after hotplug.
  if (!config_.tx_enabled) {
    ++suppressed_commands_;
    return;
  }
  if (!imu_fresh()) {
    ++suppressed_commands_;
    // [9.21-QDU] If RX alone becomes stale while the serial fd is still writable, send one neutral
    // pair so the controller cannot keep the last fire/target value latched.
    if (serial_open_.load() && !stale_neutral_queued_.exchange(true)) {
      std::lock_guard<std::mutex> lock(command_mutex_);
      pending_command_ = {};
      ++command_generation_;
    }
    return;
  }
  // [9.21-QDU-NORMAL-TX] A malformed upper-layer result must never become a SharedTopic frame.
  // A non-controlling command has no meaningful numeric target and is normalized below.
  if (command.control &&
      (!std::isfinite(command.yaw) || !std::isfinite(command.yaw_vel) ||
       !std::isfinite(command.yaw_acc) || !std::isfinite(command.pitch) ||
       !std::isfinite(command.pitch_vel) || !std::isfinite(command.pitch_acc))) {
    ++suppressed_commands_;
    std::lock_guard<std::mutex> lock(command_mutex_);
    pending_command_ = {};
    ++command_generation_;
    return;
  }
  stale_neutral_queued_.store(false);
  std::lock_guard<std::mutex> lock(command_mutex_);
  pending_command_ = command;
  ++command_generation_;
}

QduLinkStats QduSharedTopic::stats() const
{
  QduLinkStats result;
  result.serial_open = serial_open();
  result.imu_fresh = imu_fresh();
  result.open_count = open_count_.load();
  result.disconnect_count = disconnect_count_.load();
  result.received_bytes = received_bytes_.load();
  result.received_packets = received_packets_.load();
  result.received_quaternions = received_quaternions_.load();
  result.received_ahrs_quaternions = received_ahrs_quaternions_.load();
  result.received_gimbal_quaternions = received_gimbal_quaternions_.load();
  result.invalid_quaternions = invalid_quaternions_.load();
  result.unknown_topics = unknown_topics_.load();
  result.transmitted_commands = transmitted_commands_.load();
  result.suppressed_commands = suppressed_commands_.load();
  {
    std::lock_guard<std::mutex> lock(parser_stats_mutex_);
    result.parser = parser_stats_;
  }
  return result;
}

double QduSharedTopic::default_bullet_speed() const
{
  return config_.default_bullet_speed;
}

const std::string & QduSharedTopic::default_mode() const
{
  return config_.default_mode;
}

bool QduSharedTopic::tx_enabled() const
{
  return config_.tx_enabled;
}

bool QduSharedTopic::read_only() const
{
  return force_read_only_;
}

const std::string & QduSharedTopic::device() const
{
  return config_.device;
}

void QduSharedTopic::handle_packet(const qdu::Packet & packet)
{
  ++received_packets_;
  const bool quaternion_topic =
    std::find(quaternion_topic_keys_.begin(), quaternion_topic_keys_.end(), packet.topic_crc32) !=
    quaternion_topic_keys_.end();
  if (!quaternion_topic) {
    // [9.21-QDU] Other registered C-board topics may share the UART; ignore without desynchronizing.
    ++unknown_topics_;
    return;
  }
  if (packet.topic_crc32 == qdu::topic_crc32("ahrs_quaternion"))
    ++received_ahrs_quaternions_;
  else if (packet.topic_crc32 == qdu::topic_crc32("gimbal_quat"))
    ++received_gimbal_quaternions_;
  if (packet.payload.size() != kQuaternionPayloadSize) {
    ++invalid_quaternions_;
    return;
  }

  // [9.21-QDU] LibXR Quaternion inherits Eigen; its raw ABI is x,y,z,w, not constructor w,x,y,z.
  const double x = read_float_le(packet.payload.data());
  const double y = read_float_le(packet.payload.data() + 4);
  const double z = read_float_le(packet.payload.data() + 8);
  const double w = read_float_le(packet.payload.data() + 12);
  const double squared_norm = x * x + y * y + z * z + w * w;
  if (!std::isfinite(squared_norm) || squared_norm < 0.25 || squared_norm > 2.25) {
    ++invalid_quaternions_;
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    imu_samples_.push_back({Eigen::Quaterniond(w, x, y, z).normalized(), now});
    while (imu_samples_.size() > kMaxImuSamples) imu_samples_.pop_front();
  }
  last_imu_ns_.store(steady_ns(now));
  stale_neutral_queued_.store(false);
  ++received_quaternions_;
}

void QduSharedTopic::worker_loop()
{
  qdu::Parser parser;
  std::array<uint8_t, 512> read_buffer{};
  uint64_t sent_generation = 0;
  auto last_error_log = std::chrono::steady_clock::time_point::min();

  while (!stop_.load()) {
    try {
      serial::Timeout timeout = serial::Timeout::simpleTimeout(config_.read_timeout_ms);
      serial::Serial port(
        config_.device, config_.baud_rate, timeout, serial::eightbits, serial::parity_none,
        serial::stopbits_one, serial::flowcontrol_none);
      serial_open_.store(true);
      ++open_count_;
      parser.reset();
      last_imu_ns_.store(0);
      {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        imu_samples_.clear();
      }
      {
        std::lock_guard<std::mutex> lock(command_mutex_);
        pending_command_ = {};
        ++command_generation_;
        sent_generation = command_generation_;
      }
      stale_neutral_queued_.store(true);
      tools::logger()->info("[QDU] serial opened: {}", config_.device);

      if (config_.tx_enabled) {
        // [9.21-QDU] The first frame after reconnect is neutral; no pre-disconnect target survives.
        std::vector<uint8_t> neutral_payload(kTargetPayloadSize, 0);
        const auto target_packet = qdu::pack(
          target_euler_key_, steady_us_48(), neutral_payload.data(), neutral_payload.size());
        const uint8_t fire = 0;
        const auto fire_packet = qdu::pack(fire_notify_key_, steady_us_48(), &fire, 1);
        if (port.write(fire_packet) != fire_packet.size() ||
            port.write(target_packet) != target_packet.size())
          throw std::runtime_error("short QDU neutral write");
      }

      while (!stop_.load()) {
        const std::size_t count = port.read(read_buffer.data(), read_buffer.size());
        if (count != 0U) {
          last_error_log = std::chrono::steady_clock::time_point::min();
          received_bytes_ += count;
          parser.feed(read_buffer.data(), count, [this](const qdu::Packet & packet) {
            handle_packet(packet);
          });
          std::lock_guard<std::mutex> lock(parser_stats_mutex_);
          parser_stats_ = parser.stats();
        }

        QduTargetCommand command;
        uint64_t generation = sent_generation;
        {
          std::lock_guard<std::mutex> lock(command_mutex_);
          if (command_generation_ != sent_generation) {
            command = pending_command_;
            generation = command_generation_;
          }
        }
        if (config_.tx_enabled && generation != sent_generation) {
          // [9.21-QDU] QDU has no separate control flag: control=false is encoded as a zero target.
          const QduTargetCommand safe = command.control ? command : QduTargetCommand{};
          const auto target_payload = qdu::target_euler_payload(
            safe.yaw, safe.yaw_vel, safe.yaw_acc, safe.pitch, safe.pitch_vel,
            safe.pitch_acc);
          const uint8_t fire = safe.control && safe.fire ? 1U : 0U;
          const auto target_packet = qdu::pack(
            target_euler_key_, steady_us_48(), target_payload.data(), target_payload.size());
          const auto fire_packet = qdu::pack(fire_notify_key_, steady_us_48(), &fire, 1);
          // [9.21-QDU] Disarm before changing a non-firing target; arm only after its target landed.
          if (fire == 0U) {
            if (port.write(fire_packet) != fire_packet.size() ||
                port.write(target_packet) != target_packet.size())
              throw std::runtime_error("short QDU command write");
          } else if (port.write(target_packet) != target_packet.size() ||
                     port.write(fire_packet) != fire_packet.size()) {
            throw std::runtime_error("short QDU command write");
          }
          sent_generation = generation;
          ++transmitted_commands_;
        }
      }
      if (config_.tx_enabled && port.isOpen()) {
        // [9.21-QDU-NORMAL-TX] A normal process exit explicitly disarms and clears the target
        // before closing the USB serial port.  This prevents the C board from retaining the last
        // valid Aimer command while users stop `standard` with Ctrl+C.
        std::vector<uint8_t> neutral_payload(kTargetPayloadSize, 0);
        const uint8_t fire = 0;
        const auto fire_packet = qdu::pack(fire_notify_key_, steady_us_48(), &fire, 1);
        const auto target_packet = qdu::pack(
          target_euler_key_, steady_us_48(), neutral_payload.data(), neutral_payload.size());
        if (port.write(fire_packet) != fire_packet.size() ||
            port.write(target_packet) != target_packet.size())
          throw std::runtime_error("short QDU shutdown-neutral write");
      }
      if (port.isOpen()) port.close();
    } catch (const std::exception & error) {
      const bool was_open = serial_open_.exchange(false);
      last_imu_ns_.store(0);
      if (was_open) ++disconnect_count_;
      {
        std::lock_guard<std::mutex> lock(command_mutex_);
        pending_command_ = {};
        ++command_generation_;
        sent_generation = command_generation_;
      }
      stale_neutral_queued_.store(true);
      const auto now = std::chrono::steady_clock::now();
      if (last_error_log == std::chrono::steady_clock::time_point::min() ||
          now - last_error_log >= std::chrono::seconds(2)) {
        tools::logger()->warn(
          "[QDU] serial unavailable: {}; retry in {} ms", error.what(),
          config_.reconnect_interval_ms);
        last_error_log = now;
      }
      const int slices = std::max(1, config_.reconnect_interval_ms / 20);
      for (int i = 0; i < slices && !stop_.load(); ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }
  serial_open_.store(false);
}

}  // namespace io
