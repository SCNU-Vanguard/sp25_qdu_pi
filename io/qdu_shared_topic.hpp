#ifndef IO__QDU_SHARED_TOPIC_HPP
#define IO__QDU_SHARED_TOPIC_HPP

#include <Eigen/Geometry>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "io/qdu_shared_topic_protocol.hpp"

namespace io
{

struct QduTargetCommand
{
  bool control{};
  bool fire{};
  float yaw{};
  float yaw_vel{};
  float yaw_acc{};
  float pitch{};
  float pitch_vel{};
  float pitch_acc{};
};

struct QduLinkStats
{
  bool serial_open{};
  bool imu_fresh{};
  uint64_t open_count{};
  uint64_t disconnect_count{};
  uint64_t received_bytes{};
  uint64_t received_packets{};
  uint64_t received_quaternions{};
  uint64_t received_ahrs_quaternions{};
  uint64_t received_gimbal_quaternions{};
  uint64_t invalid_quaternions{};
  uint64_t unknown_topics{};
  uint64_t transmitted_commands{};
  uint64_t suppressed_commands{};
  qdu::ParserStats parser{};
};

class QduSharedTopic
{
public:
  // [9.21-QDU-READONLY] force_read_only is a hard gate, not a hint: when set, TX is disabled
  // regardless of the YAML value so a read-only acceptance entry physically cannot write to the
  // C board.  It defaults to false so every existing production entry keeps its behavior.
  explicit QduSharedTopic(const std::string & config_path, bool force_read_only = false);
  ~QduSharedTopic();

  QduSharedTopic(const QduSharedTopic &) = delete;
  QduSharedTopic & operator=(const QduSharedTopic &) = delete;

  bool imu_at(std::chrono::steady_clock::time_point timestamp, Eigen::Quaterniond & q) const;
  bool imu_fresh() const;
  // [9.21-QDU-READONLY] Age of the most recently decoded quaternion in ms, or -1 when none has
  // arrived yet.  Unlike imu_fresh() this reports the actual age, so a read-only acceptance entry
  // can show how live the attitude stream is instead of only whether it crossed the stale bound.
  double imu_age_ms() const;
  bool serial_open() const;
  void send(const QduTargetCommand & command);
  QduLinkStats stats() const;

  double default_bullet_speed() const;
  const std::string & default_mode() const;
  bool tx_enabled() const;
  bool read_only() const;
  const std::string & device() const;

private:
  struct Config
  {
    std::string device;
    uint32_t baud_rate{115200};
    bool tx_enabled{false};
    int reconnect_interval_ms{500};
    int read_timeout_ms{10};
    int stale_timeout_ms{250};
    double default_bullet_speed{21.7};
    std::string default_mode{"auto_aim"};
    std::vector<std::string> quaternion_topics{"ahrs_quaternion", "gimbal_quat"};
  };

  struct ImuSample
  {
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point host_timestamp;
  };

  Config config_;
  bool force_read_only_{false};  // [9.21-QDU-READONLY] 强制只读；禁止任何串口写操作。
  std::vector<uint32_t> quaternion_topic_keys_;
  uint32_t target_euler_key_{};
  uint32_t fire_notify_key_{};

  mutable std::mutex imu_mutex_;
  mutable std::deque<ImuSample> imu_samples_;
  std::atomic<int64_t> last_imu_ns_{0};

  mutable std::mutex command_mutex_;
  QduTargetCommand pending_command_{};
  uint64_t command_generation_{};

  std::thread worker_;
  std::atomic<bool> stop_{false};
  std::atomic<bool> serial_open_{false};
  std::atomic<bool> stale_neutral_queued_{true};
  std::atomic<uint64_t> open_count_{0};
  std::atomic<uint64_t> disconnect_count_{0};
  std::atomic<uint64_t> received_bytes_{0};
  std::atomic<uint64_t> received_packets_{0};
  std::atomic<uint64_t> received_quaternions_{0};
  std::atomic<uint64_t> received_ahrs_quaternions_{0};
  std::atomic<uint64_t> received_gimbal_quaternions_{0};
  std::atomic<uint64_t> invalid_quaternions_{0};
  std::atomic<uint64_t> unknown_topics_{0};
  std::atomic<uint64_t> transmitted_commands_{0};
  std::atomic<uint64_t> suppressed_commands_{0};

  mutable std::mutex parser_stats_mutex_;
  qdu::ParserStats parser_stats_{};

  static Config read_config(const std::string & path);
  void worker_loop();
  void handle_packet(const qdu::Packet & packet);
};

}  // namespace io

#endif  // IO__QDU_SHARED_TOPIC_HPP
