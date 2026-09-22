#ifndef IO__CBOARD_HPP
#define IO__CBOARD_HPP

#include <Eigen/Geometry>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "io/command.hpp"
#include "io/qdu_shared_topic.hpp"

namespace io
{
enum Mode
{
  idle,
  auto_aim,
  small_buff,
  big_buff,
  outpost
};
const std::vector<std::string> MODES = {"idle", "auto_aim", "small_buff", "big_buff", "outpost"};

enum ShootMode
{
  left_shoot,
  right_shoot,
  both_shoot
};
const std::vector<std::string> SHOOT_MODES = {"left_shoot", "right_shoot", "both_shoot"};

class CBoard
{
public:
  // [9.21-QDU] These compatibility fields are configuration defaults because the QDU source
  // package does not publish mode, shoot-mode, bullet-speed, or UAV follow-target angle topics.
  double bullet_speed;
  Mode mode;
  ShootMode shoot_mode;
  double ft_angle;

  // [9.21-QDU-READONLY] force_read_only hard-disables the serial write path for acceptance runs
  // that must never command a gimbal.  Defaults to false so production entries are unchanged.
  explicit CBoard(const std::string & config_path, bool force_read_only = false);
  ~CBoard();

  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);
  void send(Command command) const;

  // [9.21-QDU-READONLY] Age of the newest decoded quaternion in ms, or -1 when none has arrived.
  // The read-only acceptance entry logs this to show the attitude stream is live.
  double imu_age_ms() const;

  // [9.21-QDU] MPC entries use the same physical link and wire protocol through this richer call.
  void send_target(
    bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch,
    float pitch_vel, float pitch_acc) const;
  bool imu_fresh() const;
  bool serial_open() const;
  bool tx_enabled() const;
  bool read_only() const;
  QduLinkStats link_stats() const;

private:
  std::unique_ptr<QduSharedTopic> link_;
};

}  // namespace io

#endif  // IO__CBOARD_HPP
