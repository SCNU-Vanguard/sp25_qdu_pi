#ifndef IO__GIMBAL_HPP
#define IO__GIMBAL_HPP

#include <Eigen/Geometry>

#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>

#include "io/cboard.hpp"

namespace io
{

// [9.21-QDU] This remains an internal SP25 command container; it is no longer written as the old
// 'S','P' packed serial frame.
struct VisionToGimbal
{
  uint8_t mode{};
  float yaw{};
  float yaw_vel{};
  float yaw_acc{};
  float pitch{};
  float pitch_vel{};
  float pitch_acc{};
};

enum class GimbalMode
{
  IDLE,
  AUTO_AIM,
  SMALL_BUFF,
  BIG_BUFF
};

struct GimbalState
{
  float yaw{};
  float yaw_vel{};
  float pitch{};
  float pitch_vel{};
  float bullet_speed{};
  uint16_t bullet_count{};
};

class Gimbal
{
public:
  explicit Gimbal(const std::string & config_path);
  ~Gimbal();

  GimbalMode mode() const;
  GimbalState state() const;
  std::string str(GimbalMode mode) const;
  Eigen::Quaterniond q(std::chrono::steady_clock::time_point timestamp);

  void send(
    bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch,
    float pitch_vel, float pitch_acc);
  void send(const VisionToGimbal & command);

  bool imu_fresh() const;
  QduLinkStats link_stats() const;

private:
  CBoard cboard_;
  GimbalMode mode_{GimbalMode::IDLE};
  mutable std::mutex state_mutex_;
  GimbalState state_{};
};

}  // namespace io

#endif  // IO__GIMBAL_HPP
