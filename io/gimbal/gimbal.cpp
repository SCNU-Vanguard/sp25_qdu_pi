#include "gimbal.hpp"

#include <cmath>

namespace io
{
namespace
{
GimbalMode to_gimbal_mode(Mode mode)
{
  switch (mode) {
    case Mode::auto_aim:
      return GimbalMode::AUTO_AIM;
    case Mode::small_buff:
      return GimbalMode::SMALL_BUFF;
    case Mode::big_buff:
      return GimbalMode::BIG_BUFF;
    default:
      return GimbalMode::IDLE;
  }
}
}  // namespace

Gimbal::Gimbal(const std::string & config_path) : cboard_(config_path)
{
  // [9.21-QDU] MPC and non-MPC programs now share one DevC-USB SharedTopic implementation.
  mode_ = to_gimbal_mode(cboard_.mode);
  state_.bullet_speed = static_cast<float>(cboard_.bullet_speed);
}

Gimbal::~Gimbal() = default;

GimbalMode Gimbal::mode() const
{
  return mode_;
}

GimbalState Gimbal::state() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return state_;
}

std::string Gimbal::str(GimbalMode mode) const
{
  switch (mode) {
    case GimbalMode::IDLE:
      return "IDLE";
    case GimbalMode::AUTO_AIM:
      return "AUTO_AIM";
    case GimbalMode::SMALL_BUFF:
      return "SMALL_BUFF";
    case GimbalMode::BIG_BUFF:
      return "BIG_BUFF";
  }
  return "UNKNOWN";
}

Eigen::Quaterniond Gimbal::q(std::chrono::steady_clock::time_point timestamp)
{
  const auto quaternion = cboard_.imu_at(timestamp);
  const auto rotation = quaternion.toRotationMatrix().eulerAngles(2, 1, 0);
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    // [9.21-QDU] No dedicated angle/velocity feedback topic exists in the supplied contract.
    state_.yaw = static_cast<float>(rotation[0]);
    state_.pitch = static_cast<float>(rotation[1]);
    state_.yaw_vel = 0.0F;
    state_.pitch_vel = 0.0F;
  }
  return quaternion;
}

void Gimbal::send(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch,
  float pitch_vel, float pitch_acc)
{
  cboard_.send_target(control, fire, yaw, yaw_vel, yaw_acc, pitch, pitch_vel, pitch_acc);
}

void Gimbal::send(const VisionToGimbal & command)
{
  send(
    command.mode != 0U, command.mode == 2U, command.yaw, command.yaw_vel, command.yaw_acc,
    command.pitch, command.pitch_vel, command.pitch_acc);
}

bool Gimbal::imu_fresh() const
{
  return cboard_.imu_fresh();
}

QduLinkStats Gimbal::link_stats() const
{
  return cboard_.link_stats();
}

}  // namespace io
