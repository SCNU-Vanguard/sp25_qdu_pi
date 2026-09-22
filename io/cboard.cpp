#include "cboard.hpp"

#include <stdexcept>

#include "tools/logger.hpp"

namespace io
{
namespace
{
Mode parse_mode(const std::string & value)
{
  for (std::size_t i = 0; i < MODES.size(); ++i) {
    if (MODES[i] == value) return static_cast<Mode>(i);
  }
  throw std::runtime_error("unknown qdu_communication.default_mode: " + value);
}
}  // namespace

CBoard::CBoard(const std::string & config_path, bool force_read_only)
: bullet_speed(0.0),
  mode(Mode::idle),
  shoot_mode(ShootMode::left_shoot),
  ft_angle(0.0),
  link_(std::make_unique<QduSharedTopic>(config_path, force_read_only))
{
  // [9.21-QDU] Keep the SP25 public API while replacing its former SocketCAN transport.
  bullet_speed = link_->default_bullet_speed();
  mode = parse_mode(link_->default_mode());
  tools::logger()->info(
    "[CBoard/QDU] mode={}, default bullet speed={:.2f} m/s; waiting for quaternion topic",
    MODES[mode], bullet_speed);
}

CBoard::~CBoard() = default;

Eigen::Quaterniond CBoard::imu_at(std::chrono::steady_clock::time_point timestamp)
{
  Eigen::Quaterniond result = Eigen::Quaterniond::Identity();
  if (!link_->imu_at(timestamp, result)) {
    // [9.21-QDU] Identity is only a compatibility return; send() remains inhibited until IMU is fresh.
    static auto last_warning = std::chrono::steady_clock::time_point::min();
    const auto now = std::chrono::steady_clock::now();
    if (last_warning == std::chrono::steady_clock::time_point::min() ||
        now - last_warning >= std::chrono::seconds(1)) {
      tools::logger()->warn("[CBoard/QDU] no quaternion yet; outgoing command is inhibited");
      last_warning = now;
    }
  }
  return result;
}

void CBoard::send(Command command) const
{
  send_target(
    command.control, command.shoot, static_cast<float>(command.yaw), 0.0F, 0.0F,
    static_cast<float>(command.pitch), 0.0F, 0.0F);
}

void CBoard::send_target(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch,
  float pitch_vel, float pitch_acc) const
{
  link_->send({control, fire, yaw, yaw_vel, yaw_acc, pitch, pitch_vel, pitch_acc});
}

bool CBoard::imu_fresh() const
{
  return link_->imu_fresh();
}

double CBoard::imu_age_ms() const
{
  return link_->imu_age_ms();
}

bool CBoard::serial_open() const
{
  return link_->serial_open();
}

bool CBoard::tx_enabled() const
{
  return link_->tx_enabled();
}

bool CBoard::read_only() const
{
  return link_->read_only();
}

QduLinkStats CBoard::link_stats() const
{
  return link_->stats();
}

}  // namespace io
