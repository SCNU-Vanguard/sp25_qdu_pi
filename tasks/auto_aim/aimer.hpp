#ifndef AUTO_AIM__AIMER_HPP
#define AUTO_AIM__AIMER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>

#include "io/cboard.hpp"
#include "io/command.hpp"
#include "target.hpp"

namespace auto_aim
{

struct AimPoint
{
  bool valid = false;  // [9.9-10] 无目标/延迟无效时，不保留上次有效瞄准点。
  Eigen::Vector4d xyza = Eigen::Vector4d::Zero();
};

class Aimer
{
public:
  AimPoint debug_aim_point;
  explicit Aimer(const std::string & config_path);
  // [9.9-10] 延迟单位为秒，相对输入 Target 的状态时刻；禁止内部猜测时间域或固定 5 ms。
  io::Command aim(const std::list<Target> & targets, double bullet_speed, double pipeline_delay_s);

  io::Command aim(
    const std::list<Target> & targets, double bullet_speed, double pipeline_delay_s,
    io::ShootMode shoot_mode);

private:
  double yaw_offset_;
  std::optional<double> left_yaw_offset_, right_yaw_offset_;
  double pitch_offset_;
  double comming_angle_;
  double leaving_angle_;
  double lock_id_ = -1;
  double high_speed_delay_time_;
  double low_speed_delay_time_;
  double decision_speed_;
  double max_host_frame_age_s_ = 0.05;  // [9.9-10] Aimer 入口过旧帧上限，YAML 单位为 ms。

  AimPoint choose_aim_point(const Target & target);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__AIMER_HPP
