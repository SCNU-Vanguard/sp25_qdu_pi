#include "aimer.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <stdexcept>
#include <vector>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"

namespace auto_aim
{
Aimer::Aimer(const std::string & config_path)
: left_yaw_offset_(std::nullopt), right_yaw_offset_(std::nullopt)
{
  auto yaml = YAML::LoadFile(config_path);
  yaw_offset_ = yaml["yaw_offset"].as<double>() / 57.3;        // degree to rad
  pitch_offset_ = yaml["pitch_offset"].as<double>() / 57.3;    // degree to rad
  comming_angle_ = yaml["comming_angle"].as<double>() / 57.3;  // degree to rad
  leaving_angle_ = yaml["leaving_angle"].as<double>() / 57.3;  // degree to rad
  high_speed_delay_time_ = yaml["high_speed_delay_time"].as<double>();
  low_speed_delay_time_ = yaml["low_speed_delay_time"].as<double>();
  decision_speed_ = yaml["decision_speed"].as<double>();
  // [9.9-10] 保留原配置补偿值，和显式实测处理延迟分别相加；无效配置直接报错。
  if (yaml["max_host_frame_age_ms"])
    max_host_frame_age_s_ = yaml["max_host_frame_age_ms"].as<double>() / 1000.0;
  if (!std::isfinite(high_speed_delay_time_) || high_speed_delay_time_ < 0 || high_speed_delay_time_ > 1 ||
      !std::isfinite(low_speed_delay_time_) || low_speed_delay_time_ < 0 || low_speed_delay_time_ > 1 ||
      !std::isfinite(decision_speed_) || decision_speed_ < 0 ||
      !std::isfinite(max_host_frame_age_s_) || max_host_frame_age_s_ <= 0 || max_host_frame_age_s_ > 1)
    throw std::invalid_argument("Aimer: invalid delay configuration");
  if (yaml["left_yaw_offset"].IsDefined() && yaml["right_yaw_offset"].IsDefined()) {
    left_yaw_offset_ = yaml["left_yaw_offset"].as<double>() / 57.3;    // degree to rad
    right_yaw_offset_ = yaml["right_yaw_offset"].as<double>() / 57.3;  // degree to rad
    tools::logger()->info("[Aimer] successfully loading shootmode");
  }
  if (!std::isfinite(yaw_offset_) || !std::isfinite(pitch_offset_) || !std::isfinite(comming_angle_) ||
      !std::isfinite(leaving_angle_) || (left_yaw_offset_ && !std::isfinite(*left_yaw_offset_)) ||
      (right_yaw_offset_ && !std::isfinite(*right_yaw_offset_)))
    throw std::invalid_argument("Aimer: non-finite angular configuration");
}

io::Command Aimer::aim(
  const std::list<Target> & targets, double bullet_speed, double pipeline_delay_s)
{
  // [9.9-10] 负延迟代表未来帧/错误时间域；过旧或非有限延迟停止控制，不夹成零继续预测。
  debug_aim_point = {};
  if (targets.empty() || !std::isfinite(pipeline_delay_s) || pipeline_delay_s < 0 ||
      pipeline_delay_s > max_host_frame_age_s_ || !std::isfinite(bullet_speed)) {
    lock_id_ = -1;
    return {};
  }
  if (targets.front().ekf().x.size() != 11 || !targets.front().ekf().x.allFinite() ||
      !targets.front().ekf().P.allFinite()) return {};
  auto target = targets.front();

  // [9.9-10] 速度门限按角速度大小判断，正/反向高速旋转使用同一延迟配置。
  double delay_time =
    std::abs(target.ekf().x[7]) > decision_speed_ ? high_speed_delay_time_ : low_speed_delay_time_;

  if (bullet_speed < 14) bullet_speed = 23;

  // [9.9-10] 只按时长预测，既不读取 now()，也不把录像时间戳当成本机当前时刻。
  target.predict(pipeline_delay_s + delay_time);
  if (!target.ekf().x.allFinite() || !target.ekf().P.allFinite()) return {};

  auto aim_point0 = choose_aim_point(target);
  debug_aim_point = aim_point0;
  if (!aim_point0.valid) {
    // tools::logger()->debug("Invalid aim_point0.");
    return {false, false, 0, 0};
  }

  Eigen::Vector3d xyz0 = aim_point0.xyza.head(3);
  auto d0 = std::sqrt(xyz0[0] * xyz0[0] + xyz0[1] * xyz0[1]);
  tools::Trajectory trajectory0(bullet_speed, d0, xyz0[2]);
  if (trajectory0.unsolvable || !std::isfinite(trajectory0.fly_time) || trajectory0.fly_time <= 0 ||
      !std::isfinite(trajectory0.pitch)) {
    tools::logger()->debug(
      "[Aimer] Unsolvable trajectory0: {:.2f} {:.2f} {:.2f}", bullet_speed, d0, xyz0[2]);
    debug_aim_point.valid = false;
    return {false, false, 0, 0};
  }

  // 迭代求解飞行时间 (最多10次，收敛条件：相邻两次fly_time差 <0.001)
  double prev_fly_time = trajectory0.fly_time;
  tools::Trajectory current_traj = trajectory0;

  for (int iter = 0; iter < 10; ++iter) {
    // [9.9-10] 每次从已补偿处理延迟的状态开始，仅再预测飞行时间，不能累加各次迭代。
    auto iteration_target = target;
    iteration_target.predict(prev_fly_time);
    if (!iteration_target.ekf().x.allFinite()) {
      debug_aim_point = {};
      return {};
    }

    // 计算瞄准点
    auto aim_point = choose_aim_point(iteration_target);
    debug_aim_point = aim_point;
    if (!aim_point.valid) {
      return {false, false, 0, 0};
    }

    // 计算新弹道
    Eigen::Vector3d xyz = aim_point.xyza.head(3);
    double d = std::sqrt(xyz.x() * xyz.x() + xyz.y() * xyz.y());
    current_traj = tools::Trajectory(bullet_speed, d, xyz.z());

    // 检查弹道是否可解
    if (current_traj.unsolvable || !std::isfinite(current_traj.fly_time) || current_traj.fly_time <= 0 ||
        !std::isfinite(current_traj.pitch)) {
      tools::logger()->debug(
        "[Aimer] Unsolvable trajectory in iter {}: speed={:.2f}, d={:.2f}, z={:.2f}", iter + 1,
        bullet_speed, d, xyz.z());
      debug_aim_point.valid = false;
      return {false, false, 0, 0};
    }

    // 检查收敛条件
    if (std::abs(current_traj.fly_time - prev_fly_time) < 0.001) {
      break;
    }
    prev_fly_time = current_traj.fly_time;
  }

  // 计算最终角度
  Eigen::Vector3d final_xyz = debug_aim_point.xyza.head(3);
  double yaw = std::atan2(final_xyz.y(), final_xyz.x()) + yaw_offset_;
  double pitch = -(current_traj.pitch + pitch_offset_);  //世界坐标系下pitch向上为负
  if (!std::isfinite(yaw) || !std::isfinite(pitch)) {
    debug_aim_point = {};
    return {};
  }
  return {true, false, yaw, pitch};
}

io::Command Aimer::aim(
  const std::list<Target> & targets, double bullet_speed, double pipeline_delay_s,
  io::ShootMode shoot_mode)
{
  double yaw_offset;
  if (shoot_mode == io::left_shoot && left_yaw_offset_.has_value()) {
    yaw_offset = left_yaw_offset_.value();
  } else if (shoot_mode == io::right_shoot && right_yaw_offset_.has_value()) {
    yaw_offset = right_yaw_offset_.value();
  } else {
    yaw_offset = yaw_offset_;
  }

  auto command = aim(targets, bullet_speed, pipeline_delay_s);
  if (!command.control) return command;  // [9.9-10] 停控命令不叠加左右枪偏置。
  command.yaw = command.yaw - yaw_offset_ + yaw_offset;

  return command;
}

AimPoint Aimer::choose_aim_point(const Target & target)
{
  Eigen::VectorXd ekf_x = target.ekf_x();
  std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
  const int armor_num = static_cast<int>(armor_xyza_list.size());  // [9.9-10] 与物理装甲面 ID 类型一致。
  // [9.9-10] 空/非有限预测不能生成有效瞄准点。
  if (!ekf_x.allFinite() || armor_xyza_list.empty()) return {};
  for (const auto & xyza : armor_xyza_list) if (!xyza.allFinite()) return {};
  // 如果装甲板未发生过跳变，则只有当前装甲板的位置已知
  if (!target.jumped) return {true, armor_xyza_list[0]};

  // 整车旋转中心的球坐标yaw
  auto center_yaw = std::atan2(ekf_x[2], ekf_x[0]);

  // 如果delta_angle为0，则该装甲板中心和整车中心的连线在世界坐标系的xy平面过原点
  std::vector<double> delta_angle_list;
  for (int i = 0; i < armor_num; i++) {
    auto delta_angle = tools::limit_rad(armor_xyza_list[i][3] - center_yaw);
    delta_angle_list.emplace_back(delta_angle);
  }

  // 不考虑小陀螺
  if (std::abs(target.ekf_x()[8]) <= 2 && target.name != ArmorName::outpost) {
    // 选择在可射击范围内的装甲板
    std::vector<int> id_list;
    for (int i = 0; i < armor_num; i++) {
      if (std::abs(delta_angle_list[i]) > 60 / 57.3) continue;
      id_list.push_back(i);
    }
    // 绝无可能
    if (id_list.empty()) {
      tools::logger()->warn("Empty id list!");
      return {false, armor_xyza_list[0]};
    }

    // 锁定模式：防止在两个都呈45度的装甲板之间来回切换
    if (id_list.size() > 1) {
      int id0 = id_list[0], id1 = id_list[1];

      // 未处于锁定模式时，选择delta_angle绝对值较小的装甲板，进入锁定模式
      if (lock_id_ != id0 && lock_id_ != id1)
        lock_id_ = (std::abs(delta_angle_list[id0]) < std::abs(delta_angle_list[id1])) ? id0 : id1;

      return {true, armor_xyza_list[lock_id_]};
    }

    // 只有一个装甲板在可射击范围内时，退出锁定模式
    lock_id_ = -1;
    return {true, armor_xyza_list[id_list[0]]};
  }

  double coming_angle, leaving_angle;
  if (target.name == ArmorName::outpost) {
    coming_angle = 70 / 57.3;
    leaving_angle = 30 / 57.3;
  } else {
    coming_angle = comming_angle_;
    leaving_angle = leaving_angle_;
  }

  // 在小陀螺时，一侧的装甲板不断出现，另一侧的装甲板不断消失，显然前者被打中的概率更高
  for (int i = 0; i < armor_num; i++) {
    if (std::abs(delta_angle_list[i]) > coming_angle) continue;
    if (ekf_x[7] > 0 && delta_angle_list[i] < leaving_angle) return {true, armor_xyza_list[i]};
    if (ekf_x[7] < 0 && delta_angle_list[i] > -leaving_angle) return {true, armor_xyza_list[i]};
  }

  return {false, armor_xyza_list[0]};
}

}  // namespace auto_aim
