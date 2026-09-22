#include "solver.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
constexpr double LIGHTBAR_LENGTH = 56e-3;     // m
constexpr double BIG_ARMOR_WIDTH = 230e-3;    // m
constexpr double SMALL_ARMOR_WIDTH = 135e-3;  // m

const std::vector<cv::Point3f> BIG_ARMOR_POINTS{
  {0, BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
  {0, BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}};
const std::vector<cv::Point3f> SMALL_ARMOR_POINTS{
  {0, SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
  {0, SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}};

Solver::Solver(const std::string & config_path) : R_gimbal2world_(Eigen::Matrix3d::Identity())
{
  auto yaml = YAML::LoadFile(config_path);

  // [9.9-9] 在 Eigen 从 data() 读取固定长度数组前检查长度和有限值，拒绝坏标定文件。
  const auto read_vector = [&](const char * key, std::size_t count) {
    auto values = yaml[key].as<std::vector<double>>();
    if (values.size() != count || !std::all_of(values.begin(), values.end(),
                                              [](double v) { return std::isfinite(v); }))
      throw std::invalid_argument(std::string("Solver: invalid calibration ") + key);
    return values;
  };
  auto R_gimbal2imubody_data = read_vector("R_gimbal2imubody", 9);
  auto R_camera2gimbal_data = read_vector("R_camera2gimbal", 9);
  auto t_camera2gimbal_data = read_vector("t_camera2gimbal", 3);
  R_gimbal2imubody_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_gimbal2imubody_data.data());
  R_camera2gimbal_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_camera2gimbal_data.data());
  t_camera2gimbal_ = Eigen::Matrix<double, 3, 1>(t_camera2gimbal_data.data());

  auto camera_matrix_data = read_vector("camera_matrix", 9);
  auto distort_coeffs_data = read_vector("distort_coeffs", 5);
  if (camera_matrix_data[0] <= 0 || camera_matrix_data[4] <= 0 ||
      camera_matrix_data[6] != 0 || camera_matrix_data[7] != 0 || camera_matrix_data[8] != 1)
    throw std::invalid_argument("Solver: invalid camera intrinsics");
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> camera_matrix(camera_matrix_data.data());
  Eigen::Matrix<double, 1, 5> distort_coeffs(distort_coeffs_data.data());
  cv::eigen2cv(camera_matrix, camera_matrix_);
  cv::eigen2cv(distort_coeffs, distort_coeffs_);
}

Eigen::Matrix3d Solver::R_gimbal2world() const { return R_gimbal2world_; }

void Solver::set_R_gimbal2world(const Eigen::Quaterniond & q)
{
  // [9.9-9] 坏四元数令本次解算失效，不能静默沿用上一帧姿态；下一次有效输入可恢复。
  if (!q.coeffs().allFinite() || !std::isfinite(q.norm()) || q.norm() < 1e-9) {
    R_gimbal2world_.setConstant(std::numeric_limits<double>::quiet_NaN());
    return;
  }
  Eigen::Matrix3d R_imubody2imuabs = q.normalized().toRotationMatrix();
  R_gimbal2world_ = R_gimbal2imubody_.transpose() * R_imubody2imuabs * R_gimbal2imubody_;
}

// [9.9-9] 全部解算成功后才提交位姿，异常和失败不会留下半更新的 Armor。
bool Solver::solve(Armor & armor) const
{
  try {
    Armor solved = armor;
    if (!solve_pnp(solved)) return false;
    // 保留 SP25 平衡步兵不做 yaw 优化的策略。
    const bool is_balance = solved.type == ArmorType::big &&
      (solved.name == ArmorName::three || solved.name == ArmorName::four || solved.name == ArmorName::five);
    if (!is_balance && !optimize_yaw(solved)) return false;
    armor.xyz_in_gimbal = solved.xyz_in_gimbal;
    armor.xyz_in_world = solved.xyz_in_world;
    armor.ypr_in_gimbal = solved.ypr_in_gimbal;
    armor.ypr_in_world = solved.ypr_in_world;
    armor.ypd_in_world = solved.ypd_in_world;
    armor.yaw_raw = solved.yaw_raw;
    return true;
  } catch (const cv::Exception &) {
    return false;
  }
}

bool Solver::solve_pnp(Armor & armor) const
{
  // [9.9-9] 公共 Armor 字段可被传统/调试入口改写，因此 PnP 前独立验证四点几何。
  if (armor.points.size() != 4 || (armor.type != big && armor.type != small) ||
      armor.name < one || armor.name >= not_armor || !R_gimbal2world_.allFinite()) return false;
  for (const auto & point : armor.points)
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) return false;
  for (std::size_t i = 0; i < 4; ++i) {
    const cv::Point2d a = cv::Point2d(armor.points[(i + 1) % 4]) - cv::Point2d(armor.points[i]);
    const cv::Point2d b = cv::Point2d(armor.points[(i + 2) % 4]) - cv::Point2d(armor.points[(i + 1) % 4]);
    const double cross = a.x * b.y - a.y * b.x;
    if (!std::isfinite(cross) || cross <= 1e-6) return false;
  }
  const auto & object_points =
    (armor.type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;

  cv::Vec3d rvec{}, tvec{};
  if (!cv::solvePnP(
    object_points, armor.points, camera_matrix_, distort_coeffs_, rvec, tvec, false,
    cv::SOLVEPNP_IPPE)) return false;
  // [9.9-9] OpenCV 相机坐标系 Z 向前；成功返回也不能代替数值/正深度检查。
  for (int axis = 0; axis < 3; ++axis)
    if (!std::isfinite(rvec[axis]) || !std::isfinite(tvec[axis])) return false;
  if (tvec[2] <= 0) return false;

  Eigen::Vector3d xyz_in_camera;
  cv::cv2eigen(tvec, xyz_in_camera);
  armor.xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;
  armor.xyz_in_world = R_gimbal2world_ * armor.xyz_in_gimbal;

  cv::Mat rmat;
  cv::Rodrigues(rvec, rmat);
  Eigen::Matrix3d R_armor2camera;
  cv::cv2eigen(rmat, R_armor2camera);
  Eigen::Matrix3d R_armor2gimbal = R_camera2gimbal_ * R_armor2camera;
  Eigen::Matrix3d R_armor2world = R_gimbal2world_ * R_armor2gimbal;
  if (!armor.xyz_in_gimbal.allFinite() || !armor.xyz_in_world.allFinite() ||
      !R_armor2gimbal.allFinite() || !R_armor2world.allFinite() ||
      armor.xyz_in_world.head<2>().norm() < 1e-9) return false;
  for (const auto & point : object_points) {
    const Eigen::Vector3d xyz = R_armor2camera * Eigen::Vector3d(point.x, point.y, point.z) + xyz_in_camera;
    if (!xyz.allFinite() || xyz.z() <= 0) return false;
  }
  armor.ypr_in_gimbal = tools::eulers(R_armor2gimbal, 2, 1, 0);
  armor.ypr_in_world = tools::eulers(R_armor2world, 2, 1, 0);

  armor.ypd_in_world = tools::xyz2ypd(armor.xyz_in_world);

  armor.yaw_raw = armor.ypr_in_world[0];
  return armor.ypr_in_gimbal.allFinite() && armor.ypr_in_world.allFinite() && armor.ypd_in_world.allFinite();
}

std::vector<cv::Point2f> Solver::reproject_armor(
  const Eigen::Vector3d & xyz_in_world, double yaw, ArmorType type, ArmorName name) const
{
  auto sin_yaw = std::sin(yaw);
  auto cos_yaw = std::cos(yaw);

  auto pitch = (name == ArmorName::outpost) ? -15.0 * CV_PI / 180.0 : 15.0 * CV_PI / 180.0;
  auto sin_pitch = std::sin(pitch);
  auto cos_pitch = std::cos(pitch);

  // clang-format off
  const Eigen::Matrix3d R_armor2world {
    {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
    {sin_yaw * cos_pitch,  cos_yaw, sin_yaw * sin_pitch},
    {         -sin_pitch,        0,           cos_pitch}
  };
  // clang-format on

  // get R_armor2camera t_armor2camera
  const Eigen::Vector3d & t_armor2world = xyz_in_world;
  Eigen::Matrix3d R_armor2camera =
    R_camera2gimbal_.transpose() * R_gimbal2world_.transpose() * R_armor2world;
  Eigen::Vector3d t_armor2camera =
    R_camera2gimbal_.transpose() * (R_gimbal2world_.transpose() * t_armor2world - t_camera2gimbal_);

  // get rvec tvec
  cv::Vec3d rvec;
  cv::Mat R_armor2camera_cv;
  cv::eigen2cv(R_armor2camera, R_armor2camera_cv);
  cv::Rodrigues(R_armor2camera_cv, rvec);
  cv::Vec3d tvec(t_armor2camera[0], t_armor2camera[1], t_armor2camera[2]);

  // reproject
  std::vector<cv::Point2f> image_points;
  const auto & object_points = (type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;
  cv::projectPoints(object_points, rvec, tvec, camera_matrix_, distort_coeffs_, image_points);
  return image_points;
}

double Solver::oupost_reprojection_error(Armor armor, const double & pitch)
{
  // [9.9-9] 复用经过检查的原始 PnP，保留此诊断接口按指定 pitch 重投影的含义。
  try {
    if (!std::isfinite(pitch) || !solve_pnp(armor))
      return std::numeric_limits<double>::infinity();
    auto yaw = armor.ypr_in_world[0];
    auto xyz_in_world = armor.xyz_in_world;

    auto sin_yaw = std::sin(yaw);
    auto cos_yaw = std::cos(yaw);

    auto sin_pitch = std::sin(pitch);
    auto cos_pitch = std::cos(pitch);

    // clang-format off
    const Eigen::Matrix3d _R_armor2world {
      {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
      {sin_yaw * cos_pitch,  cos_yaw, sin_yaw * sin_pitch},
      {         -sin_pitch,        0,           cos_pitch}
    };
    // clang-format on

    // get R_armor2camera t_armor2camera
    const Eigen::Vector3d & t_armor2world = xyz_in_world;
    Eigen::Matrix3d _R_armor2camera =
      R_camera2gimbal_.transpose() * R_gimbal2world_.transpose() * _R_armor2world;
    Eigen::Vector3d t_armor2camera =
      R_camera2gimbal_.transpose() * (R_gimbal2world_.transpose() * t_armor2world - t_camera2gimbal_);

    // get rvec tvec
    cv::Vec3d _rvec;
    cv::Mat R_armor2camera_cv;
    cv::eigen2cv(_R_armor2camera, R_armor2camera_cv);
    cv::Rodrigues(R_armor2camera_cv, _rvec);
    cv::Vec3d _tvec(t_armor2camera[0], t_armor2camera[1], t_armor2camera[2]);

    // reproject
    const auto & object_points = (armor.type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;
    std::vector<cv::Point2f> image_points;
    cv::projectPoints(object_points, _rvec, _tvec, camera_matrix_, distort_coeffs_, image_points);

    auto error = 0.0;
    for (int i = 0; i < 4; i++) error += cv::norm(armor.points[i] - image_points[i]);
    return std::isfinite(error) ? error : std::numeric_limits<double>::infinity();
  } catch (const cv::Exception &) {
    return std::numeric_limits<double>::infinity();
  }
}

bool Solver::optimize_yaw(Armor & armor) const
{
  Eigen::Vector3d gimbal_ypr = tools::eulers(R_gimbal2world_, 2, 1, 0);

  constexpr double SEARCH_RANGE = 140;  // degree
  auto yaw0 = tools::limit_rad(gimbal_ypr[0] - SEARCH_RANGE / 2 * CV_PI / 180.0);

  auto min_error = std::numeric_limits<double>::infinity();  // [9.9-9] 无有限重投影结果则失败。
  auto best_yaw = armor.ypr_in_world[0];

  for (int i = 0; i < SEARCH_RANGE; i++) {
    double yaw = tools::limit_rad(yaw0 + i * CV_PI / 180.0);
    auto error = armor_reprojection_error(armor, yaw, (i - SEARCH_RANGE / 2) * CV_PI / 180.0);

    if (std::isfinite(error) && error < min_error) {
      min_error = error;
      best_yaw = yaw;
    }
  }

  if (!std::isfinite(min_error) || !std::isfinite(best_yaw)) return false;
  armor.yaw_raw = armor.ypr_in_world[0];
  armor.ypr_in_world[0] = best_yaw;
  return true;
}

double Solver::SJTU_cost(
  const std::vector<cv::Point2f> & cv_refs, const std::vector<cv::Point2f> & cv_pts,
  const double & inclined) const
{
  std::size_t size = cv_refs.size();
  std::vector<Eigen::Vector2d> refs;
  std::vector<Eigen::Vector2d> pts;
  for (std::size_t i = 0u; i < size; ++i) {
    refs.emplace_back(cv_refs[i].x, cv_refs[i].y);
    pts.emplace_back(cv_pts[i].x, cv_pts[i].y);
  }
  double cost = 0.;
  for (std::size_t i = 0u; i < size; ++i) {
    std::size_t p = (i + 1u) % size;
    // i - p 构成线段。过程：先移动起点，再补长度，再旋转
    Eigen::Vector2d ref_d = refs[p] - refs[i];  // 标准
    Eigen::Vector2d pt_d = pts[p] - pts[i];
    // 长度差代价 + 起点差代价(1 / 2)（0 度左右应该抛弃)
    double pixel_dis =  // dis 是指方差平面内到原点的距离
      (0.5 * ((refs[i] - pts[i]).norm() + (refs[p] - pts[p]).norm()) +
       std::fabs(ref_d.norm() - pt_d.norm())) /
      ref_d.norm();
    double angular_dis = ref_d.norm() * tools::get_abs_angle(ref_d, pt_d) / ref_d.norm();
    // 平方可能是为了配合 sin 和 cos
    // 弧度差代价（0 度左右占比应该大）
    double cost_i =
      tools::square(pixel_dis * std::sin(inclined)) +
      tools::square(angular_dis * std::cos(inclined)) * 2.0;  // DETECTOR_ERROR_PIXEL_BY_SLOPE
    // 重投影像素误差越大，越相信斜率
    cost += std::sqrt(cost_i);
  }
  return cost;
}

double Solver::armor_reprojection_error(
  const Armor & armor, double yaw, const double & inclined) const
{
  auto image_points = reproject_armor(armor.xyz_in_world, yaw, armor.type, armor.name);
  auto error = 0.0;
  for (int i = 0; i < 4; i++) error += cv::norm(armor.points[i] - image_points[i]);
  // auto error = SJTU_cost(image_points, armor.points, inclined);

  return error;
}

// 世界坐标到像素坐标的转换
std::vector<cv::Point2f> Solver::world2pixel(const std::vector<cv::Point3f> & worldPoints)
{
  Eigen::Matrix3d R_world2camera = R_camera2gimbal_.transpose() * R_gimbal2world_.transpose();
  Eigen::Vector3d t_world2camera = -R_camera2gimbal_.transpose() * t_camera2gimbal_;

  cv::Mat rvec;
  cv::Mat tvec;
  cv::eigen2cv(R_world2camera, rvec);
  cv::eigen2cv(t_world2camera, tvec);

  std::vector<cv::Point3f> valid_world_points;
  for (const auto & world_point : worldPoints) {
    Eigen::Vector3d world_point_eigen(world_point.x, world_point.y, world_point.z);
    Eigen::Vector3d camera_point = R_world2camera * world_point_eigen + t_world2camera;

    if (camera_point.z() > 0) {
      valid_world_points.push_back(world_point);
    }
  }
  // 如果没有有效点，返回空vector
  if (valid_world_points.empty()) {
    return std::vector<cv::Point2f>();
  }
  std::vector<cv::Point2f> pixelPoints;
  cv::projectPoints(valid_world_points, rvec, tvec, camera_matrix_, distort_coeffs_, pixelPoints);
  return pixelPoints;
}
}  // namespace auto_aim
