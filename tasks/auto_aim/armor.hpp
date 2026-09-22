#ifndef AUTO_AIM__ARMOR_HPP
#define AUTO_AIM__ARMOR_HPP

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace auto_aim
{
enum Color
{
  red,
  blue,
  extinguish,
  purple
};
const std::vector<std::string> COLORS = {"red", "blue", "extinguish", "purple"};

enum ArmorType
{
  big,
  small
};
const std::vector<std::string> ARMOR_TYPES = {"big", "small"};

enum ArmorName
{
  one,
  two,
  three,
  four,
  five,
  sentry,
  outpost,
  base,
  not_armor
};
const std::vector<std::string> ARMOR_NAMES = {"one",    "two",     "three", "four",     "five",
                                              "sentry", "outpost", "base",  "not_armor"};

enum ArmorPriority
{
  first = 1,
  second,
  third,
  forth,
  fifth
};

// [9.9-4] 删除仅供旧网络 class_id 构造使用的类别表；模型标签映射统一由 Decoder 负责。

struct Lightbar
{
  // [9.9-3] 网络 Armor 也持有默认灯条，初始化标量以保证复制检测结果时无未初始化读取。
  std::size_t id = 0;
  Color color = extinguish;
  cv::Point2f center{}, top{}, bottom{}, top2bottom{};  // [9.9-4] 显式标明默认几何状态。
  std::vector<cv::Point2f> points;
  double angle = 0, angle_error = 0, length = 0, width = 0, ratio = 0;
  cv::RotatedRect rotated_rect{};

  Lightbar(const cv::RotatedRect & rotated_rect, std::size_t id);
  Lightbar() = default;  // [9.9-4] 成员初值统一在声明处维护。
};

struct Armor
{
  // [9.9-3] 提前完成第三部分输出 Armor 必需的默认值；不改变 Tracker/PnP 等算法。
  Color color = extinguish;
  // [9.9-4] 各构造入口共用确定初值；网络角点仍保持 SP25 的 LT/RT/RB/LB 对应。
  Lightbar left{}, right{};
  cv::Point2f center{};       // 不是对角线交点，不能作为实际中心！
  cv::Point2f center_norm{};  // 归一化坐标
  std::vector<cv::Point2f> points;

  double ratio = 0;              // 两灯条的中点连线与长灯条的长度之比
  double side_ratio = 0;         // 长灯条与短灯条的长度之比
  double rectangular_error = 0;  // 灯条和中点连线所成夹角与π/2的差值

  ArmorType type = small;
  ArmorName name = not_armor;
  ArmorPriority priority = fifth;
  int class_id = -1;  // [9.9-4] 保留字段兼容 SP25；语义入口不再生成旧网络 class_id。
  cv::Rect box{};
  cv::Mat pattern;
  double confidence = 0;
  bool duplicated = false;

  Eigen::Vector3d xyz_in_gimbal = Eigen::Vector3d::Zero();  // 单位：m
  Eigen::Vector3d xyz_in_world = Eigen::Vector3d::Zero();   // 单位：m
  Eigen::Vector3d ypr_in_gimbal = Eigen::Vector3d::Zero();  // 单位：rad
  Eigen::Vector3d ypr_in_world = Eigen::Vector3d::Zero();   // 单位：rad
  Eigen::Vector3d ypd_in_world = Eigen::Vector3d::Zero();   // 球坐标系

  double yaw_raw = 0;  // rad

  // [9.9-4] 唯一网络入口：验证语义、四角点和 bbox，直接初始化并接管角点容器。
  Armor(
    Color color, ArmorName name, ArmorType type, double confidence, const cv::Rect & box,
    std::vector<cv::Point2f> keypoints, cv::Size image_size);

  Armor(const Lightbar & left, const Lightbar & right);
  // [9.9-4] 删除四套已无业务调用的裸 ID/ROI 网络构造；坐标变换在 Decoder 中完成。
};

}  // namespace auto_aim

#endif  // AUTO_AIM__ARMOR_HPP
