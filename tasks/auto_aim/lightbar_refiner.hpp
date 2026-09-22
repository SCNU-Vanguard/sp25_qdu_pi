#ifndef AUTO_AIM__LIGHTBAR_REFINER_HPP
#define AUTO_AIM__LIGHTBAR_REFINER_HPP

#include <list>
#include "armor.hpp"

namespace auto_aim
{
// [Pi灯条] 恢复原SP25“网络候选附近找灯条修正角点”的功能，不恢复旧分类器/OpenVINO。
// 默认关闭；修正失败时保留网络候选，保持原SP25“尽力修正但不因修正失败丢框”的行为。
struct LightbarRefineOptions
{
  bool enabled = false;
  int threshold = 150;
  double min_length = 6.0;  // 当前720x540图像上的像素，不是网络640x512坐标。
  double max_corner_shift_ratio = 0.75;  // 每个角点允许移动的距离 / 对应实测灯条长度。
  double big_armor_ratio = 3.2;  // 由YOLO保持与Decoder的几何分类阈值一致。
};

struct LightbarRefineStats
{
  std::size_t input = 0, lights = 0, unmatched = 0, ambiguous = 0, duplicates = 0,
              fallback = 0, output = 0;
};

class LightbarRefiner
{
public:
  explicit LightbarRefiner(LightbarRefineOptions options = {});
  // [Pi灯条] 只读同帧BGR。每帧统一编号灯条，用“同一对灯条”判重；编号不跨帧跟踪。
  std::list<Armor> refine(const cv::Mat & bgr, std::list<Armor> candidates);
  bool enabled() const { return options_.enabled; }
  const LightbarRefineStats & stats() const { return stats_; }

private:
  LightbarRefineOptions options_;
  LightbarRefineStats stats_;
  cv::Mat blue_, red_, bright_, binary_;  // [Pi灯条·实拍] 按灯色保留单色亮端点。
};
}  // namespace auto_aim
#endif
