// [9.9-7] 通过两个真实 Tracker 入口检查排序；不单独复制比较器作为测试替身。
#include "tasks/auto_aim/tracker.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace
{
using namespace auto_aim;
void require(bool value, const char * message)
{
  if (!value) throw std::runtime_error(message);
}

Armor make_armor(cv::Size size, cv::Point2f center, int tag, ArmorPriority priority = fifth)
{
  const cv::Point2f pixel{center.x * size.width, center.y * size.height};
  const std::vector<cv::Point2f> points{
    pixel + cv::Point2f(-12, -5), pixel + cv::Point2f(12, -5),
    pixel + cv::Point2f(12, 5), pixel + cv::Point2f(-12, 5)};
  const auto x = static_cast<int>(std::floor(points[0].x));
  const auto y = static_cast<int>(std::floor(points[0].y));
  const cv::Rect box{x, y, static_cast<int>(std::ceil(points[2].x)) - x,
                         static_cast<int>(std::ceil(points[2].y)) - y};
  Armor armor(red, three, big, 0.9, box, points, size);
  armor.priority = priority;
  armor.class_id = tag;  // 仅测试标识；生产语义入口仍保留 class_id=-1。
  return armor;
}

void check_order(
  const std::string & config, bool omni, std::list<Armor> armors,
  const std::vector<int> & expected)
{
  Solver solver(config);
  Tracker tracker(config, solver);
  const auto now = std::chrono::steady_clock::now();
  std::list<Target> targets;
  if (omni) {
    targets = std::get<1>(tracker.track(std::vector<omniperception::DetectionResult>{}, armors, now));
  } else {
    targets = tracker.track(armors, now);
  }
  std::vector<int> actual;
  for (const auto & armor : armors) actual.push_back(armor.class_id);
  require(actual == expected, "Tracker order depends on fixed pixel center or violates priority/ties");
  if (expected.empty()) {
    require(targets.empty() && tracker.state() == "lost", "empty detections created a target");
  } else {
    require(targets.size() == 1 && tracker.state() == "detecting", "selection broke target creation");
    require(targets.front().priority == armors.front().priority, "wrong priority selected");
    require((targets.front().armor_xyza_list().front().head<3>() - armors.front().xyz_in_world).norm() < 1e-8,
            "selected target did not come from first ranked armor");
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    require(argc == 2, "tracker_selection_test requires synthetic config path");
    for (const auto & size : {cv::Size(720, 540), cv::Size(1440, 1080), cv::Size(640, 512), cv::Size(540, 720)}) {
      for (const bool omni : {false, true}) {
        // 720x540 下旧代码反而偏向右下角；网络 Armor 自行按真实尺寸归一化。
        check_order(argv[1], omni, {make_armor(size, {0.9F, 0.9F}, 2),
                                   make_armor(size, {0.5F, 0.5F}, 1)}, {1, 2});
        check_order(argv[1], omni, {make_armor(size, {0.5F, 0.5F}, 1, fifth),
                                   make_armor(size, {0.875F, 0.875F}, 2, first),
                                   make_armor(size, {0.625F, 0.5F}, 3, first)}, {3, 2, 1});
        // priority 和距离都相同时保持输入顺序，不引入随机换序或新的兵种规则。
        check_order(argv[1], omni, {make_armor(size, {0.75F, 0.5F}, 4),
                                   make_armor(size, {0.25F, 0.5F}, 5),
                                   make_armor(size, {0.5F, 0.75F}, 6)}, {4, 5, 6});
        check_order(argv[1], omni, {}, {});
        check_order(argv[1], omni, {make_armor(size, {0.5F, 0.5F}, 7)}, {7});
      }
    }
    std::cout << "Tracker: both entry points, 4 image sizes, priority, ties and empty/single lists passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
