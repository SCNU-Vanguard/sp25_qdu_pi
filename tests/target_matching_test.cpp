// [9.9-8] 平衡步兵两面匹配及坏观测回归，运行真实 Target/EKF，Sanitizer 检查内存边界。
#include "tasks/auto_aim/target.hpp"
#include "tools/math_tools.hpp"

#include <iostream>
#include <limits>
#include <stdexcept>

namespace
{
using namespace auto_aim;
void require(bool value, const char * message)
{
  if (!value) throw std::runtime_error(message);
}

Armor observation(const Eigen::Vector4d & xyza = {4, 0.2, 0.15, 0})
{
  Armor armor(red, three, big, 0.9, {10, 10, 24, 10},
              {{10, 10}, {34, 10}, {34, 20}, {10, 20}}, {64, 64});
  armor.xyz_in_world = xyza.head<3>();
  armor.ypr_in_world = {xyza[3], 0, 0};
  armor.ypd_in_world = tools::xyz2ypd(armor.xyz_in_world);
  return armor;
}

Target target(int count)
{
  return Target(observation(), std::chrono::steady_clock::now(), 0.2, count, Eigen::VectorXd::Ones(11));
}

void unchanged_after_bad_observation(Target & candidate, const Armor & armor)
{
  const auto before_x = candidate.ekf_x();
  const auto before_p = candidate.ekf().P;
  const auto before_id = candidate.last_id;
  const auto before_jump = candidate.jumped;
  const auto before_updates = candidate.ekf().recent_nis_failures.size();
  candidate.update(armor);
  require(candidate.last_id == before_id && candidate.jumped == before_jump &&
          candidate.ekf().recent_nis_failures.size() == before_updates &&
          candidate.ekf_x().isApprox(before_x, 0) && candidate.ekf().P.isApprox(before_p, 0),
          "bad or empty candidates mutated matching state/EKF");
}
}  // namespace

int main()
{
  try {
    // 放在首项：未修复代码会访问两元素候选 vector 的第三项，触发 ASan 越界。
    for (const int count : {2, 3, 4, 1}) {
      for (const int id : {0, count - 1}) {
        auto candidate = target(count);
        const auto xyza = candidate.armor_xyza_list().at(id);
        candidate.update(observation(xyza));
        require(candidate.last_id == id, "matched wrong physical armor id");
        require(candidate.jumped == (id != 0), "changed SP25 jumped semantics");
        require(candidate.ekf_x().allFinite() && candidate.ekf().P.allFinite(), "valid update corrupted EKF");
      }
    }

    // 四面模型仍只匹配距离最近的三面；正后方的第 2 面不能意外加入候选范围。
    auto four = target(4);
    four.update(observation(four.armor_xyza_list().at(2)));
    require(four.last_id >= 0 && four.last_id < 4 && four.last_id != 2, "changed closest-three policy");

    auto balance = target(2);
    for (int step = 0; step < 100; ++step) {
      const int id = step % 2;
      balance.predict(0.01);
      balance.update(observation(balance.armor_xyza_list().at(id)));
      require(balance.last_id == id && balance.ekf_x().allFinite(), "repeated two-face updates failed");
    }
    require(balance.jumped && balance.convergened(), "valid updates lost switch/convergence state");

    for (const int count : {0, -1}) {
      auto empty = target(count);
      unchanged_after_bad_observation(empty, observation());
    }
    for (const auto bad : {std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::infinity(),
                           -std::numeric_limits<double>::infinity()}) {
      for (int field = 0; field < 3; ++field) {
        for (int axis = 0; axis < 3; ++axis) {
          auto armor = observation();
          if (field == 0) armor.xyz_in_world[axis] = bad;
          if (field == 1) armor.ypr_in_world[axis] = bad;
          if (field == 2) armor.ypd_in_world[axis] = bad;
          unchanged_after_bad_observation(balance, armor);
        }
      }
    }
    Target uninitialized;
    uninitialized.update(observation());
    require(uninitialized.ekf_x().size() == 0, "default target unexpectedly gained EKF state");

    std::cout << "Target: 1/2/3/4 candidates, closest-three policy, repeated balance updates, empty/NaN/Inf passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
