// [9.9-9] 本测试进程替换 solvePnP 符号，覆盖真实设备上难以稳定构造的失败返回/坏输出。
#include "tasks/auto_aim/solver.hpp"

#include <iostream>
#include <limits>
#include <stdexcept>

enum class Fault { none, no_solution, exception, negative_z, zero_z, nan_r, inf_t, corners_behind };
Fault fault = Fault::none;
int calls = 0;
namespace cv
{
bool solvePnP(InputArray, InputArray, InputArray, InputArray, OutputArray rvec, OutputArray tvec, bool, int)
{
  ++calls;
  if (fault == Fault::no_solution) return false;
  if (fault == Fault::exception) CV_Error(Error::StsBadArg, "injected PnP failure");
  Vec3d r(0, CV_PI / 2, 0), t(0, 0, 4);
  if (fault == Fault::negative_z) t[2] = -4;
  if (fault == Fault::zero_z) t[2] = 0;
  if (fault == Fault::nan_r) r[0] = std::numeric_limits<double>::quiet_NaN();
  if (fault == Fault::inf_t) t[1] = std::numeric_limits<double>::infinity();
  if (fault == Fault::corners_behind) { r = {0, 0, 0}; t[2] = 0.01; }
  Mat(r).copyTo(rvec); Mat(t).copyTo(tvec);
  return true;
}
}  // namespace cv

int main(int argc, char ** argv)
{
  try {
    if (argc != 2) throw std::runtime_error("config required");
    auto_aim::Solver solver(argv[1]);
    auto_aim::Armor armor(auto_aim::red, auto_aim::three, auto_aim::big, 0.9, {100, 100, 100, 40},
                          {{100, 100}, {200, 100}, {200, 140}, {100, 140}}, {1440, 1080});
    if (!solver.solve(armor) || calls != 1) throw std::runtime_error("PnP injection did not bind");
    const auto before = armor;
    for (const auto mode : {Fault::no_solution, Fault::exception, Fault::negative_z, Fault::zero_z,
                            Fault::nan_r, Fault::inf_t, Fault::corners_behind}) {
      fault = mode;
      if (solver.solve(armor) || armor.xyz_in_world != before.xyz_in_world ||
          armor.ypr_in_world != before.ypr_in_world || armor.yaw_raw != before.yaw_raw)
        throw std::runtime_error("failed PnP accepted or partially committed");
    }
    if (calls != 8) throw std::runtime_error("not all injected failures reached PnP");
    std::cout << "PnP false/exception, non-finite pose and positive-depth checks passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n'; return 1;
  }
}
