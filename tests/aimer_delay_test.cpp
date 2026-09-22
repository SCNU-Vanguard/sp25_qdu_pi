// [9.9-10] 真实 Target/EKF/Aimer/弹道回归：只改变时间域和延迟，不接任何控制设备。
#include "tasks/auto_aim/aimer.hpp"
#include "tools/math_tools.hpp"

#include <yaml-cpp/yaml.h>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace
{
using namespace auto_aim;
using Clock = std::chrono::steady_clock;
void require(bool value, const char * message)
{
  if (!value) throw std::runtime_error(message);
}

Armor observation(double seconds, double omega)
{
  Armor armor(red, three, big, 0.9, {10, 10, 24, 10},
              {{10, 10}, {34, 10}, {34, 20}, {10, 20}}, {64, 64});
  const double yaw = omega * seconds;
  armor.xyz_in_world = {4.2 - 0.2 * std::cos(yaw), 0.7 * seconds - 0.2 * std::sin(yaw), 0.15};
  armor.ypr_in_world = {yaw, 0, 0};
  armor.ypd_in_world = tools::xyz2ypd(armor.xyz_in_world);
  return armor;
}

Target moving_target(Clock::time_point epoch, double omega = 0)
{
  Eigen::VectorXd variance = Eigen::VectorXd::Ones(11);
  variance[1] = variance[3] = variance[7] = 100;
  Target target(observation(0, omega), epoch, 0.2, 2, variance);
  for (int step = 1; step <= 40; ++step) {
    target.predict(epoch + std::chrono::milliseconds(step * 10));
    target.update(observation(step * 0.01, omega));
  }
  require(!target.jumped && target.ekf_x().allFinite(), "invalid synthetic moving target");
  return target;
}

void same_command(const io::Command & a, const io::Command & b)
{
  require(a.control && b.control && !a.shoot && !b.shoot, "expected valid aiming commands");
  require(std::abs(a.yaw - b.yaw) < 1e-10 && std::abs(a.pitch - b.pitch) < 1e-10,
          "delay was applied inconsistently or more than once");
}

void stopped(Aimer & aimer, const std::list<Target> & targets, double speed, double delay)
{
  const auto command = aimer.aim(targets, speed, delay, io::left_shoot);
  require(!command.control && !command.shoot && command.yaw == 0 && command.pitch == 0 &&
          !aimer.debug_aim_point.valid, "invalid input retained control, offset, or debug point");
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    require(argc == 2, "expected generated test configuration");
    const std::string config = argv[1];
    const auto epoch = Clock::time_point(std::chrono::seconds(100));
    const auto target = moving_target(epoch);
    const auto future = moving_target(epoch + std::chrono::hours(24 * 365));
    require(std::abs(target.ekf_x()[3]) > 0.1, "fixture has no measurable lateral motion");
    const auto before_x = target.ekf_x();
    const auto before_p = target.ekf().P;
    Aimer explicit_delay(config), shifted_clock(config), zero_delay(config), advanced_aimer(config);
    const auto delayed = explicit_delay.aim({target}, 27, 0.02);
    same_command(delayed, shifted_clock.aim({future}, 27, 0.02));
    auto advanced = target;
    advanced.predict(0.02);
    same_command(delayed, advanced_aimer.aim({advanced}, 27, 0));
    require(explicit_delay.debug_aim_point.xyza.isApprox(advanced_aimer.debug_aim_point.xyza, 1e-10),
            "flight iterations accumulated pipeline delay");
    const auto immediate = zero_delay.aim({target}, 27, 0);
    require(immediate.control && std::abs(delayed.yaw - immediate.yaw) > 1e-5,
            "explicit delay has no effect on moving target");
    require(target.ekf_x().isApprox(before_x, 0) && target.ekf().P.isApprox(before_p, 0),
            "Aimer changed the caller's tracked state");

    auto yaml = YAML::LoadFile(config);
    yaml["low_speed_delay_time"] = 0.003;
    yaml["high_speed_delay_time"] = 0.011;
    const auto configured_path = std::filesystem::path(config).parent_path() / "aimer_configured.yaml";
    { std::ofstream out(configured_path); out << yaml; }
    for (const double omega : {0.0, 4.0, -4.0}) {
      const auto rotating = moving_target(epoch, omega);
      require((std::abs(rotating.ekf_x()[7]) > 2) == (omega != 0), "fixture missed speed threshold");
      auto expected_target = rotating;
      expected_target.predict(0.02 + (omega == 0 ? 0.003 : 0.011));
      Aimer configured(configured_path.string()), expected(config);
      same_command(configured.aim({rotating}, 27, 0.02), expected.aim({expected_target}, 27, 0));
    }

    Aimer guarded(config);
    require(guarded.aim({target}, 27, 0.05).control, "50 ms boundary rejected");
    for (const double invalid : {-0.001, 0.050001, std::numeric_limits<double>::quiet_NaN(),
                                 std::numeric_limits<double>::infinity()}) {
      require(guarded.aim({target}, 27, 0).control, "valid frame did not recover");
      stopped(guarded, {target}, 27, invalid);
    }
    stopped(guarded, {}, 27, 0);
    stopped(guarded, {target}, std::numeric_limits<double>::quiet_NaN(), 0);
    stopped(guarded, {target}, std::numeric_limits<double>::infinity(), 0);
    Target no_faces(observation(0, 0), epoch, 0.2, 0, Eigen::VectorXd::Ones(11));
    stopped(guarded, {no_faces}, 27, 0);
    require(guarded.aim({target}, 0, 0).control, "existing low-speed fallback changed");

    const auto invalid_path = std::filesystem::path(config).parent_path() / "aimer_invalid.yaml";
    for (const auto * key : {"low_speed_delay_time", "high_speed_delay_time", "max_host_frame_age_ms"}) {
      auto invalid_yaml = YAML::LoadFile(config);
      invalid_yaml[key] = -1;
      { std::ofstream out(invalid_path); out << invalid_yaml; }
      bool rejected = false;
      try { Aimer invalid_config(invalid_path.string()); }
      catch (const std::invalid_argument &) { rejected = true; }
      require(rejected, "invalid delay configuration accepted");
    }
    std::cout << "Aimer: clock independence, once-only delay, signed rotation, stale/invalid frames passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
