// [9.9-9] 真实 OpenCV PnP、失败不提交位姿、坏标定/IMU 与两个 Tracker 入口的回归。
#include "tasks/auto_aim/tracker.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

using namespace auto_aim;
using namespace std::chrono_literals;
void require(bool value, const char * message)
{
  if (!value) throw std::runtime_error(message);
}

Armor observation(Solver & solver, ArmorType type = big, ArmorName name = three)
{
  const auto points = solver.reproject_armor({4, 0.2, 0.15}, 0.15, type, name);
  return Armor(red, name, type, 0.9, cv::boundingRect(points), points, {1440, 1080});
}

void rejects(Solver & solver, Armor armor)
{
  armor.xyz_in_gimbal = {1, 2, 3}; armor.xyz_in_world = {4, 5, 6};
  armor.ypr_in_gimbal = {0.1, 0.2, 0.3}; armor.ypr_in_world = {0.4, 0.5, 0.6};
  armor.ypd_in_world = {0.7, 0.8, 9}; armor.yaw_raw = 0.9;
  const auto before = armor;
  require(!solver.solve(armor), "invalid observation accepted");
  require(armor.xyz_in_gimbal == before.xyz_in_gimbal && armor.xyz_in_world == before.xyz_in_world &&
          armor.ypr_in_gimbal == before.ypr_in_gimbal && armor.ypr_in_world == before.ypr_in_world &&
          armor.ypd_in_world == before.ypd_in_world && armor.yaw_raw == before.yaw_raw,
          "failed solve partially changed Armor pose");
}

int main(int argc, char ** argv)
{
  try {
    require(argc == 2, "synthetic config path required");
    Solver solver(argv[1]);
    auto valid = observation(solver);
    for (const auto type : {big, small}) {
      auto armor = observation(solver, type);
      require(solver.solve(armor), "valid projected armor rejected");
      require((armor.xyz_in_world - Eigen::Vector3d(4, 0.2, 0.15)).norm() < 1e-3 &&
              armor.ypr_in_world.allFinite() && std::isfinite(armor.yaw_raw), "pose recovery failed");
    }
    for (const int count : {0, 1, 3, 5}) {
      auto bad = valid; bad.points.resize(count);
      rejects(solver, bad);
    }
    for (const float value : {std::numeric_limits<float>::quiet_NaN(),
                              std::numeric_limits<float>::infinity(),
                              -std::numeric_limits<float>::infinity()}) {
      auto bad = valid; bad.points[0].x = value; rejects(solver, bad);
      bad = valid; bad.points[3].y = value; rejects(solver, bad);
    }
    auto bad = valid;
    bad.points = {{1, 1}, {2, 2}, {3, 3}, {4, 4}}; rejects(solver, bad);
    bad.points = {{1, 1}, {2, 1}, {1, 2}, {2, 2}}; rejects(solver, bad);
    bad = valid; bad.points[1] = bad.points[0]; rejects(solver, bad);
    bad = valid; bad.name = not_armor; rejects(solver, bad);
    require(std::isinf(solver.oupost_reprojection_error(bad, 0.1)), "diagnostic PnP did not reject bad input");
    require(std::isfinite(solver.oupost_reprojection_error(valid, 0.1)), "valid diagnostic reprojection failed");

    solver.set_R_gimbal2world({0, 0, 0, 0}); rejects(solver, valid);
    solver.set_R_gimbal2world({std::numeric_limits<double>::quiet_NaN(), 0, 0, 0}); rejects(solver, valid);
    solver.set_R_gimbal2world({2, 0, 0, 0});
    auto recovered = valid;
    require(solver.solve(recovered), "valid normalized IMU input did not recover solver");

    const auto fixture = std::filesystem::path(argv[1]).parent_path() / "bad_solver.yaml";
    for (int failure = 0; failure < 4; ++failure) {
      auto yaml = YAML::LoadFile(argv[1]);
      if (failure == 0) yaml["camera_matrix"] = std::vector<double>{1, 2};
      if (failure == 1) yaml["distort_coeffs"][0] = std::numeric_limits<double>::infinity();
      if (failure == 2) yaml["camera_matrix"][0] = 0;
      if (failure == 3) yaml["R_camera2gimbal"] = std::vector<double>{};
      { std::ofstream out(fixture); out << yaml; }
      bool rejected = false;
      try { Solver invalid(fixture.string()); } catch (const std::exception &) { rejected = true; }
      require(rejected, "malformed calibration accepted");
    }

    for (const bool omni : {false, true}) {
      Solver local_solver(argv[1]);
      Tracker tracker(argv[1], local_solver);
      auto time = std::chrono::steady_clock::now();
      const auto track = [&](std::list<Armor> & armors) {
        time += 10ms;
        if (omni) return std::get<1>(tracker.track(std::vector<omniperception::DetectionResult>{}, armors, time));
        return tracker.track(armors, time);
      };
      auto broken = valid; broken.points.clear(); broken.priority = first;
      std::list<Armor> detections{broken};
      require(track(detections).empty() && detections.empty() && tracker.state() == "lost",
              "invalid first detection created Target");
      detections = {broken, valid};
      require(track(detections).size() == 1 && detections.size() == 1, "did not fall back to next valid armor");
      detections = {valid};
      auto targets = track(detections);
      require(tracker.state() == "tracking", "valid observations failed to enter tracking");
      const auto update_count = targets.front().ekf().recent_nis_failures.size();
      detections = {broken};
      targets = track(detections);
      require(tracker.state() == "temp_lost" && detections.empty() && targets.size() == 1 &&
              targets.front().ekf().recent_nis_failures.size() == update_count,
              "bad PnP counted as found or updated EKF");
      detections = {valid}; track(detections);
      detections = {broken, valid}; targets = track(detections);
      require(tracker.state() == "tracking" && detections.size() == 1 && targets.front().priority == fifth,
              "invalid high-priority candidate forced a switch");
    }
    std::cout << "real PnP, atomic pose, invalid calibration/IMU and Tracker rejection passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n'; return 1;
  }
}
