#include "camera.hpp"

#include <stdexcept>

#include "hikrobot/hikrobot.hpp"
#include "tools/yaml.hpp"

namespace io
{
Camera::Camera(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto camera_name = tools::read<std::string>(yaml, "camera_name");

  if (camera_name == "hikrobot") {
    // [9.9-6] 完整节点配置和串号选择，替代硬编码帧率及 VID/PID USB 复位。
    camera_ = std::make_unique<HikRobot>(HikRobotOptions::from_yaml(yaml));
  }

  else {
    // [9.21-PI-HIK-ONLY] Reject stale configurations instead of silently loading another SDK.
    throw std::runtime_error(
      "This Pi build supports camera_name: hikrobot only; configured: " + camera_name);
  }
}

void Camera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  camera_->read(img, timestamp);
}

// [9.9-5] 主线程持有帧引用，直到本轮处理结束。
CameraFrame Camera::read_frame() { return camera_->read_frame(); }
CameraStats Camera::stats() const { return camera_->stats(); }

}  // namespace io
