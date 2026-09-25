// Test-only camera source. Never linked into a production target.
#include "io/camera.hpp"

#include <yaml-cpp/yaml.h>
#include <opencv2/imgcodecs.hpp>
#include <stdexcept>
#include <thread>

namespace io
{
namespace
{
class FileCamera : public CameraBase
{
  cv::Mat image_;

public:
  explicit FileCamera(const std::string & config)
  {
    image_ = cv::imread(YAML::LoadFile(config)["test_image"].as<std::string>());
    if (image_.empty()) throw std::runtime_error("test image missing");
  }

  void read(cv::Mat & image, std::chrono::steady_clock::time_point & timestamp) override
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    image = image_.clone();
    timestamp = std::chrono::steady_clock::now();
  }
};
}

Camera::Camera(const std::string & config) : camera_(std::make_unique<FileCamera>(config)) {}
CameraFrame Camera::read_frame() { return camera_->read_frame(); }
void Camera::read(cv::Mat & image, std::chrono::steady_clock::time_point & timestamp)
{
  camera_->read(image, timestamp);
}
CameraStats Camera::stats() const { return camera_->stats(); }
}
