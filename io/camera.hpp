#ifndef IO__CAMERA_HPP
#define IO__CAMERA_HPP

#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

#include "io/camera_frame.hpp"

namespace io
{
class CameraBase
{
public:
  virtual ~CameraBase() = default;
  virtual void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) = 0;
  // [9.9-5][9.21-PI-HIK-ONLY] 当前实车只保留海康的有超时池内最新帧驱动。
  virtual CameraFrame read_frame()
  {
    auto frame = std::make_shared<FrameStorage>();
    read(frame->image, frame->timestamp);
    frame->sequence = ++legacy_sequence_;
    return frame;
  }
  virtual CameraStats stats() const { return {}; }

private:
  std::uint64_t legacy_sequence_ = 0;
};

class Camera
{
public:
  Camera(const std::string & config_path);
  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp);
  // [9.9-5] 空引用表示本次没有新帧；持有引用直至本轮处理结束。
  CameraFrame read_frame();
  CameraStats stats() const;

private:
  std::unique_ptr<CameraBase> camera_;
};

}  // namespace io

#endif  // IO__CAMERA_HPP
