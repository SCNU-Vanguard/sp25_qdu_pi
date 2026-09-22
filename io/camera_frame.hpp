#ifndef IO__CAMERA_FRAME_HPP
#define IO__CAMERA_FRAME_HPP

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <opencv2/core.hpp>
#include <vector>

namespace io
{
// [9.9-5] 持有 CameraFrame 才能保证池内图像不被复用；异步传递不能只复制 Mat 头。
struct FrameStorage
{
  cv::Mat image;  // CV_8UC3, BGR；消费者需要修改/独立保存时使用 clone()。
  std::chrono::steady_clock::time_point timestamp;  // 主机收帧时刻，不是曝光时刻。
  std::uint64_t sequence = 0;
  std::uint32_t device_frame_number = 0;
};
using CameraFrame = std::shared_ptr<const FrameStorage>;

struct CameraStats
{
  std::uint64_t captured = 0;
  std::uint64_t delivered = 0;
  std::uint64_t consumer_skipped = 0;
  std::uint64_t pool_dropped = 0;
  std::uint64_t read_timeouts = 0;
  std::uint64_t capture_timeouts = 0;
  std::uint64_t reconnects = 0;
};

// [9.9-5] 单生产者、单消费流：固定槽位 + 一个 latest 引用，绝不排队积压旧帧。
class LatestFrameBuffer
{
public:
  explicit LatestFrameBuffer(std::size_t pool_size);
  void configure(cv::Size size);
  std::shared_ptr<FrameStorage> acquire(
    std::chrono::steady_clock::time_point timestamp, std::uint32_t device_frame_number);
  void publish(std::shared_ptr<FrameStorage> frame);
  CameraFrame read(std::chrono::milliseconds timeout);
  void invalidate();
  void stop();
  CameraStats stats() const;
  void capture_timeout();
  void reconnected();

private:
  mutable std::mutex mutex_;
  std::condition_variable ready_;
  std::vector<std::shared_ptr<FrameStorage>> slots_;
  cv::Size size_;
  CameraFrame latest_;
  CameraStats stats_;
  std::uint64_t sequence_ = 0;
  std::uint64_t delivered_sequence_ = 0;
  bool stopped_ = false;
  void clear_latest();  // 调用方持锁。
};
}  // namespace io
#endif
