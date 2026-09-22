#include "camera_frame.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace io
{
// [9.9-5] 池容量和总像素内存有上限，防止错误配置耗尽 2 GiB 树莓派内存。
LatestFrameBuffer::LatestFrameBuffer(std::size_t pool_size)
{
  if (pool_size < 2 || pool_size > 16) throw std::invalid_argument("pool_size must be in [2, 16]");
  slots_.reserve(pool_size);
  for (std::size_t i = 0; i < pool_size; ++i) slots_.push_back(std::make_shared<FrameStorage>());
}

void LatestFrameBuffer::configure(cv::Size size)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (size.width <= 0 || size.height <= 0 ||
      static_cast<std::uint64_t>(size.width) * size.height > 256ULL * 1024 * 1024 / 3 / slots_.size())
    throw std::invalid_argument("camera frame pool requires positive size and at most 256 MiB");
  if (size_ == size) return;
  // 重连不改变已交付帧的像素或尺寸；需要换分辨率时重新创建相机对象。
  if (size_.area() != 0) throw std::runtime_error("camera output size changed after reconnect");
  for (auto & slot : slots_) slot->image.create(size, CV_8UC3);
  size_ = size;
}

std::shared_ptr<FrameStorage> LatestFrameBuffer::acquire(
  std::chrono::steady_clock::time_point timestamp, std::uint32_t device_frame_number)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (stopped_) return {};
  if (size_.area() == 0) throw std::logic_error("configure camera pool before acquiring frames");
  ++stats_.captured;
  ++sequence_;
  for (auto & slot : slots_) {
    if (slot.use_count() != 1) continue;  // 仅池自身持有时才能写，消费者不会被覆盖。
    slot->timestamp = timestamp;
    slot->sequence = sequence_;
    slot->device_frame_number = device_frame_number;
    return slot;
  }
  ++stats_.pool_dropped;
  return {};
}

void LatestFrameBuffer::publish(std::shared_ptr<FrameStorage> frame)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (stopped_) return;
  if (!frame || std::find(slots_.begin(), slots_.end(), frame) == slots_.end() ||
      frame->sequence <= delivered_sequence_ ||
      (latest_ && frame->sequence <= latest_->sequence))
    throw std::logic_error("publish requires a new frame from this pool");
  clear_latest();
  latest_ = std::move(frame);
  ready_.notify_one();
}

CameraFrame LatestFrameBuffer::read(std::chrono::milliseconds timeout)
{
  if (timeout.count() < 0) throw std::invalid_argument("camera read timeout cannot be negative");
  std::unique_lock<std::mutex> lock(mutex_);
  if (!ready_.wait_for(lock, timeout, [&] {
        return stopped_ || (latest_ && latest_->sequence > delivered_sequence_);
      })) {
    ++stats_.read_timeouts;
    return {};
  }
  if (stopped_) return {};
  delivered_sequence_ = latest_->sequence;
  ++stats_.delivered;
  return latest_;
}

void LatestFrameBuffer::clear_latest()
{
  // 只统计已发布但未交付的帧；不通过 sequence 差值重复计算 pool_dropped。
  if (latest_ && latest_->sequence > delivered_sequence_) ++stats_.consumer_skipped;
  latest_.reset();
}

void LatestFrameBuffer::invalidate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  clear_latest();
}

void LatestFrameBuffer::stop()
{
  std::lock_guard<std::mutex> lock(mutex_);
  stopped_ = true;
  clear_latest();
  ready_.notify_all();
}

CameraStats LatestFrameBuffer::stats() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return stats_;
}

void LatestFrameBuffer::capture_timeout()
{
  std::lock_guard<std::mutex> lock(mutex_);
  ++stats_.capture_timeouts;
}

void LatestFrameBuffer::reconnected()
{
  std::lock_guard<std::mutex> lock(mutex_);
  ++stats_.reconnects;
}
}  // namespace io
