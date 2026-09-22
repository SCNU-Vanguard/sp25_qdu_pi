// [9.9-5] 验证最新帧语义、固定内存、消费者引用寿命和退出唤醒，不依赖相机。
#include "io/camera_frame.hpp"

#include <atomic>
#include <future>
#include <iostream>
#include <set>
#include <stdexcept>
#include <thread>

using namespace std::chrono_literals;
void require(bool value, const char * message)
{
  if (!value) throw std::runtime_error(message);
}
template <class F> void rejects(F action)
{
  bool rejected = false;
  try { action(); } catch (const std::exception &) { rejected = true; }
  require(rejected, "invalid pool operation was accepted");
}

void publish(io::LatestFrameBuffer & pool, unsigned char value)
{
  auto frame = pool.acquire(std::chrono::steady_clock::now(), value);
  require(bool(frame), "unexpected pool exhaustion");
  frame->image.setTo(cv::Scalar(value, value + 1, value + 2));
  pool.publish(std::move(frame));
}

int main()
{
  try {
    rejects([] { io::LatestFrameBuffer invalid(1); });
    io::LatestFrameBuffer pool(3);
    rejects([&] { pool.configure({-1, 1}); });
    rejects([&] { pool.configure({2147483647, 2147483647}); });
    pool.configure({8, 6});
    rejects([&] { pool.configure({9, 6}); });
    require(!pool.read(0ms), "empty pool returned a frame");
    publish(pool, 1);
    publish(pool, 2);
    auto first = pool.read(0ms);
    require(first && first->sequence == 2 && first->device_frame_number == 2, "not latest sequence");
    require(first->image.at<cv::Vec3b>(0, 0) == cv::Vec3b(2, 3, 4), "BGR frame differs");
    require(!pool.read(0ms), "same sequence delivered twice");
    publish(pool, 3);
    auto second = pool.read(0ms);
    publish(pool, 4);
    auto third = pool.read(0ms);
    require(!pool.acquire(std::chrono::steady_clock::now(), 5), "held frame overwritten");
    require(first->image.at<cv::Vec3b>(0, 0)[0] == 2, "leased pixels overwritten");
    first.reset();
    publish(pool, 6);
    auto fourth = pool.read(0ms);
    auto stats = pool.stats();
    require(stats.captured == 6 && stats.delivered == 4 && stats.pool_dropped == 1 &&
            stats.consumer_skipped == 1 && stats.read_timeouts == 2, "counter accounting mismatch");
    second.reset();
    third.reset();
    fourth.reset();
    publish(pool, 7);
    pool.invalidate();
    require(!pool.read(0ms), "disconnected camera exposed an old frame");
    pool.configure({8, 6});
    publish(pool, 8);
    auto after_reconnect = pool.read(0ms);
    require(after_reconnect->sequence == 8, "sequence restarted on reconnect");
    after_reconnect.reset();

    // 长循环的像素地址只能来自固定槽位，无逐帧像素分配。
    std::set<const unsigned char *> addresses;
    for (int i = 0; i < 10000; ++i) {
      publish(pool, static_cast<unsigned char>(i));
      auto frame = pool.read(0ms);
      addresses.insert(frame->image.data);
    }
    require(addresses.size() <= 3, "pixel buffers grew beyond pool capacity");
    auto waiter = std::async(std::launch::async, [&] { return pool.read(5s); });
    pool.stop();
    require(waiter.wait_for(500ms) == std::future_status::ready && !waiter.get(), "stop did not wake read");
    require(!pool.acquire(std::chrono::steady_clock::now(), 0), "stopped pool accepted a writer");

    io::CameraFrame survivor;
    {
      io::LatestFrameBuffer local(2);
      local.configure({8, 6});
      publish(local, 17);
      survivor = local.read(0ms);
      local.stop();
    }
    require(survivor->image.at<cv::Vec3b>(0, 0)[0] == 17, "frame did not outlive pool");

    // 两线程同时读写，检查整帧一致性及最终丢帧分类，覆盖实际生产/消费竞争。
    io::LatestFrameBuffer concurrent(3);
    concurrent.configure({64, 48});
    std::atomic<bool> done{false};
    auto producer = std::thread([&] {
      for (unsigned i = 0; i < 10000; ++i) {
        auto frame = concurrent.acquire(std::chrono::steady_clock::now(), i);
        if (!frame) continue;
        const auto value = static_cast<unsigned char>(frame->sequence);
        frame->image.setTo(cv::Scalar(value, value, value));
        concurrent.publish(std::move(frame));
      }
      done = true;
    });
    bool coherent = true;
    std::uint64_t last_sequence = 0, received = 0;
    while (true) {
      const auto frame = concurrent.read(10ms);
      if (!frame) { if (done) break; else continue; }
      ++received;
      coherent &= frame->sequence > last_sequence;
      last_sequence = frame->sequence;
      const auto value = static_cast<unsigned char>(frame->sequence);
      for (int row = 0; row < frame->image.rows; ++row)
        for (int col = 0; col < frame->image.cols; ++col)
          coherent &= frame->image.at<cv::Vec3b>(row, col) == cv::Vec3b(value, value, value);
      std::this_thread::yield();
    }
    producer.join();
    const auto concurrent_stats = concurrent.stats();
    require(coherent && received > 0 && concurrent_stats.captured == 10000 &&
            concurrent_stats.delivered == received && concurrent_stats.captured ==
            concurrent_stats.delivered + concurrent_stats.consumer_skipped + concurrent_stats.pool_dropped,
            "concurrent frame pixels, sequence or loss accounting inconsistent");
    std::cout << "latest-frame ownership, counters, capacity and shutdown passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
