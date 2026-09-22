#ifndef OMNIPERCEPTION__PERCEPTRON_HPP
#define OMNIPERCEPTION__PERCEPTRON_HPP

#include <chrono>
#include <condition_variable>
#include <list>
#include <memory>
#include <mutex>
#include <thread>

#include "decider.hpp"
#include "detection.hpp"
#include "io/usbcamera/usbcamera.hpp"
#include "tasks/auto_aim/armor.hpp"
// [9.9-1] 移除旧推理线程池头文件；本模块直接使用标准线程和检测入口。
#include "tasks/auto_aim/yolo.hpp"
#include "tools/thread_safe_queue.hpp"

namespace omniperception
{

class Perceptron
{
public:
  Perceptron(
    io::USBCamera * usbcma1, io::USBCamera * usbcam2, io::USBCamera * usbcam3,
    io::USBCamera * usbcam4, const std::string & config_path);

  ~Perceptron();

  std::vector<DetectionResult> get_detection_queue();

  void parallel_infer(io::USBCamera * cam, std::shared_ptr<auto_aim::YOLO> & yolo_parallel);

private:
  std::vector<std::thread> threads_;
  tools::ThreadSafeQueue<DetectionResult> detection_queue_;

  std::shared_ptr<auto_aim::YOLO> yolo_parallel1_;
  std::shared_ptr<auto_aim::YOLO> yolo_parallel2_;
  std::shared_ptr<auto_aim::YOLO> yolo_parallel3_;
  std::shared_ptr<auto_aim::YOLO> yolo_parallel4_;

  Decider decider_;
  bool stop_flag_;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
};

}  // namespace omniperception
#endif
