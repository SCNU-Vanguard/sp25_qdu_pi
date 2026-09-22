#ifndef IO__HIKROBOT_HPP
#define IO__HIKROBOT_HPP

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <yaml-cpp/yaml.h>

#include "io/camera.hpp"
#include "tools/logger.hpp"

namespace io
{
// [9.9-6] 节点集中校验；分辨率来自相机回读，不能用 resize 掩盖配置错误。
struct HikRobotOptions
{
  double exposure_ms = 3.0;
  double gain = 10.0;
  bool rotate_180 = false;  // [Pi方向] BGR发布前软件旋转180度；缺省关闭，显示与检测共用同一方向。
  std::string serial_number;
  bool external_trigger = false;
  std::string trigger_source = "Line0";
  std::string trigger_activation = "RisingEdge";
  double acquisition_frame_rate = 100.0;
  int decimation_horizontal = 1;
  int decimation_vertical = 1;
  int expected_width = 0;
  int expected_height = 0;
  int pool_size = 4;
  int grab_timeout_ms = 100;
  int read_timeout_ms = 250;
  int reconnect_delay_ms = 1000;
  int reconnect_after_timeouts = 10;

  static HikRobotOptions from_yaml(const YAML::Node & yaml);
  void validate() const;
};

class HikRobot : public CameraBase
{
public:
  explicit HikRobot(HikRobotOptions options);
  ~HikRobot() override;
  // [9.9-5] 旧接口返回独立像素副本；高帧率路径使用 read_frame() 持有槽位。
  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;
  CameraFrame read_frame() override;
  CameraStats stats() const override;

private:
  HikRobotOptions options_;
  LatestFrameBuffer frames_;
  std::shared_ptr<spdlog::logger> logger_;
  std::atomic<bool> quit_{false};
  std::mutex retry_mutex_;
  std::condition_variable retry_;
  // [9.9-6] 只有这个线程拥有 SDK 句柄；析构先唤醒读者，再停止并 join。
  std::thread worker_;
  void run();
};
}  // namespace io
#endif
