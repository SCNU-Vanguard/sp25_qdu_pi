#ifndef AUTO_AIM__YOLO_HPP
#define AUTO_AIM__YOLO_HPP

#include <list>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

#include "armor.hpp"

namespace auto_aim
{
// [9.9-1] 保留 SP25 检测入口；移除多后端基类和旧推理队列专用 postprocess。
// [9.9-2][9.9-3][Pi接入] 单路 Hailo 推理，复用缓冲并输出 SP25 Armor；同一实例限一个线程。
class YOLO
{
public:
  YOLO(const std::string & config_path, bool debug = true);
  ~YOLO();
  YOLO(const YOLO &) = delete;
  YOLO & operator=(const YOLO &) = delete;

  std::list<Armor> detect(const cv::Mat & img, int frame_count = -1);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__YOLO_HPP
