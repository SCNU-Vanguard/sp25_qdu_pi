#ifndef AUTO_BUFF__YOLO11_BUFF_HPP
#define AUTO_BUFF__YOLO11_BUFF_HPP
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace auto_buff
{
// [9.9-1] 保留打符检测结果及调用接口，删除 OpenVINO 状态和实现。
// 仅有声明；打符推理需独立模型适配，当前不构建检测程序。
class YOLO11_BUFF
{
public:
  struct Object
  {
    cv::Rect_<float> rect;
    int label;
    float prob;
    std::vector<cv::Point2f> kpt;
  };

  YOLO11_BUFF(const std::string & config);

  // 使用NMS，用来获取多个框
  std::vector<Object> get_multicandidateboxes(cv::Mat & image);

  // 寻找置信度最高的框
  std::vector<Object> get_onecandidatebox(cv::Mat & image);
};
}  // namespace auto_buff
#endif
