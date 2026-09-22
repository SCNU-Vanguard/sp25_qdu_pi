#ifndef AUTO_AIM__HAILO_OUTPUT_HPP
#define AUTO_AIM__HAILO_OUTPUT_HPP

#include <cstddef>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace auto_aim
{
// [9.9-2] 仅描述主机侧原始输出；不依赖 Hailo SDK，便于离线验证字节布局和反量化。
enum class HailoElementType { uint8, uint16, float32 };

struct HailoQuantization
{
  float zero_point;
  float scale;
};

struct HailoHeadInfo
{
  std::string name;
  cv::Size grid_size;
  int features;
  HailoElementType type;
  std::vector<HailoQuantization> quantization;
  int stride = 0;
  int first_row = 0;
};

// [9.9-2] 本次候选模型的显式结构约定：每个网格 3 个候选，每个候选 22 个值。
// 结构校验不证明通道语义正确；颜色、编号、角点及 anchor 的含义在第三部分核对。
class HailoOutput
{
public:
  static constexpr int values_per_candidate = 22;
  static constexpr int anchors_per_cell = 3;
  static constexpr int features_per_head = values_per_candidate * anchors_per_cell;

  HailoOutput(cv::Size input_size, std::vector<HailoHeadInfo> heads);
  HailoOutput(const HailoOutput &) = delete;
  HailoOutput & operator=(const HailoOutput &) = delete;

  const std::vector<HailoHeadInfo> & heads() const { return heads_; }
  void * data(std::size_t head) { return buffers_.at(head).data; }
  std::size_t frame_bytes(std::size_t head) const;

  // [9.9-2] 结果顺序：stride 8/16/32 -> y -> x -> anchor -> 22 个原始值。
  // 返回对象和数据由本对象持有，下次 fuse() 会覆盖；不执行 sigmoid、坐标解码或 NMS。
  const cv::Mat & fuse();

private:
  std::vector<HailoHeadInfo> heads_;
  std::vector<cv::Mat> buffers_;
  cv::Mat fused_;
};

}  // namespace auto_aim
#endif  // AUTO_AIM__HAILO_OUTPUT_HPP
