#ifndef AUTO_AIM__HAILO_DECODER_HPP
#define AUTO_AIM__HAILO_DECODER_HPP

#include <array>
#include <list>
#include <vector>

#include "tasks/auto_aim/armor.hpp"

namespace auto_aim
{
// [9.9-3] 模型语义必须由调用方明确提供；shape 正确不能证明类别、角点编码正确。
enum class HailoScoreEncoding { logits, probabilities };

struct HailoDecoderContract
{
  cv::Size input_size{640, 512};
  // 仅接受已经解码到网络输入像素的角点；未核对的 raw grid/anchor 输出不能直接使用。
  bool corners_are_input_pixels = false;
  HailoScoreEncoding scores = HailoScoreEncoding::logits;
  std::array<Color, 4> colors{};       // 输出 9..12 对应的 SP25 颜色，必须显式填写。
  std::array<ArmorName, 9> names{};    // 输出 13..21 对应的 SP25 编号，必须显式填写。
  std::array<int, 4> corner_indices{};  // SP25 左上/右上/右下/左下各取原始哪个点。
  // [9.9-3][Pi接入] 参考 INT16 的原始颜色 2/3 为辅助类别，不能送入 Tracker。
  std::array<bool, 4> reject_colors{};
};

// [9.9-3] 阈值集中于此；这些是离线开发初值，需用目标 HEF 的真实画面校准。
struct HailoDecoderOptions
{
  float min_confidence = 0.8F;  // 延续 SP25：confidence 只使用 objectness，不乘类别分数。
  float nms_iou = 0.5F;
  float min_quad_area = 4.0F;  // 原图像素平方。
  float duplicate_center_ratio = 0.15F;  // 中心距离 / 两框中较小的灯条平均长度；0 关闭。
  float big_armor_ratio = 3.2F;  // 3/4/5/基地的大小启发式阈值；使用原图角点几何。
  int max_candidates = 256;
  int max_detections = 128;
};

class HailoDecoder
{
public:
  // [9.9-3] 无默认模型映射，构造时验证约定；本模块不依赖 HailoRT/NPU。
  explicit HailoDecoder(
    HailoDecoderContract contract, HailoDecoderOptions options = {});

  // [Pi接入] 接收已反量化且完成模型 tail 的 N×22 FLOAT32；支持整图直接 resize 的逆映射。
  // 返回独立拥有数据的 Armor；内部候选缓冲复用，同一实例限一个处理线程。
  std::list<Armor> decode(const cv::Mat & predictions, cv::Size image_size);

private:
  struct Candidate
  {
    std::array<cv::Point2f, 4> points;
    cv::Rect2f box;
    cv::Point2f center;
    Color color;
    ArmorName name;
    ArmorType type;
    float confidence;
    float lightbar_length;
    int row;
  };

  static bool better(const Candidate & a, const Candidate & b);
  HailoDecoderContract contract_;
  HailoDecoderOptions options_;
  int expected_rows_;
  std::vector<Candidate> candidates_;
  std::vector<std::size_t> kept_;
};
}  // namespace auto_aim

#endif  // AUTO_AIM__HAILO_DECODER_HPP
