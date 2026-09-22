#ifndef AUTO_AIM__SZU_INT16_MODEL_HPP
#define AUTO_AIM__SZU_INT16_MODEL_HPP

#include "hailo_decoder.hpp"

namespace auto_aim
{
// [9.9-3][Pi接入] 仅对应 szu_int16_head_l.hef；来源及指纹见 docs/9.9-pi-integration.md。
// 将 HailoOutput 的反量化 raw head 接到现有 Decoder，不引入青大跟踪/解算框架。
class SzuInt16Model
{
public:
  explicit SzuInt16Model(HailoDecoderOptions options = {});
  static HailoDecoderContract contract();

  // raw: stride -> y -> x -> anchor；返回: stride -> anchor -> y -> x，角点为输入像素。
  // 返回缓冲由本实例拥有，下次调用覆盖；此接口也供离线核验模型 tail。
  const cv::Mat & decode_heads(const cv::Mat & raw);
  std::list<Armor> decode(const cv::Mat & raw, cv::Size image_size);

private:
  cv::Mat predictions_;
  HailoDecoder decoder_;
};
}  // namespace auto_aim
#endif
