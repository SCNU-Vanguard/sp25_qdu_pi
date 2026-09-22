#include "szu_int16_model.hpp"

#include <stdexcept>

namespace auto_aim
{
namespace
{
// [Pi-OpenCV4.10] 4.10 在 ARM NEON 下不提供 float16_t 旧名；兼容开发机 4.5 与 Pi 4.10。
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 10)
using ModelFloat16 = cv::hfloat;
#else
using ModelFloat16 = cv::float16_t;
#endif
}  // namespace

HailoDecoderContract SzuInt16Model::contract()
{
  // [9.9-3][Pi接入] 核对 ModelAdapter 的实际函数：false 分支执行 sigmoid，而非直接用概率。
  HailoDecoderContract result;
  result.corners_are_input_pixels = true;
  result.scores = HailoScoreEncoding::logits;
  result.colors = {blue, red, extinguish, purple};  // 后两个仅占位，在输出 Armor 前过滤。
  result.reject_colors = {false, false, true, true};
  result.names = {sentry, one, two, three, four, five, outpost, base, base};
  result.corner_indices = {0, 3, 2, 1};  // 原始 LT/LB/RB/RT -> SP25 LT/RT/RB/LB。
  return result;
}

SzuInt16Model::SzuInt16Model(HailoDecoderOptions options)
: predictions_(20160, 22, CV_32FC1), decoder_(contract(), options)
{
}

const cv::Mat & SzuInt16Model::decode_heads(const cv::Mat & raw)
{
  if (raw.dims != 2 || raw.type() != CV_32FC1 || raw.rows != 20160 || raw.cols != 22 ||
      raw.datastart == predictions_.datastart) {
    throw std::invalid_argument("szu-int16-head-l requires separate raw 20160 x 22 FLOAT32 heads");
  }
  // [9.9-3][Pi接入] 青大 Network::FuseOneHailoOutput 的三组 anchor；不能把 raw 点当像素。
  constexpr float anchors[3][3][2] = {
    {{10, 13}, {16, 30}, {33, 23}},
    {{30, 61}, {62, 45}, {59, 119}},
    {{116, 90}, {156, 198}, {373, 326}}};
  int first = 0;
  for (int head = 0; head < 3; ++head) {
    const int stride = 8 << head;
    const int width = 640 / stride, height = 512 / stride;
    const int cells = width * height;
    for (int anchor = 0; anchor < 3; ++anchor) {
      for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
          const int cell = y * width + x;
          const auto * source = raw.ptr<float>(first + cell * 3 + anchor);
          auto * target = predictions_.ptr<float>(first + anchor * cells + cell);
          // [9.9-3][Pi接入] 模拟参考源码在 ARM 上的 FP16 舍入，再以 FLOAT32 算坐标。
          // [Pi-OpenCV4.10] 两个版本的半精度类型都保留 FP16 舍入，不省略为 FLOAT32 直拷贝。
          for (int field = 0; field < 22; ++field) {
            target[field] = static_cast<float>(ModelFloat16(source[field]));
          }
          for (int point = 0; point < 4; ++point) {
            target[point * 2] = target[point * 2] * anchors[head][anchor][0] + x * stride;
            target[point * 2 + 1] = target[point * 2 + 1] * anchors[head][anchor][1] + y * stride;
          }
        }
      }
    }
    first += cells * 3;
  }
  return predictions_;
}

std::list<Armor> SzuInt16Model::decode(const cv::Mat & raw, cv::Size image_size)
{
  return decoder_.decode(decode_heads(raw), image_size);
}
}  // namespace auto_aim
