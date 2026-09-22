#include "hailo_decoder.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>

#include "hailo_output.hpp"

namespace auto_aim
{
namespace
{
// [9.9-3] 分支 sigmoid 避免极端负 logit 的 exp 溢出；类别 argmax 不必逐项 sigmoid。
float sigmoid(float value)
{
  if (value >= 0) return 1.0F / (1.0F + std::exp(-value));
  const float e = std::exp(value);
  return e / (1.0F + e);
}

bool probability(float value)
{
  return std::isfinite(value) && value >= 0.0F && value <= 1.0F;
}

template <typename T, std::size_t N>
bool permutation(const std::array<T, N> & values)
{
  std::array<bool, N> seen{};
  for (const auto value : values) {
    const int index = static_cast<int>(value);
    if (index < 0 || index >= static_cast<int>(N) || seen[index]) return false;
    seen[index] = true;
  }
  return true;
}

// [9.9-3] 保留模型的语义角点对应，不用坐标排序/凸包“修复”错误对应关系。
bool valid_quad(const std::array<cv::Point2f, 4> & points, float min_area)
{
  double twice_area = 0;
  for (int i = 0; i < 4; ++i) {
    const auto a = points[(i + 1) % 4] - points[i];
    const auto b = points[(i + 2) % 4] - points[(i + 1) % 4];
    const double cross = static_cast<double>(a.x) * b.y - static_cast<double>(a.y) * b.x;
    // 图像坐标向下为正，左上→右上→右下→左下必须为严格正向凸四边形。
    if (cross <= 1e-6 || cv::norm(a) < 1e-3) return false;
    twice_area += static_cast<double>(points[i].x) * points[(i + 1) % 4].y -
                  static_cast<double>(points[i].y) * points[(i + 1) % 4].x;
  }
  return twice_area >= 2.0 * min_area;
}

float iou(const cv::Rect2f & a, const cv::Rect2f & b)
{
  const float intersection = (a & b).area();
  return intersection / (a.area() + b.area() - intersection);
}
}  // namespace

HailoDecoder::HailoDecoder(HailoDecoderContract contract, HailoDecoderOptions options)
: contract_(std::move(contract)), options_(options)
{
  // [9.9-3] 与 HailoOutput 的三尺度/每网格三候选约定一致，错误配置在启动时拒绝。
  const auto size = contract_.input_size;
  // [9.9-3][Pi接入] 允许多个原始类别映射到同一语义（7/8 都是基地），仍拒绝越界枚举。
  const bool valid_names = std::all_of(contract_.names.begin(), contract_.names.end(),
    [](ArmorName name) { return name >= ArmorName::one && name <= ArmorName::not_armor; });
  if (size.width <= 0 || size.height <= 0 || size.width % 32 || size.height % 32 ||
      !contract_.corners_are_input_pixels || !permutation(contract_.colors) ||
      !valid_names || !permutation(contract_.corner_indices) ||
      (contract_.scores != HailoScoreEncoding::logits &&
       contract_.scores != HailoScoreEncoding::probabilities)) {
    throw std::invalid_argument(
      "HailoDecoder: supply explicit color/name/corner mappings and confirmed input-pixel corners; "
      "raw grid/anchor decoding is not implemented without the matching model contract");
  }
  if (!probability(options_.min_confidence) || !probability(options_.nms_iou) ||
      !std::isfinite(options_.min_quad_area) || options_.min_quad_area <= 0 ||
      !probability(options_.duplicate_center_ratio) ||
      !std::isfinite(options_.big_armor_ratio) || options_.big_armor_ratio <= 0 ||
      options_.max_candidates <= 0 || options_.max_detections <= 0 ||
      options_.max_detections > options_.max_candidates) {
    throw std::invalid_argument("HailoDecoder: invalid thresholds or candidate limits");
  }
  std::int64_t rows = 0;
  for (const int stride : {8, 16, 32}) {
    rows += static_cast<std::int64_t>(size.width / stride) * (size.height / stride) *
            HailoOutput::anchors_per_cell;
  }
  if (rows > std::numeric_limits<int>::max()) {
    throw std::invalid_argument("HailoDecoder: model output is too large");
  }
  expected_rows_ = static_cast<int>(rows);
  options_.max_candidates = std::min(options_.max_candidates, expected_rows_);
  candidates_.reserve(options_.max_candidates);
  kept_.reserve(std::min(options_.max_detections, options_.max_candidates));
}

bool HailoDecoder::better(const Candidate & a, const Candidate & b)
{
  // [9.9-3] 同分按融合行号确定顺序，避免同一输入出现不稳定的取舍。
  return a.confidence != b.confidence ? a.confidence > b.confidence : a.row < b.row;
}

std::list<Armor> HailoDecoder::decode(const cv::Mat & predictions, cv::Size image_size)
{
  candidates_.clear();
  kept_.clear();
  if (image_size.width <= 0 || image_size.height <= 0 || predictions.dims != 2 ||
      predictions.type() != CV_32FC1 || predictions.rows != expected_rows_ ||
      predictions.cols != HailoOutput::values_per_candidate) {
    throw std::invalid_argument("HailoDecoder: expected model-sized N x 22 FLOAT32 and a valid image");
  }
  const double sx = static_cast<double>(image_size.width) / contract_.input_size.width;
  const double sy = static_cast<double>(image_size.height) / contract_.input_size.height;
  for (int row = 0; row < predictions.rows; ++row) {
    const float * values = predictions.ptr<float>(row);  // 支持有行步长的 Mat，不复制全张量。
    if (!std::isfinite(values[8])) continue;
    const float confidence = contract_.scores == HailoScoreEncoding::logits
                               ? sigmoid(values[8]) : values[8];
    if (!probability(confidence) || confidence < options_.min_confidence) continue;

    // [9.9-3] 先筛 objectness，再检查类别；无效行单独丢弃，不产生 NaN 坐标/错误枚举。
    bool valid = true;
    for (int i = 9; i < 22; ++i) {
      valid &= std::isfinite(values[i]) &&
               (contract_.scores == HailoScoreEncoding::logits || probability(values[i]));
    }
    if (!valid) continue;
    const int color_id = static_cast<int>(std::max_element(values + 9, values + 13) - values - 9);
    if (contract_.reject_colors[color_id]) continue;  // [9.9-3][Pi接入] 按原始 ID 过滤辅助颜色。
    const int name_id = static_cast<int>(std::max_element(values + 13, values + 22) - values - 13);
    const auto name = contract_.names[name_id];
    if (name == ArmorName::not_armor) continue;

    Candidate candidate{};
    candidate.color = contract_.colors[color_id];
    candidate.name = name;
    candidate.confidence = confidence;
    candidate.row = row;
    for (int i = 0; i < 4; ++i) {
      const int source = 2 * contract_.corner_indices[i];
      const double x = values[source] * sx;
      const double y = values[source + 1] * sy;
      // 边缘不完整装甲直接拒绝；不通过夹紧角点来改变供 PnP 使用的几何。
      if (!std::isfinite(x) || !std::isfinite(y) || x < 0 || y < 0 ||
          x > image_size.width - 1 || y > image_size.height - 1) {
        valid = false;
        break;
      }
      candidate.points[i] = {static_cast<float>(x), static_cast<float>(y)};
      candidate.center += candidate.points[i] * 0.25F;
    }
    if (!valid || !valid_quad(candidate.points, options_.min_quad_area)) continue;

    const auto & p = candidate.points;
    const float left = static_cast<float>(cv::norm(p[3] - p[0]));
    const float right = static_cast<float>(cv::norm(p[2] - p[1]));
    candidate.lightbar_length = (left + right) * 0.5F;
    const double ratio = std::max(cv::norm(p[1] - p[0]), cv::norm(p[2] - p[3])) /
                         std::max(left, right);
    // [9.9-3] 英雄大板，工程/哨兵/前哨小板；其余依原图宽高比区分，阈值需实图校准。
    candidate.type = name == ArmorName::one ? ArmorType::big : ArmorType::small;
    if (name == ArmorName::three || name == ArmorName::four || name == ArmorName::five ||
        name == ArmorName::base) {
      candidate.type = ratio >= options_.big_armor_ratio ? ArmorType::big : ArmorType::small;
    }
    float min_x = p[0].x, max_x = p[0].x, min_y = p[0].y, max_y = p[0].y;
    for (const auto & point : p) {
      min_x = std::min(min_x, point.x);
      max_x = std::max(max_x, point.x);
      min_y = std::min(min_y, point.y);
      max_y = std::max(max_y, point.y);
    }
    candidate.box = {min_x, min_y, max_x - min_x, max_y - min_y};

    // [9.9-3] 有界堆保留最高分 K 个候选，最差候选在堆顶；不会简单截掉后面的检测头。
    if (candidates_.size() < static_cast<std::size_t>(options_.max_candidates)) {
      candidates_.push_back(candidate);
      std::push_heap(candidates_.begin(), candidates_.end(), better);
    } else if (better(candidate, candidates_.front())) {
      std::pop_heap(candidates_.begin(), candidates_.end(), better);
      candidates_.back() = candidate;
      std::push_heap(candidates_.begin(), candidates_.end(), better);
    }
  }
  std::sort(candidates_.begin(), candidates_.end(), better);

  // [9.9-3] 沿用 SP25 跨类别 bbox NMS；额外抑制同颜色/编号、中心很近的重复候选。
  std::list<Armor> armors;
  for (std::size_t i = 0; i < candidates_.size(); ++i) {
    const auto & candidate = candidates_[i];
    bool suppressed = false;
    for (const auto kept : kept_) {
      const auto & other = candidates_[kept];
      const bool duplicate = options_.duplicate_center_ratio > 0 &&
        candidate.color == other.color && candidate.name == other.name &&
        cv::norm(candidate.center - other.center) <= options_.duplicate_center_ratio *
          std::min(candidate.lightbar_length, other.lightbar_length);
      if (iou(candidate.box, other.box) > options_.nms_iou || duplicate) {
        suppressed = true;
        break;
      }
    }
    if (suppressed) continue;
    kept_.push_back(i);
    const auto & b = candidate.box;
    const int x = static_cast<int>(std::floor(b.x));
    const int y = static_cast<int>(std::floor(b.y));
    // [9.9-4] 从实际角点取右/下界；float 的 min+(max-min) 可能舍入到整数下方，漏包角点。
    const auto & p = candidate.points;
    const float max_x = std::max({p[0].x, p[1].x, p[2].x, p[3].x});
    const float max_y = std::max({p[0].y, p[1].y, p[2].y, p[3].y});
    const cv::Rect box(x, y, static_cast<int>(std::ceil(max_x)) - x,
      static_cast<int>(std::ceil(max_y)) - y);
    armors.emplace_back(candidate.color, candidate.name, candidate.type, candidate.confidence,
      box, std::vector<cv::Point2f>(candidate.points.begin(), candidate.points.end()), image_size);
    if (kept_.size() >= static_cast<std::size_t>(options_.max_detections)) break;
  }
  return armors;
}
}  // namespace auto_aim
