#include "yolo.hpp"

#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

#include "hailo/hailo_runtime.hpp"
#include "hailo/szu_int16_model.hpp"
#include "lightbar_refiner.hpp"  // [Pi灯条] 网络定位之后，按真实灯条修正并判重。

namespace auto_aim
{
namespace
{
struct Settings
{
  std::string hef_path;
  HailoDecoderOptions decoder;
  LightbarRefineOptions refiner;  // [Pi灯条] 未显式启用时保持原Hailo检测行为。
};

Settings load_settings(const std::string & path)
{
  const auto config = YAML::LoadFile(path);
  // [9.9-2][Pi接入] 模型语义显式选择，拒绝误用其它 shape 相同的 HEF 或旧 ROI 配置。
  if (!config["hailo_model"] || config["hailo_model"].as<std::string>() != "szu-int16-head-l") {
    throw std::invalid_argument("Set hailo_model: szu-int16-head-l (see configs/pi_hailo.yaml)");
  }
  if (config["use_roi"] && config["use_roi"].as<bool>()) {
    throw std::invalid_argument("Hailo detection uses full-frame resize; set use_roi: false");
  }
  if (!config["hef_path"] || config["hef_path"].as<std::string>().empty()) {
    throw std::invalid_argument("Set hef_path to the verified szu_int16_head_l.hef");
  }
  std::filesystem::path hef(config["hef_path"].as<std::string>());
  // 相对模型路径以 YAML 所在目录为准，不依赖启动程序时的工作目录。
  if (hef.is_relative()) hef = std::filesystem::absolute(path).parent_path() / hef;
  Settings result;
  result.hef_path = hef.lexically_normal().string();
  if (config["min_confidence"]) result.decoder.min_confidence = config["min_confidence"].as<float>();
  if (config["nms_iou"]) result.decoder.nms_iou = config["nms_iou"].as<float>();
  // [Pi灯条] 独立开关，保留基线以同图对照；修正版暂不同时改变曝光/NMS/模型。
  if (const auto refinement = config["lightbar_refine"]) {
    if (!refinement.IsMap()) throw std::invalid_argument("lightbar_refine must be a mapping");
    if (refinement["enabled"]) result.refiner.enabled = refinement["enabled"].as<bool>();
    if (refinement["threshold"]) result.refiner.threshold = refinement["threshold"].as<int>();
    if (refinement["min_length"]) result.refiner.min_length = refinement["min_length"].as<double>();
    if (refinement["max_corner_shift_ratio"])
      result.refiner.max_corner_shift_ratio = refinement["max_corner_shift_ratio"].as<double>();
  }
  result.refiner.big_armor_ratio = result.decoder.big_armor_ratio;
  return result;
}
}  // namespace

struct YOLO::Impl
{
  // [9.9-2][9.9-3][Pi接入] 阈值先校验，再加载一次 HEF；尺寸不变时 resize/cvtColor 复用内存。
  SzuInt16Model model;
  LightbarRefiner refiner;  // [Pi灯条] 成员顺序保证参数错误在NPU加载之前报告。
  HailoRuntime runtime;
  cv::Mat resized, rgb, raw;

  explicit Impl(const Settings & settings)
  : model(settings.decoder), refiner(settings.refiner), runtime(settings.hef_path)
  {
    const std::string prefix = "szu_fp32_conv45_46_47_a16/";
    const char * names[] = {"conv47", "conv54", "conv60"};
    const auto & heads = runtime.output_info();
    if (runtime.input_size() != cv::Size(640, 512) ||
        runtime.input_name() != prefix + "input_layer1" || heads.size() != 3) {
      throw std::invalid_argument("HEF input signature does not match szu-int16-head-l");
    }
    for (std::size_t i = 0; i < heads.size(); ++i) {
      const int stride = 8 << i;
      const auto type = i == 0 ? HailoElementType::uint16 : HailoElementType::uint8;
      if (heads[i].name != prefix + names[i] || heads[i].type != type ||
          heads[i].grid_size != cv::Size(640 / stride, 512 / stride) || heads[i].features != 66) {
        throw std::invalid_argument("HEF output signature does not match szu-int16-head-l");
      }
    }
  }
};

YOLO::YOLO(const std::string & config_path, bool debug)
: impl_(std::make_unique<Impl>(load_settings(config_path)))
{
  // [9.9-2][Pi接入] 保留已有 debug 参数，仅输出启动信息；画框由调用方负责，无逐帧窗口/写盘。
  if (debug) std::clog << "YOLO: HailoRT szu-int16-head-l, RGB UINT8 640x512, full-frame resize\n";
  // [Pi灯条] 明确运行的是基线还是严格灯条修正版，避免把无匹配导致的空输出误认成未加载模型。
  if (debug && impl_->refiner.enabled())
    std::clog << "YOLO: lightbar_refine=on; require unambiguous same-color light pair; "
                 "rebuild corners/type; merge identical light pairs\n";
}

YOLO::~YOLO() = default;

std::list<Armor> YOLO::detect(const cv::Mat & img, int /* frame_count */)
{
  // [9.9-3][Pi接入] 对齐青大 BuildNetworkInput：直接拉伸、BGR -> RGB，不除以 255、不填边。
  if (img.empty() || img.dims != 2 || img.type() != CV_8UC3) {
    throw std::invalid_argument("YOLO::detect requires a nonempty CV_8UC3 BGR image");
  }
  cv::resize(img, impl_->resized, {640, 512}, 0, 0, cv::INTER_LINEAR);
  cv::cvtColor(impl_->resized, impl_->rgb, cv::COLOR_BGR2RGB);
  impl_->runtime.infer(impl_->rgb, impl_->raw);
  // runtime 失败会抛出并清空输出；不吞掉故障或把旧帧当成新结果。
  // [Pi灯条] 沿用现有解码/NMS初筛，再用同一原图修正；不开启时原样返回候选。
  return impl_->refiner.refine(img, impl_->model.decode(impl_->raw, img.size()));
}
}  // namespace auto_aim
