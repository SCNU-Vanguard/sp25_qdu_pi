// [9.9-2][9.9-3][Pi接入] 仅替身替换 NPU I/O，其余使用生产 YOLO/模型/Decoder/Armor。
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/auto_aim/hailo/hailo_runtime.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace
{
int loads = 0, calls = 0;
bool fail = false, bad_signature = false;
cv::Vec3b expected_rgb(30, 20, 10);
std::string expected_path;
void require(bool ok, const char * message)
{
  if (!ok) throw std::runtime_error(message);
}
template <typename F> void reject(F operation)
{
  try { operation(); } catch (const std::exception &) { return; }
  throw std::runtime_error("Invalid input/configuration or inference error was swallowed");
}
}  // namespace

namespace auto_aim
{
struct HailoRuntime::Impl
{
  cv::Mat raw{20160, 22, CV_32FC1, cv::Scalar(-100)};
  std::string input = "szu_fp32_conv45_46_47_a16/input_layer1";
  std::vector<HailoHeadInfo> heads{
    {"szu_fp32_conv45_46_47_a16/conv47", {80, 64}, 66, HailoElementType::uint16, {}},
    {"szu_fp32_conv45_46_47_a16/conv54", {40, 32}, 66, HailoElementType::uint8, {}},
    {"szu_fp32_conv45_46_47_a16/conv60", {20, 16}, 66, HailoElementType::uint8, {}}};
};
HailoRuntime::HailoRuntime(const std::string & path, const HailoRuntimeOptions &)
: impl_(std::make_unique<Impl>())
{
  ++loads;
  require(path == expected_path, "Relative HEF path must resolve against YAML directory");
  if (bad_signature) impl_->heads[0].name = "another_model/conv47";
}
HailoRuntime::~HailoRuntime() = default;
cv::Size HailoRuntime::input_size() const { return {640, 512}; }
const std::string & HailoRuntime::input_name() const { return impl_->input; }
const std::vector<HailoHeadInfo> & HailoRuntime::output_info() const { return impl_->heads; }
void HailoRuntime::infer(const cv::Mat & rgb, cv::Mat & output)
{
  ++calls;
  output.release();
  if (fail) throw std::runtime_error("Injected NPU failure");
  require(rgb.size() == cv::Size(640, 512) && rgb.type() == CV_8UC3 && rgb.isContinuous(),
    "Preprocessing did not produce packed RGB UINT8 640x512");
  for (const cv::Point & point : {cv::Point(0, 0), cv::Point(639, 511), cv::Point(320, 256)}) {
    require(rgb.at<cv::Vec3b>(point) == expected_rgb, "BGR/RGB, letterbox or normalization mismatch");
  }
  impl_->raw.setTo(-100);
  const float candidate[] = {0, 0, 0, 1, 2, 1, 2, 0, 2,
                            2, -10, -10, -10, -10, -10, -10, -10, -10, -10, -10, -10, 2};
  std::copy(candidate, candidate + 22, impl_->raw.ptr<float>(810 * 3 + 1));
  output = impl_->raw;
}
}  // namespace auto_aim

int main(int argc, char ** argv)
{
  try {
    require(argc == 2, "Pass the CMake binary directory for temporary YAML files");
    const auto directory = std::filesystem::absolute(argv[1]);
    const auto config = directory / "yolo_pipeline_fixture.yaml";
    expected_path = (directory / "fixture.hef").string();
    const auto write = [&](const std::string & extra) {
      std::ofstream stream(config);
      stream << "hailo_model: szu-int16-head-l\nhef_path: fixture.hef\n" << extra;
      require(static_cast<bool>(stream), "Cannot write temporary configuration");
    };
    write("use_roi: false\nmin_confidence: 0.8\n");
    auto_aim::YOLO yolo(config.string(), false);
    cv::Mat storage(540, 722, CV_8UC3, cv::Scalar(10, 20, 30));
    const auto bgr = storage.colRange(1, 721);
    require(!bgr.isContinuous(), "Fixture must exercise camera ROI stride");
    const auto held = yolo.detect(bgr);
    require(held.size() == 1 && held.front().name == auto_aim::base &&
      held.front().points[0] == cv::Point2f(90, 84.375F), "YOLO did not produce source-space Armor");
    storage.setTo(cv::Scalar(40, 50, 60));
    expected_rgb = {60, 50, 40};
    require(yolo.detect(bgr).size() == 1 && loads == 1, "Repeated frame reloaded HEF/lost result");
    const int before = calls;
    reject([&] { yolo.detect(cv::Mat()); });
    reject([&] { yolo.detect(cv::Mat(10, 10, CV_32FC3)); });
    require(calls == before, "Invalid image reached NPU");
    fail = true;
    reject([&] { yolo.detect(bgr); });
    fail = false;
    require(held.front().points[0] == cv::Point2f(90, 84.375F), "Old Armor aliases runtime memory");
    write("use_roi: true\n");
    reject([&] { auto_aim::YOLO invalid(config.string(), false); });
    write("min_confidence: 2\n");
    reject([&] { auto_aim::YOLO invalid(config.string(), false); });
    require(loads == 1, "Invalid settings were not rejected before loading NPU");
    // [Pi灯条] 检查新参数在加载设备前拒绝；真实YOLO链路必须执行修正而不只是测试独立函数。
    write("lightbar_refine: true\n");
    reject([&] { auto_aim::YOLO invalid(config.string(), false); });
    write("lightbar_refine:\n  enabled: true\n  threshold: 255\n");
    reject([&] { auto_aim::YOLO invalid(config.string(), false); });
    require(loads == 1, "Invalid refiner settings reached NPU");
    write("lightbar_refine:\n  enabled: true\n  threshold: 150\n  min_length: 6\n");
    auto_aim::YOLO refined(config.string(), false);
    cv::rectangle(storage, {90+1-2,84}, {90+1+2,116}, {255,220,40}, -1);
    cv::rectangle(storage, {126+1-2,84}, {126+1+2,116}, {255,220,40}, -1);
    const auto refined_result = refined.detect(bgr);
    require(refined_result.size() == 1 &&
      refined_result.front().points[0] == cv::Point2f(90,84) &&
      refined_result.front().left.length == 32 && refined_result.front().right.length == 32,
      "YOLO entry did not use same-frame physical lightbar correction");
    storage.setTo(cv::Scalar(40,50,60));
    const auto fallback_result = refined.detect(bgr);
    require(fallback_result.size() == 1 && fallback_result.front().points[0] == cv::Point2f(90,84.375F),
      "YOLO did not retain the raw network candidate when lightbar refinement failed");
    write("");
    bad_signature = true;
    reject([&] { auto_aim::YOLO invalid(config.string(), false); });
    std::filesystem::remove(config);
    std::cout << "PASS: production YOLO resize/RGB, config, single load, Armor and error propagation (mock NPU)\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}
