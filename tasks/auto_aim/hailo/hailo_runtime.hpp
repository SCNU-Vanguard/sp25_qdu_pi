#ifndef AUTO_AIM__HAILO_RUNTIME_HPP
#define AUTO_AIM__HAILO_RUNTIME_HPP

#include <cstdint>
#include <memory>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

#include "hailo_output.hpp"

namespace auto_aim
{
// [9.9-2] 640x512 是 9.9 方案的待验证约定，启动时必须与实际 HEF 输入匹配。
struct HailoRuntimeOptions
{
  cv::Size expected_input_size{640, 512};
  std::uint32_t timeout_ms = 1000;
  bool float32_output = false;  // 默认保留原生 UINT8/UINT16；true 时交给 SDK 反量化。
};

class HailoRuntime
{
public:
  explicit HailoRuntime(
    const std::string & hef_path, const HailoRuntimeOptions & options = {});
  ~HailoRuntime();
  HailoRuntime(const HailoRuntime &) = delete;
  HailoRuntime & operator=(const HailoRuntime &) = delete;

  cv::Size input_size() const;
  const std::string & input_name() const;
  const std::vector<HailoHeadInfo> & output_info() const;

  // [9.9-2] 同步单帧接口：输入必须是连续、尺寸匹配的 CV_8UC3 RGB 原始像素。
  // 前处理在调用侧完成；同一实例由一个处理线程使用，不支持并发 infer。
  // output 为运行时缓冲区的共享视图，下次 infer 会覆盖；需要跨帧保存时由调用方 clone。
  // 失败时抛出异常并清空 output；SDK 推理失败后必须重建实例，不能继续读取旧帧。
  void infer(const cv::Mat & rgb, cv::Mat & output);

private:
  // [9.9-2] 隔离 Hailo SDK 类型，SP25 上层头文件不再携带推理运行库依赖。
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace auto_aim
#endif  // AUTO_AIM__HAILO_RUNTIME_HPP
