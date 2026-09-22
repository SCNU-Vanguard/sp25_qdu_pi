// [Pi主线C][Pi预览] 相机最新帧 -> 现有 YOLO/Armor；默认无窗口，可选浏览器画框，不连接控制输出。
#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <filesystem>  // [Pi实图] 少量同帧原图/结果留样，默认关闭。
#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/core.hpp>
#include <stdexcept>

#include "io/camera.hpp"
#ifdef SP_HIK_ONLY
#include "io/hikrobot/hikrobot.hpp"
#endif
#include "tasks/auto_aim/yolo.hpp"
#include "tests/live_preview.hpp"  // [Pi预览] 可选最新帧浏览器预览，不改变检测实现。
#include "tools/logger.hpp"

namespace
{
using Clock = std::chrono::steady_clock;
// [Pi主线C] 信号处理器只写 sig_atomic_t；设备清理留给主线程中的对象析构。
volatile std::sig_atomic_t interrupted = 0;
void on_interrupt(int) { interrupted = 1; }
double milliseconds(Clock::duration value)
{
  return std::chrono::duration<double, std::milli>(value).count();
}

// [Pi实图] 截图含缩放、JPEG和框，不能作为原始检测输入。仅显式启用时保存三组
// 同帧无损原图/画框图/角点；不改变YOLO，不录像，不跨帧持有相机槽位。
class LiveSamples
{
  std::filesystem::path directory_;
  Clock::time_point next_{};
  unsigned saved_ = 0;
  unsigned min_detections_ = 0;  // [Pi-field] Optional capture gate for frames containing a target.
  unsigned max_samples_ = 3;  // [Pi-dynamic-review] Configurable evidence count; default preserves prior behavior.
  std::chrono::milliseconds interval_{5000};  // [Pi-dynamic-review] Configurable cadence without changing detection.

public:
  LiveSamples(const std::string & directory, const std::string & camera_config,
              const std::string & detector_config, unsigned min_detections,
              unsigned max_samples, int interval_ms)
  : directory_(directory), min_detections_(min_detections), max_samples_(max_samples),
    interval_(interval_ms)
  {
    if (directory_.empty()) return;
    // [Pi实图] 只接受新目录，防止重复运行覆盖现场证据；同时固定本次使用的配置。
    if (!std::filesystem::create_directory(directory_))
      throw std::runtime_error("sample-dir must be a new directory: " + directory_.string());
    std::filesystem::copy_file(camera_config, directory_ / "camera-used.yaml");
    std::filesystem::copy_file(detector_config, directory_ / "detector-used.yaml");
  }

  bool enabled() const { return !directory_.empty(); }
  unsigned count() const { return saved_; }

  bool save(const cv::Mat & bgr, const std::list<auto_aim::Armor> & armors,
            std::uint64_t sequence, Clock::time_point now)
  {
    if (!enabled() || saved_ >= max_samples_) return false;  // [Pi-dynamic-review] Stop at the requested evidence limit.
    // [Pi实图] 第一张有效帧后留5秒摆板，之后至少间隔5秒；卡顿后不连续补存旧时点。
    if (next_ == Clock::time_point{}) next_ = now + interval_;  // [Pi-dynamic-review] First sample uses the requested delay.
    // [Pi-field] Wait for a detected target before spending one of the three sample slots.
    if (now < next_ || armors.size() < min_detections_) return false;
    const auto stem = directory_ / ("sample_" + std::to_string(saved_ + 1));
    if (!cv::imwrite(stem.string() + ".png", bgr) ||
        !cv::imwrite(stem.string() + "_overlay.png", live_preview::draw(bgr, armors)))
      throw std::runtime_error("Cannot save sample PNG: " + stem.string());
    std::ofstream text;
    text.exceptions(std::ios::badbit | std::ios::failbit);
    text.open(stem.string() + ".txt");
    text << std::fixed << std::setprecision(6)
         << "sequence=" << sequence << " size=" << bgr.cols << 'x' << bgr.rows
         << " detections=" << armors.size() << '\n'
         << "Original PNG is the BGR frame passed to YOLO, after configured camera rotation.\n"
         << "Coordinates: original image pixels, LT/RT/RB/LB; confidence: objectness.\n";
    for (const auto & armor : armors) {
      text << auto_aim::COLORS.at(armor.color) << ' ' << auto_aim::ARMOR_NAMES.at(armor.name)
           << ' ' << auto_aim::ARMOR_TYPES.at(armor.type) << " confidence=" << armor.confidence;
      for (const auto & point : armor.points) text << " [" << point.x << ',' << point.y << ']';
      text << '\n';
    }
    text.close();
    ++saved_;
    next_ = now + interval_;  // [Pi-dynamic-review] Space PNG writes so they do not continuously load the live path.
    return true;
  }
};
}  // namespace

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明 }"
  // [Pi主线C] 分开读取两份已验证的配置；保留可选位置参数作为旧合并配置入口。
  "{@config-path   |                        | 可选：同时用于相机和检测的合并配置 }"
  "{camera-config  |                        | 相机配置，默认configs/camera.yaml }"
  "{detector-config|                        | 检测配置，默认configs/pi_hailo.yaml }"
  "{seconds        | 10                    | 运行秒数，必须大于0 }"
  "{preview-port   | 0                     | 浏览器预览端口，0关闭，建议8080 }"  // [Pi预览]
  "{sample-dir     |                       | 可选新目录：每5秒存一组同帧原图/结果，最多3组 }";  // [Pi实图]

int main(int argc, char * argv[])
{
  // [Pi-field] Keep existing CLI defaults; add an opt-in gate for useful field samples.
  cv::CommandLineParser cli(argc, argv, keys +
    "{sample-min-detections | 0 | Save only frames with at least this many detections }"
    "{sample-max            | 3 | Maximum same-frame raw/overlay/text sample groups }"  // [Pi-dynamic-review]
    "{sample-interval-ms    | 5000 | Minimum milliseconds between sample groups }");  // [Pi-dynamic-review]
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto camera_config = cli.get<std::string>("camera-config");
  auto detector_config = cli.get<std::string>("detector-config");
  const auto combined = cli.get<std::string>(0);
  const auto seconds = cli.get<int>("seconds");
  const auto preview_port = cli.get<int>("preview-port");  // [Pi预览] 无参数时保持原无预览路径。
  const auto sample_dir = cli.get<std::string>("sample-dir");  // [Pi实图] 空值保持原来不存图。
  const int sample_min_detections = cli.get<int>("sample-min-detections");  // [Pi-field] 0 keeps old behavior.
  const int sample_max = cli.get<int>("sample-max");  // [Pi-dynamic-review] More samples are opt-in for field review.
  const int sample_interval_ms = cli.get<int>("sample-interval-ms");  // [Pi-dynamic-review] Sampling only; inference is unchanged.
  if (!cli.check() || seconds <= 0 || preview_port < 0 || preview_port > 65535 ||
      sample_min_detections < 0 || sample_min_detections > 128 ||
      sample_max < 1 || sample_max > 500 ||
      sample_interval_ms < 100 || sample_interval_ms > 60000 ||  // [Pi-dynamic-review] Bound disk-write rate and CLI mistakes.
      (!combined.empty() && (!camera_config.empty() || !detector_config.empty()))) {
    cli.printErrors();
    std::cerr << "Use positive --seconds, --preview-port=0..65535 and either two named configs or one combined config\n";
    return 1;
  }
  if (!combined.empty()) camera_config = detector_config = combined;
  if (camera_config.empty()) camera_config = "configs/camera.yaml";
  if (detector_config.empty()) detector_config = "configs/pi_hailo.yaml";

  // [Pi主线C] 只保存固定数量的累计值，不按运行时长积攒图片或逐帧计时样本。
  std::uint64_t processed = 0, with_detections = 0, total_detections = 0, last_sequence = 0;
  double detect_total_ms = 0, detect_max_ms = 0, age_total_ms = 0, age_max_ms = 0;
  try {
    std::signal(SIGINT, on_interrupt);
    // [Pi主线C] 在相机工作线程启动前创建 logger，避免两线程竞争首次初始化。
    const auto log = tools::logger();
    log->info("camera_config={}, detector_config={}, seconds={}", camera_config, detector_config, seconds);
    // [Pi实图] 路径/配置复制失败须在打开设备前报错；写盘开销不作性能基准。
    LiveSamples samples(sample_dir, camera_config, detector_config,
                        static_cast<unsigned>(sample_min_detections),
                        static_cast<unsigned>(sample_max), sample_interval_ms);  // [Pi-dynamic-review]
    if (samples.enabled())
      log->info("sample_dir={}; max_samples={}; interval_ms={}; min_detections={}; "
                "PNG writes included; not a FPS benchmark", sample_dir, sample_max,
                sample_interval_ms, sample_min_detections);  // [Pi-dynamic-review]
    // [Pi预览] 先绑定端口，端口占用时明确退出；不为预览额外打开相机或创建YOLO。
    std::unique_ptr<live_preview::Server> preview;
    if (preview_port) {
      preview = std::make_unique<live_preview::Server>(preview_port);
      log->info("preview=http://<Pi-IP>:{}/ ; max_refresh_fps=10; preview run is not a FPS benchmark", preview_port);
    }
    // [Pi主线C] 模型只创建一次，先加载模型再启动相机；以下计时不含模型加载，含相机启动和首帧推理。
    auto_aim::YOLO yolo(detector_config, true);
#ifdef SP_HIK_ONLY
    const auto yaml = YAML::LoadFile(camera_config);
    if (yaml["camera_name"].as<std::string>() != "hikrobot")
      throw std::invalid_argument("hailo_camera_test requires camera_name: hikrobot");
    io::HikRobot camera(io::HikRobotOptions::from_yaml(yaml));
#else
    io::Camera camera(camera_config);
#endif
    const auto begin = Clock::now();
    auto last_report = begin;
    Clock::time_point last_stamp{};
    std::uint64_t previous_processed = 0, previous_captured = 0;
    // [Pi预览] 预览绘制/编码有额外开销，不把该模式当成此前无预览的性能基准。
    log->info("mode={}; model_load_excluded; camera_startup_and_first_inferences_included; "
              "no local window/recording/tracking/aiming/control", preview ? "live_preview" : "live");

    while (!interrupted && Clock::now() - begin < std::chrono::seconds(seconds)) {
      if (preview) preview->check();  // [Pi预览] 传播预览线程异常，避免静默显示停止。
      // [Pi主线C] 持有帧引用直至本次同步检测结束；超时继续等新帧，不检测旧图，不做额外 clone。
      const auto frame = camera.read_frame();
      if (frame && !interrupted && Clock::now() - begin < std::chrono::seconds(seconds)) {
        const auto detect_begin = Clock::now();
        if (frame->image.empty() || frame->image.type() != CV_8UC3 ||
            frame->sequence <= last_sequence || frame->timestamp == Clock::time_point{} ||
            frame->timestamp > detect_begin || (processed && frame->timestamp < last_stamp))
          throw std::runtime_error("invalid BGR frame, non-increasing sequence or invalid host timestamp");
        if (!processed)
          log->info("first image: {}x{}, CV_8UC3 BGR, sequence={}",
                    frame->image.cols, frame->image.rows, frame->sequence);
        const auto input_age = milliseconds(detect_begin - frame->timestamp);
        const auto inference_begin = Clock::now();
        const auto armors = yolo.detect(frame->image);
        const auto detect_ms = milliseconds(Clock::now() - inference_begin);
        ++processed;
        last_sequence = frame->sequence;
        last_stamp = frame->timestamp;
        detect_total_ms += detect_ms;
        detect_max_ms = std::max(detect_max_ms, detect_ms);
        age_total_ms += input_age;
        age_max_ms = std::max(age_max_ms, input_age);
        total_detections += armors.size();
        if (!armors.empty()) {
          // [Pi主线C] 仅记录首次非空结果的一项供现场核对，不逐帧打印或把无目标当推理失败。
          if (++with_detections == 1) {
            const auto & armor = armors.front();
            log->info("first detection: sequence={}, count={}, {} {} {}, confidence={:.3f}; label_unverified",
                      frame->sequence, armors.size(), auto_aim::COLORS.at(armor.color),
                      auto_aim::ARMOR_NAMES.at(armor.name), auto_aim::ARMOR_TYPES.at(armor.type), armor.confidence);
          }
        }
        // [Pi预览] 此次图像与此次检测框一起发布；无框时仍更新画面，不沿用旧框。
        if (preview) preview->publish(frame->image, armors, frame->sequence);
        // [Pi实图] 使用此次检测的同一帧；保存函数返回后不持有Mat或FrameStorage引用。
        if (samples.enabled() && samples.save(frame->image, armors, frame->sequence, Clock::now()))
          log->info("saved_sample={} sequence={} detections={} directory={}",
                    samples.count(), frame->sequence, armors.size(), sample_dir);
      }
      // [Pi主线C] 读取超时也定期报告；process_fps 包含等图与检测，不是单独 NPU 推理 FPS。
      const auto now = Clock::now();
      const auto elapsed = std::chrono::duration<double>(now - last_report).count();
      if (elapsed >= 1.0) {
        const auto stats = camera.stats();
        log->info("capture_fps={:.1f}, process_fps={:.1f}, processed={}, frames_with_detections={}, "
                  "consumer_skipped={}, pool_dropped={}, read_timeouts={}, capture_timeouts={}, reconnects={}",
                  (stats.captured - previous_captured) / elapsed, (processed - previous_processed) / elapsed,
                  processed, with_detections, stats.consumer_skipped, stats.pool_dropped,
                  stats.read_timeouts, stats.capture_timeouts, stats.reconnects);
        previous_processed = processed;
        previous_captured = stats.captured;
        last_report = now;
      }
    }
    const auto elapsed = std::chrono::duration<double>(Clock::now() - begin).count();
    const auto stats = camera.stats();
    // [Pi实图] 明确实际保存数量，提前退出/断流时不声称已经取得三组。
    if (samples.enabled()) log->info("saved_samples={} directory={}", samples.count(), sample_dir);
    log->info("end={}, processed_frames={}, frames_with_detections={}, empty_detection_frames={}, "
              "total_detections={}, last_sequence={}, wall_seconds={:.3f}, process_fps={:.3f}",
              interrupted ? "interrupt" : "duration", processed, with_detections, processed - with_detections,
              total_detections, last_sequence, elapsed, elapsed > 0 ? processed / elapsed : 0);
    log->info("camera_stats_at_end: captured={}, delivered={}, consumer_skipped={}, pool_dropped={}, "
              "read_timeouts={}, capture_timeouts={}, reconnects={}", stats.captured, stats.delivered,
              stats.consumer_skipped, stats.pool_dropped, stats.read_timeouts, stats.capture_timeouts, stats.reconnects);
    if (processed)
      log->info("detect_ms avg={:.3f} max={:.3f}; input_host_age_ms avg={:.3f} max={:.3f}; "
                "host age excludes exposure and SDK/USB buffering", detect_total_ms / processed,
                detect_max_ms, age_total_ms / processed, age_max_ms);
    // [Pi主线C] 零检测框仍可正常完成；没有成功处理图像返回2，异常返回1，中断返回130。
    return interrupted ? 130 : (processed ? 0 : 2);
  } catch (const std::exception & error) {
    std::cerr << "camera+Hailo failed after processed_frames=" << processed << ": " << error.what() << '\n';
    return 1;
  }
}
