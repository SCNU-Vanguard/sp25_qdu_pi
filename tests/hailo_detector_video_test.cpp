// [Pi视频A] 独立连续视频验收：只调用生产 YOLO，不初始化相机、跟踪或控制。
#include "tasks/auto_aim/yolo.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace
{
using Clock = std::chrono::steady_clock;
double milliseconds(Clock::duration duration)
{
  return std::chrono::duration<double, std::milli>(duration).count();
}

// [Pi视频A] 两份固定直方图共约 313 KiB；平均/最大值覆盖全部帧，p95 为 0.05 ms 桶上界。
// 超过 1000 ms 的样本进入溢出桶，p95 落入该桶时明确输出 >1000，不能假装精确。
struct Timing
{
  static constexpr double step = 0.05;
  static constexpr std::size_t limit = 20000;
  std::array<std::uint64_t, limit + 2> bins{};
  std::uint64_t count = 0;
  double total = 0, maximum = 0;

  void add(double ms)
  {
    const auto bin = ms > limit * step ? limit + 1 : static_cast<std::size_t>(std::ceil(ms / step));
    ++bins[bin];
    ++count;
    total += ms;
    maximum = std::max(maximum, ms);
  }

  void print(const char * name) const
  {
    const auto rank = count - count / 20;  // ceil(count * 0.95)，避免浮点计数舍入。
    std::uint64_t cumulative = 0;
    std::size_t bin = 0;
    for (; bin < bins.size(); ++bin) {
      cumulative += bins[bin];
      if (cumulative >= rank) break;
    }
    std::cout << name << "_ms avg=" << total / count << " p95_upper=";
    if (bin > limit) std::cout << ">1000";
    else std::cout << bin * step;
    std::cout << " max=" << maximum << " over_1000ms=" << bins.back() << '\n';
  }
};

// [Pi视频A] 严格校验计数/零起始下标，拒绝负数、尾随字符和重复抽帧，最多保存 8 组。
std::uint64_t integer(const std::string & text)
{
  if (text.empty() || text.find_first_not_of("0123456789") != std::string::npos) {
    throw std::invalid_argument("Expected a nonnegative integer: " + text);
  }
  return std::stoull(text);
}

struct Options
{
  std::uint64_t expected = 0;
  std::filesystem::path sample_dir;
  std::vector<std::uint64_t> samples;
};

Options parse(int argc, char ** argv)
{
  Options options;
  for (int i = 3; i < argc; ++i) {
    const std::string flag = argv[i];
    if (flag == "--expect-frames" && i + 1 < argc && options.expected == 0) {
      options.expected = integer(argv[++i]);
      if (!options.expected) throw std::invalid_argument("Expected frame count must be positive");
    } else if (flag == "--samples" && i + 2 < argc && options.sample_dir.empty()) {
      options.sample_dir = argv[++i];
      const std::string indices = argv[++i];
      if (options.sample_dir.empty() || indices.empty() || indices.back() == ',') {
        throw std::invalid_argument("Use --samples NEW_DIRECTORY 0,100,...");
      }
      std::istringstream input(indices);
      std::string item;
      while (std::getline(input, item, ',')) {
        if (options.samples.size() == 8) throw std::invalid_argument("At most 8 sample frames");
        options.samples.push_back(integer(item));
      }
      std::sort(options.samples.begin(), options.samples.end());
      if (std::adjacent_find(options.samples.begin(), options.samples.end()) != options.samples.end()) {
        throw std::invalid_argument("Duplicate sample index");
      }
    } else {
      throw std::invalid_argument("Unknown, repeated or incomplete option: " + flag);
    }
  }
  if (options.expected && !options.samples.empty() && options.samples.back() >= options.expected) {
    throw std::invalid_argument("Sample index must be smaller than expected frame count");
  }
  return options;
}

// [Pi视频A] 图片抽查单独运行，不报告速度；逐次落盘并释放，不保存整段图像或检测历史。
void save_sample(const std::filesystem::path & directory, std::ofstream & record,
                 std::uint64_t index, const cv::Mat & bgr, const std::list<auto_aim::Armor> & armors)
{
  cv::Mat drawing = bgr.clone();
  record << "frame_index=" << index << " detections=" << armors.size()
         << " ground_truth=unverified\n";
  std::size_t number = 0;
  for (const auto & armor : armors) {
    const auto label = std::to_string(number++) + ": " + auto_aim::COLORS[armor.color] + " " +
      auto_aim::ARMOR_NAMES[armor.name] + " " + auto_aim::ARMOR_TYPES[armor.type];
    record << label << " confidence=" << armor.confidence << " corners(LT,RT,RB,LB)=";
    for (int i = 0; i < 4; ++i) {
      record << armor.points[i] << ' ';
      cv::line(drawing, armor.points[i], armor.points[(i + 1) % 4], {0, 255, 0}, 2);
      cv::putText(drawing, std::to_string(i), armor.points[i], cv::FONT_HERSHEY_SIMPLEX,
                  0.5, {0, 255, 255}, 1);
    }
    record << '\n';
    cv::putText(drawing, label, armor.points[0] + cv::Point2f(0, -8),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 255, 0}, 1);
  }
  const auto prefix = directory / ("frame-" + std::to_string(index));
  if (!cv::imwrite(prefix.string() + "-raw.png", bgr) ||
      !cv::imwrite(prefix.string() + "-result.png", drawing)) {
    throw std::runtime_error("Cannot save sample images");
  }
  record.flush();
  if (!record) throw std::runtime_error("Cannot write sample record");
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc == 2 && std::string(argv[1]) == "--help") {
    std::cout << "Usage: hailo_detector_video_test config.yaml video.avi "
                 "[--expect-frames N] [--samples NEW_DIRECTORY 0,100,...]\n";
    return 0;
  }
  std::uint64_t processed = 0;
  try {
    if (argc < 3) throw std::invalid_argument("Pass config.yaml video.avi; see --help");
    const auto options = parse(argc, argv);
    const bool sampling = !options.sample_dir.empty();
    if (!std::filesystem::is_regular_file(argv[2])) throw std::invalid_argument("Input must be a local video file");
    if (sampling && std::filesystem::exists(options.sample_dir)) {
      throw std::invalid_argument("Sample directory already exists; choose a new directory");
    }
    cv::VideoCapture video(argv[2]);
    if (!video.isOpened()) throw std::runtime_error("Cannot open video");
    cv::Mat bgr;
    if (!video.read(bgr) || bgr.empty()) throw std::runtime_error("Cannot decode the first frame");
    const auto size = bgr.size();
    std::cout << std::fixed << std::setprecision(3)
              << "mode=" << (sampling ? "samples (no FPS benchmark)" : "baseline")
              << " backend=" << video.getBackendName() << " size=" << size.width << 'x' << size.height
              << " nominal_fps=" << video.get(cv::CAP_PROP_FPS)
              << " metadata_frames=" << video.get(cv::CAP_PROP_FRAME_COUNT) << '\n';

    // [Pi视频A] 仅构造一次生产 YOLO，首帧额外预热 10 次；重新打开文件从第 0 帧顺序读。
    // 不用 set(POS_FRAMES) 跳转，也不拿可能为 0 的 FRAME_COUNT 作为循环边界。
    auto_aim::YOLO detector(argv[1], true);
    for (int i = 0; i < 10; ++i) detector.detect(bgr);
    video.release();
    if (!video.open(argv[2])) throw std::runtime_error("Cannot reopen video after warmup");
    std::ofstream record;
    if (sampling) {
      if (!std::filesystem::create_directories(options.sample_dir)) {
        throw std::runtime_error("Cannot create a new sample directory");
      }
      record.open(options.sample_dir / "samples.txt");
      if (!record) throw std::runtime_error("Cannot create sample record");
      record << std::fixed << std::setprecision(3)
             << "Zero-based frame indices. Ground truth requires visual/user verification.\n";
    }
    Timing detection, read_and_detection;
    std::uint64_t empty_frames = 0, multi_frames = 0, total_detections = 0;
    std::size_t next_sample = 0, max_detections = 0;
    std::cout << "warmup_calls=10; reading sequentially without playback delay\n" << std::flush;
    const auto start = Clock::now();
    for (;;) {
      const auto before_read = Clock::now();
      // [Pi视频A] read=false 时绝不再次推理旧图；异常直接退出，不能继续复用上一帧 Armor。
      if (!video.read(bgr)) break;
      if (bgr.empty() || bgr.type() != CV_8UC3 || bgr.size() != size) {
        throw std::runtime_error("Decoded frame is empty or its format/size changed");
      }
      const auto before_detect = Clock::now();
      const auto armors = detector.detect(bgr);
      const auto after_detect = Clock::now();
      if (!sampling) {
        detection.add(milliseconds(after_detect - before_detect));
        read_and_detection.add(milliseconds(after_detect - before_read));
      }
      empty_frames += armors.empty();
      multi_frames += armors.size() > 1;
      total_detections += armors.size();
      max_detections = std::max(max_detections, armors.size());
      if (sampling && next_sample < options.samples.size() && processed == options.samples[next_sample]) {
        save_sample(options.sample_dir, record, processed, bgr, armors);
        ++next_sample;
      }
      ++processed;
    }
    const double wall_ms = milliseconds(Clock::now() - start);
    if (!processed) throw std::runtime_error("No frames decoded on the measured pass");
    // [Pi视频A] OpenCV read=false 本身不能区分 EOF 与解码失败；独立实数帧数用于核对完整性。
    // 即使元数据帧数错误也读到 read=false；帧数不符时不输出成功/性能汇总。
    if (options.expected && processed != options.expected) {
      throw std::runtime_error("Decoded frame count mismatch: expected=" + std::to_string(options.expected));
    }
    if (sampling && next_sample != options.samples.size()) throw std::runtime_error("Requested sample beyond decoded video");
    std::cout << "stream_end=read_false processed_frames=" << processed
              << " expected_frames=" << options.expected
              << " count_check=" << (options.expected ? "matched" : "not_requested") << '\n'
              << "empty_detection_frames=" << empty_frames << " multi_detection_frames=" << multi_frames
              << " total_detections=" << total_detections << " max_detections=" << max_detections << '\n';
    if (sampling) {
      record << "completed_frames=" << processed << " saved_samples=" << next_sample << '\n';
      record.close();
      if (!record) throw std::runtime_error("Cannot finish sample record");
      std::cout << "saved_samples=" << next_sample << " directory=" << options.sample_dir << '\n';
    } else {
      // [Pi视频A] 检测 FPS 用 detect 累计时间；实际吞吐用整个循环墙钟，含读帧、统计和末次读。
      std::cout << "video_detector_fps=" << processed * 1000.0 / detection.total << '\n'
                << "video_read_detect_fps=" << processed * 1000.0 / wall_ms
                << " wall_seconds=" << wall_ms / 1000.0 << '\n';
      detection.print("detect");
      read_and_detection.print("read_detect");
      std::cout << "p95 histogram bin width=0.050 ms; initialization/warmup excluded.\n";
    }
    std::cout << "Read end/count match alone cannot certify absence of codec warnings or correct labels.\n"
                 "Excludes camera, tracking, solving, aiming and control.\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "Hailo video test failed after processed_frames=" << processed << ": " << error.what() << '\n';
    return 1;
  }
}
