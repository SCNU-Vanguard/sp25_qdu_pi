// Camera-only intrinsic calibration capture. No Hailo, CBoard, serial port or IMU is opened.
#include <fmt/core.h>

#include <chrono>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <thread>

#include "calibration/circle_grid.hpp"
#include "io/camera.hpp"
#include "tests/live_preview.hpp"

namespace
{
using Clock = std::chrono::steady_clock;
volatile std::sig_atomic_t interrupted = 0;
void interrupt(int) { interrupted = 1; }

const std::string preview_page = R"HTML(<!doctype html>
<html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>相机内参标定</title><style>
body{background:#15191f;color:#edf2f7;font:18px system-ui;text-align:center;margin:24px}
img{max-width:100%;border:1px solid #52606d}p{line-height:1.6}
</style><h1>相机内参标定 · 圆点采集</h1>
<p>彩色连线表示识别到的圆点。grid=OK 才会保存；saved 是已保存张数，next 是拍照倒计时。<br>
拍照前拿稳标定板，保存后换位置或倾斜角度。原图自动保存，不要用网页截图代替。</p>
<p id="status">等待相机画面……</p><img id="view" hidden>
<script>
const view=document.getElementById('view'),status=document.getElementById('status');let url=null;
async function update(){
 const controller=new AbortController();const timer=setTimeout(()=>controller.abort(),1500);
 try{
  const r=await fetch('/frame.jpg',{cache:'no-store',signal:controller.signal});
  if(!r.ok||Number(r.headers.get('X-Frame-Age-Ms'))>1000)throw Error('waiting');
  const next=URL.createObjectURL(await r.blob());view.src=next;view.hidden=false;
  if(url)URL.revokeObjectURL(url);url=next;
  status.textContent=r.headers.get('X-Overlay-Status');
 }catch(e){view.hidden=true;status.textContent='画面已暂停：请查看 SSH 窗口中的采集状态。';}
 finally{clearTimeout(timer);setTimeout(update,150);}
}update();
</script></html>)HTML";

const std::string keys =
  "{help h          |      | show help}"
  "{@camera-config  | configs/pi_standard3_qdu.yaml | use the final camera configuration}"
  "{pattern-config  | configs/calibration.yaml | aligned circle grid dimensions}"
  "{output-folder   |      | required NEW directory for original PNG images}"
  "{preview-port    | 8080 | browser preview port}"
  "{interval        | 5    | minimum seconds between accepted images}"
  "{max-samples     | 30   | stop after this many accepted images}"
  "{seconds         | 600  | stop even if not enough boards have been found}";
}

int main(int argc, char ** argv)
{
  try {
    cv::CommandLineParser cli(argc, argv, keys);
    if (cli.has("help")) { cli.printMessage(); return 0; }
    const auto camera_config = cli.get<std::string>(0);
    const auto pattern_config = cli.get<std::string>("pattern-config");
    const auto output = cli.get<std::string>("output-folder");
    const int port = cli.get<int>("preview-port");
    const int interval = cli.get<int>("interval");
    const int max_samples = cli.get<int>("max-samples");
    const int seconds = cli.get<int>("seconds");
    if (!cli.check() || output.empty() || port < 1 || port > 65535 || interval < 1 ||
        max_samples < 1 || seconds < 1) {
      cli.printErrors();
      throw std::runtime_error("provide a new --output-folder and positive capture limits");
    }
    const calibration::CircleGrid grid(pattern_config);
    if (std::filesystem::exists(output))
      throw std::runtime_error("output-folder already exists; use a new session directory");
    std::filesystem::create_directories(output);
    const auto folder = std::filesystem::path(output);
    std::filesystem::copy_file(camera_config, folder / "camera-used.yaml");
    std::filesystem::copy_file(pattern_config, folder / "pattern-used.yaml");
    std::ofstream manifest(folder / "frames.csv");
    manifest.exceptions(std::ios::badbit | std::ios::failbit);
    manifest << "file,sequence,width,height\n";

    std::signal(SIGINT, interrupt);
    std::signal(SIGTERM, interrupt);
    live_preview::Server preview(port, preview_page);
    io::Camera camera(camera_config);
    std::cout << fmt::format(
      "INTRINSICS ONLY: {} columns x {} rows, spacing {} mm; preview port {}.\n"
      "Save original frames only after all circles are found; hold still for each save.\n"
      "Move the board to different positions, distances and tilts between saves.\n",
      grid.size.width, grid.size.height, grid.spacing_mm, port) << std::flush;
    const auto deadline = Clock::now() + std::chrono::seconds(seconds);
    auto next_save = Clock::now() + std::chrono::seconds(interval);
    auto next_detection = Clock::now();
    auto next_report = Clock::now();
    cv::Size image_size;
    int saved = 0;
    while (!interrupted && Clock::now() < deadline && saved < max_samples) {
      // Circle extraction can be slow: do not hold camera pool buffers between checks.
      if (Clock::now() < next_detection) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }
      const auto frame = camera.read_frame();
      if (!frame) { preview.check(); continue; }
      const auto & raw = frame->image;
      if (image_size.empty()) image_size = raw.size();
      if (raw.size() != image_size) throw std::runtime_error("camera image size changed");
      std::vector<cv::Point2f> centers;
      const bool found = grid.find(raw, centers);
      const auto now = Clock::now();
      next_detection = now + std::chrono::milliseconds(200);
      if (found && now >= next_save) {
        const auto filename = fmt::format("{:04d}.png", saved + 1);
        // Never save the browser's JPEG or the annotated copy as calibration input.
        if (!cv::imwrite((folder / filename).string(), raw))
          throw std::runtime_error("failed saving original PNG");
        manifest << filename << ',' << frame->sequence << ',' << raw.cols << ',' << raw.rows << '\n';
        manifest.flush();
        ++saved;
        next_save = now + std::chrono::seconds(interval);
        std::cout << fmt::format("SAVED {}/{}: {} ({}x{})\n", saved, max_samples,
          (folder / filename).string(), raw.cols, raw.rows) << std::flush;
      }
      auto drawing = raw.clone();
      cv::drawChessboardCorners(drawing, grid.size, centers, found);
      live_preview::Overlay overlay;
      const auto remaining = std::max(0.0, std::chrono::duration<double>(next_save - now).count());
      overlay.status = fmt::format("CALIBRATION | grid={} | saved={}/{} | next={:.1f}s",
        found ? "OK" : "NOT FOUND", saved, max_samples, remaining);
      preview.publish(drawing, {}, frame->sequence, overlay);
      preview.check();
      if (now >= next_report) {
        std::cout << overlay.status << '\n' << std::flush;
        next_report = now + std::chrono::seconds(2);
      }
    }
    std::cout << fmt::format("capture end: saved={} folder={}\n", saved, output);
    return saved > 0 ? 0 : 2;
  } catch (const std::exception & error) {
    std::cerr << "capture_intrinsics failed: " << error.what() << '\n';
    return 1;
  }
}
