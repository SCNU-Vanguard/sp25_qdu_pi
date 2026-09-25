#pragma once

// [Pi预览] Linux/Pi 独立检测入口的可选浏览器预览；复用同帧 Armor，不参与推理或控制。
#include <arpa/inet.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <exception>
#include <list>
#include <memory>
#include <mutex>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

#include "tasks/auto_aim/armor.hpp"

namespace live_preview
{
// [Pi预览] Additive overlay payload.  The caller converts Tracker/Aimer output into pixel
// coordinates itself, so this header keeps depending only on Armor and plain OpenCV types and does
// not drag the Kalman filter or the C board into every translation unit that includes it.
struct Overlay
{
  // Corners of every armor plate of the Tracker-selected target, in pixels.
  std::vector<std::vector<cv::Point2f>> target_quads;
  // Predicted aim point in pixels; only meaningful when has_aim_point is set.
  cv::Point2f aim_point{0, 0};
  bool has_aim_point{false};
  // One-line status drawn at the top-left; empty means draw nothing.
  std::string status;
};

// [Pi预览] 只在选中展示的帧上复制/画框；原始相机槽位与送入 YOLO 的图像不被涂改。
inline void draw_detection_boxes(cv::Mat & result, const std::list<auto_aim::Armor> & armors)
{
  for (const auto & armor : armors) {
    if (armor.points.size() != 4) continue;
    for (int i = 0; i < 4; ++i) {
      cv::line(result, armor.points[i], armor.points[(i + 1) % 4], {0, 255, 0}, 2);
      cv::circle(result, armor.points[i], 3, {0, 255, 255}, -1);
    }
    const auto label = auto_aim::COLORS.at(armor.color) + " " +
      auto_aim::ARMOR_NAMES.at(armor.name) + " " + auto_aim::ARMOR_TYPES.at(armor.type) +
      cv::format(" %.2f", armor.confidence);
    const cv::Point origin(std::max(0, std::min(result.cols - 1, cvRound(armor.points[0].x))),
                           std::max(16, std::min(result.rows - 1, cvRound(armor.points[0].y) - 8)));
    cv::putText(result, label, origin, cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 0, 0}, 3);
    cv::putText(result, label, origin, cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 255, 0}, 1);
  }
}

inline cv::Mat draw(const cv::Mat & bgr, const std::list<auto_aim::Armor> & armors)
{
  cv::Mat result = bgr.clone();
  draw_detection_boxes(result, armors);
  return result;
}

// [Pi预览-OVERLAY] Overlay-aware variant.  Detection boxes stay green; the Tracker-selected target
//  is outlined in blue, the predicted aim point gets a magenta cross, and a one-line status is
//  drawn at the top-left.  The draw() above is untouched, so existing callers are unaffected.
inline cv::Mat draw(
  const cv::Mat & bgr, const std::list<auto_aim::Armor> & armors, const Overlay & overlay)
{
  cv::Mat result = bgr.clone();
  draw_detection_boxes(result, armors);

  // Tracker-selected target: blue outline plus a filled center mark, so it is unmistakably distinct
  // from the green detection boxes that merely report what the network saw.
  for (const auto & quad : overlay.target_quads) {
    if (quad.size() != 4) continue;
    for (int i = 0; i < 4; ++i)
      cv::line(result, quad[i], quad[(i + 1) % 4], {255, 128, 0}, 3);
    cv::Point2f center(0, 0);
    for (const auto & point : quad) center += point;
    center *= 0.25F;
    cv::circle(result, center, 5, {255, 128, 0}, -1);
  }

  // Predicted aim point: a magenta cross with a ring, drawn last so it stays visible on top.
  if (overlay.has_aim_point) {
    const cv::Point center(cvRound(overlay.aim_point.x), cvRound(overlay.aim_point.y));
    // Bounds are checked directly rather than through cv::pointInside, which is not available in
    // every OpenCV 4.x minor release this project builds against.
    if (center.x >= 0 && center.y >= 0 && center.x < result.cols && center.y < result.rows) {
      const int arm = 14;
      cv::line(result, {center.x - arm, center.y}, {center.x + arm, center.y}, {255, 0, 255}, 2);
      cv::line(result, {center.x, center.y - arm}, {center.x, center.y + arm}, {255, 0, 255}, 2);
      cv::circle(result, center, arm / 2, {255, 0, 255}, 2);
    }
  }

  if (!overlay.status.empty()) {
    constexpr int margin = 8;
    int baseline = 0;
    const cv::Size text =
      cv::getTextSize(overlay.status, cv::FONT_HERSHEY_SIMPLEX, 0.55, 1, &baseline);
    const cv::Rect panel(
      margin, margin, std::min(result.cols - 2 * margin, text.width + 2 * margin),
      text.height + 2 * margin);
    if (panel.width > 0 && panel.height > 0) {
      cv::rectangle(result, panel, {0, 0, 0}, cv::FILLED);
      cv::putText(
        result, overlay.status, {panel.x + margin, panel.y + margin + text.height},
        cv::FONT_HERSHEY_SIMPLEX, 0.55, {255, 255, 255}, 1, cv::LINE_AA);
    }
  }
  return result;
}

class Server
{
  using Clock = std::chrono::steady_clock;
  struct Socket {
    int fd;
    explicit Socket(int value) : fd(value) {}
    ~Socket() { if (fd >= 0) ::close(fd); }
    Socket(const Socket &) = delete;
    Socket & operator=(const Socket &) = delete;
  };
  struct Frame {
    std::vector<uchar> jpeg;
    Clock::time_point time;
    std::uint64_t sequence;
    std::size_t count;
    std::string status;  // [Pi预览-OVERLAY] 叠层状态行；空表示无叠层信息。
  };
  Socket listener_;
  std::atomic<bool> stop_{false};
  std::mutex mutex_;
  std::shared_ptr<const Frame> latest_;
  std::exception_ptr failure_;
  Clock::time_point next_publish_{};
  const std::string instructions_;
  std::thread worker_;

  // [Pi预览] 一次仅处理一个短请求，读/写都有期限；慢浏览器不会阻塞检测线程或退出。
  bool ready(int fd, short events, Clock::time_point deadline)
  {
    while (!stop_ && Clock::now() < deadline) {
      pollfd item{fd, events, 0};
      const int result = ::poll(&item, 1, 50);
      if (result < 0 && errno != EINTR) return false;
      if (result > 0) return (item.revents & events) != 0;
    }
    return false;
  }

  bool send_bytes(int fd, const void * data, std::size_t size, Clock::time_point deadline)
  {
    const auto * bytes = static_cast<const char *>(data);
    while (size && ready(fd, POLLOUT, deadline)) {
      const auto sent = ::send(fd, bytes, size, MSG_NOSIGNAL);
      if (sent > 0) { bytes += sent; size -= static_cast<std::size_t>(sent); }
      else if (sent == 0 || (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)) return false;
    }
    return size == 0;
  }

  void reply(int fd, const std::string & status, const char * type,
             const void * data, std::size_t size, const std::string & extra = "")
  {
    const auto headers = "HTTP/1.1 " + status + "\r\nContent-Type: " + type +
      "\r\nContent-Length: " + std::to_string(size) +
      "\r\nCache-Control: no-store\r\nConnection: close\r\nX-Content-Type-Options: nosniff\r\n" + extra + "\r\n";
    const auto deadline = Clock::now() + std::chrono::milliseconds(500);
    if (send_bytes(fd, headers.data(), headers.size(), deadline))
      send_bytes(fd, data, size, deadline);
  }

  void serve(int fd)
  {
    std::string request;
    const auto deadline = Clock::now() + std::chrono::milliseconds(300);
    char buffer[1024];
    while (request.size() < 4096 && request.find("\r\n\r\n") == std::string::npos) {
      if (!ready(fd, POLLIN, deadline)) return;
      const auto count = ::recv(fd, buffer, sizeof(buffer), 0);
      if (count > 0) request.append(buffer, static_cast<std::size_t>(count));
      else if (count == 0 || (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)) return;
    }
    if (request.find("\r\n\r\n") == std::string::npos) return;
    if (request.rfind("GET / HTTP/1.", 0) == 0) {
      static constexpr char page[] = R"HTML(<!doctype html>
<html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>SP25 实时预览</title><style>
body{margin:24px;background:#15191f;color:#edf2f7;font:16px system-ui;text-align:center}
img{max-width:100%;height:auto;border:1px solid #52606d}p{color:#cbd5e0}#status{min-height:1.5em}
</style><h2>SP25 实时预览</h2><p>{{instructions}}</p>
<p id="status">等待相机图像…</p><img id="view" alt="等待实时图像" hidden>
<script>
const view=document.getElementById('view'),status=document.getElementById('status');
let objectUrl=null;
async function update(){
  const controller=new AbortController();
  const timeout=setTimeout(()=>controller.abort(),1500);
  try{
    const response=await fetch('/frame.jpg',{cache:'no-store',signal:controller.signal});
    if(!response.ok)throw Error('waiting');
    const age=Number(response.headers.get('X-Frame-Age-Ms'));
    if(!Number.isFinite(age)||age>1000)throw Error('stale');
    const blob=await response.blob();
    const next=URL.createObjectURL(blob);
    view.src=next;view.hidden=false;
    if(objectUrl)URL.revokeObjectURL(objectUrl);objectUrl=next;
    const overlay=response.headers.get('X-Overlay-Status')||'';
    status.textContent='实时画面 · 帧 '+response.headers.get('X-Frame-Sequence')+
      (Number(response.headers.get('X-Detection-Count'))>0?' · 检测框 '+response.headers.get('X-Detection-Count'):'')+(overlay?' · '+overlay:'');
  }catch(error){
    view.hidden=true;
    status.textContent=error.message==='stale'?'图像更新已暂停，等待新帧…':'等待图像；请确认树莓派上的预览程序仍在运行。';
  }finally{clearTimeout(timeout);setTimeout(update,100);}
}
update();
</script></html>)HTML";
      std::string html(page);
      html.replace(html.find("{{instructions}}"), 16, instructions_);
      reply(fd, "200 OK", "text/html; charset=utf-8", html.data(), html.size());
    } else if (request.rfind("GET /frame.jpg HTTP/1.", 0) == 0) {
      std::shared_ptr<const Frame> frame;
      { std::lock_guard<std::mutex> lock(mutex_); frame = latest_; }
      if (!frame) { reply(fd, "503 Service Unavailable", "text/plain", "Waiting for frame", 17); return; }
      const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - frame->time).count();
      const auto extra = "X-Frame-Age-Ms: " + std::to_string(age) +
        "\r\nX-Frame-Sequence: " + std::to_string(frame->sequence) +
        "\r\nX-Detection-Count: " + std::to_string(frame->count) +
        "\r\nX-Overlay-Status: " + frame->status + "\r\n";
      reply(fd, "200 OK", "image/jpeg", frame->jpeg.data(), frame->jpeg.size(), extra);
    } else {
      reply(fd, "404 Not Found", "text/plain", "Not found", 9);
    }
  }

public:
  // instructions is trusted, caller-supplied page text, never a request parameter.
  explicit Server(int port, const std::string & instructions =
                    "绿框：检测到的装甲板　蓝框：Tracker 锁定目标　品红十字：预测瞄准点")
  : listener_(::socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0)),
    instructions_(instructions)
  {
    if (listener_.fd < 0) throw std::system_error(errno, std::generic_category(), "preview socket");
    const int reuse = 1;
    ::setsockopt(listener_.fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(static_cast<std::uint16_t>(port));
    if (::bind(listener_.fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0 ||
        ::listen(listener_.fd, 4) != 0)
      throw std::system_error(errno, std::generic_category(), "preview bind/listen");
    worker_ = std::thread([this] {
      try {
        while (!stop_) {
          if (!ready(listener_.fd, POLLIN, Clock::now() + std::chrono::milliseconds(100))) continue;
          Socket client(::accept4(listener_.fd, nullptr, nullptr, SOCK_NONBLOCK | SOCK_CLOEXEC));
          if (client.fd >= 0) serve(client.fd);
          else if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
            throw std::system_error(errno, std::generic_category(), "preview accept");
        }
      } catch (...) {
        std::lock_guard<std::mutex> lock(mutex_);
        failure_ = std::current_exception();
      }
    });
  }

  ~Server() { stop_ = true; if (worker_.joinable()) worker_.join(); }
  Server(const Server &) = delete;
  Server & operator=(const Server &) = delete;

  void check()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (failure_) std::rethrow_exception(failure_);
  }

  // [Pi预览] 最多10次/秒绘制编码，只保存最新JPEG；网络在独立线程，断网不积压相机帧。
  void publish(const cv::Mat & bgr, const std::list<auto_aim::Armor> & armors, std::uint64_t sequence)
  {
    publish(bgr, armors, sequence, Overlay{});
  }

  // [Pi预览-OVERLAY] Overlay-aware variant.  An empty Overlay draws exactly what the call above
  // draws, so the two entry points stay interchangeable.
  void publish(
    const cv::Mat & bgr, const std::list<auto_aim::Armor> & armors, std::uint64_t sequence,
    const Overlay & overlay)
  {
    const auto now = Clock::now();
    if (now < next_publish_) return;
    next_publish_ = now + std::chrono::milliseconds(100);
    auto frame = std::make_shared<Frame>();
    frame->time = now;
    frame->sequence = sequence;
    frame->count = armors.size();
    frame->status = overlay.status;
    if (!cv::imencode(".jpg", draw(bgr, armors, overlay), frame->jpeg, {cv::IMWRITE_JPEG_QUALITY, 85}))
      throw std::runtime_error("preview JPEG encoding failed");
    { std::lock_guard<std::mutex> lock(mutex_); latest_ = std::move(frame); }
  }
};
}  // namespace live_preview
