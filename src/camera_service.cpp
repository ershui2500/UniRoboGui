#include "g1_web/camera_service.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <cmath>
#include <cstdlib>
#include <mutex>
#include <sstream>
#include <thread>
#include <utility>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <unistd.h>

#include <json/json.h>

#ifdef G1_WEB_HAS_OPENCV
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#endif

#ifdef G1_WEB_HAS_REALSENSE
#include <librealsense2/rs.hpp>
#endif

#ifdef G1_WEB_HAS_ZMQ
#include <zmq.h>
#endif

namespace g1_web {
namespace {

std::string JsonText(const Json::Value& value) {
  Json::StreamWriterBuilder builder;
  builder["commentStyle"] = "None";
  builder["indentation"] = "";
  return Json::writeString(builder, value);
}

struct StreamState {
  bool configured{false};
  bool online{false};
  std::string source;
  std::string error;
  std::uint64_t sequence{0};
  std::vector<unsigned char> jpeg;
  std::chrono::steady_clock::time_point updated{};
};

std::int64_t AgeMs(const StreamState& stream) {
  if (!stream.online) return -1;
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::steady_clock::now() - stream.updated)
      .count();
}

bool StreamFresh(const StreamState& stream) {
  const auto age_ms = AgeMs(stream);
  return age_ms >= 0 && age_ms <= 2000;
}

bool SafeVideoDevice(const std::string& source) {
  constexpr const char* kPrefix = "/dev/video";
  if (source.empty()) return true;
  if (source.rfind(kPrefix, 0) != 0 || source.size() <= 10 ||
      source.size() > 13) {
    return false;
  }
  return std::all_of(source.begin() + 10, source.end(),
                     [](unsigned char value) {
                       return std::isdigit(value) != 0;
                     });
}

int RunFirstPersonHelper(const char* action) {
  std::string command =
      "sudo -n /usr/local/sbin/g1-web-first-person-service ";
  command += action;
  command += " >/dev/null 2>&1";
  const int result = std::system(command.c_str());
  if (result == -1 || !WIFEXITED(result)) return -1;
  return WEXITSTATUS(result);
}

#ifdef G1_WEB_HAS_REALSENSE
bool ProbeConcurrentRealSense(const CameraOptions& options,
                              std::string& probe_error) {
  try {
    rs2::pipeline pipeline;
    rs2::config config;
    config.enable_stream(RS2_STREAM_COLOR,
                         static_cast<int>(options.width),
                         static_cast<int>(options.height), RS2_FORMAT_BGR8,
                         static_cast<int>(options.fps));
    config.enable_stream(RS2_STREAM_DEPTH,
                         static_cast<int>(options.width),
                         static_cast<int>(options.height), RS2_FORMAT_Z16,
                         static_cast<int>(options.fps));
    pipeline.start(config);
    const rs2::frameset frames = pipeline.wait_for_frames(2000);
    const bool ready = static_cast<bool>(frames.get_color_frame()) &&
                       static_cast<bool>(frames.get_depth_frame());
    pipeline.stop();
    if (!ready) {
      probe_error = "realsense_probe_missing_frame";
      return false;
    }
    probe_error.clear();
    return true;
  } catch (const rs2::error& exception) {
    probe_error = exception.what();
    return false;
  } catch (const std::exception& exception) {
    probe_error = exception.what();
    return false;
  }
}
#endif

#ifdef G1_WEB_HAS_OPENCV
struct VideoDeviceProbe {
  std::string path;
  bool depth{false};
  bool gray{false};
  int color_score{0};
};

VideoDeviceProbe ProbeVideoDevice(const std::string& path) {
  VideoDeviceProbe probe;
  probe.path = path;
  const int fd = ::open(path.c_str(), O_RDWR | O_NONBLOCK);
  if (fd < 0) return probe;

  v4l2_capability capability{};
  if (::ioctl(fd, VIDIOC_QUERYCAP, &capability) != 0) {
    ::close(fd);
    return probe;
  }
  const std::uint32_t caps = capability.device_caps != 0
                                 ? capability.device_caps
                                 : capability.capabilities;
  if ((caps & V4L2_CAP_VIDEO_CAPTURE) == 0) {
    ::close(fd);
    return probe;
  }

  for (std::uint32_t index = 0;; ++index) {
    v4l2_fmtdesc format{};
    format.index = index;
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (::ioctl(fd, VIDIOC_ENUM_FMT, &format) != 0) break;
    const std::uint32_t pixel = format.pixelformat;
    if (pixel == v4l2_fourcc('Z', '1', '6', ' ')) probe.depth = true;
    if (pixel == V4L2_PIX_FMT_GREY ||
        pixel == v4l2_fourcc('Y', '8', 'I', ' ') ||
        pixel == v4l2_fourcc('Y', '1', '2', 'I')) {
      probe.gray = true;
    }
    if (pixel == V4L2_PIX_FMT_MJPEG || pixel == V4L2_PIX_FMT_JPEG) {
      probe.color_score = std::max(probe.color_score, 4);
    } else if (pixel == V4L2_PIX_FMT_YUYV ||
               pixel == V4L2_PIX_FMT_RGB24 ||
               pixel == V4L2_PIX_FMT_BGR24) {
      probe.color_score = std::max(probe.color_score, 3);
    } else if (pixel == V4L2_PIX_FMT_UYVY || pixel == V4L2_PIX_FMT_NV12) {
      probe.color_score = std::max(probe.color_score, 2);
    }
  }
  ::close(fd);
  if (probe.gray) probe.color_score = std::max(0, probe.color_score - 2);
  return probe;
}

std::pair<std::string, std::string> DetectV4l2Sources() {
  std::pair<std::string, std::string> detected;
  for (int attempt = 0; attempt < 20; ++attempt) {
    detected = {};
    int best_color_score = 0;
    for (int index = 0; index < 64; ++index) {
      const std::string path = "/dev/video" + std::to_string(index);
      if (::access(path.c_str(), F_OK) != 0) continue;
      const auto probe = ProbeVideoDevice(path);
      if (detected.second.empty() && probe.depth) detected.second = path;
      if (!probe.depth && probe.color_score > best_color_score) {
        best_color_score = probe.color_score;
        detected.first = path;
      }
    }
    if (!detected.first.empty() && !detected.second.empty()) break;
    if (attempt != 19) std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return detected;
}

#ifdef G1_WEB_HAS_ZMQ
bool TeleimagerRgbAvailable() {
  void* context = zmq_ctx_new();
  if (!context) return false;
  void* socket = zmq_socket(context, ZMQ_SUB);
  if (!socket) {
    zmq_ctx_term(context);
    return false;
  }
  const int timeout_ms = 1000;
  const int linger_ms = 0;
  const int conflate = 1;
  zmq_setsockopt(socket, ZMQ_RCVTIMEO, &timeout_ms, sizeof(timeout_ms));
  zmq_setsockopt(socket, ZMQ_LINGER, &linger_ms, sizeof(linger_ms));
  zmq_setsockopt(socket, ZMQ_CONFLATE, &conflate, sizeof(conflate));
  zmq_setsockopt(socket, ZMQ_SUBSCRIBE, "", 0);
  bool available = false;
  if (zmq_connect(socket, "tcp://127.0.0.1:55555") == 0) {
    zmq_msg_t message;
    zmq_msg_init(&message);
    if (zmq_msg_recv(&message, socket, 0) >= 0) {
      const auto* data = static_cast<const unsigned char*>(zmq_msg_data(&message));
      const std::size_t size = zmq_msg_size(&message);
      available = size >= 4 && data[0] == 0xff && data[1] == 0xd8 &&
                  data[size - 2] == 0xff && data[size - 1] == 0xd9;
    }
    zmq_msg_close(&message);
  }
  zmq_close(socket);
  zmq_ctx_term(context);
  return available;
}
#endif

bool NumericSource(const std::string& source, int& index) {
  if (source.empty() || !std::all_of(source.begin(), source.end(),
                                     [](unsigned char value) {
                                       return std::isdigit(value) != 0;
                                     })) {
    return false;
  }
  try {
    index = std::stoi(source);
    return index >= 0;
  } catch (...) {
    return false;
  }
}

bool EncodeJpeg(const cv::Mat& input, std::vector<unsigned char>& jpeg,
                unsigned int output_width, unsigned int output_height,
                int quality) {
  if (input.empty()) return false;
  cv::Mat resized;
  const cv::Mat* encoded = &input;
  if (output_width > 0 && output_height > 0 &&
      (input.cols != static_cast<int>(output_width) ||
       input.rows != static_cast<int>(output_height))) {
    cv::resize(input, resized,
               cv::Size(static_cast<int>(output_width),
                        static_cast<int>(output_height)),
               0.0, 0.0, cv::INTER_AREA);
    encoded = &resized;
  }
  return cv::imencode(".jpg", *encoded, jpeg,
                      {cv::IMWRITE_JPEG_QUALITY,
                       std::clamp(quality, 25, 90),
                       cv::IMWRITE_JPEG_OPTIMIZE, 1});
}

cv::Mat ColorizeDepth(const cv::Mat& frame) {
  cv::Mat gray;
  if (frame.type() == CV_16UC1) {
    // RealSense Z16 is conventionally millimetres. Keep a stable 0-5 m visual
    // range so colors do not pulse when one pixel changes.
    frame.convertTo(gray, CV_8U, 255.0 / 5000.0);
  } else if (frame.type() == CV_32FC1) {
    frame.convertTo(gray, CV_8U, 255.0 / 5.0);
  } else if (frame.channels() == 1) {
    frame.convertTo(gray, CV_8U);
  } else {
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
  }
  cv::Mat colorized;
  cv::applyColorMap(gray, colorized, cv::COLORMAP_TURBO);
  return colorized;
}
#endif

}  // namespace

class CameraService::Impl {
 public:
  explicit Impl(CameraOptions options) : options_(std::move(options)) {
    ApplyOptionsLocked();
  }

  void ApplyOptionsLocked() {
    rgb_ = {};
    depth_ = {};
    rgb_.configured = options_.mock || options_.realsense ||
                      options_.teleimager_rgb || !options_.rgb_source.empty();
    rgb_.source = options_.mock ? "mock://rgb" :
                  options_.realsense ? "realsense://color" :
                  options_.teleimager_rgb ? "zmq://127.0.0.1:55555" :
                  options_.rgb_source;
    depth_.configured = options_.mock || options_.realsense ||
                        !options_.depth_source.empty();
    depth_.source = options_.mock ? "mock://depth" :
                    options_.realsense ? "realsense://depth-z16" :
                    options_.depth_source;
  }

  ~Impl() { Stop(); }

  bool Start(std::string& error) {
    if (running_.exchange(true)) {
      error.clear();
      return true;
    }
#ifndef G1_WEB_HAS_OPENCV
    running_.store(false);
    error = "opencv_not_available";
    std::lock_guard<std::mutex> lock(mutex_);
    rgb_.error = error;
    depth_.error = error;
    return false;
#else
    if (!rgb_.configured && !depth_.configured && !options_.auto_detect) {
      running_.store(false);
      error = "camera_sources_not_configured";
      return false;
    }

    bool shared_realsense = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      realsense_shared_without_pause_ = false;
      if (!options_.realsense) realsense_probe_error_.clear();
    }
#ifdef G1_WEB_HAS_REALSENSE
    if (options_.realsense && !options_.mock) {
      std::string probe_error;
      if (ProbeConcurrentRealSense(options_, probe_error)) {
        shared_realsense = true;
        std::lock_guard<std::mutex> lock(mutex_);
        realsense_shared_without_pause_ = true;
        realsense_probe_error_.clear();
        first_person_was_active_ = false;
        first_person_paused_by_web_ = false;
        first_person_error_.clear();
      } else {
        std::lock_guard<std::mutex> lock(mutex_);
        realsense_shared_without_pause_ = false;
        realsense_probe_error_ = probe_error;
        options_.realsense = false;
        options_.auto_detect = true;
        options_.teleimager_rgb = false;
        options_.rgb_source.clear();
        options_.depth_source.clear();
        ApplyOptionsLocked();
      }
    }
#endif

    if (!shared_realsense && !PauseFirstPerson(error)) {
      running_.store(false);
      std::lock_guard<std::mutex> lock(mutex_);
      rgb_.error = error;
      depth_.error = error;
      return false;
    }
    if (options_.auto_detect && !options_.mock && !options_.realsense) {
      const auto detected = DetectV4l2Sources();
#ifdef G1_WEB_HAS_ZMQ
      if (detected.first.empty() && TeleimagerRgbAvailable()) {
        running_.store(false);
        error = "teleimager_camera_owner_active";
        {
          std::lock_guard<std::mutex> lock(mutex_);
          rgb_.error = error;
          depth_.error = error;
        }
        RestoreFirstPerson();
        return false;
      }
#endif
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (options_.rgb_source.empty()) options_.rgb_source = detected.first;
        if (options_.depth_source.empty()) options_.depth_source = detected.second;
        options_.teleimager_rgb = false;
        ApplyOptionsLocked();
      }
      if (!rgb_.configured && !depth_.configured) {
        running_.store(false);
        error = "camera_auto_detect_failed";
        {
          std::lock_guard<std::mutex> lock(mutex_);
          rgb_.error = error;
          depth_.error = error;
        }
        RestoreFirstPerson();
        return false;
      }
    }
    if (options_.mock) {
      mock_thread_ = std::thread([this] { MockLoop(); });
    } else if (options_.realsense) {
#ifdef G1_WEB_HAS_REALSENSE
      realsense_thread_ = std::thread([this] { RealSenseLoop(); });
#else
      running_.store(false);
      error = "librealsense2_not_available";
      {
        std::lock_guard<std::mutex> lock(mutex_);
        rgb_.error = error;
        depth_.error = error;
      }
      RestoreFirstPerson();
      return false;
#endif
    } else {
      if (rgb_.configured) {
        if (options_.teleimager_rgb) {
#ifdef G1_WEB_HAS_ZMQ
          rgb_thread_ = std::thread([this] { TeleimagerRgbLoop(); });
#else
          SetStreamError(rgb_, "teleimager_zmq_unavailable");
#endif
        } else {
          rgb_thread_ = std::thread([this] {
            CaptureLoop(options_.rgb_source, false, rgb_);
          });
        }
      }
      if (depth_.configured) {
        depth_thread_ = std::thread([this] {
          CaptureDepthLoop(options_.depth_source, depth_);
        });
      }
    }
    if (!options_.mock) {
      capture_watchdog_thread_ = std::thread([this] { CaptureWatchdog(); });
    }
    error.clear();
    return true;
#endif
  }

  void Stop() {
    running_.store(false);
    if (mock_thread_.joinable()) mock_thread_.join();
    if (realsense_thread_.joinable()) realsense_thread_.join();
    if (rgb_thread_.joinable()) rgb_thread_.join();
    if (depth_thread_.joinable()) depth_thread_.join();
    if (capture_watchdog_thread_.joinable()) capture_watchdog_thread_.join();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      rgb_.online = false;
      depth_.online = false;
    }
    RestoreFirstPerson();
  }

  CameraResult Submit(const CameraRequest& request) {
    std::lock_guard<std::mutex> command_lock(command_mutex_);
    CameraResult result;
    if (request.request_key.size() < 8 || request.request_key.size() > 128) {
      result.error = "invalid_request_key";
      return result;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (request.request_key == last_request_key_) {
        result = last_result_;
        result.duplicate = true;
        return result;
      }
    }
    if (!request.confirmed) {
      result.error = "confirmation_required";
      return result;
    }

    CameraOptions next = options_;
    if (request.command == "start_v4l2") {
      if (!SafeVideoDevice(request.rgb_source) ||
          !SafeVideoDevice(request.depth_source)) {
        result.error = "invalid_camera_source";
        return result;
      }
      next.realsense = false;
      next.teleimager_rgb = false;
      next.auto_detect = request.rgb_source.empty() || request.depth_source.empty();
      next.rgb_source = request.rgb_source;
      next.depth_source = request.depth_source;
    } else if (request.command == "start_realsense") {
      // Prefer direct librealsense2 while leaving the robot camera-owner
      // service untouched. Start() probes one RGB+depth frameset first; if
      // concurrent access is unavailable on a particular firmware/image, it
      // falls back to the existing helper + V4L2 auto-detection path.
      next.realsense = true;
      next.teleimager_rgb = false;
      next.auto_detect = false;
      next.rgb_source.clear();
      next.depth_source.clear();
    } else if (request.command != "stop") {
      result.error = "unknown_camera_command";
      return result;
    }

    if (request.command != "stop") {
      std::lock_guard<std::mutex> lock(mutex_);
      const bool same_configuration =
          running_.load() && next.realsense == options_.realsense &&
          next.auto_detect == options_.auto_detect &&
          next.teleimager_rgb == options_.teleimager_rgb &&
          (next.auto_detect ||
           (next.rgb_source == options_.rgb_source &&
            next.depth_source == options_.depth_source));
      if (same_configuration && (StreamFresh(rgb_) || StreamFresh(depth_))) {
        result.accepted = true;
        result.request_id = ++last_request_id_;
        last_request_key_ = request.request_key;
        last_result_ = result;
        return result;
      }
    }

    Stop();
    std::string start_error;
    if (request.command != "stop") {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        options_ = std::move(next);
        ApplyOptionsLocked();
      }
      result.accepted = Start(start_error);
      result.error = start_error;
    } else {
      std::lock_guard<std::mutex> lock(mutex_);
      options_.realsense = false;
      options_.auto_detect = false;
      options_.teleimager_rgb = false;
      options_.rgb_source.clear();
      options_.depth_source.clear();
      ApplyOptionsLocked();
      rgb_.configured = false;
      depth_.configured = false;
      result.accepted = true;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      result.request_id = ++last_request_id_;
      last_request_key_ = request.request_key;
      last_result_ = result;
    }
    return result;
  }

  CameraFrame GetFrame(const std::string& stream) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const StreamState* selected = nullptr;
    if (stream == "rgb") selected = &rgb_;
    if (stream == "depth") selected = &depth_;
    if (!selected) return {};
    return {StreamFresh(*selected) && !selected->jpeg.empty(),
            selected->sequence, selected->jpeg};
  }

  std::string SerializeStatus() const {
    std::lock_guard<std::mutex> lock(mutex_);
    Json::Value root(Json::objectValue);
    root["schema_version"] = 1;
    root["backend"] = options_.realsense ? "librealsense2" :
#ifdef G1_WEB_HAS_OPENCV
        options_.teleimager_rgb ? "teleimager-zmq+v4l2" : "opencv-v4l2";
#else
        "unavailable";
#endif
    root["mock"] = options_.mock;
    root["auto_detect"] = options_.auto_detect;
    root["teleimager_rgb"] = options_.teleimager_rgb;
    root["running"] = running_.load();
#ifdef G1_WEB_HAS_REALSENSE
    root["realsense_available"] = true;
#else
    root["realsense_available"] = false;
#endif
    root["width"] = options_.width;
    root["height"] = options_.height;
    root["fps"] = options_.fps;
    root["output_width"] = options_.output_width;
    root["output_height"] = options_.output_height;
    root["rgb_jpeg_quality"] = options_.rgb_jpeg_quality;
    root["depth_jpeg_quality"] = options_.depth_jpeg_quality;
    root["bandwidth_profile"] = "wifi_low_bandwidth";
    root["rgb"] = StreamJson(rgb_);
    root["depth"] = StreamJson(depth_);
    root["first_person_service"]["paused_by_web"] =
        first_person_paused_by_web_;
    root["first_person_service"]["was_active"] = first_person_was_active_;
    root["first_person_service"]["error"] = first_person_error_;
    root["realsense_shared_without_pause"] = realsense_shared_without_pause_;
    root["realsense_probe_error"] = realsense_probe_error_;
    root["notes"] =
        "D435i start first probes concurrent librealsense2 RGB+depth access without interrupting robot camera services; only unavailable/busy devices fall back to the privileged pause + V4L2 path.";
    return JsonText(root);
  }

 private:
  void CaptureWatchdog() {
    auto offline_since = std::chrono::steady_clock::now();
    while (running_.load()) {
      bool online = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        online = StreamFresh(rgb_) || StreamFresh(depth_);
      }
      if (online) {
        offline_since = std::chrono::steady_clock::now();
      } else if (std::chrono::steady_clock::now() - offline_since >=
                 std::chrono::seconds(8)) {
        running_.store(false);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        RestoreFirstPerson();
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  bool PauseFirstPerson(std::string& error) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (options_.mock) {
      first_person_was_active_ = true;
      first_person_paused_by_web_ = true;
      first_person_error_.clear();
      return true;
    }
    const int status_result = RunFirstPersonHelper("is-active");
    first_person_was_active_ = status_result == 0;
    if (!first_person_was_active_) {
      if (status_result == 3) {
        first_person_error_.clear();
        return true;
      }
      error = "first_person_status_failed";
      first_person_error_ = error;
      return false;
    }
    if (RunFirstPersonHelper("stop") != 0) {
      error = "first_person_stop_failed";
      first_person_error_ = error;
      return false;
    }
    first_person_paused_by_web_ = true;
    first_person_error_.clear();
    return true;
  }

  void RestoreFirstPerson() {
    bool restore = false;
    bool mock = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      restore = first_person_paused_by_web_ && first_person_was_active_;
      mock = options_.mock;
    }
    if (!restore) return;
    const int result = mock ? 0 : RunFirstPersonHelper("start");
    std::lock_guard<std::mutex> lock(mutex_);
    if (result == 0) {
      first_person_paused_by_web_ = false;
      first_person_error_.clear();
    } else {
      first_person_error_ = "first_person_restore_failed";
    }
  }

  static Json::Value StreamJson(const StreamState& stream) {
    Json::Value value(Json::objectValue);
    const bool fresh = StreamFresh(stream);
    value["configured"] = stream.configured;
    value["online"] = fresh;
    value["source"] = stream.source;
    value["error"] = !fresh && stream.online && stream.error.empty()
                         ? "camera_frame_stale"
                         : stream.error;
    value["sequence"] = Json::UInt64(stream.sequence);
    value["age_ms"] = Json::Int64(AgeMs(stream));
    value["jpeg_bytes"] = Json::UInt64(stream.jpeg.size());
    return value;
  }

#ifdef G1_WEB_HAS_OPENCV
  #ifdef G1_WEB_HAS_REALSENSE
  void RealSenseLoop() {
    try {
      rs2::pipeline pipeline;
      rs2::config config;
      config.enable_stream(RS2_STREAM_COLOR,
                           static_cast<int>(options_.width),
                           static_cast<int>(options_.height),
                           RS2_FORMAT_BGR8,
                           static_cast<int>(options_.fps));
      config.enable_stream(RS2_STREAM_DEPTH,
                           static_cast<int>(options_.width),
                           static_cast<int>(options_.height),
                           RS2_FORMAT_Z16,
                           static_cast<int>(options_.fps));
      pipeline.start(config);
      while (running_.load()) {
        rs2::frameset frames;
        if (!pipeline.poll_for_frames(&frames)) {
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
          continue;
        }
        const rs2::video_frame color = frames.get_color_frame();
        const rs2::depth_frame depth = frames.get_depth_frame();
        if (!color || !depth) continue;
        cv::Mat color_view(cv::Size(color.get_width(), color.get_height()),
                           CV_8UC3,
                           const_cast<void*>(color.get_data()),
                           cv::Mat::AUTO_STEP);
        cv::Mat depth_view(cv::Size(depth.get_width(), depth.get_height()),
                           CV_16UC1,
                           const_cast<void*>(depth.get_data()),
                           cv::Mat::AUTO_STEP);
        std::vector<unsigned char> rgb_jpeg;
        std::vector<unsigned char> depth_jpeg;
        if (EncodeJpeg(color_view, rgb_jpeg, options_.output_width,
                       options_.output_height,
                       options_.rgb_jpeg_quality)) {
          SetFrame(rgb_, std::move(rgb_jpeg));
        }
        if (EncodeJpeg(ColorizeDepth(depth_view), depth_jpeg,
                       options_.output_width, options_.output_height,
                       options_.depth_jpeg_quality)) {
          SetFrame(depth_, std::move(depth_jpeg));
        }
      }
      pipeline.stop();
    } catch (const rs2::error& exception) {
      SetStreamError(rgb_, exception.what());
      SetStreamError(depth_, exception.what());
      running_.store(false);
      RestoreFirstPerson();
    } catch (const std::exception& exception) {
      SetStreamError(rgb_, exception.what());
      SetStreamError(depth_, exception.what());
      running_.store(false);
      RestoreFirstPerson();
    }
  }
  #endif

  void CaptureLoop(const std::string& source, bool depth,
                   StreamState& target) {
    const auto frame_interval = std::chrono::milliseconds(
        std::max(1U, 1000U / std::max(1U, options_.fps)));
    while (running_.load()) {
      cv::VideoCapture capture;
      int index = 0;
      const bool opened = NumericSource(source, index)
                              ? capture.open(index, cv::CAP_V4L2)
                              : capture.open(source, cv::CAP_V4L2);
      if (!opened) {
        SetStreamError(target, "camera_open_failed");
        std::this_thread::sleep_for(std::chrono::milliseconds(350));
        continue;
      }
      capture.set(cv::CAP_PROP_FRAME_WIDTH, options_.width);
      capture.set(cv::CAP_PROP_FRAME_HEIGHT, options_.height);
      capture.set(cv::CAP_PROP_FPS, options_.fps);
      if (depth) capture.set(cv::CAP_PROP_CONVERT_RGB, 0);
      while (running_.load()) {
        const auto started = std::chrono::steady_clock::now();
        cv::Mat frame;
        if (!capture.read(frame) || frame.empty()) {
          SetStreamError(target, "camera_read_failed");
          break;
        }
        cv::Mat display = depth ? ColorizeDepth(frame) : frame;
        std::vector<unsigned char> jpeg;
        if (!EncodeJpeg(display, jpeg, options_.output_width,
                        options_.output_height,
                        depth ? options_.depth_jpeg_quality
                              : options_.rgb_jpeg_quality)) {
          SetStreamError(target, "jpeg_encode_failed");
          break;
        }
        SetFrame(target, std::move(jpeg));
        std::this_thread::sleep_until(started + frame_interval);
      }
      capture.release();
      if (running_.load()) std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
  }

#ifdef G1_WEB_HAS_ZMQ
  void TeleimagerRgbLoop() {
    void* context = zmq_ctx_new();
    if (!context) {
      SetStreamError(rgb_, "teleimager_zmq_context_failed");
      return;
    }
    void* socket = zmq_socket(context, ZMQ_SUB);
    if (!socket) {
      zmq_ctx_term(context);
      SetStreamError(rgb_, "teleimager_zmq_socket_failed");
      return;
    }
    const int timeout_ms = 500;
    const int linger_ms = 0;
    const int conflate = 1;
    zmq_setsockopt(socket, ZMQ_RCVTIMEO, &timeout_ms, sizeof(timeout_ms));
    zmq_setsockopt(socket, ZMQ_LINGER, &linger_ms, sizeof(linger_ms));
    zmq_setsockopt(socket, ZMQ_CONFLATE, &conflate, sizeof(conflate));
    zmq_setsockopt(socket, ZMQ_SUBSCRIBE, "", 0);
    if (zmq_connect(socket, "tcp://127.0.0.1:55555") != 0) {
      zmq_close(socket);
      zmq_ctx_term(context);
      SetStreamError(rgb_, "teleimager_zmq_connect_failed");
      return;
    }

    const auto frame_interval = std::chrono::milliseconds(
        std::max(1U, 1000U / std::max(1U, options_.fps)));
    while (running_.load()) {
      const auto started = std::chrono::steady_clock::now();
      zmq_msg_t message;
      zmq_msg_init(&message);
      const int received = zmq_msg_recv(&message, socket, 0);
      if (received >= 0) {
        const auto* data = static_cast<const unsigned char*>(zmq_msg_data(&message));
        const std::size_t size = zmq_msg_size(&message);
        if (size >= 4 && data[0] == 0xff && data[1] == 0xd8) {
          std::vector<unsigned char> source_jpeg(data, data + size);
          cv::Mat frame = cv::imdecode(source_jpeg, cv::IMREAD_COLOR);
          std::vector<unsigned char> jpeg;
          if (EncodeJpeg(frame, jpeg, options_.output_width,
                         options_.output_height, options_.rgb_jpeg_quality)) {
            SetFrame(rgb_, std::move(jpeg));
          } else {
            SetStreamError(rgb_, "teleimager_jpeg_decode_failed");
          }
        }
      }
      zmq_msg_close(&message);
      std::this_thread::sleep_until(started + frame_interval);
    }

    zmq_close(socket);
    zmq_ctx_term(context);
  }
#endif

  void CaptureDepthLoop(const std::string& source, StreamState& target) {
    struct Buffer {
      void* data{MAP_FAILED};
      std::size_t length{0};
    };
    while (running_.load()) {
      const int fd = ::open(source.c_str(), O_RDWR | O_NONBLOCK);
      if (fd < 0) {
        SetStreamError(target, "depth_open_failed");
        std::this_thread::sleep_for(std::chrono::milliseconds(350));
        continue;
      }

      v4l2_format format{};
      format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      format.fmt.pix.width = options_.width;
      format.fmt.pix.height = options_.height;
      format.fmt.pix.pixelformat = v4l2_fourcc('Z', '1', '6', ' ');
      format.fmt.pix.field = V4L2_FIELD_ANY;
      if (::ioctl(fd, VIDIOC_S_FMT, &format) != 0 ||
          format.fmt.pix.pixelformat != v4l2_fourcc('Z', '1', '6', ' ')) {
        ::close(fd);
        SetStreamError(target, "depth_format_failed");
        std::this_thread::sleep_for(std::chrono::milliseconds(350));
        continue;
      }

      v4l2_streamparm parameters{};
      parameters.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      parameters.parm.capture.timeperframe.numerator = 1;
      parameters.parm.capture.timeperframe.denominator = std::max(1U, options_.fps);
      ::ioctl(fd, VIDIOC_S_PARM, &parameters);

      v4l2_requestbuffers request{};
      request.count = 4;
      request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      request.memory = V4L2_MEMORY_MMAP;
      if (::ioctl(fd, VIDIOC_REQBUFS, &request) != 0 || request.count < 2) {
        ::close(fd);
        SetStreamError(target, "depth_buffers_failed");
        std::this_thread::sleep_for(std::chrono::milliseconds(350));
        continue;
      }

      std::vector<Buffer> buffers(request.count);
      bool ready = true;
      for (std::uint32_t index = 0; index < request.count; ++index) {
        v4l2_buffer buffer{};
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;
        buffer.index = index;
        if (::ioctl(fd, VIDIOC_QUERYBUF, &buffer) != 0) {
          ready = false;
          break;
        }
        buffers[index].length = buffer.length;
        buffers[index].data = ::mmap(nullptr, buffer.length,
                                     PROT_READ | PROT_WRITE, MAP_SHARED, fd,
                                     buffer.m.offset);
        if (buffers[index].data == MAP_FAILED ||
            ::ioctl(fd, VIDIOC_QBUF, &buffer) != 0) {
          ready = false;
          break;
        }
      }

      auto cleanup = [&] {
        v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ::ioctl(fd, VIDIOC_STREAMOFF, &type);
        for (auto& buffer : buffers) {
          if (buffer.data != MAP_FAILED) {
            ::munmap(buffer.data, buffer.length);
            buffer.data = MAP_FAILED;
          }
        }
        ::close(fd);
      };

      if (!ready) {
        cleanup();
        SetStreamError(target, "depth_mmap_failed");
        std::this_thread::sleep_for(std::chrono::milliseconds(350));
        continue;
      }

      v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      if (::ioctl(fd, VIDIOC_STREAMON, &type) != 0) {
        cleanup();
        SetStreamError(target, "depth_streamon_failed");
        std::this_thread::sleep_for(std::chrono::milliseconds(350));
        continue;
      }

      while (running_.load()) {
        pollfd poll_fd{fd, POLLIN, 0};
        const int polled = ::poll(&poll_fd, 1, 500);
        if (polled < 0) {
          SetStreamError(target, "depth_poll_failed");
          break;
        }
        if (polled == 0) continue;

        v4l2_buffer buffer{};
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;
        if (::ioctl(fd, VIDIOC_DQBUF, &buffer) != 0) continue;
        if (buffer.index >= buffers.size()) {
          SetStreamError(target, "depth_buffer_index_invalid");
          break;
        }

        const int width = static_cast<int>(format.fmt.pix.width);
        const int height = static_cast<int>(format.fmt.pix.height);
        const std::size_t row_bytes = format.fmt.pix.bytesperline != 0
                                          ? format.fmt.pix.bytesperline
                                          : static_cast<std::size_t>(width) * 2U;
        if (buffer.bytesused >= row_bytes * static_cast<std::size_t>(height)) {
          cv::Mat depth_view(height, width, CV_16UC1,
                             buffers[buffer.index].data, row_bytes);
          std::vector<unsigned char> jpeg;
          if (EncodeJpeg(ColorizeDepth(depth_view), jpeg,
                         options_.output_width, options_.output_height,
                         options_.depth_jpeg_quality)) {
            SetFrame(target, std::move(jpeg));
          } else {
            SetStreamError(target, "jpeg_encode_failed");
          }
        }
        if (::ioctl(fd, VIDIOC_QBUF, &buffer) != 0) {
          SetStreamError(target, "depth_requeue_failed");
          break;
        }
      }

      cleanup();
      if (running_.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
      }
    }
  }

  void MockLoop() {
    const auto started = std::chrono::steady_clock::now();
    const auto frame_interval = std::chrono::milliseconds(
        std::max(1U, 1000U / std::max(1U, options_.fps)));
    while (running_.load()) {
      const auto frame_started = std::chrono::steady_clock::now();
      const double elapsed =
          std::chrono::duration<double>(frame_started - started).count();
      cv::Mat rgb(static_cast<int>(options_.height),
                  static_cast<int>(options_.width), CV_8UC3);
      for (int y = 0; y < rgb.rows; ++y) {
        auto* row = rgb.ptr<cv::Vec3b>(y);
        for (int x = 0; x < rgb.cols; ++x) {
          row[x] = cv::Vec3b(
              static_cast<unsigned char>(35 + 30 * std::sin(x * 0.012 + elapsed)),
              static_cast<unsigned char>(55 + y * 120 / std::max(1, rgb.rows)),
              static_cast<unsigned char>(80 + x * 120 / std::max(1, rgb.cols)));
        }
      }
      const int marker_x = static_cast<int>(rgb.cols *
          (0.5 + 0.32 * std::sin(elapsed * 0.7)));
      cv::circle(rgb, {marker_x, rgb.rows / 2}, 42, {82, 221, 255}, 3,
                 cv::LINE_AA);
      cv::line(rgb, {rgb.cols / 2 - 60, rgb.rows / 2},
               {rgb.cols / 2 + 60, rgb.rows / 2}, {180, 230, 245}, 1,
               cv::LINE_AA);
      cv::line(rgb, {rgb.cols / 2, rgb.rows / 2 - 60},
               {rgb.cols / 2, rgb.rows / 2 + 60}, {180, 230, 245}, 1,
               cv::LINE_AA);
      cv::putText(rgb, "G1 RGB MOCK", {24, 38}, cv::FONT_HERSHEY_SIMPLEX,
                  0.72, {230, 245, 255}, 2, cv::LINE_AA);

      cv::Mat depth16(rgb.rows, rgb.cols, CV_16UC1);
      for (int y = 0; y < depth16.rows; ++y) {
        auto* row = depth16.ptr<std::uint16_t>(y);
        for (int x = 0; x < depth16.cols; ++x) {
          const double wave = 550.0 * std::sin(x * 0.016 + elapsed) +
                              350.0 * std::cos(y * 0.021 - elapsed * 0.5);
          row[x] = static_cast<std::uint16_t>(
              std::clamp(2200.0 + wave, 250.0, 5000.0));
        }
      }
      cv::Mat depth_color = ColorizeDepth(depth16);
      cv::putText(depth_color, "G1 DEPTH MOCK 0-5m", {24, 38},
                  cv::FONT_HERSHEY_SIMPLEX, 0.72, {255, 255, 255}, 2,
                  cv::LINE_AA);

      std::vector<unsigned char> rgb_jpeg;
      std::vector<unsigned char> depth_jpeg;
      if (EncodeJpeg(rgb, rgb_jpeg, options_.output_width,
                     options_.output_height, options_.rgb_jpeg_quality)) {
        SetFrame(rgb_, std::move(rgb_jpeg));
      }
      if (EncodeJpeg(depth_color, depth_jpeg, options_.output_width,
                     options_.output_height, options_.depth_jpeg_quality)) {
        SetFrame(depth_, std::move(depth_jpeg));
      }
      std::this_thread::sleep_until(frame_started + frame_interval);
    }
  }
#endif

  void SetStreamError(StreamState& target, const std::string& error) {
    std::lock_guard<std::mutex> lock(mutex_);
    target.online = false;
    target.error = error;
  }

  void SetFrame(StreamState& target, std::vector<unsigned char> jpeg) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_.load()) return;
    target.jpeg = std::move(jpeg);
    target.online = true;
    target.error.clear();
    target.updated = std::chrono::steady_clock::now();
    ++target.sequence;
  }

  CameraOptions options_;
  mutable std::mutex mutex_;
  StreamState rgb_;
  StreamState depth_;
  std::atomic<bool> running_{false};
  std::thread mock_thread_;
  std::thread realsense_thread_;
  std::thread rgb_thread_;
  std::thread depth_thread_;
  std::thread capture_watchdog_thread_;
  std::mutex command_mutex_;
  std::uint64_t last_request_id_{0};
  std::string last_request_key_;
  CameraResult last_result_;
  bool first_person_was_active_{false};
  bool first_person_paused_by_web_{false};
  std::string first_person_error_;
  bool realsense_shared_without_pause_{false};
  std::string realsense_probe_error_;
};

CameraService::CameraService(CameraOptions options)
    : impl_(std::make_unique<Impl>(std::move(options))) {}

CameraService::~CameraService() = default;

bool CameraService::Start(std::string& error) { return impl_->Start(error); }

void CameraService::Stop() { impl_->Stop(); }

CameraResult CameraService::Submit(const CameraRequest& request) {
  return impl_->Submit(request);
}

CameraFrame CameraService::GetFrame(const std::string& stream) const {
  return impl_->GetFrame(stream);
}

std::string CameraService::SerializeStatus() const {
  return impl_->SerializeStatus();
}

}  // namespace g1_web
