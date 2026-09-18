#include "g1_web/camera_service.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cerrno>
#include <cstdint>
#include <cmath>
#include <cstdlib>
#include <mutex>
#include <limits>
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
#include <unitree/robot/b2/robot_state/robot_state_client.hpp>

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

bool CameraFrameFresh(bool online, std::int64_t age_ms) {
  return online && age_ms >= 0 && age_ms <= 2000;
}

bool CameraTransportBackendAvailable(CameraTransport transport,
                                     bool opencv_available,
                                     bool gstreamer_available) {
  if (!opencv_available) return false;
  if (transport == CameraTransport::kRtpH264Udp) return gstreamer_available;
  return true;
}

std::string BuildFixedRtpH264UdpPipeline(const CameraStreamSpec& spec) {
  if (spec.transport != CameraTransport::kRtpH264Udp || spec.port == 0 ||
      spec.port > 65535) {
    return {};
  }
  return "udpsrc port=" + std::to_string(spec.port) +
         " ! application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96"
         " ! watchdog timeout=2000"
         " ! rtph264depay ! avdec_h264 ! videoconvert ! video/x-raw,format=BGR"
         " ! appsink drop=true max-buffers=1 sync=false wait-on-eos=false";
}

bool ValidateDecodedCameraFrame(const CameraStreamSpec& spec,
                                int width, int height,
                                std::string& error) {
  if (width <= 0 || height <= 0) {
    error = "camera_decode_failed";
    return false;
  }
  if (spec.width != 0 && spec.height != 0) {
    const bool expected =
        width == static_cast<int>(spec.width) &&
        height == static_cast<int>(spec.height);
    const bool rotated =
        spec.allow_runtime_dimensions &&
        width == static_cast<int>(spec.height) &&
        height == static_cast<int>(spec.width);
    if (!expected && !rotated) {
      error = "camera_frame_size_mismatch";
      return false;
    }
  }
  error.clear();
  return true;
}

bool ValidateRaw16DepthFrame(const CameraStreamSpec& spec,
                             unsigned int actual_width,
                             unsigned int actual_height,
                             std::size_t bytes_read,
                             std::string& error) {
  if (spec.transport != CameraTransport::kV4l2Raw16 || spec.width == 0 ||
      spec.height == 0 || actual_width != spec.width ||
      actual_height != spec.height) {
    error = "depth_frame_size_mismatch";
    return false;
  }
  const std::size_t expected = static_cast<std::size_t>(spec.width) *
                               static_cast<std::size_t>(spec.height) * 2U;
  if (bytes_read < expected) {
    error = "depth_short_frame";
    return false;
  }
  if (bytes_read != expected) {
    error = "depth_frame_size_invalid";
    return false;
  }
  error.clear();
  return true;
}

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
  std::uint64_t continuous_frames{0};
  int decoded_width{0};
  int decoded_height{0};
  std::vector<unsigned char> jpeg;
  std::chrono::steady_clock::time_point updated{};
  std::chrono::steady_clock::time_point continuous_since{};
};

std::int64_t AgeMs(const StreamState& stream) {
  if (!stream.online) return -1;
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::steady_clock::now() - stream.updated)
      .count();
}

bool StreamFresh(const StreamState& stream) {
  return CameraFrameFresh(stream.online, AgeMs(stream));
}

bool StreamSustainedFresh(const StreamState& stream) {
  if (!StreamFresh(stream) || stream.continuous_frames < 3) return false;
  return std::chrono::steady_clock::now() - stream.continuous_since >=
         std::chrono::seconds(1);
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

std::string NormalizeCameraServiceName(const std::string& name) {
  std::string normalized;
  normalized.reserve(name.size());
  for (const unsigned char value : name) {
    if (std::isalnum(value) != 0) {
      normalized.push_back(static_cast<char>(std::tolower(value)));
    }
  }
  return normalized;
}

enum class PrivilegedCameraAction {
  kDepthStart,
  kDepthStop,
  kDepthIsActive,
};

const char* PrivilegedCameraActionName(PrivilegedCameraAction action) {
  switch (action) {
    case PrivilegedCameraAction::kDepthStart:
      return "depth-start";
    case PrivilegedCameraAction::kDepthStop:
      return "depth-stop";
    case PrivilegedCameraAction::kDepthIsActive:
      return "depth-is-active";
  }
  return "";
}

int RunFixedCameraHelper(const std::string& helper,
                         PrivilegedCameraAction action) {
  if (helper.empty() || helper.front() != '/' ||
      helper.find_first_of(" \t\r\n;|&$`<>") != std::string::npos) {
    return -1;
  }
  const std::string action_name = PrivilegedCameraActionName(action);
  if (action_name.empty()) return -1;
  std::string command = "sudo -n " + helper + " " + action_name;
  command += " >/dev/null 2>&1";
  const int result = std::system(command.c_str());
  if (result == -1 || !WIFEXITED(result)) return -1;
  return WEXITSTATUS(result);
}

int RunFirstPersonHelper(const std::string& helper, const char* action) {
  if (helper.empty()) return -1;
  std::string command = "sudo -n " + helper + " ";
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

cv::Mat ColorizeR1DocumentDepth(const cv::Mat& frame) {
  if (frame.type() != CV_16UC1 || frame.empty()) return {};

  std::array<std::uint32_t, 65536> histogram{};
  std::size_t valid_count = 0;
  for (int y = 0; y < frame.rows; ++y) {
    const auto* row = frame.ptr<std::uint16_t>(y);
    for (int x = 0; x < frame.cols; ++x) {
      const std::uint16_t value = row[x];
      if (value != 0 && value != std::numeric_limits<std::uint16_t>::max()) {
        ++histogram[value];
        ++valid_count;
      }
    }
  }

  cv::Mat normalized(frame.rows, frame.cols, CV_8UC1, cv::Scalar(0));
  cv::Mat invalid(frame.rows, frame.cols, CV_8UC1, cv::Scalar(0));
  if (valid_count != 0) {
    const auto rank_value = [&](std::size_t rank) {
      std::size_t cumulative = 0;
      for (std::size_t value = 1; value < histogram.size() - 1; ++value) {
        const std::size_t next =
            cumulative + static_cast<std::size_t>(histogram[value]);
        if (rank < next) return static_cast<double>(value);
        cumulative = next;
      }
      return 0.0;
    };
    const auto percentile = [&](double fraction) {
      const double position =
          fraction * static_cast<double>(valid_count - 1);
      const auto lower = static_cast<std::size_t>(std::floor(position));
      const auto upper = static_cast<std::size_t>(std::ceil(position));
      const double weight = position - static_cast<double>(lower);
      const double lower_value = rank_value(lower);
      const double upper_value = rank_value(upper);
      return lower_value * (1.0 - weight) + upper_value * weight;
    };
    const double min_value = percentile(0.01);
    const double max_value = percentile(0.99);
    for (int y = 0; y < frame.rows; ++y) {
      const auto* input = frame.ptr<std::uint16_t>(y);
      auto* output = normalized.ptr<unsigned char>(y);
      auto* mask = invalid.ptr<unsigned char>(y);
      for (int x = 0; x < frame.cols; ++x) {
        const std::uint16_t value = input[x];
        if (value == 0 || value == std::numeric_limits<std::uint16_t>::max()) {
          mask[x] = 255;
          continue;
        }
        if (max_value > min_value) {
          const double unit = std::clamp(
              (static_cast<double>(value) - min_value) /
                  (max_value - min_value),
              0.0, 1.0);
          output[x] = static_cast<unsigned char>(std::lround(unit * 255.0));
        }
      }
    }
  } else {
    invalid.setTo(cv::Scalar(255));
  }

  cv::Mat colorized;
  cv::applyColorMap(normalized, colorized, cv::COLORMAP_JET);
  colorized.setTo(cv::Scalar(20, 20, 20), invalid);
  return colorized;
}

bool OpenCvHasGStreamer() {
  const std::string build = cv::getBuildInformation();
  const auto marker = build.find("GStreamer:");
  if (marker == std::string::npos) return false;
  const auto end = build.find('\n', marker);
  const std::string line = build.substr(marker, end - marker);
  return line.find("YES") != std::string::npos;
}
#endif

}  // namespace

class CameraService::Impl {
 public:
  Impl(CameraOptions options, const IDeviceCapabilityPolicy& device_policy)
      : options_(std::move(options)), device_policy_(device_policy) {
    ApplyOptionsLocked();
  }

  void ApplyOptionsLocked() {
    rgb_ = {};
    depth_ = {};
    const auto& camera_policy = device_policy_.Camera();
    const auto* fixed_rgb = FindCameraStream(camera_policy, CameraStreamRole::kRgb);
    const auto* fixed_depth =
        FindCameraStream(camera_policy, CameraStreamRole::kDepth);
    if (!camera_policy.streams.empty()) {
      rgb_.configured =
          options_.rgb_enabled &&
          (options_.mock ||
           (fixed_rgb && options_.rgb_source == fixed_rgb->fixed_source));
      depth_.configured =
          options_.depth_enabled &&
          (options_.mock ||
           (fixed_depth && options_.depth_source == fixed_depth->fixed_source));
      rgb_.source = options_.mock ? "mock://rgb"
                                  : fixed_rgb ? fixed_rgb->fixed_source : "";
      depth_.source = options_.mock ? "mock://depth"
                                    : fixed_depth ? fixed_depth->fixed_source : "";
      return;
    }
    rgb_.configured =
        options_.rgb_enabled &&
        (options_.mock || options_.realsense || options_.teleimager_rgb ||
         !options_.rgb_source.empty());
    rgb_.source = options_.mock ? "mock://rgb" :
                  options_.realsense ? "realsense://color" :
                  options_.teleimager_rgb ? "zmq://127.0.0.1:55555" :
                  options_.rgb_source;
    depth_.configured =
        options_.depth_enabled &&
        (options_.mock || options_.realsense || !options_.depth_source.empty());
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
    error = device_policy_.Camera().streams.empty()
                ? "opencv_not_available"
                : "camera_backend_unavailable";
    std::lock_guard<std::mutex> lock(mutex_);
    rgb_.error = error;
    depth_.error = error;
    return false;
#else
    const auto& camera_policy = device_policy_.Camera();
    const bool fixed_policy = !camera_policy.streams.empty();
    if (fixed_policy && !options_.mock) {
      for (const auto role : {CameraStreamRole::kRgb, CameraStreamRole::kDepth}) {
        const auto* spec = FindCameraStream(camera_policy, role);
        const bool configured = role == CameraStreamRole::kRgb
                                    ? rgb_.configured
                                    : depth_.configured;
        if (configured && spec &&
            !CameraTransportBackendAvailable(
                spec->transport, true, OpenCvHasGStreamer())) {
          running_.store(false);
          error = "camera_backend_unavailable";
          std::lock_guard<std::mutex> lock(mutex_);
          if (rgb_.configured) rgb_.error = error;
          if (depth_.configured) depth_.error = error;
          return false;
        }
      }
    }
    if (!rgb_.configured && !depth_.configured && !options_.auto_detect) {
      running_.store(false);
      error = "camera_sources_not_configured";
      return false;
    }
    if (fixed_policy && !options_.mock &&
        !PrepareExternalCameraServices(depth_.configured, error)) {
      running_.store(false);
      std::lock_guard<std::mutex> lock(mutex_);
      if (rgb_.configured) rgb_.error = error;
      if (depth_.configured) depth_.error = error;
      return false;
    }

    bool shared_realsense = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      realsense_shared_without_pause_ = false;
      if (!options_.realsense) realsense_probe_error_.clear();
    }
#ifdef G1_WEB_HAS_REALSENSE
    if (!fixed_policy && options_.realsense && !options_.mock) {
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

    if (!fixed_policy && !shared_realsense && !PauseFirstPerson(error)) {
      running_.store(false);
      std::lock_guard<std::mutex> lock(mutex_);
      rgb_.error = error;
      depth_.error = error;
      return false;
    }
    if (!fixed_policy && options_.auto_detect && !options_.mock &&
        !options_.realsense) {
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
    } else if (fixed_policy) {
      const auto* rgb_spec =
          FindCameraStream(camera_policy, CameraStreamRole::kRgb);
      const auto* depth_spec =
          FindCameraStream(camera_policy, CameraStreamRole::kDepth);
      if (rgb_.configured && rgb_spec) {
        rgb_thread_ =
            std::thread([this, spec = *rgb_spec] { CaptureRtpH264Loop(spec, rgb_); });
      }
      if (depth_.configured && depth_spec) {
        depth_thread_ = std::thread(
            [this, spec = *depth_spec] { CaptureRaw16DepthLoop(spec, depth_); });
      }
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

  void Stop(bool restore_external_services = true,
            bool stop_depth_receiver = true) {
    running_.store(false);
#ifdef G1_WEB_HAS_OPENCV
    {
      std::lock_guard<std::mutex> lock(rtp_capture_mutex_);
      if (active_rtp_capture_) active_rtp_capture_->release();
    }
#endif
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
    if (restore_external_services) {
      RestoreExternalCameraServices();
    } else if (stop_depth_receiver) {
      std::lock_guard<std::mutex> service_lock(service_mutex_);
      std::string stop_error = external_service_error_;
      StopDepthReceiverLocked(stop_error);
      external_service_error_ = stop_error;
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
    const auto& camera_policy = device_policy_.Camera();
    const bool fixed_policy = !camera_policy.streams.empty();
    const bool stop_all = request.command == "stop";
    const bool stop_rgb = request.command == "stop_rgb";
    const bool stop_depth = request.command == "stop_depth";
    const bool stop_stream = stop_rgb || stop_depth;
    if (stop_stream && !fixed_policy) {
      result.error = "unknown_camera_command";
      return result;
    }

    if (fixed_policy &&
        (request.command == "start_v4l2" ||
         request.command == "start_realsense")) {
      if (!request.rgb_source.empty() || !request.depth_source.empty()) {
        result.error = "camera_source_override_forbidden";
        return result;
      }
      const auto* rgb_spec =
          FindCameraStream(camera_policy, CameraStreamRole::kRgb);
      const auto* depth_spec =
          FindCameraStream(camera_policy, CameraStreamRole::kDepth);
      if (!rgb_spec || !depth_spec) {
        result.error = "camera_policy_incomplete";
        return result;
      }
      next.realsense = false;
      next.teleimager_rgb = false;
      next.auto_detect = false;
      if (request.command == "start_v4l2") {
        // R1 uses the right-eye RGB stream, so it can coexist with the
        // left-eye depth receiver. Keep an already configured depth stream.
        next.rgb_enabled = true;
        next.rgb_source = rgb_spec->fixed_source;
      } else {
        // Depth is additive for the fixed R1 policy. Keep an already
        // configured right-eye RGB stream instead of switching it off.
        next.depth_enabled = true;
        next.depth_source = depth_spec->fixed_source;
      }
    } else if (request.command == "start_v4l2") {
      if (!SafeVideoDevice(request.rgb_source) ||
          !SafeVideoDevice(request.depth_source)) {
        result.error = "invalid_camera_source";
        return result;
      }
      next.realsense = false;
      next.teleimager_rgb = false;
      next.auto_detect = request.rgb_source.empty() || request.depth_source.empty();
      next.rgb_enabled = true;
      next.depth_enabled = true;
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
      next.rgb_enabled = true;
      next.depth_enabled = true;
      next.rgb_source.clear();
      next.depth_source.clear();
    } else if (stop_rgb) {
      next.rgb_enabled = false;
      next.rgb_source.clear();
    } else if (stop_depth) {
      next.depth_enabled = false;
      next.depth_source.clear();
    } else if (!stop_all) {
      result.error = "unknown_camera_command";
      return result;
    }

    const bool stop_command = stop_all || stop_stream;
    if (!stop_command) {
      std::lock_guard<std::mutex> lock(mutex_);
      const bool same_configuration =
          running_.load() && next.realsense == options_.realsense &&
          next.auto_detect == options_.auto_detect &&
          next.teleimager_rgb == options_.teleimager_rgb &&
          next.rgb_enabled == options_.rgb_enabled &&
          next.depth_enabled == options_.depth_enabled &&
          (next.auto_detect ||
           (next.rgb_source == options_.rgb_source &&
            next.depth_source == options_.depth_source));
      const bool streams_ready =
          fixed_policy
              ? ((rgb_.configured || depth_.configured) &&
                 (!rgb_.configured || StreamFresh(rgb_)) &&
                 (!depth_.configured || StreamFresh(depth_)))
              : (StreamFresh(rgb_) || StreamFresh(depth_));
      if (same_configuration && streams_ready) {
        result.accepted = true;
        result.request_id = ++last_request_id_;
        last_request_key_ = request.request_key;
        last_result_ = result;
        return result;
      }
    }

    const auto* fixed_rgb =
        FindCameraStream(camera_policy, CameraStreamRole::kRgb);
    const auto* fixed_depth =
        FindCameraStream(camera_policy, CameraStreamRole::kDepth);
    const bool remaining_rgb =
        fixed_policy && next.rgb_enabled &&
        (next.mock ||
         (fixed_rgb && next.rgb_source == fixed_rgb->fixed_source));
    const bool remaining_depth =
        fixed_policy && next.depth_enabled &&
        (next.mock ||
         (fixed_depth && next.depth_source == fixed_depth->fixed_source));
    const bool preserve_external_preparation =
        fixed_policy && !stop_all && running_.load() &&
        (remaining_rgb || remaining_depth);
    const bool preserve_depth_receiver =
        preserve_external_preparation && remaining_depth;
    Stop(!preserve_external_preparation, !preserve_depth_receiver);

    std::string start_error;
    if (!stop_all) {
      bool has_remaining_stream = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        options_ = next;
        ApplyOptionsLocked();
        has_remaining_stream = rgb_.configured || depth_.configured;
      }
      if (has_remaining_stream) {
        result.accepted = Start(start_error);
        result.error = start_error;
        if (!result.accepted && preserve_external_preparation) {
          RestoreExternalCameraServices();
        }
      } else {
        result.accepted = true;
      }
    } else {
      std::lock_guard<std::mutex> lock(mutex_);
      options_.realsense = false;
      options_.auto_detect = false;
      options_.teleimager_rgb = false;
      options_.rgb_enabled = false;
      options_.depth_enabled = false;
      options_.rgb_source.clear();
      options_.depth_source.clear();
      ApplyOptionsLocked();
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
    std::scoped_lock lock(mutex_, service_mutex_);
    const auto& camera_policy = device_policy_.Camera();
    const bool fixed_policy = !camera_policy.streams.empty();
    Json::Value root(Json::objectValue);
    root["schema_version"] = 1;
#ifdef G1_WEB_HAS_OPENCV
    const bool fixed_backend_available =
        !fixed_policy || OpenCvHasGStreamer();
    root["backend"] =
        fixed_policy
            ? (fixed_backend_available ? "opencv-gstreamer+v4l2-raw16"
                                       : "unavailable")
            : options_.realsense
                  ? "librealsense2"
                  : options_.teleimager_rgb ? "teleimager-zmq+v4l2"
                                            : "opencv-v4l2";
#else
    root["backend"] = "unavailable";
#endif
    root["mock"] = options_.mock;
    root["auto_detect"] = options_.auto_detect;
    root["teleimager_rgb"] = options_.teleimager_rgb;
    root["fixed_policy"] = fixed_policy;
    root["provider"] = camera_policy.provider;
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
    root["service_version_status"] = camera_policy.service_version_status;
    const bool shared_services_ready =
        !camera_policy.manage_external_services ||
        (video_hub_status_raw_ == 1 && stereo_patch_status_raw_ == 0);
    const bool depth_receiver_ready =
        !depth_.configured || !camera_policy.manage_privileged_receiver ||
        depth_receiver_active_;
    root["external_preparation_status"] =
        options_.mock
            ? "mock"
            : !external_service_error_.empty()
                  ? "error"
                  : running_.load() && shared_services_ready &&
                            depth_receiver_ready
                        ? "ready"
                        : "idle";
    root["rgb"] = StreamJson(rgb_);
    root["depth"] = StreamJson(depth_);

    const auto annotate_stream = [&](CameraStreamRole role, Json::Value& value) {
      const auto* spec = FindCameraStream(camera_policy, role);
      if (!spec) return;
      value["fixed_source"] = spec->fixed_source;
      value["port"] = spec->port;
      value["transport"] = CameraTransportName(spec->transport);
      value["expected_width"] = spec->width;
      value["expected_height"] = spec->height;
      value["expected_fps"] = spec->fps;
      value["runtime_dimensions_allowed"] = spec->allow_runtime_dimensions;
    };
    annotate_stream(CameraStreamRole::kRgb, root["rgb"]);
    annotate_stream(CameraStreamRole::kDepth, root["depth"]);

    root["first_person_service"]["paused_by_web"] =
        first_person_paused_by_web_;
    root["first_person_service"]["was_active"] = first_person_was_active_;
    root["first_person_service"]["error"] = first_person_error_;
    root["first_person_service"]["managed_by_web"] =
        camera_policy.manage_first_person_service;
    root["realsense_shared_without_pause"] = realsense_shared_without_pause_;
    root["realsense_probe_error"] = realsense_probe_error_;
    root["device"]["provider"] = camera_policy.provider;
    root["device"]["model"] = camera_policy.camera_model;
    root["device"]["helper"] = camera_policy.first_person_helper;
    root["device"]["client_source_override_allowed"] =
        camera_policy.allow_client_source_override;
    root["device"]["manage_external_services"] =
        camera_policy.manage_external_services;
    root["device"]["manage_privileged_receiver"] =
        camera_policy.manage_privileged_receiver;
    Json::Value dependencies(Json::arrayValue);
    for (const auto& dependency :
         camera_policy.exclusive_service_dependencies) {
      dependencies.append(dependency);
    }
    root["device"]["exclusive_service_dependencies"] = dependencies;
    Json::Value requirements(Json::arrayValue);
    for (const auto& requirement : camera_policy.external_service_requirements) {
      requirements.append(requirement);
    }
    root["device"]["external_service_requirements"] = requirements;
    Json::Value streams(Json::arrayValue);
    for (const auto& spec : camera_policy.streams) {
      Json::Value stream(Json::objectValue);
      stream["role"] = CameraStreamRoleName(spec.role);
      stream["transport"] = CameraTransportName(spec.transport);
      stream["fixed_source"] = spec.fixed_source;
      stream["port"] = spec.port;
      stream["width"] = spec.width;
      stream["height"] = spec.height;
      stream["fps"] = spec.fps;
      streams.append(stream);
    }
    root["device"]["streams"] = streams;
    Json::Value related_services(Json::arrayValue);
    for (const auto& service : camera_related_services_) {
      related_services.append(service);
    }
    root["external_services"]["related_services"] = related_services;
    root["external_services"]["service_list_api_result"] =
        service_list_api_result_;
    root["external_services"]["error"] = external_service_error_;
    root["external_services"]["video_hub"]["name"] =
        video_hub_service_name_;
    root["external_services"]["video_hub"]["status_raw"] =
        video_hub_status_raw_;
    root["external_services"]["video_hub"]["protect_raw"] =
        video_hub_protect_raw_;
    root["external_services"]["video_hub"]["switch_api_result"] =
        video_hub_switch_api_result_;
    root["external_services"]["video_hub"]["changed_by_web"] =
        video_hub_changed_by_web_;
    root["external_services"]["stereo_patch_pc1"]["name"] =
        stereo_patch_service_name_;
    root["external_services"]["stereo_patch_pc1"]["status_raw"] =
        stereo_patch_status_raw_;
    root["external_services"]["stereo_patch_pc1"]["protect_raw"] =
        stereo_patch_protect_raw_;
    root["external_services"]["stereo_patch_pc1"]["switch_api_result"] =
        stereo_patch_switch_api_result_;
    root["external_services"]["stereo_patch_pc1"]["changed_by_web"] =
        stereo_patch_changed_by_web_;
    root["external_services"]["depth_receiver"]["helper"] =
        camera_policy.privileged_receiver_helper;
    root["external_services"]["depth_receiver"]["status_result"] =
        depth_helper_status_result_;
    root["external_services"]["depth_receiver"]["start_result"] =
        depth_helper_start_result_;
    root["external_services"]["depth_receiver"]["stop_result"] =
        depth_helper_stop_result_;
    root["external_services"]["depth_receiver"]["active"] =
        depth_receiver_active_;
    root["external_services"]["depth_receiver"]["started_by_web"] =
        depth_started_by_web_;
    root["notes"] =
        fixed_policy
            ? "Web uses fixed RobotState service switching and a fixed privileged depth helper; no client-supplied service or command is executed."
            : "Direct camera access is probed before the deployment helper is used for exclusive V4L2 fallback.";
    return JsonText(root);
  }

  std::vector<DeviceCapabilityRuntime> DeviceCapabilities() const {
    std::scoped_lock lock(mutex_, service_mutex_);
#ifdef G1_WEB_HAS_OPENCV
    constexpr bool kCameraBackendAvailable = true;
    const bool gstreamer_available = OpenCvHasGStreamer();
#else
    constexpr bool kCameraBackendAvailable = false;
    const bool gstreamer_available = false;
#endif
    const auto& camera_policy = device_policy_.Camera();
    const bool fixed_policy = !camera_policy.streams.empty();
    const auto runtime_for = [&](CapabilityKey key) {
      DeviceCapabilityRuntime runtime;
      runtime.key = key;
      const bool rgb = key == CapabilityKey::kCameraRgb;
      const auto& stream = rgb ? rgb_ : depth_;
      const auto* spec = fixed_policy
                             ? FindCameraStream(
                                   camera_policy,
                                   rgb ? CameraStreamRole::kRgb
                                       : CameraStreamRole::kDepth)
                             : nullptr;
      const bool fresh = StreamFresh(stream);
      runtime.hardware_presence =
          options_.mock || fresh ? HardwarePresence::kPresent
                                 : device_policy_.DetectHardware(key);
      runtime.deployment_enabled = !camera_policy.camera_model.empty();
      runtime.service_version_status = camera_policy.service_version_status;

      if (fixed_policy && spec) {
        runtime.service_available =
            options_.mock ||
            CameraTransportBackendAvailable(spec->transport,
                                            kCameraBackendAvailable,
                                            gstreamer_available);
        runtime.receiver_available = runtime.service_available;
        if (!rgb && !fresh && camera_policy.manage_privileged_receiver &&
            external_service_error_.rfind("depth_receiver_helper_unavailable", 0) == 0) {
          runtime.receiver_available = false;
        }
        runtime.external_preparation_ready = options_.mock || fresh;
        if (!options_.mock && StreamSustainedFresh(stream)) {
          runtime.verification_level = VerificationLevel::kReadonlyVerified;
        }
        if (!runtime.service_available) {
          runtime.reason = "camera_backend_unavailable";
        } else if (!external_service_error_.empty() && !fresh) {
          runtime.reason = external_service_error_;
        } else if (!stream.error.empty() && !fresh) {
          const bool external_not_ready =
              stream.error == "camera_open_failed" ||
              stream.error == "depth_open_failed" ||
              stream.error == "depth_format_query_failed" ||
              stream.error == "camera_read_failed" ||
              stream.error == "depth_read_failed";
          runtime.reason = external_not_ready
                               ? "camera_external_preparation_required"
                               : stream.error;
        } else if (runtime.hardware_presence == HardwarePresence::kAbsent) {
          runtime.reason = rgb ? "camera_stream_not_received"
                               : "camera_depth_receiver_not_ready";
        } else if (!fresh) {
          runtime.reason = "camera_external_preparation_required";
        }
      } else {
        runtime.service_available = kCameraBackendAvailable;
        if (!runtime.service_available) {
          runtime.reason = "opencv_not_available";
        } else if (runtime.hardware_presence == HardwarePresence::kAbsent) {
          runtime.reason = "camera_device_not_detected";
        }
      }
      if (!runtime.deployment_enabled && runtime.reason.empty()) {
        runtime.reason = "camera_deployment_not_configured";
      }
      return runtime;
    };
    return {runtime_for(CapabilityKey::kCameraRgb),
            runtime_for(CapabilityKey::kCameraDepth)};
  }

 private:
  bool RefreshExternalCameraServicesLocked(std::string& error) {
    const auto& policy = device_policy_.Camera();
    if (!policy.manage_external_services) {
      error.clear();
      return true;
    }
    try {
      if (!robot_state_client_) {
        robot_state_client_ =
            std::make_unique<unitree::robot::b2::RobotStateClient>();
        robot_state_client_->SetTimeout(10.0F);
        robot_state_client_->Init();
      }
      std::vector<unitree::robot::b2::ServiceState> services;
      service_list_api_result_ = robot_state_client_->ServiceList(services);
      if (service_list_api_result_ != 0) {
        error = "camera_service_list_failed:" +
                std::to_string(service_list_api_result_);
        external_service_error_ = error;
        return false;
      }

      std::vector<const unitree::robot::b2::ServiceState*> video_matches;
      std::vector<const unitree::robot::b2::ServiceState*> stereo_matches;
      camera_related_services_.clear();
      for (const auto& service : services) {
        const std::string normalized = NormalizeCameraServiceName(service.name);
        if (normalized.find("video") != std::string::npos ||
            normalized.find("stereo") != std::string::npos) {
          camera_related_services_.push_back(service.name);
        }
        if (!policy.conflicting_service_match.empty() &&
            normalized == policy.conflicting_service_match) {
          video_matches.push_back(&service);
        }
        if (!policy.required_service_match.empty() &&
            normalized == policy.required_service_match) {
          stereo_matches.push_back(&service);
        }
      }
      if (video_matches.size() != 1U) {
        error = video_matches.empty() ? "video_hub_service_not_found"
                                      : "video_hub_service_ambiguous";
        external_service_error_ = error;
        return false;
      }
      if (stereo_matches.size() != 1U) {
        error = stereo_matches.empty() ? "stereo_patch_pc1_service_not_found"
                                       : "stereo_patch_pc1_service_ambiguous";
        external_service_error_ = error;
        return false;
      }

      video_hub_service_name_ = video_matches.front()->name;
      video_hub_status_raw_ = video_matches.front()->status;
      video_hub_protect_raw_ = video_matches.front()->protect;
      stereo_patch_service_name_ = stereo_matches.front()->name;
      stereo_patch_status_raw_ = stereo_matches.front()->status;
      stereo_patch_protect_raw_ = stereo_matches.front()->protect;
      external_service_error_.clear();
      error.clear();
      return true;
    } catch (const std::exception& exception) {
      error = std::string("camera_robot_state_failed:") + exception.what();
      external_service_error_ = error;
      return false;
    }
  }

  bool SwitchExternalCameraServiceLocked(const std::string& name,
                                         bool enable,
                                         bool video_hub,
                                         std::string& error) {
    if (!robot_state_client_ || name.empty()) {
      error = "camera_service_switch_unavailable";
      return false;
    }
    std::int32_t status = -1;
    std::int32_t result = -1;
    const auto expected = [enable](int value) {
      return enable ? value == 0 : value == 1;
    };
    for (int attempt = 0; attempt < 10; ++attempt) {
      result = robot_state_client_->ServiceSwitch(
          name, enable ? 1 : 0, status);
      if (video_hub) {
        video_hub_switch_api_result_ = result;
        video_hub_status_raw_ = status;
      } else {
        stereo_patch_switch_api_result_ = result;
        stereo_patch_status_raw_ = status;
      }
      if (result == 0 && expected(status)) return true;
      if (result != 0) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    error = name + (enable ? "_start_failed:" : "_stop_failed:") +
            std::to_string(result) + ":" + std::to_string(status);
    external_service_error_ = error;
    return false;
  }

  void StopDepthReceiverLocked(std::string& error) {
    const auto& policy = device_policy_.Camera();
    if (!depth_started_by_web_ || !policy.manage_privileged_receiver) return;
    depth_helper_stop_result_ = RunFixedCameraHelper(
        policy.privileged_receiver_helper,
        PrivilegedCameraAction::kDepthStop);
    if (depth_helper_stop_result_ == 0) {
      depth_receiver_active_ = false;
      depth_started_by_web_ = false;
    } else if (error.empty()) {
      error = "depth_receiver_stop_failed:" +
              std::to_string(depth_helper_stop_result_);
    }
  }

  void RestoreExternalCameraServicesLocked() {
    std::string restore_error = external_service_error_;
    StopDepthReceiverLocked(restore_error);
    std::string ignored;
    if (stereo_patch_changed_by_web_ && !stereo_patch_service_name_.empty()) {
      if (SwitchExternalCameraServiceLocked(stereo_patch_service_name_, false,
                                            false, ignored)) {
        stereo_patch_changed_by_web_ = false;
      } else if (restore_error.empty()) {
        restore_error = ignored;
      }
    }
    if (video_hub_changed_by_web_ && !video_hub_service_name_.empty()) {
      if (SwitchExternalCameraServiceLocked(video_hub_service_name_, true,
                                            true, ignored)) {
        video_hub_changed_by_web_ = false;
      } else if (restore_error.empty()) {
        restore_error = ignored;
      }
    }
    external_service_error_ = restore_error;
  }

  void RestoreExternalCameraServices() {
    const auto& policy = device_policy_.Camera();
    if (!policy.manage_external_services &&
        !policy.manage_privileged_receiver) {
      return;
    }
    std::lock_guard<std::mutex> service_lock(service_mutex_);
    RestoreExternalCameraServicesLocked();
  }

  bool PrepareExternalCameraServices(bool need_depth,
                                     std::string& error) {
    const auto& policy = device_policy_.Camera();
    if (!policy.manage_external_services &&
        !(need_depth && policy.manage_privileged_receiver)) {
      error.clear();
      return true;
    }
    std::lock_guard<std::mutex> service_lock(service_mutex_);
    external_service_error_.clear();

    if (policy.manage_external_services) {
      if (!RefreshExternalCameraServicesLocked(error)) return false;
      video_hub_was_enabled_ = video_hub_status_raw_ == 0;
      stereo_patch_was_enabled_ = stereo_patch_status_raw_ == 0;

      if (video_hub_was_enabled_) {
        if (!SwitchExternalCameraServiceLocked(video_hub_service_name_, false,
                                               true, error)) {
          RestoreExternalCameraServicesLocked();
          return false;
        }
        video_hub_changed_by_web_ = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
      }
      if (!stereo_patch_was_enabled_) {
        if (!SwitchExternalCameraServiceLocked(stereo_patch_service_name_, true,
                                               false, error)) {
          RestoreExternalCameraServicesLocked();
          return false;
        }
        stereo_patch_changed_by_web_ = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
    }

    if (need_depth && policy.manage_privileged_receiver) {
      depth_helper_status_result_ = RunFixedCameraHelper(
          policy.privileged_receiver_helper,
          PrivilegedCameraAction::kDepthIsActive);
      if (depth_helper_status_result_ == 0) {
        depth_receiver_active_ = true;
        depth_was_active_ = true;
      } else if (depth_helper_status_result_ == 3) {
        depth_was_active_ = false;
        depth_helper_start_result_ = RunFixedCameraHelper(
            policy.privileged_receiver_helper,
            PrivilegedCameraAction::kDepthStart);
        if (depth_helper_start_result_ != 0) {
          error = "depth_receiver_start_failed:" +
                  std::to_string(depth_helper_start_result_);
          external_service_error_ = error;
          RestoreExternalCameraServicesLocked();
          return false;
        }
        depth_started_by_web_ = true;
        for (int attempt = 0; attempt < 50; ++attempt) {
          const int result = RunFixedCameraHelper(
              policy.privileged_receiver_helper,
              PrivilegedCameraAction::kDepthIsActive);
          if (result == 0) {
            depth_helper_status_result_ = 0;
            depth_receiver_active_ = true;
            break;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if (!depth_receiver_active_) {
          error = "depth_receiver_start_timeout";
          external_service_error_ = error;
          RestoreExternalCameraServicesLocked();
          return false;
        }
      } else {
        error = "depth_receiver_helper_unavailable:" +
                std::to_string(depth_helper_status_result_);
        external_service_error_ = error;
        RestoreExternalCameraServicesLocked();
        return false;
      }
    }

    external_service_error_.clear();
    error.clear();
    return true;
  }

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
        RestoreExternalCameraServices();
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
    const int status_result = RunFirstPersonHelper(
        device_policy_.Camera().first_person_helper, "is-active");
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
    if (RunFirstPersonHelper(device_policy_.Camera().first_person_helper,
                             "stop") != 0) {
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
    const int result =
        mock ? 0 : RunFirstPersonHelper(
                       device_policy_.Camera().first_person_helper, "start");
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
    value["last_frame_age_ms"] = Json::Int64(AgeMs(stream));
    value["jpeg_bytes"] = Json::UInt64(stream.jpeg.size());
    value["decoded_width"] = stream.decoded_width;
    value["decoded_height"] = stream.decoded_height;
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

  void CaptureRtpH264Loop(const CameraStreamSpec& spec, StreamState& target) {
    const std::string pipeline = BuildFixedRtpH264UdpPipeline(spec);
    if (pipeline.empty()) {
      SetStreamError(target, "camera_pipeline_invalid");
      return;
    }
    while (running_.load()) {
      auto capture = std::make_shared<cv::VideoCapture>(
          pipeline, cv::CAP_GSTREAMER);
      if (!capture->isOpened()) {
        SetStreamError(target, "camera_open_failed");
        std::this_thread::sleep_for(std::chrono::milliseconds(350));
        continue;
      }
      {
        std::lock_guard<std::mutex> lock(rtp_capture_mutex_);
        if (!running_.load()) {
          capture->release();
          return;
        }
        active_rtp_capture_ = capture;
      }
      while (running_.load()) {
        cv::Mat frame;
        if (!capture->read(frame) || frame.empty()) {
          if (running_.load()) SetStreamError(target, "camera_decode_failed");
          break;
        }
        {
          std::lock_guard<std::mutex> lock(mutex_);
          target.decoded_width = frame.cols;
          target.decoded_height = frame.rows;
        }
        std::string validation_error;
        if (!ValidateDecodedCameraFrame(
                spec, frame.cols, frame.rows, validation_error)) {
          SetStreamError(target, validation_error);
          break;
        }
        std::vector<unsigned char> jpeg;
        if (!EncodeJpeg(frame, jpeg, options_.output_width,
                        options_.output_height, options_.rgb_jpeg_quality)) {
          SetStreamError(target, "jpeg_encode_failed");
          break;
        }
        SetFrame(target, std::move(jpeg));
      }
      {
        std::lock_guard<std::mutex> lock(rtp_capture_mutex_);
        if (active_rtp_capture_ == capture) active_rtp_capture_.reset();
      }
      capture->release();
      if (running_.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
      }
    }
  }

  void CaptureRaw16DepthLoop(const CameraStreamSpec& spec,
                             StreamState& target) {
    struct Buffer {
      void* data{MAP_FAILED};
      std::size_t length{0};
    };
    while (running_.load()) {
      const int fd =
          ::open(spec.fixed_source.c_str(), O_RDWR | O_NONBLOCK);
      if (fd < 0) {
        SetStreamError(target, "depth_open_failed");
        std::this_thread::sleep_for(std::chrono::milliseconds(350));
        continue;
      }

      v4l2_format format{};
      format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      format.fmt.pix.width = spec.width;
      format.fmt.pix.height = spec.height;
      format.fmt.pix.pixelformat = v4l2_fourcc('Y', '1', '6', ' ');
      format.fmt.pix.field = V4L2_FIELD_ANY;
      if (::ioctl(fd, VIDIOC_S_FMT, &format) != 0 ||
          format.fmt.pix.pixelformat != v4l2_fourcc('Y', '1', '6', ' ') ||
          format.fmt.pix.width != spec.width ||
          format.fmt.pix.height != spec.height) {
        ::close(fd);
        SetStreamError(target, "depth_format_failed");
        std::this_thread::sleep_for(std::chrono::milliseconds(350));
        continue;
      }

      v4l2_streamparm parameters{};
      parameters.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      parameters.parm.capture.timeperframe.numerator = 1;
      parameters.parm.capture.timeperframe.denominator =
          std::max(1U, spec.fps);
      ::ioctl(fd, VIDIOC_S_PARM, &parameters);

      v4l2_requestbuffers request{};
      request.count = 4;
      request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      request.memory = V4L2_MEMORY_MMAP;
      if (::ioctl(fd, VIDIOC_REQBUFS, &request) != 0 ||
          request.count < 2) {
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

      std::string validation_error;
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
        if (::ioctl(fd, VIDIOC_DQBUF, &buffer) != 0) {
          if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
          SetStreamError(target, "depth_read_failed");
          break;
        }
        if (buffer.index >= buffers.size()) {
          SetStreamError(target, "depth_buffer_index_invalid");
          break;
        }

        if (ValidateRaw16DepthFrame(
                spec, format.fmt.pix.width, format.fmt.pix.height,
                buffer.bytesused, validation_error)) {
          const std::size_t row_bytes =
              format.fmt.pix.bytesperline != 0
                  ? format.fmt.pix.bytesperline
                  : static_cast<std::size_t>(spec.width) * 2U;
          cv::Mat depth_view(static_cast<int>(spec.height),
                             static_cast<int>(spec.width), CV_16UC1,
                             buffers[buffer.index].data, row_bytes);
          std::vector<unsigned char> jpeg;
          if (EncodeJpeg(ColorizeR1DocumentDepth(depth_view), jpeg,
                         options_.output_width, options_.output_height,
                         options_.depth_jpeg_quality)) {
            SetFrame(target, std::move(jpeg));
          } else {
            SetStreamError(target, "jpeg_encode_failed");
          }
        } else {
          SetStreamError(target, validation_error);
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
    bool publish_rgb = false;
    bool publish_depth = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      publish_rgb = rgb_.configured;
      publish_depth = depth_.configured;
    }
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
      if (publish_rgb &&
          EncodeJpeg(rgb, rgb_jpeg, options_.output_width,
                     options_.output_height, options_.rgb_jpeg_quality)) {
        SetFrame(rgb_, std::move(rgb_jpeg));
      }
      if (publish_depth &&
          EncodeJpeg(depth_color, depth_jpeg, options_.output_width,
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
    target.continuous_frames = 0;
    target.continuous_since = {};
  }

  void SetFrame(StreamState& target, std::vector<unsigned char> jpeg) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_.load()) return;
    const auto now = std::chrono::steady_clock::now();
    if (!target.online || target.updated.time_since_epoch().count() == 0 ||
        now - target.updated > std::chrono::seconds(2)) {
      target.continuous_since = now;
      target.continuous_frames = 0;
    }
    target.jpeg = std::move(jpeg);
    target.online = true;
    target.error.clear();
    target.updated = now;
    ++target.continuous_frames;
    ++target.sequence;
  }

  CameraOptions options_;
  const IDeviceCapabilityPolicy& device_policy_;
  mutable std::mutex mutex_;
  StreamState rgb_;
  StreamState depth_;
  std::atomic<bool> running_{false};
  std::thread mock_thread_;
  std::thread realsense_thread_;
  std::thread rgb_thread_;
  std::thread depth_thread_;
  std::thread capture_watchdog_thread_;
#ifdef G1_WEB_HAS_OPENCV
  std::mutex rtp_capture_mutex_;
  std::shared_ptr<cv::VideoCapture> active_rtp_capture_;
#endif
  std::mutex command_mutex_;
  std::uint64_t last_request_id_{0};
  std::string last_request_key_;
  CameraResult last_result_;
  bool first_person_was_active_{false};
  bool first_person_paused_by_web_{false};
  std::string first_person_error_;
  bool realsense_shared_without_pause_{false};
  std::string realsense_probe_error_;
  mutable std::mutex service_mutex_;
  std::unique_ptr<unitree::robot::b2::RobotStateClient> robot_state_client_;
  std::vector<std::string> camera_related_services_;
  std::string video_hub_service_name_;
  std::string stereo_patch_service_name_;
  std::string external_service_error_;
  int service_list_api_result_{-1};
  int video_hub_status_raw_{-1};
  int video_hub_protect_raw_{-1};
  int video_hub_switch_api_result_{-1};
  int stereo_patch_status_raw_{-1};
  int stereo_patch_protect_raw_{-1};
  int stereo_patch_switch_api_result_{-1};
  int depth_helper_status_result_{-1};
  int depth_helper_start_result_{-1};
  int depth_helper_stop_result_{-1};
  bool video_hub_was_enabled_{false};
  bool stereo_patch_was_enabled_{false};
  bool video_hub_changed_by_web_{false};
  bool stereo_patch_changed_by_web_{false};
  bool depth_was_active_{false};
  bool depth_receiver_active_{false};
  bool depth_started_by_web_{false};
};

CameraService::CameraService(
    CameraOptions options, const IDeviceCapabilityPolicy& device_policy)
    : impl_(std::make_unique<Impl>(std::move(options), device_policy)) {}

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

std::vector<DeviceCapabilityRuntime> CameraService::DeviceCapabilities() const {
  return impl_->DeviceCapabilities();
}

}  // namespace g1_web
