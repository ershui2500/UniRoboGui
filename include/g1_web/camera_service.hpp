#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace g1_web {

struct CameraOptions {
  bool mock{false};
  bool realsense{false};
  bool auto_detect{false};
  bool teleimager_rgb{false};
  std::string rgb_source;
  std::string depth_source;
  unsigned int width{640};
  unsigned int height{480};
  unsigned int fps{15};
  unsigned int output_width{480};
  unsigned int output_height{360};
  int rgb_jpeg_quality{55};
  int depth_jpeg_quality{45};
};

struct CameraFrame {
  bool available{false};
  std::uint64_t sequence{0};
  std::vector<unsigned char> jpeg;
};

struct CameraRequest {
  std::string request_key;
  std::string command;
  std::string rgb_source;
  std::string depth_source;
  bool confirmed{false};
};

struct CameraResult {
  bool accepted{false};
  bool duplicate{false};
  std::uint64_t request_id{0};
  std::string error;
};

class CameraService {
 public:
  explicit CameraService(CameraOptions options);
  ~CameraService();

  bool Start(std::string& error);
  void Stop();
  CameraResult Submit(const CameraRequest& request);
  CameraFrame GetFrame(const std::string& stream) const;
  std::string SerializeStatus() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace g1_web
