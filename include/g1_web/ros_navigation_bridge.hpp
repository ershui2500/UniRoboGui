#pragma once

#include <memory>
#include <string>

namespace g1_web {

// Optional ROS 2 adapter. The public interface intentionally contains no ROS
// headers so the existing Unitree SDK-only build remains supported.
class RosNavigationBridge {
 public:
  explicit RosNavigationBridge(bool mock = false);
  ~RosNavigationBridge();

  bool Start(std::string& error);
  void Stop();
  bool Available() const;
  std::string SerializeScene() const;
  std::string SerializeTopics() const;
  bool ConfigureTopic(const std::string& kind, const std::string& topic,
                      std::string& error);

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace g1_web
