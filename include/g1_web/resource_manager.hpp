#pragma once

#include <array>
#include <cstddef>
#include <mutex>
#include <string>

namespace g1_web {

enum class ControlResource : std::size_t {
  Locomotion = 0,
  Arm,
  WholeBody,
  LowCmd,
  JointTeach,
  Count,
};

struct ResourceAcquireResult {
  bool acquired{false};
  bool already_owned{false};
  std::string error;
};

class ResourceManager {
 public:
  ResourceAcquireResult Acquire(ControlResource resource,
                                const std::string& owner) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (owner.empty()) return {false, false, "resource_owner_required"};

    auto& slot = owners_.at(Index(resource));
    if (slot == owner) return {true, true, {}};
    for (const auto& held_owner : owners_) {
      if (!held_owner.empty() && held_owner != owner)
        return {false, false, "resource_busy"};
    }
    slot = owner;
    return {true, false, {}};
  }

  bool Release(ControlResource resource, const std::string& owner) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto& slot = owners_.at(Index(resource));
    if (slot != owner) return false;
    slot.clear();
    return true;
  }

  void ReleaseOwner(const std::string& owner) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto& held_owner : owners_) {
      if (held_owner == owner) held_owner.clear();
    }
  }

  std::string Query(ControlResource resource) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return owners_.at(Index(resource));
  }

 private:
  static constexpr std::size_t Index(ControlResource resource) {
    return static_cast<std::size_t>(resource);
  }

  mutable std::mutex mutex_;
  std::array<std::string, static_cast<std::size_t>(ControlResource::Count)>
      owners_{};
};

}  // namespace g1_web
