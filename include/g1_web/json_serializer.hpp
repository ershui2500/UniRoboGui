#pragma once

#include <string>

#include "g1_web/robot_profile.hpp"
#include "g1_web/snapshot_store.hpp"

namespace g1_web {

std::string SerializeSnapshot(const RobotProfile& profile,
                              const RobotSnapshot& snapshot);
std::string SerializeHealth(const RobotProfile& profile,
                            const RobotSnapshot& snapshot);
std::string SerializeVoiceStatus(const RobotSnapshot& snapshot);
std::string SerializeControlStatus(const RobotSnapshot& snapshot);
std::string SerializeRobotManifest(const RobotProfile& profile,
                                   const RobotSnapshot& snapshot);

}  // namespace g1_web
