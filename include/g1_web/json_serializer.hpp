#pragma once

#include <string>

#include "g1_web/snapshot_store.hpp"

namespace g1_web {

std::string SerializeSnapshot(const RobotSnapshot& snapshot);
std::string SerializeHealth(const RobotSnapshot& snapshot);
std::string SerializeVoiceStatus(const RobotSnapshot& snapshot);
std::string SerializeControlStatus(const RobotSnapshot& snapshot);

}  // namespace g1_web
