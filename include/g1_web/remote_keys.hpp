#pragma once

#include <array>
#include <cstdint>

namespace g1_web {

// SDK2 gamepad.hpp: two header bytes followed by the little-endian key word.
inline std::uint16_t DecodeRaw40RemoteKeys(
    const std::array<std::uint8_t, 40>& remote) {
  return static_cast<std::uint16_t>(remote[2]) |
         (static_cast<std::uint16_t>(remote[3]) << 8U);
}

}  // namespace g1_web
