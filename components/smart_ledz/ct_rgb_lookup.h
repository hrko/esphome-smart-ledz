#pragma once

#ifdef USE_ESP32

#include <array>
#include <cstdint>

namespace esphome {
namespace smart_ledz {

std::array<uint8_t, 3> lookup_ct_rgb(uint16_t kelvin, float duv);

uint16_t estimate_cct_from_rgb(uint8_t target_red, uint8_t target_green, uint8_t target_blue, float duv);

}  // namespace smart_ledz
}  // namespace esphome

#endif
