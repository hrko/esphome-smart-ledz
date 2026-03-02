#pragma once

#include <array>
#include <cstdint>

namespace smartledz_protocol {
namespace v1 {

std::array<uint8_t, 3> lookup_ct_rgb(uint16_t kelvin, float duv);
uint16_t estimate_cct_from_rgb(uint8_t target_red, uint8_t target_green, uint8_t target_blue, float duv);

}  // namespace v1
}  // namespace smartledz_protocol
