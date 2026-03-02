#include "ct_rgb_lookup.h"

#ifdef USE_ESP32

namespace esphome {
namespace smart_ledz {

std::array<uint8_t, 3> lookup_ct_rgb(uint16_t kelvin, float duv) {
  return smartledz_protocol::v1::lookup_ct_rgb(kelvin, duv);
}

uint16_t estimate_cct_from_rgb(uint8_t target_red, uint8_t target_green, uint8_t target_blue, float duv) {
  return smartledz_protocol::v1::estimate_cct_from_rgb(target_red, target_green, target_blue, duv);
}

}  // namespace smart_ledz
}  // namespace esphome

#endif
