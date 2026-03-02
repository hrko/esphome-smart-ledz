#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <vector>

namespace smartledz_protocol {
namespace v1 {

struct DeviceStateSnapshot {
  bool seen{false};
  uint8_t online_status{0};
  bool has_online_brightness{false};
  uint8_t online_brightness{0};
  bool has_power{false};
  bool power{false};
  bool has_brightness{false};
  uint8_t brightness{0};
  uint8_t type_raw{0};
  bool has_ct{false};
  uint8_t ct_raw{0};
  bool has_rgb{false};
  std::array<uint8_t, 3> rgb{0, 0, 0};
  uint32_t last_update_ms{0};
};

void apply_online_status_payload(const uint8_t *payload10, size_t payload_len, uint32_t now_ms,
                                 std::map<uint16_t, DeviceStateSnapshot> *states,
                                 std::vector<uint16_t> *updated_addresses);

void apply_status_payload(uint16_t src, const uint8_t *payload10, size_t payload_len, uint32_t now_ms,
                          std::map<uint16_t, DeviceStateSnapshot> *states,
                          std::vector<uint16_t> *updated_addresses);

void apply_f1_response_payload(uint16_t src, const uint8_t *payload10, size_t payload_len, uint32_t now_ms,
                               std::map<uint16_t, DeviceStateSnapshot> *states,
                               std::vector<uint16_t> *updated_addresses);

}  // namespace v1
}  // namespace smartledz_protocol
