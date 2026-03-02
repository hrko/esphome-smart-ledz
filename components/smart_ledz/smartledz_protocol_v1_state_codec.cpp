#include "smartledz_protocol_v1_state_codec.h"

namespace smartledz_protocol {
namespace v1 {

namespace {

DeviceStateSnapshot &state_ref_(uint16_t address, std::map<uint16_t, DeviceStateSnapshot> *states) {
  auto it = states->find(address);
  if (it != states->end()) {
    return it->second;
  }
  auto inserted = states->emplace(address, DeviceStateSnapshot{});
  return inserted.first->second;
}

void append_updated_(uint16_t address, std::vector<uint16_t> *updated_addresses) {
  if (updated_addresses == nullptr) {
    return;
  }
  updated_addresses->push_back(address);
}

}  // namespace

void apply_online_status_payload(const uint8_t *payload10, size_t payload_len, uint32_t now_ms,
                                 std::map<uint16_t, DeviceStateSnapshot> *states,
                                 std::vector<uint16_t> *updated_addresses) {
  if (payload10 == nullptr || states == nullptr) {
    return;
  }

  size_t i = 0;
  while (i + 3 < payload_len) {
    const uint8_t addr = payload10[i];
    const uint8_t status = payload10[i + 1];
    const uint8_t brightness = payload10[i + 2];
    const uint8_t type_raw = payload10[i + 3];
    i += 4;

    if (addr == 0 || (addr == 0xFF && brightness == 0xFF)) {
      break;
    }

    auto &state = state_ref_(addr, states);
    state.seen = true;
    state.online_status = status;
    state.has_online_brightness = true;
    state.online_brightness = brightness;
    state.has_power = true;
    state.power = (status != 0) && (brightness != 0);
    state.has_brightness = true;
    state.brightness = brightness;
    state.type_raw = type_raw;
    state.last_update_ms = now_ms;
    append_updated_(addr, updated_addresses);
  }
}

void apply_status_payload(uint16_t src, const uint8_t *payload10, size_t payload_len, uint32_t now_ms,
                          std::map<uint16_t, DeviceStateSnapshot> *states,
                          std::vector<uint16_t> *updated_addresses) {
  if (payload10 == nullptr || states == nullptr || payload_len < 4) {
    return;
  }

  auto &state = state_ref_(src, states);
  state.seen = true;
  state.last_update_ms = now_ms;

  const uint8_t b0 = payload10[0];
  const uint8_t b1 = payload10[1];
  const uint8_t b2 = payload10[2];
  const uint8_t b3 = payload10[3];

  state.has_rgb = true;
  state.rgb = {b0, b1, b2};
  state.has_brightness = false;
  if (18 <= b3 && b3 <= 65) {
    state.has_ct = true;
    state.ct_raw = b3;
  }

  append_updated_(src, updated_addresses);
}

void apply_f1_response_payload(uint16_t src, const uint8_t *payload10, size_t payload_len, uint32_t now_ms,
                               std::map<uint16_t, DeviceStateSnapshot> *states,
                               std::vector<uint16_t> *updated_addresses) {
  if (payload10 == nullptr || states == nullptr || payload_len < 7 || payload10[0] != 0xF1) {
    return;
  }

  const uint8_t func = payload10[2];
  const uint8_t item = payload10[3];
  const uint8_t data_len = payload10[5];
  if (func != 1 || item != 4 || data_len < 1 || payload_len < static_cast<size_t>(6 + data_len)) {
    return;
  }

  auto &state = state_ref_(src, states);
  state.seen = true;
  state.has_brightness = true;
  state.brightness = payload10[6];
  state.last_update_ms = now_ms;

  append_updated_(src, updated_addresses);
}

}  // namespace v1
}  // namespace smartledz_protocol
