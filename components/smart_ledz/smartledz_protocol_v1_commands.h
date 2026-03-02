#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace smartledz_protocol {
namespace v1 {

constexpr uint8_t kOpcodeOnOff = 0xD0;
constexpr uint8_t kOpcodeBrightness = 0xD2;
constexpr uint8_t kOpcodeColor = 0xE2;
constexpr uint8_t kOpcodeStatusQuery = 0xDA;
constexpr uint8_t kOpcodeDimmingQuery = 0xF0;

constexpr uint8_t kNotifyOpcodeOnlineStatus = 0xDC;
constexpr uint8_t kNotifyOpcodeStatus = 0xDB;
constexpr uint8_t kNotifyOpcodeExtended = 0xEA;

struct MeshCommand {
  uint8_t opcode{0};
  std::array<uint8_t, 10> payload{};
  uint8_t payload_len{0};
};

MeshCommand make_on_off(bool on);
MeshCommand make_brightness(uint8_t brightness_pct);
MeshCommand make_rgb(uint8_t red, uint8_t green, uint8_t blue);
MeshCommand make_ct_raw(uint8_t ct_raw);
MeshCommand make_status_query();
MeshCommand make_dimming_query();

}  // namespace v1
}  // namespace smartledz_protocol
