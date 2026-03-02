#include "smartledz_protocol_v1_commands.h"

#include <algorithm>

namespace smartledz_protocol {
namespace v1 {

MeshCommand make_on_off(bool on) {
  MeshCommand cmd{};
  cmd.opcode = kOpcodeOnOff;
  cmd.payload[0] = static_cast<uint8_t>(on ? 0x01 : 0x00);
  cmd.payload_len = 1;
  return cmd;
}

MeshCommand make_brightness(uint8_t brightness_pct) {
  MeshCommand cmd{};
  cmd.opcode = kOpcodeBrightness;
  cmd.payload[0] = brightness_pct;
  cmd.payload_len = 1;
  return cmd;
}

MeshCommand make_rgb(uint8_t red, uint8_t green, uint8_t blue) {
  MeshCommand cmd{};
  cmd.opcode = kOpcodeColor;
  cmd.payload[0] = 0x04;
  cmd.payload[1] = red;
  cmd.payload[2] = green;
  cmd.payload[3] = blue;
  cmd.payload_len = 4;
  return cmd;
}

MeshCommand make_ct_raw(uint8_t ct_raw) {
  MeshCommand cmd{};
  cmd.opcode = kOpcodeColor;
  cmd.payload[0] = 0x05;
  cmd.payload[1] = static_cast<uint8_t>(std::max(18, std::min(65, static_cast<int>(ct_raw))));
  cmd.payload_len = 2;
  return cmd;
}

MeshCommand make_status_query() {
  MeshCommand cmd{};
  cmd.opcode = kOpcodeStatusQuery;
  cmd.payload[0] = 0x10;
  cmd.payload_len = 1;
  return cmd;
}

MeshCommand make_dimming_query() {
  MeshCommand cmd{};
  cmd.opcode = kOpcodeDimmingQuery;
  cmd.payload[0] = 0x01;
  cmd.payload[1] = 0x01;
  cmd.payload[2] = 0x04;
  cmd.payload[3] = 0x11;
  cmd.payload[4] = 0x02;
  cmd.payload_len = 5;
  return cmd;
}

}  // namespace v1
}  // namespace smartledz_protocol
