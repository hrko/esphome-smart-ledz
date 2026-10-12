#include "test_hub.h"
#include "components/smart_ledz/light/smart_ledz_light.h"
#include <smartledz_protocol/v1/color_math.h>
#include <iostream>

static void tick(TestHub &hub, uint32_t ms = 120) {
  fake_time_us += static_cast<int64_t>(ms) * 1000;
  hub.loop();
}
static void off_notify(TestHub &hub) {
  const uint8_t payload[]{1, 1, 0, 3};
  hub.online(payload, sizeof(payload));
}

int main() {
  fake_time_us = 1000000;
  // Default, explicit legacy, unknown state, non-SYNCA and group targets.
  for (int scenario = 0; scenario < 6; ++scenario) {
    TestHub hub;
    hub.pair();
    SmartLedzLightOutput light;
    light.set_parent(&hub);
    light.set_target(scenario == 4 ? 0x8001 : scenario == 5 ? 0 : 1);
    light.set_device_type(scenario == 3 ? SMART_LEDZ_DEVICE_TYPE_TUNABLE : SMART_LEDZ_DEVICE_TYPE_SYNCA);
    if (scenario == 1) light.set_color_then_brightness(false);
    esphome::light::LightState state;
    state.output = &light;
    light.setup_state(&state);
    if (scenario != 2) off_notify(hub);
    state.remote_values.on = true;
    state.remote_values.brightness = 0.45f;
    state.remote_values.kelvin = 5000;
    state.current_values = state.remote_values;
    light.write_state(&state);
    tick(hub);
    assert(hub.active_opcode() == (scenario == 0 ? 0xe2 : 0xd0));
    if (scenario != 0) continue;
    const auto rgb = smartledz_protocol::v1::lookup_ct_rgb(5000, 0);
    for (size_t i = 0; i < 3; ++i) assert(hub.active_payload(i + 1) == rgb[i]);
    assert(hub.holds() == 0 && hub.queue_size() == 0);
    const auto writes = control_writes;
    tick(hub);
    assert(control_writes == writes);  // Must await color ACK.
    off_notify(hub);
    assert(state.remote_values.on);  // Intermediate notifications cannot undo the request.
    hub.ack();
    tick(hub);
    assert(hub.active_opcode() == 0xd2 && hub.active_payload(0) == 45);
    hub.ack();
    assert(!hub.is_rgb_turn_on_pending(1) && hub.queue_size() == 2);
  }
  // Updates while color is in flight replace both desired color and brightness.
  {
    TestHub hub;
    hub.pair();
    SmartLedzLightOutput light;
    light.set_parent(&hub);
    light.set_target(1);
    light.set_device_type(SMART_LEDZ_DEVICE_TYPE_SYNCA);
    esphome::light::LightState state;
    state.output = &light;
    light.setup_state(&state);
    off_notify(hub);
    state.remote_values.on = true;
    state.remote_values.mode = esphome::light::ColorMode::RGB;
    state.remote_values.red = 1;
    state.remote_values.brightness = 0.35f;
    light.write_state(&state);
    tick(hub);
    state.remote_values.red = 0;
    state.remote_values.blue = 1;
    state.remote_values.brightness = 0.6f;
    light.write_state(&state);
    hub.ack();
    tick(hub);
    assert(hub.active_opcode() == 0xe2 && hub.active_payload(1) == 0 && hub.active_payload(3) == 255);
    hub.ack();
    tick(hub);
    assert(hub.active_opcode() == 0xd2 && hub.active_payload(0) == 60);
    hub.ack();
    state.remote_values.brightness = 0.7f;
    light.write_state(&state);
    tick(hub);
    assert(hub.active_opcode() == 0xd2 && hub.active_payload(0) == 70);
    assert(hub.queued_opcode(0xe2) == 0);  // Already-ON brightness changes stay single-command.
  }
  {
    TestHub hub;
    hub.pair();
    assert(hub.send_rgb_turn_on(1, 10, 20, 30, 35));
    tick(hub);
    assert(hub.send_rgb_turn_on(1, 40, 50, 60, 45));
    assert(hub.send_rgb_turn_on(1, 70, 80, 90, 55));
    assert(hub.queue_size() == 1);
    hub.ack(1);  // Cancelled commands are not retried, even on failure.
    tick(hub);
    assert(hub.active_opcode() == 0xe2 && hub.active_payload(1) == 70);
    hub.ack();
    const auto writes = control_writes;
    tick(hub, 119);
    assert(control_writes == writes);
    tick(hub, 1);
    assert(hub.active_opcode() == 0xd2 && hub.active_payload(0) == 55);
  }
  // OFF cancels queued, in-flight color, and the unsent brightness stage.
  {
    TestHub hub;
    hub.pair();
    assert(hub.send_rgb_turn_on(1, 10, 20, 30, 35));
    tick(hub);
    for (int i = 0; i < 128; ++i) assert(hub.send_on_off(i + 2, false));
    assert(!hub.send_rgb_turn_on(1, 40, 50, 60, 45));
    hub.ack();  // Queue rejection must leave the accepted operation intact.
    tick(hub);
    assert(hub.active_opcode() == 0xd2 && hub.active_payload(0) == 35);
  }
  for (int stage = 0; stage < 3; ++stage) {
    TestHub hub;
    hub.pair();
    assert(hub.send_rgb_turn_on(1, 10, 20, 30, 35));
    if (stage > 0) tick(hub);
    if (stage == 2) hub.ack();
    assert(hub.send_on_off(1, false));
    if (stage == 1) hub.ack();
    assert(!hub.is_rgb_turn_on_pending(1));
    tick(hub);
    assert(hub.active_opcode() == 0xd0 && hub.active_payload(0) == 0);
    assert(hub.queued_opcode(0xd2) == 0 && hub.queued_opcode(0xe2) == 0);
  }
  // Failure of either stage disconnects after bounded retries, without replay.
  for (int stage = 0; stage < 2; ++stage) {
    TestHub hub;
    hub.pair();
    assert(hub.send_rgb_turn_on(1, 10, 20, 30, 35));
    if (stage == 1) { tick(hub); hub.ack(); }
    for (int attempt = 0; attempt < 3; ++attempt) {
      tick(hub);
      assert(hub.active_opcode() == (stage == 0 ? 0xe2 : 0xd2));
      hub.ack(1);
    }
    assert(!hub.is_session_ready() && hub.queue_size() == 0);
    hub.pair();
    assert(!hub.is_rgb_turn_on_pending(1));
  }
  {
    TestHub hub;
    hub.pair();
    off_notify(hub);
    assert(hub.is_confirmed_off(1));
    assert(hub.send_on_off(0x8001, true));
    assert(!hub.is_confirmed_off(1));  // Membership is unknown: conservatively invalidate group operations.
    tick(hub);
    hub.ack();
    assert(!hub.is_confirmed_off(1));
    off_notify(hub);
    assert(hub.is_confirmed_off(1));
    hub.reset();
    hub.pair();
    assert(!hub.is_confirmed_off(1));
    assert(hub.send_rgb_turn_on(1, 10, 20, 30, 35));
    tick(hub);
    tick(hub, 10001);
    assert(!hub.is_session_ready() && !hub.is_confirmed_off(1));
    assert(!hub.is_rgb_turn_on_pending(1));
  }
  std::cout << "SYNCA staged turn-on tests passed\n";
}
