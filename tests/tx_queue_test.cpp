#include "test_hub.h"
#include "components/smart_ledz/light/smart_ledz_light.h"
#include <smartledz_protocol/v1/commands.h>
#include <iostream>

using namespace smartledz_protocol::v1;

static void tick(TestHub &hub, uint32_t ms = 120) {
  fake_time_us += static_cast<int64_t>(ms) * 1000;
  hub.loop();
}

int main() {
  fake_time_us = 1000000;
  {
    TestHub hub;
    hub.pair();
    assert(hub.send_brightness(1, 50));
    write_result = 1;
    tick(hub);
    tick(hub);
    assert(hub.is_session_ready());
    tick(hub);
    assert(!hub.is_session_ready() && hub.client.disconnects == 1);
    assert(!hub.active() && hub.queue_size() == 0);
    write_result = ESP_GATT_OK;
    hub.pair();
    assert(hub.send_brightness(1, 60));
    tick(hub);
    assert(hub.active());
    hub.ack();
    assert(!hub.active());
  }
  {
    TestHub hub;
    hub.pair();
    const auto before = control_writes;
    assert(hub.send_brightness(1, 50));
    tick(hub);
    assert(hub.queue_size() == 0);  // No verification until ACK.
    tick(hub);
    assert(control_writes == before + 1);  // No concurrent GATT writes.
    hub.ack(1);
    tick(hub);
    assert(control_writes == before + 2);
    hub.ack();
    assert(hub.queue_size() == 2 && !hub.active());
  }
  {
    TestHub hub;
    hub.pair();
    assert(hub.send_on_off(1, true));
    for (int i = 0; i < 3; ++i) {
      tick(hub);
      hub.ack(1);
    }
    assert(hub.client.disconnects == 1 && !hub.active());
    hub.ack();  // A late ACK cannot revive the session.
    assert(!hub.is_session_ready());
  }
  {
    TestHub hub;
    hub.pair();
    assert(hub.send_packet(1, 0xd0, nullptr, 0));
    tick(hub);
    hub.ack(1);
    assert(!hub.is_session_ready());  // Do not replay arbitrary raw commands.
  }
  {
    TestHub hub;
    hub.pair();
    assert(hub.send_on_off(1, true));
    assert(hub.send_brightness(1, 50));
    assert(hub.send_rgb(1, 10, 20, 30));
    tick(hub);
    hub.ack();
    assert(hub.send_on_off(2, false));
    tick(hub);
    assert(hub.active_target() == 2 && hub.active_opcode() == kOpcodeOnOff);
    hub.ack();
    tick(hub, 280);
    assert(hub.active_target() == 1 && hub.active_opcode() == kOpcodeBrightness);
    hub.ack();
    tick(hub);
    assert(hub.active_target() == 1 && hub.active_opcode() == kOpcodeColor);
  }
  {
    TestHub hub;
    hub.pair();
    for (int i = 0; i < 1000; ++i) {
      assert(hub.send_status_query(1));
      assert(hub.send_dimming_query(1));
    }
    assert(hub.queue_size() == 2);
    assert(hub.send_brightness(1, 70));
    tick(hub);
    assert(hub.active_opcode() == kOpcodeBrightness);  // Controls precede polling.
  }
  {
    TestHub hub;
    hub.pair();
    for (int i = 0; i < 128; ++i) assert(hub.send_status_query(i + 1));
    assert(!hub.send_status_query(129));
    assert(hub.queue_size() == 128);
    assert(hub.send_on_off(129, false));  // Evict polling to admit a control.
    assert(hub.queue_size() == 128);
    tick(hub);
    assert(hub.active_target() == 129);
  }
  for (auto type : {SMART_LEDZ_DEVICE_TYPE_SYNCA, SMART_LEDZ_DEVICE_TYPE_TUNABLE}) {
    TestHub hub;
    hub.pair();
    SmartLedzLightOutput light;
    light.set_parent(&hub);
    light.set_target(1);
    light.set_device_type(type);
    esphome::light::LightState state;
    state.output = &light;
    light.setup_state(&state);
    light.write_state(&state);
    state.remote_values.kelvin = 5000;
    light.write_state(&state);
    state.remote_values.on = false;
    light.write_state(&state);
    assert(hub.queued_opcode(kOpcodeColor) == 0);
    state.remote_values.on = true;
    light.write_state(&state);
    assert(hub.queued_opcode(kOpcodeColor) == 1);
    hub.reset();
    hub.pair();
    assert(hub.queue_size() == 0);  // Reconnect does not replay old requests.
    light.write_state(&state);
    assert(hub.queued_opcode(kOpcodeColor) == 1);
    assert(hub.queued_opcode(kOpcodeOnOff) == 1);
  }
  std::cout << "TX recovery, scheduling and light cancellation tests passed\n";
}
