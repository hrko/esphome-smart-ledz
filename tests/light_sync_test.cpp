#include "components/smart_ledz/light/smart_ledz_light.h"
#include <smartledz_protocol/v1/color_math.h>
#include <cassert>
#include <cmath>
#include <iostream>
using namespace esphome::smart_ledz;

int main() {
  SmartLedzLightOutput light;
  light.set_target(1);
  light.set_device_type(SMART_LEDZ_DEVICE_TYPE_SYNCA);
  esphome::light::LightState state;
  state.output = &light;
  light.setup_state(&state);
  SmartLedzDeviceStateSnapshot device;
  device.has_rgb = true;
  for (uint16_t kelvin : {7000, 9000, 12000, 6500, 1800}) {
    device.rgb = smartledz_protocol::v1::lookup_ct_rgb(kelvin, 0);
    const auto expected = smartledz_protocol::v1::estimate_cct_from_rgb(device.rgb[0], device.rgb[1], device.rgb[2], 0);
    const auto before = state.publications;
    light.on_smart_ledz_state_update(1, device);
    assert(state.publications == before + 1);
    assert(std::abs(state.remote_values.kelvin - expected) < 1);
    light.on_smart_ledz_state_update(1, device);
    assert(state.publications == before + 1);
  }
  SmartLedzLightOutput tunable;
  tunable.set_target(2);
  tunable.set_device_type(SMART_LEDZ_DEVICE_TYPE_TUNABLE);
  esphome::light::LightState tunable_state;
  tunable_state.output = &tunable;
  tunable.setup_state(&tunable_state);
  device.has_ct = true;
  device.ct_raw = 65;
  tunable.on_smart_ledz_state_update(2, device);
  assert(std::abs(tunable_state.remote_values.kelvin - 6500) < 1);
  SmartLedzLightOutput dimmable;
  dimmable.set_target(3);
  esphome::light::LightState dimmable_state;
  dimmable_state.output = &dimmable;
  dimmable.setup_state(&dimmable_state);
  std::map<uint16_t, SmartLedzDeviceStateSnapshot> states;
  uint8_t online[] = {3, 1, 70, 1};
  smartledz_protocol::v1::apply_online_status_payload(online, sizeof(online), 1, &states, nullptr);
  dimmable.on_smart_ledz_state_update(3, states.at(3));
  const auto publications = dimmable_state.publications;
  online[1] = 0;
  online[2] = 0;
  smartledz_protocol::v1::apply_online_status_payload(online, sizeof(online), 2, &states, nullptr);
  dimmable.on_smart_ledz_state_update(3, states.at(3));
  assert(!states.at(3).reachable);
  assert(dimmable_state.remote_values.is_on());
  assert(std::abs(dimmable_state.remote_values.brightness - 0.7f) < 0.001f);
  assert(dimmable_state.publications == publications);
  online[1] = 1;
  smartledz_protocol::v1::apply_online_status_payload(online, sizeof(online), 3, &states, nullptr);
  dimmable.on_smart_ledz_state_update(3, states.at(3));
  assert(!dimmable_state.remote_values.is_on());
  std::cout << "Light temperature and offline state synchronization tests passed\n";
}
