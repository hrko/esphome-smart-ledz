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
  std::cout << "SYNCA full-range color temperature synchronization tests passed\n";
}
