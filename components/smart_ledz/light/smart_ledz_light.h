#pragma once

#ifdef USE_ESP32

#include "../smart_ledz.h"
#include "esphome/components/light/light_output.h"

namespace esphome {
namespace smart_ledz {

class SmartLedzLightOutput : public light::LightOutput,
                             public Parented<SmartLedzHub>,
                             public SmartLedzStateListener {
 public:
  void set_target(uint16_t target) { this->target_ = target; }
  void set_device_type(SmartLedzDeviceType device_type) { this->device_type_ = device_type; }
  void set_ct_duv(int8_t ct_duv) {
    switch (ct_duv) {
      case -6:
      case -3:
      case 0:
      case 3:
      case 6:
        this->ct_duv_ = ct_duv;
        break;
      default:
        this->ct_duv_ = 0;
        break;
    }
  }

  light::LightTraits get_traits() override;
  void setup_state(light::LightState *state) override;
  void write_state(light::LightState *state) override;
  void on_smart_ledz_state_update(uint16_t address, const SmartLedzDeviceStateSnapshot &state) override;

 protected:
  static uint8_t kelvin_to_ct_raw_(float kelvin);
  void sync_from_device_state_(const SmartLedzDeviceStateSnapshot &device);

  uint16_t target_{0x0000};
  SmartLedzDeviceType device_type_{SMART_LEDZ_DEVICE_TYPE_DIMMABLE};
  int8_t ct_duv_{0};
  light::LightState *state_{nullptr};
  bool listener_registered_{false};
  bool suppress_write_{false};
  bool has_last_state_{false};
  bool last_on_{false};
  uint8_t last_brightness_{0};
  bool has_last_ct_raw_{false};
  uint8_t last_ct_raw_{0};
  bool has_last_rgb_{false};
  std::array<uint8_t, 3> last_rgb_{0, 0, 0};
  light::ColorMode preferred_synca_mode_{light::ColorMode::UNKNOWN};
  uint32_t last_tx_ms_{0};
};

}  // namespace smart_ledz
}  // namespace esphome

#endif
