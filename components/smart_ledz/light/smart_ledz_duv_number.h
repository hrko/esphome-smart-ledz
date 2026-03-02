#pragma once

#ifdef USE_ESP32

#include "smart_ledz_light.h"
#include "esphome/components/number/number.h"
#include "esphome/core/component.h"
#include "esphome/core/preferences.h"

#include <algorithm>

namespace esphome {
namespace smart_ledz {

class SmartLedzDuvNumber : public number::Number, public Component, public Parented<SmartLedzLightOutput> {
 public:
  void set_initial_value(float initial_value) { this->initial_value_ = clamp_duv_(initial_value); }
  void set_restore_value(bool restore_value) { this->restore_value_ = restore_value; }

  void setup() override;
  void dump_config() override;

 protected:
  void control(float value) override;

  static float clamp_duv_(float value) { return std::max(-6.0f, std::min(6.0f, value)); }

  float initial_value_{0.0f};
  bool restore_value_{false};
  ESPPreferenceObject pref_;
};

}  // namespace smart_ledz
}  // namespace esphome

#endif
