#pragma once
#include "esphome/core/component.h"
#include <initializer_list>

namespace esphome::light {
enum class ColorMode { UNKNOWN, RGB, COLOR_TEMPERATURE, BRIGHTNESS };
struct LightColorValues {
  ColorMode mode = ColorMode::COLOR_TEMPERATURE;
  bool on = true;
  float brightness = 0.5f, kelvin = 2700, red = 0, green = 0, blue = 0;
  bool is_on() const { return on; }
  ColorMode get_color_mode() const { return mode; }
  float get_color_temperature_kelvin() const { return kelvin; }
  void as_brightness(float *value) const { *value = brightness; }
  void as_rgb(float *r, float *g, float *b) const { *r = red; *g = green; *b = blue; }
  bool operator!=(const LightColorValues &v) const {
    return mode != v.mode || on != v.on || brightness != v.brightness || kelvin != v.kelvin ||
           red != v.red || green != v.green || blue != v.blue;
  }
};
struct LightTraits {
  void set_supported_color_modes(std::initializer_list<ColorMode>) {}
  void set_min_mireds(float) {}
  void set_max_mireds(float) {}
};
class LightState;
class LightOutput {
 public:
  virtual ~LightOutput() = default;
  virtual LightTraits get_traits() = 0;
  virtual void setup_state(LightState *) {}
  virtual void write_state(LightState *) = 0;
};
class LightCall {
 public:
  explicit LightCall(LightState *state);
  void set_transition_length(unsigned) {}
  void set_publish(bool) {}
  void set_save(bool) {}
  void set_state(bool on) { values_.on = on; }
  void set_brightness(float value) { values_.brightness = value; }
  void set_color_mode_if_supported(ColorMode mode) { values_.mode = mode; }
  void set_color_temperature(float mireds) { values_.kelvin = 1000000.0f / mireds; }
  void set_rgb(float r, float g, float b) { values_.red = r; values_.green = g; values_.blue = b; }
  void perform();
 private:
  LightState *state_;
  LightColorValues values_;
};
class LightState {
 public:
  LightColorValues current_values, remote_values;
  LightOutput *output = nullptr;
  unsigned publications = 0;
  LightCall make_call() { return LightCall(this); }
};
inline LightCall::LightCall(LightState *state) : state_(state), values_(state->remote_values) {}
inline void LightCall::perform() {
  state_->remote_values = state_->current_values = values_;
  ++state_->publications;
  if (state_->output != nullptr) state_->output->write_state(state_);
}
}
