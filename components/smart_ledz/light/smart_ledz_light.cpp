#include "smart_ledz_light.h"

#ifdef USE_ESP32

#include "../ct_rgb_lookup.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

#include <algorithm>
#include <cmath>

namespace esphome {
namespace smart_ledz {

namespace {
static const char *const TAG = "smart_ledz.light";
}

uint8_t SmartLedzLightOutput::kelvin_to_ct_raw_(float kelvin) {
  if (kelvin <= 0) {
    kelvin = 2700.0f;
  }
  auto raw = static_cast<int>(roundf(kelvin / 100.0f));
  raw = std::max(18, std::min(65, raw));
  return static_cast<uint8_t>(raw);
}

light::LightTraits SmartLedzLightOutput::get_traits() {
  auto traits = light::LightTraits();

  switch (this->device_type_) {
    case SMART_LEDZ_DEVICE_TYPE_TUNABLE:
      traits.set_supported_color_modes({light::ColorMode::COLOR_TEMPERATURE});
      traits.set_min_mireds(1000000.0f / 6500.0f);
      traits.set_max_mireds(1000000.0f / 1800.0f);
      break;
    case SMART_LEDZ_DEVICE_TYPE_SYNCA:
      traits.set_supported_color_modes({light::ColorMode::RGB, light::ColorMode::COLOR_TEMPERATURE});
      traits.set_min_mireds(1000000.0f / 12000.0f);
      traits.set_max_mireds(1000000.0f / 1800.0f);
      break;
    case SMART_LEDZ_DEVICE_TYPE_DIMMABLE:
    default:
      traits.set_supported_color_modes({light::ColorMode::BRIGHTNESS});
      break;
  }
  return traits;
}

void SmartLedzLightOutput::setup_state(light::LightState *state) {
  this->state_ = state;
  if (this->parent_ != nullptr && !this->listener_registered_) {
    this->parent_->register_state_listener(this);
    this->parent_->register_polled_target(this->target_);
    this->listener_registered_ = true;
  }
}

void SmartLedzLightOutput::write_state(light::LightState *state) {
  if (this->suppress_write_) {
    this->suppress_write_ = false;
    return;
  }

  if (this->parent_ == nullptr || !this->parent_->is_session_ready()) {
    return;
  }

  const auto device_type = this->device_type_;
  const auto &values = this->ignore_transition_ ? state->remote_values : state->current_values;

  float brightness = 0.0f;
  values.as_brightness(&brightness);
  const bool is_on = values.is_on();

  auto brightness_pct = static_cast<int>(roundf(brightness * 100.0f));
  brightness_pct = std::max(0, std::min(100, brightness_pct));
  if (is_on && brightness_pct == 0) {
    brightness_pct = 1;
  }

  const bool should_send_on_off = !this->has_last_state_ || this->last_on_ != is_on;
  const bool should_send_brightness =
      is_on &&
      (!this->has_last_state_ || !this->last_on_ || this->last_brightness_ != static_cast<uint8_t>(brightness_pct));

  uint8_t desired_ct_raw = 0;
  bool should_send_ct = false;

  uint8_t desired_r = 0;
  uint8_t desired_g = 0;
  uint8_t desired_b = 0;
  bool should_send_rgb = false;

  if (is_on && device_type == SMART_LEDZ_DEVICE_TYPE_TUNABLE) {
    const auto kelvin = values.get_color_temperature_kelvin();
    desired_ct_raw = kelvin_to_ct_raw_(kelvin);
    should_send_ct = !this->has_last_ct_raw_ || this->last_ct_raw_ != desired_ct_raw;
  } else if (is_on && device_type == SMART_LEDZ_DEVICE_TYPE_SYNCA) {
    auto mode = values.get_color_mode();
    if (mode == light::ColorMode::COLOR_TEMPERATURE) {
      this->preferred_synca_mode_ = light::ColorMode::COLOR_TEMPERATURE;
      auto kelvin = values.get_color_temperature_kelvin();
      if (kelvin <= 0) {
        kelvin = 2700.0f;
      }
      auto kelvin_int = static_cast<uint16_t>(std::max(1800, std::min(12000, static_cast<int>(roundf(kelvin)))));
      const auto mapped = lookup_ct_rgb(kelvin_int, this->ct_duv_);
      desired_r = mapped[0];
      desired_g = mapped[1];
      desired_b = mapped[2];
    } else {
      if (mode == light::ColorMode::RGB) {
        this->preferred_synca_mode_ = light::ColorMode::RGB;
      }
      float red = 0.0f;
      float green = 0.0f;
      float blue = 0.0f;
      values.as_rgb(&red, &green, &blue);
      desired_r = static_cast<uint8_t>(std::max(0, std::min(255, static_cast<int>(roundf(red * 255.0f)))));
      desired_g = static_cast<uint8_t>(std::max(0, std::min(255, static_cast<int>(roundf(green * 255.0f)))));
      desired_b = static_cast<uint8_t>(std::max(0, std::min(255, static_cast<int>(roundf(blue * 255.0f)))));
    }
    should_send_rgb = !this->has_last_rgb_ || this->last_rgb_[0] != desired_r || this->last_rgb_[1] != desired_g ||
                      this->last_rgb_[2] != desired_b;
  }

  const bool has_any_command = should_send_on_off || should_send_brightness || should_send_ct || should_send_rgb;
  if (!has_any_command) {
    return;
  }

  // ESPHome transitions can call write_state very frequently; limit BLE write burst.
  const uint32_t now = millis();
  constexpr uint32_t kMinTxIntervalMs = 80;
  if (!should_send_on_off && (now - this->last_tx_ms_) < kMinTxIntervalMs) {
    return;
  }
  this->last_tx_ms_ = now;

  if (should_send_on_off && !this->parent_->send_on_off(this->target_, is_on)) {
    ESP_LOGW(TAG, "on/off send failed target=0x%04X", this->target_);
    return;
  }

  if (should_send_brightness &&
      !this->parent_->send_brightness(this->target_, static_cast<uint8_t>(brightness_pct))) {
    ESP_LOGW(TAG, "brightness send failed target=0x%04X", this->target_);
    return;
  }

  if (should_send_ct) {
    if (!this->parent_->send_ct_raw(this->target_, desired_ct_raw)) {
      ESP_LOGW(TAG, "ct send failed target=0x%04X", this->target_);
      return;
    }
    this->has_last_ct_raw_ = true;
    this->last_ct_raw_ = desired_ct_raw;
  } else if (should_send_rgb) {
    if (!this->parent_->send_rgb(this->target_, desired_r, desired_g, desired_b)) {
      ESP_LOGW(TAG, "rgb send failed target=0x%04X", this->target_);
      return;
    }
    this->has_last_rgb_ = true;
    this->last_rgb_ = {desired_r, desired_g, desired_b};
  }

  this->has_last_state_ = true;
  this->last_on_ = is_on;
  this->last_brightness_ = static_cast<uint8_t>(brightness_pct);
}

void SmartLedzLightOutput::on_smart_ledz_state_update(uint16_t address, const SmartLedzDeviceStateSnapshot &state) {
  if (address != this->target_ || this->state_ == nullptr) {
    return;
  }
  if (this->state_->current_values != this->state_->remote_values) {
    // Do not overwrite local transitions/effects mid-flight.
    return;
  }
  this->sync_from_device_state_(state);
}

void SmartLedzLightOutput::sync_from_device_state_(const SmartLedzDeviceStateSnapshot &device) {
  const auto device_type = this->device_type_;

  bool has_power = false;
  bool is_on = false;
  if (device.has_power) {
    has_power = true;
    is_on = device.power;
  } else if (device.online_status != 0 && device.has_online_brightness) {
    has_power = true;
    is_on = device.online_brightness != 0;
  } else if (this->has_last_state_) {
    has_power = true;
    is_on = this->last_on_;
  }
  const uint8_t brightness_pct = static_cast<uint8_t>(std::max(0, std::min(100, static_cast<int>(device.brightness))));
  const float brightness = brightness_pct / 100.0f;

  const bool on_changed = has_power && (!this->has_last_state_ || this->last_on_ != is_on);
  const bool brightness_changed = !this->has_last_state_ || this->last_brightness_ != brightness_pct;

  bool ct_changed = false;
  bool rgb_changed = false;
  bool apply_rgb_from_notify = false;

  if (device_type == SMART_LEDZ_DEVICE_TYPE_TUNABLE && device.has_ct && device.ct_raw >= 18) {
    ct_changed = !this->has_last_ct_raw_ || this->last_ct_raw_ != device.ct_raw;
  } else if (device_type == SMART_LEDZ_DEVICE_TYPE_SYNCA && device.has_rgb) {
    rgb_changed = !this->has_last_rgb_ || this->last_rgb_ != device.rgb;
    this->has_last_rgb_ = true;
    this->last_rgb_ = device.rgb;

    auto effective_mode = this->preferred_synca_mode_;
    if (effective_mode == light::ColorMode::UNKNOWN && this->state_ != nullptr) {
      effective_mode = this->state_->remote_values.get_color_mode();
    }
    apply_rgb_from_notify = effective_mode != light::ColorMode::COLOR_TEMPERATURE;
  }

  const bool should_apply_color = (ct_changed && device_type == SMART_LEDZ_DEVICE_TYPE_TUNABLE) ||
                                  (rgb_changed && device_type == SMART_LEDZ_DEVICE_TYPE_SYNCA && apply_rgb_from_notify);
  if (!on_changed && !brightness_changed && !should_apply_color) {
    return;
  }

  auto call = this->state_->make_call();
  call.set_transition_length(0);
  call.set_publish(true);
  call.set_save(false);
  if (on_changed) {
    call.set_state(is_on);
  }
  if (brightness_changed) {
    call.set_brightness(brightness);
  }

  if (ct_changed && device_type == SMART_LEDZ_DEVICE_TYPE_TUNABLE && device.has_ct && device.ct_raw >= 18) {
    call.set_color_mode_if_supported(light::ColorMode::COLOR_TEMPERATURE);
    const float kelvin = std::max(1800.0f, std::min(6500.0f, device.ct_raw * 100.0f));
    call.set_color_temperature(1000000.0f / kelvin);
    this->has_last_ct_raw_ = true;
    this->last_ct_raw_ = device.ct_raw;
  } else if (device_type == SMART_LEDZ_DEVICE_TYPE_SYNCA && device.has_rgb && apply_rgb_from_notify && rgb_changed) {
    call.set_color_mode_if_supported(light::ColorMode::RGB);
    call.set_rgb(device.rgb[0] / 255.0f, device.rgb[1] / 255.0f, device.rgb[2] / 255.0f);
    this->preferred_synca_mode_ = light::ColorMode::RGB;
  }

  this->suppress_write_ = true;
  call.perform();
  this->has_last_state_ = true;
  if (has_power) {
    this->last_on_ = is_on;
  }
  this->last_brightness_ = brightness_pct;
}

}  // namespace smart_ledz
}  // namespace esphome

#endif
