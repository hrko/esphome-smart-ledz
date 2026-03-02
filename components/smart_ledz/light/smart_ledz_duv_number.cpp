#include "smart_ledz_duv_number.h"

#ifdef USE_ESP32

#include "esphome/core/log.h"

namespace esphome {
namespace smart_ledz {

namespace {
static const char *const TAG = "smart_ledz.duv_number";
}

void SmartLedzDuvNumber::setup() {
  float value = this->initial_value_;
  if (this->restore_value_) {
    this->pref_ = this->make_entity_preference<float>();
    if (!this->pref_.load(&value)) {
      value = this->initial_value_;
    }
  }

  value = clamp_duv_(value);

  if (this->parent_ != nullptr) {
    this->parent_->set_ct_duv(value);
    this->parent_->apply_ct_duv_update_from_number();
  }

  this->publish_state(value);
}

void SmartLedzDuvNumber::control(float value) {
  value = clamp_duv_(value);

  if (this->parent_ != nullptr) {
    this->parent_->set_ct_duv(value);
    this->parent_->apply_ct_duv_update_from_number();
  }

  this->publish_state(value);

  if (this->restore_value_) {
    this->pref_.save(&value);
  }
}

void SmartLedzDuvNumber::dump_config() {
  LOG_NUMBER("", "Smart LEDZ DUV Number", this);
  ESP_LOGCONFIG(TAG, "  Restore Value: %s", YESNO(this->restore_value_));
}

}  // namespace smart_ledz
}  // namespace esphome

#endif
