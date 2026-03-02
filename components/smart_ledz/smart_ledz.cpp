#include "smart_ledz.h"

#ifdef USE_ESP32

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <smartledz_protocol/v1/commands.h>

#include <algorithm>

namespace esphome {
namespace smart_ledz {

namespace {

static const char *const TAG = "smart_ledz";
constexpr uint32_t VERIFY_DELAY_MS = 200;

const espbt::ESPBTUUID TELINK_SERVICE_UUID = espbt::ESPBTUUID::from_raw("00010203-0405-0607-0809-0a0b0c0d1910");
const espbt::ESPBTUUID TELINK_NOTIFY_UUID = espbt::ESPBTUUID::from_raw("00010203-0405-0607-0809-0a0b0c0d1911");
const espbt::ESPBTUUID TELINK_CONTROL_UUID = espbt::ESPBTUUID::from_raw("00010203-0405-0607-0809-0a0b0c0d1912");
const espbt::ESPBTUUID TELINK_PAIR_UUID = espbt::ESPBTUUID::from_raw("00010203-0405-0607-0809-0a0b0c0d1914");

}  // namespace

void SmartLedzHub::configure_session_() { this->session_.configure(this->mesh_name_, this->mesh_password_, this->vendor_id_); }

void SmartLedzHub::set_mesh_name(const std::string &mesh_name) {
  this->mesh_name_ = mesh_name;
  this->configure_session_();
}

void SmartLedzHub::set_mesh_password(const std::string &mesh_password) {
  this->mesh_password_ = mesh_password;
  this->configure_session_();
}

void SmartLedzHub::set_vendor_id(uint16_t vendor_id) {
  this->vendor_id_ = vendor_id;
  this->configure_session_();
}

void SmartLedzHub::dump_config() {
  ESP_LOGCONFIG(TAG, "Smart LEDZ Hub:");
  ESP_LOGCONFIG(TAG, "  Address: %s", this->parent_->address_str());
  ESP_LOGCONFIG(TAG, "  Vendor: 0x%04X", this->vendor_id_);
  ESP_LOGCONFIG(TAG, "  Session Ready: %s", YESNO(this->session_.is_session_ready()));
  ESP_LOGCONFIG(TAG, "  Poll Interval: %ums", this->poll_interval_ms_);
  ESP_LOGCONFIG(TAG, "  TX Interval: %ums", this->tx_interval_ms_);
  ESP_LOGCONFIG(TAG, "  Power-on Settle: %ums", this->power_on_settle_ms_);
}

SmartLedzHub::DeviceState &SmartLedzHub::state_ref_(uint16_t address) {
  auto it = this->device_states_.find(address);
  if (it != this->device_states_.end()) {
    return it->second;
  }
  auto inserted = this->device_states_.emplace(address, DeviceState{});
  return inserted.first->second;
}

void SmartLedzHub::register_state_listener(SmartLedzStateListener *listener) {
  if (listener == nullptr) {
    return;
  }
  for (auto *existing : this->state_listeners_) {
    if (existing == listener) {
      return;
    }
  }
  this->state_listeners_.push_back(listener);
}

void SmartLedzHub::register_polled_target(uint16_t target) {
  if (target == 0 || target == 0xFFFF) {
    return;
  }
  this->polled_targets_.insert(target);
}

void SmartLedzHub::loop() {
  if (!this->session_.is_session_ready()) {
    return;
  }

  const uint32_t now = millis();
  this->process_tx_queue_(now);

  if (this->poll_interval_ms_ == 0 || this->polled_targets_.empty()) {
    return;
  }

  if ((now - this->last_poll_ms_) < this->poll_interval_ms_) {
    return;
  }
  this->last_poll_ms_ = now;

  if (this->poll_cursor_ >= this->polled_targets_.size()) {
    this->poll_cursor_ = 0;
  }

  uint16_t target = 0;
  uint16_t i = 0;
  for (const auto value : this->polled_targets_) {
    if (i == this->poll_cursor_) {
      target = value;
      break;
    }
    i++;
  }
  this->poll_cursor_++;

  if (target != 0) {
    this->send_status_query(target);
    this->send_dimming_query(target);
  }
}

bool SmartLedzHub::get_device_state(uint16_t target, DeviceState *out) const {
  if (out == nullptr) {
    return false;
  }
  auto it = this->device_states_.find(target);
  if (it == this->device_states_.end()) {
    return false;
  }
  *out = it->second;
  return it->second.seen;
}

void SmartLedzHub::notify_state_update_(uint16_t address) {
  auto it = this->device_states_.find(address);
  if (it == this->device_states_.end()) {
    return;
  }
  for (auto *listener : this->state_listeners_) {
    if (listener == nullptr) {
      continue;
    }
    listener->on_smart_ledz_state_update(address, it->second);
  }
}

bool SmartLedzHub::discover_characteristics_() {
  auto *notify = this->parent_->get_characteristic(TELINK_SERVICE_UUID, TELINK_NOTIFY_UUID);
  auto *control = this->parent_->get_characteristic(TELINK_SERVICE_UUID, TELINK_CONTROL_UUID);
  auto *pair = this->parent_->get_characteristic(TELINK_SERVICE_UUID, TELINK_PAIR_UUID);

  if (notify == nullptr || control == nullptr || pair == nullptr) {
    ESP_LOGW(TAG, "[%s] Telink characteristics are missing", this->parent_->address_str());
    return false;
  }

  this->session_.set_characteristic_handles(notify->handle, control->handle, pair->handle);

  ESP_LOGD(TAG, "[%s] handles: notify=0x%04X control=0x%04X pair=0x%04X", this->parent_->address_str(),
           notify->handle, control->handle, pair->handle);

  return true;
}

void SmartLedzHub::update_device_from_online_status_(const uint8_t *payload10, size_t payload_len) {
  std::vector<uint16_t> updated_addresses;
  smartledz_protocol::v1::apply_online_status_payload(payload10, payload_len, millis(), &this->device_states_,
                                                       &updated_addresses);
  for (const auto address : updated_addresses) {
    this->notify_state_update_(address);
  }
}

void SmartLedzHub::update_device_from_status_(uint16_t src, const uint8_t *payload10, size_t payload_len) {
  std::vector<uint16_t> updated_addresses;
  smartledz_protocol::v1::apply_status_payload(src, payload10, payload_len, millis(), &this->device_states_,
                                               &updated_addresses);
  for (const auto address : updated_addresses) {
    this->notify_state_update_(address);
  }
}

void SmartLedzHub::update_device_from_f1_response_(uint16_t src, const uint8_t *payload10, size_t payload_len) {
  std::vector<uint16_t> updated_addresses;
  smartledz_protocol::v1::apply_f1_response_payload(src, payload10, payload_len, millis(), &this->device_states_,
                                                    &updated_addresses);
  for (const auto address : updated_addresses) {
    this->notify_state_update_(address);
  }
}

bool SmartLedzHub::send_packet_now_(uint16_t target, uint8_t opcode, const uint8_t *payload, size_t payload_len) {
  if (!this->session_.is_session_ready()) {
    ESP_LOGW(TAG, "[%s] cannot send packet before pairing", this->parent_->address_str());
    return false;
  }

  if (!this->session_.send_mesh_command(target, opcode, payload, payload_len)) {
    ESP_LOGW(TAG, "[%s] send_packet failed", this->parent_->address_str());
    return false;
  }

  return true;
}

void SmartLedzHub::trim_queued_target_commands_(uint16_t target, uint8_t opcode, int8_t e2_subtype) {
  this->tx_queue_.erase(
      std::remove_if(this->tx_queue_.begin(), this->tx_queue_.end(),
                     [target, opcode, e2_subtype](const TxCommand &queued) {
                       if (queued.target != target || queued.opcode != opcode) {
                         return false;
                       }
                       if (opcode != smartledz_protocol::v1::kOpcodeColor || e2_subtype < 0) {
                         return true;
                       }
                       return queued.payload_len > 0 && queued.payload[0] == static_cast<uint8_t>(e2_subtype);
                     }),
      this->tx_queue_.end());
}

bool SmartLedzHub::enqueue_packet_(uint16_t target, uint8_t opcode, const uint8_t *payload, size_t payload_len,
                                   uint32_t not_before_ms, bool verify_after_send) {
  if (!this->session_.is_session_ready()) {
    ESP_LOGW(TAG, "[%s] cannot queue packet before pairing", this->parent_->address_str());
    return false;
  }
  if (this->session_.control_handle() == 0) {
    ESP_LOGW(TAG, "[%s] control handle not ready", this->parent_->address_str());
    return false;
  }

  TxCommand cmd{};
  cmd.target = target;
  cmd.opcode = opcode;
  cmd.not_before_ms = not_before_ms;
  cmd.verify_after_send = verify_after_send;
  const auto data_len = std::min(payload_len, static_cast<size_t>(10));
  cmd.payload_len = static_cast<uint8_t>(data_len);
  for (size_t i = 0; i < data_len; i++) {
    cmd.payload[i] = payload[i];
  }

  if (opcode == smartledz_protocol::v1::kOpcodeBrightness || opcode == smartledz_protocol::v1::kOpcodeColor) {
    for (auto it = this->tx_queue_.rbegin(); it != this->tx_queue_.rend(); ++it) {
      if (it->target != cmd.target || it->opcode != cmd.opcode) {
        continue;
      }
      if (opcode == smartledz_protocol::v1::kOpcodeColor) {
        if (it->payload_len == 0 || cmd.payload_len == 0 || it->payload[0] != cmd.payload[0]) {
          continue;
        }
      }
      it->payload = cmd.payload;
      it->payload_len = cmd.payload_len;
      if (static_cast<int32_t>(cmd.not_before_ms - it->not_before_ms) > 0) {
        it->not_before_ms = cmd.not_before_ms;
      }
      it->verify_after_send = it->verify_after_send || cmd.verify_after_send;
      return true;
    }
  }

  this->tx_queue_.push_back(cmd);
  return true;
}

void SmartLedzHub::apply_power_on_hold_(uint16_t target, uint32_t now) {
  const uint32_t hold_until = now + this->power_on_settle_ms_;
  auto &target_hold_until = this->target_hold_until_ms_[target];
  if (static_cast<int32_t>(hold_until - target_hold_until) > 0) {
    target_hold_until = hold_until;
  }

  for (auto &queued : this->tx_queue_) {
    if (queued.target != target) {
      continue;
    }
    if (queued.opcode != smartledz_protocol::v1::kOpcodeBrightness &&
        queued.opcode != smartledz_protocol::v1::kOpcodeColor) {
      continue;
    }
    if (static_cast<int32_t>(target_hold_until - queued.not_before_ms) > 0) {
      queued.not_before_ms = target_hold_until;
    }
  }
}

void SmartLedzHub::schedule_verify_queries_(uint16_t target, uint32_t now) {
  this->trim_queued_target_commands_(target, smartledz_protocol::v1::kOpcodeStatusQuery, -1);
  this->trim_queued_target_commands_(target, smartledz_protocol::v1::kOpcodeDimmingQuery, -1);

  const uint32_t verify_not_before = now + VERIFY_DELAY_MS;
  const auto status = smartledz_protocol::v1::make_status_query();
  const auto dimming = smartledz_protocol::v1::make_dimming_query();
  this->enqueue_packet_(target, status.opcode, status.payload.data(), status.payload_len, verify_not_before, false);
  this->enqueue_packet_(target, dimming.opcode, dimming.payload.data(), dimming.payload_len, verify_not_before, false);
}

void SmartLedzHub::process_tx_queue_(uint32_t now) {
  if (this->tx_queue_.empty()) {
    return;
  }
  if ((now - this->last_tx_ms_) < this->tx_interval_ms_) {
    return;
  }

  const auto &next = this->tx_queue_.front();
  if (static_cast<int32_t>(now - next.not_before_ms) < 0) {
    return;
  }
  if (!this->send_packet_now_(next.target, next.opcode, next.payload.data(), next.payload_len)) {
    // Avoid hot-looping when the stack temporarily rejects a write.
    this->last_tx_ms_ = now;
    return;
  }

  const uint16_t sent_target = next.target;
  const uint8_t sent_opcode = next.opcode;
  const uint8_t sent_payload0 = next.payload_len > 0 ? next.payload[0] : 0;
  const bool should_verify = next.verify_after_send;
  this->tx_queue_.pop_front();
  this->last_tx_ms_ = now;

  if (sent_opcode == smartledz_protocol::v1::kOpcodeOnOff) {
    if (sent_payload0 != 0x00) {
      this->apply_power_on_hold_(sent_target, now);
    } else {
      this->target_hold_until_ms_.erase(sent_target);
    }
  }

  if (should_verify) {
    this->schedule_verify_queries_(sent_target, now);
  }
}

bool SmartLedzHub::send_packet(uint16_t target, uint8_t opcode, const uint8_t *payload, size_t payload_len) {
  return this->enqueue_packet_(target, opcode, payload, payload_len, millis(), false);
}

bool SmartLedzHub::send_on_off(uint16_t target, bool on) {
  const auto cmd = smartledz_protocol::v1::make_on_off(on);
  this->trim_queued_target_commands_(target, smartledz_protocol::v1::kOpcodeBrightness, -1);
  this->trim_queued_target_commands_(target, smartledz_protocol::v1::kOpcodeColor, -1);
  if (!on) {
    this->target_hold_until_ms_.erase(target);
  }

  auto ok = this->enqueue_packet_(target, cmd.opcode, cmd.payload.data(), cmd.payload_len, millis(), true);
  if (ok) {
    auto &state = this->state_ref_(target);
    state.seen = true;
    state.has_power = true;
    state.power = on;
    if (!on) {
      state.has_online_brightness = true;
      state.online_brightness = 0;
      state.has_brightness = true;
      state.brightness = 0;
    } else {
      state.has_brightness = false;
    }
    state.last_update_ms = millis();
  }
  return ok;
}

bool SmartLedzHub::send_brightness(uint16_t target, uint8_t brightness) {
  const auto cmd = smartledz_protocol::v1::make_brightness(brightness);
  uint32_t not_before = millis();
  auto hold_it = this->target_hold_until_ms_.find(target);
  if (hold_it != this->target_hold_until_ms_.end() && static_cast<int32_t>(hold_it->second - not_before) > 0) {
    not_before = hold_it->second;
  }

  auto ok = this->enqueue_packet_(target, cmd.opcode, cmd.payload.data(), cmd.payload_len, not_before, true);
  if (ok) {
    auto &state = this->state_ref_(target);
    state.seen = true;
    state.has_brightness = true;
    state.brightness = brightness;
    state.last_update_ms = millis();
  }
  return ok;
}

bool SmartLedzHub::send_rgb(uint16_t target, uint8_t red, uint8_t green, uint8_t blue) {
  const auto cmd = smartledz_protocol::v1::make_rgb(red, green, blue);
  uint32_t not_before = millis();
  auto hold_it = this->target_hold_until_ms_.find(target);
  if (hold_it != this->target_hold_until_ms_.end() && static_cast<int32_t>(hold_it->second - not_before) > 0) {
    not_before = hold_it->second;
  }

  auto ok = this->enqueue_packet_(target, cmd.opcode, cmd.payload.data(), cmd.payload_len, not_before, true);
  if (ok) {
    auto &state = this->state_ref_(target);
    state.seen = true;
    state.has_rgb = true;
    state.rgb = {red, green, blue};
    state.last_update_ms = millis();
  }
  return ok;
}

bool SmartLedzHub::send_ct_raw(uint16_t target, uint8_t ct_raw) {
  const auto cmd = smartledz_protocol::v1::make_ct_raw(ct_raw);
  uint32_t not_before = millis();
  auto hold_it = this->target_hold_until_ms_.find(target);
  if (hold_it != this->target_hold_until_ms_.end() && static_cast<int32_t>(hold_it->second - not_before) > 0) {
    not_before = hold_it->second;
  }

  auto ok = this->enqueue_packet_(target, cmd.opcode, cmd.payload.data(), cmd.payload_len, not_before, true);
  if (ok) {
    auto &state = this->state_ref_(target);
    state.seen = true;
    state.has_ct = true;
    state.ct_raw = cmd.payload[1];
    state.last_update_ms = millis();
  }
  return ok;
}

bool SmartLedzHub::send_status_query(uint16_t target) {
  const auto cmd = smartledz_protocol::v1::make_status_query();
  return this->enqueue_packet_(target, cmd.opcode, cmd.payload.data(), cmd.payload_len, millis(), false);
}

bool SmartLedzHub::send_dimming_query(uint16_t target) {
  const auto cmd = smartledz_protocol::v1::make_dimming_query();
  return this->enqueue_packet_(target, cmd.opcode, cmd.payload.data(), cmd.payload_len, millis(), false);
}

void SmartLedzHub::reset_runtime_state_() {
  this->configure_session_();
  this->session_.reset();
  this->device_states_.clear();
  this->last_poll_ms_ = 0;
  this->poll_cursor_ = 0;
  this->tx_queue_.clear();
  this->target_hold_until_ms_.clear();
  this->last_tx_ms_ = 0;
}

void SmartLedzHub::gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                                       esp_ble_gattc_cb_param_t *param) {
  (void) gattc_if;
  switch (event) {
    case ESP_GATTC_DISCONNECT_EVT:
    case ESP_GATTC_CLOSE_EVT:
      this->reset_runtime_state_();
      break;

    case ESP_GATTC_SEARCH_CMPL_EVT: {
      if (!this->session_.set_link_context(this->parent_->get_gattc_if(), this->parent_->get_conn_id(),
                                           this->parent_->get_remote_bda())) {
        this->parent_->disconnect();
        break;
      }
      if (!this->discover_characteristics_()) {
        this->parent_->disconnect();
        break;
      }
      if (!this->session_.begin_pairing()) {
        this->parent_->disconnect();
      }
      break;
    }

    case ESP_GATTC_WRITE_CHAR_EVT: {
      if (param->write.conn_id != this->parent_->get_conn_id()) {
        break;
      }
      bool need_disconnect = false;
      bool session_established = false;
      this->session_.handle_write_char_evt(param->write.handle, param->write.status, &need_disconnect,
                                           &session_established);
      if (need_disconnect) {
        this->parent_->disconnect();
        break;
      }
      if (session_established) {
        this->node_state = espbt::ClientState::ESTABLISHED;
        ESP_LOGI(TAG, "[%s] Smart LEDZ session established", this->parent_->address_str());
      }
      break;
    }

    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
      bool need_disconnect = false;
      bool session_established = false;
      this->session_.handle_reg_for_notify_evt(param->reg_for_notify.handle, param->reg_for_notify.status,
                                               &need_disconnect, &session_established);
      if (need_disconnect) {
        this->parent_->disconnect();
        break;
      }
      if (session_established) {
        this->node_state = espbt::ClientState::ESTABLISHED;
        ESP_LOGI(TAG, "[%s] Smart LEDZ session established (notify write no-rsp)", this->parent_->address_str());
      }
      break;
    }

    case ESP_GATTC_READ_CHAR_EVT: {
      if (param->read.conn_id != this->parent_->get_conn_id()) {
        break;
      }
      bool need_disconnect = false;
      bool session_established = false;
      this->session_.handle_read_char_evt(param->read.handle, param->read.status, param->read.value,
                                          param->read.value_len, &need_disconnect, &session_established);
      if (need_disconnect) {
        this->parent_->disconnect();
        break;
      }
      if (session_established) {
        this->node_state = espbt::ClientState::ESTABLISHED;
        ESP_LOGI(TAG, "[%s] Smart LEDZ session established", this->parent_->address_str());
      }
      break;
    }

    case ESP_GATTC_NOTIFY_EVT: {
      if (param->notify.conn_id != this->parent_->get_conn_id()) {
        break;
      }

      esp_telink_mesh::v1::DecryptedNotify notify{};
      if (!this->session_.handle_notify_packet(param->notify.handle, param->notify.value, param->notify.value_len,
                                               &notify)) {
        break;
      }

      switch (notify.opcode) {
        case smartledz_protocol::v1::kNotifyOpcodeOnlineStatus:
          this->update_device_from_online_status_(notify.payload.data(), notify.payload.size());
          break;
        case smartledz_protocol::v1::kNotifyOpcodeStatus:
          this->update_device_from_status_(notify.src, notify.payload.data(), notify.payload.size());
          break;
        case smartledz_protocol::v1::kNotifyOpcodeExtended:
          this->update_device_from_f1_response_(notify.src, notify.payload.data(), notify.payload.size());
          break;
        default:
          break;
      }

      ESP_LOGV(TAG, "[%s] notify opcode=0x%02X src=0x%04X", this->parent_->address_str(), notify.opcode, notify.src);
      break;
    }

    default:
      break;
  }
}

}  // namespace smart_ledz
}  // namespace esphome

#endif
