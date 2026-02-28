#include "smart_ledz.h"

#ifdef USE_ESP32

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <algorithm>
#include <array>
#include <cstring>

#include <esp_random.h>

#define MBEDTLS_AES_ALT
#include <aes_alt.h>

namespace esphome {
namespace smart_ledz {

namespace {

static const char *const TAG = "smart_ledz";

const espbt::ESPBTUUID TELINK_SERVICE_UUID = espbt::ESPBTUUID::from_raw("00010203-0405-0607-0809-0a0b0c0d1910");
const espbt::ESPBTUUID TELINK_NOTIFY_UUID = espbt::ESPBTUUID::from_raw("00010203-0405-0607-0809-0a0b0c0d1911");
const espbt::ESPBTUUID TELINK_CONTROL_UUID = espbt::ESPBTUUID::from_raw("00010203-0405-0607-0809-0a0b0c0d1912");
const espbt::ESPBTUUID TELINK_PAIR_UUID = espbt::ESPBTUUID::from_raw("00010203-0405-0607-0809-0a0b0c0d1914");

std::array<uint8_t, 16> mesh_xor_key(const std::string &name, const std::string &password) {
  std::array<uint8_t, 16> out{};
  for (size_t i = 0; i < out.size(); i++) {
    const auto a = static_cast<uint8_t>(i < name.size() ? name[i] : 0);
    const auto b = static_cast<uint8_t>(i < password.size() ? password[i] : 0);
    out[i] = a ^ b;
  }
  return out;
}

bool aes_encrypt_telink(const std::array<uint8_t, 16> &key, const std::array<uint8_t, 16> &data,
                        std::array<uint8_t, 16> *out) {
  uint8_t key_rev[16];
  uint8_t data_rev[16];
  uint8_t enc_rev[16];

  for (size_t i = 0; i < 16; i++) {
    key_rev[i] = key[15 - i];
    data_rev[i] = data[15 - i];
  }

  mbedtls_aes_context ctx;
  mbedtls_aes_init(&ctx);
  auto key_status = mbedtls_aes_setkey_enc(&ctx, key_rev, 128);
  if (key_status != 0) {
    mbedtls_aes_free(&ctx);
    return false;
  }
  auto enc_status = mbedtls_aes_crypt_ecb(&ctx, ESP_AES_ENCRYPT, data_rev, enc_rev);
  mbedtls_aes_free(&ctx);
  if (enc_status != 0) {
    return false;
  }

  for (size_t i = 0; i < 16; i++) {
    (*out)[i] = enc_rev[15 - i];
  }
  return true;
}

bool key_encrypt(const std::string &mesh_name, const std::string &mesh_password, const std::array<uint8_t, 16> &key,
                 std::array<uint8_t, 16> *out) {
  return aes_encrypt_telink(key, mesh_xor_key(mesh_name, mesh_password), out);
}

bool generate_session_key(const std::string &mesh_name, const std::string &mesh_password,
                          const std::array<uint8_t, 8> &data1, const uint8_t *data2_8,
                          std::array<uint8_t, 16> *out) {
  std::array<uint8_t, 16> data{};
  for (size_t i = 0; i < 8; i++) {
    data[i] = data1[i];
    data[8 + i] = data2_8[i];
  }
  return aes_encrypt_telink(mesh_xor_key(mesh_name, mesh_password), data, out);
}

}  // namespace

void SmartLedzHub::dump_config() {
  ESP_LOGCONFIG(TAG, "Smart LEDZ Hub:");
  ESP_LOGCONFIG(TAG, "  Address: %s", this->parent_->address_str());
  ESP_LOGCONFIG(TAG, "  Vendor: 0x%04X", this->vendor_id_);
  ESP_LOGCONFIG(TAG, "  Session Ready: %s", YESNO(this->session_ready_));
  ESP_LOGCONFIG(TAG, "  Poll Interval: %ums", this->poll_interval_ms_);
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
  if (!this->session_ready_ || this->poll_interval_ms_ == 0 || this->polled_targets_.empty()) {
    return;
  }
  const uint32_t now = millis();
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

  this->notify_handle_ = notify->handle;
  this->control_handle_ = control->handle;
  this->pair_handle_ = pair->handle;

  ESP_LOGD(TAG, "[%s] handles: notify=0x%04X control=0x%04X pair=0x%04X", this->parent_->address_str(),
           this->notify_handle_, this->control_handle_, this->pair_handle_);

  return true;
}

void SmartLedzHub::update_mac_data_() {
  const uint8_t *mac = this->parent_->get_remote_bda();
  this->mac_data_[0] = mac[5];
  this->mac_data_[1] = mac[4];
  this->mac_data_[2] = mac[3];
  this->mac_data_[3] = mac[2];
  this->mac_data_[4] = mac[1];
  this->mac_data_[5] = mac[0];
}

void SmartLedzHub::begin_pairing_() {
  this->update_mac_data_();

  std::array<uint8_t, 16> pair_data{};
  esp_fill_random(this->pair_random_.data(), this->pair_random_.size());
  for (size_t i = 0; i < this->pair_random_.size(); i++) {
    pair_data[i] = this->pair_random_[i];
  }

  std::array<uint8_t, 16> enc_data{};
  if (!key_encrypt(this->mesh_name_, this->mesh_password_, pair_data, &enc_data)) {
    ESP_LOGE(TAG, "[%s] key_encrypt failed", this->parent_->address_str());
    this->parent_->disconnect();
    return;
  }

  uint8_t packet[17] = {0};
  packet[0] = 0x0C;
  for (size_t i = 0; i < 8; i++) {
    packet[1 + i] = pair_data[i];
    packet[9 + i] = enc_data[i];
  }

  auto status = esp_ble_gattc_write_char(this->parent_->get_gattc_if(), this->parent_->get_conn_id(), this->pair_handle_,
                                         sizeof(packet), packet, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
  if (status != ESP_GATT_OK) {
    ESP_LOGW(TAG, "[%s] pair write failed, status=%d", this->parent_->address_str(), status);
    this->parent_->disconnect();
    return;
  }

  this->pairing_write_pending_ = true;
}

bool SmartLedzHub::write_notify_enable_() {
  uint8_t value = 0x01;
  auto status = esp_ble_gattc_write_char(this->parent_->get_gattc_if(), this->parent_->get_conn_id(), this->notify_handle_,
                                         sizeof(value), &value, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
  if (status == ESP_GATT_OK) {
    this->notify_enable_pending_ = true;
    return true;
  }

  status = esp_ble_gattc_write_char(this->parent_->get_gattc_if(), this->parent_->get_conn_id(), this->notify_handle_,
                                    sizeof(value), &value, ESP_GATT_WRITE_TYPE_NO_RSP, ESP_GATT_AUTH_REQ_NONE);
  if (status != ESP_GATT_OK) {
    ESP_LOGW(TAG, "[%s] notify enable write failed, status=%d", this->parent_->address_str(), status);
    return false;
  }

  this->node_state = espbt::ClientState::ESTABLISHED;
  ESP_LOGI(TAG, "[%s] Smart LEDZ session established (notify write no-rsp)", this->parent_->address_str());
  return true;
}

bool SmartLedzHub::register_notify_() {
  auto status = esp_ble_gattc_register_for_notify(this->parent_->get_gattc_if(), this->parent_->get_remote_bda(),
                                                  this->notify_handle_);
  if (status != ESP_GATT_OK) {
    ESP_LOGW(TAG, "[%s] register_for_notify failed status=%d", this->parent_->address_str(), status);
    return false;
  }
  this->notify_register_pending_ = true;
  return true;
}

void SmartLedzHub::handle_pairing_response_(const uint8_t *data, uint16_t len) {
  if (len < 9) {
    ESP_LOGW(TAG, "[%s] invalid pairing response len=%u", this->parent_->address_str(), len);
    this->parent_->disconnect();
    return;
  }

  if (!generate_session_key(this->mesh_name_, this->mesh_password_, this->pair_random_, data + 1, &this->session_key_)) {
    ESP_LOGW(TAG, "[%s] session key generation failed", this->parent_->address_str());
    this->parent_->disconnect();
    return;
  }

  this->session_ready_ = true;
  this->packet_count_ = static_cast<uint16_t>(esp_random() & 0xFFFF);
  if (this->packet_count_ == 0) {
    this->packet_count_ = 1;
  }

  if (!this->register_notify_()) {
    this->write_notify_enable_();
  }

  ESP_LOGI(TAG, "[%s] Smart LEDZ session key ready", this->parent_->address_str());
}

void SmartLedzHub::update_device_from_online_status_(const uint8_t *payload10, size_t payload_len) {
  size_t i = 0;
  while (i + 3 < payload_len) {
    const uint8_t addr = payload10[i];
    const uint8_t status = payload10[i + 1];
    const uint8_t brightness = payload10[i + 2];
    const uint8_t type_raw = payload10[i + 3];
    i += 4;

    if (addr == 0 || (addr == 0xFF && brightness == 0xFF)) {
      break;
    }

    auto &state = this->state_ref_(addr);
    state.seen = true;
    state.online_status = status;
    state.has_online_brightness = true;
    state.online_brightness = brightness;
    state.has_power = true;
    state.power = (status != 0) && (brightness != 0);
    state.brightness = brightness;
    state.type_raw = type_raw;
    state.last_update_ms = millis();
    this->notify_state_update_(addr);
  }
}

void SmartLedzHub::update_device_from_status_(uint16_t src, const uint8_t *payload10, size_t payload_len) {
  if (payload_len < 4) {
    return;
  }

  auto &state = this->state_ref_(src);
  state.seen = true;
  state.last_update_ms = millis();

  const uint8_t b0 = payload10[0];
  const uint8_t b1 = payload10[1];
  const uint8_t b2 = payload10[2];
  const uint8_t b3 = payload10[3];

  state.has_rgb = true;
  state.rgb = {b0, b1, b2};
  if (18 <= b3 && b3 <= 65) {
    state.has_ct = true;
    state.ct_raw = b3;
  }
  this->notify_state_update_(src);
}

void SmartLedzHub::update_device_from_f1_response_(uint16_t src, const uint8_t *payload10, size_t payload_len) {
  if (payload_len < 7 || payload10[0] != 0xF1) {
    return;
  }

  const uint8_t func = payload10[2];
  const uint8_t item = payload10[3];
  const uint8_t data_len = payload10[5];

  if (func != 1 || item != 4 || data_len < 1 || payload_len < static_cast<size_t>(6 + data_len)) {
    return;
  }

  auto &state = this->state_ref_(src);
  state.seen = true;
  state.brightness = payload10[6];
  state.last_update_ms = millis();
  this->notify_state_update_(src);
}

void SmartLedzHub::encrypt_packet_(uint8_t *packet20) const {
  std::array<uint8_t, 16> auth_nonce{};
  auth_nonce[0] = this->mac_data_[0];
  auth_nonce[1] = this->mac_data_[1];
  auth_nonce[2] = this->mac_data_[2];
  auth_nonce[3] = this->mac_data_[3];
  auth_nonce[4] = 0x01;
  auth_nonce[5] = packet20[0];
  auth_nonce[6] = packet20[1];
  auth_nonce[7] = packet20[2];
  auth_nonce[8] = 15;

  std::array<uint8_t, 16> authenticator{};
  aes_encrypt_telink(this->session_key_, auth_nonce, &authenticator);
  for (size_t i = 0; i < 15; i++) {
    authenticator[i] ^= packet20[i + 5];
  }

  std::array<uint8_t, 16> mic{};
  aes_encrypt_telink(this->session_key_, authenticator, &mic);
  packet20[3] = mic[0];
  packet20[4] = mic[1];

  std::array<uint8_t, 16> iv{};
  iv[1] = this->mac_data_[0];
  iv[2] = this->mac_data_[1];
  iv[3] = this->mac_data_[2];
  iv[4] = this->mac_data_[3];
  iv[5] = 0x01;
  iv[6] = packet20[0];
  iv[7] = packet20[1];
  iv[8] = packet20[2];

  std::array<uint8_t, 16> stream{};
  aes_encrypt_telink(this->session_key_, iv, &stream);
  for (size_t i = 0; i < 15; i++) {
    packet20[i + 5] ^= stream[i];
  }
}

void SmartLedzHub::decrypt_packet_(uint8_t *packet20) const {
  std::array<uint8_t, 16> iv{};
  iv[0] = this->mac_data_[0];
  iv[1] = this->mac_data_[1];
  iv[2] = this->mac_data_[2];
  iv[3] = packet20[0];
  iv[4] = packet20[1];
  iv[5] = packet20[2];
  iv[6] = packet20[3];
  iv[7] = packet20[4];

  std::array<uint8_t, 16> plaintext{};
  plaintext[0] = 0;
  for (size_t i = 0; i < 15; i++) {
    plaintext[i + 1] = iv[i];
  }

  std::array<uint8_t, 16> stream{};
  aes_encrypt_telink(this->session_key_, plaintext, &stream);
  for (size_t i = 0; i + 7 < 20; i++) {
    packet20[i + 7] ^= stream[i];
  }
}

bool SmartLedzHub::send_packet(uint16_t target, uint8_t opcode, const uint8_t *payload, size_t payload_len) {
  if (!this->session_ready_) {
    ESP_LOGW(TAG, "[%s] cannot send packet before pairing", this->parent_->address_str());
    return false;
  }
  if (this->control_handle_ == 0) {
    ESP_LOGW(TAG, "[%s] control handle not ready", this->parent_->address_str());
    return false;
  }

  uint8_t packet[20] = {0};
  packet[0] = this->packet_count_ & 0xFF;
  packet[1] = (this->packet_count_ >> 8) & 0xFF;
  packet[5] = target & 0xFF;
  packet[6] = (target >> 8) & 0xFF;
  packet[7] = opcode;
  packet[8] = this->vendor_id_ & 0xFF;
  packet[9] = (this->vendor_id_ >> 8) & 0xFF;

  auto data_len = std::min(payload_len, static_cast<size_t>(10));
  for (size_t i = 0; i < data_len; i++) {
    packet[10 + i] = payload[i];
  }

  this->encrypt_packet_(packet);

  auto status = esp_ble_gattc_write_char(this->parent_->get_gattc_if(), this->parent_->get_conn_id(),
                                         this->control_handle_, sizeof(packet), packet,
                                         ESP_GATT_WRITE_TYPE_NO_RSP, ESP_GATT_AUTH_REQ_NONE);
  if (status != ESP_GATT_OK) {
    ESP_LOGW(TAG, "[%s] send_packet write failed status=%d", this->parent_->address_str(), status);
    return false;
  }

  this->packet_count_++;
  if (this->packet_count_ == 0) {
    this->packet_count_ = 1;
  }

  return true;
}

bool SmartLedzHub::send_on_off(uint16_t target, bool on) {
  const uint8_t payload[1] = {static_cast<uint8_t>(on ? 0x01 : 0x00)};
  auto ok = this->send_packet(target, 0xD0, payload, sizeof(payload));
  if (ok) {
    auto &state = this->state_ref_(target);
    state.seen = true;
    state.has_power = true;
    state.power = on;
    if (!on) {
      state.has_online_brightness = true;
      state.online_brightness = 0;
    }
    state.last_update_ms = millis();
  }
  return ok;
}

bool SmartLedzHub::send_brightness(uint16_t target, uint8_t brightness) {
  const uint8_t payload[1] = {brightness};
  auto ok = this->send_packet(target, 0xD2, payload, sizeof(payload));
  if (ok) {
    auto &state = this->state_ref_(target);
    state.seen = true;
    state.brightness = brightness;
    state.last_update_ms = millis();
  }
  return ok;
}

bool SmartLedzHub::send_rgb(uint16_t target, uint8_t red, uint8_t green, uint8_t blue) {
  const uint8_t payload[4] = {0x04, red, green, blue};
  auto ok = this->send_packet(target, 0xE2, payload, sizeof(payload));
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
  const uint8_t payload[2] = {0x05, static_cast<uint8_t>(std::max(18, std::min(65, static_cast<int>(ct_raw))))};
  auto ok = this->send_packet(target, 0xE2, payload, sizeof(payload));
  if (ok) {
    auto &state = this->state_ref_(target);
    state.seen = true;
    state.has_ct = true;
    state.ct_raw = payload[1];
    state.last_update_ms = millis();
  }
  return ok;
}

bool SmartLedzHub::send_status_query(uint16_t target) {
  const uint8_t payload[1] = {0x10};
  return this->send_packet(target, 0xDA, payload, sizeof(payload));
}

bool SmartLedzHub::send_dimming_query(uint16_t target) {
  const uint8_t payload[5] = {0x01, 0x01, 0x04, 0x11, 0x02};
  return this->send_packet(target, 0xF0, payload, sizeof(payload));
}

void SmartLedzHub::reset_runtime_state_() {
  this->pairing_write_pending_ = false;
  this->pairing_read_pending_ = false;
  this->notify_register_pending_ = false;
  this->notify_enable_pending_ = false;
  this->session_ready_ = false;
  this->notify_handle_ = 0;
  this->control_handle_ = 0;
  this->pair_handle_ = 0;
  this->device_states_.clear();
  this->last_poll_ms_ = 0;
  this->poll_cursor_ = 0;
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
      if (!this->discover_characteristics_()) {
        this->parent_->disconnect();
        break;
      }
      this->begin_pairing_();
      break;
    }

    case ESP_GATTC_WRITE_CHAR_EVT: {
      if (param->write.conn_id != this->parent_->get_conn_id()) {
        break;
      }
      if (param->write.status != ESP_GATT_OK) {
        ESP_LOGW(TAG, "[%s] write failed handle=0x%04X status=%d", this->parent_->address_str(), param->write.handle,
                 param->write.status);
        if (param->write.handle == this->pair_handle_) {
          this->parent_->disconnect();
        }
        break;
      }
      if (this->pairing_write_pending_ && param->write.handle == this->pair_handle_) {
        this->pairing_write_pending_ = false;
        this->pairing_read_pending_ = true;
        auto status = esp_ble_gattc_read_char(this->parent_->get_gattc_if(), this->parent_->get_conn_id(),
                                              this->pair_handle_, ESP_GATT_AUTH_REQ_NONE);
        if (status != ESP_GATT_OK) {
          this->pairing_read_pending_ = false;
          ESP_LOGW(TAG, "[%s] pair read failed status=%d", this->parent_->address_str(), status);
          this->parent_->disconnect();
        }
      } else if (this->notify_enable_pending_ && param->write.handle == this->notify_handle_) {
        this->notify_enable_pending_ = false;
        this->node_state = espbt::ClientState::ESTABLISHED;
        ESP_LOGI(TAG, "[%s] Smart LEDZ session established", this->parent_->address_str());
      }
      break;
    }

    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
      if (!this->notify_register_pending_ || param->reg_for_notify.handle != this->notify_handle_) {
        break;
      }
      this->notify_register_pending_ = false;
      if (param->reg_for_notify.status != ESP_GATT_OK) {
        ESP_LOGW(TAG, "[%s] REG_FOR_NOTIFY failed status=%d", this->parent_->address_str(), param->reg_for_notify.status);
      }
      this->write_notify_enable_();
      break;
    }

    case ESP_GATTC_READ_CHAR_EVT: {
      if (param->read.conn_id != this->parent_->get_conn_id()) {
        break;
      }
      if (!this->pairing_read_pending_ || param->read.handle != this->pair_handle_) {
        break;
      }
      this->pairing_read_pending_ = false;
      if (param->read.status != ESP_GATT_OK) {
        ESP_LOGW(TAG, "[%s] pair read status=%d", this->parent_->address_str(), param->read.status);
        this->parent_->disconnect();
        break;
      }
      this->handle_pairing_response_(param->read.value, param->read.value_len);
      break;
    }

    case ESP_GATTC_NOTIFY_EVT: {
      if (!this->session_ready_) {
        break;
      }
      if (param->notify.conn_id != this->parent_->get_conn_id() || param->notify.handle != this->notify_handle_) {
        break;
      }
      if (param->notify.value_len != 20) {
        break;
      }

      uint8_t packet[20] = {0};
      memcpy(packet, param->notify.value, sizeof(packet));
      this->decrypt_packet_(packet);

      const uint8_t opcode = packet[7];
      const uint16_t src = packet[3] | (static_cast<uint16_t>(packet[4]) << 8);
      const uint8_t *payload10 = &packet[10];
      constexpr size_t payload_len = 10;

      switch (opcode) {
        case 0xDC:
          this->update_device_from_online_status_(payload10, payload_len);
          break;
        case 0xDB:
          this->update_device_from_status_(src, payload10, payload_len);
          break;
        case 0xEA:
          this->update_device_from_f1_response_(src, payload10, payload_len);
          break;
        default:
          break;
      }

      ESP_LOGV(TAG, "[%s] notify opcode=0x%02X src=0x%04X", this->parent_->address_str(), opcode, src);
      break;
    }

    default:
      break;
  }
}

}  // namespace smart_ledz
}  // namespace esphome

#endif
