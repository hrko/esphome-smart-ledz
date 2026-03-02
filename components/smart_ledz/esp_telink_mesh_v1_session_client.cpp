#include "esp_telink_mesh_v1_session_client.h"

#include <algorithm>
#include <array>
#include <cstring>

#include <esp_log.h>
#include <esp_random.h>

#define MBEDTLS_AES_ALT
#include <aes_alt.h>

namespace esp_telink_mesh {
namespace v1 {

namespace {

static const char *const TAG = "esp_telink_mesh";

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
  if (out == nullptr) {
    return false;
  }

  uint8_t key_rev[16];
  uint8_t data_rev[16];
  uint8_t enc_rev[16];

  for (size_t i = 0; i < 16; i++) {
    key_rev[i] = key[15 - i];
    data_rev[i] = data[15 - i];
  }

  mbedtls_aes_context ctx;
  mbedtls_aes_init(&ctx);
  const auto key_status = mbedtls_aes_setkey_enc(&ctx, key_rev, 128);
  if (key_status != 0) {
    mbedtls_aes_free(&ctx);
    return false;
  }
  const auto enc_status = mbedtls_aes_crypt_ecb(&ctx, ESP_AES_ENCRYPT, data_rev, enc_rev);
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
  if (data2_8 == nullptr || out == nullptr) {
    return false;
  }

  std::array<uint8_t, 16> data{};
  for (size_t i = 0; i < 8; i++) {
    data[i] = data1[i];
    data[8 + i] = data2_8[i];
  }
  return aes_encrypt_telink(mesh_xor_key(mesh_name, mesh_password), data, out);
}

void set_flag_(bool *out, bool value) {
  if (out != nullptr) {
    *out = value;
  }
}

}  // namespace

void SessionClient::configure(const std::string &mesh_name, const std::string &mesh_password, uint16_t vendor_id) {
  this->mesh_name_ = mesh_name;
  this->mesh_password_ = mesh_password;
  this->vendor_id_ = vendor_id;
}

void SessionClient::reset() {
  this->notify_handle_ = 0;
  this->control_handle_ = 0;
  this->pair_handle_ = 0;

  this->pairing_write_pending_ = false;
  this->pairing_read_pending_ = false;
  this->notify_register_pending_ = false;
  this->notify_enable_pending_ = false;
  this->session_ready_ = false;

  this->packet_count_ = 1;
}

bool SessionClient::set_link_context(esp_gatt_if_t gattc_if, uint16_t conn_id, const esp_bd_addr_t remote_bda) {
  if (gattc_if == ESP_GATT_IF_NONE) {
    return false;
  }
  this->gattc_if_ = gattc_if;
  this->conn_id_ = conn_id;
  memcpy(this->remote_bda_, remote_bda, sizeof(this->remote_bda_));
  this->set_mac_data_from_bda_(remote_bda);
  return true;
}

void SessionClient::set_characteristic_handles(uint16_t notify_handle, uint16_t control_handle, uint16_t pair_handle) {
  this->notify_handle_ = notify_handle;
  this->control_handle_ = control_handle;
  this->pair_handle_ = pair_handle;
}

void SessionClient::set_mac_data_from_bda_(const esp_bd_addr_t remote_bda) {
  this->mac_data_[0] = remote_bda[5];
  this->mac_data_[1] = remote_bda[4];
  this->mac_data_[2] = remote_bda[3];
  this->mac_data_[3] = remote_bda[2];
  this->mac_data_[4] = remote_bda[1];
  this->mac_data_[5] = remote_bda[0];
}

bool SessionClient::begin_pairing() {
  if (this->pair_handle_ == 0 || this->gattc_if_ == ESP_GATT_IF_NONE) {
    ESP_LOGW(TAG, "pairing prerequisites are not ready");
    return false;
  }

  std::array<uint8_t, 16> pair_data{};
  esp_fill_random(this->pair_random_.data(), this->pair_random_.size());
  for (size_t i = 0; i < this->pair_random_.size(); i++) {
    pair_data[i] = this->pair_random_[i];
  }

  std::array<uint8_t, 16> enc_data{};
  if (!key_encrypt(this->mesh_name_, this->mesh_password_, pair_data, &enc_data)) {
    ESP_LOGE(TAG, "key_encrypt failed");
    return false;
  }

  uint8_t packet[17] = {0};
  packet[0] = 0x0C;
  for (size_t i = 0; i < 8; i++) {
    packet[1 + i] = pair_data[i];
    packet[9 + i] = enc_data[i];
  }

  const auto status = esp_ble_gattc_write_char(this->gattc_if_, this->conn_id_, this->pair_handle_, sizeof(packet),
                                                packet, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
  if (status != ESP_GATT_OK) {
    ESP_LOGW(TAG, "pair write failed, status=%d", status);
    return false;
  }

  this->pairing_write_pending_ = true;
  return true;
}

bool SessionClient::handle_write_char_evt(uint16_t handle, esp_gatt_status_t status, bool *need_disconnect,
                                          bool *session_established) {
  set_flag_(need_disconnect, false);
  set_flag_(session_established, false);

  if (status != ESP_GATT_OK) {
    ESP_LOGW(TAG, "write failed handle=0x%04X status=%d", handle, status);
    if (handle == this->pair_handle_) {
      set_flag_(need_disconnect, true);
    }
    return false;
  }

  if (this->pairing_write_pending_ && handle == this->pair_handle_) {
    this->pairing_write_pending_ = false;
    this->pairing_read_pending_ = true;
    const auto read_status =
        esp_ble_gattc_read_char(this->gattc_if_, this->conn_id_, this->pair_handle_, ESP_GATT_AUTH_REQ_NONE);
    if (read_status != ESP_GATT_OK) {
      this->pairing_read_pending_ = false;
      ESP_LOGW(TAG, "pair read failed status=%d", read_status);
      set_flag_(need_disconnect, true);
      return false;
    }
    return true;
  }

  if (this->notify_enable_pending_ && handle == this->notify_handle_) {
    this->notify_enable_pending_ = false;
    set_flag_(session_established, true);
    return true;
  }

  return false;
}

bool SessionClient::handle_reg_for_notify_evt(uint16_t handle, esp_gatt_status_t status, bool *need_disconnect,
                                              bool *session_established) {
  set_flag_(need_disconnect, false);
  set_flag_(session_established, false);

  if (!this->notify_register_pending_ || handle != this->notify_handle_) {
    return false;
  }

  this->notify_register_pending_ = false;
  if (status != ESP_GATT_OK) {
    ESP_LOGW(TAG, "REG_FOR_NOTIFY failed status=%d", status);
  }
  return this->write_notify_enable_(session_established);
}

bool SessionClient::handle_read_char_evt(uint16_t handle, esp_gatt_status_t status, const uint8_t *value,
                                         uint16_t value_len, bool *need_disconnect, bool *session_established) {
  set_flag_(need_disconnect, false);
  set_flag_(session_established, false);

  if (!this->pairing_read_pending_ || handle != this->pair_handle_) {
    return false;
  }

  this->pairing_read_pending_ = false;
  if (status != ESP_GATT_OK) {
    ESP_LOGW(TAG, "pair read status=%d", status);
    set_flag_(need_disconnect, true);
    return false;
  }

  return this->handle_pairing_response_(value, value_len, need_disconnect, session_established);
}

bool SessionClient::handle_pairing_response_(const uint8_t *data, uint16_t len, bool *need_disconnect,
                                             bool *session_established) {
  if (data == nullptr || len < 9) {
    ESP_LOGW(TAG, "invalid pairing response len=%u", len);
    set_flag_(need_disconnect, true);
    return false;
  }

  if (!generate_session_key(this->mesh_name_, this->mesh_password_, this->pair_random_, data + 1, &this->session_key_)) {
    ESP_LOGW(TAG, "session key generation failed");
    set_flag_(need_disconnect, true);
    return false;
  }

  this->session_ready_ = true;
  this->packet_count_ = static_cast<uint16_t>(esp_random() & 0xFFFF);
  if (this->packet_count_ == 0) {
    this->packet_count_ = 1;
  }

  if (!this->register_notify_()) {
    this->write_notify_enable_(session_established);
  }

  return true;
}

bool SessionClient::register_notify_() {
  if (this->notify_handle_ == 0 || this->gattc_if_ == ESP_GATT_IF_NONE) {
    return false;
  }

  const auto status = esp_ble_gattc_register_for_notify(this->gattc_if_, this->remote_bda_, this->notify_handle_);
  if (status != ESP_GATT_OK) {
    ESP_LOGW(TAG, "register_for_notify failed status=%d", status);
    return false;
  }

  this->notify_register_pending_ = true;
  return true;
}

bool SessionClient::write_notify_enable_(bool *session_established) {
  if (this->notify_handle_ == 0 || this->gattc_if_ == ESP_GATT_IF_NONE) {
    return false;
  }

  uint8_t value = 0x01;
  auto status = esp_ble_gattc_write_char(this->gattc_if_, this->conn_id_, this->notify_handle_, sizeof(value), &value,
                                         ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
  if (status == ESP_GATT_OK) {
    this->notify_enable_pending_ = true;
    return true;
  }

  status = esp_ble_gattc_write_char(this->gattc_if_, this->conn_id_, this->notify_handle_, sizeof(value), &value,
                                    ESP_GATT_WRITE_TYPE_NO_RSP, ESP_GATT_AUTH_REQ_NONE);
  if (status != ESP_GATT_OK) {
    ESP_LOGW(TAG, "notify enable write failed, status=%d", status);
    return false;
  }

  set_flag_(session_established, true);
  return true;
}

void SessionClient::encrypt_packet_(uint8_t *packet20) const {
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

void SessionClient::decrypt_packet_(uint8_t *packet20) const {
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

bool SessionClient::handle_notify_packet(uint16_t handle, const uint8_t *value, uint16_t value_len,
                                         DecryptedNotify *out) const {
  if (!this->session_ready_ || handle != this->notify_handle_ || value == nullptr || value_len != 20 || out == nullptr) {
    return false;
  }

  uint8_t packet[20] = {0};
  memcpy(packet, value, sizeof(packet));
  this->decrypt_packet_(packet);

  out->opcode = packet[7];
  out->src = packet[3] | (static_cast<uint16_t>(packet[4]) << 8);
  for (size_t i = 0; i < out->payload.size(); i++) {
    out->payload[i] = packet[10 + i];
  }
  return true;
}

bool SessionClient::send_mesh_command(uint16_t target, uint8_t opcode, const uint8_t *payload, size_t payload_len) {
  if (!this->session_ready_) {
    return false;
  }
  if (this->control_handle_ == 0 || this->gattc_if_ == ESP_GATT_IF_NONE) {
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

  const auto data_len = std::min(payload_len, static_cast<size_t>(10));
  for (size_t i = 0; i < data_len; i++) {
    packet[10 + i] = payload[i];
  }

  this->encrypt_packet_(packet);

  const auto status =
      esp_ble_gattc_write_char(this->gattc_if_, this->conn_id_, this->control_handle_, sizeof(packet), packet,
                               ESP_GATT_WRITE_TYPE_NO_RSP, ESP_GATT_AUTH_REQ_NONE);
  if (status != ESP_GATT_OK) {
    ESP_LOGW(TAG, "send_packet write failed status=%d", status);
    return false;
  }

  this->packet_count_++;
  if (this->packet_count_ == 0) {
    this->packet_count_ = 1;
  }

  return true;
}

}  // namespace v1
}  // namespace esp_telink_mesh
