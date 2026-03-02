#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

#include <esp_gattc_api.h>

namespace esp_telink_mesh {
namespace v1 {

struct DecryptedNotify {
  uint8_t opcode{0};
  uint16_t src{0};
  std::array<uint8_t, 10> payload{};
};

class SessionClient {
 public:
  void configure(const std::string &mesh_name, const std::string &mesh_password, uint16_t vendor_id);
  void reset();

  bool set_link_context(esp_gatt_if_t gattc_if, uint16_t conn_id, const esp_bd_addr_t remote_bda);
  void set_characteristic_handles(uint16_t notify_handle, uint16_t control_handle, uint16_t pair_handle);

  bool begin_pairing();
  bool handle_write_char_evt(uint16_t handle, esp_gatt_status_t status, bool *need_disconnect,
                             bool *session_established);
  bool handle_reg_for_notify_evt(uint16_t handle, esp_gatt_status_t status, bool *need_disconnect,
                                  bool *session_established);
  bool handle_read_char_evt(uint16_t handle, esp_gatt_status_t status, const uint8_t *value, uint16_t value_len,
                            bool *need_disconnect, bool *session_established);
  bool handle_notify_packet(uint16_t handle, const uint8_t *value, uint16_t value_len, DecryptedNotify *out) const;

  bool send_mesh_command(uint16_t target, uint8_t opcode, const uint8_t *payload, size_t payload_len);

  bool is_session_ready() const { return this->session_ready_; }
  uint16_t notify_handle() const { return this->notify_handle_; }
  uint16_t control_handle() const { return this->control_handle_; }
  uint16_t pair_handle() const { return this->pair_handle_; }

 private:
  bool register_notify_();
  bool write_notify_enable_(bool *session_established);
  bool handle_pairing_response_(const uint8_t *data, uint16_t len, bool *need_disconnect, bool *session_established);
  void encrypt_packet_(uint8_t *packet20) const;
  void decrypt_packet_(uint8_t *packet20) const;
  void set_mac_data_from_bda_(const esp_bd_addr_t remote_bda);

  std::string mesh_name_;
  std::string mesh_password_;

  uint16_t vendor_id_{0x0211};

  esp_gatt_if_t gattc_if_{ESP_GATT_IF_NONE};
  uint16_t conn_id_{0};
  esp_bd_addr_t remote_bda_{};

  uint16_t notify_handle_{0};
  uint16_t control_handle_{0};
  uint16_t pair_handle_{0};

  std::array<uint8_t, 6> mac_data_{};
  std::array<uint8_t, 8> pair_random_{};
  std::array<uint8_t, 16> session_key_{};

  bool pairing_write_pending_{false};
  bool pairing_read_pending_{false};
  bool notify_register_pending_{false};
  bool notify_enable_pending_{false};
  bool control_write_pending_{false};
  bool session_ready_{false};

  uint16_t packet_count_{1};
};

}  // namespace v1
}  // namespace esp_telink_mesh
