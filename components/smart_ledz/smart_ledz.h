#pragma once

#ifdef USE_ESP32

#include "esphome/components/ble_client/ble_client.h"
#include "esphome/components/esp32_ble_tracker/esp32_ble_tracker.h"
#include "esphome/core/component.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace esphome {
namespace smart_ledz {

namespace espbt = esphome::esp32_ble_tracker;

struct SmartLedzDeviceStateSnapshot {
  bool seen{false};
  uint8_t online_status{0};
  bool has_online_brightness{false};
  uint8_t online_brightness{0};
  bool has_power{false};
  bool power{false};
  // True when the most recent update for this snapshot included brightness.
  bool has_brightness{false};
  uint8_t brightness{0};
  // Raw type byte from 0xDC online status notification (historically called CH).
  uint8_t type_raw{0};
  bool has_ct{false};
  uint8_t ct_raw{0};
  bool has_rgb{false};
  std::array<uint8_t, 3> rgb{0, 0, 0};
  uint32_t last_update_ms{0};
};

enum SmartLedzDeviceType : uint8_t {
  SMART_LEDZ_DEVICE_TYPE_DIMMABLE = 1,
  SMART_LEDZ_DEVICE_TYPE_TUNABLE = 2,
  SMART_LEDZ_DEVICE_TYPE_SYNCA = 3,
};

class SmartLedzStateListener {
 public:
  virtual ~SmartLedzStateListener() = default;
  virtual void on_smart_ledz_state_update(uint16_t address, const SmartLedzDeviceStateSnapshot &state) = 0;
};

class SmartLedzHub : public ble_client::BLEClientNode, public Component {
 public:
  using DeviceState = SmartLedzDeviceStateSnapshot;

  void set_mesh_name(const std::string &mesh_name) { this->mesh_name_ = mesh_name; }
  void set_mesh_password(const std::string &mesh_password) { this->mesh_password_ = mesh_password; }
  void set_vendor_id(uint16_t vendor_id) { this->vendor_id_ = vendor_id; }

  bool is_session_ready() const { return this->session_ready_; }

  bool send_packet(uint16_t target, uint8_t opcode, const uint8_t *payload, size_t payload_len);
  bool send_on_off(uint16_t target, bool on);
  bool send_brightness(uint16_t target, uint8_t brightness);
  bool send_rgb(uint16_t target, uint8_t red, uint8_t green, uint8_t blue);
  bool send_ct_raw(uint16_t target, uint8_t ct_raw);
  bool send_status_query(uint16_t target);
  bool send_dimming_query(uint16_t target);

  bool get_device_state(uint16_t target, DeviceState *out) const;
  void register_state_listener(SmartLedzStateListener *listener);
  void register_polled_target(uint16_t target);
  void set_poll_interval_ms(uint32_t poll_interval_ms) { this->poll_interval_ms_ = poll_interval_ms; }

  void setup() override { this->reset_runtime_state_(); }
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::BLUETOOTH; }

  void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                           esp_ble_gattc_cb_param_t *param) override;

 protected:
  bool discover_characteristics_();
  bool register_notify_();
  bool write_notify_enable_();
  void begin_pairing_();
  void handle_pairing_response_(const uint8_t *data, uint16_t len);
  void reset_runtime_state_();
  void update_mac_data_();
  void encrypt_packet_(uint8_t *packet20) const;
  void decrypt_packet_(uint8_t *packet20) const;
  void update_device_from_online_status_(const uint8_t *payload10, size_t payload_len);
  void update_device_from_status_(uint16_t src, const uint8_t *payload10, size_t payload_len);
  void update_device_from_f1_response_(uint16_t src, const uint8_t *payload10, size_t payload_len);
  void notify_state_update_(uint16_t address);
  DeviceState &state_ref_(uint16_t address);

  std::string mesh_name_;
  std::string mesh_password_;

  uint16_t vendor_id_{0x0211};
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
  bool session_ready_{false};

  uint16_t packet_count_{1};
  std::map<uint16_t, DeviceState> device_states_;
  std::vector<SmartLedzStateListener *> state_listeners_;
  std::set<uint16_t> polled_targets_;
  uint32_t poll_interval_ms_{2000};
  uint32_t last_poll_ms_{0};
  uint16_t poll_cursor_{0};
};

}  // namespace smart_ledz
}  // namespace esphome

#endif
