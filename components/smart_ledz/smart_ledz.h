#pragma once

#ifdef USE_ESP32

#include "esphome/components/ble_client/ble_client.h"
#include "esphome/components/esp32_ble_tracker/esp32_ble_tracker.h"
#include "esphome/core/component.h"

#include "esp_telink_mesh_v1_session_client.h"
#include "smartledz_protocol_v1_state_codec.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace esphome {
namespace smart_ledz {

namespace espbt = esphome::esp32_ble_tracker;

using SmartLedzDeviceStateSnapshot = smartledz_protocol::v1::DeviceStateSnapshot;

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

  void set_mesh_name(const std::string &mesh_name);
  void set_mesh_password(const std::string &mesh_password);
  void set_vendor_id(uint16_t vendor_id);

  bool is_session_ready() const { return this->session_.is_session_ready(); }

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
  void set_tx_interval_ms(uint32_t tx_interval_ms) { this->tx_interval_ms_ = tx_interval_ms == 0 ? 1 : tx_interval_ms; }
  void set_power_on_settle_ms(uint32_t power_on_settle_ms) { this->power_on_settle_ms_ = power_on_settle_ms; }

  void setup() override { this->reset_runtime_state_(); }
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::BLUETOOTH; }

  void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                           esp_ble_gattc_cb_param_t *param) override;

 protected:
  struct TxCommand {
    uint16_t target{0};
    uint8_t opcode{0};
    std::array<uint8_t, 10> payload{};
    uint8_t payload_len{0};
    uint32_t not_before_ms{0};
    bool verify_after_send{false};
  };

  void configure_session_();
  bool discover_characteristics_();
  void reset_runtime_state_();
  bool send_packet_now_(uint16_t target, uint8_t opcode, const uint8_t *payload, size_t payload_len);
  bool enqueue_packet_(uint16_t target, uint8_t opcode, const uint8_t *payload, size_t payload_len,
                       uint32_t not_before_ms, bool verify_after_send);
  void process_tx_queue_(uint32_t now);
  void apply_power_on_hold_(uint16_t target, uint32_t now);
  void trim_queued_target_commands_(uint16_t target, uint8_t opcode, int8_t e2_subtype);
  void schedule_verify_queries_(uint16_t target, uint32_t now);

  void update_device_from_online_status_(const uint8_t *payload10, size_t payload_len);
  void update_device_from_status_(uint16_t src, const uint8_t *payload10, size_t payload_len);
  void update_device_from_f1_response_(uint16_t src, const uint8_t *payload10, size_t payload_len);
  void notify_state_update_(uint16_t address);
  DeviceState &state_ref_(uint16_t address);

  std::string mesh_name_;
  std::string mesh_password_;
  uint16_t vendor_id_{0x0211};

  esp_telink_mesh::v1::SessionClient session_;

  std::map<uint16_t, DeviceState> device_states_;
  std::vector<SmartLedzStateListener *> state_listeners_;
  std::set<uint16_t> polled_targets_;
  uint32_t poll_interval_ms_{2000};
  uint32_t last_poll_ms_{0};
  uint16_t poll_cursor_{0};
  std::deque<TxCommand> tx_queue_;
  std::map<uint16_t, uint32_t> target_hold_until_ms_;
  uint32_t tx_interval_ms_{120};
  uint32_t power_on_settle_ms_{400};
  uint32_t last_tx_ms_{0};
};

}  // namespace smart_ledz
}  // namespace esphome

#endif
