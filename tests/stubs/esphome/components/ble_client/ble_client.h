#pragma once
#include <esp_gattc_api.h>
#include "esphome/components/esp32_ble_tracker/esp32_ble_tracker.h"
namespace esphome::ble_client {
struct Characteristic { uint16_t handle = 1; };
class BLEClient {
 public:
  unsigned disconnects = 0;
  esp_bd_addr_t address{};
  Characteristic characteristic;
  const char *address_str() { return "test"; }
  esp_gatt_if_t get_gattc_if() { return 1; }
  uint16_t get_conn_id() { return 1; }
  uint8_t *get_remote_bda() { return address; }
  Characteristic *get_characteristic(esp32_ble_tracker::ESPBTUUID, esp32_ble_tracker::ESPBTUUID) { return &characteristic; }
  void disconnect() { ++disconnects; }
};
class BLEClientNode {
 public:
  virtual ~BLEClientNode() = default;
  virtual void gattc_event_handler(esp_gattc_cb_event_t, esp_gatt_if_t, esp_ble_gattc_cb_param_t *) {}
  esp32_ble_tracker::ClientState node_state{};
 protected:
  BLEClient *parent_ = nullptr;
};
}
