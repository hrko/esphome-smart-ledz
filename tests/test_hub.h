#pragma once
#include "components/smart_ledz/smart_ledz.h"
#include <esp_timer.h>
#include <esp_random.h>
#include <pair_fixture.h>
#include <cassert>
using namespace esphome::smart_ledz;

class TestHub : public SmartLedzHub {
 public:
  esphome::ble_client::BLEClient client;
  TestHub() { parent_ = &client; setup(); set_poll_interval_ms(0); }
  void start_setup(unsigned step) {
    session_.set_link_context(1, 1, client.address);
    session_.set_characteristic_handles(1, 2, 3);
    assert(session_.begin_pairing());
    if (step >= 1) assert(session_.handle_write_char_evt(3, ESP_GATT_OK, nullptr, nullptr));
    if (step >= 2) assert(session_.handle_read_char_evt(3, ESP_GATT_OK, pair_response.data(), pair_response.size(), nullptr, nullptr));
    if (step >= 3) assert(session_.handle_reg_for_notify_evt(1, ESP_GATT_OK, nullptr, nullptr));
    assert(!is_session_ready());
  }
  void pair() {
    session_.set_link_context(1, 1, client.address);
    session_.set_characteristic_handles(1, 2, 3);
    assert(session_.begin_pairing());
    session_.handle_write_char_evt(3, ESP_GATT_OK, nullptr, nullptr);
    session_.handle_read_char_evt(3, ESP_GATT_OK, pair_response.data(), pair_response.size(), nullptr, nullptr);
    session_.handle_reg_for_notify_evt(1, ESP_GATT_OK, nullptr, nullptr);
    session_.handle_write_char_evt(1, ESP_GATT_OK, nullptr, nullptr);
    assert(is_session_ready());
  }
  void ack(int status = ESP_GATT_OK) {
    esp_ble_gattc_cb_param_t event{};
    event.write = {1, 2, status};
    gattc_event_handler(ESP_GATTC_WRITE_CHAR_EVT, 1, &event);
  }
  void reset() { reset_runtime_state_(); }
  uint16_t active_target() const { return active_tx_.target; }
  uint8_t active_opcode() const { return active_tx_.opcode; }
  bool active() const { return has_active_tx_; }
  size_t queued_opcode(uint8_t opcode) const {
    size_t count = 0;
    for (const auto &cmd : tx_queue_) if (cmd.opcode == opcode) ++count;
    return count;
  }
  size_t queue_size() { return tx_queue_.size(); }
  size_t holds() { return target_hold_until_ms_.size(); }
};
