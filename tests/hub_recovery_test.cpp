#include "components/smart_ledz/smart_ledz.h"
#include <esp_timer.h>
#include <esp_random.h>
#include <pair_fixture.h>
#include <cassert>
#include <iostream>
using namespace esphome::smart_ledz;

class TestHub : public SmartLedzHub {
 public:
  esphome::ble_client::BLEClient client;
  TestHub() { parent_ = &client; setup(); set_poll_interval_ms(0); }
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
  void ack() { session_.handle_write_char_evt(2, ESP_GATT_OK, nullptr, nullptr); }
  size_t queue_size() { return tx_queue_.size(); }
  size_t holds() { return target_hold_until_ms_.size(); }
};

static void advance(TestHub &hub, uint32_t ms, bool ack = true) {
  fake_time_us += static_cast<int64_t>(ms) * 1000;
  if (ack) hub.ack();
  hub.loop();
}

int main() {
  // Both sides of the signed boundary, and the 49.7-day millis rollover.
  for (int64_t start : {1000LL, 2147483247LL, 2147483648LL, 2160000000LL, 2592000000LL, 4294967095LL}) {
    fake_time_us = start * 1000;
    TestHub hub;
    hub.pair();
    control_writes = 0;
    assert(hub.send_on_off(1, true));
    assert(hub.send_brightness(1, 50));
    assert(hub.send_rgb(1, 10, 20, 30));
    hub.loop();
    assert(control_writes == 1);
    advance(hub, 399);
    assert(control_writes == 1);  // Brightness/color must respect the full hold.
    advance(hub, 1);
    assert(control_writes == 2);
    assert(hub.holds() == 0);
    for (int i = 0; i < 10; ++i) advance(hub, 200);
    assert(hub.queue_size() == 0);
    assert(hub.send_on_off(1, false));
    advance(hub, 200);
    assert(hub.holds() == 0);
    assert(hub.send_on_off(1, true));
    assert(hub.send_brightness(1, 70));
    for (int i = 0; i < 10; ++i) advance(hub, 200);
    assert(hub.queue_size() == 0);
    assert(hub.client.disconnects == 0);
  }

  for (bool polling : {false, true}) {
    fake_time_us = 4294967000LL * 1000;
    TestHub hub;
    hub.pair();
    if (polling) { hub.set_poll_interval_ms(2000); hub.register_polled_target(1); }
    assert(hub.send_packet(1, 0xd0, nullptr, 0));
    hub.loop(); // No verification queries: also test timeout with an empty queue.
    if (!polling) assert(hub.queue_size() == 0);
    for (int i = 0; i < 9; ++i) advance(hub, 1000, false);
    assert(hub.client.disconnects == 0);
    assert(hub.send_brightness(1, 30));
    advance(hub, 1000, false);
    assert(hub.client.disconnects == 1);
    assert(!hub.is_session_ready());
    assert(hub.queue_size() == 0);
    assert(hub.holds() == 0);
    advance(hub, 1000, false);
    assert(hub.client.disconnects == 1);
    assert(!hub.send_on_off(1, true));
    hub.ack(); // A late completion must not restore the old session.
    assert(!hub.is_session_ready());
    hub.pair();
    auto before = control_writes;
    assert(hub.send_on_off(1, true));
    advance(hub, 200);
    assert(control_writes == before + 1);
  }
  {
    TestHub hub;
    fake_random = 0xffffff;
    hub.pair();
    assert(hub.send_on_off(1, true));
    hub.loop();
    assert(hub.client.disconnects == 0);
    advance(hub, 200);
    assert(hub.client.disconnects == 1 && !hub.is_session_ready());
    assert(hub.queue_size() == 0);
    fake_random = 123;
    hub.pair();
    assert(hub.send_on_off(1, true));
    advance(hub, 200);
    assert(hub.is_session_ready());
  }
  std::cout << "Hub rollover and recovery tests passed\n";
}
