#include "components/smart_ledz/smart_ledz.h"
#include <esp_timer.h>
#include <esp_random.h>
#include <pair_fixture.h>
#include <cassert>
#include <iostream>
using namespace esphome::smart_ledz;

#include "test_hub.h"

static void advance(TestHub &hub, uint32_t ms, bool ack = true) {
  fake_time_us += static_cast<int64_t>(ms) * 1000;
  if (ack) hub.ack();
  hub.loop();
}

int main() {
  for (unsigned step = 0; step < 4; ++step) {
    fake_time_us = 0;
    TestHub hub;
    hub.start_setup(step);
    advance(hub, 9999, false);
    assert(hub.client.disconnects == 0);
    advance(hub, 1, false);
    assert(hub.client.disconnects == 1 && !hub.is_session_ready());
    advance(hub, 10000, false);
    assert(hub.client.disconnects == 1);
    esp_ble_gattc_cb_param_t event{};
    event.write = {1, 1, ESP_GATT_OK};
    hub.gattc_event_handler(ESP_GATTC_WRITE_CHAR_EVT, 1, &event);
    assert(!hub.is_session_ready());
    hub.pair();
    assert(hub.is_session_ready());
  }
  for (unsigned step = 0; step < 4; ++step) {
    TestHub hub;
    hub.start_setup(step);
    esp_ble_gattc_cb_param_t event{};
    if (step == 0 || step == 3) {
      event.write = {1, static_cast<uint16_t>(step == 0 ? 3 : 1), 1};
      hub.gattc_event_handler(ESP_GATTC_WRITE_CHAR_EVT, 1, &event);
    } else if (step == 1) {
      event.read = {1, 3, 1, nullptr, 0};
      hub.gattc_event_handler(ESP_GATTC_READ_CHAR_EVT, 1, &event);
    } else {
      event.reg_for_notify = {1, 1};
      hub.gattc_event_handler(ESP_GATTC_REG_FOR_NOTIFY_EVT, 1, &event);
    }
    assert(hub.client.disconnects == 1 && !hub.is_session_ready());
    assert(hub.queue_size() == 0);
    hub.pair();
    assert(hub.is_session_ready());
  }
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
  std::cout << "Hub setup, rollover and recovery tests passed\n";
}
