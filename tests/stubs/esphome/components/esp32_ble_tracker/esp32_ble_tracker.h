#pragma once
namespace esphome::esp32_ble_tracker {
struct ESPBTUUID { static ESPBTUUID from_raw(const char *) { return {}; } };
enum class ClientState { ESTABLISHED };
}
