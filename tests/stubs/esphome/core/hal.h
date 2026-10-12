#pragma once
#include <esp_timer.h>
namespace esphome { inline uint32_t millis() { return static_cast<uint32_t>(fake_time_us / 1000); } }
