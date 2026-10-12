#pragma once
#include <esp_log.h>
#define ESP_LOGI(...) test_log(__VA_ARGS__)
#define ESP_LOGD(...) test_log(__VA_ARGS__)
#define ESP_LOGCONFIG(...) test_log(__VA_ARGS__)
#define YESNO(x) ((x) ? "YES" : "NO")
