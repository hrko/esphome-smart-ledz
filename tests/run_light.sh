#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
mesh=${ESP_TELINK_MESH_DIR:-../esp-telink-mesh}
protocol=${SMARTLEDZ_PROTOCOL_DIR:-../smartledz-protocol}
binary=$(mktemp)
trap 'rm -f "$binary"' EXIT
g++ -std=c++17 -Wall -Wextra -Werror -DUSE_ESP32 \
  -Itests/stubs -I"$mesh/tests/stubs" -I"$mesh/include" -I"$protocol/include" -I. \
  components/smart_ledz/light/smart_ledz_light.cpp components/smart_ledz/smart_ledz.cpp \
  "$mesh/src/session_client.cpp" "$protocol/src/commands.cpp" "$protocol/src/state_codec.cpp" \
  "$protocol/src/color_math.cpp" tests/light_sync_test.cpp -lcrypto -o "$binary"
"$binary"
