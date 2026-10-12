# Regression tests

Clone `esp-telink-mesh` and `smartledz-protocol` beside this repository, at the
commits pinned in `components/smart_ledz/__init__.py`. Then run `bash tests/run.sh`
on Linux/WSL with g++ (C++17). Alternate checkout locations can be supplied with
`ESP_TELINK_MESH_DIR` and `SMARTLEDZ_PROTOCOL_DIR`.

The tests compile the production hub, session client, command encoder, and state
decoder against fake ESPHome/GATT/clock/AES APIs. They verify the 400 ms power-on
hold before and after 2^31 ms, at 25 and 30 days, and across the uint32_t rollover;
expiry cleanup; repeated power cycles; missing write completion with an empty or
polling queue; timeout cleanup; late completion; and sending after re-pairing.

Physical BLE disconnection/reconnection and encryption are not simulated. The
host tests verify that recovery requests a disconnect; reconnecting the actual
device still depends on ESPHome's BLE client and `auto_connect: true`.

Development tools are pinned by `mise.lock` (Python/uv, Linux x64 and Windows
x64) and `uv.lock` (ESPHome, Ruff, ty, and transitive Python dependencies).
Run `mise install --locked python uv`, `mise run lint`, and `mise run esphome-compile`.
The compile fixture targets ESP32-S3 with the ESP-IDF framework. These commands
compile only; they do not upload firmware to a device.
