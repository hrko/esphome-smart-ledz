# Architecture

## Goals

- Keep existing ESPHome YAML interface stable (`smart_ledz` + `light platform: smart_ledz`).
- Separate Telink mesh transport/session logic from Smart LEDZ protocol logic.
- Prepare code for future extraction as reusable libraries.

## Layered Structure

The component is split into three layers.

1. Integration layer (`esphome::smart_ledz`)
- Path: `components/smart_ledz/smart_ledz.*`, `components/smart_ledz/light/*`
- Depends on ESPHome (`BLEClientNode`, `LightOutput`).
- Owns scheduling concerns: TX queue, poll timing, listener fanout.

2. ESP transport/session layer (`esp_telink_mesh::v1`)
- Path: external PlatformIO library `esp-telink-mesh`
- Depends on ESP-IDF only.
- Owns Telink pairing/session key lifecycle, packet crypto, GATT write/read/notify decode.

3. Product protocol layer (`smartledz_protocol::v1`)
- Path: external PlatformIO library `smartledz-protocol`
- Pure C++ (STL only).
- Owns Smart LEDZ opcode/payload builders, notify payload parsing, device state patch logic, and CT/RGB conversion.

## Data Flow

- `SmartLedzLightOutput::write_state()` converts HA light state to Smart LEDZ commands.
- `SmartLedzHub` enqueues commands and sends via `esp_telink_mesh::v1::SessionClient`.
- Notify packets are decrypted by `SessionClient`, then parsed/applied by `smartledz_protocol::v1` into `DeviceStateSnapshot`.
- Hub notifies listeners (`SmartLedzStateListener`) for state sync.

## Compatibility Contract

- YAML schema remains unchanged.
- Existing behavior for on/off, brightness, CT, RGB, polling and post-write verification is preserved.
- `*_v1_*` namespaces/files represent the first versioned API boundary for future extraction.

## Planned Next Steps

- Add reusable test vectors for crypto and protocol frames.
- Add CI checks in `esp-telink-mesh` and `smartledz-protocol` to keep API compatibility explicit.
