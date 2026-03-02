# esphome-smart-ledz

This is an external component for controlling Smart LEDZ devices from ESPHome.  
An ESP32 acts as a bridge to Smart LEDZ Mesh, so devices can be handled as standard `light` entities in Home Assistant.

Smart LEDZ is a Bluetooth mesh system built on top of Telink Mesh.

[日本語版はこちら](README.ja.md)

## What This Component Can Do

- Control fixtures and groups already registered in SmartLEDZ Fit as ESPHome `light` entities
- Control on/off, brightness, color temperature, and RGB for fixtures/groups

## What This Component Does Not Handle

- Adding new fixtures to the mesh
- Creating groups
- Adding/removing fixtures to/from groups
- Configuration management in SmartLEDZ Fit (all management operations above)

If you change fixture or group configuration, update it in SmartLEDZ Fit first, then re-export JSON and update your ESPHome YAML.

## Supported Environment

- ESPHome + ESP32
- `esp-idf` framework (recommended)

## Supported Device Types and Verification Status

`device_type` maps as follows:

- `dimmable`: Wireless dimming fixtures
- `tunable`: Tunable LEDZ series
- `synca`: Synca series

Verification status:

- Hardware testing has been performed only with the Synca series spotlight `SXS3025WB`.
- The implementation is intended to work with multi-device meshes, but only single-device mesh operation has been verified because only one compatible device is available.

## Prerequisites

- Fixtures are already added in the SmartLEDZ Fit app
- You can obtain a backup JSON that includes fixture/group information

## Export JSON from SmartLEDZ Fit

1. Open the SmartLEDZ Fit app
2. Open Settings
3. Open Backup/Restore
4. Select Backup
5. Select Backup to file
6. Transfer the saved JSON file to the machine where you edit ESPHome config

## Quick Start

1. Prepare the JSON exported from SmartLEDZ Fit (steps above)
2. Open the converter tool: https://hrko.github.io/esphome-smart-ledz/
3. Load the SmartLEDZ Fit export JSON
4. Select conversion targets (devices/groups)
5. Copy generated `ESPHome YAML` and `secrets.yaml`
6. Paste into your ESPHome node configuration and install

The converter tool is a static page. Loaded JSON is processed only in your browser and is not sent to a server. If you prioritize confidentiality, you can open `tools/smartledz-export-spa/index.html` locally. This also works fully offline (airplane mode equivalent).

## Configuration Reference

### `smart_ledz`

- Required: `id`, `ble_client_id`, `mesh_name`, `mesh_password`
- Optional: `vendor_id` (default: `0x0211`)
- Optional: `poll_interval` (default: `2s`)
- Optional: `tx_interval` (default: `120ms`)
- Optional: `power_on_settle` (default: `400ms`)

### `light` (`platform: smart_ledz`)

- Required: `id`, `name`, `smart_ledz_id`, `target`, `device_type`
- Optional: `ct_duv` (default: `0`, range: `-6.0` to `6.0`)
- Optional: `duv_number` (synca only, auto-created by default)
- Optional: `ignore_transition` (default: `true`)

### `duv_number` (synca only)

`duv_number` exposes DUV as a runtime-adjustable `number` entity.

- Available only when `device_type: synca`
- Automatically created for synca lights even when omitted
- Range is fixed to `-6.0` to `6.0`
- Optional: `step` (default: `0.1`)
- Optional: `restore_value` (default: `false`)
- Optional: `initial_value` (default: `ct_duv`)
- Optional: `name`, `id` (auto-generated when omitted)

Behavior:

- Changing DUV while the synca light is ON in color-temperature mode applies immediately.
- If the light is OFF, only the DUV value is updated; the new value is used on the next light operation.

### `target`

- Individual device: e.g. `0x0001`
- Group: e.g. `0x8001` (groups use address space `>= 0x8000`)

## Operational Notes

- Because the Bluetooth LE stack consumes significant CPU/memory resources, use an ESP32 board dedicated to this component.
- It is recommended to store `mesh_name` / `mesh_password` as `!secret`.

## Internal Architecture

- Telink Mesh ESP-IDF session layer: https://github.com/hrko/esp-telink-mesh
- Smart LEDZ protocol layer: https://github.com/hrko/smartledz-protocol
- ESPHome integration layer: `components/smart_ledz/`

## Disclaimer

This is an unofficial project created for personal technical research and to ensure interoperability with open-source home automation systems like Home Assistant. 

- **Unofficial:** This project is NOT affiliated with, endorsed by, or associated with Endo Lighting Corp. Do NOT contact official manufacturer support regarding issues related to this software.
- **Use at Your Own Risk:** This software is provided "AS IS", without warranty of any kind. The author is not responsible for any damage, malfunction, or loss of warranty caused by using this software. Use it entirely at your own risk.
- **Interoperability:** This project aims to provide interoperability through protocol analysis and does not intend to infringe on any intellectual property rights of the manufacturer.
- **Future Breakage:** Firmware updates or changes to the official app may break this software at any time without notice. No ongoing support or maintenance is guaranteed.
- **Trademarks:** "ENDO", "LEDZ", "Smart LEDZ", "SmartLEDZ Fit", "SmartLEDZ Fit Plus", "Tunable LEDZ", and "Synca" are trademarks or registered trademarks of Endo Lighting Corp. All other company names and product names are trademarks or registered trademarks of their respective owners. The use of these trademarks in this project is solely for identification purposes (nominative fair use) to enable interoperability and does not imply any affiliation with, or endorsement by, the trademark holders.
