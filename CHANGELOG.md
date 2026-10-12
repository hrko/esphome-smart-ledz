# Changelog

## 0.2.2 - 2026-10-12

- Synchronize SYNCA color temperature across the full 1800–12000 K range.
- Preserve the last confirmed power and brightness when a light reports offline.
  Per-light availability in Home Assistant is unchanged.
- Reconnect after exhausting the mesh command sequence instead of reusing it.
- Apply a 10-second deadline to each mesh session setup stage and recover from
  setup failures before accepting commands.
- Update the pinned transport library to validate pairing responses, verify
  notification integrity, and handle encryption failures.
- Extend host regression tests for light state synchronization, authenticated
  communication, command sequence boundaries, and session setup recovery.

Host regression tests, static analysis, and ESP32-S3 firmware builds pass.
These changes have not been validated on physical lighting hardware.

## 0.2.1 - 2026-10-12

- Fix power-on hold deadlines blocking the transmit queue after approximately
  24.9 days of uptime. Expired holds are removed and deadlines remain valid across
  the 32-bit millisecond clock rollover.
- Detect missing BLE control-write completion events after 10 seconds, clear the
  session and queued commands, and request a disconnect. With `auto_connect: true`,
  ESPHome can reconnect and pair again. Old queued commands are not replayed.
- Pin the timeout-capable `esp-telink-mesh` revision and the latest
  `smartledz-protocol` revision.
- Update development dependencies to ESPHome 2026.9.1, Python 3.15.0, uv 0.13.0,
  Ruff 0.17.0, and ty 0.0.86, and update GitHub Actions dependencies. Action
  references are pinned to full commit SHAs with verified version comments using pinact.
- Enable `mise.lock` for Python/uv on Linux x64 and Windows x64, and add `uv.lock`
  for Python packages (including source distributions needed by ESPHome).
- Add host regression tests and ESP32-S3 firmware compilation to CI.

The host tests simulate long uptime and missing completion events. Long-duration
operation and reconnect behavior on physical lighting hardware still require
device validation.
