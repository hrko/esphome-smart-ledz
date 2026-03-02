# SmartLEDZ Export YAML SPA

Static SPA to convert SmartLEDZ Fit raw export JSON into ESPHome YAML for the `smart_ledz` external component.

## Files

- `index.html`
- `styles.css`
- `app.js`

## Usage

1. Open `index.html` in a browser.
2. Load SmartLEDZ Fit raw export JSON.
3. Select target devices/groups.
4. Adjust BLE, hub, and per-light settings.
5. Copy generated `ESPHome YAML` and `secrets.yaml`.

## Output Scope

Generated YAML includes exactly these top-level sections:

- `external_components`
- `ble_client`
- `smart_ledz`
- `light`

## Notes

- Input format is raw SmartLEDZ Fit export JSON.
- `mesh_name` and `mesh_password` are emitted as `!secret` references.
- Group target default is `0x8000 | group_low_byte`.
- Extra YAML lines are inserted as-is; verify syntax before use.
