# esphome-smart-ledz

## State Sync Notes

- For `synca` lights in color temperature mode, RGB notify updates are converted back to
  estimated CCT and reflected in Home Assistant as color temperature.
- Brightness is updated only when the latest device update includes brightness data
  (online status or dimming response), preventing stale brightness from RGB-only notifies.
