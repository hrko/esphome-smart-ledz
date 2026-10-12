"""Check the public turn-on mode schema with ESPHome's actual validators."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "components"))

import esphome.config_validation as cv
from esphome.core import CORE
from smart_ledz.light import CONFIG_SCHEMA


def validate(**overrides):
    CORE.reset()
    return CONFIG_SCHEMA(
        {"name": "Test", "target": 1, "device_type": "synca", **overrides}
    )


assert validate()["turn_on_mode"] == "color_then_brightness"
assert validate(turn_on_mode="legacy")["turn_on_mode"] == "legacy"
try:
    validate(turn_on_mode="invalid")
except cv.Invalid:
    pass
else:
    raise AssertionError("Unknown turn_on_mode was accepted")
print("Turn-on configuration tests passed")
