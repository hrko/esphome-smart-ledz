import esphome.codegen as cg
from esphome.components import light
import esphome.config_validation as cv
from esphome.const import CONF_OUTPUT_ID

from .. import CONF_SMART_LEDZ_ID, smart_ledz_ns

DEPENDENCIES = ["smart_ledz"]

CONF_DEVICE_TYPE = "device_type"
CONF_CT_DUV = "ct_duv"
CONF_TARGET = "target"

SmartLedzHub = smart_ledz_ns.class_("SmartLedzHub")
SmartLedzLightOutput = smart_ledz_ns.class_("SmartLedzLightOutput", light.LightOutput)
SmartLedzDeviceType = smart_ledz_ns.enum("SmartLedzDeviceType")

DEVICE_TYPE_OPTIONS = {
    "dimmable": SmartLedzDeviceType.SMART_LEDZ_DEVICE_TYPE_DIMMABLE,
    "tunable": SmartLedzDeviceType.SMART_LEDZ_DEVICE_TYPE_TUNABLE,
    "synca": SmartLedzDeviceType.SMART_LEDZ_DEVICE_TYPE_SYNCA,
}


def validate_ct_duv(value):
    value = cv.int_(value)
    if value not in (-6, -3, 0, 3, 6):
        raise cv.Invalid("ct_duv must be one of: -6, -3, 0, 3, 6")
    return value

CONFIG_SCHEMA = light.BRIGHTNESS_ONLY_LIGHT_SCHEMA.extend(
    {
        cv.GenerateID(CONF_OUTPUT_ID): cv.declare_id(SmartLedzLightOutput),
        cv.GenerateID(CONF_SMART_LEDZ_ID): cv.use_id(SmartLedzHub),
        cv.Required(CONF_TARGET): cv.hex_uint16_t,
        cv.Required(CONF_DEVICE_TYPE): cv.enum(DEVICE_TYPE_OPTIONS, lower=True),
        cv.Optional(CONF_CT_DUV, default=0): validate_ct_duv,
    }
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_OUTPUT_ID])
    await light.register_light(var, config)

    parent = await cg.get_variable(config[CONF_SMART_LEDZ_ID])
    cg.add(var.set_parent(parent))
    cg.add(var.set_target(config[CONF_TARGET]))
    cg.add(var.set_device_type(config[CONF_DEVICE_TYPE]))
    cg.add(var.set_ct_duv(config[CONF_CT_DUV]))
