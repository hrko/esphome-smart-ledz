import esphome.codegen as cg
from esphome.components import light, number
import esphome.config_validation as cv
from esphome.const import (
    CONF_INITIAL_VALUE,
    CONF_NAME,
    CONF_OUTPUT_ID,
    CONF_RESTORE_VALUE,
    CONF_STEP,
    ENTITY_CATEGORY_CONFIG,
)

from .. import CONF_SMART_LEDZ_ID, smart_ledz_ns

DEPENDENCIES = ["smart_ledz"]
AUTO_LOAD = ["number"]

CONF_DEVICE_TYPE = "device_type"
CONF_CT_DUV = "ct_duv"
CONF_DUV_NUMBER = "duv_number"
CONF_IGNORE_TRANSITION = "ignore_transition"
CONF_TARGET = "target"

SmartLedzHub = smart_ledz_ns.class_("SmartLedzHub")
SmartLedzLightOutput = smart_ledz_ns.class_("SmartLedzLightOutput", light.LightOutput)
SmartLedzDuvNumber = smart_ledz_ns.class_("SmartLedzDuvNumber", number.Number, cg.Component)
SmartLedzDeviceType = smart_ledz_ns.enum("SmartLedzDeviceType")

DEVICE_TYPE_OPTIONS = {
    "dimmable": SmartLedzDeviceType.SMART_LEDZ_DEVICE_TYPE_DIMMABLE,
    "tunable": SmartLedzDeviceType.SMART_LEDZ_DEVICE_TYPE_TUNABLE,
    "synca": SmartLedzDeviceType.SMART_LEDZ_DEVICE_TYPE_SYNCA,
}

DUV_NUMBER_SCHEMA = cv.maybe_simple_value(
    number.number_schema(SmartLedzDuvNumber, entity_category=ENTITY_CATEGORY_CONFIG)
    .extend(
        {
            cv.Optional(CONF_INITIAL_VALUE): cv.float_range(min=-6, max=6),
            cv.Optional(CONF_RESTORE_VALUE, default=False): cv.boolean,
            cv.Optional(CONF_STEP, default=0.1): cv.positive_float,
        }
    )
    .extend(cv.COMPONENT_SCHEMA),
    key=CONF_NAME,
)


def _default_duv_number_name(config):
    light_name = config.get(CONF_NAME)
    if isinstance(light_name, str):
        light_name = light_name.strip()
    if light_name:
        return f"{light_name} DUV"
    return "DUV"


def _validate_dynamic_duv(config):
    config = dict(config)
    device_type = config[CONF_DEVICE_TYPE]
    device_type_str = str(device_type).lower()
    is_synca = device_type == "synca" or "smart_ledz_device_type_synca" in device_type_str

    if not is_synca:
        if CONF_DUV_NUMBER in config:
            raise cv.Invalid(
                f"{CONF_DUV_NUMBER} is supported only when {CONF_DEVICE_TYPE} is synca."
            )
        return config

    duv_config = dict(config.get(CONF_DUV_NUMBER, {}))

    if CONF_NAME not in duv_config:
        duv_config[CONF_NAME] = _default_duv_number_name(config)
    if CONF_INITIAL_VALUE not in duv_config:
        duv_config[CONF_INITIAL_VALUE] = config[CONF_CT_DUV]

    config[CONF_DUV_NUMBER] = DUV_NUMBER_SCHEMA(duv_config)
    return config


CONFIG_SCHEMA = cv.All(
    light.BRIGHTNESS_ONLY_LIGHT_SCHEMA.extend(
    {
        cv.GenerateID(CONF_OUTPUT_ID): cv.declare_id(SmartLedzLightOutput),
        cv.GenerateID(CONF_SMART_LEDZ_ID): cv.use_id(SmartLedzHub),
        cv.Required(CONF_TARGET): cv.hex_uint16_t,
        cv.Required(CONF_DEVICE_TYPE): cv.enum(DEVICE_TYPE_OPTIONS, lower=True),
        cv.Optional(CONF_CT_DUV, default=0): cv.float_range(min=-6, max=6),
        cv.Optional(CONF_DUV_NUMBER): DUV_NUMBER_SCHEMA,
        cv.Optional(CONF_IGNORE_TRANSITION, default=True): cv.boolean,
    }
    ),
    _validate_dynamic_duv,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_OUTPUT_ID])
    await light.register_light(var, config)

    parent = await cg.get_variable(config[CONF_SMART_LEDZ_ID])
    cg.add(var.set_parent(parent))
    cg.add(var.set_target(config[CONF_TARGET]))
    cg.add(var.set_device_type(config[CONF_DEVICE_TYPE]))
    cg.add(var.set_ct_duv(config[CONF_CT_DUV]))
    cg.add(var.set_ignore_transition(config[CONF_IGNORE_TRANSITION]))

    if CONF_DUV_NUMBER in config:
        duv_config = config[CONF_DUV_NUMBER]
        duv = await number.new_number(
            duv_config, min_value=-6.0, max_value=6.0, step=duv_config[CONF_STEP]
        )
        await cg.register_component(duv, duv_config)
        cg.add(duv.set_parent(var))
        cg.add(duv.set_initial_value(duv_config[CONF_INITIAL_VALUE]))
        cg.add(duv.set_restore_value(duv_config[CONF_RESTORE_VALUE]))
