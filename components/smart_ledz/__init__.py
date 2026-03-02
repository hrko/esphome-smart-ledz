import esphome.codegen as cg
from esphome.components import ble_client
import esphome.config_validation as cv
from esphome.const import CONF_ID

CONF_MESH_NAME = "mesh_name"
CONF_MESH_PASSWORD = "mesh_password"
CONF_POLL_INTERVAL = "poll_interval"
CONF_POWER_ON_SETTLE = "power_on_settle"
CONF_SMART_LEDZ_ID = "smart_ledz_id"
CONF_TX_INTERVAL = "tx_interval"
CONF_VENDOR_ID = "vendor_id"

DEPENDENCIES = ["ble_client"]
AUTO_LOAD = ["ble_client"]
MULTI_CONF = True

smart_ledz_ns = cg.esphome_ns.namespace("smart_ledz")
SmartLedzHub = smart_ledz_ns.class_(
    "SmartLedzHub", ble_client.BLEClientNode, cg.Component
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(SmartLedzHub),
            cv.Required(CONF_MESH_NAME): cv.string,
            cv.Required(CONF_MESH_PASSWORD): cv.string,
            cv.Optional(CONF_VENDOR_ID, default=0x0211): cv.hex_uint16_t,
            cv.Optional(CONF_POLL_INTERVAL, default="2s"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_TX_INTERVAL, default="120ms"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_POWER_ON_SETTLE, default="400ms"): cv.positive_time_period_milliseconds,
        }
    )
    .extend(ble_client.BLE_CLIENT_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await ble_client.register_ble_node(var, config)
    cg.add(var.set_mesh_name(config[CONF_MESH_NAME]))
    cg.add(var.set_mesh_password(config[CONF_MESH_PASSWORD]))
    cg.add(var.set_vendor_id(config[CONF_VENDOR_ID]))
    cg.add(var.set_poll_interval_ms(config[CONF_POLL_INTERVAL]))
    cg.add(var.set_tx_interval_ms(config[CONF_TX_INTERVAL]))
    cg.add(var.set_power_on_settle_ms(config[CONF_POWER_ON_SETTLE]))
