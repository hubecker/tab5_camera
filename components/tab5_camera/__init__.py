import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_NAME, CONF_PIN
from esphome import pins

DEPENDENCIES = ["esp32"]
CODEOWNERS = ["@youkorr"]

tab5_camera_ns = cg.esphome_ns.namespace("tab5_camera")
Tab5Camera = tab5_camera_ns.class_("Tab5Camera", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(Tab5Camera),
        cv.Required(CONF_NAME): cv.string,
        cv.Optional("external_clock_pin", default=36): cv.int_,
        cv.Optional("external_clock_frequency", default=20000000): cv.int_,
        cv.Optional("reset_pin"): pins.gpio_output_pin_schema,
    }
).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    
    cg.add(var.set_name(config[CONF_NAME]))
    cg.add(var.set_external_clock_pin(config["external_clock_pin"]))
    cg.add(var.set_external_clock_frequency(config["external_clock_frequency"]))
    
    if "reset_pin" in config:
        pin = await cg.gpio_pin_expression(config["reset_pin"])
        cg.add(var.set_reset_pin(pin))

