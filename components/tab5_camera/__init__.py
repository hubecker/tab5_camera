import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import gpio
from esphome.const import (
    CONF_FREQUENCY,
    CONF_NAME,
    CONF_PIN,
    CONF_RESET_PIN,
)

DEPENDENCIES = ["esp32"]
CODEOWNERS = ["@youkorr"]

CONF_EXTERNAL_CLOCK = "external_clock"
CONF_EXTERNAL_CLOCK_PIN = "external_clock_pin"
CONF_EXTERNAL_CLOCK_FREQUENCY = "external_clock_frequency"

tab5_camera_ns = cg.esphome_ns.namespace("tab5_camera")
Tab5Camera = tab5_camera_ns.class_("Tab5Camera", cg.Component)

def validate_frequency(value):
    """Valide la fréquence et la convertit en Hz."""
    if isinstance(value, str):
        if value.endswith("MHz"):
            return int(float(value[:-3]) * 1_000_000)
        elif value.endswith("kHz"):
            return int(float(value[:-3]) * 1_000)
        elif value.endswith("Hz"):
            return int(float(value[:-2]))
    return cv.frequency(value)

# Schéma pour external_clock
EXTERNAL_CLOCK_SCHEMA = cv.Schema({
    cv.Required(CONF_PIN): cv.pin,
    cv.Required(CONF_FREQUENCY): validate_frequency,
})

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(Tab5Camera),
    cv.Optional(CONF_NAME, default="Tab5 Camera"): cv.string_strict,
    cv.Optional(CONF_EXTERNAL_CLOCK): EXTERNAL_CLOCK_SCHEMA,
    cv.Optional(CONF_EXTERNAL_CLOCK_PIN): cv.pin,
    cv.Optional(CONF_EXTERNAL_CLOCK_FREQUENCY): validate_frequency,
    cv.Optional(CONF_RESET_PIN): gpio.gpio_output_pin_schema,
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[cv.GenerateID()])
    await cg.register_component(var, config)
    
    # Configuration du nom
    cg.add(var.set_name(config[CONF_NAME]))
    
    # Configuration external_clock (nouvelle syntaxe)
    if CONF_EXTERNAL_CLOCK in config:
        ext_clock = config[CONF_EXTERNAL_CLOCK]
        pin_num = ext_clock[CONF_PIN]
        frequency = ext_clock[CONF_FREQUENCY]
        
        cg.add(var.set_external_clock_pin(pin_num))
        cg.add(var.set_external_clock_frequency(frequency))
    
    # Configuration legacy (ancienne syntaxe)
    elif CONF_EXTERNAL_CLOCK_PIN in config:
        cg.add(var.set_external_clock_pin(config[CONF_EXTERNAL_CLOCK_PIN]))
    
    if CONF_EXTERNAL_CLOCK_FREQUENCY in config:
        cg.add(var.set_external_clock_frequency(config[CONF_EXTERNAL_CLOCK_FREQUENCY]))
    
    # Configuration du reset pin
    if CONF_RESET_PIN in config:
        reset_pin = await gpio.gpio_pin_expression(config[CONF_RESET_PIN])
        cg.add(var.set_reset_pin(reset_pin))


