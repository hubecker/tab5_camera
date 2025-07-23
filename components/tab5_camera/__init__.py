import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import gpio, i2c
from esphome.const import CONF_ID, CONF_NAME, CONF_PIN, CONF_FREQUENCY, CONF_ADDRESS
from esphome import pins

DEPENDENCIES = ['esp32', 'i2c']

tab5_camera_ns = cg.esphome_ns.namespace('tab5_camera')
Tab5Camera = tab5_camera_ns.class_('Tab5Camera', cg.Component)

CONF_EXTERNAL_CLOCK = "external_clock"
CONF_RESET_PIN = "reset_pin"
CONF_I2C_ID = "i2c_id"
CONF_SENSOR_ADDRESS = "sensor_address"

# Schéma pour external_clock
EXTERNAL_CLOCK_SCHEMA = cv.Schema({
    cv.Required(CONF_PIN): pins.gpio_output_pin_schema,
    cv.Optional(CONF_FREQUENCY, default="20MHz"): cv.frequency,
})

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(Tab5Camera),
    cv.Optional(CONF_NAME, default="Tab5 Camera"): cv.string,
    cv.Optional(CONF_EXTERNAL_CLOCK): EXTERNAL_CLOCK_SCHEMA,
    cv.Optional(CONF_RESET_PIN): pins.gpio_output_pin_schema,
    cv.Optional(CONF_I2C_ID): cv.use_id(i2c.I2CComponent),
    cv.Optional(CONF_SENSOR_ADDRESS, default=0x24): cv.i2c_address,
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    
    # Configuration du nom
    cg.add(var.set_name(config[CONF_NAME]))
    
    # Configuration I2C
    if CONF_I2C_ID in config:
        i2c_component = await cg.get_variable(config[CONF_I2C_ID])
        cg.add(var.set_i2c_parent(i2c_component))
    
    # Configuration de l'adresse du capteur
    cg.add(var.set_sensor_address(config[CONF_SENSOR_ADDRESS]))
    
    # Configuration de l'horloge externe
    if CONF_EXTERNAL_CLOCK in config:
        clock_config = config[CONF_EXTERNAL_CLOCK]
        
        # Pin de l'horloge
        pin = await gpio.gpio_pin_expression(clock_config[CONF_PIN])
        cg.add(var.set_external_clock_pin(pin))
        
        # Fréquence de l'horloge (convertir en Hz)
        freq = clock_config[CONF_FREQUENCY]
        cg.add(var.set_external_clock_frequency(int(freq)))
    
    # Configuration du pin de reset
    if CONF_RESET_PIN in config:
        reset_pin = await gpio.gpio_pin_expression(config[CONF_RESET_PIN])
        cg.add(var.set_reset_pin(reset_pin))
        
    # Ajout des flags de compilation nécessaires
    cg.add_build_flag("-DHAS_ESP32_P4_CAMERA=1")
    cg.add_build_flag("-DCONFIG_IDF_TARGET_ESP32P4=1")




