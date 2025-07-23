import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    CONF_NAME,
    CONF_FREQUENCY,
)

DEPENDENCIES = ["esp32"]
CODEOWNERS = ["@youkorr"]

# Champs personnalisés
CONF_EXTERNAL_CLOCK_PIN = "external_clock_pin"
CONF_EXTERNAL_CLOCK_FREQUENCY = "external_clock_frequency"
CONF_AUTO_START_STREAMING = "auto_start_streaming"

tab5_camera_ns = cg.esphome_ns.namespace("tab5_camera")
Tab5Camera = tab5_camera_ns.class_("Tab5Camera", cg.Component)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(Tab5Camera),
    cv.Optional(CONF_NAME, default="Tab5 Camera"): cv.string_strict,
    cv.Optional(CONF_EXTERNAL_CLOCK_PIN, default=36): cv.positive_int,
    cv.Optional(CONF_EXTERNAL_CLOCK_FREQUENCY, default=20_000_000): cv.frequency,
    cv.Optional(CONF_AUTO_START_STREAMING, default=False): cv.boolean,
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    # Nom personnalisé
    cg.add(var.set_name(config[CONF_NAME]))

    # Configuration de l’horloge externe
    cg.add(var.set_external_clock_pin(config[CONF_EXTERNAL_CLOCK_PIN]))
    cg.add(var.set_external_clock_frequency(config[CONF_EXTERNAL_CLOCK_FREQUENCY]))

    # Démarrage auto du flux
    cg.add(var.set_auto_start_streaming(config[CONF_AUTO_START_STREAMING]))






