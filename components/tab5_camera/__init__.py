import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.const import (
    CONF_ID,
    CONF_PIN,
    CONF_FREQUENCY,
    PLATFORM_ESP32,
)
from esphome.core import CORE

DEPENDENCIES = ["esp32"]
CODEOWNERS = ["@youkorr"]  # Remplacez par votre nom d'utilisateur GitHub

# Vérification ESP32-P4
def validate_esp32_p4(config):
    if CORE.is_esp32:
        variant = CORE.data.get("esp32", {}).get("variant")
        if variant != "esp32p4":
            raise cv.Invalid("Tab5 Camera component requires ESP32-P4 variant")
    return config

tab5_camera_ns = cg.esphome_ns.namespace("tab5_camera")
Tab5Camera = tab5_camera_ns.class_("Tab5Camera", cg.Component)

# Configuration
CONF_EXTERNAL_CLOCK_PIN = "external_clock_pin"
CONF_EXTERNAL_CLOCK_FREQUENCY = "external_clock_frequency"
CONF_RESET_PIN = "reset_pin"
CONF_AUTO_START_STREAMING = "auto_start_streaming"

CONFIG_SCHEMA = cv.All(
    cv.Schema({
        cv.GenerateID(): cv.declare_id(Tab5Camera),
        cv.Optional(CONF_EXTERNAL_CLOCK_PIN, default=0): cv.int_range(min=0, max=48),
        cv.Optional(CONF_EXTERNAL_CLOCK_FREQUENCY, default=20000000): cv.int_range(min=1000000, max=40000000),
        cv.Optional(CONF_RESET_PIN): pins.gpio_output_pin_schema,
        cv.Optional(CONF_AUTO_START_STREAMING, default=False): cv.boolean,
    }).extend(cv.COMPONENT_SCHEMA),
    validate_esp32_p4,
    cv.only_on(PLATFORM_ESP32),
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    
    # Configuration des pins
    cg.add(var.set_external_clock_pin(config[CONF_EXTERNAL_CLOCK_PIN]))
    cg.add(var.set_external_clock_frequency(config[CONF_EXTERNAL_CLOCK_FREQUENCY]))
    
    if CONF_RESET_PIN in config:
        reset_pin = await cg.gpio_pin_expression(config[CONF_RESET_PIN])
        cg.add(var.set_reset_pin(reset_pin))
    
    # Définitions de compilation conditionnelles
    cg.add_define("USE_ESP32_P4_CAMERA")
    cg.add_define("HAS_ESP32_P4_CAMERA")
    
    # Auto-start streaming si configuré
    if config.get(CONF_AUTO_START_STREAMING, False):
        cg.add_library("AsyncTCP", None)  # Pour le streaming
        
        # Code d'initialisation du streaming
        cg.add(cg.RawExpression(f"""
        App.register_component_finalizer([](Component *comp) {{
            auto *camera = static_cast<tab5_camera::Tab5Camera*>(comp);
            if (camera != nullptr) {{
                // Démarrer le streaming après l'initialisation
                App.scheduler.set_timeout(camera, "start_streaming", 1000, [camera]() {{
                    camera->start_streaming();
                }});
            }}
        }});
        """))

# Actions disponibles
TAB5_CAMERA_TAKE_SNAPSHOT_ACTION_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.use_id(Tab5Camera),
})

TAB5_CAMERA_START_STREAMING_ACTION_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.use_id(Tab5Camera),
})

TAB5_CAMERA_STOP_STREAMING_ACTION_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.use_id(Tab5Camera),
})

@automation.register_action(
    "tab5_camera.take_snapshot",
    cg.Pvariable,
    TAB5_CAMERA_TAKE_SNAPSHOT_ACTION_SCHEMA,
)
async def tab5_camera_take_snapshot_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var

@automation.register_action(
    "tab5_camera.start_streaming",
    cg.Pvariable,
    TAB5_CAMERA_START_STREAMING_ACTION_SCHEMA,
)
async def tab5_camera_start_streaming_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var

@automation.register_action(
    "tab5_camera.stop_streaming",
    cg.Pvariable,
    TAB5_CAMERA_STOP_STREAMING_ACTION_SCHEMA,
)
async def tab5_camera_stop_streaming_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var



