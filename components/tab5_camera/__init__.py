import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c
from esphome.const import (
    CONF_ID,
    CONF_NAME,
    CONF_ADDRESS,
    CONF_FREQUENCY,
)
from esphome import pins, automation
from esphome.core import CORE

CODEOWNERS = ["@youkorr"]
DEPENDENCIES = ["i2c"]
MULTI_CONF = True

tab5_camera_ns = cg.esphome_ns.namespace("tab5_camera")
Tab5Camera = tab5_camera_ns.class_("Tab5Camera", cg.Component, i2c.I2CDevice)

# Nouvelles constantes
CONF_RESOLUTION = "resolution"
CONF_PIXEL_FORMAT = "pixel_format"
CONF_JPEG_QUALITY = "jpeg_quality"
CONF_FRAMERATE = "framerate"
CONF_EXTERNAL_CLOCK_PIN = "external_clock_pin"
CONF_RESET_PIN = "reset_pin"
CONF_SENSOR_ADDRESS = "sensor_address"
CONF_ON_FRAME = "on_frame"

# Résolutions supportées
CAMERA_RESOLUTIONS = {
    "1080P": (1920, 1080),
    "720P": (1280, 720),
    "VGA": (640, 480),
    "QVGA": (320, 240),
}

# Formats de pixel supportés
PIXEL_FORMATS = {
    "RAW8": "RAW8",
    "RAW10": "RAW10",
    "YUV422": "YUV422",
    "RGB565": "RGB565",
    "JPEG": "JPEG",
}

def validate_resolution(value):
    if isinstance(value, str):
        value = cv.one_of(*CAMERA_RESOLUTIONS.keys(), upper=True)(value)
        return value
    return cv.invalid("Resolution must be one of: {}".format(", ".join(CAMERA_RESOLUTIONS.keys())))

# Actions et triggers pour les frames
OnFrameTrigger = tab5_camera_ns.class_(
    "OnFrameTrigger", automation.Trigger.template(cg.uint8.operator("ptr"), cg.size_t)
)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(Tab5Camera),
            cv.Optional(CONF_NAME, default="Tab5 Camera"): cv.string,
            cv.Optional(CONF_EXTERNAL_CLOCK_PIN, default=0): cv.int_range(min=0, max=255),
            cv.Optional(CONF_FREQUENCY, default=24000000): cv.positive_int,
            cv.Optional(CONF_RESET_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_SENSOR_ADDRESS, default=0x36): cv.i2c_address,
            # Nouveaux paramètres
            cv.Optional(CONF_RESOLUTION, default="VGA"): validate_resolution,
            cv.Optional(CONF_PIXEL_FORMAT, default="RGB565"): cv.one_of(*PIXEL_FORMATS.keys(), upper=True),
            cv.Optional(CONF_JPEG_QUALITY, default=10): cv.int_range(min=1, max=63),
            cv.Optional(CONF_FRAMERATE, default=15): cv.int_range(min=1, max=60),
            # Callback pour les frames reçues
            cv.Optional(CONF_ON_FRAME): automation.validate_automation(
                {
                    cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(OnFrameTrigger),
                }
            ),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(i2c.i2c_device_schema(0x36))
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
    
    # Configuration de base
    cg.add(var.set_name(config[CONF_NAME]))
    cg.add(var.set_external_clock_pin(config[CONF_EXTERNAL_CLOCK_PIN]))
    cg.add(var.set_external_clock_frequency(config[CONF_FREQUENCY]))
    cg.add(var.set_sensor_address(config[CONF_SENSOR_ADDRESS]))
    
    # Nouveaux paramètres
    # Résolution
    resolution_str = config[CONF_RESOLUTION]
    width, height = CAMERA_RESOLUTIONS[resolution_str]
    cg.add(var.set_resolution(width, height))
    
    # Format pixel
    cg.add(var.set_pixel_format(config[CONF_PIXEL_FORMAT]))
    
    # Qualité JPEG
    cg.add(var.set_jpeg_quality(config[CONF_JPEG_QUALITY]))
    
    # Framerate
    cg.add(var.set_framerate(config[CONF_FRAMERATE]))
    
    # Pin de reset (optionnel)
    if CONF_RESET_PIN in config:
        reset_pin = await cg.gpio_pin_expression(config[CONF_RESET_PIN])
        cg.add(var.set_reset_pin(reset_pin))

    # Configuration des callbacks on_frame
    if CONF_ON_FRAME in config:
        for conf in config[CONF_ON_FRAME]:
            trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID])
            cg.add(var.add_on_frame_trigger(trigger))
            await automation.build_automation(
                trigger, [(cg.uint8.operator("ptr"), "data"), (cg.size_t, "data_len")], conf
            )

# Actions pour contrôler la caméra
@automation.register_action(
    "tab5_camera.take_snapshot",
    cg.Pvariable, 
    cv.Schema({cv.GenerateID(): cv.use_id(Tab5Camera)})
)
async def tab5_camera_take_snapshot_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var

@automation.register_action(
    "tab5_camera.start_streaming",
    cg.Pvariable,
    cv.Schema({cv.GenerateID(): cv.use_id(Tab5Camera)})
)
async def tab5_camera_start_streaming_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var

@automation.register_action(
    "tab5_camera.stop_streaming", 
    cg.Pvariable,
    cv.Schema({cv.GenerateID(): cv.use_id(Tab5Camera)})
)
async def tab5_camera_stop_streaming_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var



