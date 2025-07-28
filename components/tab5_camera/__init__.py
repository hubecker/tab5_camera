import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c
from esphome.const import (
    CONF_ID,
    CONF_NAME,
    CONF_ADDRESS,
)
from esphome import pins

CODEOWNERS = ["@youkorr"]
DEPENDENCIES = ["i2c"]
MULTI_CONF = True

tab5_camera_ns = cg.esphome_ns.namespace("tab5_camera")
Tab5Camera = tab5_camera_ns.class_("Tab5Camera", cg.Component, i2c.I2CDevice)

# Enums SC2356
SC2356Resolution = tab5_camera_ns.enum("SC2356Resolution")
SC2356_RESOLUTIONS = {
    "QVGA": SC2356Resolution.QVGA_320x240,
    "VGA": SC2356Resolution.VGA_640x480,
    "SVGA": SC2356Resolution.SVGA_800x600,
    "HD": SC2356Resolution.HD_1280x720,
    "720P": SC2356Resolution.HD_1280x720,
    "UXGA": SC2356Resolution.UXGA_1600x1200,
    "FHD": SC2356Resolution.FHD_1920x1080,
    "1080P": SC2356Resolution.FHD_1920x1080,
}

SC2356PixelFormat = tab5_camera_ns.enum("SC2356PixelFormat")
SC2356_PIXEL_FORMATS = {
    "RAW10": SC2356PixelFormat.RAW10,
    "YUV422": SC2356PixelFormat.YUV422,
    "RGB565": SC2356PixelFormat.RGB565,
    "JPEG": SC2356PixelFormat.JPEG,
}

# Constantes de configuration
CONF_RESOLUTION = "resolution"
CONF_PIXEL_FORMAT = "pixel_format"
CONF_JPEG_QUALITY = "jpeg_quality"
CONF_FRAMERATE = "framerate"
CONF_EXTERNAL_CLOCK_PIN = "external_clock_pin"
CONF_EXTERNAL_CLOCK_FREQUENCY = "external_clock_frequency"
CONF_RESET_PIN = "reset_pin"
CONF_EXPOSURE_TIME = "exposure_time"
CONF_ANALOG_GAIN = "analog_gain"
CONF_DIGITAL_GAIN = "digital_gain"
CONF_TEST_PATTERN = "test_pattern"

# Validation de résolution
def validate_resolution(value):
    if isinstance(value, str):
        value = cv.one_of(*SC2356_RESOLUTIONS.keys(), upper=True)(value)
        return value
    return cv.invalid("Resolution must be one of: {}".format(", ".join(SC2356_RESOLUTIONS.keys())))

# Validation de format pixel
def validate_pixel_format(value):
    if isinstance(value, str):
        value = cv.one_of(*SC2356_PIXEL_FORMATS.keys(), upper=True)(value)
        return value
    return cv.invalid("Pixel format must be one of: {}".format(", ".join(SC2356_PIXEL_FORMATS.keys())))

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(Tab5Camera),
            cv.Optional(CONF_NAME, default="Tab5 Camera SC2356"): cv.string,
            
            # Configuration matérielle
            cv.Optional(CONF_EXTERNAL_CLOCK_PIN, default=36): cv.int_range(min=0, max=255),
            cv.Optional(CONF_EXTERNAL_CLOCK_FREQUENCY, default=24000000): cv.positive_int,
            cv.Optional(CONF_RESET_PIN): pins.gpio_output_pin_schema,
            
            # Configuration SC2356
            cv.Optional(CONF_RESOLUTION, default="VGA"): validate_resolution,
            cv.Optional(CONF_PIXEL_FORMAT, default="RGB565"): validate_pixel_format,
            cv.Optional(CONF_JPEG_QUALITY, default=10): cv.int_range(min=1, max=63),
            cv.Optional(CONF_FRAMERATE, default=15): cv.int_range(min=5, max=30),
            
            # Paramètres avancés SC2356
            cv.Optional(CONF_EXPOSURE_TIME, default=10000): cv.positive_int,  # µs
            cv.Optional(CONF_ANALOG_GAIN, default=128): cv.int_range(min=64, max=512),
            cv.Optional(CONF_DIGITAL_GAIN, default=128): cv.int_range(min=64, max=512),
            cv.Optional(CONF_TEST_PATTERN, default=False): cv.boolean,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(i2c.i2c_device_schema(0x43))  # SC2356 adresse fixe
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
    
    # Configuration de base
    cg.add(var.set_name(config[CONF_NAME]))
    cg.add(var.set_external_clock_pin(config[CONF_EXTERNAL_CLOCK_PIN]))
    cg.add(var.set_external_clock_frequency(config[CONF_EXTERNAL_CLOCK_FREQUENCY]))
    
    # Configuration SC2356 spécifique
    cg.add(var.set_resolution(SC2356_RESOLUTIONS[config[CONF_RESOLUTION]]))
    cg.add(var.set_pixel_format(SC2356_PIXEL_FORMATS[config[CONF_PIXEL_FORMAT]]))
    cg.add(var.set_jpeg_quality(config[CONF_JPEG_QUALITY]))
    cg.add(var.set_framerate(config[CONF_FRAMERATE]))
    
    # Paramètres avancés
    cg.add(var.set_exposure_time(config[CONF_EXPOSURE_TIME]))
    cg.add(var.set_analog_gain(config[CONF_ANALOG_GAIN]))
    cg.add(var.set_digital_gain(config[CONF_DIGITAL_GAIN]))
    cg.add(var.set_test_pattern(config[CONF_TEST_PATTERN]))
    
    # Pin de reset (optionnel)
    if CONF_RESET_PIN in config:
        reset_pin = await cg.gpio_pin_expression(config[CONF_RESET_PIN])
        cg.add(var.set_reset_pin(reset_pin))




