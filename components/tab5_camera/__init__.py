import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.pins import gpio_output_pin_schema
from esphome.const import (
    CONF_FREQUENCY,
    CONF_NAME,
    CONF_RESET_PIN,
)

# Dépendances et propriétaires du code
DEPENDENCIES = ["esp32"]
CODEOWNERS = ["@youkorr"]

# Constantes pour la configuration
CONF_EXTERNAL_CLOCK = "external_clock"
CONF_EXTERNAL_CLOCK_PIN = "external_clock_pin"
CONF_EXTERNAL_CLOCK_FREQUENCY = "external_clock_frequency"
CONF_AUTO_START_STREAMING = "auto_start_streaming"
CONF_PIN = "pin"

# Namespace pour le composant
tab5_camera_ns = cg.esphome_ns.namespace("tab5_camera")
Tab5Camera = tab5_camera_ns.class_("Tab5Camera", cg.Component)

# Validation de la fréquence
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

# Validation du numéro de pin GPIO
def validate_pin(value):
    """Valide un numéro de pin GPIO."""
    if isinstance(value, str) and value.startswith("GPIO"):
        return int(value[4:])  # Retire "GPIO" et convertit en int
    return cv.positive_int(value)

# Schéma pour external_clock
EXTERNAL_CLOCK_SCHEMA = cv.Schema({
    cv.Required(CONF_PIN): validate_pin,
    cv.Required(CONF_FREQUENCY): validate_frequency,
})

# Schéma principal de configuration
CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(Tab5Camera),
    cv.Optional(CONF_NAME, default="Tab5 Camera"): cv.string_strict,
    cv.Optional(CONF_EXTERNAL_CLOCK): EXTERNAL_CLOCK_SCHEMA,
    cv.Optional(CONF_EXTERNAL_CLOCK_PIN): validate_pin,
    cv.Optional(CONF_EXTERNAL_CLOCK_FREQUENCY): validate_frequency,
    cv.Optional(CONF_RESET_PIN): gpio_output_pin_schema,
    cv.Optional(CONF_AUTO_START_STREAMING, default=False): cv.boolean,
}).extend(cv.COMPONENT_SCHEMA)

# Génération du code C++
async def to_code(config):
    # Création de la variable principale
    var = cg.new_Pvariable(config[cv.GenerateID()])
    await cg.register_component(var, config)

    # Configuration du nom
    cg.add(var.set_name(config[CONF_NAME]))

    # Configuration par défaut pour l'horloge externe
    external_clock_pin = 36
    external_clock_frequency = 20_000_000

    # Nouvelle syntaxe external_clock
    if CONF_EXTERNAL_CLOCK in config:
        ext_clock = config[CONF_EXTERNAL_CLOCK]
        external_clock_pin = ext_clock[CONF_PIN]
        external_clock_frequency = ext_clock[CONF_FREQUENCY]

    # Ancienne syntaxe
    if CONF_EXTERNAL_CLOCK_PIN in config:
        external_clock_pin = config[CONF_EXTERNAL_CLOCK_PIN]

    if CONF_EXTERNAL_CLOCK_FREQUENCY in config:
        external_clock_frequency = config[CONF_EXTERNAL_CLOCK_FREQUENCY]

    # Applique la configuration de l'horloge externe
    cg.add(var.set_external_clock_pin(external_clock_pin))
    cg.add(var.set_external_clock_frequency(external_clock_frequency))

    # Configuration de la broche de reset
    if CONF_RESET_PIN in config:
        try:
            reset_pin = await gpio_output_pin_schema(config[CONF_RESET_PIN])
            cg.add(var.set_reset_pin(reset_pin))
        except Exception as e:
            raise cv.Invalid(f"Invalid reset pin configuration: {e}")
    else:
        raise cv.Invalid("Reset pin is required but not provided.")

    # Configuration du démarrage automatique du streaming
    cg.add(var.set_auto_start_streaming(config[CONF_AUTO_START_STREAMING]))




