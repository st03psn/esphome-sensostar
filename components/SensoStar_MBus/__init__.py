import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, output # output for LED output
from esphome.const import CONF_ID

DEPENDENCIES = ["uart"]

CONF_SENSOSTAR_ID = "sensostar_id"
CONF_DATA_LED = "data_led" # data_led output
CONF_TEMPERATURE_ROUNDING = "temperature_rounding"  # how the meter reduces flow/return temperature to whole degrees
CONF_PROBE = "probe"  # diagnostic: send probe requests and log decoded answers

sensostar = cg.esphome_ns.namespace("sensostar")
SensoStarComponent = sensostar.class_(
    "SensoStarComponent", cg.PollingComponent, uart.UARTDevice
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(SensoStarComponent),
            # make data_led available for .yaml file
            cv.Optional(CONF_DATA_LED): cv.use_id(output.BinaryOutput),
            cv.Optional(CONF_PROBE, default=False): cv.boolean,
            cv.Optional(CONF_TEMPERATURE_ROUNDING, default="round"): cv.one_of("round", "truncate", lower=True),
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.polling_component_schema("30s"))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    if config[CONF_TEMPERATURE_ROUNDING] == "truncate":
        cg.add(var.set_temperature_truncated(True))
    if config[CONF_PROBE]:
        cg.add(var.set_probe(True))

    # data_led
    if data_led_id := config.get(CONF_DATA_LED):
        data_led = await cg.get_variable(data_led_id)
        cg.add(var.set_data_led(data_led)) # Calls the set_data_led method you define in .h/.cpp