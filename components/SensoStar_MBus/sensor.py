import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ENERGY,
    CONF_VOLUME,
    
    CONF_POWER,
    CONF_FLOW,
    
    DEVICE_CLASS_ENERGY,
    DEVICE_CLASS_VOLUME,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_VOLUME_FLOW_RATE,
    DEVICE_CLASS_TEMPERATURE,
    
    UNIT_KILOWATT_HOURS,
    UNIT_CUBIC_METER,
    UNIT_WATT,
    UNIT_CUBIC_METER_PER_HOUR,
    UNIT_CELSIUS,
    
    ICON_POWER,
    ICON_THERMOMETER,
    
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,

)
from . import SensoStarComponent, CONF_SENSOSTAR_ID

CONF_TEMPERATURE_FLOW = "temperature_flow"
CONF_TEMPERATURE_RETURN = "temperature_return"
CONF_TEMPERATURE_DIFF = "temperature_diff"
CONF_CALCULATED_POWER = "calculated_power"
CONF_TEMPERATURE_FLOW_ESTIMATED = "temperature_flow_estimated"
CONF_TEMPERATURE_RETURN_ESTIMATED = "temperature_return_estimated"
CONF_MAX_POWER = "max_power"
CONF_MAX_FLOW = "max_flow"
CONF_MAX_TEMPERATURE_FLOW = "max_temperature_flow"
CONF_MAX_TEMPERATURE_RETURN = "max_temperature_return"
CONF_MAX_TEMPERATURE_DIFF = "max_temperature_diff"
CONF_DUE_DATE_ENERGY = "due_date_energy"
CONF_DUE_DATE_VOLUME = "due_date_volume"
CONF_OPERATING_DAYS = "operating_days"

TYPES = [
    CONF_ENERGY,
    CONF_VOLUME,
    CONF_POWER,
    CONF_FLOW,
    CONF_TEMPERATURE_FLOW,
    CONF_TEMPERATURE_RETURN,
    CONF_TEMPERATURE_DIFF,
    CONF_CALCULATED_POWER,
    CONF_TEMPERATURE_FLOW_ESTIMATED,
    CONF_TEMPERATURE_RETURN_ESTIMATED,
    CONF_MAX_POWER,
    CONF_MAX_FLOW,
    CONF_MAX_TEMPERATURE_FLOW,
    CONF_MAX_TEMPERATURE_RETURN,
    CONF_MAX_TEMPERATURE_DIFF,
    CONF_DUE_DATE_ENERGY,
    CONF_DUE_DATE_VOLUME,
    CONF_OPERATING_DAYS,
]


def _temp(decimals):
    return sensor.sensor_schema(
        unit_of_measurement=UNIT_CELSIUS,
        icon=ICON_THERMOMETER,
        accuracy_decimals=decimals,
        device_class=DEVICE_CLASS_TEMPERATURE,
        state_class=STATE_CLASS_MEASUREMENT,
    )

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(CONF_SENSOSTAR_ID): cv.use_id(SensoStarComponent),
            cv.Optional(CONF_ENERGY): sensor.sensor_schema(
                unit_of_measurement=UNIT_KILOWATT_HOURS,
                icon=ICON_POWER,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_ENERGY,
                state_class=STATE_CLASS_TOTAL_INCREASING,
            ),
            cv.Optional(CONF_VOLUME): sensor.sensor_schema(
                unit_of_measurement=UNIT_CUBIC_METER,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_VOLUME,
                state_class=STATE_CLASS_TOTAL_INCREASING,
            ),
            cv.Optional(CONF_POWER): sensor.sensor_schema(
                unit_of_measurement=UNIT_WATT,
                icon=ICON_POWER,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_POWER,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_FLOW): sensor.sensor_schema(
                unit_of_measurement=UNIT_CUBIC_METER_PER_HOUR,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_VOLUME_FLOW_RATE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_TEMPERATURE_FLOW): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                icon=ICON_THERMOMETER,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_TEMPERATURE_RETURN): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                icon=ICON_THERMOMETER,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_TEMPERATURE_DIFF): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                icon=ICON_THERMOMETER,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_CALCULATED_POWER): sensor.sensor_schema(
                unit_of_measurement=UNIT_WATT,
                icon=ICON_POWER,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_POWER,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_TEMPERATURE_FLOW_ESTIMATED): _temp(2),
            cv.Optional(CONF_TEMPERATURE_RETURN_ESTIMATED): _temp(2),
            cv.Optional(CONF_MAX_TEMPERATURE_FLOW): _temp(2),
            cv.Optional(CONF_MAX_TEMPERATURE_RETURN): _temp(2),
            cv.Optional(CONF_MAX_TEMPERATURE_DIFF): _temp(2),
            cv.Optional(CONF_MAX_POWER): sensor.sensor_schema(
                unit_of_measurement=UNIT_WATT,
                icon=ICON_POWER,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_POWER,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_MAX_FLOW): sensor.sensor_schema(
                unit_of_measurement=UNIT_CUBIC_METER_PER_HOUR,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_VOLUME_FLOW_RATE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_DUE_DATE_ENERGY): sensor.sensor_schema(
                unit_of_measurement=UNIT_KILOWATT_HOURS,
                icon=ICON_POWER,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_ENERGY,
            ),
            cv.Optional(CONF_DUE_DATE_VOLUME): sensor.sensor_schema(
                unit_of_measurement=UNIT_CUBIC_METER,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_VOLUME,
            ),
            cv.Optional(CONF_OPERATING_DAYS): sensor.sensor_schema(
                unit_of_measurement="d",
                icon="mdi:timer-outline",
                accuracy_decimals=0,
                state_class=STATE_CLASS_TOTAL_INCREASING,
            ),
        }
    ).extend(cv.COMPONENT_SCHEMA)
)

async def setup_conf(config, key, hub):
    if sensor_config := config.get(key):
        sens = await sensor.new_sensor(sensor_config)
        cg.add(getattr(hub, f"set_{key}_sensor")(sens))


async def to_code(config):
    hub = await cg.get_variable(config[CONF_SENSOSTAR_ID])
    for key in TYPES:
        await setup_conf(config, key, hub)
