"""bme688_scan - BME688 heater-profile gas scan without Bosch's BSEC library (2026-11-17, NU-Trash-1 step 2).

The chip runs in parallel mode through a 10-step heater profile (Bosch's standard HP-354 by default). One resistance per
step = a 10-number fingerprint of the air. Uses only Bosch's open BME68x sensor API (BSD-3), so there is no BSEC licence
limit, and no IAQ output either. Publishes temperature / humidity / pressure, the gas resistance of the last (320 C) step,
and a text sensor with the ten step resistances in kilo-ohms.
"""
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import core
from esphome.components import i2c, sensor, text_sensor
from esphome.const import (
    CONF_GAS_RESISTANCE,
    CONF_HUMIDITY,
    CONF_ID,
    CONF_PRESSURE,
    CONF_TEMPERATURE,
    DEVICE_CLASS_ATMOSPHERIC_PRESSURE,
    DEVICE_CLASS_HUMIDITY,
    DEVICE_CLASS_TEMPERATURE,
    ICON_GAS_CYLINDER,
    ICON_THERMOMETER,
    ICON_WATER_PERCENT,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_HECTOPASCAL,
    UNIT_OHM,
    UNIT_PERCENT,
)

CODEOWNERS = []
DEPENDENCIES = ["i2c", "esp32"]
AUTO_LOAD = ["sensor", "text_sensor"]

CONF_SCAN = "scan"
CONF_PUBLISH_INTERVAL = "publish_interval"
CONF_SCAN_INTERVAL = "scan_interval"
CONF_HEATER_TEMPERATURES = "heater_temperatures"
CONF_HEATER_STEPS = "heater_steps"
CONF_STEP_BASE = "step_base"

BME68X_LIBRARY_VERSION = "v1.3.40408"   # same pin as esphome's bme68x_bsec2

bme688_scan_ns = cg.esphome_ns.namespace("bme688_scan")
BME688ScanComponent = bme688_scan_ns.class_("BME688ScanComponent", cg.Component, i2c.I2CDevice)


def _same_len(config):
    if len(config[CONF_HEATER_TEMPERATURES]) != len(config[CONF_HEATER_STEPS]):
        raise cv.Invalid("heater_temperatures and heater_steps need the same number of entries")
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(BME688ScanComponent),
            # Bosch heater profile HP-354: temperature (C) per step, and how many time bases each step is held
            cv.Optional(CONF_HEATER_TEMPERATURES, default=[320, 100, 100, 100, 200, 200, 200, 320, 320, 320]): cv.All(
                cv.ensure_list(cv.int_range(min=50, max=400)), cv.Length(min=2, max=10)
            ),
            cv.Optional(CONF_HEATER_STEPS, default=[5, 2, 10, 30, 5, 5, 5, 5, 5, 5]): cv.All(
                cv.ensure_list(cv.int_range(min=1, max=60)), cv.Length(min=2, max=10)
            ),
            cv.Optional(CONF_STEP_BASE, default="140ms"): cv.All(
                cv.positive_time_period_milliseconds, cv.Range(min=core.TimePeriod(milliseconds=60), max=core.TimePeriod(milliseconds=1000))
            ),
            cv.Optional(CONF_PUBLISH_INTERVAL, default="60s"): cv.positive_time_period_milliseconds,
            # start of one scan cycle to the start of the next; the heater rests in between (less self-heating).
            # 0s = scan non-stop.
            cv.Optional(CONF_SCAN_INTERVAL, default="30s"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS, icon=ICON_THERMOMETER, accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE, state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_HUMIDITY): sensor.sensor_schema(
                unit_of_measurement=UNIT_PERCENT, icon=ICON_WATER_PERCENT, accuracy_decimals=1,
                device_class=DEVICE_CLASS_HUMIDITY, state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_PRESSURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_HECTOPASCAL, accuracy_decimals=1,
                device_class=DEVICE_CLASS_ATMOSPHERIC_PRESSURE, state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_GAS_RESISTANCE): sensor.sensor_schema(
                unit_of_measurement=UNIT_OHM, icon=ICON_GAS_CYLINDER, accuracy_decimals=0,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_SCAN): text_sensor.text_sensor_schema(icon="mdi:fingerprint"),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(i2c.i2c_device_schema(0x77)),
    _same_len,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    from esphome.components.esp32 import add_idf_component

    add_idf_component(
        name="boschsensortec/Bosch-BME68x-Library",
        repo="https://github.com/esphome-libs/Bosch-BME68x-Library",
        ref=BME68X_LIBRARY_VERSION,
    )

    for i, (t, n) in enumerate(zip(config[CONF_HEATER_TEMPERATURES], config[CONF_HEATER_STEPS])):
        cg.add(var.set_step(i, t, n))
    cg.add(var.set_profile_len(len(config[CONF_HEATER_TEMPERATURES])))
    cg.add(var.set_step_base_ms(config[CONF_STEP_BASE].total_milliseconds))
    cg.add(var.set_publish_interval_ms(config[CONF_PUBLISH_INTERVAL].total_milliseconds))
    cg.add(var.set_scan_interval_ms(config[CONF_SCAN_INTERVAL].total_milliseconds))

    for key, setter in (
        (CONF_TEMPERATURE, "set_temperature_sensor"),
        (CONF_HUMIDITY, "set_humidity_sensor"),
        (CONF_PRESSURE, "set_pressure_sensor"),
        (CONF_GAS_RESISTANCE, "set_gas_resistance_sensor"),
    ):
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(getattr(var, setter)(sens))
    if CONF_SCAN in config:
        ts = await text_sensor.new_text_sensor(config[CONF_SCAN])
        cg.add(var.set_scan_text_sensor(ts))
