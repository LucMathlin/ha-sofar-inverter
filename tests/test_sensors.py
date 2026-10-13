"""ME3000 sensor decoding and scaling against known raw register values."""

from __future__ import annotations

import pytest

from homeassistant.core import HomeAssistant

from .conftest import setup_entry
from .fake_sofar import FakeSofarGateway, ascii_registers, u16

PREFIX = "sensor.sofar_inverter_"

# (entity suffix, raw register values the fake holds, expected HA state)
# Raw values are the ones set in fake_sofar.default_registers().
EXPECTED = [
    ("running_state", "0x0200=2", "Charging"),
    ("grid_voltage", "0x0206=2405 x0.1", "240.5"),
    ("grid_current", "0x0207=512 x0.01", "5.12"),
    ("grid_frequency", "0x020C=5002 x0.01", "50.02"),
    ("grid_power", "0x0212=0xFFCE (-50) x10, signed", "-500.0"),
    ("battery_power", "0x020D=150 x10, signed", "1500.0"),
    ("battery_voltage", "0x020E=5210 x0.01", "52.1"),
    ("battery_current", "0x020F=2880 x0.01, signed", "28.8"),
    ("inverter_power", "0x0214=0xFF6A (-150) x10, signed", "-1500.0"),
    ("battery_soc", "0x0210=60", "60.0"),
    ("battery_soh", "0x0237=98", "98.0"),
    ("battery_temperature", "0x0211=21, signed", "21.0"),
    ("battery_cycles", "0x022C=412", "412.0"),
    ("consumption", "0x0213=80 x10, signed", "800.0"),
    ("solar_pv_power", "0x0215=230 x10", "2300.0"),
    ("today_generation", "0x0218=1234 x0.01", "12.34"),
    ("today_exported", "0x0219=321 x0.01", "3.21"),
    ("today_purchased", "0x021A=456 x0.01", "4.56"),
    ("today_consumption", "0x021B=789 x0.01", "7.89"),
    ("today_charged", "0x0224=500 x0.01, fw>=V120", "5.0"),
    ("today_discharged", "0x0225=300 x0.01, fw>=V120", "3.0"),
    ("total_generation", "0x021C:0x021D=0:4321, 32-bit", "4321.0"),
    ("total_exported", "0x021E:0x021F=1:2, 32-bit", "65538.0"),
    ("total_purchased", "0x0220:0x0221=0:999", "999.0"),
    ("total_consumption", "0x0222:0x0223=0:5000", "5000.0"),
    ("total_charged", "0x0226:0x0227=0:1500", "1500.0"),
    ("total_discharged", "0x0228:0x0229=0:1400", "1400.0"),
    ("inverter_temperature", "0x0238=35, signed", "35.0"),
    ("inverter_heatsink_temperature", "0x0239=0xFFFE (-2), signed", "-2.0"),
    ("working_mode", "0x1200=3", "3.0"),
    ("operating_mode", "0x1200=3 decoded", "Passive"),
    ("eps_output_voltage", "0x0216=0", "0.0"),
    ("eps_output_power", "0x0217=0", "0.0"),
]


@pytest.mark.parametrize(("suffix", "raw", "expected"), EXPECTED, ids=[e[0] for e in EXPECTED])
async def test_me3000_decoding(
    hass: HomeAssistant, gateway: FakeSofarGateway, suffix: str, raw: str, expected: str
) -> None:
    await setup_entry(hass, gateway)
    state = hass.states.get(PREFIX + suffix)
    assert state is not None, f"{PREFIX + suffix} missing ({raw})"
    assert state.state == expected, raw


async def test_units_and_classes(hass: HomeAssistant, gateway: FakeSofarGateway) -> None:
    await setup_entry(hass, gateway)
    attrs = hass.states.get(PREFIX + "battery_power").attributes
    assert attrs["unit_of_measurement"] == "W" and attrs["device_class"] == "power"
    attrs = hass.states.get(PREFIX + "total_exported").attributes
    assert attrs["unit_of_measurement"] == "kWh" and attrs["state_class"] == "total_increasing"
    attrs = hass.states.get(PREFIX + "battery_soc").attributes
    assert attrs["unit_of_measurement"] == "%" and attrs["device_class"] == "battery"


async def test_diagnostic_and_device_values(hass: HomeAssistant, gateway: FakeSofarGateway) -> None:
    """Values that are disabled-by-default entities are checked on the coordinator."""
    _, coordinator = await setup_entry(hass, gateway)
    data = coordinator.data
    assert data["grid_code"] == "UK-G59"
    assert data["battery_type_text"] == "PYLON"
    assert data["battery_capacity"] == 300
    assert data["battery_max_charge_voltage"] == 532.0
    assert data["battery_dod"] == 80
    assert data["bus_voltage"] == 380.0
    assert coordinator.serial_number == "SE1E003012345A"
    assert coordinator.arm_version == "V120"
    assert data["rated_power"] == 3000
    assert data["arm_firmware_version"] == "V120"
    assert data["last_event"] == "ID57 Battery temperature protection"


async def test_firmware_gating_hides_battery_energy_counters(
    hass: HomeAssistant, gateway: FakeSofarGateway
) -> None:
    for offset, value in enumerate(ascii_registers("V110", 2)):
        gateway.regs[0x2008 + offset] = value
    await setup_entry(hass, gateway)
    assert hass.states.get(PREFIX + "today_charged").state == "unavailable"
    assert hass.states.get(PREFIX + "total_discharged").state == "unavailable"
    assert hass.states.get(PREFIX + "today_exported").state == "3.21"


async def test_negative_battery_and_32bit_high_word(hass: HomeAssistant, gateway: FakeSofarGateway) -> None:
    gateway.regs[0x020D] = u16(-300)  # discharging 3000 W
    gateway.regs[0x020F] = u16(-5770)
    gateway.regs[0x021C], gateway.regs[0x021D] = 0x0002, 0x0003
    await setup_entry(hass, gateway)
    assert hass.states.get(PREFIX + "battery_power").state == "-3000.0"
    assert hass.states.get(PREFIX + "battery_current").state == "-57.7"
    assert hass.states.get(PREFIX + "total_generation").state == str(float(2 * 65536 + 3))


async def test_fault_bits_decode(hass: HomeAssistant, gateway: FakeSofarGateway) -> None:
    gateway.regs[0x0201] = 0x0001  # ID01 grid over voltage
    gateway.regs[0x0204] = 0x0100  # ID57 battery over temperature
    gateway.regs[0x022B] = 0x0001  # alarm ID81 temperature derating
    gateway.regs[0x0203] = 0x0002  # undocumented bit
    _, coordinator = await setup_entry(hass, gateway)
    faults = hass.states.get(PREFIX + "active_faults")
    assert faults.state.startswith("ID01 Grid over voltage, ID57 Battery temperature protection")
    assert "Undocumented bit set (fault_word_3 byte0 bit1)" in faults.state
    assert faults.attributes["count"] == 3
    assert hass.states.get(PREFIX + "active_alarms").state == "ID81 Over temperature derating"
    assert hass.states.get("binary_sensor.sofar_inverter_problem").state == "on"


async def test_no_fault_text_is_the_string_none(hass: HomeAssistant, gateway: FakeSofarGateway) -> None:
    """Documents upstream behaviour: an empty fault list is the state "None"."""
    await setup_entry(hass, gateway)
    assert hass.states.get(PREFIX + "active_faults").state == "None"
    assert hass.states.get("binary_sensor.sofar_inverter_problem").state == "off"
