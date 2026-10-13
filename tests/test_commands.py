"""Charge / discharge / standby / auto / battery-save commands and their 0x42 frames,
and parsing of the 0x42 status word (including rejections)."""

from __future__ import annotations

import pytest

from custom_components.sofar_inverter.coordinator import (
    CommandResult,
    SofarCoordinator,
    SofarPassiveModeResponse,
)
from custom_components.sofar_inverter.const import MODE_CHARGE
from homeassistant.core import HomeAssistant
from pymodbus.pdu import ExceptionResponse

from .conftest import setup_entry
from .fake_sofar import FC_PASSIVE, FakeSofarGateway, u16

MODE = "select.sofar_inverter_battery_mode"
POWER = "number.sofar_inverter_battery_power_setpoint"


async def _set_power(hass: HomeAssistant, watts: float) -> None:
    await hass.services.async_call("number", "set_value", {"entity_id": POWER, "value": watts}, blocking=True)


async def _select(hass: HomeAssistant, option: str) -> None:
    await hass.services.async_call("select", "select_option", {"entity_id": MODE, "option": option}, blocking=True)
    await hass.async_block_till_done()


def _pdus(gateway: FakeSofarGateway) -> list[str]:
    return [f.pdu.hex(" ") for f in gateway.frames_for(FC_PASSIVE)]


@pytest.fixture
async def control(hass: HomeAssistant, gateway: FakeSofarGateway):
    """Control enabled; the start-up fail-safe frame is cleared away."""
    _, coordinator = await setup_entry(hass, gateway, enable_control=True)
    gateway.clear()
    return coordinator


# ---------------------------------------------------------------- frames
@pytest.mark.parametrize(
    ("option", "watts", "pdu", "state"),
    [
        ("Charge", 2000, "42 01 02 07 d0", "charge"),
        ("Discharge", 1500, "42 01 01 05 dc", "discharge"),
        ("Standby", 0, "42 01 00 55 55", "standby"),
        ("Auto (Self Use)", 0, "42 01 03 55 55", "auto"),
    ],
)
async def test_mode_frames(
    hass: HomeAssistant, gateway: FakeSofarGateway, control: SofarCoordinator, option, watts, pdu, state
) -> None:
    await _set_power(hass, watts)
    assert _pdus(gateway) == []  # a setpoint alone sends nothing
    await _select(hass, option)
    frames = gateway.frames_for(FC_PASSIVE)
    assert frames and all(f.unit == 1 for f in frames)
    assert _pdus(gateway)[0] == pdu
    assert gateway.passive_state == state
    assert control.last_command_result == "Accepted"
    assert hass.states.get(MODE).state == option


async def test_auto_parameter_is_0x5555(hass: HomeAssistant, gateway: FakeSofarGateway, control) -> None:
    gateway.strict_params = True  # fake rejects auto with any other parameter
    await _select(hass, "Auto (Self Use)")
    assert gateway.passive_commands()[0] == (0x0103, 0x5555)
    assert gateway.passive_state == "auto"


async def test_selecting_a_mode_sends_it_twice_then_every_poll(
    hass: HomeAssistant, gateway: FakeSofarGateway, control: SofarCoordinator
) -> None:
    """The select dispatches once, then its refresh re-applies; every poll re-sends."""
    await _set_power(hass, 2000)
    await _select(hass, "Charge")
    assert gateway.passive_commands() == [(0x0102, 2000)] * 2
    await control.async_refresh()
    await control.async_refresh()
    assert gateway.passive_commands() == [(0x0102, 2000)] * 4


async def test_setpoint_change_while_charging_is_pushed(
    hass: HomeAssistant, gateway: FakeSofarGateway, control
) -> None:
    await _set_power(hass, 1000)
    await _select(hass, "Charge")
    gateway.clear()
    await _set_power(hass, 2500)
    assert gateway.passive_commands()[0] == (0x0102, 2500)
    assert gateway.passive_watts == 2500


async def test_no_override_sends_nothing(hass: HomeAssistant, gateway: FakeSofarGateway, control) -> None:
    await _select(hass, "Charge")
    gateway.clear()
    await _select(hass, "No Override")
    await control.async_refresh()
    assert gateway.passive_commands() == []
    assert gateway.passive_state == "charge"  # the inverter keeps the last setpoint


async def test_buttons(hass: HomeAssistant, gateway: FakeSofarGateway, control) -> None:
    await hass.services.async_call("button", "press", {"entity_id": "button.sofar_inverter_set_standby"}, blocking=True)
    assert gateway.passive_commands()[0] == (0x0100, 0x5555)
    gateway.clear()
    await hass.services.async_call("button", "press", {"entity_id": "button.sofar_inverter_set_auto_mode"}, blocking=True)
    assert gateway.passive_commands()[0] == (0x0103, 0x5555)


@pytest.mark.parametrize(
    ("grid_raw", "expected_cmd"),
    [
        (u16(50), 0x0103),  # +500 W: integration treats positive as export -> auto
        (u16(-5), 0x0103),  # -50 W: inside the -100 W dead band -> auto
        (u16(-50), 0x0100),  # -500 W: treated as import -> standby
    ],
)
async def test_battery_save(
    hass: HomeAssistant, gateway: FakeSofarGateway, control: SofarCoordinator, grid_raw, expected_cmd
) -> None:
    """Battery Save picks Auto or Standby from the sign of grid_power (0x0212).

    The integration (like the Sofar2mqtt firmware) treats positive as export.
    At least one Home Assistant MQTT setup for an ME3000SP treats positive as
    import, so the sign should be confirmed on hardware.
    """
    gateway.regs[0x0212] = grid_raw
    await control.async_refresh()
    gateway.clear()
    await _select(hass, "Battery Save")
    # The select dispatches with grid_power=None, which always means Auto; the
    # refresh that follows then applies the grid-power rule.
    assert gateway.passive_commands() == [(0x0103, 0x5555), (expected_cmd, 0x5555)]
    assert control.effective_mode == ("auto" if expected_cmd == 0x0103 else "standby")


async def test_power_is_clamped_to_max_power(
    hass: HomeAssistant, gateway: FakeSofarGateway, control: SofarCoordinator
) -> None:
    control.set_active_mode(MODE_CHARGE, 5000)
    assert control.active_power == 3000
    await control.async_set_mode(MODE_CHARGE)
    assert gateway.passive_commands()[0] == (0x0102, 3000)


async def test_power_is_clamped_to_detected_rating(hass: HomeAssistant, gateway: FakeSofarGateway) -> None:
    """Configured 5000 W, barcode says ME3000SP (3000 W): commands use 3000 W."""
    _, coordinator = await setup_entry(hass, gateway, enable_control=True, max_power=5000)
    gateway.clear()
    await _set_power(hass, 4500)
    await _select(hass, "Discharge")
    assert gateway.passive_commands()[0] == (0x0101, 3000)
    assert coordinator.effective_max_power == 3000


# ----------------------------------------------------------- status word
def _parse(word_or_response) -> CommandResult:
    response = (
        SofarPassiveModeResponse(values=[word_or_response])
        if isinstance(word_or_response, int)
        else word_or_response
    )
    return SofarCoordinator._parse_passive_response(None, response)


@pytest.mark.parametrize(
    ("word", "ok", "message", "flags"),
    [
        (0x0300, True, "Accepted", (True, True, False, False)),
        (0x0001, False, "Rejected: invalid mode (inverter is not in Passive Mode)", (False,) * 4),
        (0x0002, False, "Rejected: CRC failed", (False,) * 4),
        (0x0303, False, "Rejected: inverter busy", (True, True, False, False)),
        (0x0004, False, "Rejected: invalid data", (False,) * 4),
        (0x0600, True, "Accepted", (False, True, True, False)),  # full
        (0x0900, True, "Accepted", (True, False, False, True)),  # empty
        (0x0F07, False, "Unknown status 0x07", (True,) * 4),
    ],
)
def test_status_word_parsing(word, ok, message, flags) -> None:
    result = _parse(word)
    assert result.ok is ok
    assert result.message == message
    assert result.status_code == word & 0xFF
    assert tuple(result.flags.values()) == flags
    assert list(result.flags) == [
        "battery_charge_ready",
        "battery_discharge_ready",
        "battery_full",
        "battery_empty",
    ]


def test_status_parsing_edge_cases() -> None:
    assert _parse(None).message == "No response from inverter"
    assert _parse(SofarPassiveModeResponse(values=[])).ok is True  # no word: assumed ok
    exc = ExceptionResponse(FC_PASSIVE, 0x01)
    result = _parse(exc)
    assert result.ok is False and result.message == "Modbus exception 0x01: Illegal function"


async def test_not_in_passive_mode_is_reported(
    hass: HomeAssistant, gateway: FakeSofarGateway, control: SofarCoordinator
) -> None:
    gateway.working_mode = 0
    await control.async_refresh()
    await _set_power(hass, 2000)
    await _select(hass, "Charge")
    assert control.last_command_result == "Rejected: invalid mode (inverter is not in Passive Mode)"
    state = hass.states.get(MODE)
    # The select still shows the requested mode; the rejection is only in the
    # attributes and the (disabled-by-default) control_last_result sensor.
    assert state.state == "Charge"
    assert state.attributes["last_command_result"].startswith("Rejected: invalid mode")
    assert gateway.passive_state == "auto"  # still the start-up fail-safe
    # ...and the rejected command keeps being re-sent on every poll.
    before = len(gateway.passive_commands())
    await control.async_refresh()
    assert len(gateway.passive_commands()) == before + 1


async def test_busy_and_exception_rejections(
    hass: HomeAssistant, gateway: FakeSofarGateway, control: SofarCoordinator
) -> None:
    gateway.busy = True
    await _select(hass, "Standby")
    assert control.last_command_result == "Rejected: inverter busy"
    gateway.busy = False
    gateway.exception_code, gateway.exception_fcs = 0x01, {FC_PASSIVE}
    await _select(hass, "Auto (Self Use)")
    assert control.last_command_result == "Modbus exception 0x01: Illegal function"


async def test_passive_flags_reach_binary_sensors(
    hass: HomeAssistant, gateway: FakeSofarGateway, control: SofarCoordinator
) -> None:
    gateway.regs[0x0210] = 100
    await _select(hass, "Auto (Self Use)")
    assert control.passive_flags == {
        "battery_charge_ready": False,
        "battery_discharge_ready": True,
        "battery_full": True,
        "battery_empty": False,
    }


async def test_echoed_reply_is_misread_as_rejection(
    hass: HomeAssistant, gateway: FakeSofarGateway, control: SofarCoordinator
) -> None:
    """If the real ME3000SP echoes cmd+param instead of returning a status word, every command is reported as
    rejected although the inverter carried it out. Which layout the unit uses
    must be captured on hardware before control is enabled."""
    gateway.passive_reply = "echo"
    await _set_power(hass, 2000)
    await _select(hass, "Charge")
    assert gateway.passive_state == "charge"  # the inverter did it
    assert control.last_command_result == "Unknown status 0x07"  # 0x0207 misparsed
    await _select(hass, "Auto (Self Use)")
    assert control.last_command_result == "Unknown status 0x55"
