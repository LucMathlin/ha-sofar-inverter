"""The start-up fail-safe and what a Home Assistant restart does to the inverter."""

from __future__ import annotations

import asyncio

from homeassistant.core import HomeAssistant

from .conftest import setup_entry
from .fake_sofar import FC_HEARTBEAT, WATCHDOG_SECONDS, FakeSofarGateway

AUTO = (0x0103, 0x5555)
MODE = "select.sofar_inverter_battery_mode"


async def test_one_auto_when_passive_and_no_override(hass: HomeAssistant, gateway: FakeSofarGateway) -> None:
    _, coordinator = await setup_entry(hass, gateway, enable_control=True)
    assert gateway.passive_commands() == [AUTO]
    assert coordinator.last_command_result == "Accepted"
    for _ in range(3):
        await coordinator.async_refresh()
    assert gateway.passive_commands() == [AUTO]  # only once
    assert hass.states.get(MODE).state == "No Override"


async def test_no_failsafe_outside_passive_mode(hass: HomeAssistant, gateway: FakeSofarGateway) -> None:
    gateway.working_mode = 0  # Auto / self-use set on the inverter
    _, coordinator = await setup_entry(hass, gateway, enable_control=True)
    gateway.working_mode = 3
    await coordinator.async_refresh()
    assert gateway.passive_commands() == []  # decided once, on the first good read


async def test_no_failsafe_with_control_disabled(hass: HomeAssistant, gateway: FakeSofarGateway) -> None:
    _, coordinator = await setup_entry(hass, gateway, enable_control=False)
    await coordinator.async_refresh()
    assert gateway.passive_commands() == []


async def test_failsafe_given_up_when_working_mode_unreadable(
    hass: HomeAssistant, gateway: FakeSofarGateway
) -> None:
    gateway.silent_ranges = [range(0x1200, 0x1201)]
    _, coordinator = await setup_entry(hass, gateway, enable_control=True)
    for _ in range(3):
        await coordinator.async_refresh()
    gateway.silent_ranges = []
    await coordinator.async_refresh()
    assert coordinator.data["working_mode"] == 3
    assert gateway.passive_commands() == []


async def test_restart_with_forced_charge(hass: HomeAssistant, gateway: FakeSofarGateway) -> None:
    """Charge -> HA stops (no heartbeat) -> watchdog standby -> HA starts -> one Auto."""
    entry, coordinator = await setup_entry(hass, gateway, enable_control=True)
    await hass.services.async_call(
        "number", "set_value",
        {"entity_id": "number.sofar_inverter_battery_power_setpoint", "value": 2500}, blocking=True,
    )
    await hass.services.async_call("select", "select_option", {"entity_id": MODE, "option": "Charge"}, blocking=True)
    assert gateway.passive_state == "charge"

    # Home Assistant stops: the entry unloads, heartbeats stop, nothing is sent.
    assert await hass.config_entries.async_unload(entry.entry_id)
    await hass.async_block_till_done()
    await asyncio.sleep(0.02)
    gateway.clear()
    gateway.advance(WATCHDOG_SECONDS - 1)
    assert gateway.passive_state == "charge"  # still charging just inside the window
    gateway.advance(2)
    assert gateway.passive_state == "standby" and gateway.watchdog_trips == 1
    assert gateway.frames == []

    # Home Assistant starts again.
    assert await hass.config_entries.async_setup(entry.entry_id)
    await hass.async_block_till_done()
    assert gateway.passive_commands() == [AUTO]
    assert gateway.passive_state == "auto"
    assert hass.states.get(MODE).state == "No Override"  # forced mode is not restored
    assert hass.states.get("number.sofar_inverter_battery_power_setpoint").state == "0.0"


async def test_quick_restart_still_ends_a_forced_charge(hass: HomeAssistant, gateway: FakeSofarGateway) -> None:
    """A restart under 60 s skips standby, but the fail-safe still releases the charge."""
    entry, _ = await setup_entry(hass, gateway, enable_control=True)
    await hass.services.async_call("select", "select_option", {"entity_id": MODE, "option": "Charge"}, blocking=True)
    assert await hass.config_entries.async_unload(entry.entry_id)
    await hass.async_block_till_done()
    gateway.advance(30)
    assert gateway.passive_state == "charge"
    assert await hass.config_entries.async_setup(entry.entry_id)
    await hass.async_block_till_done()
    assert gateway.watchdog_trips == 0
    assert gateway.passive_state == "auto"


async def test_restart_with_control_disabled_leaves_setpoint(hass: HomeAssistant, gateway: FakeSofarGateway) -> None:
    """With control off nothing is released: a setpoint left by another
    controller (e.g. the MQTT bridge) stays until the watchdog drops it."""
    gateway.passive_state, gateway.passive_watts = "charge", 2000
    gateway.last_heartbeat = gateway.now()
    _, coordinator = await setup_entry(hass, gateway, enable_control=False)
    await coordinator.async_refresh()
    assert gateway.passive_commands() == []
    assert gateway.passive_state == "charge"
    # ...but the integration's own heartbeat keeps that setpoint alive.
    gateway.advance(WATCHDOG_SECONDS + 5)
    assert gateway.passive_state == "standby"  # in this test no beat was due yet
    assert FC_HEARTBEAT  # documented in test_heartbeat: beats run with control off
