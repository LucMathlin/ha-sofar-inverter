"""What reaches the inverter without anyone asking.

Every frame the integration sends on its own (no service call, no entity change)
is recorded and classified. Plain FC 0x03 / 0x04 reads are fine; everything else
is listed explicitly so that a change in upstream shows up as a test failure.
"""

from __future__ import annotations

import asyncio
import struct
from unittest.mock import patch

import pytest

import custom_components.sofar_inverter.coordinator as coordinator_module
from homeassistant.core import HomeAssistant

from .conftest import setup_entry
from .fake_sofar import (
    FC_BROADCAST_TIME,
    FC_EVENT_LOG,
    FC_HEARTBEAT,
    FC_PASSIVE,
    FC_WRITE_MULTIPLE,
    FC_WRITE_SINGLE,
    FakeSofarGateway,
)

CLOCK_READ = (FC_WRITE_MULTIPLE, "80000000")  # Sofar "read RTC": FC 0x10, 0x8000, count 0


def _signature(frames) -> set[tuple[int, str]]:
    """(function code, data) for every frame that is not a plain read,
    with the heartbeat and event-log read reduced to their function code."""
    out = set()
    for f in frames:
        if f.fc in (0x03, 0x04):
            continue
        if f.fc in (FC_HEARTBEAT, FC_EVENT_LOG):
            out.add((f.fc, ""))
        else:
            out.add((f.fc, f.data.hex()))
    return out


@pytest.fixture(autouse=True)
def fast_heartbeat():
    with (
        patch.object(coordinator_module, "HEARTBEAT_INTERVAL", 0.01),
        patch.object(coordinator_module, "HEARTBEAT_RETRY_DELAY", 0.01),
    ):
        yield


async def _run_a_while(hass: HomeAssistant, coordinator, polls: int = 4) -> None:
    """Several polls, an hourly clock read, a 5-minute event read, heartbeats."""
    for _ in range(polls):
        coordinator._clock_next_read = 0.0
        coordinator._event_next_read = 0.0
        await coordinator.async_refresh()
        await asyncio.sleep(0.03)


@pytest.mark.parametrize("working_mode", [0, 3])
async def test_control_off_sends_only_standard_reads(
    hass: HomeAssistant, gateway: FakeSofarGateway, working_mode
) -> None:
    """Defaults (control off): nothing but FC 0x03 / 0x04 reaches the inverter."""
    gateway.working_mode = working_mode
    _, coordinator = await setup_entry(hass, gateway)
    await _run_a_while(hass, coordinator)
    assert gateway.frames
    assert {f.fc for f in gateway.frames} <= {0x03, 0x04}
    assert gateway.writes == []


async def test_control_off_ignores_schedule_and_clock_options(
    hass: HomeAssistant, gateway: FakeSofarGateway
) -> None:
    """Schedule and clock monitoring switched on do not undo read-only mode."""
    _, coordinator = await setup_entry(
        hass, gateway, enable_schedule=True, enable_clock_monitor=True
    )
    await _run_a_while(hass, coordinator)
    assert {f.fc for f in gateway.frames} <= {0x03, 0x04}
    assert coordinator.supports_schedule is False and coordinator.supports_clock is False
    assert hass.states.get("select.sofar_inverter_working_mode") is None
    assert hass.states.get("number.sofar_inverter_timing_charge_power") is None


async def test_control_on_non_read_frames(hass: HomeAssistant, gateway: FakeSofarGateway) -> None:
    """With control on: heartbeat, event-log read, Sofar's RTC read (FC 0x10 at
    0x8000, count 0) and the one start-up fail-safe Auto. Nothing is written."""
    _, coordinator = await setup_entry(hass, gateway, enable_control=True)
    await _run_a_while(hass, coordinator)
    assert gateway.writes == []
    assert _signature(gateway.frames) == {
        (FC_HEARTBEAT, ""),
        (FC_EVENT_LOG, ""),
        CLOCK_READ,
        (FC_PASSIVE, "01035555"),
    }


async def test_control_enabled_sends_only_the_failsafe(hass: HomeAssistant, gateway: FakeSofarGateway) -> None:
    _, coordinator = await setup_entry(hass, gateway, enable_control=True)
    await _run_a_while(hass, coordinator)
    assert gateway.passive_commands() == [(0x0103, 0x5555)]
    assert gateway.writes == []


async def test_control_enabled_outside_passive_sends_nothing(
    hass: HomeAssistant, gateway: FakeSofarGateway
) -> None:
    gateway.working_mode = 0
    _, coordinator = await setup_entry(hass, gateway, enable_control=True)
    await _run_a_while(hass, coordinator)
    assert gateway.frames_for(FC_PASSIVE) == []
    assert gateway.writes == []


async def test_schedule_support_reads_but_never_writes(hass: HomeAssistant, gateway: FakeSofarGateway) -> None:
    gateway.regs[0x1201] = 0x0200  # timing charge start 02:00
    gateway.regs[0x120E] = 0x007F  # TOU every day
    _, coordinator = await setup_entry(hass, gateway, enable_schedule=True, enable_control=True)
    await _run_a_while(hass, coordinator)
    assert gateway.writes == []
    assert gateway.working_mode == 3  # never changed on its own
    reads = {f.data[:4].hex() for f in gateway.frames if f.fc == 0x03}
    assert "12000010" in reads  # 0x1200-0x120F in one block
    assert hass.states.get("time.sofar_inverter_timing_charge_start").state == "02:00:00"
    assert hass.states.get("switch.sofar_inverter_tou_monday").state == "on"


async def test_schedule_write_only_when_asked(hass: HomeAssistant, gateway: FakeSofarGateway) -> None:
    _, coordinator = await setup_entry(hass, gateway, enable_schedule=True, enable_control=True)
    await hass.services.async_call(
        "number", "set_value",
        {"entity_id": "number.sofar_inverter_timing_charge_power", "value": 1500}, blocking=True,
    )
    # The whole 6-register timing block is rewritten, then read back.
    assert gateway.writes == [(0x1201, [0, 0, 0, 0, 1500, 0])]


async def test_clock_is_only_set_by_the_button(hass: HomeAssistant, gateway: FakeSofarGateway) -> None:
    _, coordinator = await setup_entry(hass, gateway, enable_control=True)
    await _run_a_while(hass, coordinator)
    assert gateway.frames_for(FC_BROADCAST_TIME) == []
    await hass.services.async_call(
        "button", "press", {"entity_id": "button.sofar_inverter_sync_inverter_clock"}, blocking=True
    )
    frames = gateway.frames_for(FC_BROADCAST_TIME)
    assert len(frames) == 1 and frames[0].unit == 0x88  # broadcast to the whole bus
    assert struct.unpack(">HH", frames[0].data[:4]) == (0x5000, 3)


async def test_working_mode_is_only_written_by_its_select(hass: HomeAssistant, gateway: FakeSofarGateway) -> None:
    _, coordinator = await setup_entry(hass, gateway, enable_control=True)
    await _run_a_while(hass, coordinator)
    assert gateway.writes == []
    await hass.services.async_call(
        "select", "select_option",
        {"entity_id": "select.sofar_inverter_working_mode", "option": "Auto"}, blocking=True,
    )
    assert gateway.writes == [(0x1200, [0])]
