"""The FC 0x49 passive-mode heartbeat: frame, cadence, fast retry, lock contention."""

from __future__ import annotations

import asyncio
import logging
import time
import types
from unittest.mock import patch

import pytest

import custom_components.sofar_inverter.coordinator as coordinator_module
from custom_components.sofar_inverter.coordinator import (
    HEARTBEAT_INTERVAL,
    HEARTBEAT_MAX_FAST_RETRIES,
    HEARTBEAT_RETRY_DELAY,
)
from homeassistant.core import HomeAssistant

from .conftest import FAST_TIMEOUT, PYMODBUS_RETRIES, REAL_TIMEOUT, setup_entry
from .fake_sofar import FC_HEARTBEAT, WATCHDOG_SECONDS, FakeSofarGateway

SCALE = REAL_TIMEOUT / FAST_TIMEOUT  # test seconds -> live seconds for I/O waits


class SleepRecorder:
    """Stands in for the asyncio module inside coordinator.py.

    Records every delay the heartbeat loop asks for and sleeps a scaled-down
    amount instead, so the loop's real schedule can be checked quickly.
    """

    def __init__(self, scale: float = 0.001) -> None:
        self.delays: list[float] = []
        self._scale = scale
        self.module = types.SimpleNamespace(**vars(asyncio))
        self.module.sleep = self._sleep

    async def _sleep(self, delay: float, *args, **kwargs):
        self.delays.append(delay)
        await asyncio.sleep(delay * self._scale)

    async def wait_for(self, count: int, timeout: float = 5.0) -> None:
        deadline = time.monotonic() + timeout
        while len(self.delays) < count:
            assert time.monotonic() < deadline, f"only {len(self.delays)} sleeps: {self.delays}"
            await asyncio.sleep(0.005)


@pytest.fixture
def sleeps():
    recorder = SleepRecorder()
    with patch.object(coordinator_module, "asyncio", recorder.module):
        yield recorder


def test_design_margin_against_the_60s_watchdog() -> None:
    """Worst case with the gateway refusing connections: one interval plus
    every fast retry, each paying a full connect timeout."""
    assert (HEARTBEAT_INTERVAL, HEARTBEAT_RETRY_DELAY, HEARTBEAT_MAX_FAST_RETRIES) == (9, 2, 3)
    worst = (
        HEARTBEAT_INTERVAL
        + HEARTBEAT_MAX_FAST_RETRIES * HEARTBEAT_RETRY_DELAY
        + (HEARTBEAT_MAX_FAST_RETRIES + 1) * REAL_TIMEOUT
    )
    assert worst == 35 < WATCHDOG_SECONDS


async def test_heartbeat_frame(hass: HomeAssistant, gateway: FakeSofarGateway, sleeps: SleepRecorder) -> None:
    await setup_entry(hass, gateway)
    await sleeps.wait_for(3)
    beats = gateway.frames_for(FC_HEARTBEAT)
    assert beats, "no heartbeat sent"
    assert {f.unit for f in beats} == {1}
    assert {f.pdu.hex(" ") for f in beats} == {"49 22 01 22 02"}
    assert not any(f.answered for f in beats)  # fire-and-forget


async def test_heartbeat_cadence(hass: HomeAssistant, gateway: FakeSofarGateway, sleeps: SleepRecorder) -> None:
    # Measured from the first beat on the wire, so set-up is not part of it.
    await setup_entry(hass, gateway)
    deadline = time.monotonic() + 5
    while not gateway.frames_for(FC_HEARTBEAT):  # first beat on the wire
        assert time.monotonic() < deadline
        await asyncio.sleep(0.002)
    start = len(sleeps.delays)
    gateway.clear()
    await sleeps.wait_for(start + 6)
    assert sleeps.delays[start : start + 6] == [HEARTBEAT_INTERVAL] * 6
    assert len(gateway.frames_for(FC_HEARTBEAT)) >= 5


async def test_heartbeat_is_sent_even_with_control_disabled(
    hass: HomeAssistant, gateway: FakeSofarGateway, sleeps: SleepRecorder
) -> None:
    """There is no option that stops the heartbeat on an ME3000 entry, so the
    planned read-only first step is not free of non-read frames."""
    _, coordinator = await setup_entry(hass, gateway, enable_control=False)
    assert coordinator.enable_control is False
    await sleeps.wait_for(3)
    assert gateway.frames_for(FC_HEARTBEAT)


async def test_fast_retry_then_recovery(
    hass: HomeAssistant, gateway: FakeSofarGateway, sleeps: SleepRecorder, caplog
) -> None:
    _, coordinator = await setup_entry(hass, gateway)
    await sleeps.wait_for(2)
    port = gateway.port
    await gateway.stop()  # connection refused from now on
    coordinator._gateway._close_shared_client()

    start = len(sleeps.delays)
    await sleeps.wait_for(start + 9)
    window = sleeps.delays[start:]
    # Each failed beat: three quick retries 2 s apart, then back to 9 s.
    assert window[:8] == [2, 2, 2, 9, 2, 2, 2, 9]
    assert coordinator._gateway.heartbeat_failures >= 8
    assert "Heartbeat send failed" in caplog.text

    await gateway.start(port)
    await sleeps.wait_for(len(sleeps.delays) + 6)
    assert coordinator._gateway.heartbeat_failures == 0
    assert "recovered after" in caplog.text
    assert gateway.frames_for(FC_HEARTBEAT)


async def test_no_heartbeat_after_unload(
    hass: HomeAssistant, gateway: FakeSofarGateway, sleeps: SleepRecorder
) -> None:
    entry, _ = await setup_entry(hass, gateway)
    await sleeps.wait_for(2)
    assert await hass.config_entries.async_unload(entry.entry_id)
    await hass.async_block_till_done()
    await asyncio.sleep(0.02)  # let the fake drain a beat already on the wire
    gateway.clear()
    await asyncio.sleep(0.1)  # ~100 recorded intervals if the task were alive
    assert gateway.frames_for(FC_HEARTBEAT) == []


async def test_one_heartbeat_per_slave_on_a_shared_gateway(
    hass: HomeAssistant, gateway: FakeSofarGateway, sleeps: SleepRecorder
) -> None:
    gateway.slave = 1
    await setup_entry(hass, gateway)
    # A second inverter on the same RS485 bus (unit 2). The fake only answers
    # unit 1, so the second entry is set up with its first poll patched.
    from .conftest import make_entry

    entry2 = make_entry(gateway, slave=2, name="Sofar 2")
    entry2.add_to_hass(hass)
    with patch.object(coordinator_module.SofarCoordinator, "_async_update_data", return_value={}):
        assert await hass.config_entries.async_setup(entry2.entry_id)
    gateway.clear()
    await sleeps.wait_for(len(sleeps.delays) + 3)
    units = [f.unit for f in gateway.frames_for(FC_HEARTBEAT)]
    assert set(units) == {1, 2}
    assert sleeps.delays.count(HEARTBEAT_INTERVAL) >= 3  # still one loop


class LoopErrors:
    """Collect loop-level errors (e.g. data_received() failures) for a test."""

    def __init__(self) -> None:
        self.errors: list[dict] = []
        self._loop = asyncio.get_running_loop()
        self._previous = self._loop.get_exception_handler()

    def __enter__(self) -> list[dict]:
        self._loop.set_exception_handler(lambda _loop, context: self.errors.append(context))
        return self.errors

    def __exit__(self, *exc) -> None:
        self._loop.set_exception_handler(self._previous)


async def _run_with_answered_heartbeats(hass, gateway, sleeps) -> list[dict]:
    gateway.heartbeat_reply = "echo"
    with LoopErrors() as errors:
        entry, _ = await setup_entry(hass, gateway)
        await sleeps.wait_for(6)
        assert await hass.config_entries.async_unload(entry.entry_id)
        await hass.async_block_till_done()
        await asyncio.sleep(0.02)
    return errors


# ------------------------------------------------------------ known problems
def _measure_gap(gateway: FakeSofarGateway) -> float:
    beats = [f.at for f in gateway.frames_for(FC_HEARTBEAT)]
    return max(b - a for a, b in zip(beats, beats[1:]))


@pytest.mark.xfail(
    strict=True,
    reason="BUG: a poll holds the gateway lock through every timeout and retry, so a "
    "register range the inverter does not answer starves the heartbeat past 60 s",
)
async def test_silent_register_range_does_not_starve_heartbeat(
    hass: HomeAssistant, gateway: FakeSofarGateway
) -> None:
    """One unanswered range (here the 0x10B0 battery parameters) costs
    2 function codes x 4 attempts per block plus 2 x 4 per register on fallback.
    Each attempt waits the full 5 s timeout while holding the lock."""
    gateway.silent_ranges = [range(0x10B0, 0x10BD)]
    interval = HEARTBEAT_INTERVAL / SCALE
    with patch.object(coordinator_module, "HEARTBEAT_INTERVAL", interval):
        _, coordinator = await setup_entry(hass, gateway)
        gateway.clear()
        await asyncio.sleep(interval * 3)
        await coordinator.async_refresh()
        await asyncio.sleep(interval * 3)
    unanswered = [f for f in gateway.frames if not f.answered and f.fc != FC_HEARTBEAT]
    live_gap = _measure_gap(gateway) * SCALE
    logging.getLogger(__name__).warning(
        "%s unanswered requests in one poll; longest heartbeat gap ~%.0f s live",
        len(unanswered), live_gap,
    )
    assert live_gap < WATCHDOG_SECONDS


@pytest.mark.xfail(
    strict=True,
    reason="BUG/RISK: if the inverter answers FC 0x49, pymodbus cannot decode the "
    "reply and data_received() raises, tearing down the shared connection",
)
async def test_answered_heartbeat_keeps_the_connection(
    hass: HomeAssistant, gateway: FakeSofarGateway, sleeps: SleepRecorder
) -> None:
    errors = await _run_with_answered_heartbeats(hass, gateway, sleeps)
    assert len(gateway.frames_for(FC_HEARTBEAT)) >= 4
    assert errors == []
    assert gateway.connections <= 2  # shared client + one clock-read connection


async def test_answered_heartbeat_reconnects_every_beat(
    hass: HomeAssistant, gateway: FakeSofarGateway, sleeps: SleepRecorder
) -> None:
    """Pins today's behaviour behind the xfail above."""
    errors = await _run_with_answered_heartbeats(hass, gateway, sleeps)
    beats = len(gateway.frames_for(FC_HEARTBEAT))
    assert beats >= 4
    assert len(errors) >= beats - 1
    assert "data_received() call failed" in errors[0]["message"]
    assert gateway.connections >= beats
