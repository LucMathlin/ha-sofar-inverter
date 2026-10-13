"""Entities going unavailable when the gateway stops answering, and recovery."""

from __future__ import annotations

import logging
import time

import pytest

from homeassistant.core import HomeAssistant

from .conftest import FAST_TIMEOUT, REAL_TIMEOUT, setup_entry
from .fake_sofar import FC_HEARTBEAT, FakeSofarGateway

SOC = "sensor.sofar_inverter_battery_soc"
SCALE = REAL_TIMEOUT / FAST_TIMEOUT
_LOGGER = logging.getLogger(__name__)


def _poll_requests(gateway: FakeSofarGateway) -> list:
    return [f for f in gateway.frames if f.fc != FC_HEARTBEAT]


async def _timed_refresh(coordinator) -> float:
    """Wall time of one poll; only meaningful when it is dominated by timeouts."""
    start = time.monotonic()
    await coordinator.async_refresh()
    return time.monotonic() - start


async def test_gateway_down_then_back(hass: HomeAssistant, gateway: FakeSofarGateway) -> None:
    _, coordinator = await setup_entry(hass, gateway)
    assert hass.states.get(SOC).state == "60.0"
    port = gateway.port
    await gateway.stop()

    await coordinator.async_refresh()
    await hass.async_block_till_done()
    assert coordinator.last_update_success is False
    assert hass.states.get(SOC).state == "unavailable"
    assert hass.states.get("binary_sensor.sofar_inverter_problem").state == "unavailable"

    await gateway.start(port)
    await coordinator.async_refresh()
    await hass.async_block_till_done()
    assert coordinator.last_update_success is True
    assert hass.states.get(SOC).state == "60.0"


async def test_connection_dropped_by_gateway_reconnects(hass: HomeAssistant, gateway: FakeSofarGateway) -> None:
    _, coordinator = await setup_entry(hass, gateway)
    connections = gateway.connections
    gateway.drop_connections()
    await coordinator.async_refresh()
    await coordinator.async_refresh()
    assert coordinator.last_update_success is True
    assert gateway.connections > connections


@pytest.mark.parametrize(
    "fault",
    [
        {"silent": True},
        {"silent": True, "gateway_timeout_exception": True},
        {"exception_code": 0x04},
        {"short_reply": True},
        {"wrong_unit": True},
    ],
    ids=["silent", "gateway-0x0B", "exception", "short-reply", "wrong-unit"],
)
async def test_faults_make_entities_unavailable(
    hass: HomeAssistant, gateway: FakeSofarGateway, fault: dict
) -> None:
    _, coordinator = await setup_entry(hass, gateway)
    for key, value in fault.items():
        setattr(gateway, key, value)
    gateway.clear()
    await coordinator.async_refresh()
    await hass.async_block_till_done()
    assert coordinator.last_update_success is False
    assert hass.states.get(SOC).state == "unavailable"
    # "Unanswered" is from the gateway's side; a wrong-unit reply is answered
    # on the wire but discarded by pymodbus, so it times out in the client too.
    requests = _poll_requests(gateway)
    unanswered = [f for f in requests if not f.answered or gateway.wrong_unit]
    _LOGGER.warning(
        "%s: %s requests, %s unanswered (~%.0f s of %.0f s timeouts) before unavailable",
        fault, len(requests), len(unanswered), len(unanswered) * REAL_TIMEOUT, REAL_TIMEOUT,
    )


@pytest.mark.xfail(
    strict=True,
    reason="BUG: with a silent RS485 side each poll waits out every timeout and "
    "retry under the gateway lock (~3 min live) before entities go unavailable",
)
async def test_silent_inverter_detected_within_a_minute(hass: HomeAssistant, gateway: FakeSofarGateway) -> None:
    _, coordinator = await setup_entry(hass, gateway)
    gateway.silent = True
    elapsed = await _timed_refresh(coordinator)
    assert coordinator.last_update_success is False
    assert elapsed * SCALE < 60


async def test_recovery_after_silence(hass: HomeAssistant, gateway: FakeSofarGateway) -> None:
    _, coordinator = await setup_entry(hass, gateway)
    gateway.silent = True
    for _ in range(3):
        await coordinator.async_refresh()
    assert coordinator.last_update_success is False
    gateway.silent = False
    await coordinator.async_refresh()
    await hass.async_block_till_done()
    assert coordinator.last_update_success is True
    assert hass.states.get(SOC).state == "60.0"


async def test_reads_after_recovery_are_one_register_each(hass: HomeAssistant, gateway: FakeSofarGateway) -> None:
    """Pins today's behaviour behind the xfail below."""
    _, coordinator = await setup_entry(hass, gateway)
    gateway.clear()
    await coordinator.async_refresh()
    healthy = len(_poll_requests(gateway))
    gateway.silent = True
    for _ in range(3):
        await coordinator.async_refresh()
    gateway.silent = False
    gateway.clear()
    await coordinator.async_refresh()
    await coordinator.async_refresh()
    after = len(_poll_requests(gateway)) // 2
    assert healthy == 4  # 0x0200+62, 0x0242+7, 0x10B0+13, 0x1200+1
    assert after >= 60  # one transaction per register, for good


@pytest.mark.xfail(
    strict=True,
    reason="BUG: after three failed polls each ReadBlock sets individual=True and "
    "nothing ever resets it, so block reads never come back without a reload",
)
async def test_block_reads_resume_after_recovery(hass: HomeAssistant, gateway: FakeSofarGateway) -> None:
    _, coordinator = await setup_entry(hass, gateway)
    gateway.clear()
    await coordinator.async_refresh()
    healthy = len(_poll_requests(gateway))
    gateway.silent = True
    for _ in range(3):
        await coordinator.async_refresh()
    gateway.silent = False
    await coordinator.async_refresh()
    gateway.clear()
    await coordinator.async_refresh()
    assert len(_poll_requests(gateway)) <= healthy + 1


async def test_commands_wait_for_a_slow_poll(hass: HomeAssistant, gateway: FakeSofarGateway) -> None:
    """A Battery Mode change cannot reach the bus while a poll holds the lock."""
    import asyncio

    _, coordinator = await setup_entry(hass, gateway, enable_control=True)
    gateway.silent_ranges = [range(0x10B0, 0x10BD)]
    gateway.clear()
    poll = hass.async_create_task(coordinator.async_refresh())
    await asyncio.sleep(FAST_TIMEOUT)
    await hass.services.async_call(
        "select", "select_option",
        {"entity_id": "select.sofar_inverter_battery_mode", "option": "Standby"}, blocking=True,
    )
    await poll
    assert gateway.passive_state == "standby"
    first_command = next(i for i, f in enumerate(gateway.frames) if f.fc == 0x42)
    timeouts = [f for f in gateway.frames[:first_command] if not f.answered and f.fc != FC_HEARTBEAT]
    _LOGGER.warning(
        "Standby command queued behind %s timed-out requests (~%.0f s live)",
        len(timeouts), len(timeouts) * REAL_TIMEOUT,
    )
    assert len(timeouts) * REAL_TIMEOUT > 60
