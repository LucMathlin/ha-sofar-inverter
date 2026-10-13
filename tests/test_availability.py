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


async def test_silent_inverter_detected_within_a_minute(hass: HomeAssistant, gateway: FakeSofarGateway) -> None:
    _, coordinator = await setup_entry(hass, gateway)
    gateway.silent = True
    gateway.clear()
    elapsed = await _timed_refresh(coordinator)
    assert coordinator.last_update_success is False
    assert elapsed * SCALE < 60
    # One block read, first try + 3 retries: about 20 s live.
    assert len(_poll_requests(gateway)) == 4


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


async def test_block_reads_resume_after_recovery(hass: HomeAssistant, gateway: FakeSofarGateway) -> None:
    """An outage is not the block's fault: block reads come straight back."""
    _, coordinator = await setup_entry(hass, gateway)
    gateway.clear()
    await coordinator.async_refresh()
    healthy = len(_poll_requests(gateway))
    assert healthy == 4  # 0x0200+62, 0x0242+7, 0x10B0+13, 0x1200+1
    gateway.silent = True
    for _ in range(4):
        await coordinator.async_refresh()
    gateway.silent = False
    await coordinator.async_refresh()
    gateway.clear()
    await coordinator.async_refresh()
    assert len(_poll_requests(gateway)) == healthy
    assert not any(block.individual for block in coordinator._blocks)


async def test_refused_block_falls_back_then_is_retried(
    hass: HomeAssistant, gateway: FakeSofarGateway
) -> None:
    """A span the unit refuses while single registers still read falls back to
    per-register reads after three polls, and is tried as a block again later."""
    from custom_components.sofar_inverter.coordinator import BLOCK_RETRY_POLLS

    gateway.reject_span_ranges = [range(0x10B0, 0x10BD)]
    _, coordinator = await setup_entry(hass, gateway)
    for _ in range(3):
        await coordinator.async_refresh()
    block = next(b for b in coordinator._blocks if b.start == 0x10B0)
    assert block.individual is True
    assert coordinator.data["battery_capacity"] == 300  # still read, one by one

    gateway.reject_span_ranges = []
    for _ in range(BLOCK_RETRY_POLLS):
        await coordinator.async_refresh()
    assert block.individual is False
    gateway.clear()
    await coordinator.async_refresh()
    assert len(_poll_requests(gateway)) == 4


async def test_commands_do_not_wait_for_a_slow_poll(hass: HomeAssistant, gateway: FakeSofarGateway) -> None:
    """A Battery Mode change reaches the bus between blocks of a slow poll."""
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
    # At most one block's transaction (first try + 3 retries) ahead of it.
    assert len(timeouts) * REAL_TIMEOUT <= 20


async def test_silent_range_is_skipped_with_backoff(hass: HomeAssistant, gateway: FakeSofarGateway) -> None:
    """A range that never answers costs one transaction now and then, not every poll."""
    gateway.silent_ranges = [range(0x10B0, 0x10BD)]
    _, coordinator = await setup_entry(hass, gateway)
    gateway.clear()
    for _ in range(20):
        await coordinator.async_refresh()
        assert coordinator.last_update_success is True
        assert hass.states.get(SOC).state == "60.0"
    tries = [f for f in gateway.frames if f.fc in (0x03, 0x04) and f.data[:2] == b"\x10\xb0"]
    # Tried on polls 2, 5, 10 and 19 of these 20 (back-off 1, 2, 4, 8), four
    # attempts each, instead of every poll.
    assert len(tries) <= 4 * 4
    assert hass.states.get("sensor.sofar_inverter_battery_soc").state == "60.0"
