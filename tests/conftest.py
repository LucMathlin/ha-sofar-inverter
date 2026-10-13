"""Shared fixtures: a fake ME3000SP gateway and a fast pymodbus client.

The integration builds its pymodbus clients with a 5 s timeout and pymodbus's
default 3 retries. Tests swap in the same client class with a short timeout so
that silence and retries can be exercised in milliseconds; ``REAL_TIMEOUT`` and
``FAST_TIMEOUT`` let a test translate what it measured back into real seconds.
"""

from __future__ import annotations

import asyncio
import traceback
from collections.abc import AsyncGenerator
from unittest.mock import patch

import pytest
from pymodbus.client import AsyncModbusTcpClient

from custom_components.sofar_inverter.const import (
    CONF_ENABLE_CLOCK_MONITOR,
    CONF_ENABLE_CONTROL,
    CONF_ENABLE_SCHEDULE,
    CONF_MAX_POWER,
    CONF_MODEL,
    CONF_NAME,
    CONF_SCAN_INTERVAL,
    CONF_SLAVE,
    DOMAIN,
    MODEL_ME3000,
)
from homeassistant.config_entries import ConfigEntryState
from homeassistant.const import CONF_HOST, CONF_PORT
from homeassistant.core import HomeAssistant
from homeassistant.util import dt as dt_util
from pytest_homeassistant_custom_component.common import MockConfigEntry

from .fake_sofar import FakeSofarGateway

REAL_TIMEOUT = 5.0  # what coordinator.py and config_flow.py pass to pymodbus
FAST_TIMEOUT = 0.05
CONNECT_TIMEOUT = 2.0
PYMODBUS_RETRIES = 3  # pymodbus default, left unchanged by the integration


@pytest.fixture(autouse=True)
def auto_enable_custom_integrations(enable_custom_integrations):
    """Let Home Assistant load custom_components/sofar_inverter."""
    yield


@pytest.fixture(autouse=True)
def fast_modbus_client():
    """Same client class and defaults (retries, reconnect back-off), shorter timeout."""

    class FastClient(AsyncModbusTcpClient):
        def __init__(self, *args, **kwargs):
            kwargs["timeout"] = FAST_TIMEOUT
            super().__init__(*args, **kwargs)

        async def connect(self) -> bool:
            # pymodbus uses one timeout for connecting and for each response.
            # Only response waits are shortened; a connect keeps a realistic
            # budget so a busy test loop cannot fail it spuriously.
            params = self.ctx.comm_params
            params.timeout_connect = CONNECT_TIMEOUT
            try:
                return await super().connect()
            finally:
                params.timeout_connect = FAST_TIMEOUT

    with patch("pymodbus.client.AsyncModbusTcpClient", FastClient):
        yield FastClient


def _is_accept_close_race(context: dict) -> bool:
    """asyncio accepting a connection just as the fake server closes.

    When a client connects at the moment the fake gateway stops (tests that
    stop it while the heartbeat retries every few milliseconds), asyncio's
    queued accept trips ``assert self._sockets is not None`` in Server._attach.
    Seen on Python 3.14.8. It is internal to the fake, not the integration.
    """
    exc = context.get("exception")
    if not isinstance(exc, AssertionError) or exc.__traceback__ is None:
        return False
    return any(frame.name == "_attach" for frame in traceback.extract_tb(exc.__traceback__))


@pytest.fixture
async def gateway(hass: HomeAssistant, socket_enabled) -> AsyncGenerator[FakeSofarGateway]:
    """A running fake Waveshare + ME3000SP on 127.0.0.1."""
    loop = asyncio.get_running_loop()
    previous = loop.get_exception_handler()

    def handler(_loop, context):
        if _is_accept_close_race(context):
            return
        if previous is not None:
            previous(_loop, context)
        else:
            _loop.default_exception_handler(context)

    loop.set_exception_handler(handler)
    fake = FakeSofarGateway(rtc=dt_util.now)
    await fake.start()
    try:
        yield fake
        await _unload_all(hass)  # before the fake goes, so nothing reconnects to it
        await fake.stop()
    finally:
        loop.set_exception_handler(previous)


def make_entry(gateway: FakeSofarGateway, **overrides) -> MockConfigEntry:
    """A config entry pointing at the fake gateway (ME3000, control off)."""
    data = {
        CONF_NAME: "Sofar Inverter",
        CONF_HOST: "127.0.0.1",
        CONF_PORT: gateway.port,
        CONF_SLAVE: gateway.slave,
        CONF_MODEL: MODEL_ME3000,
        CONF_SCAN_INTERVAL: 10,
        CONF_ENABLE_CONTROL: False,
        CONF_MAX_POWER: 3000,
        CONF_ENABLE_SCHEDULE: False,
        CONF_ENABLE_CLOCK_MONITOR: True,
    }
    data.update(overrides)
    return MockConfigEntry(
        domain=DOMAIN,
        title=data[CONF_NAME],
        data=data,
        unique_id=f"127.0.0.1_{gateway.port}_{data[CONF_SLAVE]}",
        version=2,
    )


async def setup_entry(hass: HomeAssistant, gateway: FakeSofarGateway, **overrides):
    """Add and set up an entry; return (entry, coordinator)."""
    entry = make_entry(gateway, **overrides)
    entry.add_to_hass(hass)
    assert await hass.config_entries.async_setup(entry.entry_id)
    await hass.async_block_till_done()
    return entry, hass.data[DOMAIN][entry.entry_id]


async def _unload_all(hass: HomeAssistant) -> None:
    """Unload every loaded entry so the heartbeat task is cancelled."""
    loaded = hass.data.get(DOMAIN, {})
    for entry in hass.config_entries.async_entries(DOMAIN):
        if entry.state is ConfigEntryState.LOADED and entry.entry_id in loaded:
            await hass.config_entries.async_unload(entry.entry_id)
    await hass.async_block_till_done()
