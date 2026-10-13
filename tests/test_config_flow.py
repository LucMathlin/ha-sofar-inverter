"""Config flow and options flow."""

from __future__ import annotations

from unittest.mock import patch

import pytest

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
from homeassistant.config_entries import SOURCE_USER
from homeassistant.const import CONF_HOST, CONF_PORT
from homeassistant.core import HomeAssistant
from homeassistant.data_entry_flow import FlowResultType

from .conftest import make_entry
from .fake_sofar import FC_READ_HOLDING, FakeSofarGateway


def _user_input(port: int, slave: int = 1) -> dict:
    return {
        CONF_NAME: "Sofar Inverter",
        CONF_HOST: " 127.0.0.1 ",
        CONF_PORT: port,
        CONF_SLAVE: slave,
        CONF_MODEL: MODEL_ME3000,
        CONF_SCAN_INTERVAL: 10,
        CONF_ENABLE_CONTROL: False,
        CONF_MAX_POWER: 3000,
        CONF_ENABLE_SCHEDULE: False,
        CONF_ENABLE_CLOCK_MONITOR: True,
    }


@pytest.fixture
def no_setup():
    """Keep the flow tests to the flow: do not start the integration."""
    with patch("custom_components.sofar_inverter.async_setup_entry", return_value=True):
        yield


async def _submit(hass: HomeAssistant, user_input: dict):
    result = await hass.config_entries.flow.async_init(DOMAIN, context={"source": SOURCE_USER})
    assert result["type"] is FlowResultType.FORM and result["step_id"] == "user"
    return await hass.config_entries.flow.async_configure(result["flow_id"], user_input)


async def test_user_flow_creates_entry(hass: HomeAssistant, gateway: FakeSofarGateway, no_setup) -> None:
    result = await _submit(hass, _user_input(gateway.port))
    assert result["type"] is FlowResultType.CREATE_ENTRY
    assert result["title"] == "Sofar Inverter"
    assert result["data"][CONF_HOST] == "127.0.0.1"  # stripped
    assert result["data"][CONF_MODEL] == MODEL_ME3000
    assert result["data"][CONF_ENABLE_CONTROL] is False
    entry = hass.config_entries.async_entries(DOMAIN)[0]
    assert entry.unique_id == f"127.0.0.1_{gateway.port}_1"
    # The connection test is a single read of the running-state register.
    assert [(f.fc, f.data.hex()) for f in gateway.frames] == [(FC_READ_HOLDING, "02000001")]


async def test_user_flow_cannot_connect(hass: HomeAssistant, gateway: FakeSofarGateway, no_setup) -> None:
    port = gateway.port
    await gateway.stop()
    result = await _submit(hass, _user_input(port))
    assert result["type"] is FlowResultType.FORM
    assert result["errors"] == {"base": "cannot_connect"}


@pytest.mark.parametrize("expected_lingering_tasks", [True])  # see the xfail below
async def test_wrong_unit_id_with_silent_gateway_is_unknown(
    hass: HomeAssistant, gateway: FakeSofarGateway, no_setup, expected_lingering_tasks
) -> None:
    """A wrong unit ID behind a gateway that stays silent surfaces as "unknown".

    pymodbus raises after timeout x (retries + 1), i.e. about 20 s live, and the
    flow maps the exception to the generic error rather than invalid_slave.
    """
    result = await _submit(hass, _user_input(gateway.port, slave=7))
    assert result["errors"] == {"base": "unknown"}
    assert len(gateway.frames) == 4  # first try + 3 retries


@pytest.mark.parametrize("expected_lingering_tasks", [True])
@pytest.mark.xfail(
    strict=True,
    reason="BUG: _test_connection only closes its client on the success path; after "
    "an exception the connection and a pymodbus reconnect task are left running",
)
async def test_connection_test_closes_its_client_on_error(
    hass: HomeAssistant, gateway: FakeSofarGateway, no_setup, fast_modbus_client, expected_lingering_tasks
) -> None:
    closed: list[bool] = []
    original = fast_modbus_client.close

    def tracking_close(self, *args, **kwargs):
        closed.append(True)
        return original(self, *args, **kwargs)

    with patch.object(fast_modbus_client, "close", tracking_close):
        await _submit(hass, _user_input(gateway.port, slave=7))
    assert closed


@pytest.mark.xfail(
    strict=True,
    reason="BUG: _test_connection never checks isError(); pymodbus 3.13's "
    "ExceptionResponse has registers == [], so an exception reply is accepted",
)
async def test_wrong_unit_id_with_gateway_exception_is_invalid_slave(
    hass: HomeAssistant, gateway: FakeSofarGateway, no_setup
) -> None:
    gateway.gateway_timeout_exception = True  # Waveshare answers 0x0B itself
    result = await _submit(hass, _user_input(gateway.port, slave=7))
    assert result["type"] is FlowResultType.FORM
    assert result["errors"] == {"base": "invalid_slave"}


async def test_exception_reply_currently_creates_an_entry(
    hass: HomeAssistant, gateway: FakeSofarGateway, no_setup
) -> None:
    """Pins today's behaviour behind the xfail above."""
    gateway.exception_code = 0x02
    result = await _submit(hass, _user_input(gateway.port))
    assert result["type"] is FlowResultType.CREATE_ENTRY


async def test_duplicate_is_aborted(hass: HomeAssistant, gateway: FakeSofarGateway, no_setup) -> None:
    make_entry(gateway).add_to_hass(hass)
    result = await _submit(hass, _user_input(gateway.port))
    assert result["type"] is FlowResultType.ABORT
    assert result["reason"] == "already_configured"
    assert gateway.frames == []  # aborted before touching the bus


async def test_options_flow(hass: HomeAssistant, gateway: FakeSofarGateway, no_setup) -> None:
    entry = make_entry(gateway)
    entry.add_to_hass(hass)
    result = await hass.config_entries.options.async_init(entry.entry_id)
    assert result["type"] is FlowResultType.FORM
    schema_keys = {str(k) for k in result["data_schema"].schema}
    assert CONF_HOST not in schema_keys and CONF_MODEL not in schema_keys
    result = await hass.config_entries.options.async_configure(
        result["flow_id"],
        {
            CONF_SCAN_INTERVAL: 15,
            CONF_ENABLE_CONTROL: True,
            CONF_MAX_POWER: 2500,
            CONF_ENABLE_SCHEDULE: False,
            CONF_ENABLE_CLOCK_MONITOR: False,
        },
    )
    assert result["type"] is FlowResultType.CREATE_ENTRY
    assert entry.options[CONF_ENABLE_CONTROL] is True
    assert entry.options[CONF_MAX_POWER] == 2500
    assert entry.options[CONF_ENABLE_CLOCK_MONITOR] is False


async def test_version_1_entry_migrates(hass: HomeAssistant, gateway: FakeSofarGateway) -> None:
    entry = make_entry(gateway)
    entry.add_to_hass(hass)
    hass.config_entries.async_update_entry(entry, version=1, options={})
    with patch("custom_components.sofar_inverter.async_setup_entry", return_value=True):
        assert await hass.config_entries.async_setup(entry.entry_id)
    assert entry.version == 2
    assert entry.options[CONF_SCAN_INTERVAL] == 10
    assert entry.options[CONF_ENABLE_CONTROL] is False
