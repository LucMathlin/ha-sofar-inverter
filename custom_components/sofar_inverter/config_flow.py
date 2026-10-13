"""Config and options flow for the Sofar Inverter Modbus TCP integration."""
from __future__ import annotations

import logging
from typing import Any

import voluptuous as vol

from homeassistant import config_entries
from homeassistant.const import CONF_HOST, CONF_PORT
from homeassistant.core import callback
from homeassistant.helpers.selector import (
    BooleanSelector,
    NumberSelector,
    NumberSelectorConfig,
    NumberSelectorMode,
    SelectOptionDict,
    SelectSelector,
    SelectSelectorConfig,
    SelectSelectorMode,
    TextSelector,
)

from .const import (
    CONF_ENABLE_CLOCK_MONITOR,
    CONF_ENABLE_CONTROL,
    CONF_ENABLE_SCHEDULE,
    CONF_MAX_POWER,
    CONF_MODEL,
    CONF_NAME,
    CONF_SCAN_INTERVAL,
    CONF_SLAVE,
    DEFAULT_ENABLE_CLOCK_MONITOR,
    DEFAULT_ENABLE_SCHEDULE,
    DEFAULT_MAX_POWER,
    DEFAULT_PORT,
    DEFAULT_SCAN_INTERVAL,
    DEFAULT_SLAVE,
    DOMAIN,
    MODEL_CONFIG,
    MODEL_LABELS,
)

_LOGGER = logging.getLogger(__name__)

CURRENT_VERSION = 2


def _user_schema() -> vol.Schema:
    model_options = [
        SelectOptionDict(value=key, label=label) for key, label in MODEL_LABELS.items()
    ]
    return vol.Schema(
        {
            vol.Required(CONF_NAME, default="Sofar Inverter"): TextSelector(),
            vol.Required(CONF_HOST): TextSelector(),
            vol.Optional(CONF_PORT, default=DEFAULT_PORT): NumberSelector(
                NumberSelectorConfig(min=1, max=65535, step=1, mode=NumberSelectorMode.BOX)
            ),
            vol.Optional(CONF_SLAVE, default=DEFAULT_SLAVE): NumberSelector(
                NumberSelectorConfig(min=0, max=255, step=1, mode=NumberSelectorMode.BOX)
            ),
            vol.Required(CONF_MODEL, default=next(iter(MODEL_LABELS))): SelectSelector(
                SelectSelectorConfig(options=model_options, mode=SelectSelectorMode.DROPDOWN)
            ),
            vol.Optional(CONF_SCAN_INTERVAL, default=DEFAULT_SCAN_INTERVAL): NumberSelector(
                NumberSelectorConfig(min=2, max=300, step=1, mode=NumberSelectorMode.BOX)
            ),
            vol.Optional(CONF_ENABLE_CONTROL, default=False): BooleanSelector(),
            vol.Optional(CONF_MAX_POWER, default=DEFAULT_MAX_POWER): NumberSelector(
                NumberSelectorConfig(min=100, max=20000, step=100, mode=NumberSelectorMode.BOX)
            ),
            vol.Optional(
                CONF_ENABLE_SCHEDULE, default=DEFAULT_ENABLE_SCHEDULE
            ): BooleanSelector(),
            vol.Optional(
                CONF_ENABLE_CLOCK_MONITOR, default=DEFAULT_ENABLE_CLOCK_MONITOR
            ): BooleanSelector(),
        }
    )


def _options_schema(entry: config_entries.ConfigEntry) -> vol.Schema:
    """Options form, pre-filled with the values the entry is currently using.

    The connection details and model are deliberately absent: changing them would
    change the entry's identity, and the entry should be re-added instead.
    """

    def current(key: str, default):
        if key in entry.options:
            return entry.options[key]
        return entry.data.get(key, default)

    return vol.Schema(
        {
            vol.Optional(
                CONF_SCAN_INTERVAL,
                default=current(CONF_SCAN_INTERVAL, DEFAULT_SCAN_INTERVAL),
            ): NumberSelector(
                NumberSelectorConfig(min=2, max=300, step=1, mode=NumberSelectorMode.BOX)
            ),
            vol.Optional(
                CONF_ENABLE_CONTROL, default=current(CONF_ENABLE_CONTROL, False)
            ): BooleanSelector(),
            vol.Optional(
                CONF_MAX_POWER, default=current(CONF_MAX_POWER, DEFAULT_MAX_POWER)
            ): NumberSelector(
                NumberSelectorConfig(min=100, max=20000, step=100, mode=NumberSelectorMode.BOX)
            ),
            vol.Optional(
                CONF_ENABLE_SCHEDULE,
                default=current(CONF_ENABLE_SCHEDULE, DEFAULT_ENABLE_SCHEDULE),
            ): BooleanSelector(),
            vol.Optional(
                CONF_ENABLE_CLOCK_MONITOR,
                default=current(CONF_ENABLE_CLOCK_MONITOR, DEFAULT_ENABLE_CLOCK_MONITOR),
            ): BooleanSelector(),
        }
    )


async def _test_connection(host: str, port: int, slave: int, model: str) -> str | None:
    """Try to connect and read the running-state register. Returns an error key or None."""
    client = None
    try:
        from pymodbus.client import AsyncModbusTcpClient  # noqa: PLC0415

        client = AsyncModbusTcpClient(host=host, port=port, timeout=5)
        if not await client.connect():
            return "cannot_connect"

        # The running_state register is the first sensor for every model.
        state_reg = MODEL_CONFIG[model]["sensors"][0].register
        result = await client.read_holding_registers(
            address=state_reg, count=1, device_id=slave
        )

        # An exception reply (e.g. 0x0B from the gateway when nothing answers
        # this unit ID) has registers == [], not None, so check isError() too.
        if (
            result is None
            or (hasattr(result, "isError") and result.isError())
            or not getattr(result, "registers", None)
        ):
            return "invalid_slave"
        return None
    except ConnectionRefusedError:
        return "cannot_connect"
    except Exception as exc:  # noqa: BLE001
        _LOGGER.exception("Error testing Sofar connection to %s:%s: %s", host, port, exc)
        return "unknown"
    finally:
        # On every path: a client left open keeps a gateway TCP session and a
        # pymodbus reconnect task alive after the form has been answered.
        if client is not None:
            client.close()


class SofarInverterConfigFlow(config_entries.ConfigFlow, domain=DOMAIN):
    """Handle the config flow for Sofar Inverter."""

    VERSION = CURRENT_VERSION

    async def async_step_user(self, user_input: dict[str, Any] | None = None):
        """Single-step setup: connection + model + control options."""
        errors: dict[str, str] = {}

        if user_input is not None:
            host: str = user_input[CONF_HOST].strip()
            port: int = int(user_input[CONF_PORT])
            slave: int = int(user_input[CONF_SLAVE])
            model: str = user_input[CONF_MODEL]
            name: str = user_input[CONF_NAME].strip()

            await self.async_set_unique_id(f"{host}_{port}_{slave}")
            self._abort_if_unique_id_configured()

            error = await _test_connection(host, port, slave, model)
            if error:
                errors["base"] = error
            else:
                return self.async_create_entry(
                    title=name,
                    data={
                        CONF_NAME: name,
                        CONF_HOST: host,
                        CONF_PORT: port,
                        CONF_SLAVE: slave,
                        CONF_MODEL: model,
                        CONF_SCAN_INTERVAL: int(user_input[CONF_SCAN_INTERVAL]),
                        CONF_ENABLE_CONTROL: bool(user_input[CONF_ENABLE_CONTROL]),
                        CONF_MAX_POWER: int(user_input[CONF_MAX_POWER]),
                        CONF_ENABLE_SCHEDULE: bool(user_input[CONF_ENABLE_SCHEDULE]),
                        CONF_ENABLE_CLOCK_MONITOR: bool(
                            user_input[CONF_ENABLE_CLOCK_MONITOR]
                        ),
                    },
                )

        return self.async_show_form(
            step_id="user",
            data_schema=_user_schema(),
            errors=errors,
        )

    @staticmethod
    @callback
    def async_get_options_flow(
        entry: config_entries.ConfigEntry,
    ) -> SofarInverterOptionsFlow:
        """Return the options flow handler."""
        return SofarInverterOptionsFlow()


class SofarInverterOptionsFlow(config_entries.OptionsFlow):
    """Let the tunables be changed without re-adding the integration."""

    async def async_step_init(self, user_input: dict[str, Any] | None = None):
        """Show and store the options form."""
        if user_input is not None:
            return self.async_create_entry(
                data={
                    CONF_SCAN_INTERVAL: int(user_input[CONF_SCAN_INTERVAL]),
                    CONF_ENABLE_CONTROL: bool(user_input[CONF_ENABLE_CONTROL]),
                    CONF_MAX_POWER: int(user_input[CONF_MAX_POWER]),
                    CONF_ENABLE_SCHEDULE: bool(user_input[CONF_ENABLE_SCHEDULE]),
                    CONF_ENABLE_CLOCK_MONITOR: bool(user_input[CONF_ENABLE_CLOCK_MONITOR]),
                }
            )

        return self.async_show_form(
            step_id="init",
            data_schema=_options_schema(self.config_entry),
        )
