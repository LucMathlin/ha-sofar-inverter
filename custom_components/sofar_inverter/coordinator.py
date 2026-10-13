"""Modbus TCP polling/control coordinator for Sofar inverters.

Register maps and scaling are ported from the Sofar2mqtt ESP firmware; the
control, fault, schedule and clock handling follows the official Sofar
"ModBus-RTU Protocol" document for HYD / ME3000SP.

Two control protocols are supported:

  * ME3000 / HYD-ES : the proprietary Sofar "passive mode" function code 0x42.
    This is NOT a standard Modbus function, so it is implemented with a custom
    pymodbus PDU (SofarPassiveModeRequest / Response). The response carries an
    acceptance/rejection status word which is decoded, so a rejected command is
    never reported as success.

  * HYD-EP/KTL      : a standard write-multiple-registers (FC 0x10) to the
    passive control register 0x1187, writing a 6x16-bit payload.

All coordinators that share the same host:port (i.e. the same Modbus TCP /
RS485 gateway) share a single pymodbus client and a single asyncio.Lock, so
polls, commands and heartbeats from different slave IDs are serialised onto the
one RS485 bus behind the gateway. This avoids transaction-id collisions where one
coordinator reads back another coordinator's response. The shared client and the
per-gateway heartbeat task are reference-counted and torn down when the last
coordinator for that gateway unloads.

Reads are batched: contiguous (and near-contiguous) registers are fetched in one
transaction instead of one transaction per entity, which matters because the bus
behind the gateway runs at 9600 baud. A block that the inverter refuses falls
back to per-register reads so one bad address cannot blank a dozen sensors.
"""
from __future__ import annotations

import asyncio
import logging
import struct
from dataclasses import dataclass, field
from datetime import datetime, timedelta

from pymodbus.pdu import ModbusPDU

from homeassistant.core import HomeAssistant
from homeassistant.helpers.update_coordinator import DataUpdateCoordinator, UpdateFailed
from homeassistant.util import dt as dt_util

from .const import (
    ALARM_BITS,
    BATTERY_FAULT_BITS,
    BATTERY_TYPE_MAP,
    CLOCK_DRIFT_WARN_SECONDS,
    CLOCK_POLL_INTERVAL,
    COMM_BOARD_BITS,
    COUNTRY_CODE_MAP,
    DOMAIN,
    EPS_RUNSTATES,
    FAULT_BITS,
    FN_BROADCAST_SET_TIME,
    FN_READ_DEVICE_INFO_EXT,
    FN_READ_EEPROM,
    FN_READ_HOLDING,
    FN_READ_INPUT,
    FN_WRITE_REGISTERS,
    MAX_BLOCK_GAP,
    MAX_BLOCK_REGISTERS,
    MODE_AUTO,
    MODE_BATTERY_SAVE,
    MODE_CHARGE,
    MODE_DISCHARGE,
    MODE_LABELS,
    MODE_NONE,
    MODE_STANDBY,
    MODEL_CONFIG,
    MODEL_HYD_EP_KTL,
    MODEL_HYD_ES,
    MODEL_ME3000,
    PASSIVE_FLAG_BATTERY_EMPTY,
    PASSIVE_FLAG_BATTERY_FULL,
    PASSIVE_FLAG_CHARGE_READY,
    PASSIVE_FLAG_DISCHARGE_READY,
    PASSIVE_STATUS_ACCEPTED,
    PASSIVE_STATUS_MAP,
    PROTO_PASSIVE_FN42,
    PROTO_PASSIVE_REG,
    RATED_POWER_BY_FAMILY,
    REG_DEVICE_INFO,
    REG_EVENT_RECORD,
    REG_SET_TIME,
    REG_SYSTEM_TIME,
    REG_TIMING_BLOCK,
    REG_TOU_BLOCK,
    REG_TOU_RULE_NUMBER,
    SCHEDULE_REGISTERS,
    SENSORS_HYD_EP_KTL_BATTERY2,
    SOFAR2_REG_PASSIVECONTROL,
    SOFAR_BROADCAST_ADDRESS,
    SOFAR_FN_AUTO,
    SOFAR_FN_CHARGE,
    SOFAR_FN_DISCHARGE,
    SOFAR_FN_STANDBY,
    SOFAR_PARAM_AUTO,
    SOFAR_PARAM_STANDBY,
    WORKING_MODE_MAP,
    WORKING_MODE_PASSIVE,
)

_LOGGER = logging.getLogger(__name__)

# Sofar proprietary passive-mode function code (not standard Modbus).
SOFAR_FN_PASSIVEMODE = 0x42

# Sofar passive-mode heartbeat (FC 0x49). ME3000 / HYD-ES require this to stay
# responsive in passive mode; the firmware skips it for HYDV2.
SOFAR_FN_HEARTBEAT = 0x49
# The inverter drops to standby after 60s without a heartbeat, and leaving
# standby is rate-limited by the grid-code power ramp, so a brief comms glitch
# would cost minutes of reduced output. Send well inside the window and retry
# quickly on failure rather than waiting a whole interval.
HEARTBEAT_INTERVAL = 9  # seconds, fixed (not user-configurable)
HEARTBEAT_RETRY_DELAY = 2  # seconds, after a failed send
HEARTBEAT_MAX_FAST_RETRIES = 3

# A block that fell back to per-register reads is tried as a block again after
# this many polls (about 5 minutes at the default 10 s interval).
BLOCK_RETRY_POLLS = 30

# A block that gets no reply at all while other blocks answer is skipped for
# 1, 2, 4 ... polls, up to this many, so one silent range costs one timed-out
# transaction now and then instead of one per register on every poll.
BLOCK_SILENT_MAX_SKIP = 32

# Standard Modbus exception codes, for readable logging of a 0x8x response.
MODBUS_EXCEPTIONS = {
    1: "Illegal function",
    2: "Illegal data address",
    3: "Illegal data value",
    4: "Slave device failure",
    5: "Acknowledge",
    6: "Slave device busy",
    8: "Memory parity error",
    10: "Gateway path unavailable",
    11: "Gateway target device failed to respond",
}

# Home Assistant truncates states at 255 characters.
MAX_STATE_LENGTH = 250

# Which (byte, bit) positions of each raw fault word the protocol document
# describes. Anything set outside this set is surfaced as "Unknown" rather than
# being silently dropped.
_DOCUMENTED_BITS: dict[str, set[tuple[int, int]]] = {}
# Fault ID -> definition, used to name entries read back from the event log.
_FAULT_BY_ID: dict[int, object] = {}
for _table in (FAULT_BITS, ALARM_BITS, BATTERY_FAULT_BITS, COMM_BOARD_BITS):
    for _fb in _table:
        _DOCUMENTED_BITS.setdefault(_fb.source_key, set()).add((_fb.byte, _fb.bit))
        if _fb.fault_id is not None:
            _FAULT_BY_ID.setdefault(_fb.fault_id, _fb)


# ---------------------------------------------------------------- BCD helpers
def _bcd_to_int(value: int) -> int | None:
    """Decode one packed-BCD byte, or None if it is not valid BCD."""
    high, low = (value >> 4) & 0x0F, value & 0x0F
    if high > 9 or low > 9:
        return None
    return high * 10 + low


def _int_to_bcd(value: int) -> int:
    """Encode 0-99 as one packed-BCD byte."""
    value = max(0, min(99, int(value)))
    return ((value // 10) << 4) | (value % 10)


def _registers_to_ascii(registers: list[int]) -> str:
    """Decode registers to ASCII, high byte first (protocol doc 2.3.4)."""
    raw = bytearray()
    for register in registers:
        raw.append((register >> 8) & 0xFF)
        raw.append(register & 0xFF)
    return raw.decode("ascii", errors="ignore").strip("\x00 \t\r\n")


# ==========================================================================
# Custom PDUs
# ==========================================================================
class SofarPassiveModeResponse(ModbusPDU):
    """Response PDU for the Sofar proprietary passive-mode command (0x42).

    The inverter echoes a 2-byte data payload holding a status word: the low
    byte is the acceptance/rejection code and the high byte carries battery
    readiness flags (protocol doc p.62).
    """

    function_code = SOFAR_FN_PASSIVEMODE
    rtu_byte_count_pos = 2

    def __init__(self, values=None, slave=1, transaction=0):
        super().__init__(dev_id=slave, transaction_id=transaction)
        self.values = values or []

    def encode(self) -> bytes:
        res = struct.pack(">B", len(self.values) * 2)
        for register in self.values:
            res += struct.pack(">H", register)
        return res

    def decode(self, data: bytes) -> None:
        byte_count = int(data[0])
        self.values = []
        for i in range(1, byte_count + 1, 2):
            self.values.append(struct.unpack(">H", data[i : i + 2])[0])


class SofarPassiveModeRequest(ModbusPDU):
    """Request PDU for the Sofar proprietary passive-mode command (0x42).

    Frame body is: <function code 0x42><cmd:16><param:16> exactly as the
    firmware's sendPassiveCmd() constructs it.
    """

    function_code = SOFAR_FN_PASSIVEMODE
    rtu_frame_size = 8

    def __init__(self, cmd: int = 0, param: int = 0, slave: int = 1, transaction: int = 0):
        super().__init__(dev_id=slave, transaction_id=transaction)
        self.cmd = cmd
        self.param = param

    def encode(self) -> bytes:
        return struct.pack(">HH", self.cmd, self.param & 0xFFFF)

    def decode(self, data: bytes) -> None:
        self.cmd, self.param = struct.unpack(">HH", data)

    def get_response_pdu_size(self) -> int:
        # 1 byte count + 2 data bytes
        return 1 + 1 + 2


class SofarHeartbeatRequest(ModbusPDU):
    """Request PDU for the Sofar passive-mode heartbeat (FC 0x49).

    The protocol document gives the complete example frame for slave 1 as
    ``01 49 22 01 22 02 1E DD``. CRC-16/Modbus over ``01 49 22 01 22 02`` is
    exactly ``1E DD``, which confirms the data payload is the four bytes
    ``22 01 22 02`` and that the frame is 8 bytes long. The firmware this
    integration was ported from appended two extra zero bytes, producing a
    different (CRC ``C9 F9``) frame rather than harmless padding.

    Sent fire-and-forget; the inverter's reply is not needed.
    """

    function_code = SOFAR_FN_HEARTBEAT
    rtu_frame_size = 8  # id + fc + 4 data + 2 crc

    def __init__(self, slave: int = 1, transaction: int = 0):
        super().__init__(dev_id=slave, transaction_id=transaction)

    def encode(self) -> bytes:
        return bytes([0x22, 0x01, 0x22, 0x02])

    def decode(self, data: bytes) -> None:
        pass

    def get_response_pdu_size(self) -> int:
        return 0


class SofarHeartbeatResponse(ModbusPDU):
    """Decoder for a reply to the FC 0x49 heartbeat, if the inverter sends one.

    The heartbeat is fire-and-forget, but without a registered decoder an
    answered heartbeat could not be decoded: pymodbus then raises inside
    data_received() and asyncio tears down the shared connection. The reply's
    content is not used, so any payload is accepted and kept only for logging.
    """

    function_code = SOFAR_FN_HEARTBEAT
    rtu_frame_size = 8  # id + fc + 4 data + 2 crc, mirroring the request

    def __init__(self, slave=1, transaction=0):
        super().__init__(dev_id=slave, transaction_id=transaction)
        self.raw = b""

    def encode(self) -> bytes:
        return self.raw

    def decode(self, data: bytes) -> None:
        self.raw = bytes(data)


class SofarReadTimeRequest(ModbusPDU):
    """Read the inverter RTC (protocol doc 2.11): FC 0x10, address 0x8000.

    Sofar reuses function code 0x10 — normally Write Multiple Registers — for
    this read, with the register count field set to zero. Registering a response
    decoder for 0x10 on the shared client would break the standard
    write-registers path used for schedule and HYD-EP/KTL passive control, so
    this is only ever issued on a temporary, dedicated connection.
    """

    function_code = FN_WRITE_REGISTERS
    rtu_frame_size = 8

    def __init__(self, address: int = REG_SYSTEM_TIME, slave: int = 1, transaction: int = 0):
        super().__init__(dev_id=slave, transaction_id=transaction)
        self.address = address

    def encode(self) -> bytes:
        return struct.pack(">HH", self.address, 0x0000)

    def decode(self, data: bytes) -> None:
        self.address, _ = struct.unpack(">HH", data[:4])

    def get_response_pdu_size(self) -> int:
        # 1 byte count + 7 BCD bytes; a 2-byte count variant is tolerated by
        # the response decoder below.
        return 1 + 1 + 7


class SofarReadTimeResponse(ModbusPDU):
    """Response holding 7 packed-BCD bytes: second, minute, hour, weekday,
    day, month, year.

    The document's response table is ambiguous about whether the byte-count
    field is one or two bytes wide, so the raw payload is kept and the caller
    picks the offset that yields a valid date.
    """

    function_code = FN_WRITE_REGISTERS
    rtu_byte_count_pos = 2

    def __init__(self, slave=1, transaction=0):
        super().__init__(dev_id=slave, transaction_id=transaction)
        self.raw = b""

    def encode(self) -> bytes:
        return bytes([len(self.raw)]) + self.raw

    def decode(self, data: bytes) -> None:
        self.raw = bytes(data)


class SofarSetTimeRequest(ModbusPDU):
    """Broadcast the wall clock to every inverter (protocol doc 2.1.1).

    Slave address 0x88, FC 0x02, register 0x5000, three registers carrying six
    packed-BCD bytes: second, minute, hour, day, month, year. Broadcasts are
    accepted but never answered, so this is sent fire-and-forget.
    """

    function_code = FN_BROADCAST_SET_TIME
    rtu_frame_size = 13

    def __init__(self, when: datetime, slave: int = 0x88, transaction: int = 0):
        super().__init__(dev_id=slave, transaction_id=transaction)
        self.when = when

    def encode(self) -> bytes:
        when = self.when
        return struct.pack(">HH", REG_SET_TIME, 0x0003) + bytes(
            [
                _int_to_bcd(when.second),
                _int_to_bcd(when.minute),
                _int_to_bcd(when.hour),
                _int_to_bcd(when.day),
                _int_to_bcd(when.month),
                _int_to_bcd(when.year % 100),
            ]
        )

    def decode(self, data: bytes) -> None:
        pass

    def get_response_pdu_size(self) -> int:
        return 0


class SofarExtReadRequest(ModbusPDU):
    """Extended read (FC 0x21) for the 0x2000-0x20FF production-info range.

    Table 1.4 assigns this range to ext code 0x21, but most units also answer a
    plain FC 0x03, which is tried first. 0x21 is not a standard Modbus function
    so registering a decoder for it cannot disturb anything else.
    """

    function_code = FN_READ_DEVICE_INFO_EXT
    rtu_frame_size = 8

    def __init__(self, address: int = 0, count: int = 1, slave: int = 1, transaction: int = 0):
        super().__init__(dev_id=slave, transaction_id=transaction)
        self.address = address
        self.count = count

    def encode(self) -> bytes:
        return struct.pack(">HH", self.address, self.count)

    def decode(self, data: bytes) -> None:
        self.address, self.count = struct.unpack(">HH", data[:4])

    def get_response_pdu_size(self) -> int:
        return 1 + 1 + 2 * self.count


class SofarExtReadResponse(ModbusPDU):
    """Byte-count-prefixed register payload for FC 0x21."""

    function_code = FN_READ_DEVICE_INFO_EXT
    rtu_byte_count_pos = 2

    def __init__(self, slave=1, transaction=0):
        super().__init__(dev_id=slave, transaction_id=transaction)
        self.registers: list[int] = []

    def encode(self) -> bytes:
        res = struct.pack(">B", len(self.registers) * 2)
        for register in self.registers:
            res += struct.pack(">H", register)
        return res

    def decode(self, data: bytes) -> None:
        byte_count = int(data[0])
        self.registers = []
        for i in range(1, min(byte_count, len(data) - 1) + 1, 2):
            if i + 1 < len(data):
                self.registers.append(struct.unpack(">H", data[i : i + 2])[0])


class SofarEventLogRequest(ModbusPDU):
    """Read the Nth most recent event record (FC 0x50, address 0x6005)."""

    function_code = FN_READ_EEPROM
    rtu_frame_size = 8

    def __init__(self, index: int = 1, slave: int = 1, transaction: int = 0):
        super().__init__(dev_id=slave, transaction_id=transaction)
        self.index = index

    def encode(self) -> bytes:
        return struct.pack(">HH", REG_EVENT_RECORD, self.index)

    def decode(self, data: bytes) -> None:
        _, self.index = struct.unpack(">HH", data[:4])

    def get_response_pdu_size(self) -> int:
        return 1 + 1 + 8


class SofarEventLogResponse(ModbusPDU):
    """Byte-count-prefixed event record payload for FC 0x50."""

    function_code = FN_READ_EEPROM
    rtu_byte_count_pos = 2

    def __init__(self, slave=1, transaction=0):
        super().__init__(dev_id=slave, transaction_id=transaction)
        self.raw = b""

    def encode(self) -> bytes:
        return bytes([len(self.raw)]) + self.raw

    def decode(self, data: bytes) -> None:
        byte_count = int(data[0]) if data else 0
        self.raw = bytes(data[1 : 1 + byte_count]) if byte_count else b""


# ==========================================================================
# Command result
# ==========================================================================
@dataclass(slots=True)
class CommandResult:
    """Outcome of a control command, including the inverter's own verdict."""

    ok: bool
    message: str
    status_code: int | None = None
    flags: dict[str, bool] = field(default_factory=dict)

    @property
    def summary(self) -> str:
        return self.message


# ==========================================================================
# Read batching
# ==========================================================================
@dataclass
class ReadBlock:
    """A contiguous span of registers fetched in a single Modbus transaction."""

    start: int
    end: int
    sensors: list = field(default_factory=list)
    func: int | None = None
    failures: int = 0
    individual: bool = False
    polls_since_fallback: int = 0
    # Consecutive polls in which the block got no reply at all, and how many
    # more polls to skip it for (exponential back-off, see BLOCK_SILENT_MAX_SKIP).
    silent_polls: int = 0
    skip_polls: int = 0

    @property
    def count(self) -> int:
        return self.end - self.start + 1

    def func_order(self) -> tuple[int, ...]:
        """Function codes to try, best-known first.

        Table 1.4 assigns 0x0000-0x02FF to FC 0x03 and 0x1000-0x10FF to FC 0x04,
        but in practice most units answer FC 0x03 everywhere, which is what the
        original firmware relied on. Try the last code that worked, then the
        other one.
        """
        if self.func is not None:
            return (self.func,)
        if self.start >= 0x1000:
            return (FN_READ_HOLDING, FN_READ_INPUT)
        return (FN_READ_HOLDING, FN_READ_INPUT)


def build_read_blocks(
    sensors,
    max_registers: int = MAX_BLOCK_REGISTERS,
    max_gap: int = MAX_BLOCK_GAP,
) -> list[ReadBlock]:
    """Group sensors into as few Modbus reads as possible.

    At 9600 baud a register of payload costs roughly 2ms while each extra
    transaction costs tens of ms, so bridging a small gap of unused registers is
    cheaper than issuing another request. Large gaps are not bridged, which keeps
    the separate register banks (0x02xx, 0x04xx, 0x10xx, 0x12xx) apart.
    """
    ordered = sorted(sensors, key=lambda s: (s.register, s.count))
    blocks: list[ReadBlock] = []
    current: ReadBlock | None = None

    for sensor in ordered:
        sensor_end = sensor.register + sensor.count - 1
        if current is None:
            current = ReadBlock(start=sensor.register, end=sensor_end, sensors=[sensor])
            continue

        new_end = max(current.end, sensor_end)
        gap = sensor.register - current.end - 1
        if gap <= max_gap and (new_end - current.start + 1) <= max_registers:
            current.end = new_end
            current.sensors.append(sensor)
        else:
            blocks.append(current)
            current = ReadBlock(start=sensor.register, end=sensor_end, sensors=[sensor])

    if current is not None:
        blocks.append(current)
    return blocks


# ==========================================================================
# Shared gateway connection
# ==========================================================================
class _GatewayConnection:
    """Shared pymodbus client + lock + heartbeat for one gateway (host:port).

    A single RS485 bus sits behind the Modbus TCP gateway, so every coordinator
    that targets this gateway (regardless of slave ID) must serialise its bus
    access through one lock and one client. Reference-counted by config-entry
    id: the client and heartbeat are created on first use and closed when the
    last coordinator for this gateway unloads.
    """

    def __init__(self, hass: HomeAssistant, host: str, port: int) -> None:
        self.hass = hass
        self.host = host
        self.port = port
        self.lock = asyncio.Lock()
        self._users: set[str] = set()
        self._client = None
        self._heartbeat_task = None
        # Every slave that needs the passive-mode heartbeat. The protocol allows
        # broadcasting heartbeats to address 0x88, but a unit or gateway that
        # ignores broadcasts would silently fall to standby after 60s, so each
        # slave is addressed individually instead. With a single inverter this is
        # identical to the previous behaviour.
        self._heartbeat_slaves: set[int] = set()
        self.heartbeat_failures = 0

    # -- reference counting --------------------------------------------------
    def acquire(self, user_id: str) -> None:
        """Register a coordinator (by entry id) as a user of this gateway."""
        self._users.add(user_id)

    def release(self, user_id: str) -> bool:
        """Drop a user; return True when no users remain (safe to close)."""
        self._users.discard(user_id)
        return not self._users

    # -- shared client -------------------------------------------------------
    def _build_client(self):
        from pymodbus.client import AsyncModbusTcpClient  # noqa: PLC0415

        return AsyncModbusTcpClient(host=self.host, port=self.port, timeout=5)

    @staticmethod
    def _register_pdus(client) -> None:
        """Register the proprietary response decoders on the client.

        Only function codes that are not part of standard Modbus are registered
        here. In particular FC 0x10 is deliberately excluded: Sofar reuses it for
        "read time", but pymodbus needs its standard Write Multiple Registers
        decoder for schedule writes and HYD-EP/KTL passive control.
        """
        for pdu in (
            SofarPassiveModeResponse,
            SofarHeartbeatResponse,
            SofarExtReadResponse,
            SofarEventLogResponse,
        ):
            try:
                client.register(pdu)
            except Exception as exc:  # noqa: BLE001
                _LOGGER.debug("Could not register custom PDU %s: %s", pdu.__name__, exc)

    async def async_get_client(self):
        """Return a connected shared client. Must be called holding self.lock."""
        if self._client is None:
            self._client = self._build_client()
        if not self._client.connected:
            if not await self._client.connect():
                raise ConnectionError(f"Cannot connect to {self.host}:{self.port}")
            # Re-register the custom decoders on every (re)connect. A dropped and
            # rebuilt connection can reset the framer/decoder, which would
            # otherwise silently break ME3000/HYD-ES commands after a reconnect.
            self._register_pdus(self._client)
        return self._client

    def _close_shared_client(self) -> None:
        if self._client is not None:
            try:
                self._client.close()
            except Exception as exc:  # noqa: BLE001
                _LOGGER.debug("Error closing shared client: %s", exc)
            self._client = None

    async def async_isolated_read(self, request_factory, *pdu_classes):
        """Run one request on a dedicated connection with custom decoders.

        Used only for the FC 0x10 clock read, whose decoder would otherwise
        shadow the standard Write Multiple Registers response. The shared client
        is closed first so that gateways which accept only one TCP session at a
        time still work; it reconnects lazily on the next poll.

        Must be called holding self.lock. Returns the response, or None.
        """
        self._close_shared_client()
        client = self._build_client()
        try:
            if not await client.connect():
                raise ConnectionError(f"Cannot connect to {self.host}:{self.port}")
            for pdu in pdu_classes:
                try:
                    client.register(pdu)
                except Exception as exc:  # noqa: BLE001
                    _LOGGER.debug("Could not register %s: %s", pdu.__name__, exc)
            return await client.execute(False, request_factory())
        finally:
            try:
                client.close()
            except Exception as exc:  # noqa: BLE001
                _LOGGER.debug("Error closing isolated client: %s", exc)

    # -- heartbeat (one task per gateway, one frame per slave) ----------------
    def want_heartbeat(self, slave: int) -> None:
        """Mark that this slave needs the passive-mode heartbeat."""
        self._heartbeat_slaves.add(slave)

    def ensure_heartbeat(self) -> None:
        """Start the single per-gateway heartbeat task once."""
        if self._heartbeat_slaves and self._heartbeat_task is None:
            self._heartbeat_task = self.hass.async_create_background_task(
                self._heartbeat_loop(),
                name=f"{DOMAIN}_heartbeat_{self.host}_{self.port}",
            )

    async def _send_heartbeats(self) -> bool:
        """Send one heartbeat per registered slave. True if all succeeded."""
        async with self.lock:
            client = await self.async_get_client()
            for slave in sorted(self._heartbeat_slaves):
                # Fire-and-forget: the inverter's reply is not needed.
                await client.execute(True, SofarHeartbeatRequest(slave=slave))
        return True

    async def _heartbeat_loop(self) -> None:
        """Keep every passive-mode slave alive on the shared bus.

        Losing the heartbeat for 60s drops the inverter to standby, and coming
        back out of standby is rate-limited by the grid-code ramp, so a failed
        send is retried within seconds instead of waiting a full interval.
        """
        while True:
            await asyncio.sleep(HEARTBEAT_INTERVAL)
            for attempt in range(HEARTBEAT_MAX_FAST_RETRIES + 1):
                try:
                    await self._send_heartbeats()
                    if self.heartbeat_failures:
                        _LOGGER.info(
                            "Heartbeat to %s:%s recovered after %s failure(s)",
                            self.host, self.port, self.heartbeat_failures,
                        )
                    self.heartbeat_failures = 0
                    break
                except asyncio.CancelledError:
                    raise
                except Exception as exc:  # noqa: BLE001
                    self.heartbeat_failures += 1
                    if attempt >= HEARTBEAT_MAX_FAST_RETRIES:
                        _LOGGER.warning(
                            "Heartbeat send failed for %s:%s (%s consecutive): %s",
                            self.host, self.port, self.heartbeat_failures, exc,
                        )
                    else:
                        _LOGGER.debug(
                            "Heartbeat retry %s/%s for %s:%s: %s",
                            attempt + 1, HEARTBEAT_MAX_FAST_RETRIES,
                            self.host, self.port, exc,
                        )
                        await asyncio.sleep(HEARTBEAT_RETRY_DELAY)

    async def async_close(self) -> None:
        """Cancel the heartbeat and close the shared client (last user gone)."""
        if self._heartbeat_task is not None:
            self._heartbeat_task.cancel()
            self._heartbeat_task = None
        self._close_shared_client()


def _get_gateway(hass: HomeAssistant, host: str, port: int) -> _GatewayConnection:
    """Return the shared gateway connection for host:port, creating if needed."""
    registry = hass.data.setdefault(DOMAIN, {}).setdefault("_gateways", {})
    key = (host, port)
    gateway = registry.get(key)
    if gateway is None:
        gateway = _GatewayConnection(hass, host, port)
        registry[key] = gateway
    return gateway


# Register order within each atomically-written schedule block.
TIMING_BLOCK_KEYS = (
    "sched_charge_start",
    "sched_charge_end",
    "sched_discharge_start",
    "sched_discharge_end",
    "sched_charge_power",
    "sched_discharge_power",
)
TOU_BLOCK_KEYS = (
    "tou_charge_start",
    "tou_charge_end",
    "tou_soc",
    "tou_power",
    "tou_start_date",
    "tou_end_date",
    "tou_weekdays",
    "tou_enabled",
)

# Raw fault words feeding each decoded group.
_FAULT_SOURCES = ("fault_word_1", "fault_word_2", "fault_word_3", "fault_word_4", "fault_word_5")
_ALARM_SOURCES = ("alarm_word",)
_BATTERY_FAULT_SOURCES = ("battery_fault_word_1",)
_COMM_SOURCES = ("comm_board_word",)

# How often to re-read the inverter's own event log.
EVENT_LOG_INTERVAL = 300  # seconds


def _decode_bits(results: dict, table) -> list[dict]:
    """Return the documented faults whose bit is set."""
    active: list[dict] = []
    for fault in table:
        raw = results.get(fault.source_key)
        if raw is None:
            continue
        byte = (int(raw) >> (8 * fault.byte)) & 0xFF
        if byte & (1 << fault.bit):
            active.append(
                {
                    "id": fault.fault_id,
                    "code": fault.code,
                    "description": fault.description,
                }
            )
    return active


def _decode_unknown_bits(results: dict, source_keys) -> list[dict]:
    """Return set bits the protocol document does not describe.

    Reserved-but-set bits are surfaced rather than dropped, so an unexpected
    fault on a firmware revision newer than the document is still visible.
    """
    unknown: list[dict] = []
    for key in source_keys:
        raw = results.get(key)
        if raw is None:
            continue
        documented = _DOCUMENTED_BITS.get(key, set())
        for byte_index in (0, 1):
            byte = (int(raw) >> (8 * byte_index)) & 0xFF
            for bit in range(8):
                if byte & (1 << bit) and (byte_index, bit) not in documented:
                    unknown.append(
                        {
                            "id": None,
                            "code": f"unknown_{key}_b{byte_index}_{bit}",
                            "description": (
                                f"Undocumented bit set ({key} byte{byte_index} bit{bit})"
                            ),
                        }
                    )
    return unknown


def _format_fault_list(items: list[dict]) -> str:
    """Render a fault list as a state string, respecting HA's length limit."""
    if not items:
        return "None"
    parts = []
    for item in items:
        if item["id"] is not None:
            parts.append(f"ID{item['id']:02d} {item['description']}")
        else:
            parts.append(item["description"])
    text = ", ".join(parts)
    if len(text) > MAX_STATE_LENGTH:
        text = text[: MAX_STATE_LENGTH - 1].rstrip(", ") + "…"
    return text


class SofarCoordinator(DataUpdateCoordinator):
    """Polls a Sofar inverter over Modbus TCP and issues control commands."""

    def __init__(
        self,
        hass: HomeAssistant,
        host: str,
        port: int,
        slave: int,
        model: str,
        max_power: int,
        update_interval: timedelta,
        enable_control: bool = False,
        enable_schedule: bool = False,
        enable_clock_monitor: bool = True,
    ) -> None:
        self.host = host
        self.port = port
        self.slave = slave
        self.model = model
        self.max_power = max_power
        self.enable_control = enable_control
        self._send_heartbeat = model in (MODEL_ME3000, MODEL_HYD_ES)

        # Share one client + lock + heartbeat per gateway (host:port).
        self._gateway = _get_gateway(hass, host, port)
        self._lock = self._gateway.lock
        if self._send_heartbeat:
            self._gateway.want_heartbeat(slave)

        model_cfg = MODEL_CONFIG[model]
        self.sensors = list(model_cfg["sensors"])
        self.protocol = model_cfg["protocol"]
        self.runstate_map = model_cfg["runstate"]
        self.supports_faults: bool = model_cfg["faults"]
        self.working_mode_register: int = model_cfg["working_mode_register"]
        self.supports_schedule: bool = bool(model_cfg["schedule"]) and enable_schedule
        self.supports_clock: bool = bool(model_cfg["clock"]) and enable_clock_monitor
        self.supports_working_mode_write: bool = bool(model_cfg["schedule"])

        self._battery2_checked = False
        self._battery2_attempts = 0
        # Set by _read_registers: the last failed read got no reply at all.
        self._no_response = False

        # Device identification, filled in best-effort on first poll.
        self.product_code: str | None = None
        self.serial_number: str | None = None
        self.arm_version: str | None = None
        self.hardware_version: str | None = None
        self.rated_power: int | None = None
        self._firmware_int: int | None = None
        self._device_info_attempts = 0
        self._device_info_done = False
        self._rated_power_warned = False

        # Active control state. Re-applied on each poll so passive-mode
        # setpoints stay latched (matching the firmware's periodic re-send).
        self.active_mode: str = MODE_NONE
        self.active_power: int = 0
        self.effective_mode: str = MODE_NONE
        self.last_command_result: str | None = None
        self.last_command_at: datetime | None = None
        self.passive_flags: dict[str, bool] = {}
        self._failsafe_done = False
        self._failsafe_attempts = 0

        # Clock / event-log bookkeeping.
        self.inverter_clock: datetime | None = None
        self.clock_drift: int | None = None
        self._clock_next_read = 0.0
        self.last_event: str | None = None
        self.last_event_attrs: dict | None = None
        self._event_next_read = 0.0

        # Pending schedule values written but not yet confirmed by a re-read.
        self._pending_schedule: dict[str, int] = {}

        self._rebuild_blocks()

        super().__init__(
            hass,
            _LOGGER,
            name=f"{DOMAIN}_{host}_{slave}",
            update_interval=update_interval,
        )

        # Register this entry as a user of the shared gateway. Keyed by entry
        # id so it is idempotent across ConfigEntryNotReady retries (which
        # rebuild the coordinator but reuse the same entry id).
        entry = getattr(self, "config_entry", None)
        self._gateway_user = entry.entry_id if entry is not None else f"obj-{id(self)}"
        self._gateway.acquire(self._gateway_user)

    # ------------------------------------------------------------- entity sets
    @property
    def derived_keys(self) -> list[str]:
        """Computed sensor keys for this configuration."""
        from .const import (  # noqa: PLC0415
            DERIVED_CLOCK,
            DERIVED_COMMON,
            DERIVED_DEVICE,
            DERIVED_EVENT,
            DERIVED_EXTRA,
            DERIVED_FAULTS,
        )

        keys = [d.key for d in DERIVED_COMMON] + [d.key for d in DERIVED_DEVICE]
        if self.supports_faults:
            keys += [d.key for d in DERIVED_FAULTS]
            keys += [d.key for d in DERIVED_EXTRA]
            keys += [d.key for d in DERIVED_EVENT]
        if self.supports_clock:
            keys += [d.key for d in DERIVED_CLOCK]
        return keys

    @property
    def binary_sensor_keys(self) -> list[str]:
        """Binary sensor keys for this configuration."""
        from .const import (  # noqa: PLC0415
            BINARY_SENSORS_CLOCK,
            BINARY_SENSORS_EPS,
            BINARY_SENSORS_FAULTS,
            BINARY_SENSORS_PASSIVE_FLAGS,
        )

        keys: list[str] = []
        if self.supports_faults:
            keys += list(BINARY_SENSORS_FAULTS) + list(BINARY_SENSORS_EPS)
        if self.enable_control and self.protocol == PROTO_PASSIVE_FN42:
            keys += list(BINARY_SENSORS_PASSIVE_FLAGS)
        if self.supports_clock:
            keys += list(BINARY_SENSORS_CLOCK)
        return keys

    @property
    def effective_max_power(self) -> int:
        """Command clamp: the configured limit, capped by a detected rating.

        The rating is only applied when it was decoded confidently from the
        barcode, so a correctly configured 6 kW hybrid is never crippled by the
        3 kW figure that the protocol document quotes for the smallest model.
        """
        if self.rated_power and self.rated_power < self.max_power:
            if not self._rated_power_warned:
                _LOGGER.warning(
                    "Configured maximum power %sW exceeds the detected rating of "
                    "%sW for %s; commands will be clamped to %sW",
                    self.max_power, self.rated_power,
                    self.serial_number or self.product_code or self.model,
                    self.rated_power,
                )
                self._rated_power_warned = True
            return self.rated_power
        return self.max_power

    def _rebuild_blocks(self) -> None:
        """Recompute the batched read plan."""
        read_list = list(self.sensors)
        if self.supports_schedule:
            read_list += list(SCHEDULE_REGISTERS)
        self._blocks = build_read_blocks(read_list)
        self._sensor_by_key = {s.key: s for s in read_list}
        _LOGGER.debug(
            "%s:%s slave %s: %s registers grouped into %s read block(s): %s",
            self.host, self.port, self.slave, len(read_list), len(self._blocks),
            ", ".join(f"0x{b.start:04X}+{b.count}" for b in self._blocks),
        )

    # ----------------------------------------------------------- lifecycle
    async def async_shutdown(self) -> None:
        """Release this coordinator's hold on the shared gateway connection."""
        if self._gateway.release(self._gateway_user):
            await self._gateway.async_close()
            self.hass.data.get(DOMAIN, {}).get("_gateways", {}).pop(
                (self.host, self.port), None
            )
        await super().async_shutdown()

    # --------------------------------------------------------------- decoding
    @staticmethod
    def _decode(registers: list[int], count: int, signed: bool) -> int:
        """Decode 1 or 2 big-endian holding registers into an int."""
        if count == 2:
            raw = (registers[0] << 16) | registers[1]
            if signed and raw >= 0x80000000:
                raw -= 0x100000000
            return raw
        raw = registers[0] & 0xFFFF
        if signed and raw >= 0x8000:
            raw -= 0x10000
        return raw

    def _log_modbus_error(self, what: str, response) -> None:
        """Log a Modbus exception response with its decoded meaning."""
        code = getattr(response, "exception_code", None)
        if code is not None:
            _LOGGER.debug(
                "%s: Modbus exception 0x%02X from function 0x%02X (%s)",
                what,
                code,
                getattr(response, "function_code", 0) & 0x7F,
                MODBUS_EXCEPTIONS.get(code, "unknown exception"),
            )
        else:
            _LOGGER.debug("%s: error response %s", what, response)

    async def _read_registers(
        self, client, func: int, address: int, count: int, what: str
    ) -> list[int] | None:
        """Read registers with one function code. None on any failure.

        Sets ``self._no_response`` when nothing came back (timeout after
        pymodbus's retries, or no connection), as opposed to an exception reply
        or a short one. Only an answered failure is worth retrying with the
        other function code or register by register.
        """
        self._no_response = False
        try:
            if func == FN_READ_INPUT:
                response = await client.read_input_registers(
                    address=address, count=count, device_id=self.slave
                )
            else:
                response = await client.read_holding_registers(
                    address=address, count=count, device_id=self.slave
                )
        except Exception as exc:  # noqa: BLE001
            _LOGGER.debug("%s: read 0x%04X+%s failed: %s", what, address, count, exc)
            self._no_response = True
            return None

        if response is None:
            self._no_response = True
            return None
        if hasattr(response, "isError") and response.isError():
            self._log_modbus_error(f"{what} (0x{address:04X}+{count})", response)
            return None
        registers = getattr(response, "registers", None)
        if not registers or len(registers) < count:
            return None
        return list(registers)

    # ------------------------------------------------------------------ poll
    async def _connected_client(self):
        """The shared client, connected. Must be called holding self._lock."""
        try:
            return await self._gateway.async_get_client()
        except Exception as exc:  # noqa: BLE001
            raise UpdateFailed(f"Cannot connect to {self.host}:{self.port}: {exc}") from exc

    async def _async_update_data(self) -> dict[str, object]:
        self._gateway.ensure_heartbeat()
        # Each step takes the gateway lock on its own, so the heartbeat and
        # commands can reach the bus between steps and between register blocks
        # instead of waiting out a whole poll. Holding it for the entire poll let
        # one unanswered register range keep the heartbeat off the bus for
        # minutes, past the inverter's 60 s passive-mode watchdog.
        try:
            async with self._lock:
                client = await self._connected_client()
                if not self._device_info_done:
                    await self._read_device_info(client)
                if self.model == MODEL_HYD_EP_KTL and not self._battery2_checked:
                    await self._maybe_add_battery2(client)

            results = await self._read_all()
            if not results:
                raise UpdateFailed("No registers could be read from the inverter")

            async with self._lock:
                client = await self._connected_client()
                await self._maybe_read_event_log(client)

                self._derive(results)

                # Clear any forced passive setpoint left latched by a previous
                # run before honouring the (restart-default) "No Override".
                await self._maybe_failsafe(client, results)

                # Re-apply / evaluate the active control mode.
                await self._apply_active_mode(client, results)

            # Last, because reading the clock needs a dedicated connection and
            # therefore drops the shared client for the rest of the cycle.
            async with self._lock:
                await self._maybe_read_clock()

            self._add_status_values(results)
            return results
        except UpdateFailed:
            raise
        except Exception as exc:  # noqa: BLE001
            raise UpdateFailed(f"Error polling {self.host}:{self.port}: {exc}") from exc

    async def _read_all(self) -> dict[str, object]:
        """Read every configured register, batched into as few requests as possible.

        The gateway lock is taken per block. If nothing answers before any
        register has been read this poll, the inverter is not responding and the
        poll fails at once rather than timing out block after block.
        """
        results: dict[str, object] = {}

        for block in self._blocks:
            if block.skip_polls > 0:
                block.skip_polls -= 1
                continue
            async with self._lock:
                client = await self._connected_client()
                await self._read_block(client, block, results)

        # Values written but not yet visible in a re-read keep their staged value
        # so the UI does not flicker back to the old setting.
        for key, value in self._pending_schedule.items():
            results.setdefault(key, value)

        return results

    async def _read_block(self, client, block: ReadBlock, results: dict) -> None:
        """Read one block, falling back to per-register reads when refused."""
        if block.individual:
            block.polls_since_fallback += 1
            if block.polls_since_fallback < BLOCK_RETRY_POLLS:
                read = await self._read_block_individually(client, block, results)
                if not read and self._no_response:
                    self._note_silent_block(block, results)
                return
            # Periodically try the whole block again, so one bad spell does not
            # cost a transaction per register for good.
            block.polls_since_fallback = 0

        registers = None
        for func in block.func_order():
            registers = await self._read_registers(
                client, func, block.start, block.count, "Block read"
            )
            if registers is not None:
                block.func = func
                break
            if self._no_response:
                break  # silence: another function code will not help

        if registers is None:
            if self._no_response:
                self._note_silent_block(block, results)
                return
            if not block.individual:
                # Do not lose a whole block to one transient refusal.
                block.func = None
            read = await self._read_block_individually(client, block, results)
            if not read and self._no_response:
                self._note_silent_block(block, results)
                return
            # Only a block the inverter refuses while its registers still read
            # one by one counts against it; silence part-way through does not.
            if read and not block.individual and not self._no_response:
                block.failures += 1
                if block.failures >= 3:
                    block.individual = True
                    block.polls_since_fallback = 0
                    _LOGGER.info(
                        "Register block 0x%04X+%s keeps failing on %s; falling "
                        "back to individual reads for its %s register(s)",
                        block.start, block.count, self.host, len(block.sensors),
                    )
            if read:
                block.silent_polls = 0
            return

        if block.individual:
            _LOGGER.info(
                "Register block 0x%04X+%s on %s reads as a block again",
                block.start, block.count, self.host,
            )
            block.individual = False
        block.failures = 0
        block.silent_polls = 0
        for sensor in block.sensors:
            offset = sensor.register - block.start
            chunk = registers[offset : offset + sensor.count]
            if len(chunk) < sensor.count:
                continue
            self._store_value(results, sensor, chunk)

    def _note_silent_block(self, block: ReadBlock, results: dict) -> None:
        """React to a block that got no reply at all."""
        if not results:
            # Nothing has answered this poll: the inverter (or the RS485 side of
            # the gateway) is silent. Fail now instead of timing out every block.
            raise UpdateFailed(
                f"No response from unit {self.slave} at {self.host}:{self.port}"
            )
        block.silent_polls += 1
        block.skip_polls = min(2 ** (block.silent_polls - 1), BLOCK_SILENT_MAX_SKIP)
        _LOGGER.debug(
            "Register block 0x%04X+%s got no reply from %s; skipping it for %s poll(s)",
            block.start, block.count, self.host, block.skip_polls,
        )

    async def _read_block_individually(self, client, block, results: dict) -> int:
        """Per-register fallback so one bad address cannot blank a whole block.

        Returns how many of the block's registers were read.
        """
        read = 0
        for sensor in block.sensors:
            if sensor.key in results:
                continue
            registers = None
            for func in block.func_order():
                registers = await self._read_registers(
                    client, func, sensor.register, sensor.count, f"Read {sensor.key}"
                )
                if registers is not None or self._no_response:
                    break
            if registers is None:
                if self._no_response:
                    break  # nothing is answering; do not time out every register
                continue
            read += 1
            self._store_value(results, sensor, registers)
        return read

    def _store_value(self, results: dict, sensor, registers: list[int]) -> None:
        """Scale and store one decoded register value."""
        if (
            sensor.min_fw is not None
            and self._firmware_int is not None
            and self._firmware_int < sensor.min_fw
        ):
            # Documented as unpopulated on this firmware: report unavailable
            # rather than a misleading zero.
            return

        raw = self._decode(registers, sensor.count, sensor.signed)

        if sensor.is_state:
            results[sensor.key] = self.runstate_map.get(raw, f"Unknown ({raw})")
            results[f"{sensor.key}_raw"] = raw
        else:
            results[sensor.key] = round(raw * sensor.scale, 3)

    async def _maybe_add_battery2(self, client) -> None:
        """Add second-battery sensors if the inverter reports a 2nd battery.

        A transient failure on the very first poll used to suppress the
        second-battery entities until the entry was reloaded, so detection is
        retried a few times before being given up on.
        """
        self._battery2_attempts += 1
        registers = await self._read_registers(
            client,
            FN_READ_HOLDING,
            SENSORS_HYD_EP_KTL_BATTERY2[0].register,
            1,
            "Battery 2 detection",
        )
        if registers is None:
            if self._battery2_attempts >= 3:
                self._battery2_checked = True
                _LOGGER.debug(
                    "Giving up on second-battery detection for %s after %s attempts",
                    self.host, self._battery2_attempts,
                )
            return

        self._battery2_checked = True
        if registers[0] > 0:
            existing = {s.key for s in self.sensors}
            added = [s for s in SENSORS_HYD_EP_KTL_BATTERY2 if s.key not in existing]
            if added:
                self.sensors.extend(added)
                self._rebuild_blocks()
            _LOGGER.info("Second battery detected on %s", self.host)

    # ------------------------------------------------------------ device info
    async def _read_device_info(self, client) -> None:
        """Best-effort read of product code, serial and firmware versions.

        Never allowed to block or fail a poll: after a few attempts it is simply
        given up on, leaving the rated-power clamp and firmware gating inactive.
        """
        self._device_info_attempts += 1
        registers = None

        for func in (FN_READ_HOLDING, FN_READ_INPUT):
            registers = await self._read_registers(
                client, func, REG_DEVICE_INFO, 16, "Device info"
            )
            if registers is not None or self._no_response:
                break

        if registers is None and not self._no_response:
            # Fall back to the extended function code the protocol table assigns
            # to this range.
            try:
                response = await client.execute(
                    False,
                    SofarExtReadRequest(address=REG_DEVICE_INFO, count=16, slave=self.slave),
                )
                if response is not None and not (
                    hasattr(response, "isError") and response.isError()
                ):
                    candidate = getattr(response, "registers", None)
                    if candidate and len(candidate) >= 16:
                        registers = list(candidate)
            except Exception as exc:  # noqa: BLE001
                _LOGGER.debug("Extended device-info read failed: %s", exc)

        if registers is None:
            if self._device_info_attempts >= 3:
                self._device_info_done = True
                _LOGGER.debug(
                    "Could not read device information from %s; rated-power "
                    "detection and firmware gating stay disabled",
                    self.host,
                )
            return

        self._device_info_done = True
        self.product_code = _registers_to_ascii(registers[0:1]) or None
        self.serial_number = _registers_to_ascii(registers[1:8]) or None
        self.arm_version = _registers_to_ascii(registers[8:10]) or None
        self.hardware_version = _registers_to_ascii(registers[10:12]) or None
        self._firmware_int = self._parse_version(self.arm_version)
        self.rated_power = self._detect_rated_power()

        _LOGGER.debug(
            "%s device info: serial=%s product=%s arm=%s hw=%s rated=%s",
            self.host, self.serial_number, self.product_code,
            self.arm_version, self.hardware_version, self.rated_power,
        )

    @staticmethod
    def _parse_version(version: str | None) -> int | None:
        """"V120" -> 120, so firmware-gated registers can be compared numerically."""
        if not version:
            return None
        digits = "".join(ch for ch in version if ch.isdigit())
        if not digits:
            return None
        try:
            return int(digits)
        except ValueError:
            return None

    def _detect_rated_power(self) -> int | None:
        """Decode rated power from the barcode (protocol doc 2.3.4).

        Barcode layout: digit 1 brand, digits 2-3 family, digit 4 region,
        digits 5-6 configuration, digits 7-8 power. Only the storage and
        single-phase-hybrid families are decoded; anything else returns None so
        that the user's configured limit is used unchanged.
        """
        barcode = self.serial_number or ""
        if len(barcode) < 8:
            return None
        family = barcode[1:3].upper()
        power_code = barcode[6:8]
        return RATED_POWER_BY_FAMILY.get(family, {}).get(power_code)

    # ---------------------------------------------------------------- derived
    def _derive(self, results: dict) -> None:
        """Compute the values that need no extra bus traffic."""
        # Operating mode as text, alongside the original numeric working_mode.
        working_mode = results.get("working_mode")
        if working_mode is not None:
            raw_mode = int(working_mode)
            results["operating_mode"] = WORKING_MODE_MAP.get(
                raw_mode, f"Unknown ({raw_mode})"
            )

        if self.supports_faults:
            faults = _decode_bits(results, FAULT_BITS) + _decode_unknown_bits(
                results, _FAULT_SOURCES
            )
            alarms = _decode_bits(results, ALARM_BITS) + _decode_unknown_bits(
                results, _ALARM_SOURCES
            )
            battery_faults = _decode_bits(results, BATTERY_FAULT_BITS) + _decode_unknown_bits(
                results, _BATTERY_FAULT_SOURCES
            )
            comm_faults = _decode_bits(results, COMM_BOARD_BITS) + _decode_unknown_bits(
                results, _COMM_SOURCES
            )

            results["active_faults"] = _format_fault_list(faults)
            results["active_faults_attrs"] = {"count": len(faults), "faults": faults}
            results["active_alarms"] = _format_fault_list(alarms)
            results["active_alarms_attrs"] = {"count": len(alarms), "alarms": alarms}
            results["battery_faults"] = _format_fault_list(battery_faults)
            results["battery_faults_attrs"] = {
                "count": len(battery_faults),
                "faults": battery_faults,
            }
            results["comm_board_faults"] = _format_fault_list(comm_faults)
            results["comm_board_faults_attrs"] = {
                "count": len(comm_faults),
                "faults": comm_faults,
            }

            state_text = str(results.get("running_state") or "")
            in_fault_state = "fault" in state_text.lower()
            results["inverter_problem"] = bool(faults or battery_faults or in_fault_state)
            results["inverter_problem_attrs"] = {
                "running_state": results.get("running_state"),
                "fault_count": len(faults),
                "battery_fault_count": len(battery_faults),
                "alarm_count": len(alarms),
                "faults": faults,
                "battery_faults": battery_faults,
                "alarms": alarms,
            }

            # EPS / backup output.
            state_raw = results.get("running_state_raw")
            eps_states = EPS_RUNSTATES.get(self.model, set())
            eps_voltage = results.get("eps_voltage") or 0
            eps_power = results.get("eps_power") or 0
            results["eps_active"] = bool(
                (state_raw is not None and int(state_raw) in eps_states)
                or eps_voltage > 20
                or eps_power > 0
            )

            country = results.get("country_code")
            if country is not None:
                results["grid_code"] = COUNTRY_CODE_MAP.get(
                    int(country), f"Unknown ({int(country)})"
                )

            battery_type = results.get("battery_type")
            if battery_type is not None:
                results["battery_type_text"] = BATTERY_TYPE_MAP.get(
                    int(battery_type), f"Unknown (0x{int(battery_type):04X})"
                )

    def _add_status_values(self, results: dict) -> None:
        """Attach control, device and clock status to the poll result."""
        results["control_requested_mode"] = MODE_LABELS.get(
            self.active_mode, self.active_mode
        )
        results["control_effective_mode"] = MODE_LABELS.get(
            self.effective_mode, self.effective_mode
        )
        results["control_last_result"] = self.last_command_result or "No command sent"
        results["control_last_command_at"] = self.last_command_at
        if self.passive_flags:
            results["control_last_result_attrs"] = dict(self.passive_flags)
            for flag, value in self.passive_flags.items():
                results[flag] = value

        if self.rated_power is not None:
            results["rated_power"] = self.rated_power
        if self.arm_version:
            results["arm_firmware_version"] = self.arm_version

        if self.supports_clock:
            results["inverter_clock"] = self.inverter_clock
            results["clock_drift"] = self.clock_drift
            if self.clock_drift is not None:
                results["clock_out_of_sync"] = (
                    abs(self.clock_drift) > CLOCK_DRIFT_WARN_SECONDS
                )

        if self.last_event is not None:
            results["last_event"] = self.last_event
            if self.last_event_attrs:
                results["last_event_attrs"] = dict(self.last_event_attrs)

        device_attrs = {
            "serial_number": self.serial_number,
            "product_code": self.product_code,
            "arm_version": self.arm_version,
            "hardware_version": self.hardware_version,
            "rated_power": self.rated_power,
            "configured_max_power": self.max_power,
            "effective_max_power": self.effective_max_power,
            "heartbeat_failures": self._gateway.heartbeat_failures,
        }
        results["rated_power_attrs"] = device_attrs

    # ---------------------------------------------------------- clock / events
    async def _maybe_read_clock(self) -> None:
        """Read the inverter RTC on a dedicated connection, at most hourly.

        Timing and Time-of-Use schedules run against this clock, and the protocol
        defines an RTC fault (ID96), so silent drift would shift an off-peak
        window with no indication in Home Assistant.
        """
        if not self.supports_clock:
            return
        now = self.hass.loop.time()
        if now < self._clock_next_read:
            return
        self._clock_next_read = now + CLOCK_POLL_INTERVAL

        try:
            response = await self._gateway.async_isolated_read(
                lambda: SofarReadTimeRequest(slave=self.slave),
                SofarReadTimeResponse,
            )
        except Exception as exc:  # noqa: BLE001
            _LOGGER.debug("Clock read failed for %s: %s", self.host, exc)
            return

        clock = self._parse_clock(response)
        if clock is None:
            _LOGGER.debug("Could not parse the clock response from %s", self.host)
            return

        self.inverter_clock = clock
        drift = round((clock - dt_util.now()).total_seconds())
        previous = self.clock_drift
        self.clock_drift = drift
        if abs(drift) > CLOCK_DRIFT_WARN_SECONDS and (
            previous is None or abs(previous) <= CLOCK_DRIFT_WARN_SECONDS
        ):
            _LOGGER.warning(
                "Inverter clock on %s is %+ds away from Home Assistant. Timing and "
                "Time-of-Use windows follow the inverter clock, so correct it with "
                "the 'Sync Inverter Clock' button.",
                self.host, drift,
            )

    @staticmethod
    def _parse_clock(response) -> datetime | None:
        """Decode the 7 packed-BCD clock bytes: s, m, h, weekday, d, mon, y.

        The document's response table is ambiguous about whether the byte-count
        field is one or two bytes wide, so the declared count is used to locate
        the payload. Probing offsets blindly instead would happily accept a
        shifted read as a valid-looking date.
        """
        raw = getattr(response, "raw", b"") or b""

        if len(raw) >= 8 and raw[0] == 7:
            payload = raw[1:8]
        elif len(raw) >= 9 and raw[0] == 0 and raw[1] == 7:
            payload = raw[2:9]
        else:
            return None

        fields = [_bcd_to_int(byte) for byte in payload]
        if any(field_value is None for field_value in fields):
            return None

        second, minute, hour, weekday, day, month, year = fields
        if second > 59 or minute > 59 or hour > 23 or weekday > 7:
            return None
        if not 1 <= month <= 12 or not 1 <= day <= 31:
            return None

        try:
            return datetime(
                2000 + year, month, day, hour, minute, second,
                tzinfo=dt_util.DEFAULT_TIME_ZONE,
            )
        except ValueError:
            return None

    async def async_sync_clock(self) -> bool:
        """Broadcast Home Assistant's wall clock to the inverter.

        The protocol only defines clock setting as a broadcast to address 0x88,
        so this sets the time on every Sofar inverter sharing the RS485 bus, not
        just this one.
        """
        try:
            async with self._lock:
                client = await self._gateway.async_get_client()
                await client.execute(
                    True,
                    SofarSetTimeRequest(
                        when=dt_util.now(), slave=SOFAR_BROADCAST_ADDRESS
                    ),
                )
        except Exception as exc:  # noqa: BLE001
            _LOGGER.error("Failed to broadcast the clock to %s: %s", self.host, exc)
            return False

        _LOGGER.info(
            "Broadcast the current time to all inverters on %s:%s", self.host, self.port
        )
        # Force a fresh read so the drift sensor reflects the new value.
        self._clock_next_read = 0.0
        await self.async_request_refresh()
        return True

    async def _maybe_read_event_log(self, client) -> None:
        """Read the most recent entry from the inverter's own event log.

        The inverter keeps the last 100 events with timestamps, which survives
        Home Assistant downtime and is a better basis for "last fault" than
        edge-detecting live bitfields.
        """
        if not self.supports_faults:
            return
        now = self.hass.loop.time()
        if now < self._event_next_read:
            return
        self._event_next_read = now + EVENT_LOG_INTERVAL

        try:
            response = await client.execute(
                False, SofarEventLogRequest(index=1, slave=self.slave)
            )
        except Exception as exc:  # noqa: BLE001
            _LOGGER.debug("Event log read failed for %s: %s", self.host, exc)
            return

        if response is None or (hasattr(response, "isError") and response.isError()):
            self._log_modbus_error("Event log read", response)
            return

        parsed = self._parse_event(getattr(response, "raw", b"") or b"")
        if parsed is None:
            _LOGGER.debug("Could not parse the event record from %s", self.host)
            return

        event_id, when = parsed
        known = _FAULT_BY_ID.get(event_id)
        description = known.description if known else f"Event {event_id}"
        self.last_event = f"ID{event_id:02d} {description}"
        self.last_event_attrs = {
            "event_id": event_id,
            "code": known.code if known else None,
            "description": description,
            "timestamp": when.isoformat(),
        }

    @staticmethod
    def _parse_event(raw: bytes) -> tuple[int, datetime] | None:
        """Decode an event record: id, year, month, day, weekday, h, m, s.

        The address table calls the record "Hex" while the timing records in the
        same section are BCD, so both encodings are tried and validated.
        """
        if len(raw) < 8:
            return None
        for use_bcd in (True, False):
            values: list[int | None] = []
            for byte in raw[:8]:
                values.append(_bcd_to_int(byte) if use_bcd else byte)
            if any(value is None for value in values):
                continue
            event_id, year, month, day, _weekday, hour, minute, second = values
            if not 1 <= month <= 12 or not 1 <= day <= 31:
                continue
            if hour > 23 or minute > 59 or second > 59:
                continue
            try:
                when = datetime(
                    2000 + year, month, day, hour, minute, second,
                    tzinfo=dt_util.DEFAULT_TIME_ZONE,
                )
            except ValueError:
                continue
            return event_id, when
        return None

    # --------------------------------------------------------------- control
    def set_active_mode(self, mode: str, power: int | None = None) -> None:
        """Store the desired control mode; applied on the next poll/refresh."""
        self.active_mode = mode
        if power is not None:
            self.active_power = max(0, min(power, self.effective_max_power))

    async def async_set_mode(self, mode: str, power: int | None = None) -> bool:
        """Set the control mode and apply it immediately."""
        self.set_active_mode(mode, power)
        if mode == MODE_NONE:
            self.effective_mode = MODE_NONE
            self._record_command(CommandResult(True, "Override cleared"))
            return True
        async with self._lock:
            try:
                client = await self._gateway.async_get_client()
            except Exception as exc:  # noqa: BLE001
                _LOGGER.error(
                    "Cannot connect to %s:%s to send command: %s",
                    self.host, self.port, exc,
                )
                self._record_command(CommandResult(False, f"Connection failed: {exc}"))
                return False
            # Battery Save chooses Auto or Standby from grid power; use the last
            # polled value rather than None, which always meant Auto.
            grid_power = self.data.get("grid_power") if self.data else None
            result = await self._dispatch(client, mode, self.active_power, grid_power)
        self._record_command(result)
        # Refresh so entities reflect the new state quickly.
        await self.async_request_refresh()
        return result.ok

    def _record_command(self, result: CommandResult) -> None:
        """Remember the inverter's verdict so it can be surfaced as a sensor."""
        self.last_command_result = result.message
        self.last_command_at = dt_util.now()
        if result.flags:
            self.passive_flags = result.flags

    async def _maybe_failsafe(self, client, results: dict) -> None:
        """Clear a forced setpoint left latched from a previous run.

        Home Assistant deliberately starts in "No Override", but the inverter
        keeps whatever passive setpoint it was last given, so a forced charge
        could otherwise stay latched across a restart. Sending Auto once releases
        it without restoring the previous forced mode. Reaching standby is
        avoided on purpose: leaving standby is rate-limited by the grid-code
        power ramp.
        """
        if self._failsafe_done:
            return
        if not self.enable_control or self.protocol != PROTO_PASSIVE_FN42:
            self._failsafe_done = True
            return
        if self.active_mode != MODE_NONE:
            self._failsafe_done = True
            return

        working_mode = results.get("working_mode")
        if working_mode is None:
            # The working-mode register decides whether a latched setpoint is even
            # possible, so allow a few polls to read it before giving up.
            self._failsafe_attempts += 1
            if self._failsafe_attempts >= 3:
                self._failsafe_done = True
                _LOGGER.debug(
                    "Could not read the working mode from %s; skipping the "
                    "fail-safe Auto command",
                    self.host,
                )
            return

        self._failsafe_done = True
        if int(working_mode) != WORKING_MODE_PASSIVE:
            return

        _LOGGER.info(
            "%s is in Passive Mode with no Home Assistant override; sending Auto "
            "once to release any setpoint latched before the restart",
            self.host,
        )
        result = await self._do_passive_fn42(client, MODE_AUTO, 0)
        self._record_command(result)
        if not result.ok:
            _LOGGER.warning(
                "Fail-safe Auto command was not accepted by %s: %s",
                self.host, result.message,
            )

    async def _apply_active_mode(self, client, results: dict) -> None:
        """Re-apply the active mode during a poll (lock already held)."""
        if self.active_mode == MODE_NONE:
            return
        grid_power = results.get("grid_power")
        try:
            result = await self._dispatch(
                client, self.active_mode, self.active_power, grid_power
            )
        except Exception as exc:  # noqa: BLE001
            _LOGGER.debug("Re-applying active mode failed: %s", exc)
            return
        # Surface a mode that has started being rejected — for example because
        # the inverter was taken out of Passive Mode at the panel.
        if not result.ok and result.message != self.last_command_result:
            _LOGGER.warning(
                "Sofar %s rejected the re-applied '%s' command: %s",
                self.host, self.active_mode, result.message,
            )
        self._record_command(result)

    async def _dispatch(
        self, client, mode: str, power: int, grid_power
    ) -> CommandResult:
        """Route a control request to the right protocol handler."""
        if mode == MODE_BATTERY_SAVE:
            # Hybrid auto: charge from excess solar but never discharge.
            # Exporting (grid_power >= ~0) -> auto, otherwise force standby.
            mode = MODE_AUTO if (grid_power is None or grid_power >= -100) else MODE_STANDBY

        self.effective_mode = mode

        if self.protocol == PROTO_PASSIVE_FN42:
            return await self._do_passive_fn42(client, mode, power)
        if self.protocol == PROTO_PASSIVE_REG:
            return await self._do_passive_reg(client, mode, power)
        return CommandResult(False, f"No control protocol for model {self.model}")

    def _parse_passive_response(self, response) -> CommandResult:
        """Decode the 0x42 status word instead of assuming success.

        The low byte says whether the command was accepted and why not; the high
        byte reports battery readiness. Without this a command rejected because
        the inverter is not in Passive Mode looked like a success.
        """
        if response is None:
            return CommandResult(False, "No response from inverter")
        if hasattr(response, "isError") and response.isError():
            code = getattr(response, "exception_code", None)
            if code is not None:
                return CommandResult(
                    False,
                    f"Modbus exception 0x{code:02X}: "
                    f"{MODBUS_EXCEPTIONS.get(code, 'unknown exception')}",
                )
            return CommandResult(False, f"Error response: {response}")

        values = getattr(response, "values", None)
        if not values:
            return CommandResult(True, "Accepted (no status word returned)")

        word = int(values[0]) & 0xFFFF
        status = word & 0xFF
        flag_byte = (word >> 8) & 0xFF
        flags = {
            "battery_charge_ready": bool(flag_byte & (1 << PASSIVE_FLAG_CHARGE_READY)),
            "battery_discharge_ready": bool(flag_byte & (1 << PASSIVE_FLAG_DISCHARGE_READY)),
            "battery_full": bool(flag_byte & (1 << PASSIVE_FLAG_BATTERY_FULL)),
            "battery_empty": bool(flag_byte & (1 << PASSIVE_FLAG_BATTERY_EMPTY)),
        }
        message = PASSIVE_STATUS_MAP.get(status, f"Unknown status 0x{status:02X}")
        return CommandResult(status == PASSIVE_STATUS_ACCEPTED, message, status, flags)

    async def _do_passive_fn42(self, client, mode: str, power: int) -> CommandResult:
        """ME3000 / HYD-ES passive-mode command via function code 0x42."""
        limit = self.effective_max_power
        if mode == MODE_STANDBY:
            fn_code, param = SOFAR_FN_STANDBY, SOFAR_PARAM_STANDBY
        elif mode == MODE_AUTO:
            # The protocol document lists Auto with the 0x5555 sentinel, unlike
            # the 0x0000 the ported firmware sent.
            fn_code, param = SOFAR_FN_AUTO, SOFAR_PARAM_AUTO
        elif mode == MODE_CHARGE:
            fn_code, param = SOFAR_FN_CHARGE, max(0, min(power, limit))
        elif mode == MODE_DISCHARGE:
            fn_code, param = SOFAR_FN_DISCHARGE, max(0, min(power, limit))
        else:
            return CommandResult(False, f"Unsupported mode '{mode}'")

        try:
            request = SofarPassiveModeRequest(cmd=fn_code, param=param, slave=self.slave)
            response = await client.execute(False, request)
        except Exception as exc:  # noqa: BLE001
            _LOGGER.error("Error sending passive-mode command: %s", exc)
            return CommandResult(False, f"Send failed: {exc}")

        result = self._parse_passive_response(response)
        if result.ok:
            _LOGGER.debug(
                "Passive-mode command ok (mode=%s, param=%s): %s",
                mode, param, result.message,
            )
        else:
            _LOGGER.error(
                "Passive-mode command failed (mode=%s, param=%s): %s",
                mode, param, result.message,
            )
        return result

    async def _do_passive_reg(self, client, mode: str, power: int) -> CommandResult:
        """HYD-EP/KTL passive control via write-multiple-registers (0x1187)."""
        limit = self.effective_max_power
        if mode == MODE_STANDBY:
            min_power = max_power = 0
        elif mode == MODE_AUTO:
            if power > 0:
                min_power, max_power = -power, power
            else:
                min_power, max_power = -16384, 16383
        elif mode == MODE_CHARGE:
            min_power = max_power = max(0, min(power, limit))
        elif mode == MODE_DISCHARGE:
            min_power = max_power = -max(0, min(power, limit))
        else:
            return CommandResult(False, f"Unsupported mode '{mode}'")

        def _split(value: int) -> tuple[int, int]:
            raw = value & 0xFFFFFFFF
            return (raw >> 16) & 0xFFFF, raw & 0xFFFF

        min_hi, min_lo = _split(min_power)
        max_hi, max_lo = _split(max_power)
        # 6x 16-bit regs = desired32 (forced to 0), min32, max32 — matching the
        # firmware's sendPassiveCmdV2 frame (two leading zero bytes per word).
        payload = [0x0000, 0x0000, min_hi, min_lo, max_hi, max_lo]

        ok, message = await self._write_registers(
            client, SOFAR2_REG_PASSIVECONTROL, payload, "Passive control"
        )
        if ok:
            _LOGGER.debug(
                "Passive control ok (mode=%s, min=%s, max=%s)", mode, min_power, max_power
            )
            return CommandResult(True, "Accepted")
        return CommandResult(False, message)

    async def _write_registers(
        self, client, address: int, values: list[int], what: str
    ) -> tuple[bool, str]:
        """Write registers, decoding a Modbus exception response explicitly."""
        try:
            response = await client.write_registers(
                address=address, values=values, device_id=self.slave
            )
        except Exception as exc:  # noqa: BLE001
            _LOGGER.error("%s: write to 0x%04X failed: %s", what, address, exc)
            return False, f"Write failed: {exc}"

        if response is None:
            return False, "No response from inverter"
        if hasattr(response, "isError") and response.isError():
            code = getattr(response, "exception_code", None)
            if code is not None:
                message = (
                    f"Modbus exception 0x{code:02X}: "
                    f"{MODBUS_EXCEPTIONS.get(code, 'unknown exception')}"
                )
            else:
                message = f"Error response: {response}"
            _LOGGER.error("%s: write to 0x%04X rejected: %s", what, address, message)
            return False, message
        return True, "Accepted"

    # ------------------------------------------------------ schedule / TOU
    def schedule_value(self, key: str) -> int | None:
        """Current raw register value for a schedule key.

        A value that has been written but not yet confirmed by a re-read takes
        precedence, so the UI does not flick back to the previous setting.
        """
        if key in self._pending_schedule:
            return self._pending_schedule[key]
        if self.data is None:
            return None
        value = self.data.get(key)
        return None if value is None else int(value)

    def _collect_block(self, keys: tuple[str, ...]) -> list[int] | None:
        """Gather the raw values for one atomically-written block."""
        values: list[int] = []
        for key in keys:
            value = self.schedule_value(key)
            if value is None:
                _LOGGER.error(
                    "Refusing to write the schedule block: %s has not been read "
                    "from the inverter yet, and writing a default would overwrite "
                    "the inverter's own setting",
                    key,
                )
                return None
            values.append(int(value) & 0xFFFF)
        return values

    async def async_set_schedule_value(self, key: str, raw_value: int) -> bool:
        """Stage one schedule register and write its whole block atomically.

        The protocol document writes the timing parameters as a single six
        register block, and requires the Time-of-Use rule number to be set before
        any other TOU parameter, so individual registers are never written alone.
        """
        if not self.supports_schedule:
            return False

        previous = self._pending_schedule.get(key)
        self._pending_schedule[key] = int(raw_value) & 0xFFFF

        try:
            if key in TIMING_BLOCK_KEYS:
                ok = await self._write_schedule_block(REG_TIMING_BLOCK, TIMING_BLOCK_KEYS)
            elif key == "tou_rule_number":
                ok = await self._write_tou_rule_number()
            elif key in TOU_BLOCK_KEYS:
                ok = await self._write_tou_block()
            else:
                _LOGGER.error("Unknown schedule key '%s'", key)
                ok = False
        except Exception as exc:  # noqa: BLE001
            _LOGGER.error("Error writing schedule key %s: %s", key, exc)
            ok = False

        if not ok:
            # Do not leave a value staged that the inverter never accepted.
            if previous is None:
                self._pending_schedule.pop(key, None)
            else:
                self._pending_schedule[key] = previous

        await self.async_request_refresh()
        return ok

    async def _write_schedule_block(self, start: int, keys: tuple[str, ...]) -> bool:
        """Write and verify one contiguous schedule block."""
        values = self._collect_block(keys)
        if values is None:
            return False

        async with self._lock:
            client = await self._gateway.async_get_client()
            ok, message = await self._write_registers(
                client, start, values, "Schedule write"
            )
            if not ok:
                _LOGGER.error("Schedule write to 0x%04X failed: %s", start, message)
                return False
            return await self._verify_schedule(client, start, keys, values)

    async def _write_tou_rule_number(self) -> bool:
        """Select the Time-of-Use rule that subsequent parameters belong to."""
        rule = self.schedule_value("tou_rule_number")
        if rule is None:
            return False
        async with self._lock:
            client = await self._gateway.async_get_client()
            ok, message = await self._write_registers(
                client, REG_TOU_RULE_NUMBER, [int(rule) & 0xFFFF], "TOU rule number"
            )
            if not ok:
                _LOGGER.error("Could not select TOU rule %s: %s", rule, message)
                return False
            return await self._verify_schedule(
                client, REG_TOU_RULE_NUMBER, ("tou_rule_number",), [int(rule) & 0xFFFF]
            )

    async def _write_tou_block(self) -> bool:
        """Write the TOU parameter block, selecting its rule number first."""
        rule = self.schedule_value("tou_rule_number")
        values = self._collect_block(TOU_BLOCK_KEYS)
        if rule is None or values is None:
            return False

        async with self._lock:
            client = await self._gateway.async_get_client()
            # "Be sure to set the rule number before setting other parameters."
            ok, message = await self._write_registers(
                client, REG_TOU_RULE_NUMBER, [int(rule) & 0xFFFF], "TOU rule number"
            )
            if not ok:
                _LOGGER.error("Could not select TOU rule %s: %s", rule, message)
                return False
            ok, message = await self._write_registers(
                client, REG_TOU_BLOCK, values, "TOU parameters"
            )
            if not ok:
                _LOGGER.error("TOU parameter write failed: %s", message)
                return False
            return await self._verify_schedule(client, REG_TOU_BLOCK, TOU_BLOCK_KEYS, values)

    async def _verify_schedule(
        self, client, start: int, keys: tuple[str, ...], written: list[int]
    ) -> bool:
        """Read the block back and confirm every register actually took."""
        registers = await self._read_registers(
            client, FN_READ_HOLDING, start, len(written), "Schedule verify"
        )
        if registers is None:
            _LOGGER.warning(
                "Could not read back 0x%04X to verify the schedule write", start
            )
            return False

        ok = True
        for key, expected, actual in zip(keys, written, registers, strict=False):
            if expected != actual:
                _LOGGER.error(
                    "Schedule register 0x%04X (%s) did not take: wrote %s, read %s",
                    start + keys.index(key), key, expected, actual,
                )
                ok = False
            else:
                self._pending_schedule.pop(key, None)
        return ok

    async def async_set_working_mode(self, mode_value: int) -> bool:
        """Change the inverter working mode (0x1200), verifying the result.

        Leaving Passive Mode while a Home Assistant override is active would
        abandon a forced setpoint, so the override is released first.
        """
        if not self.supports_working_mode_write:
            _LOGGER.error(
                "Changing the working mode is not supported for model %s", self.model
            )
            return False

        async with self._lock:
            client = await self._gateway.async_get_client()

            if mode_value != WORKING_MODE_PASSIVE and self.active_mode != MODE_NONE:
                _LOGGER.info(
                    "Releasing the '%s' override before leaving Passive Mode",
                    self.active_mode,
                )
                await self._dispatch(client, MODE_AUTO, 0, None)
                self.active_mode = MODE_NONE
                self.active_power = 0
                self.effective_mode = MODE_NONE

            ok, message = await self._write_registers(
                client, self.working_mode_register, [mode_value & 0xFFFF], "Working mode"
            )
            if not ok:
                _LOGGER.error("Could not change the working mode: %s", message)
                return False

            registers = await self._read_registers(
                client, FN_READ_HOLDING, self.working_mode_register, 1, "Working mode verify"
            )

        if registers is None:
            _LOGGER.warning("Could not read back the working mode to verify the change")
            ok = False
        elif registers[0] != mode_value:
            _LOGGER.error(
                "Working mode did not change: wrote %s (%s), read %s (%s)",
                mode_value, WORKING_MODE_MAP.get(mode_value),
                registers[0], WORKING_MODE_MAP.get(registers[0]),
            )
            ok = False

        await self.async_request_refresh()
        return ok
