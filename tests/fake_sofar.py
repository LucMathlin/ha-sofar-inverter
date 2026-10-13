"""A fake SOFAR ME3000SP behind a Waveshare gateway in "Modbus TCP to RTU" mode.

Every request and reply carries a 7-byte MBAP header (transaction id, protocol
id, length, unit id), exactly as a Waveshare gateway in "Modbus TCP to RTU" mode
presents an RS485 inverter to a TCP client.

What it models
--------------
* FC 0x03 / 0x04 reads of 0x0200-0x0255, 0x1200-0x120F, 0x10B0-0x10BC and
  0x2000-0x200F (an unknown address returns exception 0x02).
* FC 0x42 passive commands (0x0100 standby, 0x0101 discharge W, 0x0102 charge W,
  0x0103 auto). The reply carries a status word: the low byte is
  0 accepted / 1 not in Passive mode / 3 busy / 4 invalid data, the high byte
  holds the charge-ready, discharge-ready, full and empty flags (bits 0-3).
  ``passive_reply = "echo"`` instead echoes cmd + param. Which layout a real
  ME3000SP uses has not been captured on hardware, so tests can exercise both.
* FC 0x49 heartbeat with payload ``22 01 22 02``. By default it is not answered;
  ``heartbeat_reply = "echo"`` makes the inverter echo it.
* The passive-mode watchdog: in Passive mode, 60 s without a heartbeat drops the
  current setpoint to standby. Time comes from ``now()``, which tests can move
  forward with ``advance()`` instead of sleeping.
* FC 0x10 writes (and Sofar's FC 0x10 "read clock" at 0x8000 with count 0),
  FC 0x06 writes, FC 0x02 broadcast clock set to unit 0x88 (never answered),
  FC 0x21 extended read of 0x2000 and FC 0x50 event-log read.

Injectable faults
-----------------
``silent`` (gateway up, inverter never answers), ``silent_ranges`` (only some
reads go unanswered), ``exception_code`` / ``exception_fcs`` (Modbus exception
replies), ``short_reply`` (one register fewer than asked), ``wrong_unit`` (reply
carries another unit id), ``busy`` (0x42 status 3), ``gateway_timeout_exception``
(the gateway answers 0x0B itself when the RS485 side is silent), Passive mode off
(``working_mode`` != 3), ``drop_connections()`` and ``stop()`` / ``start()``.
"""

from __future__ import annotations

import asyncio
import struct
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from datetime import datetime

UNIT_BROADCAST = 0x88

FC_READ_HOLDING = 0x03
FC_READ_INPUT = 0x04
FC_WRITE_SINGLE = 0x06
FC_WRITE_MULTIPLE = 0x10
FC_BROADCAST_TIME = 0x02
FC_EXT_READ = 0x21
FC_PASSIVE = 0x42
FC_HEARTBEAT = 0x49
FC_EVENT_LOG = 0x50

PASSIVE_STANDBY = 0x0100
PASSIVE_DISCHARGE = 0x0101
PASSIVE_CHARGE = 0x0102
PASSIVE_AUTO = 0x0103
PASSIVE_NO_VALUE = 0x5555

STATUS_ACCEPTED = 0
STATUS_NOT_PASSIVE = 1
STATUS_BUSY = 3
STATUS_INVALID = 4

FLAG_CHARGE_READY = 1 << 0
FLAG_DISCHARGE_READY = 1 << 1
FLAG_FULL = 1 << 2
FLAG_EMPTY = 1 << 3

HEARTBEAT_PAYLOAD = bytes((0x22, 0x01, 0x22, 0x02))
WATCHDOG_SECONDS = 60.0
WORKING_MODE_PASSIVE = 3
MAX_PASSIVE_WATTS = 3000

# Address ranges the fake inverter answers reads for.
READABLE_RANGES = (
    range(0x0200, 0x0256),
    range(0x1200, 0x1210),
    range(0x10B0, 0x10BD),
    range(0x2000, 0x2010),
)


def u16(value: int) -> int:
    """Two's-complement 16-bit encoding of a signed value."""
    return value & 0xFFFF


def ascii_registers(text: str, count: int) -> list[int]:
    """Pack text into ``count`` registers, high byte first, NUL padded."""
    raw = text.encode("ascii").ljust(count * 2, b"\x00")[: count * 2]
    return [(raw[i] << 8) | raw[i + 1] for i in range(0, len(raw), 2)]


def _bcd(value: int) -> int:
    return ((value // 10) << 4) | (value % 10)


def default_registers() -> dict[int, int]:
    """A plausible ME3000SP snapshot with known raw values."""
    regs = {a: 0 for r in READABLE_RANGES for a in r}
    regs.update(
        {
            0x0200: 2,  # running state: Charging
            0x0206: 2405,  # grid voltage 240.5 V
            0x0207: 512,  # grid current
            0x020C: 5002,  # 50.02 Hz
            0x020D: 150,  # battery power, x10 W
            0x020E: 5210,  # battery voltage, /100 V
            0x020F: 2880,  # battery current, /100 A
            0x0210: 60,  # SOC %
            0x0211: 21,  # battery temp
            0x0212: u16(-50),  # grid power, x10 W, signed
            0x0213: 80,  # consumption, x10 W
            0x0214: u16(-150),  # inverter power, x10 W, signed
            0x0215: 230,  # solar PV, x10 W
            0x0218: 1234,  # today generation, /100 kWh
            0x0219: 321,  # today exported
            0x021A: 456,  # today purchased
            0x021B: 789,  # today consumption
            0x021C: 0, 0x021D: 4321,  # total generation (32-bit)
            0x021E: 1, 0x021F: 2,  # total exported (32-bit) = 65538
            0x0220: 0, 0x0221: 999,  # total purchased
            0x0222: 0, 0x0223: 5000,  # total consumption
            0x0224: 500,  # today charged
            0x0225: 300,  # today discharged
            0x0226: 0, 0x0227: 1500,  # total charged
            0x0228: 0, 0x0229: 1400,  # total discharged
            0x022C: 412,  # battery cycles
            0x022D: 3800,  # bus voltage /10
            0x0237: 98,  # SOH
            0x0238: 35,  # inverter temp
            0x0239: u16(-2),  # heatsink temp, signed
            0x023A: 9,  # country code: UK-G59
            0x1200: WORKING_MODE_PASSIVE,
            0x10B0: 0x0001,  # battery type PYLON
            0x10B1: 300,  # capacity Ah
            0x10B3: 5320,  # max charge voltage /10
            0x10B9: 80,  # DOD %
        }
    )
    # Device info: product code, 14-char barcode (family E1, power code 30),
    # ARM firmware, hardware version.
    info = (
        ascii_registers("E1", 1)
        + ascii_registers("SE1E003012345A", 7)
        + ascii_registers("V120", 2)
        + ascii_registers("V100", 2)
    )
    for offset, value in enumerate(info):
        regs[0x2000 + offset] = value
    return regs


@dataclass
class Frame:
    """One request as the gateway received it."""

    at: float
    unit: int
    fc: int
    data: bytes
    answered: bool = False

    @property
    def pdu(self) -> bytes:
        return bytes((self.fc,)) + self.data


@dataclass
class FakeSofarGateway:
    """Waveshare (Modbus TCP to RTU) with a fake ME3000SP on its RS485 side."""

    slave: int = 1
    regs: dict[int, int] = field(default_factory=default_registers)

    # -- inverter behaviour knobs ------------------------------------------
    passive_reply: str = "status"  # "status" | "echo"
    heartbeat_reply: str | None = None  # None | "echo"
    strict_params: bool = True  # standby/auto must carry 0x5555
    command_counts_as_heartbeat: bool = False
    empty_soc: int = 10
    # Wall clock the inverter RTC reports (tests set it to HA's local time).
    rtc: Callable[[], datetime] = datetime.now

    # -- faults ------------------------------------------------------------
    silent: bool = False
    silent_ranges: list[range] = field(default_factory=list)
    # Multi-register reads touching these ranges get exception 0x02, while
    # single-register reads still work (a unit that refuses some spans).
    reject_span_ranges: list[range] = field(default_factory=list)
    exception_code: int | None = None
    exception_fcs: set[int] | None = None  # None = every function code
    short_reply: bool = False
    wrong_unit: bool = False
    busy: bool = False
    gateway_timeout_exception: bool = False

    def __post_init__(self) -> None:
        self.frames: list[Frame] = []
        self.writes: list[tuple[int, list[int]]] = []
        self.passive_state: str | None = None
        self.passive_watts = 0
        self.watchdog_trips = 0
        self.connections = 0
        self.port = 0
        self._offset = 0.0
        self._server: asyncio.base_events.Server | None = None
        self._writers: set[asyncio.StreamWriter] = set()
        self.last_heartbeat = self.now()
        self.setpoint_at = self.now()

    # ------------------------------------------------------------- clock
    def now(self) -> float:
        return time.monotonic() + self._offset

    def advance(self, seconds: float) -> None:
        """Move the inverter's clock forward and run the watchdog."""
        self._offset += seconds
        self.check_watchdog()

    # ------------------------------------------------------------ queries
    @property
    def working_mode(self) -> int:
        return self.regs[0x1200]

    @working_mode.setter
    def working_mode(self, value: int) -> None:
        self.regs[0x1200] = value

    @property
    def soc(self) -> int:
        return self.regs[0x0210]

    def frames_for(self, fc: int) -> list[Frame]:
        return [f for f in self.frames if f.fc == fc]

    def passive_commands(self) -> list[tuple[int, int]]:
        """(cmd, param) for every 0x42 frame received."""
        return [struct.unpack(">HH", f.data[:4]) for f in self.frames_for(FC_PASSIVE)]

    def non_read_frames(self) -> list[Frame]:
        """Everything that is not a plain FC 0x03 / 0x04 read."""
        return [f for f in self.frames if f.fc not in (FC_READ_HOLDING, FC_READ_INPUT)]

    def clear(self) -> None:
        self.frames.clear()
        self.writes.clear()

    # ------------------------------------------------------------ physics
    def _set_battery_watts(self, watts: int) -> None:
        self.regs[0x020D] = u16(watts // 10)

    def check_watchdog(self) -> None:
        if self.working_mode != WORKING_MODE_PASSIVE:
            return
        if self.passive_state in (None, "standby"):
            return
        # An accepted command opens a fresh window; the unit then needs a
        # heartbeat within 60 s to keep the setpoint.
        if self.now() - max(self.last_heartbeat, self.setpoint_at) > WATCHDOG_SECONDS:
            self.passive_state = "standby"
            self.passive_watts = 0
            self._set_battery_watts(0)
            self.watchdog_trips += 1

    def _flags(self) -> int:
        flags = 0
        if self.soc < 100:
            flags |= FLAG_CHARGE_READY
        if self.soc > self.empty_soc:
            flags |= FLAG_DISCHARGE_READY
        if self.soc >= 100:
            flags |= FLAG_FULL
        if self.soc <= self.empty_soc:
            flags |= FLAG_EMPTY
        return flags

    # ----------------------------------------------------------- protocol
    @staticmethod
    def _exception(fc: int, code: int) -> bytes:
        return bytes((fc | 0x80, code))

    def _readable(self, address: int, count: int) -> bool:
        wanted = range(address, address + count)
        return any(wanted[0] in r and wanted[-1] in r for r in READABLE_RANGES)

    def _silenced(self, fc: int, address: int, count: int) -> bool:
        if fc not in (FC_READ_HOLDING, FC_READ_INPUT, FC_EXT_READ):
            return False
        wanted = set(range(address, address + count))
        return any(wanted & set(r) for r in self.silent_ranges)

    def _read(self, fc: int, data: bytes) -> bytes | None:
        address, count = struct.unpack(">HH", data[:4])
        if self._silenced(fc, address, count):
            return None
        if not self._readable(address, count):
            return self._exception(fc, 0x02)
        if count > 1 and any(
            set(range(address, address + count)) & set(r) for r in self.reject_span_ranges
        ):
            return self._exception(fc, 0x02)
        values = [self.regs[a] for a in range(address, address + count)]
        if self.short_reply:
            values = values[:-1]
        payload = b"".join(struct.pack(">H", v) for v in values)
        return bytes((fc, len(payload))) + payload

    def _passive(self, data: bytes) -> bytes:
        cmd, param = struct.unpack(">HH", data[:4])
        if self.command_counts_as_heartbeat:
            self.last_heartbeat = self.now()
        self.check_watchdog()

        status = STATUS_ACCEPTED
        if cmd not in (PASSIVE_STANDBY, PASSIVE_DISCHARGE, PASSIVE_CHARGE, PASSIVE_AUTO):
            status = STATUS_INVALID
        elif cmd in (PASSIVE_STANDBY, PASSIVE_AUTO) and self.strict_params and param != PASSIVE_NO_VALUE:
            status = STATUS_INVALID
        elif cmd in (PASSIVE_CHARGE, PASSIVE_DISCHARGE) and param > MAX_PASSIVE_WATTS:
            status = STATUS_INVALID
        elif self.working_mode != WORKING_MODE_PASSIVE:
            status = STATUS_NOT_PASSIVE
        elif self.busy:
            status = STATUS_BUSY

        if status == STATUS_ACCEPTED:
            self.setpoint_at = self.now()
            if cmd == PASSIVE_STANDBY:
                self.passive_state, self.passive_watts = "standby", 0
                self._set_battery_watts(0)
            elif cmd == PASSIVE_AUTO:
                self.passive_state, self.passive_watts = "auto", 0
            elif cmd == PASSIVE_CHARGE:
                self.passive_state, self.passive_watts = "charge", param
                self._set_battery_watts(param)
            else:
                self.passive_state, self.passive_watts = "discharge", param
                self._set_battery_watts(-param)

        if self.passive_reply == "echo":
            return bytes((FC_PASSIVE,)) + data[:4]
        word = (self._flags() << 8) | status
        return bytes((FC_PASSIVE, 2)) + struct.pack(">H", word)

    def _write_multiple(self, data: bytes) -> bytes | None:
        address, count = struct.unpack(">HH", data[:4])
        if address == 0x8000 and count == 0:
            # Sofar's "read system time" reuses FC 0x10 (protocol doc 2.11).
            t = self.rtc()
            clock = bytes(
                _bcd(v)
                for v in (t.second, t.minute, t.hour, t.isoweekday(), t.day, t.month, t.year % 100)
            )
            return bytes((FC_WRITE_MULTIPLE, len(clock))) + clock
        byte_count = data[4]
        raw = data[5 : 5 + byte_count]
        values = [struct.unpack(">H", raw[i : i + 2])[0] for i in range(0, len(raw), 2)]
        if not self._readable(address, count) or address < 0x1200:
            return self._exception(FC_WRITE_MULTIPLE, 0x02)
        for offset, value in enumerate(values):
            self.regs[address + offset] = value
        self.writes.append((address, values))
        return bytes((FC_WRITE_MULTIPLE,)) + data[:4]

    def _write_single(self, data: bytes) -> bytes:
        address, value = struct.unpack(">HH", data[:4])
        if not self._readable(address, 1) or address < 0x1200:
            return self._exception(FC_WRITE_SINGLE, 0x02)
        self.regs[address] = value
        self.writes.append((address, [value]))
        return bytes((FC_WRITE_SINGLE,)) + data[:4]

    def _event_log(self, data: bytes) -> bytes:
        # Event ID 57 (battery over-temperature) on 2026-10-01 12:34:56, BCD.
        record = bytes((_bcd(57), _bcd(26), _bcd(10), _bcd(1), _bcd(4), _bcd(12), _bcd(34), _bcd(56)))
        return bytes((FC_EVENT_LOG, len(record))) + record

    def respond(self, unit: int, pdu: bytes) -> bytes | None:
        """The inverter's reply PDU for one request, or None for silence."""
        fc, data = pdu[0], pdu[1:]
        frame = Frame(self.now(), unit, fc, bytes(data))
        self.frames.append(frame)
        self.check_watchdog()

        if unit == UNIT_BROADCAST:
            return None  # broadcasts are accepted but never answered
        if unit != self.slave or self.silent:
            reply = self._gateway_timeout(fc)
            frame.answered = reply is not None  # answered by the gateway itself
            return reply

        if fc == FC_HEARTBEAT:
            if data[:4] == HEARTBEAT_PAYLOAD:
                self.last_heartbeat = self.now()
            frame.answered = self.heartbeat_reply is not None
            return pdu if self.heartbeat_reply == "echo" else None

        if self.exception_code is not None and (
            self.exception_fcs is None or fc in self.exception_fcs
        ):
            frame.answered = True
            return self._exception(fc, self.exception_code)

        if fc in (FC_READ_HOLDING, FC_READ_INPUT, FC_EXT_READ):
            reply = self._read(fc, data)
        elif fc == FC_PASSIVE:
            reply = self._passive(data)
        elif fc == FC_WRITE_MULTIPLE:
            reply = self._write_multiple(data)
        elif fc == FC_WRITE_SINGLE:
            reply = self._write_single(data)
        elif fc == FC_EVENT_LOG:
            reply = self._event_log(data)
        else:
            reply = self._exception(fc, 0x01)
        if reply is None:
            reply = self._gateway_timeout(fc)
            frame.answered = reply is not None
            return reply
        frame.answered = True
        return reply

    def _gateway_timeout(self, fc: int) -> bytes | None:
        if self.gateway_timeout_exception:
            return self._exception(fc, 0x0B)
        return None

    # ----------------------------------------------------------- transport
    async def _handle(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        self.connections += 1
        self._writers.add(writer)
        try:
            while True:
                header = await reader.readexactly(7)
                tid, pid, length, unit = struct.unpack(">HHHB", header)
                pdu = await reader.readexactly(length - 1)
                reply = self.respond(unit, pdu)
                if reply is None:
                    continue
                reply_unit = (unit + 1) & 0xFF if self.wrong_unit else unit
                writer.write(struct.pack(">HHHB", tid, pid, len(reply) + 1, reply_unit) + reply)
                await writer.drain()
        except (asyncio.IncompleteReadError, ConnectionError):
            pass
        finally:
            self._writers.discard(writer)
            writer.close()

    async def start(self, port: int = 0) -> int:
        self._server = await asyncio.start_server(
            self._handle, "127.0.0.1", port or self.port, reuse_address=True
        )
        self.port = self._server.sockets[0].getsockname()[1]
        return self.port

    def drop_connections(self) -> None:
        for writer in list(self._writers):
            writer.close()

    async def stop(self) -> None:
        if self._server is not None:
            self._server.close()  # stop accepting first, then drop clients
            self._server.close_clients()
            self.drop_connections()
            await self._server.wait_closed()
            self._server = None
        self.drop_connections()
