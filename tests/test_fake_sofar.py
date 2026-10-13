"""The fake gateway itself: MBAP framing, replies, watchdog and faults.

These talk raw Modbus TCP over a socket so the fake is checked independently of
pymodbus and of the integration.
"""

from __future__ import annotations

import asyncio
import struct

import pytest

from .fake_sofar import (
    FC_HEARTBEAT,
    FC_PASSIVE,
    HEARTBEAT_PAYLOAD,
    PASSIVE_AUTO,
    PASSIVE_CHARGE,
    PASSIVE_NO_VALUE,
    FakeSofarGateway,
)


async def _transact(fake: FakeSofarGateway, unit: int, pdu: bytes, tid: int = 7, timeout: float = 0.3):
    reader, writer = await asyncio.open_connection("127.0.0.1", fake.port)
    try:
        writer.write(struct.pack(">HHHB", tid, 0, len(pdu) + 1, unit) + pdu)
        await writer.drain()
        try:
            header = await asyncio.wait_for(reader.readexactly(7), timeout)
        except TimeoutError:
            return None
        rtid, pid, length, runit = struct.unpack(">HHHB", header)
        body = await reader.readexactly(length - 1)
        assert rtid == tid and pid == 0
        return runit, body
    finally:
        writer.close()


async def test_read_holding_registers_mbap(gateway: FakeSofarGateway) -> None:
    unit, body = await _transact(gateway, 1, bytes((0x03, 0x02, 0x06, 0x00, 0x02)))
    assert unit == 1
    assert body == bytes((0x03, 4)) + struct.pack(">HH", 2405, 512)


async def test_input_register_read_and_unknown_address(gateway: FakeSofarGateway) -> None:
    _, body = await _transact(gateway, 1, bytes((0x04, 0x10, 0xB0, 0x00, 0x01)))
    assert body == bytes((0x04, 2, 0x00, 0x01))
    _, body = await _transact(gateway, 1, bytes((0x03, 0x30, 0x00, 0x00, 0x01)))
    assert body == bytes((0x83, 0x02))


async def test_passive_status_word(gateway: FakeSofarGateway) -> None:
    pdu = bytes((FC_PASSIVE,)) + struct.pack(">HH", PASSIVE_CHARGE, 2000)
    _, body = await _transact(gateway, 1, pdu)
    # SOC 60 %: charge-ready and discharge-ready set, status accepted.
    assert body == bytes((FC_PASSIVE, 2, 0x03, 0x00))
    assert gateway.passive_state == "charge" and gateway.passive_watts == 2000

    gateway.working_mode = 0
    _, body = await _transact(gateway, 1, pdu)
    assert body[2:] == bytes((0x03, 0x01))  # not in passive mode

    gateway.working_mode = 3
    gateway.busy = True
    _, body = await _transact(gateway, 1, pdu)
    assert body[3] == 0x03

    gateway.busy = False
    _, body = await _transact(gateway, 1, bytes((FC_PASSIVE,)) + struct.pack(">HH", PASSIVE_AUTO, 0))
    assert body[3] == 0x04  # auto must carry 0x5555


async def test_passive_flags_full_and_empty(gateway: FakeSofarGateway) -> None:
    pdu = bytes((FC_PASSIVE,)) + struct.pack(">HH", PASSIVE_AUTO, PASSIVE_NO_VALUE)
    gateway.regs[0x0210] = 100
    _, body = await _transact(gateway, 1, pdu)
    assert body[2] == 0b0110  # discharge-ready + full
    gateway.regs[0x0210] = 5
    _, body = await _transact(gateway, 1, pdu)
    assert body[2] == 0b1001  # charge-ready + empty


async def test_passive_echo_variant(gateway: FakeSofarGateway) -> None:
    gateway.passive_reply = "echo"
    pdu = bytes((FC_PASSIVE,)) + struct.pack(">HH", PASSIVE_CHARGE, 2000)
    _, body = await _transact(gateway, 1, pdu)
    assert body == pdu


async def test_heartbeat_and_watchdog(gateway: FakeSofarGateway) -> None:
    await _transact(gateway, 1, bytes((FC_PASSIVE,)) + struct.pack(">HH", PASSIVE_CHARGE, 1000))
    assert await _transact(gateway, 1, bytes((FC_HEARTBEAT,)) + HEARTBEAT_PAYLOAD) is None
    gateway.advance(59)
    assert gateway.passive_state == "charge"
    gateway.advance(2)
    assert gateway.passive_state == "standby" and gateway.watchdog_trips == 1
    assert gateway.regs[0x020D] == 0


async def test_heartbeat_keeps_passive_alive(gateway: FakeSofarGateway) -> None:
    await _transact(gateway, 1, bytes((FC_PASSIVE,)) + struct.pack(">HH", PASSIVE_CHARGE, 1000))
    for _ in range(10):
        gateway.advance(30)
        await _transact(gateway, 1, bytes((FC_HEARTBEAT,)) + HEARTBEAT_PAYLOAD)
    assert gateway.passive_state == "charge" and gateway.watchdog_trips == 0


async def test_heartbeat_echo_variant(gateway: FakeSofarGateway) -> None:
    gateway.heartbeat_reply = "echo"
    pdu = bytes((FC_HEARTBEAT,)) + HEARTBEAT_PAYLOAD
    _, body = await _transact(gateway, 1, pdu)
    assert body == pdu


@pytest.mark.parametrize(
    ("fault", "expected"),
    [
        ({"silent": True}, None),
        ({"exception_code": 0x04}, (1, bytes((0x83, 0x04)))),
        ({"short_reply": True}, (1, bytes((0x03, 2)) + struct.pack(">H", 2405))),
        ({"wrong_unit": True}, (2, bytes((0x03, 4)) + struct.pack(">HH", 2405, 512))),
        ({"silent": True, "gateway_timeout_exception": True}, (1, bytes((0x83, 0x0B)))),
    ],
)
async def test_injectable_faults(gateway: FakeSofarGateway, fault, expected) -> None:
    for key, value in fault.items():
        setattr(gateway, key, value)
    reply = await _transact(gateway, 1, bytes((0x03, 0x02, 0x06, 0x00, 0x02)))
    assert reply == expected


async def test_other_unit_ids_get_no_reply(gateway: FakeSofarGateway) -> None:
    assert await _transact(gateway, 2, bytes((0x03, 0x02, 0x06, 0x00, 0x01))) is None
    assert await _transact(gateway, 0x88, bytes((0x02, 0x50, 0x00, 0x00, 0x03))) is None


def test_accept_close_race_filter_is_narrow() -> None:
    """The harness ignores only asyncio's Server._attach assertion."""
    from .conftest import _is_accept_close_race

    def _attach():
        assert None is not None

    def other():
        assert False

    for func, expected in ((_attach, True), (other, False)):
        try:
            func()
        except AssertionError as exc:
            assert _is_accept_close_race({"exception": exc}) is expected
    assert _is_accept_close_race({"exception": ValueError()}) is False
    assert _is_accept_close_race({"message": "no exception"}) is False
