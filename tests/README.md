# Tests

```bash
python3.14 -m venv .venv
.venv/bin/pip install -r requirements_test.txt
.venv/bin/python -m pytest -q -rfExX
```

- `fake_sofar.py` is a fake Modbus TCP gateway (MBAP framing) with a fake
  ME3000SP behind it: FC 0x03/0x04 reads, FC 0x42 passive commands with a
  status word, the FC 0x49 heartbeat, the 60 s passive watchdog, Sofar's
  proprietary reads, and injectable faults (silence, exceptions, short replies,
  wrong unit ID, busy, not in Passive mode, dropped connections).
- `conftest.py` swaps pymodbus's client for the same class with a 0.05 s
  response timeout, so silence and retries run in milliseconds. The integration
  uses 5 s, so a count of timed-out requests x 5 s is the real-world delay.
- Known bugs are `xfail(strict=True)` tests with the reason in the marker; a fix
  makes the test XPASS, which fails the run until the marker is removed.
