# Testing

## Layout

| Suite | Dependencies | Coverage |
|-------|--------------|----------|
| `tests/test_security.py` | Flask only | Path traversal, auth, rate limiting, public routes |
| `tests/test_data_quality.py` | stdlib only | IQR/Z-score/rate filters, quality score, temp compensation |
| `tests/test_driver_mock.py` | pyserial | Frame parsing, mocked read loop, reconnection, error propagation |
| `tests/test_lidar.py` | numpy, scipy, Open3D | SLAM, localization, map manager, object detection, profiles |

## Running

```bash
# Everything (needs the full scientific stack)
python -m pytest tests/ -v

# CI-friendly subset (no numpy/Open3D)
python -m pytest tests/test_security.py tests/test_data_quality.py tests/test_driver_mock.py -v
```

## Hardware Mocking

`tests/test_driver_mock.py` provides `FakeSerial`, a scripted byte-stream stand-in
for `serial.Serial`, and `build_frame()` to generate valid 9-byte TFmini-S frames
with correct checksums. This lets the read loop, buffer handling, and
reconnection logic be tested with **no real hardware**.

Example:

```python
fake = FakeSerial(build_frame(150) * 5)   # five 1.5 m readings
driver.serial_conn = fake
# ... run _read_loop in a thread, assert callbacks fired
```

To simulate a mid-stream disconnect, pass `fail_after=N` so `in_waiting` raises
`SerialException` after N reads.

## CI Notes

- The dependency-light subset is intended for CI where installing Open3D
  (~hundreds of MB) is undesirable.
- No test touches a real serial port; all hardware interaction is mocked.
- Data directories are created under pytest's `tmp_path` fixtures, never the
  real `DATA_DIR`.

## Adding Tests

- Prefer the dependency-light pattern (mock `serial`, avoid importing SLAM
  modules) so the test runs in CI.
- Security-sensitive code paths (path handling, auth) should get an explicit
  negative test (traversal attempt, missing token).
