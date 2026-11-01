# STM32 ⇄ Mini-PC serial protocol

This is the contract between the **embedded team** (firmware on the DJI RoboMaster
Development Board C) and the **CV team** (Python on the mini-PC). If either side changes
the byte layout, update this file in the same pull request.

| Item | Value |
|---|---|
| MCU port | `USART6`, pins PG14 TX / PG9 RX (check the board manual for which 4-pin connector) |
| PC port | usually `/dev/ttyUSB0` (USB-TTL adapter) |
| Settings | 115200 baud, 8 data bits, no parity, 1 stop bit |
| Firmware code | `firmware/application/referee_usart_task.c` (+ `.h`) |
| CV code | `UART_UTIL.py` in `standard-cv` / `Robox2025` |

Angle units are **radians** in both directions.

---

## 1. Downlink: aim command (PC → STM32)

Binary, fixed length **14 bytes**, sent once per processed camera frame.

| Byte | Field | Type | Notes |
|---|---|---|---|
| 0 | header | `0xA5` | |
| 1 | header | `0x5A` | |
| 2 | length | `u8` | CV currently writes `0x0D` (13). **Not checked** by firmware. |
| 3–6 | yaw | `float32`, big-endian | target angle relative to the camera, rad |
| 7–10 | pitch | `float32`, big-endian | target angle relative to the camera, rad |
| 11 | checksum | `u8` | `(byte3 + … + byte10) & 0xFF` |
| 12 | detect | `u8` | `0x01` target found, `0x00` not found |
| 13 | footer | `0xFF` | |

Python reference (what CV sends today):

```python
import struct
def pack_aim(yaw_rad: float, pitch_rad: float, found: bool) -> bytes:
    body = struct.pack('>ff', yaw_rad, pitch_rad)
    return bytes([0xA5, 0x5A, 0x0D]) + body + bytes([sum(body) & 0xFF, int(found), 0xFF])
```

### How the firmware uses it
- RX is DMA (circular) into a 50-byte buffer. When the buffer fills, the parser searches it
  **from the end** and takes the newest packet whose header, footer and checksum are valid.
- Valid packet → `cv_Data.yaw / pitch / find_target`, sets `new_cv_data_flag`, feeds the
  link watchdog `USER_USART_DATA_TOE` (offline after 800 ms without a packet).
- In `GIMBAL_AUTO` (right switch mid, or right mouse button held):
  `target = current_gimbal_angle − cv_angle`, smoothed by a Kalman filter.
  If the link is offline the gimbal holds its current angle.
- When no target is found CV sends `yaw = pitch = 0`, which makes the gimbal hold.
  `find_target` is stored but not used yet.

### Known limitations
- Latency: the parser only runs when the 50-byte buffer is full (every ~3.5 packets).
  Fix: use the UART IDLE interrupt (see `docs/known-issues.md`).
- A packet that wraps around the end of the circular buffer is skipped.

---

## 2. Uplink: IMU attitude (STM32 → PC)

**ASCII text**, no separator between frames:

```
A5<yaw>,<pitch>,<roll>A5<yaw>,<pitch>,<roll>A5...
```

- `yaw, pitch, roll` from `INS_task` (`INS_angle[0..2]`), radians, printed with `%f`
  (6 decimals). Note `A5` here is the two **characters** 'A' '5', not the byte `0xA5`.
- Yaw is relative to the heading at power-on (no magnetometer heading correction).
- CV parses it by finding two consecutive `A5` and splitting on `,`.

### Known problems (to fix — owned by the embedded team)
1. TX DMA is configured **circular**: the DMA keeps re-sending `dma_buf` with the length of
   the *first* string, while `sprintf` rewrites it every 1 ms. Frames can be cut off or
   mixed. Frame length changes when a sign appears (`-`), so the tail is often truncated.
2. No checksum, no timestamp, no fixed rate.
3. CV side: if the PC does not read continuously, the OS buffer fills with old IMU data.
   `Robox2025` reads in a background thread (`IMUReader`), which mostly avoids this;
   `standard-cv` does not.

### Proposed v2 (NOT implemented yet)
Binary, fixed length, sent at a fixed rate (e.g. 200 Hz) with TX DMA in **normal** mode:

| Byte | Field | Type |
|---|---|---|
| 0–1 | header `0x5A 0xA5` | |
| 2–5 | yaw | `float32` BE, rad |
| 6–9 | pitch | `float32` BE, rad |
| 10–13 | roll | `float32` BE, rad |
| 14–17 | timestamp | `u32` BE, ms since boot (`HAL_GetTick`) |
| 18 | checksum | `(byte2 + … + byte17) & 0xFF` |
| 19 | footer `0xFF` | |

The timestamp lets CV pair each camera frame with the IMU sample closest in time.
Switching to v2 needs a matching change in CV's `get_imu()`; agree on a date with the CV team.

---

## Testing without the CV team
See [`tools/`](tools/):

- `fake_cv.py` — sends aim packets (fixed or sweeping angles) to the robot.
- `imu_monitor.py` — prints the IMU uplink and its rate, and counts broken frames.
