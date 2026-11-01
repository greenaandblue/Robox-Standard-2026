#!/usr/bin/env python3
"""Watch the IMU uplink from the STM32 (current ASCII format, ../PROTOCOL.md section 2).

Prints yaw/pitch/roll in degrees, frames per second, and how many frames were broken
(missing fields or not numbers). Use it to check the uplink before/after firmware changes.

Examples:
    python3 imu_monitor.py /dev/ttyUSB0
    python3 imu_monitor.py /dev/ttyUSB0 --raw          # also dump the raw text
    python3 imu_monitor.py --selftest                   # check the parser, no hardware
"""
import argparse
import math
import sys
import time

MARK = "A5"


def split_frames(buffer: str):
    """Split the text stream into complete frames.

    A frame is the text between one 'A5' and the next 'A5'. The part after the last
    'A5' may still be arriving, so it is returned as the leftover to keep.
    """
    parts = buffer.split(MARK)
    leftover = MARK + parts[-1] if len(parts) > 1 else buffer
    return parts[1:-1], leftover


def parse_frame(frame: str):
    """'0.1,-0.2,0.003' -> (yaw, pitch, roll) in radians, or None if broken."""
    fields = frame.split(",")
    if len(fields) != 3:
        return None
    try:
        values = tuple(float(f) for f in fields)
    except ValueError:
        return None
    if not all(math.isfinite(v) and abs(v) <= 4.0 for v in values):  # radians, |angle| <= pi
        return None
    return values


def selftest() -> int:
    stream = "1,2A50.100000,-0.200000,0.003000A5-3.141593,0.0,0.0A5bad,1A50.5,0.6,0."
    frames, leftover = split_frames(stream)
    parsed = [parse_frame(f) for f in frames]
    assert parsed == [(0.1, -0.2, 0.003), (-3.141593, 0.0, 0.0), None], parsed
    assert leftover == "A50.5,0.6,0.", leftover
    print("selftest ok")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", nargs="?", help="serial port, e.g. /dev/ttyUSB0 or COM5")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--raw", action="store_true", help="print raw text as it arrives")
    ap.add_argument("--selftest", action="store_true", help="test the parser without hardware")
    args = ap.parse_args()

    if args.selftest:
        return selftest()
    if not args.port:
        ap.error("give a serial port, or use --selftest")

    try:
        import serial  # pyserial
    except ImportError:
        print("pyserial missing: pip install pyserial", file=sys.stderr)
        return 1

    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    ser.reset_input_buffer()  # drop old data so we see the live stream
    buf, good, bad = "", 0, 0
    last_print, last_good = time.monotonic(), 0
    latest = None
    try:
        while True:
            chunk = ser.read(ser.in_waiting or 1).decode("ascii", "replace")
            if args.raw and chunk:
                sys.stdout.write(chunk)
            buf += chunk
            frames, buf = split_frames(buf)
            for f in frames:
                v = parse_frame(f)
                if v is None:
                    bad += 1
                else:
                    good += 1
                    latest = v
            now = time.monotonic()
            if now - last_print >= 0.5 and latest:
                rate = (good - last_good) / (now - last_print)
                y, p, r = (math.degrees(a) for a in latest)
                print(f"yaw {y:+8.2f}  pitch {p:+8.2f}  roll {r:+8.2f} deg | "
                      f"{rate:6.0f} frames/s | broken {bad}/{good + bad}")
                last_print, last_good = now, good
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
