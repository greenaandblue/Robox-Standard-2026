#!/usr/bin/env python3
"""Pretend to be the CV mini-PC: send aim packets to the STM32 over serial.

Lets the embedded team test the gimbal's auto-aim path without running CV.
Packet layout: see ../PROTOCOL.md, section 1.

Examples:
    python3 fake_cv.py /dev/ttyUSB0                      # hold: yaw=0, pitch=0, found
    python3 fake_cv.py /dev/ttyUSB0 --yaw 2 --pitch -1   # constant offset (degrees)
    python3 fake_cv.py /dev/ttyUSB0 --sweep 5 --period 2 # yaw sine sweep, +-5 deg, 2 s
    python3 fake_cv.py /dev/ttyUSB0 --lost               # send "no target" packets
    python3 fake_cv.py --dry-run --yaw 1                 # print packets, no serial port

Put the robot in GIMBAL_AUTO (right switch mid, or hold right mouse button) first.
"""
import argparse
import math
import struct
import sys
import time

PACKET_LEN = 14


def pack_aim(yaw_rad: float, pitch_rad: float, found: bool) -> bytes:
    """Build one 14-byte aim packet exactly like CV's UART_UTIL.send_data."""
    body = struct.pack(">ff", yaw_rad, pitch_rad)
    packet = bytes([0xA5, 0x5A, 0x0D]) + body + bytes([sum(body) & 0xFF, 1 if found else 0, 0xFF])
    assert len(packet) == PACKET_LEN
    return packet


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", nargs="?", help="serial port, e.g. /dev/ttyUSB0 or COM5")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--rate", type=float, default=60.0, help="packets per second (default 60, like a camera)")
    ap.add_argument("--yaw", type=float, default=0.0, help="yaw offset in DEGREES (sent as radians)")
    ap.add_argument("--pitch", type=float, default=0.0, help="pitch offset in DEGREES (sent as radians)")
    ap.add_argument("--sweep", type=float, default=0.0, help="add a yaw sine of this amplitude in degrees")
    ap.add_argument("--period", type=float, default=2.0, help="sweep period in seconds")
    ap.add_argument("--lost", action="store_true", help="send detect=0 (no target), angles 0")
    ap.add_argument("--duration", type=float, default=0.0, help="stop after N seconds (0 = until Ctrl-C)")
    ap.add_argument("--dry-run", action="store_true", help="print packets instead of opening a port")
    args = ap.parse_args()

    if not args.dry_run and not args.port:
        ap.error("give a serial port, or use --dry-run")

    ser = None
    if not args.dry_run:
        try:
            import serial  # pyserial
        except ImportError:
            print("pyserial missing: pip install pyserial", file=sys.stderr)
            return 1
        ser = serial.Serial(args.port, args.baud, timeout=0)

    dt = 1.0 / args.rate
    t0 = time.monotonic()
    sent = 0
    try:
        while True:
            t = time.monotonic() - t0
            if args.duration and t >= args.duration:
                break
            if args.lost:
                yaw_deg, pitch_deg, found = 0.0, 0.0, False
            else:
                yaw_deg = args.yaw + args.sweep * math.sin(2 * math.pi * t / args.period)
                pitch_deg, found = args.pitch, True

            pkt = pack_aim(math.radians(yaw_deg), math.radians(pitch_deg), found)
            if ser:
                ser.write(pkt)
                ser.reset_input_buffer()  # the board streams IMU text back; we don't need it here
            if args.dry_run or sent % int(max(args.rate, 1)) == 0:
                print(f"t={t:6.2f}s yaw={yaw_deg:+7.3f}deg pitch={pitch_deg:+7.3f}deg "
                      f"found={int(found)}  {pkt.hex(' ')}")
            sent += 1
            if args.dry_run and sent >= 5 and not args.duration:
                break
            time.sleep(max(0.0, t0 + sent * dt - time.monotonic()))
    except KeyboardInterrupt:
        pass
    finally:
        if ser:
            ser.close()
    print(f"sent {sent} packets")
    return 0


if __name__ == "__main__":
    sys.exit(main())
