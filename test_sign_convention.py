#!/usr/bin/env python3
"""
test_sign_convention.py — Determine which joints have sign flips between
the myCobot 280-M5 firmware convention and the URDF joint convention.

For each joint:
  1. Home all joints to 0°
  2. Send +30° to that joint
  3. Read back the actual reported angle
  4. If reported ≈ +30° → sign convention MATCHES
  5. If reported ≈ -30° → sign is INVERTED — needs correction in MyCobotBridge

Usage:
  python3 test_sign_convention.py
"""

import serial
import struct
import time
import termios
import fcntl
import os

PORT      = "/dev/cu.usbserial-52D20281481"
BAUD      = 115200
SPEED     = 20
TARGET    = 30.0   # degrees — far enough to be unambiguous, small enough to be safe


def configure_port(ser: serial.Serial) -> None:
    """Apply the same termios settings the Swift app uses:
    VMIN=0, VTIME=2 (200 ms read timeout), clear O_NONBLOCK."""
    fd = ser.fileno()
    # Clear O_NONBLOCK so writes block until UART drains.
    flags = fcntl.fcntl(fd, fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, flags & ~os.O_NONBLOCK)
    # Set VMIN=0, VTIME=2 via termios.
    attrs = termios.tcgetattr(fd)
    attrs[6][termios.VMIN]  = 0   # don't wait for minimum bytes
    attrs[6][termios.VTIME] = 2   # 200 ms timeout (units of 100 ms)
    termios.tcsetattr(fd, termios.TCSANOW, attrs)


def encode_angle(degrees: float) -> tuple[int, int]:
    raw = int(round(degrees * 100))
    raw = max(-32768, min(32767, raw))
    encoded = raw if raw >= 0 else raw + 65536
    return (encoded >> 8) & 0xFF, encoded & 0xFF


def set_angle_frame(joint_id: int, degrees: float, speed: int = SPEED) -> bytes:
    hi, lo = encode_angle(degrees)
    return bytes([0xFE, 0xFE, 0x06, 0x21, joint_id, hi, lo, max(1, min(100, speed)), 0xFA])


def get_angles(ser: serial.Serial) -> list[float] | None:
    fd = ser.fileno()
    # Flush stale input exactly as the Swift app does before each poll.
    termios.tcflush(fd, termios.TCIFLUSH)
    ser.write(bytes([0xFE, 0xFE, 0x02, 0x20, 0xFA]))
    time.sleep(0.08)
    # Read in up to 4 chunks (mirrors Swift's retry loop).
    data = bytearray()
    for _ in range(4):
        chunk = ser.read(64)
        data.extend(chunk)
        if len(data) >= 17:
            break
    for i in range(len(data) - 16):
        if (data[i] == 0xFE and data[i+1] == 0xFE
                and data[i+3] == 0x20 and data[i+16] == 0xFA):
            return [struct.unpack_from(">h", data, i + 4 + j * 2)[0] / 100.0
                    for j in range(6)]
    return None


def home_all(ser: serial.Serial) -> None:
    for jid in range(1, 7):
        ser.write(set_angle_frame(jid, 0.0, SPEED))
        time.sleep(0.05)
    time.sleep(3.0)


def main():
    corrections = {}

    print(f"Opening {PORT} …")
    with serial.Serial(PORT, BAUD, bytesize=8, parity='N', stopbits=1, timeout=1.0) as ser:
        configure_port(ser)
        time.sleep(0.3)

        print("\n=== Homing all joints to 0° ===")
        home_all(ser)
        baseline = get_angles(ser)
        print(f"Baseline: {baseline}")
        if baseline is None:
            print("ERROR: no response from arm. Is it powered and in Transponder mode?")
            return

        print(f"\n=== Sign convention test (sending +{TARGET}° to each joint) ===\n")

        joint_labels = [
            "joint2_to_joint1  (base rotation)",
            "joint3_to_joint2  (shoulder)",
            "joint4_to_joint3  (elbow upper)",
            "joint5_to_joint4  (elbow lower)",
            "joint6_to_joint5  (wrist pitch)",
            "joint6output_to_joint6 (wrist roll)",
        ]

        for jid in range(1, 7):
            home_all(ser)

            ser.write(set_angle_frame(jid, TARGET, SPEED))
            time.sleep(2.0)

            reported = get_angles(ser)
            if reported is None:
                print(f"J{jid} ({joint_labels[jid-1]}): NO RESPONSE")
                continue

            actual = reported[jid - 1]
            match = abs(actual - TARGET) < 5.0
            inv   = abs(actual + TARGET) < 5.0
            if match:
                status = "✅  MATCH   — no correction needed"
            elif inv:
                status = "❌  INVERTED — needs sign flip in MyCobotBridge"
                corrections[jid] = -1
            else:
                status = f"❓  UNEXPECTED (got {actual:+.2f}°) — check arm/firmware"

            print(f"J{jid} {joint_labels[jid-1]}")
            print(f"   Sent: +{TARGET:.1f}°  Reported back: {actual:+.2f}°  → {status}\n")

        print("=== Homing to 0° (cleanup) ===")
        home_all(ser)

    print("\n=== Summary ===")
    if not corrections:
        print("  All joints match — no sign corrections needed in MyCobotBridge.")
    else:
        print("  Joints needing sign correction in MyCobotBridge.sendAngle():")
        print("  Add this array and apply the multiplier before encoding:\n")
        signs = [corrections.get(j, 1) for j in range(1, 7)]
        print(f"  let jointSignCorrection: [Double] = {signs}")
        print("\n  In sendAngle(), replace:")
        print("    let deg = radians * (180.0 / .pi)")
        print("  with:")
        print("    let deg = radians * (180.0 / .pi) * jointSignCorrection[jointIndex]")
        print("\n  And in pollAngles() response parsing, invert the same joints.")


if __name__ == "__main__":
    main()
