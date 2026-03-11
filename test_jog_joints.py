#!/usr/bin/env python3
"""
test_jog_joints.py — Directly test jogging individual joints on a myCobot 280-M5.

Protocol (CMD 0x21 SET_ANGLE):
  [0xFE, 0xFE, 0x06, 0x21, JOINT_ID, ANGLE_H, ANGLE_L, SPEED, 0xFA]
  JOINT_ID: 1-based (1–6)
  Angle: 0.01 degree units, big-endian signed int16
  LEN byte = 0x06 (4 params + 2 overhead)

Protocol (CMD 0x20 GET_ANGLES):
  Request:  [0xFE, 0xFE, 0x02, 0x20, 0xFA]
  Response: [0xFE, 0xFE, LEN, 0x20, J1H, J1L, J2H, J2L, ..., J6H, J6L, 0xFA]  (17 bytes)

Usage:
  pip install pyserial
  python3 test_jog_joints.py
"""

import serial
import struct
import time

# ── Config ────────────────────────────────────────────────────────────────────
PORT      = "/dev/cu.usbserial-52D20281481"
BAUD      = 115200
SPEED     = 20      # robot speed 1–100
TIMEOUT_S = 0.5
# ─────────────────────────────────────────────────────────────────────────────


def encode_angle(degrees: float) -> tuple[int, int]:
    """Convert degrees to 0.01° big-endian int16 bytes."""
    raw = int(round(degrees * 100))
    raw = max(-32768, min(32767, raw))
    encoded = raw if raw >= 0 else raw + 65536
    return (encoded >> 8) & 0xFF, encoded & 0xFF


def set_angle_frame(joint_id: int, degrees: float, speed: int) -> bytes:
    """Build a SET_ANGLE (CMD 0x21) frame for one joint."""
    hi, lo = encode_angle(degrees)
    spd = max(1, min(100, speed))
    return bytes([0xFE, 0xFE, 0x06, 0x21, joint_id, hi, lo, spd, 0xFA])


def get_angles(ser: serial.Serial) -> list[float] | None:
    """Send GET_ANGLES (CMD 0x20) and parse the 17-byte response.
    Returns 6 angles in degrees, or None on failure."""
    ser.reset_input_buffer()
    ser.write(bytes([0xFE, 0xFE, 0x02, 0x20, 0xFA]))
    time.sleep(0.08)
    data = ser.read(64)
    for i in range(len(data) - 16):
        if (data[i] == 0xFE and data[i+1] == 0xFE
                and data[i+3] == 0x20 and data[i+16] == 0xFA):
            angles = []
            for j in range(6):
                raw = struct.unpack_from(">h", data, i + 4 + j * 2)[0]
                angles.append(raw / 100.0)
            return angles
    return None


def send_and_confirm(ser: serial.Serial, joint_id: int, target_deg: float,
                     label: str) -> None:
    """Send a jog command then read back actual angles and report."""
    print(f"\n  → Jogging Joint {joint_id} ({label}) to {target_deg:+.1f}°")
    before = get_angles(ser)
    print(f"    Before: {fmt(before)}")

    frame = set_angle_frame(joint_id, target_deg, SPEED)
    ser.write(frame)
    print(f"    Frame:  {frame.hex(' ').upper()}")

    time.sleep(1.5)   # give the arm time to move
    after = get_angles(ser)
    print(f"    After:  {fmt(after)}")

    if before and after:
        delta = after[joint_id - 1] - before[joint_id - 1]
        moved = abs(delta) > 1.0
        status = "✅ MOVED" if moved else "❌ DID NOT MOVE"
        print(f"    Δ J{joint_id} = {delta:+.2f}°  →  {status}")


def fmt(angles: list[float] | None) -> str:
    if angles is None:
        return "NO RESPONSE"
    return "  ".join(f"J{i+1}={a:+7.2f}°" for i, a in enumerate(angles))


def main():
    print(f"Opening {PORT} at {BAUD} baud …")
    with serial.Serial(PORT, BAUD, bytesize=8, parity='N',
                       stopbits=1, timeout=TIMEOUT_S) as ser:
        time.sleep(0.1)

        print("\n=== Baseline angles ===")
        baseline = get_angles(ser)
        print(f"  {fmt(baseline)}")

        # Return to zero first so we have room to move in both directions
        print("\n=== Homing all joints to 0° (one by one, 50ms apart) ===")
        for jid in range(1, 7):
            ser.write(set_angle_frame(jid, 0.0, SPEED))
            time.sleep(0.05)
        time.sleep(2.5)
        print(f"  After home: {fmt(get_angles(ser))}")

        # ── Test joints 1–6 ───────────────────────────────────────────────
        tests = [
            (1, +30.0, "rotation"),
            (2, +30.0, "shoulder"),
            (3, +30.0, "elbow upper"),   # <-- reported problem
            (4, +30.0, "elbow lower"),   # <-- reported problem
            (5, +30.0, "wrist pitch"),
            (6, +30.0, "wrist roll"),
        ]

        for joint_id, target, label in tests:
            send_and_confirm(ser, joint_id, target, label)
            # Return to 0 between tests so joints don't compound
            time.sleep(0.1)
            ser.write(set_angle_frame(joint_id, 0.0, SPEED))
            time.sleep(1.5)

        # ── Deep diagnostic for joint 3 ───────────────────────────────────
        print("\n=== Joint 3 deep diagnostic ===")
        print("  (tries small increments and negative direction)")
        for target, note in [(+5.0, "+5°"), (-5.0, "-5°"), (+10.0, "+10°"), (-10.0, "-10°"),
                              (+20.0, "+20°"), (-20.0, "-20°")]:
            before = get_angles(ser)
            j3_before = before[2] if before else None
            ser.write(set_angle_frame(3, target, SPEED))
            time.sleep(1.5)
            after = get_angles(ser)
            j3_after = after[2] if after else None
            if j3_before is not None and j3_after is not None:
                delta = j3_after - j3_before
                moved = abs(delta) > 0.5
                print(f"  J3 → {note}: before={j3_before:+.2f}° after={j3_after:+.2f}° Δ={delta:+.2f}°  {'✅' if moved else '❌'}")
            else:
                print(f"  J3 → {note}: NO RESPONSE")
            ser.write(set_angle_frame(3, 0.0, SPEED))
            time.sleep(1.5)

        print("\n=== Test complete ===")
        print(f"  Final angles: {fmt(get_angles(ser))}")


if __name__ == "__main__":
    main()
