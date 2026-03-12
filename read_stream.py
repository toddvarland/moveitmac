#!/usr/bin/env python3
"""
read_stream.py — Parse the continuous status stream from myCobot 280-M5.

The arm broadcasts 8-byte per-servo packets continuously:
  FF FF JID 04 DATA0 DATA1 DATA2 DATA3
This script reads and prints current positions from that stream.

Usage:
  python3 read_stream.py
"""

import serial
import struct
import time
import termios
import fcntl
import os

PORT = "/dev/cu.usbserial-52D20281481"
BAUD = 115200


def configure_port(ser):
    fd = ser.fileno()
    flags = fcntl.fcntl(fd, fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, flags & ~os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    attrs[6][termios.VMIN]  = 0
    attrs[6][termios.VTIME] = 2
    termios.tcsetattr(fd, termios.TCSANOW, attrs)


def encode_angle(degrees):
    raw = int(round(degrees * 100))
    raw = max(-32768, min(32767, raw))
    enc = raw if raw >= 0 else raw + 65536
    return (enc >> 8) & 0xFF, enc & 0xFF


def set_angle_frame(joint_id, degrees, speed=20):
    hi, lo = encode_angle(degrees)
    return bytes([0xFE, 0xFE, 0x06, 0x21, joint_id, hi, lo, speed, 0xFA])


def read_angles_from_stream(ser, timeout=0.5):
    """Read the raw stream and extract per-joint positions."""
    start = time.time()
    buf = bytearray()
    while time.time() - start < timeout:
        chunk = ser.read(256)
        if chunk:
            buf.extend(chunk)
    
    # Try GET_ANGLES frame format first: FE FE LEN 20 [12 bytes] FA
    for i in range(len(buf) - 16):
        if (buf[i] == 0xFE and buf[i+1] == 0xFE
                and buf[i+3] == 0x20 and buf[i+16] == 0xFA):
            angles = {}
            for j in range(6):
                raw = struct.unpack_from(">h", buf, i + 4 + j * 2)[0]
                angles[j+1] = raw / 100.0
            return angles, "GET_ANGLES response"

    # Fall back: parse streaming FF FF JID 04 ... packets
    # Each packet: FF FF JID 04 B0 B1 B2 B3
    joints = {}
    i = 0
    while i < len(buf) - 7:
        if buf[i] == 0xFF and buf[i+1] == 0xFF and buf[i+3] == 0x04:
            jid = buf[i+2]
            if 1 <= jid <= 6:
                # Position is typically bytes 4-5 as uint16, 0-4095 range → 0-360°
                # Then adjusted for center (2048 = 0°)
                pos_raw = (buf[i+4] << 8) | buf[i+5]
                deg = (pos_raw - 2048) * (360.0 / 4096.0)
                joints[jid] = deg
        i += 1
    
    if joints:
        return joints, "streaming packets"
    return None, "no data"


def main():
    print(f"Opening {PORT} …\n")
    with serial.Serial(PORT, BAUD, bytesize=8, parity='N', stopbits=1, timeout=0.3) as ser:
        configure_port(ser)
        time.sleep(0.3)

        print("=== Current arm positions ===")
        angles, source = read_angles_from_stream(ser, timeout=0.5)
        if angles:
            print(f"(parsed from: {source})")
            for jid in sorted(angles):
                print(f"  J{jid}: {angles[jid]:+.2f}°")
        else:
            print("No parseable data received.")
            return

        print("\n=== Sign convention test ===")
        print("Sending +30° to each joint one at a time.\n")

        joint_labels = [
            "base rotation",
            "shoulder",
            "elbow upper",
            "elbow lower",
            "wrist pitch",
            "wrist roll",
        ]

        results = {}
        for jid in range(1, 7):
            # Home all
            for j in range(1, 7):
                ser.write(set_angle_frame(j, 0.0, 20))
                time.sleep(0.05)
            time.sleep(3.0)

            # Send +30° to this joint
            ser.write(set_angle_frame(jid, 30.0, 20))
            time.sleep(2.0)

            after, src = read_angles_from_stream(ser, timeout=0.5)
            if after and jid in after:
                actual = after[jid]
                match  = abs(actual - 30.0) < 8.0
                inv    = abs(actual + 30.0) < 8.0
                if match:
                    status = "✅  MATCH"
                elif inv:
                    status = "❌  INVERTED"
                    results[jid] = -1
                else:
                    status = f"❓  UNEXPECTED ({actual:+.2f}°)"
                print(f"J{jid} {joint_labels[jid-1]}: sent +30°, got {actual:+.2f}°  → {status}")
            else:
                print(f"J{jid} {joint_labels[jid-1]}: NO RESPONSE (src={src})")

        # Home at end
        for j in range(1, 7):
            ser.write(set_angle_frame(j, 0.0, 20))
            time.sleep(0.05)

    print("\n=== Summary ===")
    if not results:
        print("  All joints match — no sign corrections needed.")
    else:
        signs = [results.get(j, 1) for j in range(1, 7)]
        print(f"  Inverted joints: {[j for j,v in results.items()]}")
        print(f"  jointSignCorrection = {signs}")


if __name__ == "__main__":
    main()
