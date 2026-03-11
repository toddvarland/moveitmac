#!/usr/bin/env python3
"""
Move joint 1 of myCobot 280-M5 to 30°, wait, then return to 0°.
Port:  /dev/cu.usbserial-52D20281481
Baud:  115200
"""
from pymycobot import MyCobot
import time

PORT = "/dev/cu.usbserial-52D20281481"
BAUD = 115200
SPEED = 30  # 1–100

mc = MyCobot(PORT, BAUD)
time.sleep(0.5)

print("Current angles:", mc.get_angles())

print("Moving joint 1 to 30°...")
mc.send_angle(1, 30, SPEED)
time.sleep(3)

print("Angles after move:", mc.get_angles())

print("Returning to 0°...")
mc.send_angle(1, 0, SPEED)
time.sleep(3)

print("Done. Final angles:", mc.get_angles())
