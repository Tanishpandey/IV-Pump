#!/usr/bin/env python3
from gpiozero import Button
from signal import pause
import time

# Limit switch pin mapping
X_MIN_PIN = 17
X_MAX_PIN = 27
Y_MIN_PIN = 22
Y_MAX_PIN = 23

# Create Button objects (active_low=True assumes switches pull to GND when pressed)
x_min = Button(X_MIN_PIN, pull_up=True)
x_max = Button(X_MAX_PIN, pull_up=True)
y_min = Button(Y_MIN_PIN, pull_up=True)
y_max = Button(Y_MAX_PIN, pull_up=True)

def print_states():
    print(
        f"Xmin: {x_min.is_pressed}   "
        f"Xmax: {x_max.is_pressed}   "
        f"Ymin: {y_min.is_pressed}   "
        f"Ymax: {y_max.is_pressed}"
    )

print("📟 Limit Switch Tester Running")
print("Press Ctrl+C to exit.\n")

try:
    while True:
        print_states()
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nExiting limit switch tester.")
