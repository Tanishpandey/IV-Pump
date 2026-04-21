#!/usr/bin/env python3
"""
pi_agent.py — Runs on the Raspberry Pi
Listens for MQTT commands from the laptop GUI and controls hardware.

Install dependencies on Pi:
    pip install paho-mqtt gpiozero smbus2 ticlib pigpio

Run:
    python3 pi_agent.py
"""

import json
import math
import time
import warnings
import threading

import paho.mqtt.client as mqtt

try:
    from gpiozero import Button, AngularServo
    from gpiozero.pins.pigpio import PiGPIOFactory
    from smbus2 import SMBus
    from ticlib import TicI2C, SMBus2Backend
    HW_AVAILABLE = True
except ImportError:
    HW_AVAILABLE = False
    print("[warn] Hardware libraries not found – running in mock mode")

# ============================================================================
# Configuration — edit these to match your hardware
# ============================================================================

BROKER_HOST = "localhost"   # Mosquitto is on the Pi itself
BROKER_PORT  = 1883

PLOTTER_CONFIG = {
    'i2c_bus':               1,
    'x_tic_address':         14,
    'y_tic_address':         15,
    'x_pulley_radius_mm':    12.58,
    'y_pulley_radius_mm':    12.58,
    'x_tics_per_revolution': 200,
    'y_tics_per_revolution': 200,
    'servo1_pin':            18,
    'servo2_pin':            19,
    'servo_gear_radius_mm':  5.0,
    'servo1_offset_x_mm':   -20.0,
    'servo1_offset_y_mm':    0.0,
    'servo2_offset_x_mm':    35.0,
    'servo2_offset_y_mm':    0.0,
    'servo2_reversed':       True,
    'x_limit_min_pin':       17,
    'x_limit_max_pin':       27,
    'y_limit_min_pin':       22,
    'y_limit_max_pin':       23,
    'homing_speed_tics':     2000000,
    'movement_speed_tics':   4000000,
}

# MQTT topics
TOPIC_COMMANDS = "plotter/commands"
TOPIC_RESPONSE = "plotter/response"
TOPIC_STATUS   = "plotter/status"   # heartbeat / agent status

# ============================================================================
# XYPlotter class (full hardware implementation)
# ============================================================================

class XYPlotter:
    def __init__(
        self,
        i2c_bus, x_tic_address, y_tic_address,
        x_pulley_radius_mm, y_pulley_radius_mm,
        x_tics_per_revolution, y_tics_per_revolution,
        servo1_pin, servo2_pin, servo_gear_radius_mm,
        servo_tics_per_revolution=180,
        servo1_offset_x_mm=0.0, servo1_offset_y_mm=0.0,
        servo2_offset_x_mm=0.0, servo2_offset_y_mm=0.0,
        servo2_reversed=False,
        x_limit_min_pin=17, x_limit_max_pin=27,
        y_limit_min_pin=22, y_limit_max_pin=23,
        homing_speed_tics=2000000, movement_speed_tics=4000000,
    ):
        self.x_pulley_radius_mm      = x_pulley_radius_mm
        self.y_pulley_radius_mm      = y_pulley_radius_mm
        self.x_tics_per_revolution   = x_tics_per_revolution
        self.y_tics_per_revolution   = y_tics_per_revolution
        self.servo_gear_radius_mm    = servo_gear_radius_mm
        self.servo_tics_per_revolution = servo_tics_per_revolution
        self.homing_speed_tics       = homing_speed_tics
        self.movement_speed_tics     = movement_speed_tics
        self.servo1_offset_x_mm      = servo1_offset_x_mm
        self.servo1_offset_y_mm      = servo1_offset_y_mm
        self.servo2_offset_x_mm      = servo2_offset_x_mm
        self.servo2_offset_y_mm      = servo2_offset_y_mm
        self.servo2_reversed         = servo2_reversed

        self.x_tics_per_mm = x_tics_per_revolution / (2 * math.pi * x_pulley_radius_mm)
        self.y_tics_per_mm = y_tics_per_revolution / (2 * math.pi * y_pulley_radius_mm)
        servo_circumference = 2 * math.pi * servo_gear_radius_mm
        self.servo_degrees_per_mm = servo_tics_per_revolution / servo_circumference

        self.x_limit_min = Button(x_limit_min_pin, pull_up=True)
        self.x_limit_max = Button(x_limit_max_pin, pull_up=True)
        self.y_limit_min = Button(y_limit_min_pin, pull_up=True)
        self.y_limit_max = Button(y_limit_max_pin, pull_up=True)

        pin_factory = None
        try:
            from gpiozero.pins.lgpio import LGPIOFactory
            pin_factory = LGPIOFactory()
            print("[gpio] Using lgpio backend")
        except Exception as e:
            print(f"[gpio] lgpio failed: {e}, trying pigpio...")
            try:
                pin_factory = PiGPIOFactory()
            except Exception as e2:
                print(f"[gpio] pigpio also failed: {e2}, using default")

        self.servo1 = AngularServo(servo1_pin, min_angle=0,
                                   max_angle=servo_tics_per_revolution,
                                   initial_angle=None,
                                   min_pulse_width=0.5/1000,
                                   max_pulse_width=2.5/1000,
                                   pin_factory=pin_factory)
        self.servo2 = AngularServo(servo2_pin, min_angle=0,
                                   max_angle=servo_tics_per_revolution,
                                   initial_angle=None,
                                   min_pulse_width=0.5/1000,
                                   max_pulse_width=2.5/1000,
                                   pin_factory=pin_factory)
        self.servo1_position = 90.0
        self.servo2_position = 90.0

        self.bus = SMBus(i2c_bus)
        x_backend = SMBus2Backend(self.bus, x_tic_address)
        y_backend = SMBus2Backend(self.bus, y_tic_address)
        self.x_tic = TicI2C(x_backend)
        self.y_tic = TicI2C(y_backend)
        self._initialize_motor(self.x_tic)
        self._initialize_motor(self.y_tic)

        self.is_calibrated  = False
        self.x_max_mm       = None
        self.y_max_mm       = None
        self.x_max_tics     = None
        self.y_max_tics     = None
        self.current_x_mm   = 0.0
        self.current_y_mm   = 0.0

    def _initialize_motor(self, tic):
        tic.halt_and_set_position(0)
        tic.energize()
        tic.exit_safe_start()

    def _is_limit_switch_pressed(self, limit_switch):
        return limit_switch.is_pressed

    def home_and_calibrate(self, x_backoff_mm=5.0, y_backoff_mm=5.0):
        self.x_tic.set_max_speed(self.homing_speed_tics)
        self.y_tic.set_max_speed(self.homing_speed_tics)
        self._home_axis_to_minimum(self.x_tic, self.x_limit_min, "X")
        self._home_axis_to_minimum(self.y_tic, self.y_limit_min, "Y")
        self.x_tic.halt_and_set_position(0)
        self.y_tic.halt_and_set_position(0)
        self.current_x_mm = 0.0
        self.current_y_mm = 0.0
        time.sleep(0.5)
        self.x_max_tics = self._find_axis_maximum(self.x_tic, self.x_limit_max, "X")
        self.y_max_tics = self._find_axis_maximum(self.y_tic, self.y_limit_max, "Y")
        self.x_max_mm = self.x_max_tics / self.x_tics_per_mm
        self.y_max_mm = self.y_max_tics / self.y_tics_per_mm
        x_backoff_tics = int(-x_backoff_mm * self.x_tics_per_mm)
        y_backoff_tics = int(-y_backoff_mm * self.y_tics_per_mm)
        self._move_axis_relative(self.x_tic, x_backoff_tics)
        self._move_axis_relative(self.y_tic, y_backoff_tics)
        self.x_max_mm  -= x_backoff_mm
        self.y_max_mm  -= y_backoff_mm
        self.x_max_tics = int(self.x_max_mm * self.x_tics_per_mm)
        self.y_max_tics = int(self.y_max_mm * self.y_tics_per_mm)
        self.x_tic.set_target_position(0)
        self.y_tic.set_target_position(0)
        self._wait_for_movement(self.x_tic)
        self._wait_for_movement(self.y_tic)
        self.current_x_mm = 0.0
        self.current_y_mm = 0.0
        self.x_tic.set_max_speed(self.movement_speed_tics)
        self.y_tic.set_max_speed(self.movement_speed_tics)
        self.is_calibrated = True
        return self.x_max_mm, self.y_max_mm

    def _home_axis_to_minimum(self, tic, limit_switch, axis_name):
        tic.set_target_velocity(-self.homing_speed_tics)
        while not self._is_limit_switch_pressed(limit_switch):
            time.sleep(0.01)
        tic.halt_and_hold()
        time.sleep(0.1)

    def _find_axis_maximum(self, tic, limit_switch, axis_name):
        tic.set_target_velocity(self.homing_speed_tics)
        while not self._is_limit_switch_pressed(limit_switch):
            time.sleep(0.01)
        tic.halt_and_hold()
        time.sleep(0.1)
        return tic.get_current_position()

    def _move_axis_relative(self, tic, tics):
        target_pos = tic.get_current_position() + tics
        tic.set_target_position(target_pos)
        self._wait_for_movement(tic)

    def _wait_for_movement(self, tic):
        while tic.get_current_velocity() != 0:
            time.sleep(0.01)
        time.sleep(0.1)

    def move_to(self, x_mm, y_mm, check_bounds=True):
        if not self.is_calibrated:
            raise RuntimeError("Plotter must be calibrated first.")
        if check_bounds and not self.is_within_bounds(x_mm, y_mm):
            raise ValueError(f"Position ({x_mm}, {y_mm}) out of bounds.")
        self.x_tic.set_target_position(int(x_mm * self.x_tics_per_mm))
        self.y_tic.set_target_position(int(y_mm * self.y_tics_per_mm))
        self._wait_for_movement(self.x_tic)
        self._wait_for_movement(self.y_tic)
        self.current_x_mm = x_mm
        self.current_y_mm = y_mm

    def move_relative(self, dx_mm, dy_mm, check_bounds=True):
        self.move_to(self.current_x_mm + dx_mm, self.current_y_mm + dy_mm, check_bounds)

    def is_within_bounds(self, x_mm, y_mm):
        if not self.is_calibrated:
            return False
        return (0 <= x_mm <= self.x_max_mm) and (0 <= y_mm <= self.y_max_mm)

    def get_current_position(self):
        return (self.current_x_mm, self.current_y_mm)

    def get_working_area(self):
        if not self.is_calibrated:
            raise RuntimeError("Plotter not calibrated")
        return (self.x_max_mm, self.y_max_mm)

    def move_servo(self, servo_num, distance_mm):
        angle_change = distance_mm * self.servo_degrees_per_mm
        current_angle = self.servo1_position if servo_num == 1 else self.servo2_position
        new_angle = max(0, min(self.servo_tics_per_revolution, current_angle + angle_change))
        self.set_servo_angle(servo_num, new_angle)

    def set_servo_angle(self, servo_num, angle):
        if servo_num not in [1, 2]:
            raise ValueError("servo_num must be 1 or 2")
        if not 0 <= angle <= self.servo_tics_per_revolution:
            raise ValueError(f"Angle must be 0-{self.servo_tics_per_revolution}")
        if servo_num == 1:
            self.servo1.angle = angle
            self.servo1_position = angle
        else:
            actual_angle = (self.servo_tics_per_revolution - angle) if self.servo2_reversed else angle
            self.servo2.angle = actual_angle
            self.servo2_position = angle
        time.sleep(0.3)

    def press_button(self, servo_num, press_distance_mm=5.0, press_duration=0.5):
        self.move_servo(servo_num, press_distance_mm)
        time.sleep(press_duration)
        self.move_servo(servo_num, -press_distance_mm)

    def get_servo_position(self, servo_num):
        if servo_num == 1:
            return (self.current_x_mm + self.servo1_offset_x_mm,
                    self.current_y_mm + self.servo1_offset_y_mm)
        elif servo_num == 2:
            return (self.current_x_mm + self.servo2_offset_x_mm,
                    self.current_y_mm + self.servo2_offset_y_mm)
        raise ValueError("servo_num must be 1 or 2")

    def set_servo_offsets(self, servo_num, offset_x_mm, offset_y_mm):
        if servo_num == 1:
            self.servo1_offset_x_mm = offset_x_mm
            self.servo1_offset_y_mm = offset_y_mm
        elif servo_num == 2:
            self.servo2_offset_x_mm = offset_x_mm
            self.servo2_offset_y_mm = offset_y_mm
        else:
            raise ValueError("servo_num must be 1 or 2")

    def _select_best_servo(self, target_x_mm, target_y_mm, check_bounds=True):
        px1 = target_x_mm - self.servo1_offset_x_mm
        py1 = target_y_mm - self.servo1_offset_y_mm
        px2 = target_x_mm - self.servo2_offset_x_mm
        py2 = target_y_mm - self.servo2_offset_y_mm
        s1_ok = not check_bounds or not self.is_calibrated or self.is_within_bounds(px1, py1)
        s2_ok = not check_bounds or not self.is_calibrated or self.is_within_bounds(px2, py2)
        if s1_ok and not s2_ok:
            return 1
        if s2_ok and not s1_ok:
            return 2
        if not s1_ok and not s2_ok:
            return None
        d1 = math.hypot(px1 - self.current_x_mm, py1 - self.current_y_mm)
        d2 = math.hypot(px2 - self.current_x_mm, py2 - self.current_y_mm)
        return 1 if d1 <= d2 else 2

    def move_and_press(self, target_x_mm, target_y_mm, press_distance_mm=5.0,
                       press_duration=0.5, servo_num=None, check_bounds=True):
        if not self.is_calibrated:
            raise RuntimeError("Plotter must be calibrated first.")
        if servo_num is None:
            servo_num = self._select_best_servo(target_x_mm, target_y_mm, check_bounds)
            if servo_num is None:
                raise ValueError(f"Target ({target_x_mm:.2f}, {target_y_mm:.2f}) unreachable.")
        elif servo_num not in [1, 2]:
            raise ValueError("servo_num must be 1, 2, or None")
        if servo_num == 1:
            plotter_x = target_x_mm - self.servo1_offset_x_mm
            plotter_y = target_y_mm - self.servo1_offset_y_mm
        else:
            plotter_x = target_x_mm - self.servo2_offset_x_mm
            plotter_y = target_y_mm - self.servo2_offset_y_mm
        self.move_to(plotter_x, plotter_y, check_bounds)
        self.press_button(servo_num, press_distance_mm, press_duration)
        return servo_num

    def emergency_stop(self):
        print("[plotter] EMERGENCY STOP!")
        self.x_tic.halt_and_hold()
        self.y_tic.halt_and_hold()
        self.x_tic.deenergize()
        self.y_tic.deenergize()

    def cleanup(self):
        self.x_tic.deenergize()
        self.y_tic.deenergize()
        self.servo1.close()
        self.servo2.close()
        self.x_limit_min.close()
        self.x_limit_max.close()
        self.y_limit_min.close()
        self.y_limit_max.close()
        self.bus.close()

    def __enter__(self):  return self
    def __exit__(self, *_): self.cleanup()


# ============================================================================
# Mock plotter (used when hardware is not available — for testing)
# ============================================================================

class MockXYPlotter:
    """Fake plotter that just prints commands — useful for testing the MQTT link."""

    def __init__(self, **_kwargs):
        self.is_calibrated    = False
        self.current_x_mm     = 0.0
        self.current_y_mm     = 0.0
        self.x_max_mm         = 300.0
        self.y_max_mm         = 200.0
        self.servo1_position  = 90.0
        self.servo2_position  = 90.0
        self.servo1_offset_x_mm = -20.0
        self.servo1_offset_y_mm = 0.0
        self.servo2_offset_x_mm = 35.0
        self.servo2_offset_y_mm = 0.0

    def home_and_calibrate(self, **_):
        print("[mock] Homing and calibrating…")
        time.sleep(1.5)
        self.is_calibrated = True
        print(f"[mock] Calibrated. Working area: {self.x_max_mm}×{self.y_max_mm}mm")
        return self.x_max_mm, self.y_max_mm

    def move_to(self, x_mm, y_mm, check_bounds=True):
        print(f"[mock] Moving to ({x_mm:.2f}, {y_mm:.2f})")
        time.sleep(0.3)
        self.current_x_mm = x_mm
        self.current_y_mm = y_mm

    def move_relative(self, dx, dy, check_bounds=True):
        self.move_to(self.current_x_mm + dx, self.current_y_mm + dy, check_bounds)

    def is_within_bounds(self, x, y):
        return (0 <= x <= self.x_max_mm) and (0 <= y <= self.y_max_mm)

    def get_current_position(self):
        return (self.current_x_mm, self.current_y_mm)

    def get_servo_position(self, servo_num):
        if servo_num == 1:
            return (self.current_x_mm + self.servo1_offset_x_mm,
                    self.current_y_mm + self.servo1_offset_y_mm)
        return (self.current_x_mm + self.servo2_offset_x_mm,
                self.current_y_mm + self.servo2_offset_y_mm)

    def set_servo_angle(self, servo_num, angle):
        print(f"[mock] Servo {servo_num} → {angle:.1f}°")
        if servo_num == 1: self.servo1_position = angle
        else:              self.servo2_position  = angle

    def move_and_press(self, target_x_mm, target_y_mm,
                       press_distance_mm=5.0, press_duration=0.5,
                       servo_num=None, check_bounds=True):
        servo_num = servo_num or 1
        print(f"[mock] move_and_press → ({target_x_mm:.2f}, {target_y_mm:.2f}) "
              f"servo={servo_num} dist={press_distance_mm}mm")
        time.sleep(0.5)
        return servo_num

    def emergency_stop(self):
        print("[mock] EMERGENCY STOP")

    def cleanup(self):
        print("[mock] Cleanup")


# ============================================================================
# MQTT Agent
# ============================================================================

class PlotterAgent:
    def __init__(self):
        self.plotter      = None
        self.lock         = threading.Lock()   # one command at a time
        self._busy        = False

        self.client = mqtt.Client(client_id="pi_plotter_agent")
        self.client.on_connect    = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message    = self._on_message

    # ── MQTT callbacks ────────────────────────────────────────────────────────

    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print(f"[agent] Connected to MQTT broker at {BROKER_HOST}:{BROKER_PORT}")
            client.subscribe(TOPIC_COMMANDS)
            print(f"[agent] Subscribed to '{TOPIC_COMMANDS}'")
            self._publish_status("online")
        else:
            print(f"[agent] Connection failed, rc={rc}")

    def _on_disconnect(self, client, userdata, rc):
        print(f"[agent] Disconnected from broker (rc={rc}), will auto-reconnect…")

    def _on_message(self, client, userdata, msg):
        """Dispatch incoming commands — each runs in its own thread so MQTT loop stays free."""
        try:
            cmd = json.loads(msg.payload.decode())
        except json.JSONDecodeError as e:
            print(f"[agent] Bad JSON: {e}")
            return

        action = cmd.get("action", "")
        print(f"[agent] ← command: {action}  payload={cmd}")

        # Run the handler in a background thread so the MQTT loop isn't blocked
        threading.Thread(target=self._dispatch, args=(cmd,), daemon=True).start()

    # ── Command dispatcher ────────────────────────────────────────────────────

    def _dispatch(self, cmd):
        action = cmd.get("action", "")

        # Emergency stop bypasses the busy-lock so it always works
        if action == "emergency_stop":
            self._handle_emergency_stop()
            return

        # All other commands are serialized
        with self.lock:
            self._busy = True
            self._publish_status("busy")
            try:
                if   action == "home":           self._handle_home(cmd)
                elif action == "move_and_press": self._handle_move_and_press(cmd)
                elif action == "move_to":        self._handle_move_to(cmd)
                elif action == "jog":            self._handle_jog(cmd)
                elif action == "set_servo":      self._handle_set_servo(cmd)
                elif action == "get_position":   self._handle_get_position()
                elif action == "ping":           self._reply({"status": "ok", "action": "ping", "message": "pong"})
                else:
                    self._reply({"status": "error", "action": action,
                                 "message": f"Unknown action: {action}"})
            except Exception as e:
                print(f"[agent] Error handling '{action}': {e}")
                self._reply({"status": "error", "action": action, "message": str(e)})
            finally:
                self._busy = False
                self._publish_status("online")

    # ── Handlers ──────────────────────────────────────────────────────────────

    def _handle_home(self, cmd):
        print("[agent] Initialising plotter and homing…")
        PlotterClass = XYPlotter if HW_AVAILABLE else MockXYPlotter
        if self.plotter is not None:
            try:
                self.plotter.cleanup()
            except Exception:
                pass
        self.plotter = PlotterClass(**PLOTTER_CONFIG)
        x_max, y_max = self.plotter.home_and_calibrate()
        print(f"[agent] Homed. Working area: {x_max:.1f} × {y_max:.1f} mm")
        self._reply({
            "status": "ok",
            "action": "home",
            "x_max":  round(x_max, 2),
            "y_max":  round(y_max, 2),
        })

    def _handle_move_and_press(self, cmd):
        self._require_calibrated()
        x          = float(cmd["x"])
        y          = float(cmd["y"])
        press_dist = float(cmd.get("press_dist", 5.0))
        servo_num  = cmd.get("servo_num", None)   # None = auto-select
        servo_used = self.plotter.move_and_press(
            x, y,
            press_distance_mm=press_dist,
            servo_num=servo_num,
        )
        cx, cy = self.plotter.get_current_position()
        self._reply({
            "status":     "ok",
            "action":     "move_and_press",
            "servo_used": servo_used,
            "x":          round(x,  2),
            "y":          round(y,  2),
            "current_x":  round(cx, 2),
            "current_y":  round(cy, 2),
        })

    def _handle_move_to(self, cmd):
        self._require_calibrated()
        x = float(cmd["x"])
        y = float(cmd["y"])
        self.plotter.move_to(x, y)
        cx, cy = self.plotter.get_current_position()
        self._reply({
            "status":    "ok",
            "action":    "move_to",
            "current_x": round(cx, 2),
            "current_y": round(cy, 2),
        })

    def _handle_jog(self, cmd):
        self._require_calibrated()
        dx = float(cmd.get("dx", 0))
        dy = float(cmd.get("dy", 0))
        self.plotter.move_relative(dx, dy)
        cx, cy = self.plotter.get_current_position()
        e1x, e1y = self.plotter.get_servo_position(1)
        e2x, e2y = self.plotter.get_servo_position(2)
        self._reply({
            "status":    "ok",
            "action":    "jog",
            "current_x": round(cx,  2),
            "current_y": round(cy,  2),
            "ef1_x":     round(e1x, 2),
            "ef1_y":     round(e1y, 2),
            "ef2_x":     round(e2x, 2),
            "ef2_y":     round(e2y, 2),
        })

    def _handle_set_servo(self, cmd):
        self._require_calibrated()
        servo_num = int(cmd["servo_num"])
        angle     = float(cmd["angle"])
        self.plotter.set_servo_angle(servo_num, angle)
        self._reply({
            "status":    "ok",
            "action":    "set_servo",
            "servo_num": servo_num,
            "angle":     angle,
        })

    def _handle_get_position(self):
        self._require_calibrated()
        cx, cy   = self.plotter.get_current_position()
        e1x, e1y = self.plotter.get_servo_position(1)
        e2x, e2y = self.plotter.get_servo_position(2)
        self._reply({
            "status":    "ok",
            "action":    "get_position",
            "current_x": round(cx,  2),
            "current_y": round(cy,  2),
            "ef1_x":     round(e1x, 2),
            "ef1_y":     round(e1y, 2),
            "ef2_x":     round(e2x, 2),
            "ef2_y":     round(e2y, 2),
        })

    def _handle_emergency_stop(self):
        print("[agent] *** EMERGENCY STOP ***")
        if self.plotter:
            try:
                self.plotter.emergency_stop()
            except Exception as e:
                print(f"[agent] E-stop error: {e}")
        self._reply({"status": "ok", "action": "emergency_stop"})
        self._publish_status("stopped")

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _require_calibrated(self):
        if self.plotter is None or not self.plotter.is_calibrated:
            raise RuntimeError("Plotter not initialised. Send 'home' command first.")

    def _reply(self, payload: dict):
        msg = json.dumps(payload)
        self.client.publish(TOPIC_RESPONSE, msg)
        print(f"[agent] → response: {msg}")

    def _publish_status(self, status: str):
        self.client.publish(TOPIC_STATUS, json.dumps({"agent_status": status}))

    # ── Run ───────────────────────────────────────────────────────────────────

    def run(self):
        print(f"[agent] Connecting to broker at {BROKER_HOST}:{BROKER_PORT} …")
        self.client.connect(BROKER_HOST, BROKER_PORT, keepalive=60)
        # reconnect_delay_set ensures auto-reconnect on network hiccups
        self.client.reconnect_delay_set(min_delay=1, max_delay=30)
        try:
            self.client.loop_forever()
        except KeyboardInterrupt:
            print("\n[agent] Shutting down…")
        finally:
            self._publish_status("offline")
            if self.plotter:
                self.plotter.cleanup()
            self.client.disconnect()


# ============================================================================
# Entry point
# ============================================================================

if __name__ == "__main__":
    agent = PlotterAgent()
    agent.run()
