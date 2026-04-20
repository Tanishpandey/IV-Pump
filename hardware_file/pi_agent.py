# pico_agent.py — Runs on Pico 2W (MicroPython)
#
# Flash MicroPython firmware first:
#   https://micropython.org/download/RPI_PICO2_W/
#
# Upload this file as main.py on the Pico using Thonny or mpremote:
#   mpremote cp pico_agent.py :main.py
#
# Dependencies (already built into MicroPython for Pico W):
#   - umqtt.simple  (built-in)
#   - machine       (built-in)
#   - network       (built-in)
#
# Mosquitto broker must be running on your LAPTOP (not the Pico).
# On laptop:  sudo apt install mosquitto  OR  brew install mosquitto
#             mosquitto -v
#
# ============================================================================

import json
import time
import math
import network
import machine
from machine import Pin, PWM, I2C
from umqtt.simple import MQTTClient

# ============================================================================
# CONFIG — edit these
# ============================================================================

WIFI_SSID     = "YourWiFiName"        # ← your WiFi SSID
WIFI_PASSWORD = "YourWiFiPassword"    # ← your WiFi password

BROKER_HOST   = "192.168.1.50"        # ← your LAPTOP's IP address (run `hostname -I` on laptop)
BROKER_PORT   = 1883
CLIENT_ID     = "pico2w_plotter"

# MQTT topics
TOPIC_COMMANDS = b"plotter/commands"
TOPIC_RESPONSE = b"plotter/response"
TOPIC_STATUS   = b"plotter/status"

# ── Hardware pin config ──────────────────────────────────────────────────────
# I2C for Tic motor controllers
I2C_BUS       = 0         # I2C bus 0
I2C_SDA_PIN   = 4         # GP4
I2C_SCL_PIN   = 5         # GP5
X_TIC_ADDR    = 14        # I2C address of X-axis Tic
Y_TIC_ADDR    = 15        # I2C address of Y-axis Tic

# Servo PWM pins
SERVO1_PIN    = 18        # GP18
SERVO2_PIN    = 19        # GP19

# Limit switch pins (pulled up internally)
X_LIMIT_MIN_PIN = 17
X_LIMIT_MAX_PIN = 27
Y_LIMIT_MIN_PIN = 22
Y_LIMIT_MAX_PIN = 23

# Plotter mechanical config
X_PULLEY_RADIUS_MM    = 12.58
Y_PULLEY_RADIUS_MM    = 12.58
X_TICS_PER_REV        = 200
Y_TICS_PER_REV        = 200
SERVO_GEAR_RADIUS_MM  = 5.0
SERVO1_OFFSET_X_MM    = -20.0
SERVO1_OFFSET_Y_MM    = 0.0
SERVO2_OFFSET_X_MM    = 35.0
SERVO2_OFFSET_Y_MM    = 0.0
SERVO2_REVERSED       = True
HOMING_SPEED          = 2000000
MOVEMENT_SPEED        = 4000000

# ============================================================================
# Tic I2C driver (minimal — only what we need)
# MicroPython doesn't have ticlib, so we implement the commands manually.
# Tic I2C command reference:
#   https://www.pololu.com/docs/0J71/8
# ============================================================================

class TicI2C:
    # Command codes
    CMD_SET_TARGET_POS      = 0xE0
    CMD_SET_TARGET_VEL      = 0xE5
    CMD_HALT_AND_SET_POS    = 0xEC
    CMD_HALT_AND_HOLD       = 0x89
    CMD_GO_HOME             = 0x97
    CMD_DEENERGIZE          = 0x86
    CMD_ENERGIZE            = 0x85
    CMD_EXIT_SAFE_START     = 0x83
    CMD_SET_SPEED_MAX       = 0xE6

    def __init__(self, i2c: I2C, address: int):
        self.i2c  = i2c
        self.addr = address

    def _write32(self, cmd, value):
        """Send a 32-bit signed command."""
        b = value & 0xFFFFFFFF
        self.i2c.writeto(self.addr, bytes([
            cmd,
            b & 0x7F,
            (b >> 7)  & 0x7F,
            (b >> 14) & 0x7F,
            (b >> 21) & 0x7F,
            (b >> 28) & 0x0F,
        ]))

    def _write7(self, cmd):
        self.i2c.writeto(self.addr, bytes([cmd]))

    def _get_variable(self, offset, length):
        self.i2c.writeto(self.addr, bytes([0xA1, offset]))
        return self.i2c.readfrom(self.addr, length)

    def halt_and_set_position(self, pos):
        self._write32(self.CMD_HALT_AND_SET_POS, pos)

    def halt_and_hold(self):
        self._write7(self.CMD_HALT_AND_HOLD)

    def energize(self):
        self._write7(self.CMD_ENERGIZE)

    def exit_safe_start(self):
        self._write7(self.CMD_EXIT_SAFE_START)

    def deenergize(self):
        self._write7(self.CMD_DEENERGIZE)

    def set_target_position(self, pos):
        self._write32(self.CMD_SET_TARGET_POS, pos)

    def set_target_velocity(self, vel):
        self._write32(self.CMD_SET_TARGET_VEL, vel)

    def set_max_speed(self, speed):
        self._write32(self.CMD_SET_SPEED_MAX, speed)

    def get_current_position(self):
        data = self._get_variable(0x22, 4)
        val = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24)
        # convert to signed 32-bit
        if val >= 0x80000000:
            val -= 0x100000000
        return val

    def get_current_velocity(self):
        data = self._get_variable(0x26, 4)
        val = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24)
        if val >= 0x80000000:
            val -= 0x100000000
        return val


# ============================================================================
# XYPlotter (MicroPython version)
# ============================================================================

class XYPlotter:
    def __init__(self):
        # Compute conversion factors
        self.x_tics_per_mm = X_TICS_PER_REV / (2 * math.pi * X_PULLEY_RADIUS_MM)
        self.y_tics_per_mm = Y_TICS_PER_REV / (2 * math.pi * Y_PULLEY_RADIUS_MM)
        servo_circ = 2 * math.pi * SERVO_GEAR_RADIUS_MM
        self.servo_degrees_per_mm = 180 / servo_circ

        # I2C
        self.i2c = I2C(I2C_BUS, sda=Pin(I2C_SDA_PIN), scl=Pin(I2C_SCL_PIN), freq=400_000)
        self.x_tic = TicI2C(self.i2c, X_TIC_ADDR)
        self.y_tic = TicI2C(self.i2c, Y_TIC_ADDR)

        # Limit switches (pull-up, active LOW when pressed)
        self.x_lim_min = Pin(X_LIMIT_MIN_PIN, Pin.IN, Pin.PULL_UP)
        self.x_lim_max = Pin(X_LIMIT_MAX_PIN, Pin.IN, Pin.PULL_UP)
        self.y_lim_min = Pin(Y_LIMIT_MIN_PIN, Pin.IN, Pin.PULL_UP)
        self.y_lim_max = Pin(Y_LIMIT_MAX_PIN, Pin.IN, Pin.PULL_UP)

        # Servos — 50Hz PWM, duty 1000–2000µs = 0°–180°
        self._s1_pwm = PWM(Pin(SERVO1_PIN), freq=50)
        self._s2_pwm = PWM(Pin(SERVO2_PIN), freq=50)
        self.servo1_angle = 90.0
        self.servo2_angle = 90.0
        self._set_servo_pwm(self._s1_pwm, 90)
        self._set_servo_pwm(self._s2_pwm, 90)

        # Init motors
        for tic in (self.x_tic, self.y_tic):
            tic.halt_and_set_position(0)
            tic.energize()
            tic.exit_safe_start()

        self.is_calibrated = False
        self.x_max_mm = 0.0
        self.y_max_mm = 0.0
        self.current_x_mm = 0.0
        self.current_y_mm = 0.0

    # ── Servo helpers ──────────────────────────────────────────────────────────

    def _set_servo_pwm(self, pwm_obj, angle_deg):
        """Convert 0–180° to PWM duty cycle (500–2500µs at 50Hz = 65535 max duty)."""
        angle_deg = max(0, min(180, angle_deg))
        # 500µs → 0°,  2500µs → 180°
        pulse_us  = 500 + (angle_deg / 180.0) * 2000
        duty      = int(pulse_us / 20000 * 65535)   # 20000µs = 20ms period
        pwm_obj.duty_u16(duty)

    def set_servo_angle(self, servo_num, angle):
        angle = max(0, min(180, angle))
        if servo_num == 1:
            self._set_servo_pwm(self._s1_pwm, angle)
            self.servo1_angle = angle
        else:
            actual = (180 - angle) if SERVO2_REVERSED else angle
            self._set_servo_pwm(self._s2_pwm, actual)
            self.servo2_angle = angle
        time.sleep_ms(300)

    def move_servo(self, servo_num, distance_mm):
        delta = distance_mm * self.servo_degrees_per_mm
        cur   = self.servo1_angle if servo_num == 1 else self.servo2_angle
        self.set_servo_angle(servo_num, cur + delta)

    def press_button(self, servo_num, press_distance_mm=5.0, press_duration_ms=500):
        self.move_servo(servo_num, press_distance_mm)
        time.sleep_ms(press_duration_ms)
        self.move_servo(servo_num, -press_distance_mm)

    # ── Limit switch helpers ───────────────────────────────────────────────────

    def _lim_pressed(self, pin):
        return pin.value() == 0   # active LOW

    # ── Motor movement ─────────────────────────────────────────────────────────

    def _wait_stopped(self, tic):
        while tic.get_current_velocity() != 0:
            time.sleep_ms(10)
        time.sleep_ms(100)

    def _home_to_min(self, tic, lim_pin):
        tic.set_target_velocity(-HOMING_SPEED)
        while not self._lim_pressed(lim_pin):
            time.sleep_ms(10)
        tic.halt_and_hold()
        time.sleep_ms(100)

    def _find_max(self, tic, lim_pin):
        tic.set_target_velocity(HOMING_SPEED)
        while not self._lim_pressed(lim_pin):
            time.sleep_ms(10)
        tic.halt_and_hold()
        time.sleep_ms(100)
        return tic.get_current_position()

    def home_and_calibrate(self, x_backoff_mm=5.0, y_backoff_mm=5.0):
        print("[plotter] Homing…")
        self.x_tic.set_max_speed(HOMING_SPEED)
        self.y_tic.set_max_speed(HOMING_SPEED)

        self._home_to_min(self.x_tic, self.x_lim_min)
        self._home_to_min(self.y_tic, self.y_lim_min)
        self.x_tic.halt_and_set_position(0)
        self.y_tic.halt_and_set_position(0)
        time.sleep_ms(500)

        x_max_tics = self._find_max(self.x_tic, self.x_lim_max)
        y_max_tics = self._find_max(self.y_tic, self.y_lim_max)

        self.x_max_mm = x_max_tics / self.x_tics_per_mm
        self.y_max_mm = y_max_tics / self.y_tics_per_mm

        # Back off from hard limits
        bx = int(-x_backoff_mm * self.x_tics_per_mm)
        by = int(-y_backoff_mm * self.y_tics_per_mm)
        self.x_tic.set_target_position(self.x_tic.get_current_position() + bx)
        self.y_tic.set_target_position(self.y_tic.get_current_position() + by)
        self._wait_stopped(self.x_tic)
        self._wait_stopped(self.y_tic)

        self.x_max_mm -= x_backoff_mm
        self.y_max_mm -= y_backoff_mm

        self.x_tic.set_target_position(0)
        self.y_tic.set_target_position(0)
        self._wait_stopped(self.x_tic)
        self._wait_stopped(self.y_tic)

        self.x_tic.set_max_speed(MOVEMENT_SPEED)
        self.y_tic.set_max_speed(MOVEMENT_SPEED)
        self.current_x_mm = 0.0
        self.current_y_mm = 0.0
        self.is_calibrated = True
        print(f"[plotter] Calibrated: {self.x_max_mm:.1f} x {self.y_max_mm:.1f} mm")
        return self.x_max_mm, self.y_max_mm

    def is_within_bounds(self, x, y):
        return 0 <= x <= self.x_max_mm and 0 <= y <= self.y_max_mm

    def move_to(self, x_mm, y_mm):
        if not self.is_calibrated:
            raise Exception("Not calibrated")
        if not self.is_within_bounds(x_mm, y_mm):
            raise Exception(f"Out of bounds: ({x_mm:.2f}, {y_mm:.2f})")
        self.x_tic.set_target_position(int(x_mm * self.x_tics_per_mm))
        self.y_tic.set_target_position(int(y_mm * self.y_tics_per_mm))
        self._wait_stopped(self.x_tic)
        self._wait_stopped(self.y_tic)
        self.current_x_mm = x_mm
        self.current_y_mm = y_mm

    def move_relative(self, dx, dy):
        self.move_to(self.current_x_mm + dx, self.current_y_mm + dy)

    def get_current_position(self):
        return self.current_x_mm, self.current_y_mm

    def get_servo_position(self, servo_num):
        if servo_num == 1:
            return (self.current_x_mm + SERVO1_OFFSET_X_MM,
                    self.current_y_mm + SERVO1_OFFSET_Y_MM)
        return (self.current_x_mm + SERVO2_OFFSET_X_MM,
                self.current_y_mm + SERVO2_OFFSET_Y_MM)

    def _select_best_servo(self, tx, ty):
        px1 = tx - SERVO1_OFFSET_X_MM
        py1 = ty - SERVO1_OFFSET_Y_MM
        px2 = tx - SERVO2_OFFSET_X_MM
        py2 = ty - SERVO2_OFFSET_Y_MM
        s1_ok = self.is_within_bounds(px1, py1)
        s2_ok = self.is_within_bounds(px2, py2)
        if s1_ok and not s2_ok: return 1
        if s2_ok and not s1_ok: return 2
        if not s1_ok and not s2_ok: return None
        d1 = math.sqrt((px1 - self.current_x_mm)**2 + (py1 - self.current_y_mm)**2)
        d2 = math.sqrt((px2 - self.current_x_mm)**2 + (py2 - self.current_y_mm)**2)
        return 1 if d1 <= d2 else 2

    def move_and_press(self, tx, ty, press_dist=5.0, servo_num=None):
        if not self.is_calibrated:
            raise Exception("Not calibrated")
        if servo_num is None:
            servo_num = self._select_best_servo(tx, ty)
        if servo_num is None:
            raise Exception(f"Target ({tx:.2f}, {ty:.2f}) unreachable")
        if servo_num == 1:
            px, py = tx - SERVO1_OFFSET_X_MM, ty - SERVO1_OFFSET_Y_MM
        else:
            px, py = tx - SERVO2_OFFSET_X_MM, ty - SERVO2_OFFSET_Y_MM
        self.move_to(px, py)
        self.press_button(servo_num, press_dist)
        return servo_num

    def emergency_stop(self):
        self.x_tic.halt_and_hold()
        self.y_tic.halt_and_hold()
        self.x_tic.deenergize()
        self.y_tic.deenergize()

    def cleanup(self):
        self.emergency_stop()
        self._s1_pwm.deinit()
        self._s2_pwm.deinit()


# ============================================================================
# WiFi connection
# ============================================================================

def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if wlan.isconnected():
        print(f"[wifi] Already connected: {wlan.ifconfig()[0]}")
        return wlan.ifconfig()[0]

    print(f"[wifi] Connecting to {WIFI_SSID}…")
    wlan.connect(WIFI_SSID, WIFI_PASSWORD)

    for _ in range(30):      # wait up to 15 seconds
        if wlan.isconnected():
            ip = wlan.ifconfig()[0]
            print(f"[wifi] Connected! IP: {ip}")
            return ip
        time.sleep_ms(500)

    raise Exception(f"[wifi] Failed to connect to {WIFI_SSID}")


# ============================================================================
# MQTT Agent
# ============================================================================

plotter = None

def handle_command(topic, msg):
    global plotter

    try:
        cmd = json.loads(msg)
    except Exception as e:
        print(f"[mqtt] Bad JSON: {e}")
        return

    action = cmd.get("action", "")
    print(f"[mqtt] ← {action}")

    try:
        if action == "ping":
            publish(TOPIC_RESPONSE, {"status": "ok", "action": "ping", "message": "pong"})

        elif action == "home":
            if plotter is None:
                plotter = XYPlotter()
            x_max, y_max = plotter.home_and_calibrate()
            publish(TOPIC_RESPONSE, {
                "status": "ok", "action": "home",
                "x_max": round(x_max, 2), "y_max": round(y_max, 2)
            })

        elif action == "move_and_press":
            _require_calibrated()
            x    = float(cmd["x"])
            y    = float(cmd["y"])
            dist = float(cmd.get("press_dist", 5.0))
            snum = cmd.get("servo_num", None)
            servo_used = plotter.move_and_press(x, y, dist, snum)
            cx, cy = plotter.get_current_position()
            publish(TOPIC_RESPONSE, {
                "status": "ok", "action": "move_and_press",
                "servo_used": servo_used,
                "x": round(x, 2), "y": round(y, 2),
                "current_x": round(cx, 2), "current_y": round(cy, 2)
            })

        elif action == "move_to":
            _require_calibrated()
            plotter.move_to(float(cmd["x"]), float(cmd["y"]))
            cx, cy = plotter.get_current_position()
            publish(TOPIC_RESPONSE, {
                "status": "ok", "action": "move_to",
                "current_x": round(cx, 2), "current_y": round(cy, 2)
            })

        elif action == "jog":
            _require_calibrated()
            plotter.move_relative(float(cmd.get("dx", 0)), float(cmd.get("dy", 0)))
            cx, cy = plotter.get_current_position()
            e1x, e1y = plotter.get_servo_position(1)
            e2x, e2y = plotter.get_servo_position(2)
            publish(TOPIC_RESPONSE, {
                "status": "ok", "action": "jog",
                "current_x": round(cx, 2), "current_y": round(cy, 2),
                "ef1_x": round(e1x, 2), "ef1_y": round(e1y, 2),
                "ef2_x": round(e2x, 2), "ef2_y": round(e2y, 2)
            })

        elif action == "get_position":
            _require_calibrated()
            cx, cy = plotter.get_current_position()
            e1x, e1y = plotter.get_servo_position(1)
            e2x, e2y = plotter.get_servo_position(2)
            publish(TOPIC_RESPONSE, {
                "status": "ok", "action": "get_position",
                "current_x": round(cx, 2), "current_y": round(cy, 2),
                "ef1_x": round(e1x, 2), "ef1_y": round(e1y, 2),
                "ef2_x": round(e2x, 2), "ef2_y": round(e2y, 2)
            })

        elif action == "set_servo":
            _require_calibrated()
            snum  = int(cmd["servo_num"])
            angle = float(cmd["angle"])
            plotter.set_servo_angle(snum, angle)
            publish(TOPIC_RESPONSE, {
                "status": "ok", "action": "set_servo",
                "servo_num": snum, "angle": angle
            })

        elif action == "emergency_stop":
            if plotter:
                plotter.emergency_stop()
            publish(TOPIC_RESPONSE, {"status": "ok", "action": "emergency_stop"})
            publish(TOPIC_STATUS, {"agent_status": "stopped"})

        else:
            publish(TOPIC_RESPONSE, {
                "status": "error", "action": action,
                "message": f"Unknown action: {action}"
            })

    except Exception as e:
        print(f"[agent] Error: {e}")
        publish(TOPIC_RESPONSE, {
            "status": "error", "action": action, "message": str(e)
        })


def _require_calibrated():
    if plotter is None or not plotter.is_calibrated:
        raise Exception("Not calibrated — send 'home' first")


# ── MQTT publish helper ───────────────────────────────────────────────────────

_mqtt_client = None

def publish(topic, payload: dict):
    global _mqtt_client
    msg = json.dumps(payload)
    try:
        _mqtt_client.publish(topic, msg.encode())
        print(f"[mqtt] → {msg}")
    except Exception as e:
        print(f"[mqtt] Publish error: {e}")


# ============================================================================
# Main loop
# ============================================================================

def main():
    global _mqtt_client

    # 1. Connect WiFi
    ip = connect_wifi()

    # 2. Onboard LED — blink to show we're alive
    led = Pin("LED", Pin.OUT)
    led.on()

    # 3. Connect MQTT
    print(f"[mqtt] Connecting to broker at {BROKER_HOST}:{BROKER_PORT}…")
    _mqtt_client = MQTTClient(
        CLIENT_ID, BROKER_HOST, port=BROKER_PORT,
        keepalive=60
    )
    _mqtt_client.set_callback(handle_command)
    _mqtt_client.connect()
    _mqtt_client.subscribe(TOPIC_COMMANDS)
    print(f"[mqtt] Connected and subscribed to {TOPIC_COMMANDS}")

    publish(TOPIC_STATUS, {"agent_status": "online", "ip": ip})

    # 4. Main loop — check for messages, blink LED
    tick = 0
    while True:
        try:
            _mqtt_client.check_msg()   # non-blocking check
        except Exception as e:
            print(f"[mqtt] Error in check_msg: {e}")
            # Try to reconnect
            time.sleep_ms(2000)
            try:
                _mqtt_client.connect()
                _mqtt_client.subscribe(TOPIC_COMMANDS)
                print("[mqtt] Reconnected")
            except Exception as e2:
                print(f"[mqtt] Reconnect failed: {e2}")

        # Blink LED every ~2s to show the loop is alive
        tick += 1
        if tick % 200 == 0:
            led.toggle()

        time.sleep_ms(10)


# ── Entry point ───────────────────────────────────────────────────────────────
main()
