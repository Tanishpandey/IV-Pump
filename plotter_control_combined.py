#!/usr/bin/env python3
"""
XY Plotter Control System - Combined Version

This file contains both:
1. XYPlotter - Core plotter control class with I2C stepper motors and servos
2. PlotterControlGUI - Interactive GUI for visual plotter control

Run the GUI: python3 plotter_control_combined.py
"""

import math
import time
import warnings
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import threading
from typing import Tuple, Optional, List
from gpiozero import Button, AngularServo
from gpiozero.pins.pigpio import PiGPIOFactory
from smbus2 import SMBus
from ticlib import TicI2C, SMBus2Backend
from PIL import Image, ImageTk


# ============================================================================
# XYPlotter Class
# ============================================================================

class XYPlotter:
    """
    XY Plotter controller with automatic homing/calibration and bounds checking.
   
    Coordinate system: (0,0) is at the bottom-left corner
    """
   
    def __init__(
        self,
        i2c_bus: int,
        x_tic_address: int,
        y_tic_address: int,
        x_pulley_radius_mm: float,
        y_pulley_radius_mm: float,
        x_tics_per_revolution: int,
        y_tics_per_revolution: int,
        servo1_pin: int,
        servo2_pin: int,
        servo_gear_radius_mm: float,
        servo_tics_per_revolution: int = 180,
        servo1_offset_x_mm: float = 0.0,
        servo1_offset_y_mm: float = 0.0,
        servo2_offset_x_mm: float = 0.0,
        servo2_offset_y_mm: float = 0.0,
        servo2_reversed: bool = False,
        x_limit_min_pin: int = 17,
        x_limit_max_pin: int = 27,
        y_limit_min_pin: int = 22,
        y_limit_max_pin: int = 23,
        homing_speed_tics: int = 2000000,
        movement_speed_tics: int = 4000000,
    ):
        """
        Initialize the XY Plotter.
       
        Parameters:
        -----------
        i2c_bus : int
            I2C bus number (e.g., 1 for /dev/i2c-1)
        x_tic_address : int
            I2C address for X-axis stepper controller
        y_tic_address : int
            I2C address for Y-axis stepper controller
        x_pulley_radius_mm : float
            Radius of the X-axis pulley in millimeters
        y_pulley_radius_mm : float
            Radius of the Y-axis pulley in millimeters
        x_tics_per_revolution : int
            Number of tics per complete revolution for X-axis motor
        y_tics_per_revolution : int
            Number of tics per complete revolution for Y-axis motor
        servo1_pin : int
            GPIO pin number for first servo
        servo2_pin : int
            GPIO pin number for second servo
        servo_gear_radius_mm : float
            Radius of the gear on the servo
        servo_tics_per_revolution : int
            Degrees of rotation for servo
        servo1_offset_x_mm : float
            X offset of servo 1 from plotter reference point
        servo1_offset_y_mm : float
            Y offset of servo 1 from plotter reference point
        servo2_offset_x_mm : float
            X offset of servo 2 from plotter reference point
        servo2_offset_y_mm : float
            Y offset of servo 2 from plotter reference point
        x_limit_min_pin : int
            GPIO pin for X-axis minimum limit switch
        x_limit_max_pin : int
            GPIO pin for X-axis maximum limit switch
        y_limit_min_pin : int
            GPIO pin for Y-axis minimum limit switch
        y_limit_max_pin : int
            GPIO pin for Y-axis maximum limit switch
        homing_speed_tics : int
            Speed for homing operations
        movement_speed_tics : int
            Speed for normal movements
        """
        # Store parameters
        self.i2c_bus = i2c_bus
        self.x_tic_address = x_tic_address
        self.y_tic_address = y_tic_address
        self.x_pulley_radius_mm = x_pulley_radius_mm
        self.y_pulley_radius_mm = y_pulley_radius_mm
        self.x_tics_per_revolution = x_tics_per_revolution
        self.y_tics_per_revolution = y_tics_per_revolution
        self.servo_gear_radius_mm = servo_gear_radius_mm
        self.servo_tics_per_revolution = servo_tics_per_revolution
        self.homing_speed_tics = homing_speed_tics
        self.movement_speed_tics = movement_speed_tics
       
        # Store servo offsets
        self.servo1_offset_x_mm = servo1_offset_x_mm
        self.servo1_offset_y_mm = servo1_offset_y_mm
        self.servo2_offset_x_mm = servo2_offset_x_mm
        self.servo2_offset_y_mm = servo2_offset_y_mm
        self.servo2_reversed = servo2_reversed
       
        # Calculate conversion factors (tics per mm)
        self.x_tics_per_mm = x_tics_per_revolution / (2 * math.pi * x_pulley_radius_mm)
        self.y_tics_per_mm = y_tics_per_revolution / (2 * math.pi * y_pulley_radius_mm)
       
        # Calculate servo conversion factor (degrees per mm for rack and pinion)
        servo_circumference = 2 * math.pi * servo_gear_radius_mm
        self.servo_degrees_per_mm = servo_tics_per_revolution / servo_circumference
       
        # Setup limit switches with gpiozero (active low with pull-up)
        self.x_limit_min_pin = x_limit_min_pin
        self.x_limit_max_pin = x_limit_max_pin
        self.y_limit_min_pin = y_limit_min_pin
        self.y_limit_max_pin = y_limit_max_pin
       
        # Create Button objects for limit switches (active when pressed/low)
        self.x_limit_min = Button(x_limit_min_pin, pull_up=True)
        self.x_limit_max = Button(x_limit_max_pin, pull_up=True)
        self.y_limit_min = Button(y_limit_min_pin, pull_up=True)
        self.y_limit_max = Button(y_limit_max_pin, pull_up=True)
       
        # Initialize servos with gpiozero
        self.servo1_pin = servo1_pin
        self.servo2_pin = servo2_pin
       
        # Try to use pigpio for hardware PWM (reduces jitter)
        # Falls back to software PWM if pigpio not available
        pin_factory = None
        try:
            from gpiozero.pins.pigpio import PiGPIOFactory
            pin_factory = PiGPIOFactory()
            print("Using pigpio pin factory for servos (hardware PWM)")
        except Exception:
            # Suppress the warning since we're acknowledging it
            warnings.filterwarnings('ignore', category=UserWarning,
                                  message='.*PWMSoftwareFallback.*')
            print("Using software PWM for servos (pigpio not available)")
       
        # Use AngularServo for precise angle control
        # min_angle=0, max_angle based on servo range, initial_angle=90 (neutral)
        self.servo1 = AngularServo(
            servo1_pin,
            min_angle=0,
            max_angle=servo_tics_per_revolution,
            initial_angle=None,  # Don't move on init
            min_pulse_width=0.5/1000,  # 0.5ms
            max_pulse_width=2.5/1000,   # 2.5ms
            pin_factory=pin_factory
        )
        self.servo2 = AngularServo(
            servo2_pin,
            min_angle=0,
            max_angle=servo_tics_per_revolution,
            initial_angle=None,
            min_pulse_width=0.5/1000,
            max_pulse_width=2.5/1000,
            pin_factory=pin_factory
        )
       
        self.servo1_position = 90.0  # Track current servo positions
        self.servo2_position = 90.0
       
        # Initialize I2C bus and stepper controllers
        self.bus = SMBus(i2c_bus)
       
        # Create I2C backends for each Tic controller
        x_backend = SMBus2Backend(self.bus, x_tic_address)
        y_backend = SMBus2Backend(self.bus, y_tic_address)
       
        # Create Tic controller instances
        self.x_tic = TicI2C(x_backend)
        self.y_tic = TicI2C(y_backend)
       
        # Initialize both motors
        self._initialize_motor(self.x_tic)
        self._initialize_motor(self.y_tic)
       
        # Calibration data
        self.is_calibrated = False
        self.x_max_mm = None
        self.y_max_mm = None
        self.x_max_tics = None
        self.y_max_tics = None
        self.current_x_mm = 0.0
        self.current_y_mm = 0.0
   
    def _initialize_motor(self, tic):
        """Initialize a stepper motor controller."""
        tic.halt_and_set_position(0)
        tic.energize()
        tic.exit_safe_start()
   
    def _is_limit_switch_pressed(self, limit_switch: Button) -> bool:
        """Check if a limit switch is pressed."""
        return limit_switch.is_pressed
   
    def home_and_calibrate(self, x_backoff_mm: float = 5.0,
                          y_backoff_mm: float = 5.0) -> Tuple[float, float]:
        """
        Home the plotter and determine working area.
       
        Process:
        1. Move to minimum limits (0, 0)
        2. Find maximum limits
        3. Back off from limits
        4. Set working area
       
        Parameters:
        -----------
        x_backoff_mm : float
            Distance to back off from X maximum limit (mm)
        y_backoff_mm : float
            Distance to back off from Y maximum limit (mm)
       
        Returns:
        --------
        Tuple[float, float]
            Maximum working area (x_max, y_max) in millimeters
        """
        print("Starting homing and calibration sequence...")
       
        # Set homing speed
        self.x_tic.set_max_speed(self.homing_speed_tics)
        self.y_tic.set_max_speed(self.homing_speed_tics)
       
        # Phase 1: Home to minimum limits (bottom-left corner)
        print("Phase 1: Moving to minimum limits (0, 0)...")
        self._home_axis_to_minimum(self.x_tic, self.x_limit_min, "X")
        self._home_axis_to_minimum(self.y_tic, self.y_limit_min, "Y")
       
        # Set (0, 0) position
        self.x_tic.halt_and_set_position(0)
        self.y_tic.halt_and_set_position(0)
        self.current_x_mm = 0.0
        self.current_y_mm = 0.0
       
        print("Homed to (0, 0)")
        time.sleep(0.5)
       
        # Phase 2: Find maximum limits
        print("Phase 2: Finding maximum limits...")
        self.x_max_tics = self._find_axis_maximum(self.x_tic, self.x_limit_max, "X")
        self.y_max_tics = self._find_axis_maximum(self.y_tic, self.y_limit_max, "Y")
       
        # Convert to mm
        self.x_max_mm = self.x_max_tics / self.x_tics_per_mm
        self.y_max_mm = self.y_max_tics / self.y_tics_per_mm
       
        print(f"Maximum limits found: X={self.x_max_mm:.2f}mm, Y={self.y_max_mm:.2f}mm")
       
        # Phase 3: Back off from maximum limits
        print(f"Phase 3: Backing off {x_backoff_mm}mm from X limit, {y_backoff_mm}mm from Y limit...")
        x_backoff_tics = int(-x_backoff_mm * self.x_tics_per_mm)
        y_backoff_tics = int(-y_backoff_mm * self.y_tics_per_mm)
       
        self._move_axis_relative(self.x_tic, x_backoff_tics)
        self._move_axis_relative(self.y_tic, y_backoff_tics)
       
        # Update maximum working area
        self.x_max_mm -= x_backoff_mm
        self.y_max_mm -= y_backoff_mm
        self.x_max_tics = int(self.x_max_mm * self.x_tics_per_mm)
        self.y_max_tics = int(self.y_max_mm * self.y_tics_per_mm)
       
        # Move to (0, 0) for final position
        print("Returning to (0, 0)...")
        self.x_tic.set_target_position(0)
        self.y_tic.set_target_position(0)
        self._wait_for_movement(self.x_tic)
        self._wait_for_movement(self.y_tic)
       
        self.current_x_mm = 0.0
        self.current_y_mm = 0.0
       
        # Restore normal movement speed
        self.x_tic.set_max_speed(self.movement_speed_tics)
        self.y_tic.set_max_speed(self.movement_speed_tics)
       
        self.is_calibrated = True
        print(f"Calibration complete! Working area: {self.x_max_mm:.2f}mm × {self.y_max_mm:.2f}mm")
       
        return self.x_max_mm, self.y_max_mm
   
    def _home_axis_to_minimum(self, tic, limit_switch: Button, axis_name: str):
        """Move an axis in negative direction until limit switch is hit."""
        print(f"  Homing {axis_name}-axis to minimum...")
       
        # Move in negative direction
        tic.set_target_velocity(-self.homing_speed_tics)
       
        # Wait until limit switch is pressed
        while not self._is_limit_switch_pressed(limit_switch):
            time.sleep(0.01)
       
        # Stop immediately
        tic.halt_and_hold()
        time.sleep(0.1)
        print(f"  {axis_name}-axis minimum limit reached")
   
    def _find_axis_maximum(self, tic, limit_switch: Button, axis_name: str) -> int:
        """
        Move axis in positive direction to find maximum limit.
       
        Returns:
        --------
        int
            Position in tics when maximum limit is reached
        """
        print(f"  Finding {axis_name}-axis maximum...")
       
        # Move in positive direction
        tic.set_target_velocity(self.homing_speed_tics)
       
        # Wait until limit switch is pressed
        while not self._is_limit_switch_pressed(limit_switch):
            time.sleep(0.01)
       
        # Stop and get position
        tic.halt_and_hold()
        time.sleep(0.1)
        max_position = tic.get_current_position()
       
        print(f"  {axis_name}-axis maximum limit at {max_position} tics")
        return max_position
   
    def _move_axis_relative(self, tic, tics: int):
        """Move an axis relative to current position."""
        current_pos = tic.get_current_position()
        target_pos = current_pos + tics
        tic.set_target_position(target_pos)
        self._wait_for_movement(tic)
   
    def _wait_for_movement(self, tic):
        """Wait for a motor to finish moving."""
        while tic.get_current_velocity() != 0:
            time.sleep(0.01)
        time.sleep(0.1)  # Small settling time
   
    def move_to(self, x_mm: float, y_mm: float, check_bounds: bool = True):
        """
        Move to absolute position in millimeters.
       
        Parameters:
        -----------
        x_mm : float
            X position in millimeters
        y_mm : float
            Y position in millimeters
        check_bounds : bool
            If True, verify position is within calibrated bounds
       
        Raises:
        -------
        RuntimeError
            If plotter is not calibrated or position is out of bounds
        """
        if not self.is_calibrated:
            raise RuntimeError("Plotter must be calibrated before moving. Call home_and_calibrate() first.")
       
        if check_bounds and not self.is_within_bounds(x_mm, y_mm):
            raise ValueError(f"Position ({x_mm}, {y_mm}) is out of bounds. "
                           f"Valid range: (0-{self.x_max_mm:.2f}, 0-{self.y_max_mm:.2f})")
       
        # Convert to tics
        x_tics = int(x_mm * self.x_tics_per_mm)
        y_tics = int(y_mm * self.y_tics_per_mm)
       
        # Move both axes
        self.x_tic.set_target_position(x_tics)
        self.y_tic.set_target_position(y_tics)
       
        # Wait for completion
        self._wait_for_movement(self.x_tic)
        self._wait_for_movement(self.y_tic)
       
        # Update current position
        self.current_x_mm = x_mm
        self.current_y_mm = y_mm
   
    def move_relative(self, dx_mm: float, dy_mm: float, check_bounds: bool = True):
        """
        Move relative to current position.
       
        Parameters:
        -----------
        dx_mm : float
            Change in X position (mm)
        dy_mm : float
            Change in Y position (mm)
        check_bounds : bool
            If True, verify final position is within bounds
        """
        new_x = self.current_x_mm + dx_mm
        new_y = self.current_y_mm + dy_mm
        self.move_to(new_x, new_y, check_bounds)
   
    def is_within_bounds(self, x_mm: float, y_mm: float) -> bool:
        """Check if a position is within the calibrated working area."""
        if not self.is_calibrated:
            return False
        return (0 <= x_mm <= self.x_max_mm) and (0 <= y_mm <= self.y_max_mm)
   
    def get_current_position(self) -> Tuple[float, float]:
        """Get current position in millimeters."""
        return (self.current_x_mm, self.current_y_mm)
   
    def get_working_area(self) -> Tuple[float, float]:
        """Get maximum working area in millimeters."""
        if not self.is_calibrated:
            raise RuntimeError("Plotter not calibrated")
        return (self.x_max_mm, self.y_max_mm)
   
    def move_servo(self, servo_num: int, distance_mm: float):
        """
        Move a servo by a distance (for rack and pinion).
       
        Parameters:
        -----------
        servo_num : int
            Servo number (1 or 2)
        distance_mm : float
            Distance to move in millimeters (positive or negative)
        """
        # Calculate angle change needed
        angle_change = distance_mm * self.servo_degrees_per_mm
       
        # Get current position and calculate new position
        current_angle = self.servo1_position if servo_num == 1 else self.servo2_position
        new_angle = current_angle + angle_change
       
        # Clamp to valid servo range (0-180 degrees, or adjust based on your servo)
        new_angle = max(0, min(self.servo_tics_per_revolution, new_angle))
       
        # Set servo position
        self.set_servo_angle(servo_num, new_angle)
   
    def set_servo_angle(self, servo_num: int, angle: float):
        """
        Set a servo to a specific angle.
       
        Parameters:
        -----------
        servo_num : int
            Servo number (1 or 2)
        angle : float
            Angle in degrees (0 to servo_tics_per_revolution, typically 0-180)
       
        Raises:
        -------
        ValueError
            If servo_num is not 1 or 2, or if angle is out of range
        """
        if servo_num not in [1, 2]:
            raise ValueError("servo_num must be 1 or 2")
       
        if not 0 <= angle <= self.servo_tics_per_revolution:
            raise ValueError(f"Angle must be between 0 and {self.servo_tics_per_revolution}")
       
        # Set angle using gpiozero AngularServo
        if servo_num == 1:
            self.servo1.angle = angle
            self.servo1_position = angle
        else:
            # Reverse angle for servo 2 if mounted flipped
            actual_angle = (self.servo_tics_per_revolution - angle) if self.servo2_reversed else angle
            self.servo2.angle = actual_angle
            self.servo2_position = angle  # Store the logical position
       
        # Give servo time to move
        time.sleep(0.3)
   
    def press_button(self, servo_num: int, press_distance_mm: float = 5.0,
                    press_duration: float = 0.5):
        """
        Press a button using the specified servo.
       
        This moves the servo forward by press_distance_mm, waits, then returns.
       
        Parameters:
        -----------
        servo_num : int
            Servo number (1 or 2)
        press_distance_mm : float
            Distance to move forward to press the button (mm)
        press_duration : float
            How long to hold the button press (seconds)
        """
        # Move servo forward to press
        self.move_servo(servo_num, press_distance_mm)
       
        # Hold the press
        time.sleep(press_duration)
       
        # Return to original position
        self.move_servo(servo_num, -press_distance_mm)
   
    def get_servo_position(self, servo_num: int) -> Tuple[float, float]:
        """
        Get the current actual position of a servo button presser in mm.
       
        Parameters:
        -----------
        servo_num : int
            Servo number (1 or 2)
       
        Returns:
        --------
        Tuple[float, float]
            (x, y) position of the servo button presser in mm
        """
        if servo_num == 1:
            return (self.current_x_mm + self.servo1_offset_x_mm,
                   self.current_y_mm + self.servo1_offset_y_mm)
        elif servo_num == 2:
            return (self.current_x_mm + self.servo2_offset_x_mm,
                   self.current_y_mm + self.servo2_offset_y_mm)
        else:
            raise ValueError("servo_num must be 1 or 2")
   
    def set_servo_offsets(self, servo_num: int, offset_x_mm: float, offset_y_mm: float):
        """
        Update the offset for a servo button presser.
       
        Parameters:
        -----------
        servo_num : int
            Servo number (1 or 2)
        offset_x_mm : float
            X offset from plotter reference point (mm)
        offset_y_mm : float
            Y offset from plotter reference point (mm)
        """
        if servo_num == 1:
            self.servo1_offset_x_mm = offset_x_mm
            self.servo1_offset_y_mm = offset_y_mm
            print(f"Servo 1 offset updated to ({offset_x_mm:.2f}, {offset_y_mm:.2f}) mm")
        elif servo_num == 2:
            self.servo2_offset_x_mm = offset_x_mm
            self.servo2_offset_y_mm = offset_y_mm
            print(f"Servo 2 offset updated to ({offset_x_mm:.2f}, {offset_y_mm:.2f}) mm")
        else:
            raise ValueError("servo_num must be 1 or 2")
   
    def _select_best_servo(self, target_x_mm: float, target_y_mm: float,
                          check_bounds: bool = True) -> Optional[int]:
        """
        Select which servo is best to press a button at the target position.
       
        Considers:
        1. Whether the plotter can reach the position with each servo
        2. Distance from current position
       
        Parameters:
        -----------
        target_x_mm : float
            Target X position for button press
        target_y_mm : float
            Target Y position for button press
        check_bounds : bool
            If True, only consider servos that keep plotter within bounds
       
        Returns:
        --------
        Optional[int]
            Servo number (1 or 2) or None if neither servo can reach
        """
        # Calculate where plotter needs to be for each servo
        plotter_x_for_servo1 = target_x_mm - self.servo1_offset_x_mm
        plotter_y_for_servo1 = target_y_mm - self.servo1_offset_y_mm
       
        plotter_x_for_servo2 = target_x_mm - self.servo2_offset_x_mm
        plotter_y_for_servo2 = target_y_mm - self.servo2_offset_y_mm
       
        # Check if each servo position is within bounds
        servo1_in_bounds = True
        servo2_in_bounds = True
       
        if check_bounds and self.is_calibrated:
            servo1_in_bounds = self.is_within_bounds(plotter_x_for_servo1, plotter_y_for_servo1)
            servo2_in_bounds = self.is_within_bounds(plotter_x_for_servo2, plotter_y_for_servo2)
       
        # If only one is in bounds, use that one
        if servo1_in_bounds and not servo2_in_bounds:
            return 1
        if servo2_in_bounds and not servo1_in_bounds:
            return 2
        if not servo1_in_bounds and not servo2_in_bounds:
            return None  # Neither can reach
       
        # Both are in bounds - choose the closer one
        distance_servo1 = math.sqrt(
            (plotter_x_for_servo1 - self.current_x_mm) ** 2 +
            (plotter_y_for_servo1 - self.current_y_mm) ** 2
        )
        distance_servo2 = math.sqrt(
            (plotter_x_for_servo2 - self.current_x_mm) ** 2 +
            (plotter_y_for_servo2 - self.current_y_mm) ** 2
        )
       
        return 1 if distance_servo1 <= distance_servo2 else 2
   
    def move_and_press(self, target_x_mm: float, target_y_mm: float,
                      press_distance_mm: float = 5.0,
                      press_duration: float = 0.5,
                      servo_num: Optional[int] = None,
                      check_bounds: bool = True) -> int:
        """
        Move to a position and press a button with intelligent servo selection.
       
        This function:
        1. Determines which servo is best positioned to reach the target
        2. Accounts for servo offsets from the plotter reference point
        3. Moves the plotter so the selected servo aligns with the target
        4. Presses the button
       
        Parameters:
        -----------
        target_x_mm : float
            Target X position for button press (absolute coordinates)
        target_y_mm : float
            Target Y position for button press (absolute coordinates)
        press_distance_mm : float
            Distance to move servo forward to press button (default: 5.0 mm)
        press_duration : float
            How long to hold the button press (default: 0.5 seconds)
        servo_num : Optional[int]
            Force use of specific servo (1 or 2), or None for automatic selection
        check_bounds : bool
            If True, verify positions are within calibrated bounds
       
        Returns:
        --------
        int
            Servo number that was used (1 or 2)
       
        Raises:
        -------
        RuntimeError
            If plotter is not calibrated
        ValueError
            If no servo can reach the target position
        """
        if not self.is_calibrated:
            raise RuntimeError("Plotter must be calibrated before moving. Call home_and_calibrate() first.")
       
        # Select servo if not specified
        if servo_num is None:
            servo_num = self._select_best_servo(target_x_mm, target_y_mm, check_bounds)
            if servo_num is None:
                raise ValueError(
                    f"Target position ({target_x_mm:.2f}, {target_y_mm:.2f}) cannot be reached "
                    f"with either servo. Check bounds and servo offsets."
                )
            print(f"Auto-selected servo {servo_num} for target ({target_x_mm:.2f}, {target_y_mm:.2f})")
        elif servo_num not in [1, 2]:
            raise ValueError("servo_num must be 1, 2, or None")
       
        # Calculate where the plotter needs to move
        if servo_num == 1:
            plotter_x = target_x_mm - self.servo1_offset_x_mm
            plotter_y = target_y_mm - self.servo1_offset_y_mm
        else:  # servo_num == 2
            plotter_x = target_x_mm - self.servo2_offset_x_mm
            plotter_y = target_y_mm - self.servo2_offset_y_mm
       
        # Move plotter to position
        self.move_to(plotter_x, plotter_y, check_bounds)
       
        # Press button with selected servo
        self.press_button(servo_num, press_distance_mm, press_duration)
       
        return servo_num
   
    def emergency_stop(self):
        """Emergency stop - halt all motors immediately."""
        print("EMERGENCY STOP!")
        self.x_tic.halt_and_hold()
        self.y_tic.halt_and_hold()
        self.x_tic.deenergize()
        self.y_tic.deenergize()
   
    def cleanup(self):
        """Clean up GPIO and I2C resources."""
        print("Cleaning up...")
        self.x_tic.deenergize()
        self.y_tic.deenergize()
       
        # Close gpiozero devices
        self.servo1.close()
        self.servo2.close()
        self.x_limit_min.close()
        self.x_limit_max.close()
        self.y_limit_min.close()
        self.y_limit_max.close()
       
        self.bus.close()
        print("Cleanup complete")
   
    def __enter__(self):
        """Context manager entry."""
        return self
   
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - cleanup resources."""
        self.cleanup()


# ============================================================================
# PlotterControlGUI Class
# ============================================================================

class PlotterControlGUI:
    """
    Integrated GUI for XY plotter control.
   
    Two modes:
    1. Live Mode - Click executes immediately
    2. Sequence Mode - Build list, then execute with Run button
    """
   
    def __init__(self, plotter_config: dict):
        """
        Initialize the plotter control GUI.
       
        Parameters:
        -----------
        plotter_config : dict
            Configuration dictionary for XYPlotter initialization
        """
        self.root = tk.Tk()
        self.root.title("XY Plotter Control")
       
        # Plotter configuration and instance
        self.plotter_config = plotter_config
        self.plotter = None
        self.plotter_initialized = False
        self.is_homed = False
       
        # Image data
        self.image = None
        self.image_tk = None
        self.image_path = None
        self.real_width_mm = None
        self.real_height_mm = None
        self.scale_factor = 1.0
       
        # Points and state
        self.points = []  # List of (x_mm, y_mm) tuples
        self.point_markers = []  # Visual markers on canvas
        self.current_mode = tk.StringVar(value="sequence")  # "live" or "sequence"
        self.is_executing = False
        self.current_executing_index = None
       
        # Settings
        self.press_distance = tk.DoubleVar(value=5.0)
        self.offset_x = tk.DoubleVar(value=0.0)
        self.offset_y = tk.DoubleVar(value=0.0)
       
        # Build UI
        self._create_ui()
       
    def _create_ui(self):
        """Create the user interface."""
        # Main layout: Left side = controls, Right side = image canvas
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
       
        # Left panel - Controls (with scrollbar)
        control_container = ttk.Frame(main_frame, width=300)
        control_container.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))
        control_container.pack_propagate(False)
       
        # Create canvas and scrollbar for controls
        control_canvas = tk.Canvas(control_container, width=280, highlightthickness=0)
        scrollbar = ttk.Scrollbar(control_container, orient="vertical", command=control_canvas.yview)
        control_frame = ttk.Frame(control_canvas)
       
        control_frame.bind(
            "<Configure>",
            lambda e: control_canvas.configure(scrollregion=control_canvas.bbox("all"))
        )
       
        control_canvas.create_window((0, 0), window=control_frame, anchor="nw")
        control_canvas.configure(yscrollcommand=scrollbar.set)
       
        control_canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
       
        # Bind mousewheel for scrolling
        def _on_mousewheel(event):
            control_canvas.yview_scroll(int(-1*(event.delta/120)), "units")
        control_canvas.bind_all("<MouseWheel>", _on_mousewheel)
       
        # Right panel - Canvas
        canvas_frame = ttk.Frame(main_frame)
        canvas_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
       
        self._create_controls(control_frame)
        self._create_canvas(canvas_frame)
       
    def _create_controls(self, parent):
        """Create control panel."""
       
        # === Image Section ===
        image_section = ttk.LabelFrame(parent, text="Image", padding=10)
        image_section.pack(fill=tk.X, pady=(0, 10))
       
        ttk.Button(image_section, text="Load Image",
                  command=self._load_image).pack(fill=tk.X, pady=2)
       
        self.image_info_label = ttk.Label(image_section, text="No image loaded",
                                         wraplength=250)
        self.image_info_label.pack(fill=tk.X, pady=2)
       
        # === Plotter Section ===
        plotter_section = ttk.LabelFrame(parent, text="Plotter", padding=10)
        plotter_section.pack(fill=tk.X, pady=(0, 10))
       
        ttk.Button(plotter_section, text="Initialize & Home",
                  command=self._initialize_plotter).pack(fill=tk.X, pady=2)
       
        self.plotter_status_label = ttk.Label(plotter_section,
                                             text="Not initialized",
                                             foreground="red")
        self.plotter_status_label.pack(fill=tk.X, pady=2)
       
        # === Mode Section ===
        mode_section = ttk.LabelFrame(parent, text="Mode", padding=10)
        mode_section.pack(fill=tk.X, pady=(0, 10))
       
        ttk.Radiobutton(mode_section, text="Live Mode",
                       variable=self.current_mode, value="live",
                       command=self._on_mode_change).pack(anchor=tk.W)
        ttk.Label(mode_section, text="  Click → Execute immediately",
                 font=('TkDefaultFont', 9),
                 foreground='gray').pack(anchor=tk.W, padx=(20, 0))
       
        ttk.Radiobutton(mode_section, text="Sequence Mode",
                       variable=self.current_mode, value="sequence",
                       command=self._on_mode_change).pack(anchor=tk.W, pady=(5, 0))
        ttk.Label(mode_section, text="  Build list → Run all",
                 font=('TkDefaultFont', 9),
                 foreground='gray').pack(anchor=tk.W, padx=(20, 0))
       
        # === Settings Section ===
        settings_section = ttk.LabelFrame(parent, text="Settings", padding=10)
        settings_section.pack(fill=tk.X, pady=(0, 10))
       
        # Press distance
        ttk.Label(settings_section, text="Press Distance (mm):").pack(anchor=tk.W)
        press_frame = ttk.Frame(settings_section)
        press_frame.pack(fill=tk.X, pady=(0, 5))
        ttk.Entry(press_frame, textvariable=self.press_distance,
                 width=10).pack(side=tk.LEFT)
       
        # X offset
        ttk.Label(settings_section, text="X Offset (mm):").pack(anchor=tk.W)
        offset_x_frame = ttk.Frame(settings_section)
        offset_x_frame.pack(fill=tk.X, pady=(0, 5))
        ttk.Entry(offset_x_frame, textvariable=self.offset_x,
                 width=10).pack(side=tk.LEFT)
       
        # Y offset
        ttk.Label(settings_section, text="Y Offset (mm):").pack(anchor=tk.W)
        offset_y_frame = ttk.Frame(settings_section)
        offset_y_frame.pack(fill=tk.X, pady=(0, 5))
        ttk.Entry(offset_y_frame, textvariable=self.offset_y,
                 width=10).pack(side=tk.LEFT)
       
        # === Points Section ===
        points_section = ttk.LabelFrame(parent, text="Points", padding=10)
        points_section.pack(fill=tk.BOTH, expand=True, pady=(0, 10))
       
        self.points_label = ttk.Label(points_section, text="Points: 0")
        self.points_label.pack(anchor=tk.W, pady=(0, 5))
       
        # Points listbox
        list_frame = ttk.Frame(points_section)
        list_frame.pack(fill=tk.BOTH, expand=True)
       
        scrollbar = ttk.Scrollbar(list_frame)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
       
        self.points_listbox = tk.Listbox(list_frame,
                                         yscrollcommand=scrollbar.set,
                                         height=8)
        self.points_listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.config(command=self.points_listbox.yview)
       
        # Point management buttons
        btn_frame = ttk.Frame(points_section)
        btn_frame.pack(fill=tk.X, pady=(5, 0))
       
        ttk.Button(btn_frame, text="Delete Selected",
                  command=self._delete_selected_point).pack(side=tk.LEFT,
                                                           fill=tk.X,
                                                           expand=True,
                                                           padx=(0, 2))
        ttk.Button(btn_frame, text="Clear All",
                  command=self._clear_all_points).pack(side=tk.LEFT,
                                                       fill=tk.X,
                                                       expand=True,
                                                       padx=(2, 0))
       
        # === End Effector Setup Section ===
        setup_section = ttk.LabelFrame(parent, text="End Effector Setup", padding=10)
        setup_section.pack(fill=tk.X, pady=(0, 10))
       
        ttk.Button(setup_section, text="Setup Tool",
                  command=self._open_setup_dialog).pack(fill=tk.X)
       
        ttk.Label(setup_section, text="Load racks into servo gears",
                 font=('TkDefaultFont', 9),
                 foreground='gray').pack(anchor=tk.W)
       
        # === Manual Jog Section ===
        jog_section = ttk.LabelFrame(parent, text="Manual Jog", padding=10)
        jog_section.pack(fill=tk.X, pady=(0, 10))
       
        ttk.Button(jog_section, text="Jog Control",
                  command=self._open_jog_dialog).pack(fill=tk.X)
       
        ttk.Label(jog_section, text="Manual axis movement",
                 font=('TkDefaultFont', 9),
                 foreground='gray').pack(anchor=tk.W)
       
        # === Execution Section ===
        exec_section = ttk.LabelFrame(parent, text="Execution", padding=10)
        exec_section.pack(fill=tk.X)
       
        self.run_button = ttk.Button(exec_section, text="Run Sequence",
                                     command=self._run_sequence,
                                     state=tk.DISABLED)
        self.run_button.pack(fill=tk.X, pady=(0, 5))
       
        self.stop_button = ttk.Button(exec_section, text="Emergency Stop",
                                      command=self._emergency_stop,
                                      state=tk.DISABLED)
        self.stop_button.pack(fill=tk.X)
       
        self.status_label = ttk.Label(exec_section, text="Ready",
                                      foreground="green")
        self.status_label.pack(fill=tk.X, pady=(5, 0))
       
    def _create_canvas(self, parent):
        """Create image display canvas."""
        # Canvas with scrollbars
        canvas_container = ttk.Frame(parent)
        canvas_container.pack(fill=tk.BOTH, expand=True)
       
        # Scrollbars
        h_scrollbar = ttk.Scrollbar(canvas_container, orient=tk.HORIZONTAL)
        h_scrollbar.pack(side=tk.BOTTOM, fill=tk.X)
       
        v_scrollbar = ttk.Scrollbar(canvas_container, orient=tk.VERTICAL)
        v_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
       
        # Canvas
        self.canvas = tk.Canvas(canvas_container,
                               bg='gray',
                               xscrollcommand=h_scrollbar.set,
                               yscrollcommand=v_scrollbar.set)
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
       
        h_scrollbar.config(command=self.canvas.xview)
        v_scrollbar.config(command=self.canvas.yview)
       
        # Bind click event
        self.canvas.bind('<Button-1>', self._on_canvas_click)
       
        # Instructions
        self.canvas.create_text(
            400, 300,
            text="Load an image to begin",
            font=('Arial', 16),
            fill='white',
            tags='instructions'
        )
       
    def _load_image(self):
        """Load image and set dimensions."""
        filepath = filedialog.askopenfilename(
            title="Select Image",
            filetypes=[
                ("Image files", "*.png *.jpg *.jpeg *.bmp *.gif *.tiff"),
                ("All files", "*.*")
            ]
        )
       
        if not filepath:
            return
       
        # Get dimensions dialog
        dialog = tk.Toplevel(self.root)
        dialog.title("Image Dimensions")
        dialog.geometry("300x150")
        dialog.transient(self.root)
        dialog.grab_set()
       
        ttk.Label(dialog, text="Enter real-world dimensions:").pack(pady=10)
       
        # Width
        width_frame = ttk.Frame(dialog)
        width_frame.pack(pady=5)
        ttk.Label(width_frame, text="Width (mm):").pack(side=tk.LEFT)
        width_var = tk.DoubleVar(value=200.0)
        ttk.Entry(width_frame, textvariable=width_var, width=10).pack(side=tk.LEFT)
       
        # Height
        height_frame = ttk.Frame(dialog)
        height_frame.pack(pady=5)
        ttk.Label(height_frame, text="Height (mm):").pack(side=tk.LEFT)
        height_var = tk.DoubleVar(value=150.0)
        ttk.Entry(height_frame, textvariable=height_var, width=10).pack(side=tk.LEFT)
       
        def on_ok():
            self.real_width_mm = width_var.get()
            self.real_height_mm = height_var.get()
            self.image_path = filepath
            dialog.destroy()
            self._display_image()
       
        ttk.Button(dialog, text="OK", command=on_ok).pack(pady=10)
       
        dialog.wait_window()
       
    def _display_image(self):
        """Display loaded image on canvas."""
        try:
            # Load image
            self.image = Image.open(self.image_path)
           
            # Scale to fit canvas (max 800x600)
            max_width = 800
            max_height = 600
           
            img_width, img_height = self.image.size
            scale_w = max_width / img_width
            scale_h = max_height / img_height
            self.scale_factor = min(scale_w, scale_h, 1.0)
           
            display_width = int(img_width * self.scale_factor)
            display_height = int(img_height * self.scale_factor)
           
            # Resize for display
            display_image = self.image.resize((display_width, display_height),
                                             Image.Resampling.LANCZOS)
            self.image_tk = ImageTk.PhotoImage(display_image)
           
            # Clear canvas and display
            self.canvas.delete('all')
            self.canvas.create_image(0, 0, anchor=tk.NW, image=self.image_tk)
            self.canvas.config(scrollregion=(0, 0, display_width, display_height))
           
            # Update info
            info = f"Image: {display_width}×{display_height}px\n"
            info += f"Real: {self.real_width_mm}×{self.real_height_mm}mm"
            self.image_info_label.config(text=info)
           
            # Clear existing points
            self._clear_all_points()
           
        except Exception as e:
            messagebox.showerror("Error", f"Failed to load image: {e}")
           
    def _initialize_plotter(self):
        """Initialize and home the plotter."""
        if self.plotter_initialized:
            result = messagebox.askyesno("Re-initialize?",
                                        "Plotter already initialized. Re-home?")
            if not result:
                return
       
        def init_thread():
            try:
                self._update_status("Initializing plotter...", "orange")
                self.plotter_status_label.config(text="Initializing...",
                                                foreground="orange")
               
                # Initialize plotter
                if self.plotter is None:
                    self.plotter = XYPlotter(**self.plotter_config)
               
                # Home and calibrate
                self._update_status("Homing and calibrating...", "orange")
                max_x, max_y = self.plotter.home_and_calibrate()
               
                self.plotter_initialized = True
                self.is_homed = True
               
                self._update_status("Plotter ready", "green")
                self.plotter_status_label.config(
                    text=f"Ready ({max_x:.1f}×{max_y:.1f}mm)",
                    foreground="green"
                )
               
                # Enable stop button
                self.stop_button.config(state=tk.NORMAL)
               
                # Enable run button in sequence mode
                if self.current_mode.get() == "sequence":
                    self.run_button.config(state=tk.NORMAL)
               
            except Exception as e:
                self._update_status(f"Error: {e}", "red")
                self.plotter_status_label.config(text=f"Error: {e}",
                                                foreground="red")
                messagebox.showerror("Initialization Error", str(e))
       
        # Run in thread to prevent UI freeze
        thread = threading.Thread(target=init_thread, daemon=True)
        thread.start()
       
    def _on_canvas_click(self, event):
        """Handle canvas click."""
        if self.image is None:
            messagebox.showwarning("No Image", "Please load an image first")
            return
       
        if not self.plotter_initialized:
            messagebox.showwarning("Plotter Not Ready",
                                  "Please initialize plotter first")
            return
       
        if self.is_executing:
            # Can't add points during execution
            return
       
        # Get canvas coordinates
        canvas_x = self.canvas.canvasx(event.x)
        canvas_y = self.canvas.canvasy(event.y)
       
        # Convert to mm coordinates
        pixel_x = canvas_x / self.scale_factor
        pixel_y = canvas_y / self.scale_factor
       
        mm_x = (pixel_x / self.image.width) * self.real_width_mm
        mm_y = self.real_height_mm - (pixel_y / self.image.height) * self.real_height_mm
       
        # Apply offsets
        mm_x += self.offset_x.get()
        mm_y += self.offset_y.get()
       
        # Add point
        self._add_point(mm_x, mm_y, canvas_x, canvas_y)
       
        # In live mode, execute immediately
        if self.current_mode.get() == "live":
            self._execute_live_point(len(self.points) - 1)
           
    def _add_point(self, mm_x: float, mm_y: float,
                   canvas_x: float, canvas_y: float):
        """Add a point to the list."""
        # Add to data
        self.points.append((mm_x, mm_y))
       
        # Add to listbox
        self.points_listbox.insert(tk.END,
                                   f"{len(self.points)}: ({mm_x:.1f}, {mm_y:.1f})")
       
        # Draw marker
        color = "yellow" if self.current_mode.get() == "sequence" else "cyan"
        marker = self.canvas.create_oval(
            canvas_x - 5, canvas_y - 5,
            canvas_x + 5, canvas_y + 5,
            fill=color, outline="black", width=2,
            tags=f'point_{len(self.points)-1}'
        )
        self.point_markers.append(marker)
       
        # Draw number
        self.canvas.create_text(
            canvas_x, canvas_y - 15,
            text=str(len(self.points)),
            fill="white", font=('Arial', 10, 'bold'),
            tags=f'point_{len(self.points)-1}'
        )
       
        # Update count
        self.points_label.config(text=f"Points: {len(self.points)}")
       
    def _delete_selected_point(self):
        """Delete selected point from list."""
        if self.is_executing:
            return
       
        selection = self.points_listbox.curselection()
        if not selection:
            return
       
        index = selection[0]
       
        # Remove from data
        self.points.pop(index)
       
        # Remove from listbox
        self.points_listbox.delete(index)
       
        # Remove marker
        self.canvas.delete(f'point_{index}')
       
        # Rebuild display
        self._refresh_points_display()
       
    def _clear_all_points(self):
        """Clear all points."""
        if self.is_executing:
            return
       
        self.points.clear()
        self.points_listbox.delete(0, tk.END)
       
        # Remove all markers
        for marker in self.point_markers:
            self.canvas.delete(marker)
        self.point_markers.clear()
       
        # Remove all point tags
        self.canvas.delete('point')
       
        self.points_label.config(text="Points: 0")
       
    def _refresh_points_display(self):
        """Refresh points display after deletion."""
        # Clear listbox
        self.points_listbox.delete(0, tk.END)
       
        # Remove all point markers
        for i in range(len(self.point_markers) + 10):  # Extra to be safe
            self.canvas.delete(f'point_{i}')
        self.point_markers.clear()
       
        # Redraw all points
        for i, (mm_x, mm_y) in enumerate(self.points):
            # Convert back to canvas coordinates
            pixel_x = (mm_x - self.offset_x.get()) * self.image.width / self.real_width_mm
            pixel_y = self.image.height - ((mm_y - self.offset_y.get()) *
                                           self.image.height / self.real_height_mm)
            canvas_x = pixel_x * self.scale_factor
            canvas_y = pixel_y * self.scale_factor
           
            # Add to listbox
            self.points_listbox.insert(tk.END, f"{i+1}: ({mm_x:.1f}, {mm_y:.1f})")
           
            # Draw marker
            color = "yellow" if self.current_mode.get() == "sequence" else "cyan"
            marker = self.canvas.create_oval(
                canvas_x - 5, canvas_y - 5,
                canvas_x + 5, canvas_y + 5,
                fill=color, outline="black", width=2,
                tags=f'point_{i}'
            )
            self.point_markers.append(marker)
           
            # Draw number
            self.canvas.create_text(
                canvas_x, canvas_y - 15,
                text=str(i + 1),
                fill="white", font=('Arial', 10, 'bold'),
                tags=f'point_{i}'
            )
       
        self.points_label.config(text=f"Points: {len(self.points)}")
       
    def _execute_live_point(self, index: int):
        """Execute a single point in live mode."""
        def execute():
            try:
                self.is_executing = True
                self.current_executing_index = index
               
                mm_x, mm_y = self.points[index]
               
                # Highlight current point (red)
                self.canvas.itemconfig(f'point_{index}', fill='red')
               
                self._update_status(f"Moving to ({mm_x:.1f}, {mm_y:.1f})...",
                                  "orange")
               
                # Execute move and press
                press_dist = self.press_distance.get()
                servo_used = self.plotter.move_and_press(
                    mm_x, mm_y,
                    press_distance_mm=press_dist
                )
               
                self._update_status(
                    f"Point {index+1} done (servo {servo_used})",
                    "green"
                )
               
                # Remove point marker in live mode
                self.canvas.delete(f'point_{index}')
                self.points_listbox.delete(index)
                self.points.pop(index)
               
                # Refresh display
                self._refresh_points_display()
               
            except Exception as e:
                self._update_status(f"Error: {e}", "red")
                messagebox.showerror("Execution Error", str(e))
            finally:
                self.is_executing = False
                self.current_executing_index = None
       
        # Run in thread
        thread = threading.Thread(target=execute, daemon=True)
        thread.start()
       
    def _run_sequence(self):
        """Run all points in sequence mode."""
        if not self.points:
            messagebox.showinfo("No Points", "No points to execute")
            return
       
        def execute():
            try:
                self.is_executing = True
                self.run_button.config(state=tk.DISABLED)
               
                for i, (mm_x, mm_y) in enumerate(self.points):
                    if not self.is_executing:  # Check for stop
                        break
                   
                    self.current_executing_index = i
                   
                    # Highlight current point
                    self.canvas.itemconfig(f'point_{i}', fill='red')
                    self.points_listbox.selection_clear(0, tk.END)
                    self.points_listbox.selection_set(i)
                    self.points_listbox.see(i)
                   
                    self._update_status(
                        f"Point {i+1}/{len(self.points)}: ({mm_x:.1f}, {mm_y:.1f})",
                        "orange"
                    )
                   
                    # Execute
                    press_dist = self.press_distance.get()
                    servo_used = self.plotter.move_and_press(
                        mm_x, mm_y,
                        press_distance_mm=press_dist
                    )
                   
                    # Mark as done (green)
                    self.canvas.itemconfig(f'point_{i}', fill='green')
                   
                    time.sleep(0.2)  # Brief pause between points
               
                self._update_status("Sequence complete", "green")
               
            except Exception as e:
                self._update_status(f"Error: {e}", "red")
                messagebox.showerror("Execution Error", str(e))
            finally:
                self.is_executing = False
                self.current_executing_index = None
                self.run_button.config(state=tk.NORMAL)
       
        # Run in thread
        thread = threading.Thread(target=execute, daemon=True)
        thread.start()
       
    def _emergency_stop(self):
        """Emergency stop."""
        self.is_executing = False
        if self.plotter:
            self.plotter.emergency_stop()
        self._update_status("EMERGENCY STOP", "red")
       
    def _on_mode_change(self):
        """Handle mode change."""
        mode = self.current_mode.get()
       
        if mode == "sequence":
            self.run_button.config(state=tk.NORMAL if self.plotter_initialized
                                  else tk.DISABLED)
        else:  # live mode
            self.run_button.config(state=tk.DISABLED)
       
        # Update point colors
        self._refresh_points_display()
       
    def _update_status(self, message: str, color: str = "black"):
        """Update status label."""
        self.status_label.config(text=message, foreground=color)
        self.root.update_idletasks()
   
    def _open_jog_dialog(self):
        """Open manual jog control dialog."""
        if not self.plotter_initialized:
            messagebox.showwarning("Plotter Not Ready",
                                  "Please initialize plotter first")
            return
       
        # Create jog dialog
        dialog = tk.Toplevel(self.root)
        dialog.title("Manual Jog Control")
        dialog.geometry("500x600")
        dialog.transient(self.root)
       
        # Instructions
        instructions = ttk.Label(
            dialog,
            text="Manual control of X/Y axes\n"
                 "Use buttons to jog plotter position",
            justify=tk.CENTER,
            padding=10
        )
        instructions.pack(fill=tk.X)
       
        ttk.Separator(dialog, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=10)
       
        # Position displays
        pos_frame = ttk.LabelFrame(dialog, text="Current Positions", padding=10)
        pos_frame.pack(fill=tk.X, padx=10, pady=5)
       
        # Gantry position
        gantry_label = ttk.Label(pos_frame, text="Gantry Position:",
                                font=('TkDefaultFont', 10, 'bold'))
        gantry_label.pack(anchor=tk.W)
       
        gantry_pos_label = ttk.Label(pos_frame, text="X: 0.0mm, Y: 0.0mm",
                                     font=('TkDefaultFont', 12))
        gantry_pos_label.pack(anchor=tk.W, padx=20)
       
        ttk.Separator(pos_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=5)
       
        # End effector 1
        ef1_label = ttk.Label(pos_frame, text="End Effector 1 (Servo 1):",
                             font=('TkDefaultFont', 10, 'bold'))
        ef1_label.pack(anchor=tk.W, pady=(5, 0))
       
        ef1_pos_label = ttk.Label(pos_frame, text="X: 0.0mm, Y: 0.0mm",
                                 font=('TkDefaultFont', 12))
        ef1_pos_label.pack(anchor=tk.W, padx=20)
       
        ttk.Separator(pos_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=5)
       
        # End effector 2
        ef2_label = ttk.Label(pos_frame, text="End Effector 2 (Servo 2):",
                             font=('TkDefaultFont', 10, 'bold'))
        ef2_label.pack(anchor=tk.W, pady=(5, 0))
       
        ef2_pos_label = ttk.Label(pos_frame, text="X: 0.0mm, Y: 0.0mm",
                                 font=('TkDefaultFont', 12))
        ef2_pos_label.pack(anchor=tk.W, padx=20)
       
        def update_positions():
            """Update position displays."""
            try:
                # Get gantry position
                x, y = self.plotter.get_current_position()
                gantry_pos_label.config(text=f"X: {x:.1f}mm, Y: {y:.1f}mm")
               
                # Get end effector positions
                ef1_x, ef1_y = self.plotter.get_servo_position(1)
                ef1_pos_label.config(text=f"X: {ef1_x:.1f}mm, Y: {ef1_y:.1f}mm")
               
                ef2_x, ef2_y = self.plotter.get_servo_position(2)
                ef2_pos_label.config(text=f"X: {ef2_x:.1f}mm, Y: {ef2_y:.1f}mm")
            except Exception as e:
                print(f"Error updating positions: {e}")
       
        # Jog step size
        step_frame = ttk.LabelFrame(dialog, text="Step Size", padding=10)
        step_frame.pack(fill=tk.X, padx=10, pady=5)
       
        step_size = tk.DoubleVar(value=10.0)
       
        step_buttons = ttk.Frame(step_frame)
        step_buttons.pack(fill=tk.X)
       
        ttk.Radiobutton(step_buttons, text="1mm", variable=step_size,
                       value=1.0).pack(side=tk.LEFT, expand=True)
        ttk.Radiobutton(step_buttons, text="5mm", variable=step_size,
                       value=5.0).pack(side=tk.LEFT, expand=True)
        ttk.Radiobutton(step_buttons, text="10mm", variable=step_size,
                       value=10.0).pack(side=tk.LEFT, expand=True)
        ttk.Radiobutton(step_buttons, text="50mm", variable=step_size,
                       value=50.0).pack(side=tk.LEFT, expand=True)
       
        # Jog controls
        jog_frame = ttk.LabelFrame(dialog, text="Jog Controls", padding=10)
        jog_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
       
        def jog(axis, direction):
            """Jog an axis in a direction."""
            try:
                step = step_size.get()
                if axis == 'x':
                    self.plotter.move_relative(direction * step, 0)
                else:  # y axis
                    self.plotter.move_relative(0, direction * step)
                update_positions()
            except Exception as e:
                messagebox.showerror("Jog Error", str(e))
       
        # Create jog button layout
        #     [Y+]
        # [X-] [0] [X+]
        #     [Y-]
       
        button_grid = ttk.Frame(jog_frame)
        button_grid.pack(expand=True)
       
        # Y+ button (top)
        ttk.Button(button_grid, text="Y+", width=8,
                  command=lambda: jog('y', 1)).grid(row=0, column=1, padx=5, pady=5)
       
        # X- button (left)
        ttk.Button(button_grid, text="X-", width=8,
                  command=lambda: jog('x', -1)).grid(row=1, column=0, padx=5, pady=5)
       
        # Home button (center)
        def go_home():
            try:
                self.plotter.move_to(0, 0)
                update_positions()
            except Exception as e:
                messagebox.showerror("Home Error", str(e))
       
        ttk.Button(button_grid, text="Home\n(0,0)", width=8,
                  command=go_home).grid(row=1, column=1, padx=5, pady=5)
       
        # X+ button (right)
        ttk.Button(button_grid, text="X+", width=8,
                  command=lambda: jog('x', 1)).grid(row=1, column=2, padx=5, pady=5)
       
        # Y- button (bottom)
        ttk.Button(button_grid, text="Y-", width=8,
                  command=lambda: jog('y', -1)).grid(row=2, column=1, padx=5, pady=5)
       
        # Go to position
        goto_frame = ttk.LabelFrame(dialog, text="Go To Position", padding=10)
        goto_frame.pack(fill=tk.X, padx=10, pady=5)
       
        goto_entry_frame = ttk.Frame(goto_frame)
        goto_entry_frame.pack(fill=tk.X)
       
        ttk.Label(goto_entry_frame, text="X:").pack(side=tk.LEFT)
        goto_x = tk.DoubleVar(value=0.0)
        ttk.Entry(goto_entry_frame, textvariable=goto_x, width=8).pack(side=tk.LEFT, padx=5)
       
        ttk.Label(goto_entry_frame, text="Y:").pack(side=tk.LEFT)
        goto_y = tk.DoubleVar(value=0.0)
        ttk.Entry(goto_entry_frame, textvariable=goto_y, width=8).pack(side=tk.LEFT, padx=5)
       
        def go_to_position():
            try:
                self.plotter.move_to(goto_x.get(), goto_y.get())
                update_positions()
            except Exception as e:
                messagebox.showerror("Move Error", str(e))
       
        ttk.Button(goto_entry_frame, text="Go",
                  command=go_to_position).pack(side=tk.LEFT, padx=5)
       
        # Initial position update
        update_positions()
       
        # Refresh button
        ttk.Button(dialog, text="Refresh Positions",
                  command=update_positions).pack(pady=5)
       
        # Close button
        ttk.Button(dialog, text="Close",
                  command=dialog.destroy).pack(pady=10)
   
    def _open_setup_dialog(self):
        """Open end effector setup dialog."""
        if not self.plotter_initialized:
            messagebox.showwarning("Plotter Not Ready",
                                  "Please initialize plotter first")
            return
       
        # Create setup dialog
        dialog = tk.Toplevel(self.root)
        dialog.title("End Effector Setup")
        dialog.geometry("400x500")
        dialog.transient(self.root)
       
        # Instructions
        instructions = ttk.Label(
            dialog,
            text="Load racks into servo gears:\n"
                 "1. Select servo\n"
                 "2. Push Forward (max extension)\n"
                 "3. Back Up until rack engages\n"
                 "4. Repeat for other servo",
            justify=tk.LEFT,
            padding=10
        )
        instructions.pack(fill=tk.X)
       
        ttk.Separator(dialog, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=10)
       
        # Servo selection
        servo_frame = ttk.LabelFrame(dialog, text="Select Servo", padding=10)
        servo_frame.pack(fill=tk.X, padx=10, pady=5)
       
        selected_servo = tk.IntVar(value=1)
        ttk.Radiobutton(servo_frame, text="Servo 1",
                       variable=selected_servo, value=1).pack(anchor=tk.W)
        ttk.Radiobutton(servo_frame, text="Servo 2",
                       variable=selected_servo, value=2).pack(anchor=tk.W)
       
        # Current angle display
        angle_frame = ttk.LabelFrame(dialog, text="Current Position", padding=10)
        angle_frame.pack(fill=tk.X, padx=10, pady=5)
       
        angle_label = ttk.Label(angle_frame, text="Angle: 90.0°",
                               font=('TkDefaultFont', 12, 'bold'))
        angle_label.pack()
       
        def update_angle_display():
            """Update the angle display."""
            servo = selected_servo.get()
            if servo == 1:
                angle = self.plotter.servo1_position
            else:
                angle = self.plotter.servo2_position
            angle_label.config(text=f"Angle: {angle:.1f}°")
       
        # Push forward button (max extension)
        push_frame = ttk.LabelFrame(dialog, text="Step 1: Push Forward", padding=10)
        push_frame.pack(fill=tk.X, padx=10, pady=5)
       
        def push_forward():
            servo = selected_servo.get()
            try:
                # Set to maximum angle (full extension)
                self.plotter.set_servo_angle(servo, 180)
                update_angle_display()
                messagebox.showinfo(
                    "Ready",
                    f"Servo {servo} at maximum extension.\n"
                    "Now back up the rack until it engages with the gear."
                )
            except Exception as e:
                messagebox.showerror("Error", str(e))
       
        ttk.Button(push_frame, text="▶ Push Forward (Max)",
                  command=push_forward).pack(fill=tk.X)
       
        # Back up controls
        backup_frame = ttk.LabelFrame(dialog, text="Step 2: Back Up Rack", padding=10)
        backup_frame.pack(fill=tk.X, padx=10, pady=5)
       
        def backup_large():
            """Back up by large step (10 degrees)."""
            servo = selected_servo.get()
            try:
                current = self.plotter.servo1_position if servo == 1 else self.plotter.servo2_position
                new_angle = max(0, current - 10)
                self.plotter.set_servo_angle(servo, new_angle)
                update_angle_display()
            except Exception as e:
                messagebox.showerror("Error", str(e))
       
        def backup_small():
            """Back up by small step (1 degree)."""
            servo = selected_servo.get()
            try:
                current = self.plotter.servo1_position if servo == 1 else self.plotter.servo2_position
                new_angle = max(0, current - 1)
                self.plotter.set_servo_angle(servo, new_angle)
                update_angle_display()
            except Exception as e:
                messagebox.showerror("Error", str(e))
       
        def forward_small():
            """Move forward by small step (1 degree)."""
            servo = selected_servo.get()
            try:
                current = self.plotter.servo1_position if servo == 1 else self.plotter.servo2_position
                new_angle = min(180, current + 1)
                self.plotter.set_servo_angle(servo, new_angle)
                update_angle_display()
            except Exception as e:
                messagebox.showerror("Error", str(e))
       
        btn_frame = ttk.Frame(backup_frame)
        btn_frame.pack(fill=tk.X)
       
        ttk.Button(btn_frame, text="◀◀ Back Up (10°)",
                  command=backup_large).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
        ttk.Button(btn_frame, text="◀ Back Up (1°)",
                  command=backup_small).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
        ttk.Button(btn_frame, text="Forward (1°) ▶",
                  command=forward_small).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
       
        # Reset button
        reset_frame = ttk.LabelFrame(dialog, text="Reset", padding=10)
        reset_frame.pack(fill=tk.X, padx=10, pady=5)
       
        def reset_neutral():
            """Reset servo to neutral position (90 degrees)."""
            servo = selected_servo.get()
            try:
                self.plotter.set_servo_angle(servo, 90)
                update_angle_display()
            except Exception as e:
                messagebox.showerror("Error", str(e))
       
        ttk.Button(reset_frame, text="Reset to Neutral (90°)",
                  command=reset_neutral).pack(fill=tk.X)
       
        # Update display on servo selection change
        selected_servo.trace_add('write', lambda *args: update_angle_display())
       
        # Initial update
        update_angle_display()
       
        # Close button
        ttk.Button(dialog, text="Close",
                  command=dialog.destroy).pack(pady=10)
       
    def run(self):
        """Start the GUI."""
        self.root.mainloop()
       
    def cleanup(self):
        """Clean up resources."""
        if self.plotter:
            self.plotter.cleanup()


# ============================================================================
# Main Entry Point
# ============================================================================

def main():
    """Main entry point for the plotter control GUI."""
   
    # XY Plotter configuration
    # EDIT THESE VALUES FOR YOUR SETUP!
    config = {
        'i2c_bus': 1,
        'x_tic_address': 14,
        'y_tic_address': 15,
        'x_pulley_radius_mm': 12.58,
        'y_pulley_radius_mm': 12.58,
        'x_tics_per_revolution': 200,
        'y_tics_per_revolution': 200,
        'servo1_pin': 18,
        'servo2_pin': 19,
        'servo_gear_radius_mm': 5.0,
       
        # Servo offsets (MEASURE THESE ON YOUR END EFFECTOR!)
        'servo1_offset_x_mm': -20.0,
        'servo1_offset_y_mm': 0.0,
        'servo2_offset_x_mm': 35.0,
        'servo2_offset_y_mm': 0.0,
        'servo2_reversed': True,  # Set to True if servo 2 is mounted flipped
    }
   
    # Create and run GUI
    app = PlotterControlGUI(config)
   
    try:
        app.run()
    finally:
        app.cleanup()


if __name__ == "__main__":
    main()