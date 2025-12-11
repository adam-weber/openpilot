#!/usr/bin/env python3
"""
Angle-based joystick control for Ford vehicles using Mode 1 (SAPP angle control)

This bypasses the model and curvature limiting to directly command steering angles.
Use this to test if angle limiting is the issue preventing tight turns.

Usage:
  1. SSH into comma device
  2. Activate Mode 1 (engage OP, disengage, engage, triple-tap GAP)
  3. Run: tools/joystick/joystick_angle_control.py --keyboard
  4. Use A/D keys to command steering angles directly
"""
import os
import argparse
import numpy as np
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.tools.lib.kbhit import KBHit
from cereal import messaging, log

MAX_ANGLE = 400.0  # Maximum steering wheel angle in degrees (+/-)
ANGLE_INCREMENT = 10.0  # Degrees to change per key press


class KeyboardAngleControl:
  def __init__(self):
    self.kb = KBHit()
    self.angle_deg = 0.0  # Current commanded angle
    self.accel = 0.0  # Gas/brake (for longitudinal control)
    self.cancel = False

  def update(self):
    key = self.kb.getch().lower()
    self.cancel = False

    if key == 'r':
      # Reset to center
      self.angle_deg = 0.0
      self.accel = 0.0
      print(f"RESET: Angle={self.angle_deg:.1f}°, Accel={self.accel:.2f}")
    elif key == 'c':
      # Cancel cruise control
      self.cancel = True
      print("CANCEL pressed")
    elif key == 'a':
      # Steer left (negative angle)
      self.angle_deg = float(np.clip(self.angle_deg - ANGLE_INCREMENT, -MAX_ANGLE, MAX_ANGLE))
      print(f"LEFT: Angle={self.angle_deg:.1f}°")
    elif key == 'd':
      # Steer right (positive angle)
      self.angle_deg = float(np.clip(self.angle_deg + ANGLE_INCREMENT, -MAX_ANGLE, MAX_ANGLE))
      print(f"RIGHT: Angle={self.angle_deg:.1f}°")
    elif key == 'w':
      # Accelerate
      self.accel = float(np.clip(self.accel + 0.1, -1, 1))
      print(f"GAS: Accel={self.accel:.2f}")
    elif key == 's':
      # Brake
      self.accel = float(np.clip(self.accel - 0.1, -1, 1))
      print(f"BRAKE: Accel={self.accel:.2f}")
    elif key == '0':
      # Quick return to center
      self.angle_deg = 0.0
      print(f"CENTER: Angle={self.angle_deg:.1f}°")
    elif key == 'q':
      # Quit
      return None
    else:
      return False
    return True


def send_angle_thread(kb):
  """
  Send angle commands directly via carControl actuators
  This bypasses the model and all curvature limiting
  """
  pm = messaging.PubMaster(['carControl'])
  sm = messaging.SubMaster(['carState'])

  rk = Ratekeeper(100, print_delay_threshold=None)

  print("\n" + "="*60)
  print("ANGLE-BASED JOYSTICK CONTROL ACTIVE")
  print("="*60)
  print("\nControls:")
  print("  A/D - Steer left/right (±10° per press)")
  print("  W/S - Gas/brake")
  print("  0   - Return to center")
  print("  R   - Reset all to zero")
  print("  C   - Cancel cruise control")
  print("  Q   - Quit")
  print(f"\nMax angle: ±{MAX_ANGLE}°")
  print("="*60 + "\n")

  while True:
    sm.update(0)

    # Update keyboard input
    result = kb.update()
    if result is None:  # Quit requested
      print("\nExiting angle control...")
      break

    # Create carControl message with direct angle command
    msg = messaging.new_message('carControl')
    msg.valid = True

    # Set actuators
    msg.carControl.enabled = True
    msg.carControl.latActive = True  # Enable lateral control
    msg.carControl.longActive = True  # Enable longitudinal control

    # Direct angle command (bypasses all limiting!)
    msg.carControl.actuators.steeringAngleDeg = kb.angle_deg

    # Longitudinal control
    msg.carControl.actuators.accel = kb.accel

    # Cancel if requested
    if kb.cancel:
      msg.carControl.enabled = False
      msg.carControl.latActive = False
      msg.carControl.longActive = False

    # Send the message
    pm.send('carControl', msg)

    # Print status every second
    if rk.frame % 100 == 0:
      cs = sm['carState']
      print(f"Commanded: {kb.angle_deg:+6.1f}° | Actual: {cs.steeringAngleDeg:+6.1f}° | "
            f"Speed: {cs.vEgo*3.6:5.1f} km/h | Accel: {kb.accel:+5.2f}")

    rk.keep_time()


def main():
  parser = argparse.ArgumentParser(
    description='Direct angle control for Ford Mode 1 testing.\n'
                'This bypasses the model and all curvature/acceleration limits.',
    formatter_class=argparse.ArgumentDefaultsHelpFormatter
  )
  parser.add_argument('--keyboard', action='store_true', help='Use keyboard control (required for now)')
  parser.add_argument('--max-angle', type=float, default=400.0, help='Maximum steering angle (degrees)')
  parser.add_argument('--increment', type=float, default=10.0, help='Angle change per keypress (degrees)')
  args = parser.parse_args()

  if not args.keyboard:
    print("ERROR: Only keyboard mode is currently supported.")
    print("Run with: --keyboard")
    exit(1)

  # Update global limits if specified
  global MAX_ANGLE, ANGLE_INCREMENT
  MAX_ANGLE = args.max_angle
  ANGLE_INCREMENT = args.increment

  # Check if Mode 1 should be active
  print("\nIMPORTANT:")
  print("1. Make sure you've activated Mode 1 (triple-tap GAP button)")
  print("2. This tool sends angle commands directly to the car")
  print("3. The car must be in gear and cruise engaged for steering to work")
  print("\nPress ENTER to start, or Ctrl+C to cancel...")
  input()

  kb = KeyboardAngleControl()
  send_angle_thread(kb)


if __name__ == '__main__':
  main()
