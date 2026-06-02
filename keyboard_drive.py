import time
import math
import numpy as np
from lerobot.robots.xlerobot_mecanum import XLerobotConfig, XLerobot
from lerobot.teleoperators.keyboard.teleop_keyboard import KeyboardTeleop
from lerobot.teleoperators.keyboard.configuration_keyboard import KeyboardTeleopConfig

class MecanumController:
    def __init__(self):
        # --- 1. EXACT PHYSICAL DIMENSIONS ---
        self.Lx = 0.30  # Meters from center of cart to front axle
        self.Ly = 0.20  # Meters from center of cart to left wheels
        self.R = 0.05   # Wheel radius in meters (100mm wheel = 0.05m radius)
        
        # --- 2. SPEED & ACCELERATION LIMITS ---
        self.MAX_SPEED = 50.0       # Max command sent to STS3215 servos
        self.ACCEL_RATE = 0.15      # How fast the robot ramps up to max speed (0.0 to 1.0)
        self.DECEL_RATE = 0.30      # How fast it brakes (usually faster than accel)
        
        # Internal state for smooth ramping
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0

    def smooth_value(self, current, target, is_decelerating):
        """Applies an exponential smoothing function to prevent violent jerks."""
        rate = self.DECEL_RATE if is_decelerating else self.ACCEL_RATE
        return current + rate * (target - current)

    def get_action(self, pressed_keys):
        # 1. Read raw target intents
        target_x, target_y, target_z = 0.0, 0.0, 0.0
        
        if 'i' in pressed_keys: target_x += 1.0
        if 'k' in pressed_keys: target_x -= 1.0
        if 'j' in pressed_keys: target_y += 1.0
        if 'l' in pressed_keys: target_y -= 1.0
        if 'u' in pressed_keys: target_z += 1.0
        if 'o' in pressed_keys: target_z -= 1.0

        # 2. VECTOR NORMALIZATION
        # If you press Forward(1) and Left(1), the combined vector is 1.41.
        # This prevents the robot from driving 40% faster diagonally.
        magnitude = math.hypot(target_x, target_y)
        if magnitude > 1.0:
            target_x /= magnitude
            target_y /= magnitude

        # 3. ACCELERATION RAMPING
        # Smoothly transition current speeds toward target speeds
        self.current_x = self.smooth_value(self.current_x, target_x, target_x == 0)
        self.current_y = self.smooth_value(self.current_y, target_y, target_y == 0)
        self.current_z = self.smooth_value(self.current_z, target_z, target_z == 0)

        # 4. FORMAL INVERSE KINEMATICS
        # Uses the exact mathematical matrix for a mecanum chassis
        rot_factor = self.Lx + self.Ly
        
        # The (1 / R) translates desired chassis velocity into raw wheel rotation
        speed_fl = (1 / self.R) * (self.current_x + self.current_y + (self.current_z * rot_factor))
        speed_fr = (1 / self.R) * (self.current_x - self.current_y - (self.current_z * rot_factor))
        speed_bl = (1 / self.R) * (self.current_x - self.current_y + (self.current_z * rot_factor))
        speed_br = (1 / self.R) * (self.current_x + self.current_y - (self.current_z * rot_factor))

        # 5. SCALE TO MOTOR LIMITS
        # Ensure the final mathematical output never exceeds the motor's max safe speed
        speeds = np.array([speed_fl, speed_fr, speed_bl, speed_br])
        max_calculated = np.max(np.abs(speeds))
        
        if max_calculated > 0:
            # Scale proportionally so it maintains the correct direction ratio
            scale = min(self.MAX_SPEED / max_calculated, 1.0)
            speeds *= scale
        
        # Stop sending commands if perfectly still
        if np.all(np.abs(speeds) < 0.1):
            return {}

        return {
            "9.pos": speeds[0], 
            "10.pos": speeds[1], 
            "11.pos": speeds[2], 
            "12.pos": speeds[3]
        }

def main():
    print("Initializing Smooth Mecanum Base...")
    robot_config = XLerobotConfig()
    robot = XLerobot(robot_config)
    
    try:
        robot.connect()
    except Exception as e:
        print(f"Failed to connect: {e}")
        return
        
    keyboard_config = KeyboardTeleopConfig()
    keyboard = KeyboardTeleop(keyboard_config)
    keyboard.connect()

    # Instantiate our new smooth controller
    controller = MecanumController()

    try:
        while True:
            pressed_keys = set(keyboard.get_action().keys())
            
            # Feed keys into the physics controller
            base_action = controller.get_action(pressed_keys)
            
            if base_action:
                robot.send_action(base_action)
            
            time.sleep(0.02)
            
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        robot.disconnect()
        keyboard.disconnect()

if __name__ == "__main__":
    main()