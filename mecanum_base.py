import time
import math
import numpy as np
from lerobot.robots.xlerobot_mecanum import XLerobotConfig, XLerobot

class PositionalMecanumController:
    def __init__(self):
        # --- 1. EXACT PHYSICAL DIMENSIONS ---
        self.Lx = 0.30  # Meters from center of cart to front axle
        self.Ly = 0.20  # Meters from center of cart to left wheels
        self.R = 0.05   # Wheel radius in meters (100mm wheel = 0.05m radius)
        
        # --- 2. MOTOR LIMITS ---
        self.MAX_SPEED = 50.0  # Max command sent to STS3215 servos
        
        # --- 3. 1:1 TRACKING VARIABLES ---
        self.prev_time = time.time()
        self.prev_headset_x = 0.0
        self.prev_headset_z = 0.0  # Z is "forward/backward" in VR
        self.prev_headset_yaw = 0.0

    def get_action_from_position(self, headset_x, headset_z, headset_yaw):
        current_time = time.time()
        dt = current_time - self.prev_time
        
        # Prevent divide-by-zero errors if the loop runs too fast
        if dt <= 0.001: 
            return {}

        # 1. CALCULATE THE DELTA (How far did you physically walk?)
        dx = headset_x - self.prev_headset_x
        dz = headset_z - self.prev_headset_z 
        dyaw = headset_yaw - self.prev_headset_yaw

        # 2. THE AXIS SWAP (VR to Robot)
        # In VR: Z is forward/backward, X is left/right.
        # On the robot: X is forward/backward, Y is left/right.
        target_vx = dz / dt    # Robot X velocity (Forward)
        target_vy = dx / dt    # Robot Y velocity (Strafe)
        target_vz = dyaw / dt  # Robot Z rotational velocity (Spin)

        # 3. UPDATE HISTORY FOR THE NEXT FRAME
        self.prev_time = current_time
        self.prev_headset_x = headset_x
        self.prev_headset_z = headset_z
        self.prev_headset_yaw = headset_yaw

        # 4. FORMAL INVERSE KINEMATICS
        rot_factor = self.Lx + self.Ly
        
        speed_fl = (1 / self.R) * (target_vx + target_vy + (target_vz * rot_factor))
        speed_fr = (1 / self.R) * (target_vx - target_vy - (target_vz * rot_factor))
        speed_bl = (1 / self.R) * (target_vx - target_vy + (target_vz * rot_factor))
        speed_br = (1 / self.R) * (target_vx + target_vy - (target_vz * rot_factor))

        # 5. SCALE TO MOTOR LIMITS
        # Ensure the final mathematical output never exceeds the motor's max safe speed
        speeds = np.array([speed_fl, speed_fr, speed_bl, speed_br])
        max_calculated = np.max(np.abs(speeds))
        
        if max_calculated > 0:
            scale = min(self.MAX_SPEED / max_calculated, 1.0)
            speeds *= scale
            
        # Stop sending commands if you are standing perfectly still in the room
        if np.all(np.abs(speeds) < 0.1):
            return {}

        return {
            "9.pos": speeds[0], 
            "10.pos": speeds[1], 
            "11.pos": speeds[2], 
            "12.pos": speeds[3]
        }

def main():
    print("Initializing 1:1 Positional Mecanum Base...")
    robot_config = XLerobotConfig()
    robot = XLerobot(robot_config)
    
    try:
        robot.connect()
    except Exception as e:
        print(f"Failed to connect to cart: {e}")
        return
        
    # Instantiate the positional controller
    controller = PositionalMecanumController()

    try:
        while True:
            # ==========================================
            # TODO: THE NETWORK BRIDGE GOES HERE
            # Once we build the Meta Quest link, this is where 
            # we will pull the live (X, Z, Yaw) coordinates from the headset over Wi-Fi.
            # ==========================================
            current_headset_x = 0.0
            current_headset_z = 0.0
            current_headset_yaw = 0.0
            
            # Feed spatial coordinates into the physics controller
            base_action = controller.get_action_from_position(
                current_headset_x, 
                current_headset_z, 
                current_headset_yaw
            )
            
            if base_action:
                robot.send_action(base_action)
            
            time.sleep(0.02) # 50Hz control loop
            
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        robot.disconnect()

if __name__ == "__main__":
    main()