#!/usr/bin/env python3
"""
PiperX VR Teleoperation Script
Control PiperX robot using Quest 3 VR controllers via Vuer
Only uses left controller, robot moves only when trigger is pressed
"""

import sys
import threading
import time
import numpy as np
import argparse
import json
from datetime import datetime
from asyncio import sleep
import asyncio
from inverse_kinematics.piper_pinocchio_ik import PiperIK

try:
    from piper_sdk import C_PiperInterface_V2
except ImportError:
    print("Error: piper_sdk not found. Please install with: pip install piper-sdk")
    sys.exit(1)

try:
    from vuer import Vuer, VuerSession
    from vuer.schemas import MotionControllers
except ImportError:
    print("Error: vuer not found. Please install with: pip install vuer")
    sys.exit(1)

# Add the parent directory to the path
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from xemb_scripts.cartesian_control.transformations import (
    rotation_matrix_to_quaternion,
    rmat_to_euler as rotation_matrix_to_euler
)

# --- Configuration Parameters ---
CONTROL_PERIOD = 0.05  # 20Hz control loop
LINEAR_VELOCITY_SCALE = 2.0   # Scale factor for linear movement
ANGULAR_VELOCITY_SCALE = 3.0  # Scale factor for angular movement
TRIGGER_THRESHOLD = 0.1       # Minimum trigger value to enable movement
DEAD_ZONE = 0.01             # Dead zone for controller movement
MAX_POSITION_DELTA = 0.5     # Maximum position change per control cycle
MAX_ORIENTATION_DELTA = 0.5  # Maximum orientation change per control cycle

# --- Global Variables ---
g_controller_data = {
    'position': np.zeros(3),
    'orientation': np.zeros(3),  # Euler angles
    'trigger_value': 0.0,
    'buttons': {},
    'connected': False
}
g_reference_position = np.zeros(3)
g_reference_orientation = np.zeros(3)
g_reference_set = False
g_stop_motion = False
g_gripper_command = None
g_lock = threading.Lock()

class PiperXVRTeleop:
    def __init__(self, can_interface='can0', robot_id=0):
        """Initialize the PiperX VR teleoperation."""
        self.can_interface = can_interface
        self.robot_id = robot_id
        
        # Initialize Pinocchio IK solver
        print(f"[Robot {robot_id}] Initializing Pinocchio IK solver...")
        try:
            self.ik = PiperIK(max_iterations=100, tolerance=1e-3, damping=1e-3)
            print(f"[Robot {robot_id}] ✅ Pinocchio IK solver initialized successfully")
        except Exception as e:
            print(f"[Robot {robot_id}] ❌ Failed to initialize Pinocchio IK: {e}")
            sys.exit(1)
        
        # Initialize PiperX robot
        print(f"[Robot {robot_id}] Connecting to PiperX on {can_interface}...")
        try:
            self.piper = C_PiperInterface_V2(can_interface)
            self.piper.ConnectPort()
            
            # Wait for connection
            timeout = 5.0
            start_time = time.time()
            while not self.piper.get_connect_status():
                if time.time() - start_time > timeout:
                    print(f"[Robot {robot_id}] ❌ Connection timeout for {can_interface}")
                    sys.exit(1)
                time.sleep(0.01)
            
            # Enable the robot
            start_time = time.time()
            while not self.piper.EnablePiper():
                if time.time() - start_time > timeout:
                    print(f"[Robot {robot_id}] ❌ Enable timeout for {can_interface}")
                    sys.exit(1)
                time.sleep(0.01)
            
            print(f"[Robot {robot_id}] ✅ PiperX connected on {can_interface}")
            
        except Exception as e:
            print(f"[Robot {robot_id}] ❌ Failed to connect to PiperX: {e}")
            sys.exit(1)
        
        # Load calibration data
        self.load_calibration()
        
        # Initialize robot to neutral position
        self.init_robot()
        
        # Current state
        self.current_joints_rad = np.array(self.neutral_positions) * np.pi / 180000.0  # Convert to radians
        self.current_gripper_rad = self.gripper_closed * np.pi / 180000.0  # Convert to radians
        
        # Get initial end-effector position
        self.update_current_pose()
        
        print(f"[Robot {self.robot_id}] ✅ PiperX VR teleoperation initialized")
    
    def load_calibration(self):
        """Load PiperX calibration data."""
        try:
            with open('piperx_calibration.json', 'r') as f:
                calib = json.load(f)
            self.neutral_positions = calib['neutral_positions']
            self.gripper_closed = calib['gripper_limits'][0]
            self.gripper_open = calib['gripper_limits'][1]
            print(f"[Robot {self.robot_id}] ✅ Loaded calibration: neutral={self.neutral_positions}")
        except Exception as e:
            print(f"[Robot {self.robot_id}] Warning: Could not load calibration: {e}")
            print(f"[Robot {self.robot_id}] Using default values")
            self.neutral_positions = [0, 0, 0, 0, 0, 0]
            self.gripper_closed = 0
            self.gripper_open = 30000
    
    def init_robot(self):
        """Initialize robot to neutral position."""
        try:
            # Set gripper to closed position
            self.piper.GripperCtrl(int(self.gripper_closed), 1000, 0x01, 0)
            
            # Set joints to neutral position
            self.piper.MotionCtrl_2(0x01, 0x01, 100, 0x00)
            self.piper.JointCtrl(*[int(pos) for pos in self.neutral_positions])
            
            # Wait for movement to complete
            time.sleep(2.0)
            
            print(f"[Robot {self.robot_id}] ✅ Robot initialized to neutral position")
            
        except Exception as e:
            print(f"[Robot {self.robot_id}] ❌ Failed to initialize robot: {e}")
            sys.exit(1)
    
    def update_current_pose(self):
        """Update current end-effector pose using forward kinematics."""
        try:
            full_config = self.ik.get_full_configuration(self.current_joints_rad)
            fk_result = self.ik.forward_kinematics(full_config)
            
            if fk_result:
                self.current_position = np.array(fk_result['position'])
                self.current_orientation = np.array(fk_result['orientation'])
                return True
            else:
                print(f"[Robot {self.robot_id}] Warning: Forward kinematics failed")
                return False
                
        except Exception as e:
            print(f"[Robot {self.robot_id}] Warning: Error in forward kinematics: {e}")
            return False
    
    def apply_cartesian_velocity(self, linear_vel, angular_vel):
        """Apply Cartesian velocity using Pinocchio IK."""
        try:
            # Create twist vector [vx, vy, vz, wx, wy, wz]
            twist = np.array([
                linear_vel[0], linear_vel[1], linear_vel[2],
                angular_vel[0], angular_vel[1], angular_vel[2]
            ])
            
            # Get full configuration (with gripper joints)
            current_full_config = self.ik.get_full_configuration(self.current_joints_rad)
            
            # Compute joint velocities using Pinocchio IK
            joint_velocities = self.ik.velocity_ik(twist, current_full_config)
            
            if joint_velocities is None:
                print(f"[Robot {self.robot_id}] Warning: Velocity IK failed")
                return False
            
            # Only use arm joint velocities (first 6)
            arm_joint_velocities = joint_velocities[:6]
            
            # Integrate to get new joint positions
            dt = CONTROL_PERIOD
            new_joints_rad = self.current_joints_rad + arm_joint_velocities * dt
            
            # Check joint limits
            if not self.ik.check_joint_limits(self.ik.get_full_configuration(new_joints_rad)):
                print(f"[Robot {self.robot_id}] Warning: Joint limits would be violated")
                return False
            
            # Convert to PiperX format (millidegrees) and send command
            new_joints_piper = new_joints_rad * 180000.0 / np.pi
            
            self.piper.MotionCtrl_2(0x01, 0x01, 100, 0x00)
            self.piper.JointCtrl(*[int(angle) for angle in new_joints_piper])
            
            # Update internal state
            self.current_joints_rad = new_joints_rad
            self.update_current_pose()
            
            return True
            
        except Exception as e:
            print(f"[Robot {self.robot_id}] Error in velocity control: {e}")
            return False
    
    def update_gripper(self, target_gripper_rad):
        """Update gripper position."""
        try:
            # Convert to PiperX format and send command
            gripper_piper = target_gripper_rad * 180000.0 / np.pi
            gripper_piper = np.clip(gripper_piper, self.gripper_closed, self.gripper_open)
            
            self.piper.GripperCtrl(int(gripper_piper), 1000, 0x01, 0)
            self.current_gripper_rad = gripper_piper * np.pi / 180000.0
            
        except Exception as e:
            print(f"[Robot {self.robot_id}] Error updating gripper: {e}")
    
    def run(self):
        """Main control loop."""
        global g_stop_motion, g_gripper_command, g_reference_set
        
        print(f"[Robot {self.robot_id}] 🚀 PiperX VR teleoperation active!")
        print(f"[Robot {self.robot_id}] Controls:")
        print(f"[Robot {self.robot_id}]   Left Controller Position: Robot position (X, Y, Z)")
        print(f"[Robot {self.robot_id}]   Left Controller Orientation: Robot orientation (Roll, Pitch, Yaw)")
        print(f"[Robot {self.robot_id}]   Left Trigger: Enable movement (analog scaling)")
        print(f"[Robot {self.robot_id}]   A Button: Open gripper")
        print(f"[Robot {self.robot_id}]   B Button: Close gripper")
        print(f"[Robot {self.robot_id}]   Squeeze: Emergency stop")
        print(f"[Robot {self.robot_id}]   Ctrl+C: Exit")
        
        # Show initial position
        if hasattr(self, 'current_position'):
            pos = self.current_position
            print(f"[Robot {self.robot_id}] Initial position: x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f}")
        
        try:
            while True:
                loop_start = time.time()
                
                linear_vel = np.zeros(3)
                angular_vel = np.zeros(3)
                
                with g_lock:
                    # Check if controller is connected and trigger is pressed
                    if (g_controller_data['connected'] and 
                        g_controller_data['trigger_value'] > TRIGGER_THRESHOLD and
                        not g_stop_motion):
                        
                        # Set reference position on first trigger press
                        if not g_reference_set:
                            g_reference_position[:] = g_controller_data['position']
                            g_reference_orientation[:] = g_controller_data['orientation']
                            g_reference_set = True
                            print(f"[Robot {self.robot_id}] 📍 Reference position set")
                        
                        # Calculate position and orientation deltas
                        pos_delta = g_controller_data['position'] - g_reference_position
                        ori_delta = g_controller_data['orientation'] - g_reference_orientation
                        
                        # Apply dead zone
                        pos_delta = np.where(np.abs(pos_delta) < DEAD_ZONE, 0, pos_delta)
                        ori_delta = np.where(np.abs(ori_delta) < DEAD_ZONE, 0, ori_delta)
                        
                        # Limit maximum deltas
                        pos_delta = np.clip(pos_delta, -MAX_POSITION_DELTA, MAX_POSITION_DELTA)
                        ori_delta = np.clip(ori_delta, -MAX_ORIENTATION_DELTA, MAX_ORIENTATION_DELTA)
                        
                        # Convert to velocities
                        trigger_scale = g_controller_data['trigger_value']
                        linear_vel = pos_delta * LINEAR_VELOCITY_SCALE * trigger_scale
                        angular_vel = ori_delta * ANGULAR_VELOCITY_SCALE * trigger_scale
                        
                    elif g_stop_motion:
                        # Emergency stop - send current position to stop motion
                        current_joints_piper = self.current_joints_rad * 180000.0 / np.pi
                        self.piper.MotionCtrl_2(0x01, 0x01, 100, 0x00)
                        self.piper.JointCtrl(*[int(angle) for angle in current_joints_piper])
                        g_stop_motion = False
                        g_reference_set = False  # Reset reference
                        print(f"[Robot {self.robot_id}] 🛑 Robot stopped")
                    
                    # Handle gripper commands
                    if g_gripper_command == 'open':
                        self.update_gripper(self.gripper_open * np.pi / 180000.0)
                        g_gripper_command = None
                        print(f"[Robot {self.robot_id}] 🤏 Gripper opening")
                    elif g_gripper_command == 'close':
                        self.update_gripper(self.gripper_closed * np.pi / 180000.0)
                        g_gripper_command = None
                        print(f"[Robot {self.robot_id}] ✊ Gripper closing")
                
                # Apply velocities if any
                if np.any(linear_vel) or np.any(angular_vel):
                    success = self.apply_cartesian_velocity(linear_vel, angular_vel)
                    if not success:
                        print(f"[Robot {self.robot_id}] ❌ Motion command failed")
                
                # Maintain control frequency
                elapsed = time.time() - loop_start
                sleep_time = max(0, CONTROL_PERIOD - elapsed)
                time.sleep(sleep_time)
                
        except KeyboardInterrupt:
            print(f"\n[Robot {self.robot_id}] 🛑 Teleoperation stopped by user")
        finally:
            self.shutdown()
    
    def shutdown(self):
        """Shutdown the robot safely."""
        print(f"[Robot {self.robot_id}] Shutting down...")
        try:
            # Return to neutral position
            self.piper.MotionCtrl_2(0x01, 0x01, 100, 0x00)
            self.piper.JointCtrl(*[int(pos) for pos in self.neutral_positions])
            
            # Close gripper
            self.piper.GripperCtrl(int(self.gripper_closed), 1000, 0x01, 0)
            
            time.sleep(1.0)
            print(f"[Robot {self.robot_id}] ✅ Robot returned to neutral position")
            
        except Exception as e:
            print(f"[Robot {self.robot_id}] Error during shutdown: {e}")
        finally:
            print(f"[Robot {self.robot_id}] 🔚 PiperX VR teleoperation terminated")

def setup_vr_system():
    """Setup the VR controller system using Vuer."""
    global g_controller_data, g_stop_motion, g_gripper_command
    
    print("Setting up VR controller system...")
    print("IMPORTANT: Make sure ngrok is running: ngrok http 8012")
    print("Then access the ngrok URL from Meta Quest Browser")
    print("Make sure your Quest 3 controllers are paired and active!")
    
    # Create Vuer app
    app = Vuer(host='0.0.0.0', queries=dict(grid=False), queue_len=3)
    
    @app.add_handler("CONTROLLER_MOVE")
    async def controller_handler(event, session):
        """Handle controller movement events - only process left controller."""
        global g_controller_data, g_stop_motion, g_gripper_command
        
        data = event.value
        
        with g_lock:
            # Only process left controller
            if 'left' in data:
                try:
                    # Parse transformation matrix
                    transform_matrix = np.array(data['left']).reshape(4, 4, order='F')
                    
                    # Extract position (last column)
                    position = transform_matrix[:3, 3]
                    
                    # Extract rotation matrix and convert to Euler angles
                    rotation_matrix = transform_matrix[:3, :3]
                    orientation = rotation_matrix_to_euler(rotation_matrix)
                    
                    # Update controller data
                    g_controller_data['position'] = position
                    g_controller_data['orientation'] = orientation
                    g_controller_data['connected'] = True
                    
                    # Process left controller state
                    if 'leftState' in data:
                        state = data['leftState']
                        
                        # Update trigger value
                        g_controller_data['trigger_value'] = state.get('triggerValue', 0.0)
                        
                        # Handle button presses
                        if state.get('aButton', False):
                            g_gripper_command = 'open'
                        elif state.get('bButton', False):
                            g_gripper_command = 'close'
                        
                        # Handle emergency stop
                        if state.get('squeeze', False):
                            g_stop_motion = True
                        
                        # Store all button states
                        g_controller_data['buttons'] = state
                    
                except Exception as e:
                    print(f"Error processing left controller data: {e}")
                    g_controller_data['connected'] = False
            else:
                # No left controller data
                g_controller_data['connected'] = False
    
    @app.spawn(start=True)
    async def main_vr(session: VuerSession):
        """Main VR session that sets up motion controllers."""
        print("Setting up Motion Controllers...")
        
        # Enable only left controller streaming
        session.upsert @ MotionControllers(
            stream=True, 
            key="motion-controller", 
            left=True,   # Only left controller
            right=False  # Disable right controller
        )
        
        print("✅ VR Motion Controller activated!")
        print("Left controller is now controlling the robot")
        
        while True:
            await sleep(1)
    
    return app

def vr_controller_thread(app):
    """Thread function to run the VR controller system."""
    try:
        print("🎮 Starting VR controller system...")
        app.run()
    except Exception as e:
        print(f"VR controller error: {e}")
        import traceback
        traceback.print_exc()

def main():
    """Main function."""
    global g_controller_data
    
    parser = argparse.ArgumentParser(description='PiperX VR Teleoperation')
    parser.add_argument('--can', default='can0', help='CAN interface for PiperX (default: can0)')
    parser.add_argument('--robot-id', type=int, default=0, help='Robot ID for multi-robot setup (default: 0)')
    args = parser.parse_args()
    
    try:
        print(f"Starting VR teleoperation for Robot {args.robot_id}...")
        
        # Setup VR system
        vr_app = setup_vr_system()
        
        # Start VR controller in separate thread
        vr_thread = threading.Thread(target=vr_controller_thread, args=(vr_app,))
        vr_thread.daemon = True
        vr_thread.start()
        
        # Wait for VR system to initialize
        print("Waiting for VR controller connection...")
        while not g_controller_data['connected']:
            time.sleep(0.1)
        
        print("✅ VR controller connected!")
        
        # Start the robot teleop
        teleop = PiperXVRTeleop(can_interface=args.can, robot_id=args.robot_id)
        teleop.run()
            
    except KeyboardInterrupt:
        print("\nTeleoperation stopped by user.")
    except Exception as e:
        print(f"Error in teleoperation: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()