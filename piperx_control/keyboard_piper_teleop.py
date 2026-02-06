#!/usr/bin/env python3
"""
PiperX IK Keyboard Teleoperation Script
Direct keyboard control of PiperX robot using improved Pinocchio IK solver
with intelligent joint limit scaling and singularity avoidance.
"""

import sys
import select
import tty
import termios
import threading
import time
import numpy as np
import argparse
import json
from inverse_kinematics.piper_pinocchio_ik import PiperIK

try:
    from piper_sdk import C_PiperInterface_V2
except ImportError:
    print("Error: piper_sdk not found. Please install with: pip install piper-sdk")
    sys.exit(1)

# --- Configuration Parameters ---
CONTROL_PERIOD = 0.05  # 20Hz control loop (slower for larger steps)
LINEAR_VELOCITY_STEP = 0.2   # m/s - much smaller for safer movement
ANGULAR_VELOCITY_STEP = 0.2   # rad/s - much smaller for safer movement
DEBUG_VERBOSE = False  # Set to True for detailed debugging

# --- Keyboard Mapping ---
KEY_MAPPINGS = {
    # Linear Control
    'w': (0, 1),   # Forward (+X)
    's': (0, -1),  # Backward (-X)
    'a': (1, 1),   # Left (+Y)
    'd': (1, -1),  # Right (-Y)
    'q': (2, 1),   # Up (+Z)
    'e': (2, -1),  # Down (-Z)
    
    # Angular Control
    'z': (5, 1),   # Yaw Left (+Z)
    'c': (5, -1),  # Yaw Right (-Z)
    'r': (4, 1),   # Pitch Up (+Y)
    'f': (4, -1),  # Pitch Down (-Y)
    't': (3, 1),   # Roll Left (+X)
    'g': (3, -1),  # Roll Right (-X)
    
    # Gripper Control
    'o': 'gripper_open',
    'p': 'gripper_close',
}

# --- Global Variables ---
g_settings = None
g_current_keys = set()
g_stop_all_motion = False
g_gripper_target = 0.0
g_lock = threading.Lock()

class PiperXIKTeleop:
    def __init__(self, can_interface='can0'):
        """Initialize the PiperX IK teleoperation."""
        self.can_interface = can_interface
        
        # Initialize Pinocchio IK solver
        print("Initializing Pinocchio IK solver...")
        try:
            self.ik = PiperIK(max_iterations=100, tolerance=1e-3, damping=1e-3)
            print("✅ Pinocchio IK solver initialized successfully")
        except Exception as e:
            print(f"❌ Failed to initialize Pinocchio IK: {e}")
            sys.exit(1)
        
        # Initialize PiperX robot
        print(f"Connecting to PiperX on {can_interface}...")
        try:
            self.piper = C_PiperInterface_V2(can_interface)
            self.piper.ConnectPort()
            
            # Wait for connection
            timeout = 5.0
            start_time = time.time()
            while not self.piper.get_connect_status():
                if time.time() - start_time > timeout:
                    print(f"❌ Connection timeout for {can_interface}")
                    sys.exit(1)
                time.sleep(0.01)
            
            # Enable the robot
            start_time = time.time()
            while not self.piper.EnablePiper():
                if time.time() - start_time > timeout:
                    print(f"❌ Enable timeout for {can_interface}")
                    sys.exit(1)
                time.sleep(0.01)
            
            print(f"✅ PiperX connected on {can_interface}")
            
        except Exception as e:
            print(f"❌ Failed to connect to PiperX: {e}")
            sys.exit(1)
        
        # Load calibration data
        self.load_calibration()
        
        # Initialize robot to neutral position
        self.init_robot()
        
        # Get initial end-effector position using forward kinematics
        self.update_current_pose()
        
        print("✅ PiperX IK teleoperation initialized")
    
    def load_calibration(self):
        """Load PiperX calibration data."""
        try:
            with open('piperx_calibration.json', 'r') as f:
                calib = json.load(f)
            self.neutral_positions = calib['neutral_positions']
            self.gripper_closed = calib['gripper_limits'][0]
            self.gripper_open = calib['gripper_limits'][1]
            print(f"✅ Loaded calibration: neutral={self.neutral_positions}")
        except Exception as e:
            print(f"Warning: Could not load calibration: {e}")
            print("Using default values")
            self.neutral_positions = [0, 0, 0, 0, 0, 0]
            self.gripper_closed = 0
            self.gripper_open = 30000
    
    def init_robot(self):
        """Initialize robot to neutral position."""
        try:
            print("🔧 Initializing robot to neutral position...")
            
            # Set gripper to closed position
            self.piper.GripperCtrl(int(self.gripper_closed), 1000, 0x01, 0)
            
            # Set joints to neutral position
            self.piper.MotionCtrl_2(0x01, 0x01, 100, 0x00)
            self.piper.JointCtrl(*[int(pos) for pos in self.neutral_positions])
            
            # Wait for movement to complete
            print("⏳ Waiting for robot to reach neutral position...")
            time.sleep(3.0)
            
            # Verify the robot reached the position by reading actual joint angles
            try:
                # Get current robot status
                status = self.piper.GetArmStatus()
                print(f"Robot status after initialization: {status}")
            except Exception as e:
                print(f"Warning: Could not get robot status: {e}")
            
            # Initialize our internal joint state with the actual neutral positions
            self.current_joints_rad = np.array(self.neutral_positions) * np.pi / 180000.0
            self.current_gripper_rad = self.gripper_closed * np.pi / 180000.0
            
            # Validate that neutral positions are within IK joint limits
            full_config = self.ik.get_full_configuration(self.current_joints_rad)
            within_limits = self.ik.check_joint_limits(full_config)
            
            print(f"Neutral position joint angles (rad): {self.current_joints_rad}")
            print(f"Neutral position within IK limits: {within_limits}")
            
            if not within_limits:
                print("⚠️ Warning: Neutral position is outside IK joint limits!")
                print(f"Current joints: {self.current_joints_rad}")
                print(f"IK limits lower: {self.ik.joint_limits_lower[:6]}")
                print(f"IK limits upper: {self.ik.joint_limits_upper[:6]}")
                
                # Try to clamp to valid range
                clamped_joints = np.clip(self.current_joints_rad, 
                                       self.ik.joint_limits_lower[:6], 
                                       self.ik.joint_limits_upper[:6])
                
                if not np.allclose(clamped_joints, self.current_joints_rad):
                    print("🔧 Adjusting to valid joint limits...")
                    self.current_joints_rad = clamped_joints
                    
                    # Send corrected position to robot
                    corrected_piper = self.current_joints_rad * 180000.0 / np.pi
                    self.piper.MotionCtrl_2(0x01, 0x01, 50, 0x00)  # Slower movement
                    self.piper.JointCtrl(*[int(angle) for angle in corrected_piper])
                    time.sleep(2.0)
                    
                    print(f"Corrected joints: {self.current_joints_rad}")
            
            print("✅ Robot initialized to neutral position")
            
        except Exception as e:
            print(f"❌ Failed to initialize robot: {e}")
            import traceback
            traceback.print_exc()
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
                print("Warning: Forward kinematics failed")
                return False
                
        except Exception as e:
            print(f"Warning: Error in forward kinematics: {e}")
            return False
    
    def apply_cartesian_velocity(self, linear_vel, angular_vel):
        """
        Apply Cartesian velocity using Pinocchio IK with intelligent scaling.
        
        The improved IK system now handles:
        - Intelligent joint limit avoidance through velocity scaling
        - Singularity-aware damping for numerical stability
        - Smooth motion without hard rejections at joint limits
        """
        try:
            # Create twist vector [vx, vy, vz, wx, wy, wz]
            twist = np.array([
                linear_vel[0], linear_vel[1], linear_vel[2],
                angular_vel[0], angular_vel[1], angular_vel[2]
            ])
            
            # Debug: Print velocity commands
            if DEBUG_VERBOSE and np.any(twist):
                print(f"Debug: Applying twist: linear={linear_vel}, angular={angular_vel}")
            
            # Get full configuration (with gripper joints)
            current_full_config = self.ik.get_full_configuration(self.current_joints_rad)
            if DEBUG_VERBOSE:
                print(f"Debug: Current joints (rad): {self.current_joints_rad}")
                print(f"Debug: Full config: {current_full_config}")
            
            # Compute joint velocities using Pinocchio IK with intelligent scaling
            # The velocity_ik now handles joint limit avoidance and singularity handling automatically
            joint_velocities = self.ik.velocity_ik(
                twist, 
                current_full_config,
                damping=0.01,                    # Base damping for numerical stability
                joint_limit_margin=0.1,         # Start scaling when within 0.1 rad of limits  
                singularity_threshold=1e-4,     # Threshold for singularity detection
                max_damping=0.1                  # Maximum damping near singularities
            )
            
            if joint_velocities is None:
                print("❌ Error: Velocity IK returned None - this is unexpected with the new scaling system")
                
                # With the new scaling system, failures should be rare. If they occur, 
                # it's likely due to a more fundamental issue.
                jacobian = self.ik.compute_jacobian(current_full_config)
                if jacobian is None:
                    print("  - Jacobian computation failed, attempting joint recovery...")
                    # Try to move to a safer joint configuration
                    self.recover_to_safe_position()
                    return False
                else:
                    print("  - Jacobian OK, but velocity IK still failed. Skipping this motion.")
                    # The new system should handle edge cases, so we don't retry with reduced velocity
                    return False
            
            # Only use arm joint velocities (first 6)
            arm_joint_velocities = joint_velocities[:6]
            if DEBUG_VERBOSE:
                print(f"Debug: Joint velocities: {arm_joint_velocities}")
            
            # Integrate to get new joint positions
            dt = CONTROL_PERIOD
            new_joints_rad = self.current_joints_rad + arm_joint_velocities * dt
            if DEBUG_VERBOSE:
                print(f"Debug: New joint positions: {new_joints_rad}")
            
            # Note: Joint limit handling is now done automatically by the velocity_ik method
            # which uses intelligent scaling to smoothly reduce velocities as joints approach limits.
            # No need for manual clamping as the IK solver handles this gracefully.
            
            # Optional safety check for debugging - warn if joints are approaching limits
            if DEBUG_VERBOSE:
                new_full_config = self.ik.get_full_configuration(new_joints_rad)
                if not self.ik.check_joint_limits(new_full_config):
                    distances_to_limits = []
                    for i in range(len(new_joints_rad)):
                        dist_lower = new_joints_rad[i] - self.ik.joint_limits_lower[i]
                        dist_upper = self.ik.joint_limits_upper[i] - new_joints_rad[i]
                        min_dist = min(dist_lower, dist_upper)
                        distances_to_limits.append(min_dist)
                    
                    closest_joint = np.argmin(distances_to_limits)
                    min_distance = distances_to_limits[closest_joint]
                    print(f"Debug: Joint {closest_joint} is {min_distance:.3f} rad from limit (velocity scaling active)")
            
            # Apply final safety clamp only for extreme cases to prevent hardware damage
            new_joints_rad = np.clip(new_joints_rad, 
                                   self.ik.joint_limits_lower[:6], 
                                   self.ik.joint_limits_upper[:6])
            
            # Convert to PiperX format (millidegrees) and send command
            new_joints_piper = new_joints_rad * 180000.0 / np.pi
            if DEBUG_VERBOSE:
                print(f"Debug: Sending PiperX commands: {[int(angle) for angle in new_joints_piper]}")
            
            self.piper.MotionCtrl_2(0x01, 0x01, 100, 0x00)
            self.piper.JointCtrl(*[int(angle) for angle in new_joints_piper])
            
            # Update internal state
            self.current_joints_rad = new_joints_rad
            success = self.update_current_pose()
            
            if DEBUG_VERBOSE and success and hasattr(self, 'current_position'):
                pos = self.current_position
                print(f"Debug: New position: x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f}")
            
            return True
            
        except Exception as e:
            print(f"❌ Error in velocity control: {e}")
            if DEBUG_VERBOSE:
                import traceback
                traceback.print_exc()
            return False
    
    def recover_to_safe_position(self):
        """Attempt to move robot to a safer joint configuration."""
        try:
            print("🔧 Attempting recovery to safer position...")
            
            # Try to move towards neutral position gradually
            neutral_rad = np.array(self.neutral_positions) * np.pi / 180000.0
            
            # Move 10% of the way towards neutral
            recovery_joints = self.current_joints_rad + 0.1 * (neutral_rad - self.current_joints_rad)
            
            # Ensure within limits
            recovery_joints = np.clip(recovery_joints, 
                                    self.ik.joint_limits_lower[:6], 
                                    self.ik.joint_limits_upper[:6])
            
            # Send recovery command
            recovery_piper = recovery_joints * 180000.0 / np.pi
            self.piper.MotionCtrl_2(0x01, 0x01, 50, 0x00)  # Slower speed
            self.piper.JointCtrl(*[int(angle) for angle in recovery_piper])
            
            # Update state
            self.current_joints_rad = recovery_joints
            self.update_current_pose()
            
            print("✅ Recovery completed")
            
        except Exception as e:
            print(f"❌ Recovery failed: {e}")
    
    def update_gripper(self, target_gripper_rad):
        """Update gripper position."""
        try:
            # Convert to PiperX format and send command
            gripper_piper = target_gripper_rad * 180000.0 / np.pi
            gripper_piper = np.clip(gripper_piper, self.gripper_closed, self.gripper_open)
            
            self.piper.GripperCtrl(int(gripper_piper), 1000, 0x01, 0)
            self.current_gripper_rad = gripper_piper * np.pi / 180000.0
            
        except Exception as e:
            print(f"Error updating gripper: {e}")
    
    def run(self):
        """Main control loop."""
        global g_stop_all_motion, g_gripper_target
        
        # Start keyboard listener thread
        kb_thread = threading.Thread(target=keyboard_listener)
        kb_thread.daemon = True
        kb_thread.start()
        
        print("🚀 PiperX IK teleoperation active!")
        print("Controls:")
        print("  WASD + QE: Linear motion (X, Y, Z)")
        print("  ZC: Yaw rotation")
        print("  RF: Pitch rotation")
        print("  TG: Roll rotation")
        print("  O: Open gripper")
        print("  P: Close gripper")
        print("  X: Stop motion")
        print("  Ctrl+C: Exit")
        
        # Show initial position
        if hasattr(self, 'current_position'):
            pos = self.current_position
            print(f"Initial position: x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f}")
        
        g_gripper_target = self.current_gripper_rad
        
        try:
            while True:
                loop_start = time.time()
                
                linear_vel = np.zeros(3)
                angular_vel = np.zeros(3)
                
                with g_lock:
                    if g_stop_all_motion:
                        # Send current position (stop motion)
                        current_joints_piper = self.current_joints_rad * 180000.0 / np.pi
                        self.piper.MotionCtrl_2(0x01, 0x01, 100, 0x00)
                        self.piper.JointCtrl(*[int(angle) for angle in current_joints_piper])
                        g_stop_all_motion = False
                        print("🛑 Robot stopped")
                    else:
                        # Process active keys
                        for key in g_current_keys:
                            if key in KEY_MAPPINGS:
                                mapping = KEY_MAPPINGS[key]
                                
                                if mapping == 'gripper_open':
                                    g_gripper_target = self.gripper_open * np.pi / 180000.0
                                elif mapping == 'gripper_close':
                                    g_gripper_target = self.gripper_closed * np.pi / 180000.0
                                else:
                                    axis_idx, direction = mapping
                                    
                                    if axis_idx < 3:  # Linear motion
                                        linear_vel[axis_idx] = direction * LINEAR_VELOCITY_STEP
                                    else:  # Angular motion
                                        angular_vel[axis_idx - 3] = direction * ANGULAR_VELOCITY_STEP
                
                # Apply velocities if any
                if np.any(linear_vel) or np.any(angular_vel):
                    success = self.apply_cartesian_velocity(linear_vel, angular_vel)
                    if not success:
                        print("❌ Motion command failed")
                
                # Update gripper if target changed
                if abs(g_gripper_target - self.current_gripper_rad) > 0.01:
                    self.update_gripper(g_gripper_target)
                
                # Maintain control frequency
                elapsed = time.time() - loop_start
                sleep_time = max(0, CONTROL_PERIOD - elapsed)
                time.sleep(sleep_time)
                
        except KeyboardInterrupt:
            print("\n🛑 Teleoperation stopped by user")
        finally:
            self.shutdown()
    
    def shutdown(self):
        """Shutdown the robot safely."""
        print("Shutting down...")
        try:
            # Return to neutral position
            self.piper.MotionCtrl_2(0x01, 0x01, 100, 0x00)
            self.piper.JointCtrl(*[int(pos) for pos in self.neutral_positions])
            
            # Close gripper
            self.piper.GripperCtrl(int(self.gripper_closed), 1000, 0x01, 0)
            
            time.sleep(1.0)
            print("✅ Robot returned to neutral position")
            
        except Exception as e:
            print(f"Error during shutdown: {e}")
        finally:
            # Restore terminal settings
            if g_settings is not None:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, g_settings)
            print("🔚 PiperX IK teleoperation terminated")

def get_key():
    """Reads a single character from stdin without blocking."""
    global g_settings
    if g_settings is None:
        g_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
    
    if select.select([sys.stdin], [], [], 0.001)[0]:
        return sys.stdin.read(1)
    return None

def keyboard_listener():
    """Thread function to continuously read keyboard input."""
    global g_current_keys, g_stop_all_motion
    
    active_keys = set()
    key_timeout = {}
    KEY_HOLD_TIME = 0.1  # Keys expire after 100ms of no detection
    
    while True:
        key = get_key()
        current_time = time.time()
        
        with g_lock:
            if key is not None:
                if key == 'x':
                    g_stop_all_motion = True
                    g_current_keys.clear()
                    active_keys.clear()
                    key_timeout.clear()
                elif key == '\x03':  # Ctrl+C
                    break
                elif key in KEY_MAPPINGS:
                    # Add/refresh key with timeout
                    active_keys.add(key)
                    key_timeout[key] = current_time + KEY_HOLD_TIME
                    print(f"Key: {key}")
            
            # Remove expired keys
            expired_keys = [k for k, timeout in key_timeout.items() if current_time > timeout]
            for k in expired_keys:
                active_keys.discard(k)
                del key_timeout[k]
            
            # Update global key set
            g_current_keys = active_keys.copy()
        
        time.sleep(0.005)

def main():
    """Main function."""
    parser = argparse.ArgumentParser(description='PiperX IK Teleoperation (Non-ROS)')
    parser.add_argument('--can', default='can0', help='CAN interface for PiperX (default: can0)')
    parser.add_argument('--debug', action='store_true', help='Enable verbose debugging output')
    args = parser.parse_args()
    
    # Set global debug flag
    global DEBUG_VERBOSE
    DEBUG_VERBOSE = args.debug
    
    try:
        print("Starting keyboard teleoperation...")
        if DEBUG_VERBOSE:
            print("🐛 Debug mode enabled")
        
        # Start the IK teleop
        teleop = PiperXIKTeleop(can_interface=args.can)
        teleop.run()
            
    except KeyboardInterrupt:
        print("\nTeleoperation stopped by user.")
    except Exception as e:
        print(f"Error in teleoperation: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Ensure terminal is restored
        if g_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, g_settings)

if __name__ == '__main__':
    main()
