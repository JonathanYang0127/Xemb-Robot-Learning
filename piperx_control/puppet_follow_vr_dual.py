#!/usr/bin/env python3
"""
PiperX Dual-Arm VR Puppet Robot Control Script
Receives VR controller data for BOTH left and right controllers and performs Cartesian control using inverse kinematics
Left Controller → Robot 0 (Left Arm), Right Controller → Robot 1 (Right Arm)
"""

import socket
import pickle
import time
import sys
import os
import numpy as np
import argparse
import json
import threading

# Add the parent directory to the path
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from xemb_scripts.constants import *
from xemb_scripts.cartesian_control.transformations import (
    rotation_matrix_to_quaternion, 
    quaternion_to_euler_wxyz as quaternion_to_euler,
    normalize_quaternion,
    quaternion_multiply
)
from packing_utils_vr_dual import unpack_vr_data_from_udp, unpack_dual_vr_data_from_udp
from inverse_kinematics.piper_pinocchio_ik import PiperIK
from enhanced_base_controller import EnhancedBaseController, MockEnhancedBaseController

try:
    from piper_sdk import C_PiperInterface_V2
except ImportError:
    print("Error: piper_sdk not found. Please install with: pip install piper-sdk")
    sys.exit(1)

# Try to import hardware controllers, use mocks if not available
try:
    from stepper_control.stepper_controller import StepperController
    STEPPER_AVAILABLE = True
except ImportError:
    print("Warning: stepper_control not available. Stepper will be mocked.")
    STEPPER_AVAILABLE = False

try:
    from base_control.base_controller import MDCRobot
    BASE_AVAILABLE = True
except ImportError:
    print("Warning: base_control not available. Base will be mocked.")
    BASE_AVAILABLE = False

# Mock classes for hardware we don't have
class MockStepperController:
    def __init__(self, port):
        self.port = port
        print(f"Mock StepperController initialized on {port}")
    
    def enable(self):
        print("Mock stepper enabled")
    
    def move_steps(self, steps):
        print(f"Mock stepper move: {steps} steps")

class MockMDCRobot:
    def __init__(self, port, default_speed):
        self.port = port
        self.default_speed = default_speed
        print(f"Mock MDCRobot initialized on {port}")
    
    def connect(self):
        print("Mock base robot connected")
    
    def move_forward(self, distance):
        print(f"Mock base move forward: {distance}")
    
    def move_backward(self, distance):
        print(f"Mock base move backward: {distance}")
    
    def turn_left(self, angle):
        print(f"Mock base turn left: {angle}")
    
    def turn_right(self, angle):
        print(f"Mock base turn right: {angle}")

# Constants
UDP_PORT = 5005
BASE_SPEED = 1
CONTROL_PERIOD = 0.02  # 50Hz control loop
LINEAR_VELOCITY_SCALE = 0.5   # Scale factor for linear movement
ANGULAR_VELOCITY_SCALE = 0.5  # Scale factor for angular movement
TRIGGER_THRESHOLD = 0.1       # Minimum trigger value to enable movement
MAX_POSITION_DELTA = 0.3      # Maximum position change per control cycle
MAX_ORIENTATION_DELTA = 0.3   # Maximum orientation change per control cycle

# Coordinate frame transformation settings
# VR coordinate system: Y-up, Z-forward, X-right (typical for Meta Quest)
# Robot coordinate system: Z-up, X-forward, Y-left (typical for robotics)
# FIXED: User reported VR left/right was mapping to robot down/up, and forward/backward was inverted

# Fixed transformation matrix from VR to Robot coordinates
VR_TO_ROBOT_POSITION_TRANSFORM = np.array([
    [0, 0, -1],  # VR Z (forward) → Robot -X (backward) - FIXED INVERSION
    [-1, 0, 0],  # VR X (right) → Robot -Y (right, since Y+ is left)
    [0, 1, 0]    # VR Y (up) → Robot Z (up)
])

VR_TO_ROBOT_ROTATION_TRANSFORM = np.array([
    [0, 0, -1],  # VR Z-axis → Robot -X-axis - FIXED INVERSION
    [-1, 0, 0],  # VR X-axis → Robot -Y-axis  
    [0, 1, 0]    # VR Y-axis → Robot Z-axis
])

# Global debug flag
DEBUG_MODE = False

# Global coordinate transformation flag
ENABLE_COORDINATE_TRANSFORM = True

def debug_print(robot_id, message):
    """Print debug message only if DEBUG_MODE is enabled."""
    if DEBUG_MODE:
        print(f"[Robot {robot_id}] 🔧 DEBUG: {message}")

def transform_vr_position_to_robot(vr_position):
    """Transform VR position to robot coordinate frame."""
    if not ENABLE_COORDINATE_TRANSFORM:
        return vr_position
        
    try:
        # Apply coordinate transformation matrix
        robot_position = VR_TO_ROBOT_POSITION_TRANSFORM @ vr_position
        
        # Debug: Show transformation details (only in verbose debug mode)
        if DEBUG_MODE:
            print(f"🔄 TRANSFORM DEBUG:")
            print(f"   VR input: [{vr_position[0]:.3f}, {vr_position[1]:.3f}, {vr_position[2]:.3f}]")
            print(f"   Robot output: [{robot_position[0]:.3f}, {robot_position[1]:.3f}, {robot_position[2]:.3f}]")
            print(f"   Mapping: VR_X→Robot_Y, VR_Y→Robot_Z, VR_Z→Robot_X")
        
        return robot_position
    except Exception as e:
        print(f"Error transforming VR position: {e}")
        return vr_position  # Fallback to original

def transform_vr_quaternion_to_robot(vr_quaternion):
    """Transform VR quaternion to robot coordinate frame."""
    if not ENABLE_COORDINATE_TRANSFORM:
        return vr_quaternion
        
    try:
        # Convert quaternion to rotation matrix
        w, x, y, z = vr_quaternion
        
        # Create rotation matrix from quaternion
        R_vr = np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
            [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
            [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]
        ])
        
        # Apply coordinate frame transformation
        R_robot = VR_TO_ROBOT_ROTATION_TRANSFORM @ R_vr @ VR_TO_ROBOT_ROTATION_TRANSFORM.T
        
        # Convert back to quaternion
        robot_quaternion = rotation_matrix_to_quaternion(R_robot)
        return robot_quaternion
        
    except Exception as e:
        print(f"Error transforming VR quaternion: {e}")
        return vr_quaternion  # Fallback to original

def process_auxiliary_controls(button_states, stepper, enhanced_base, joystick, last_button_states=None):
    """Process auxiliary controls for stepper and enhanced base via button presses and joystick"""
    if last_button_states is None:
        last_button_states = {}
    
    # Stepper control using menu button (rising edge)
    if button_states.get('menu', False) and not last_button_states.get('menu', False):
        if stepper and hasattr(stepper, 'move_steps'):
            stepper.move_steps(50)
        elif stepper:
            print("Mock: Stepper up")
    
    # Enhanced base control using joystick continuous input
    if enhanced_base and joystick is not None and len(joystick) >= 2:
        joystick_x, joystick_y = joystick[0], joystick[1]
        
        # Only update if joystick has meaningful input (outside dead zone)
        if abs(joystick_x) > 0.05 or abs(joystick_y) > 0.05:
            enhanced_base.update_joystick(joystick_x, joystick_y)
        else:
            # Stop movement if joystick is centered
            enhanced_base.update_joystick(0.0, 0.0)
    elif enhanced_base:
        # Ensure base stops if no joystick data
        enhanced_base.update_joystick(0.0, 0.0)
    
    return button_states.copy()

def try_connect_piperx(can_interface, robot_id):
    """Try to connect to a PiperX robot on the given CAN interface"""
    try:
        print(f"[Robot {robot_id}] Attempting to connect to PiperX on {can_interface}...")
        piper = C_PiperInterface_V2(can_interface)
        piper.ConnectPort()
        
        # Wait for connection with timeout
        timeout = 5.0  # 5 second timeout
        start_time = time.time()
        while not piper.get_connect_status():
            if time.time() - start_time > timeout:
                print(f"[Robot {robot_id}] Connection timeout for {can_interface}")
                return None
            time.sleep(0.01)
        
        # Try to enable the robot
        print(f"[Robot {robot_id}] Enabling PiperX on {can_interface}...")
        start_time = time.time()
        while not piper.EnablePiper():
            if time.time() - start_time > timeout:
                print(f"[Robot {robot_id}] Enable timeout for {can_interface}")
                return None
            time.sleep(0.01)
        
        print(f"[Robot {robot_id}] ✅ PiperX on {can_interface} ready for VR teleoperation")
        return piper
        
    except Exception as e:
        print(f"[Robot {robot_id}] ❌ Failed to connect to PiperX on {can_interface}: {e}")
        return None

class PiperXVRController:
    def __init__(self, can_interface, robot_id, use_ik=True):
        """Initialize PiperX VR controller."""
        debug_print(robot_id, "Initializing PiperXVRController...")
        debug_print(robot_id, f"CAN interface: {can_interface}")
        debug_print(robot_id, f"IK enabled: {use_ik}")
        
        self.can_interface = can_interface
        self.robot_id = robot_id
        self.use_ik = use_ik
        
        # Connect to PiperX robot
        debug_print(robot_id, "Attempting robot connection...")
        self.piper = try_connect_piperx(can_interface, robot_id)
        if self.piper is None:
            raise Exception(f"Failed to connect to PiperX on {can_interface}")
        debug_print(robot_id, "Robot connection successful")
        
        # Load calibration data
        debug_print(robot_id, "Loading calibration data...")
        self.load_calibration()
        
        # Initialize IK solver if requested
        self.ik = None
        if use_ik:
            try:
                debug_print(robot_id, "Initializing Pinocchio IK solver...")
                self.ik = PiperIK(max_iterations=100, tolerance=1e-3, damping=1e-3)
                print(f"[Robot {robot_id}] ✅ Pinocchio IK solver initialized successfully")
                debug_print(robot_id, "IK solver parameters - max_iter: 100, tol: 1e-3, damping: 1e-3")
            except Exception as e:
                print(f"[Robot {robot_id}] ❌ Failed to initialize IK: {e}")
                debug_print(robot_id, f"Exception details: {type(e).__name__}: {str(e)}")
                print(f"[Robot {robot_id}] Falling back to joint position mode")
                self.use_ik = False
        else:
            debug_print(robot_id, "IK disabled by user request")
        
        # Initialize robot state
        debug_print(robot_id, "Initializing robot to neutral position...")
        self.init_robot()
        
        # Current state tracking
        self.current_joints_rad = np.array(self.neutral_positions) * np.pi / 180000.0
        self.current_gripper_rad = self.gripper_closed * np.pi / 180000.0
        debug_print(robot_id, f"Initial joint positions (rad): {self.current_joints_rad}")
        debug_print(robot_id, f"Initial gripper position (rad): {self.current_gripper_rad}")
        
        # VR control state
        self.reference_position = np.zeros(3)
        self.reference_orientation = np.zeros(3)  # Euler angles
        self.reference_set = False
        self.last_movement_state = False
        self.last_reset_state = False # For reset button edge detection
        debug_print(robot_id, "VR control state initialized")
        
        # Get initial end-effector pose if using IK
        if self.use_ik:
            debug_print(robot_id, "Computing initial end-effector pose...")
            self.update_current_pose()
        
        debug_print(robot_id, "PiperXVRController initialization complete!")
    
    def load_calibration(self):
        """Load PiperX calibration data."""
        try:
            with open('piperx_calibration.json', 'r') as f:
                calib = json.load(f)
            self.neutral_positions = calib['neutral_positions']
            self.gripper_closed = calib['gripper_limits'][0]
            self.gripper_open = calib['gripper_limits'][1]
            print(f"[Robot {self.robot_id}] ✅ Loaded calibration successfully")
        except Exception as e:
            print(f"[Robot {self.robot_id}] ⚠️  Warning: Could not load calibration: {e}")
            self.neutral_positions = [0, 0, 0, 0, 0, 0]
            self.gripper_closed = 0
            self.gripper_open = 30000
    
    def init_robot(self):
        """Initialize robot to neutral position."""
        try:
            # Set gripper to closed position
            self.piper.GripperCtrl(int(self.gripper_closed), 1000, 0x01, 0)
            
            # Set joints to neutral position
            neutral_ints = [int(pos) for pos in self.neutral_positions]
            self.piper.MotionCtrl_2(0x01, 0x01, 100, 0x00)
            self.piper.JointCtrl(*neutral_ints)
            
            # Wait for movement to complete
            time.sleep(2.0)
            
            print(f"[Robot {self.robot_id}] ✅ Robot initialized to neutral position successfully")
            
        except Exception as e:
            print(f"[Robot {self.robot_id}] ❌ Failed to initialize robot: {e}")
            raise
    
    def update_current_pose(self):
        """Update current end-effector pose using forward kinematics."""
        if not self.use_ik:
            return False
            
        try:
            full_config = self.ik.get_full_configuration(self.current_joints_rad)
            fk_result = self.ik.forward_kinematics(full_config)
            
            if fk_result:
                self.current_position = np.array(fk_result['position'])
                self.current_orientation = np.array(fk_result['orientation'])
                return True
            else:
                print(f"[Robot {self.robot_id}] ⚠️  Warning: Forward kinematics returned None")
                return False
                
        except Exception as e:
            print(f"[Robot {self.robot_id}] ⚠️  Warning: Error in forward kinematics: {e}")
            return False
    
    def apply_cartesian_velocity(self, linear_vel, angular_vel):
        """Apply Cartesian velocity using Pinocchio IK."""
        if not self.use_ik:
            print(f"[Robot {self.robot_id}] ⚠️  Warning: IK not available, cannot apply Cartesian velocity")
            return False
            
        try:
            # Create twist vector [vx, vy, vz, wx, wy, wz]
            twist = np.array([
                linear_vel[0], linear_vel[1], linear_vel[2],
                angular_vel[0], angular_vel[1], angular_vel[2]
            ])
            
            # Get full configuration (with gripper joints)
            current_full_config = self.ik.get_full_configuration(self.current_joints_rad)
            
            # Compute joint velocities using the improved IK solver
            # This now includes both joint limit and singularity avoidance
            joint_velocities = self.ik.velocity_ik(
                twist, 
                current_full_config,
                joint_limit_margin=np.deg2rad(10),
                singularity_threshold=0.005, # Start damping when manipulability is low
                max_damping=0.2 # Increase damping up to this value
            )
            
            if joint_velocities is None:
                print(f"[Robot {self.robot_id}] ⚠️  Warning: Velocity IK returned None")
                return False
            
            # Only use arm joint velocities (first 6)
            arm_joint_velocities = joint_velocities[:6]
            
            # Integrate to get new joint positions
            dt = CONTROL_PERIOD
            new_joints_rad = self.current_joints_rad + arm_joint_velocities * dt
            
            # The new IK solver handles joint limits, so we can remove the old check.
            # As a final safety net, we can still clamp the values.
            new_joints_rad = np.clip(
                new_joints_rad, 
                self.ik.joint_limits_lower[:6], 
                self.ik.joint_limits_upper[:6]
            )
            
            # Convert to PiperX format (millidegrees) and send command
            new_joints_piper = new_joints_rad * 180000.0 / np.pi
            new_joints_int = [int(angle) for angle in new_joints_piper]
            
            self.piper.MotionCtrl_2(0x01, 0x01, 100, 0x00)
            self.piper.JointCtrl(*new_joints_int)
            
            # Update internal state
            self.current_joints_rad = new_joints_rad
            self.update_current_pose()
            
            return True
            
        except Exception as e:
            print(f"[Robot {self.robot_id}] ❌ Error in Cartesian velocity control: {e}")
            return False
    
    def set_joint_positions(self, joint_positions_rad):
        """Directly set joint positions (fallback when IK not available)."""
        try:
            # Convert to PiperX format (millidegrees)
            joints_piper = joint_positions_rad * 180000.0 / np.pi
            
            self.piper.MotionCtrl_2(0x01, 0x01, 100, 0x00)
            self.piper.JointCtrl(*[int(angle) for angle in joints_piper])
            
            # Update internal state
            self.current_joints_rad = joint_positions_rad.copy()
            
            return True
            
        except Exception as e:
            print(f"[Robot {self.robot_id}] Error setting joint positions: {e}")
            return False
    
    def update_gripper(self, gripper_command):
        """Update gripper based on button commands."""
        try:
            if gripper_command == 'open':
                gripper_piper = self.gripper_open
            elif gripper_command == 'close':
                gripper_piper = self.gripper_closed
            else:
                return  # No command
            
            self.piper.GripperCtrl(int(gripper_piper), 1000, 0x01, 0)
            self.current_gripper_rad = gripper_piper * np.pi / 180000.0
            
        except Exception as e:
            print(f"[Robot {self.robot_id}] Error updating gripper: {e}")
    
    def _debug_log_vr_input(self, position, orientation, button_states, trigger_value):
        """Log VR input data for debugging purposes."""
        if DEBUG_MODE:
            print(f"[Robot {self.robot_id}] 🔧 DEBUG: Processing VR command")
            print(f"[Robot {self.robot_id}] 🔧 DEBUG: Raw VR position: [{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}]")
            print(f"[Robot {self.robot_id}] 🔧 DEBUG: Raw VR orientation (quat): [{orientation[0]:.3f}, {orientation[1]:.3f}, {orientation[2]:.3f}, {orientation[3]:.3f}]")
    
    def _debug_log_transformed_data(self, robot_position, robot_orientation, button_states, trigger_value):
        """Log transformed robot data for debugging purposes."""
        if DEBUG_MODE:
            print(f"[Robot {self.robot_id}] 🔧 DEBUG: Transformed robot position: [{robot_position[0]:.3f}, {robot_position[1]:.3f}, {robot_position[2]:.3f}]")
            print(f"[Robot {self.robot_id}] 🔧 DEBUG: Transformed robot orientation: [{robot_orientation[0]:.3f}, {robot_orientation[1]:.3f}, {robot_orientation[2]:.3f}, {robot_orientation[3]:.3f}]")
            print(f"[Robot {self.robot_id}] 🔧 DEBUG: Trigger value: {trigger_value:.3f}")
            print(f"[Robot {self.robot_id}] 🔧 DEBUG: Button states: {button_states}")
        
        # Always show button states when any button is pressed (helpful for debugging)
        pressed_buttons = [btn for btn, state in button_states.items() if state]
        if pressed_buttons:
            print(f"[Robot {self.robot_id}] 🎮 BUTTONS PRESSED: {pressed_buttons}")

    def _handle_reset_button(self, button_states):
        """Handle reset button press with edge detection - supports both X and A buttons for dual-arm."""
        # Set a flag to track if a reset is in progress
        if not hasattr(self, 'reset_in_progress'):
            self.reset_in_progress = False

        # Check for reset button - PRIORITY COMMAND
        # Both robots can use either X or A button for reset (either controller can reset either arm)
        # Use edge detection: trigger only on the initial press
        x_pressed = button_states.get('x_button', False)
        a_pressed = button_states.get('a_button', False)
        is_reset_pressed = x_pressed or a_pressed
        button_name = "X" if x_pressed else ("A" if a_pressed else "none")
            
        if is_reset_pressed and not self.last_reset_state:
            self.reset_in_progress = True
            print(f"[Robot {self.robot_id}] 🔄 RESET BUTTON ({button_name}) PRESSED - Moving to neutral position!")
        
        # Store the current state for the next loop's edge detection
        self.last_reset_state = is_reset_pressed

        # If a reset is triggered, handle it and skip other commands
        if self.reset_in_progress:
            try:
                # Reset robot to neutral position
                self.init_robot()
                
                # Reset VR control state
                self.reference_position = np.zeros(3)
                self.reference_orientation = np.zeros(3)
                self.reference_set = False
                self.last_movement_state = False
                
                # Update internal state to neutral
                self.current_joints_rad = np.array(self.neutral_positions) * np.pi / 180000.0
                self.current_gripper_rad = self.gripper_closed * np.pi / 180000.0
                
                # Update current pose if using IK
                if self.use_ik:
                    self.update_current_pose()
                
                print(f"[Robot {self.robot_id}] ✅ Robot successfully reset to neutral position")
                
            except Exception as e:
                print(f"[Robot {self.robot_id}] ❌ Error during reset: {e}")
            finally:
                # Reset the flag after the action is complete
                self.reset_in_progress = False
                
            return True  # Reset performed, skip further processing
        
        return False  # No reset, continue normal processing

    def _calculate_movement_velocities(self, robot_position, robot_orientation, button_states):
        """Calculate movement velocities based on VR controller input."""
        linear_vel = np.zeros(3)
        angular_vel = np.zeros(3)
        
        # SWAPPED CONTROLS: squeeze enables movement, trigger controls gripper
        # Check if squeeze is pressed (enable movement)
        movement_enabled = button_states.get('squeeze', False)
        
        # Set reference position on squeeze press (use transformed coordinates)
        if movement_enabled and not self.last_movement_state:
            self.reference_position = robot_position.copy()
            self.reference_orientation = quaternion_to_euler(robot_orientation)
            self.reference_set = True
            print(f"[Robot {self.robot_id}] 🎯 MOVEMENT ENABLED - Reference pose set")
            if DEBUG_MODE:
                print(f"[Robot {self.robot_id}] 🔧 DEBUG: Reference position: {self.reference_position}")
                print(f"[Robot {self.robot_id}] 🔧 DEBUG: Reference orientation (euler): {self.reference_orientation}")
            print(f"[Robot {self.robot_id}] 💡 Move VR controller to control robot")
        
        # Calculate velocities if squeeze is pressed and reference is set (use transformed coordinates)
        if movement_enabled and self.reference_set:
            # Calculate position and orientation deltas using robot coordinates
            pos_delta = robot_position - self.reference_position
            ori_euler = quaternion_to_euler(robot_orientation)
            ori_delta = ori_euler - self.reference_orientation
            
            # Apply dead zone (small movements ignored)
            dead_zone = 0.01
            pos_delta = np.where(np.abs(pos_delta) < dead_zone, 0, pos_delta)
            ori_delta = np.where(np.abs(ori_delta) < dead_zone, 0, ori_delta)
            
            # Limit maximum deltas
            pos_delta = np.clip(pos_delta, -MAX_POSITION_DELTA, MAX_POSITION_DELTA)
            ori_delta = np.clip(ori_delta, -MAX_ORIENTATION_DELTA, MAX_ORIENTATION_DELTA)
            
            # Convert to velocities (no scaling since squeeze is binary)
            linear_vel = pos_delta * LINEAR_VELOCITY_SCALE
            angular_vel = ori_delta * ANGULAR_VELOCITY_SCALE
        
        # Reset reference if squeeze released
        elif not movement_enabled and self.last_movement_state:
            self.reference_set = False
            print(f"[Robot {self.robot_id}] 🛑 MOVEMENT DISABLED - Robot stopped")
        
        return linear_vel, angular_vel, movement_enabled

    def _handle_gripper_control(self, button_states):
        """Handle gripper control based on trigger button state changes."""
        gripper_command = None
        
        # Handle gripper commands (trigger for natural grip control)
        # Only send gripper commands when trigger state CHANGES to prevent erratic behavior
        current_trigger = button_states.get('trigger', False)
        if not hasattr(self, 'last_gripper_trigger_state'):
            self.last_gripper_trigger_state = False
            
        if current_trigger != self.last_gripper_trigger_state:
            if current_trigger:
                gripper_command = 'close'  # Trigger pressed = close gripper
                print(f"[Robot {self.robot_id}] 🤏 GRIPPER CLOSING")
            else:
                gripper_command = 'open'   # Trigger released = open gripper
                print(f"[Robot {self.robot_id}] 🤏 GRIPPER OPENING")
            self.last_gripper_trigger_state = current_trigger
        # No gripper command if trigger state unchanged
        
        return gripper_command

    def _execute_robot_commands(self, linear_vel, angular_vel, gripper_command):
        """Execute motion and gripper commands on the robot."""
        # Apply motion commands
        if np.any(linear_vel) or np.any(angular_vel):
            if self.use_ik:
                success = self.apply_cartesian_velocity(linear_vel, angular_vel)
                if not success:
                    print(f"[Robot {self.robot_id}] ❌ Cartesian motion failed")
            else:
                print(f"[Robot {self.robot_id}] ⚠️  Warning: No IK available for Cartesian control")
        
        # Apply gripper commands
        if gripper_command:
            self.update_gripper(gripper_command)
            print(f"[Robot {self.robot_id}] 🤏 Gripper: {gripper_command}")

    def process_vr_command(self, position, orientation, button_states, trigger_value):
        """Process VR controller command and update robot."""
        # Debug logging for input data
        self._debug_log_vr_input(position, orientation, button_states, trigger_value)
        
        # Transform VR coordinates to robot coordinates
        robot_position = transform_vr_position_to_robot(position)
        robot_orientation = transform_vr_quaternion_to_robot(orientation)
        
        # Debug logging for transformed data
        self._debug_log_transformed_data(robot_position, robot_orientation, button_states, trigger_value)
        
        # Handle reset button (highest priority)
        if self._handle_reset_button(button_states):
            return False  # Reset performed, stop further processing
        
        # Calculate movement velocities
        linear_vel, angular_vel, movement_enabled = self._calculate_movement_velocities(
            robot_position, robot_orientation, button_states
        )
        
        # Handle gripper control
        gripper_command = self._handle_gripper_control(button_states)
        
        # Execute robot commands
        self._execute_robot_commands(linear_vel, angular_vel, gripper_command)
        
        # Update movement state
        self.last_movement_state = movement_enabled
        
        return movement_enabled
    
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

def parse_arguments():
    """Parse command line arguments for dual-arm VR puppet control."""
    parser = argparse.ArgumentParser(description='PiperX Dual-Arm VR Puppet Robot Control')
    parser.add_argument('--single-arm', action='store_true', help='Use only one arm (left arm only)')
    parser.add_argument('--can-left', default='can_left', help='CAN interface for left arm (default: can0)')
    parser.add_argument('--can-right', default='can_right', help='CAN interface for right arm (default: can1)')
    parser.add_argument('--no-stepper', action='store_true', help='Disable stepper motor control')
    parser.add_argument('--no-base', action='store_true', help='Disable base robot control')
    parser.add_argument('--no-ik', action='store_true', help='Disable inverse kinematics (joint mode only)')
    parser.add_argument('--debug', action='store_true', help='Enable debug output')
    parser.add_argument('--verbose-debug', action='store_true', help='Enable extremely verbose debug output (includes all internal operations)')
    parser.add_argument('--no-coordinate-transform', action='store_true', help='Disable VR to robot coordinate transformation (use raw VR coordinates)')
    parser.add_argument('--coordinate-info', action='store_true', help='Show coordinate transformation information and exit')
    parser.add_argument('--skip-init', action='store_true', help='Skip VR initialization handshake (for debugging)')
    return parser.parse_args()

def show_coordinate_info():
    """Display coordinate transformation information and exit."""
    print("🗂️  VR to Robot Coordinate Transformation Information")
    print("=" * 60)
    print("VR Coordinate System (Meta Quest typical):")
    print("  - X: Right (+) / Left (-)")
    print("  - Y: Up (+) / Down (-)")  
    print("  - Z: Forward (+) / Backward (-)")
    print()
    print("Robot Coordinate System (Robotics typical):")
    print("  - X: Forward (+) / Backward (-)")
    print("  - Y: Left (+) / Right (-)")
    print("  - Z: Up (+) / Down (-)")
    print()
    print("CORRECTED Transformation Matrix (VR → Robot):")
    print("Position Transform:")
    print(VR_TO_ROBOT_POSITION_TRANSFORM)
    print("Rotation Transform:")
    print(VR_TO_ROBOT_ROTATION_TRANSFORM)
    print()
    print("Fixed Mapping:")
    print("  VR X (right/left) → Robot -Y (right/left) ✓")
    print("  VR Y (up/down) → Robot Z (up/down) ✓")  
    print("  VR Z (forward/backward) → Robot -X (backward/forward) ✓ FIXED")
    print()
    print("Forward/backward inversion has been corrected!")
    print("Use --no-coordinate-transform to disable transformation")

def configure_global_settings(args):
    """Configure global debug and coordinate transform settings."""
    global DEBUG_MODE, ENABLE_COORDINATE_TRANSFORM
    DEBUG_MODE = args.verbose_debug
    ENABLE_COORDINATE_TRANSFORM = not args.no_coordinate_transform
    
    print("🤖 Initializing PiperX DUAL-ARM VR Puppet Robot...")
    print(f"Dual arm mode: {not args.single_arm}")
    print(f"Single arm mode: {args.single_arm}")
    print(f"IK enabled: {not args.no_ik}")
    print(f"Stepper control: {not args.no_stepper and STEPPER_AVAILABLE}")
    print(f"Base control: {not args.no_base and BASE_AVAILABLE}")
    print(f"Debug mode: {args.debug}")
    print(f"Verbose debug mode: {args.verbose_debug}")
    print(f"Coordinate transform enabled: {ENABLE_COORDINATE_TRANSFORM}")
    
    if args.verbose_debug:
        print("🔧 WARNING: Verbose debug mode enabled - expect extensive logging!")
        print("🔧 This will significantly impact performance and readability.")

def initialize_robots(args):
    """Initialize PiperX robot controllers."""
    puppet_left = None
    puppet_right = None
    
    try:
        # Initialize left arm (always initialize for dual-arm mode)
        print(f"🤖 Initializing LEFT robot on {args.can_left} with robot_id=0...")
        puppet_left = PiperXVRController(args.can_left, robot_id=0, use_ik=not args.no_ik)
        print("✅ LEFT robot initialized successfully!")
        
        # Initialize right arm if not single-arm mode
        if not args.single_arm:
            try:
                print(f"🤖 Initializing RIGHT robot on {args.can_right} with robot_id=1...")
                puppet_right = PiperXVRController(args.can_right, robot_id=1, use_ik=not args.no_ik)
                print("✅ RIGHT robot initialized successfully!")
                print("✅ DUAL-ARM setup: Both left and right robots initialized!")
            except Exception as e:
                print(f"Warning: Could not initialize right arm: {e}")
                print("Continuing with single arm mode...")
                args.single_arm = True
        else:
            print("✅ SINGLE-ARM setup: Only left robot initialized")
    
    except Exception as e:
        print(f"❌ Failed to initialize PiperX robots: {e}")
        sys.exit(1)
    
    return puppet_left, puppet_right

def initialize_auxiliary_controllers(args):
    """Initialize stepper and enhanced base controllers."""
    stepper = None
    if not args.no_stepper:
        if STEPPER_AVAILABLE:
            try:
                stepper = StepperController(port="/dev/tty_stepper")
                stepper.enable()
                print("✅ Real stepper controller initialized")
            except Exception as e:
                print(f"Failed to initialize stepper: {e}, using mock")
                stepper = MockStepperController(port="/dev/tty_stepper")
        else:
            stepper = MockStepperController(port="/dev/tty_stepper")
    
    enhanced_base = None
    if not args.no_base:
        if BASE_AVAILABLE:
            try:
                base_robot = MDCRobot(port='/dev/tty_base', default_speed=BASE_SPEED)
                base_robot.connect()
                enhanced_base = EnhancedBaseController(
                    base_controller=base_robot,
                    max_linear_speed=BASE_SPEED,  # Use BASE_SPEED for max linear speed
                    max_angular_speed=1.0  # 1.0 rad/s max angular speed
                )
                print("✅ Real enhanced base controller initialized with joystick support")
            except Exception as e:
                print(f"Failed to initialize real base: {e}, using mock enhanced base")
                enhanced_base = MockEnhancedBaseController(
                    max_linear_speed=BASE_SPEED,
                    max_angular_speed=1.0
                )
        else:
            enhanced_base = MockEnhancedBaseController(
                max_linear_speed=BASE_SPEED,
                max_angular_speed=1.0
            )
    
    return stepper, enhanced_base

def setup_vr_communication():
    """Setup UDP socket for VR communication."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", UDP_PORT))
    sock.settimeout(0.1)
    return sock

def wait_for_vr_initialization(sock):
    """Wait for VR controller initialization handshake - supports both single and dual-arm modes."""
    print("🎮 [INIT] Waiting for VR controller initialization...")
    print("🔧 DEBUG: UDP socket bound to port", UDP_PORT)
    session_id = None
    active_robot_count = 1
    single_arm_mode = True
    
    while True:
        try:
            print("🔧 DEBUG: Waiting for init packet...")
            data, addr = sock.recvfrom(4096)
            print(f"🔧 DEBUG: Received packet from {addr}, size: {len(data)} bytes")
            
            packet = pickle.loads(data)
            print(f"🔧 DEBUG: Unpacked packet: {packet}")
            
            # Handle new dual-arm initialization
            if packet.get('mode') == 'vr_dual_init':
                session_id = packet.get('init_session_id')
                active_robot_count = packet.get('active_robot_count', 1)
                single_arm_mode = packet.get('single_arm_mode', True)
                
                print(f"✅ [INIT] Dual-arm VR system connected!")
                print(f"   📊 Session: {session_id[:8]}...")
                print(f"   🤖 Active robots: {active_robot_count}")
                print(f"   🦾 Single-arm mode: {single_arm_mode}")
                print(f"🔧 DEBUG: Full session ID: {session_id}")
                
                # Send acknowledgment with actual robot count available
                # For now, we'll confirm the requested count (could be modified based on actual hardware)
                confirmed_robot_count = min(active_robot_count, 2)  # Max 2 robots supported
                ack_packet = {
                    'mode': 'vr_dual_init_ack',
                    'confirmed_robot_count': confirmed_robot_count
                }
                print(f"🔧 DEBUG: Sending dual-arm acknowledgment: {ack_packet}")
                sock.sendto(pickle.dumps(ack_packet), addr)
                print("🔧 DEBUG: Dual-arm acknowledgment sent successfully")
                
                break
                
            # Handle legacy single-robot initialization for backward compatibility
            elif packet.get('mode') == 'vr_init':
                session_id = packet.get('init_session_id')
                robot_id = packet.get('robot_id', 0)
                active_robot_count = 1
                single_arm_mode = True
                
                print(f"✅ [INIT] Legacy VR controller connected! Session: {session_id[:8]}..., Robot: {robot_id}")
                print(f"🔧 DEBUG: Full session ID: {session_id}")
                print(f"🔧 DEBUG: Using legacy single-robot mode")
                
                # Send legacy acknowledgment
                ack_packet = {'mode': 'vr_init_ack'}
                print(f"🔧 DEBUG: Sending legacy acknowledgment: {ack_packet}")
                sock.sendto(pickle.dumps(ack_packet), addr)
                print("🔧 DEBUG: Legacy acknowledgment sent successfully")
                
                break
            else:
                print(f"🔧 DEBUG: Ignoring packet with mode: {packet.get('mode')}")
                
        except socket.timeout:
            print("🔧 DEBUG: Socket timeout, retrying...")
            continue
        except Exception as e:
            print(f"❌ [INIT] Error during initialization: {e}")
            print(f"🔧 DEBUG: Init exception details: {type(e).__name__}: {str(e)}")
            continue
    
    return session_id, active_robot_count, single_arm_mode

def print_control_instructions():
    """Print VR control instructions for the user."""
    print("🚀 Starting DUAL-ARM VR teleoperation...")
    print("🎯 Control Instructions:")
    print("   🎮 LEFT Controller → Robot 0 (Left Arm)")
    print("   🎮 RIGHT Controller → Robot 1 (Right Arm)")
    print()
    print("   - Squeeze button: 🎯 MOVEMENT (hold to move robot)")
    print("   - Trigger: 🤏 GRIP (press=close, release=open)")
    print("   - LEFT Controller X button: 🔄 RESET left arm to neutral")
    print("   - RIGHT Controller A button: 🔄 RESET right arm to neutral") 
    print("   - Y/B buttons: (Available for future features)")
    print("   - Menu button: Stepper up")
    print("   - 🕹️  Joystick/Thumbstick: 🚗 BASE MOVEMENT (smooth drive control)")
    print("     • Push forward/back: Move base forward/backward")
    print("     • Push left/right: Turn base left/right")
    print("     • Center joystick: Stop base movement")
    print("   - Ctrl+C: Exit")
    print()
    print("💡 Each controller operates independently!")
    print("🚗 NEW: Smooth joystick-based base control enabled!")

def process_vr_packet(packet, session_id, vr_connected, puppet_left, puppet_right, 
                     stepper, enhanced_base, last_button_states, args):
    """Process a single VR packet and update robots."""
    # Check packet mode and session
    packet_mode = packet.get('mode')
    if packet.get('session_id') != session_id:
        return False, last_button_states
    
    packets_processed = False
    
    # Handle new COMBINED dual VR format
    if packet_mode == 'vr_dual_teleop':
        # Unpack both controllers' data from single packet
        left_data, right_data, _ = unpack_dual_vr_data_from_udp(packet)
        
        print(f"📦 COMBINED PACKET: LEFT={left_data is not None} RIGHT={right_data is not None}")
        
        # Process LEFT controller data
        if left_data and vr_connected and puppet_left:
            try:
                # Debug button states for left controller
                buttons = left_data['button_states']
                print(f"🐛 LEFT buttons: x_button={buttons.get('x_button', False)}, a_button={buttons.get('a_button', False)}, squeeze={buttons.get('squeeze', False)}")
                
                trigger_active = puppet_left.process_vr_command(
                    np.array(left_data['position']), np.array(left_data['orientation']), 
                    left_data['button_states'], left_data['trigger_value']
                )
                print(f"🎮 LEFT CONTROLLER → Robot 0 | Movement: {trigger_active} | Squeeze: {left_data['button_states'].get('squeeze', False)} | Position: [{left_data['position'][0]:.3f},{left_data['position'][1]:.3f},{left_data['position'][2]:.3f}]")
                packets_processed = True
                
                # Process auxiliary controls from left controller
                if left_data['button_states'] != last_button_states or left_data['joystick'] is not None:
                    last_button_states = process_auxiliary_controls(
                        left_data['button_states'], stepper, enhanced_base, left_data['joystick'], last_button_states
                    )
                    
            except Exception as e:
                print(f"Error processing left controller data: {e}")
        
        # Process RIGHT controller data
        if right_data and vr_connected and puppet_right:
            try:
                # Debug button states for right controller
                buttons = right_data['button_states']
                print(f"🐛 RIGHT buttons: x_button={buttons.get('x_button', False)}, a_button={buttons.get('a_button', False)}, squeeze={buttons.get('squeeze', False)}")
                
                trigger_active = puppet_right.process_vr_command(
                    np.array(right_data['position']), np.array(right_data['orientation']), 
                    right_data['button_states'], right_data['trigger_value']
                )
                print(f"🎮 RIGHT CONTROLLER → Robot 1 | Movement: {trigger_active} | Squeeze: {right_data['button_states'].get('squeeze', False)} | Position: [{right_data['position'][0]:.3f},{right_data['position'][1]:.3f},{right_data['position'][2]:.3f}]")
                packets_processed = True
                
                # Process auxiliary controls from right controller (may override left)
                if right_data['button_states'] != last_button_states or right_data['joystick'] is not None:
                    last_button_states = process_auxiliary_controls(
                        right_data['button_states'], stepper, enhanced_base, right_data['joystick'], last_button_states
                    )
                    
            except Exception as e:
                print(f"Error processing right controller data: {e}")
    
    # Handle legacy single controller format (for backward compatibility)
    elif packet_mode == 'vr_teleop':
        # Unpack single VR data (legacy format)
        position, orientation, button_states, trigger_value, _, pkt_robot_id, joystick = unpack_vr_data_from_udp(packet)
        
        # Debug: Show actual VR values received by puppet (only in verbose debug mode)
        if DEBUG_MODE and position is not None and orientation is not None:
            print(f"🎮 LEGACY PACKET: robot_id={pkt_robot_id} pos=[{position[0]:.3f},{position[1]:.3f},{position[2]:.3f}] "
                  f"trigger={trigger_value:.3f} buttons={button_states}")
        
        if position is None or orientation is None:
            return False, last_button_states
        
        # Process VR commands for appropriate robot (only if connected)
        trigger_active = False
        if vr_connected:
            active_robot = None
            controller_name = "UNKNOWN"
            
            # Route based on robot_id with clear feedback
            if pkt_robot_id == 0 and puppet_left:
                active_robot = puppet_left
                controller_name = "LEFT"
            elif pkt_robot_id == 1 and puppet_right:
                active_robot = puppet_right
                controller_name = "RIGHT"
            elif puppet_left:  # Default to left if robot ID doesn't match
                active_robot = puppet_left
                controller_name = "LEFT (default)"
                print(f"⚠️ Warning: Unknown robot_id {pkt_robot_id}, defaulting to left arm")
            
            if active_robot:
                trigger_active = active_robot.process_vr_command(
                    np.array(position), np.array(orientation), 
                    button_states, trigger_value
                )
                
                # Show which robot is being controlled (always show for debugging left arm issues)
                print(f"🎮 {controller_name} CONTROLLER → Robot {pkt_robot_id} | Movement: {trigger_active} | Squeeze: {button_states.get('squeeze', False)} | Position: [{position[0]:.3f},{position[1]:.3f},{position[2]:.3f}]")
                packets_processed = True
            
            # Process auxiliary controls (shared between both controllers) - now includes joystick-based base control
            if button_states != last_button_states or joystick is not None:
                last_button_states = process_auxiliary_controls(
                    button_states, stepper, enhanced_base, joystick, last_button_states
                )
            
            # Debug output
            if args.debug:
                print(f"🐛 VR ({controller_name}): Pos=[{position[0]:.3f},{position[1]:.3f},{position[2]:.3f}] "
                      f"Trigger={trigger_value:.2f} Active={trigger_active}")
        else:
            print("⚠️ VR DISCONNECTED - Skipping command processing")
    
    else:
        # Unknown packet mode
        return False, last_button_states
    
    return packets_processed, last_button_states

def run_teleoperation_loop(sock, session_id, puppet_left, puppet_right, stepper, enhanced_base, args):
    """Run the main VR teleoperation loop."""
    last_button_states = {}
    last_status_time = time.time()
    packet_count = 0
    last_vr_data_time = time.time()
    vr_connection_timeout = 2.0  # Stop if no VR data for 2 seconds
    vr_connected = True
    
    # Dual-arm packet tracking
    robot_packet_counts = {0: 0, 1: 0}  # Track packets per robot
    
    print_control_instructions()
    
    while True:
        loop_start = time.time()
        
        try:
            data, _ = sock.recvfrom(4096)
            packet = pickle.loads(data)
            
            # Update VR connection time - we received valid data!
            last_vr_data_time = time.time()
            if not vr_connected:
                print("✅ VR CONNECTION RESTORED!")
                vr_connected = True
            
            # Process VR packet
            packet_processed, last_button_states = process_vr_packet(
                packet, session_id, vr_connected, puppet_left, puppet_right,
                stepper, enhanced_base, last_button_states, args
            )
            
            if packet_processed:
                packet_count += 1
                
                # Track per-robot packet counts for dual-arm status
                robot_id = packet.get('robot_id', 0)
                if robot_id in robot_packet_counts:
                    robot_packet_counts[robot_id] += 1
                
        except socket.timeout:
            # Check for VR connection timeout
            if time.time() - last_vr_data_time > vr_connection_timeout:
                if vr_connected:
                    print("❌ VR CONNECTION LOST - Robots will stop responding to prevent erratic behavior")
                    vr_connected = False
                # Don't spam disconnection messages
                if (time.time() - last_vr_data_time) % 5.0 < 1.0:  # Every 5 seconds
                    print(f"⏰ VR disconnected for {time.time() - last_vr_data_time:.1f}s - waiting for reconnection...")
            pass
        except Exception as e:
            print(f"❌ Error processing VR data: {e}")
            print(f"🔧 DEBUG: Main loop exception: {type(e).__name__}: {str(e)}")
            if args.debug:
                import traceback
                traceback.print_exc()
        
        # Status reporting
        if time.time() - last_status_time > 5.0:
            elapsed = time.time() - last_status_time
            rate = packet_count / elapsed if elapsed > 0 else 0
            
            # Calculate per-robot rates
            left_rate = robot_packet_counts[0] / elapsed if elapsed > 0 else 0
            right_rate = robot_packet_counts[1] / elapsed if elapsed > 0 else 0
            
            print(f"📊 Dual VR Puppet Status: {rate:.1f} Hz total (Left: {left_rate:.1f}Hz, Right: {right_rate:.1f}Hz)")
            
            packet_count = 0
            robot_packet_counts = {0: 0, 1: 0}
            last_status_time = time.time()
        
        # Maintain control frequency
        elapsed = time.time() - loop_start
        sleep_time = max(0, CONTROL_PERIOD - elapsed)
        time.sleep(sleep_time)

def cleanup_resources(puppet_left, puppet_right, enhanced_base, sock):
    """Cleanup and shutdown all resources."""
    print("🔧 Shutting down DUAL-ARM system...")
    
    # Stop enhanced base controller
    if enhanced_base:
        enhanced_base.stop_control_thread()
    
    # Shutdown robots
    if puppet_left:
        puppet_left.shutdown()
    if puppet_right:
        puppet_right.shutdown()
    
    sock.close()
    print("🔚 Dual-arm VR puppet control terminated")

def main():
    """Main function for PiperX dual-arm VR puppet control."""
    # Parse arguments and handle early exits
    args = parse_arguments()
    
    # Show coordinate transformation info if requested
    if args.coordinate_info:
        show_coordinate_info()
        return
    
    # Configure global settings
    configure_global_settings(args)
    
    # Initialize all components
    puppet_left, puppet_right = initialize_robots(args)
    stepper, enhanced_base = initialize_auxiliary_controllers(args)
    sock = setup_vr_communication()
    
    try:
        # VR initialization handshake
        session_id, active_robot_count, single_arm_mode = wait_for_vr_initialization(sock)
        
        print(f"🤖 VR System Configuration:")
        print(f"   📊 Active robots: {active_robot_count}")
        print(f"   🦾 Single-arm mode: {single_arm_mode}")
        print(f"   🔗 Session ID: {session_id[:8]}...")
        
        # Validate robot configuration matches initialization
        available_robots = sum(1 for robot in [puppet_left, puppet_right] if robot is not None)
        if active_robot_count > available_robots:
            print(f"⚠️ Warning: VR system expects {active_robot_count} robots but only {available_robots} are available")
            print("Continuing with available robots...")
        
        # Run main teleoperation loop
        run_teleoperation_loop(sock, session_id, puppet_left, puppet_right, stepper, enhanced_base, args)
        
    except KeyboardInterrupt:
        print("\n🛑 Dual-arm VR teleoperation stopped by user")
    except Exception as e:
        print(f"❌ Dual-arm VR teleoperation error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        cleanup_resources(puppet_left, puppet_right, enhanced_base, sock)

if __name__ == '__main__':
    main()
