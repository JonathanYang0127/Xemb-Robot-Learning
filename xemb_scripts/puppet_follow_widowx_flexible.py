#!/usr/bin/env python3
"""
WidowX Puppet Robot Control Script (Flexible Version)
Supports optional hardware and single-arm operation
"""

import socket
import pickle
import time
import sys
import os
import numpy as np
import argparse
import rospy
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from interbotix_xs_msgs.msg import JointSingleCommand

# Add the parent directory to the path
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from constants import *
from packing_utils_direct import unpack_data_from_udp
from robot_utils import torque_on, torque_off, move_arms, move_grippers

# Try to import hardware controllers, use mocks if not available
try:
    from stepper_control.stepper_controller import StepperController
    STEPPER_AVAILABLE = True
except ImportError:
    print("Warning: Stepper controller not available, using mock")
    STEPPER_AVAILABLE = False

try:
    from base_control.base_controller import MDCRobot
    BASE_AVAILABLE = True
except ImportError:
    print("Warning: Base controller not available, using mock")
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

def process_key_states(key_states, stepper, base, last_states=None):
    """Process keyboard input for stepper and base control"""
    if key_states is None:
        return last_states
    
    # Handle stepper controls (arrow keys)
    if key_states.get('up', False) and stepper:
        if hasattr(stepper, 'move_steps'):
            stepper.move_steps(10)
        else:
            print("Mock: Stepper up")
    if key_states.get('down', False) and stepper:
        if hasattr(stepper, 'move_steps'):
            stepper.move_steps(-10)
        else:
            print("Mock: Stepper down")
    
    # Handle base controls (WASD keys)
    if key_states.get('w', False) and base:
        if hasattr(base, 'move_forward'):
            base.move_forward(0.1)
        else:
            print("Mock: Base forward")
    if key_states.get('s', False) and base:
        if hasattr(base, 'move_backward'):
            base.move_backward(0.1)
        else:
            print("Mock: Base backward")
    if key_states.get('a', False) and base:
        if hasattr(base, 'turn_left'):
            base.turn_left(0.1)
        else:
            print("Mock: Base left")
    if key_states.get('d', False) and base:
        if hasattr(base, 'turn_right'):
            base.turn_right(0.1)
        else:
            print("Mock: Base right")
    
    return key_states

def enforce_joint_limits(joint_positions, joint_limits):
    """Enforce joint limits on the given positions"""
    limited_positions = joint_positions.copy()
    violations = []
    
    for i, (pos, limits) in enumerate(zip(joint_positions, joint_limits)):
        if limits[0] is not None and pos < limits[0]:
            limited_positions[i] = limits[0]
            violations.append(f"Joint {i}: {pos:.3f} < {limits[0]:.3f}")
        elif limits[1] is not None and pos > limits[1]:
            limited_positions[i] = limits[1]
            violations.append(f"Joint {i}: {pos:.3f} > {limits[1]:.3f}")
    
    return limited_positions, violations

def try_connect_widowx(robot_name, init_node=True):
    """Try to connect to a WidowX robot"""
    try:
        print(f"Attempting to connect to WidowX robot: {robot_name}...")
        robot = InterbotixManipulatorXS(
            robot_model="vx300s",
            group_name="arm",
            gripper_name="gripper",
            robot_name=robot_name,
            init_node=init_node
        )
        
        # Set operating modes
        robot.dxl.robot_set_operating_modes("group", "arm", "position")
        robot.dxl.robot_set_operating_modes("single", "gripper", "current_based_position")
        
        # Enable torque
        torque_on(robot)
        
        print(f"WidowX robot {robot_name} ready for teleoperation")
        return robot
        
    except Exception as e:
        print(f"Failed to connect to WidowX robot {robot_name}: {e}")
        return None

def main():
    parser = argparse.ArgumentParser(description='WidowX Puppet Robot Control (Flexible)')
    parser.add_argument('--single-arm', action='store_true', help='Use only left arm')
    parser.add_argument('--no-stepper', action='store_true', help='Disable stepper motor control')
    parser.add_argument('--no-base', action='store_true', help='Disable base robot control')
    parser.add_argument('--left-name', default='puppet_left', help='Name for left robot (default: puppet_left)')
    parser.add_argument('--right-name', default='puppet_right', help='Name for right robot (default: puppet_right)')
    args = parser.parse_args()
    
    print("Initializing WidowX Puppet Robot (Flexible Version)...")
    print(f"Single arm mode: {args.single_arm}")
    print(f"Stepper control: {not args.no_stepper and STEPPER_AVAILABLE}")
    print(f"Base control: {not args.no_base and BASE_AVAILABLE}")
    
    # Initialize ROS node
    rospy.init_node('widowx_puppet_flexible', anonymous=True)
    
    # Load WidowX calibration data for neutral positions
    import json
    try:
        with open('piperx_control/widowx_calibration.json', 'r') as f:
            widowx_calib = json.load(f)
        widowx_neutral_positions = widowx_calib['neutral_positions']
        widowx_gripper_closed = widowx_calib['gripper_limits'][0]  # Closed position
        print(f"Loaded WidowX neutral positions: {widowx_neutral_positions}")
        print(f"WidowX gripper closed position: {widowx_gripper_closed}")
    except Exception as e:
        print(f"Error loading WidowX calibration: {e}")
        print("Using default neutral positions from START_ARM_POSE")
        widowx_neutral_positions = START_ARM_POSE[:6]
        widowx_gripper_closed = PUPPET_GRIPPER_JOINT_CLOSE
    
    # Initialize WidowX robots
    puppet_left = try_connect_widowx(args.left_name, init_node=False)  # ROS already initialized
    if puppet_left is None:
        print(f"ERROR: Could not connect to left arm ({args.left_name})")
        sys.exit(1)
    
    puppet_right = None
    if not args.single_arm:
        puppet_right = try_connect_widowx(args.right_name, init_node=False)
        if puppet_right is None:
            print(f"WARNING: Could not connect to right arm ({args.right_name})")
            print("Continuing with single arm mode...")
            args.single_arm = True

    # Initialize controllers based on availability and arguments
    stepper = None
    if not args.no_stepper:
        if STEPPER_AVAILABLE:
            try:
                stepper = StepperController(port="/dev/tty_stepper")
                stepper.enable()
                print("Real stepper controller initialized")
            except Exception as e:
                print(f"Failed to initialize stepper: {e}, using mock")
                stepper = MockStepperController(port="/dev/tty_stepper")
        else:
            stepper = MockStepperController(port="/dev/tty_stepper")
    
    base = None
    if not args.no_base:
        if BASE_AVAILABLE:
            try:
                base = MDCRobot(port='/dev/tty_base', default_speed=BASE_SPEED)
                base.connect()
                print("Real base controller initialized")
            except Exception as e:
                print(f"Failed to initialize base: {e}, using mock")
                base = MockMDCRobot(port='/dev/tty_base', default_speed=BASE_SPEED)
        else:
            base = MockMDCRobot(port='/dev/tty_base', default_speed=BASE_SPEED)
    
    # Initialize joint limits
    JOINT_LIMITS_LEFT = [
        [-0.4939418137073517, None],   # waist (left)
        [-1.6, 0.348],                 # shoulder (left)
        [-0.45, 1.5],                  # elbow (left)
        [None, None],                  # wrist_angle (left)
        [None, None],                  # wrist_rotate (left)
        [None, None],                  # gripper or other (left)
    ]
    
    JOINT_LIMITS_RIGHT = [
        [None, 0.39],                  # waist (right)
        [-1.6, 0.348],                 # shoulder (right)
        [-0.45, 1.5],                  # elbow (right)
        [None, None],                  # wrist_angle (right)
        [None, None],                  # wrist_rotate (right)
        [None, None],                  # gripper or other (right)
    ]
    
    # Initialize control variables
    gripper_command_left = JointSingleCommand(name="gripper")
    gripper_command_right = JointSingleCommand(name="gripper") if not args.single_arm else None
    
    # Initialize joint position storage using calibration neutral positions
    joint_commands_left = widowx_neutral_positions.copy()
    joint_commands_right = widowx_neutral_positions.copy() if not args.single_arm else None
    smoothed_joint_left = joint_commands_left.copy()
    smoothed_joint_right = joint_commands_right.copy() if not args.single_arm else None
    smoothed_gripper_left = widowx_gripper_closed
    smoothed_gripper_right = widowx_gripper_closed if not args.single_arm else None
    target_gripper_left = widowx_gripper_closed
    target_gripper_right = widowx_gripper_closed if not args.single_arm else None
    last_key_states = None
    most_recent_key_states = None
    
    # Setup UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", UDP_PORT))
    sock.settimeout(0.1)

    # INIT HANDSHAKE: Wait for initial positions from master
    print("[INIT] Waiting for initial positions from master...")
    session_id = None
    while True:
        try:
            data, addr = sock.recvfrom(4096)
            packet = pickle.loads(data)
            if packet.get('mode') == 'init_request':
                # Reply with ack and continue waiting for actual init
                ack_packet = {'mode': 'init_ack'}
                sock.sendto(pickle.dumps(ack_packet), addr)
                print("[INIT] Received init_request, sent ack. Waiting for initial positions...")
                continue
            if packet.get('mode') == 'init':
                jl = packet['jl']
                jr = packet['jr']
                gl = packet['gl']
                gr = packet['gr']
                session_id = packet.get('init_session_id')
                
                # Enforce joint limits
                limited_jl, violations_left = enforce_joint_limits(jl, JOINT_LIMITS_LEFT)
                limited_jr, violations_right = enforce_joint_limits(jr, JOINT_LIMITS_RIGHT) if not args.single_arm else (None, [])
                
                if violations_left:
                    print(f"[INIT] Left arm joint limit violations: {violations_left}")
                if violations_right:
                    print(f"[INIT] Right arm joint limit violations: {violations_right}")
                
                print("[INIT] Moving to received initial positions...")
                
                # Move to initial positions
                move_arms([puppet_left], [limited_jl], move_time=1.5)
                move_grippers([puppet_left], [MASTER2PUPPET_JOINT_FN(gl)], move_time=0.7)
                
                # Update smoothed positions
                joint_commands_left = limited_jl.copy()
                smoothed_joint_left = limited_jl.copy()
                smoothed_gripper_left = MASTER2PUPPET_JOINT_FN(gl)
                target_gripper_left = MASTER2PUPPET_JOINT_FN(gl)
                
                # Handle right arm if available
                if puppet_right is not None and limited_jr is not None:
                    move_arms([puppet_right], [limited_jr], move_time=1.5)
                    move_grippers([puppet_right], [MASTER2PUPPET_JOINT_FN(gr)], move_time=0.7)
                    
                    joint_commands_right = limited_jr.copy()
                    smoothed_joint_right = limited_jr.copy()
                    smoothed_gripper_right = MASTER2PUPPET_JOINT_FN(gr)
                    target_gripper_right = MASTER2PUPPET_JOINT_FN(gr)
                
                # Send confirmation
                confirm_packet = {'mode': 'init_confirm'}
                sock.sendto(pickle.dumps(confirm_packet), addr)
                print("[INIT] Sent confirmation. Starting teleoperation...")
                break
                
        except socket.timeout:
            continue
        except Exception as e:
            print(f"[INIT] Error during initialization: {e}")
            continue

    try:
        print("Starting direct UDP teleoperation with robust smoothing...")
        print(f"Control mode: {'Single arm' if args.single_arm else 'Dual arm'}")
        while True:
            loop_start = time.time()
            try:
                data, _ = sock.recvfrom(4096)
                packed = pickle.loads(data)
                # Ignore packets that are not teleop packets or from a different session
                if 'heartbeat' not in packed or packed.get('session_id') != session_id:
                    continue
                jl, jr, gl, gr, key_states = unpack_data_from_udp(packed)
                
                # Update target joint commands if new data is present
                if jl is not None:
                    for i in range(6):
                        if jl[i] is not None:
                            joint_commands_left[i] = jl[i]
                if jr is not None and not args.single_arm:
                    for i in range(6):
                        if jr[i] is not None:
                            joint_commands_right[i] = jr[i]
                if gl is not None:
                    target_gripper_left = MASTER2PUPPET_JOINT_FN(gl)
                if gr is not None and not args.single_arm:
                    target_gripper_right = MASTER2PUPPET_JOINT_FN(gr)

                if key_states is not None:
                    most_recent_key_states = key_states
            except socket.timeout:
                pass
            
            # Always smooth toward the most recent target at every tick
            for i in range(6):
                smoothed_joint_left[i] = SMOOTH_ALPHA * joint_commands_left[i] + (1 - SMOOTH_ALPHA) * smoothed_joint_left[i]
                if not args.single_arm and smoothed_joint_right is not None:
                    smoothed_joint_right[i] = SMOOTH_ALPHA * joint_commands_right[i] + (1 - SMOOTH_ALPHA) * smoothed_joint_right[i]
            
            smoothed_gripper_left = SMOOTH_ALPHA * target_gripper_left + (1 - SMOOTH_ALPHA) * smoothed_gripper_left
            if not args.single_arm and smoothed_gripper_right is not None:
                smoothed_gripper_right = SMOOTH_ALPHA * target_gripper_right + (1 - SMOOTH_ALPHA) * smoothed_gripper_right

            # Enforce joint limits on smoothed commands
            limited_left, _ = enforce_joint_limits(smoothed_joint_left, JOINT_LIMITS_LEFT)
            limited_right, _ = enforce_joint_limits(smoothed_joint_right, JOINT_LIMITS_RIGHT) if not args.single_arm and smoothed_joint_right is not None else (None, [])

            # Send commands to robots
            puppet_left.arm.set_joint_positions(limited_left, blocking=False)
            gripper_command_left.cmd = smoothed_gripper_left
            puppet_left.gripper.core.pub_single.publish(gripper_command_left)
            
            # Handle right arm if available
            if puppet_right is not None and limited_right is not None:
                puppet_right.arm.set_joint_positions(limited_right, blocking=False)
                gripper_command_right.cmd = smoothed_gripper_right
                puppet_right.gripper.core.pub_single.publish(gripper_command_right)
            
            # Handle key-based controls for stepper and base
            if most_recent_key_states != last_key_states:
                last_key_states = process_key_states(most_recent_key_states, stepper, base, last_key_states)

            # Dynamic sleep to maintain control frequency
            elapsed = time.time() - loop_start
            sleep_time = max(0, CONTROL_PERIOD - elapsed)
            time.sleep(sleep_time)
            
    except KeyboardInterrupt:
        print("\nTeleoperation stopped by user.")
    finally:
        print("Shutting down controllers...")
        try:
            # Move to neutral position before shutdown
            move_arms([puppet_left], [widowx_neutral_positions], move_time=1.0)
            move_grippers([puppet_left], [widowx_gripper_closed], move_time=0.5)
            
            if puppet_right is not None:
                move_arms([puppet_right], [widowx_neutral_positions], move_time=1.0)
                move_grippers([puppet_right], [widowx_gripper_closed], move_time=0.5)
            
            # Turn off torque
            torque_off(puppet_left)
            if puppet_right is not None:
                torque_off(puppet_right)
            
            time.sleep(1.0)
        except Exception as e:
            print(f"Error during shutdown: {e}")
        
        print("Puppet control terminated.")

if __name__ == '__main__':
    main() 