#!/usr/bin/env python3
"""
PiperX Puppet Robot Control Script (Flexible Version)
Supports optional hardware and single-arm operation
"""

import socket
import pickle
import time
import sys
import os
import numpy as np
import argparse

# Add the parent directory to the path
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from xemb_scripts.constants import *
from packing_utils_widowx_piperx import unpack_data_from_udp
from robot_mapper import JointMapper

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
UDP_PORT = 5010
BASE_SPEED = 1
CONTROL_PERIOD = 0.02  # 50Hz control loop
MAX_JOINT_VELOCITY = [2.0, 2.0, 2.0, 3.0, 3.0, 3.0] # rad/s, chosen for smooth but responsive movement





def clip_joint_velocity(achieved, target, max_vel, dt):
    """Clips the velocity between the achieved and target joint positions."""
    clipped_target = []
    for i in range(len(target)):
        # Handle None case for single-arm mode
        if target[i] is None or achieved[i] is None:
            clipped_target.append(target[i])
            continue
            
        delta = target[i] - achieved[i]
        max_delta = max_vel[i] * dt
        
        if abs(delta) > max_delta:
            clipped_target.append(achieved[i] + (max_delta if delta > 0 else -max_delta))
        else:
            clipped_target.append(target[i])
            
    return clipped_target

def process_key_states(key_states, stepper, base, last_states=None):
    """Process keyboard input for stepper and base control"""
    if last_states is None:
        last_states = {}
    
    # Stepper control (W/S for up/down)
    if key_states.get('w', False) and not last_states.get('w', False):
        if stepper and hasattr(stepper, 'move_steps'):
            stepper.move_steps(50)
        elif stepper:
            print("Mock: Stepper up")
    if key_states.get('s', False) and not last_states.get('s', False):
        if stepper and hasattr(stepper, 'move_steps'):
            stepper.move_steps(-50)
        elif stepper:
            print("Mock: Stepper down")
    
    # Base control (A/D for left/right)
    if key_states.get('a', False) and not last_states.get('a', False):
        if base and hasattr(base, 'turn_left'):
            base.turn_left(0.2)
        elif base:
            print("Mock: Base left")
    if key_states.get('d', False) and not last_states.get('d', False):
        if base and hasattr(base, 'turn_right'):
            base.turn_right(0.2)
        elif base:
            print("Mock: Base right")
    
    # Arrow keys for additional stepper control
    if key_states.get('up', False) and not last_states.get('up', False):
        if stepper and hasattr(stepper, 'move_steps'):
            stepper.move_steps(10)
        elif stepper:
            print("Mock: Stepper up (fine)")
    if key_states.get('down', False) and not last_states.get('down', False):
        if stepper and hasattr(stepper, 'move_steps'):
            stepper.move_steps(-10)
        elif stepper:
            print("Mock: Stepper down (fine)")
    
    return key_states

def prep_piperx_robot(piper, neutral_positions, gripper_closed):
    """Prepare PiperX robot for operation"""
    try:
        # Initialize gripper to closed position
        piper.GripperCtrl(int(gripper_closed), 1000, 0x01, 0)
        
        # Set initial joint positions to neutral from calibration
        piper.MotionCtrl_2(0x01, 0x01, 100, 0x00)
        piper.JointCtrl(*[int(pos) for pos in neutral_positions])
        
        print("PiperX robot prepared successfully with neutral positions")
        return True
    except Exception as e:
        print(f"Error preparing PiperX robot: {e}")
        return False

def try_connect_piperx(can_interface):
    """Try to connect to a PiperX robot on the given CAN interface"""
    try:
        print(f"Attempting to connect to PiperX on {can_interface}...")
        piper = C_PiperInterface_V2(can_interface)
        piper.ConnectPort()
        
        # Wait for connection with timeout
        timeout = 5.0  # 5 second timeout
        start_time = time.time()
        while not piper.get_connect_status():
            if time.time() - start_time > timeout:
                print(f"Connection timeout for {can_interface}")
                return None
            time.sleep(0.01)
        
        # Try to enable the robot
        print(f"Enabling PiperX on {can_interface}...")
        start_time = time.time()
        while not piper.EnablePiper():
            if time.time() - start_time > timeout:
                print(f"Enable timeout for {can_interface}")
                return None
            time.sleep(0.01)
        
        # We'll prepare the robot after loading calibration in main()
        # Just return the connected robot for now
        
        print(f"PiperX on {can_interface} ready for teleoperation")
        return piper
        
    except Exception as e:
        print(f"Failed to connect to PiperX on {can_interface}: {e}")
        return None

def main():
    parser = argparse.ArgumentParser(description='PiperX Puppet Robot Control (Flexible)')
    parser.add_argument('--single-arm', action='store_true', help='Use a single arm (select with --arm-side)')
    parser.add_argument('--arm-side', choices=['left', 'right'], default='left',
                        help='Which arm to use in single-arm mode (default: left)')
    parser.add_argument('--no-stepper', action='store_true', help='Disable stepper motor control')
    parser.add_argument('--no-base', action='store_true', help='Disable base robot control')
    parser.add_argument('--can-left', default='can_left', help='CAN interface for left arm (e.g., can_left)')
    parser.add_argument('--can-right', default='can_right', help='CAN interface for right arm (e.g., can_right)')
    parser.add_argument('--smoothing-alpha', type=float, default=0.8, 
                       help='Smoothing factor (0.0-1.0): lower=smoother but slower, higher=more responsive (default: 0.8)')
    args = parser.parse_args()
    
    # Validate smoothing alpha
    if not 0.0 <= args.smoothing_alpha <= 1.0:
        print(f"ERROR: --smoothing-alpha must be between 0.0 and 1.0, got {args.smoothing_alpha}")
        sys.exit(1)
    
    print("Initializing PiperX Puppet Robot (Flexible Version)...")
    print(f"Single arm mode: {args.single_arm} (side: {args.arm_side if args.single_arm else 'N/A'})")
    print(f"Stepper control: {not args.no_stepper and STEPPER_AVAILABLE}")
    print(f"Base control: {not args.no_base and BASE_AVAILABLE}")
    print(f"Smoothing alpha: {args.smoothing_alpha} (lower=smoother/slower, higher=more responsive)")
    
    # Check if we're expecting no-heartbeat mode
    print("Expecting heartbeat mode (12.5Hz per joint) unless master uses --no-heartbeat flag")
    
    # Initialize PiperX robots
    puppet_left = None
    puppet_right = None
    if args.single_arm:
        if args.arm_side == 'left':
            puppet_left = try_connect_piperx(args.can_left)
            if puppet_left is None:
                print(f"ERROR: Could not connect to LEFT arm on {args.can_left}")
                sys.exit(1)
        else:  # right-only
            puppet_right = try_connect_piperx(args.can_right)
            if puppet_right is None:
                print(f"ERROR: Could not connect to RIGHT arm on {args.can_right}")
                sys.exit(1)
    else:
        puppet_left = try_connect_piperx(args.can_left)
        if puppet_left is None:
            print(f"ERROR: Could not connect to left arm on {args.can_left}")
            sys.exit(1)
        puppet_right = try_connect_piperx(args.can_right)
        if puppet_right is None:
            print(f"WARNING: Could not connect to right arm on {args.can_right}")
            print("Continuing with single arm mode (left-only)...")
            args.single_arm = True
            args.arm_side = 'left'
    
    # Initialize the JointMapper
    print("Loading calibration files...")
    mapper = JointMapper(
        master_calib_path='widowx_calibration.json',
        puppet_calib_path='piperx_calibration.json'
    )
    
    # Load PiperX calibration data for neutral positions
    import json
    try:
        with open('piperx_calibration.json', 'r') as f:
            piperx_calib = json.load(f)
        neutral_positions = piperx_calib['neutral_positions']
        gripper_closed = piperx_calib['gripper_limits'][0]  # Closed position
        print(f"Loaded neutral positions: {neutral_positions}")
        print(f"Gripper closed position: {gripper_closed}")
    except Exception as e:
        print(f"Error loading PiperX calibration: {e}")
        print("Using default neutral positions (all zeros)")
        neutral_positions = [0, 0, 0, 0, 0, 0]
        gripper_closed = 0
    
    # Prepare the robots with neutral positions
    if puppet_left:
        if not prep_piperx_robot(puppet_left, neutral_positions, gripper_closed):
            print("Failed to prepare left PiperX robot")
            sys.exit(1)
    
    if puppet_right:
        if not prep_piperx_robot(puppet_right, neutral_positions, gripper_closed):
            print("Failed to prepare right PiperX robot")
            sys.exit(1)

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
    
    # Initialize joint position storage per selected mode
    joint_commands_left = None
    joint_commands_right = None
    smoothed_joint_left = None
    smoothed_joint_right = None
    smoothed_gripper_left = None
    smoothed_gripper_right = None
    target_gripper_left = None
    target_gripper_right = None

    if args.single_arm:
        if args.arm_side == 'left':
            joint_commands_left = [0.0] * 6
            smoothed_joint_left = joint_commands_left.copy()
            smoothed_gripper_left = 0.0
            target_gripper_left = 0.0
        else:  # right-only
            joint_commands_right = [0.0] * 6
            smoothed_joint_right = joint_commands_right.copy()
            smoothed_gripper_right = 0.0
            target_gripper_right = 0.0
    else:
        joint_commands_left = [0.0] * 6
        joint_commands_right = [0.0] * 6
        smoothed_joint_left = joint_commands_left.copy()
        smoothed_joint_right = joint_commands_right.copy()
        smoothed_gripper_left = 0.0
        smoothed_gripper_right = 0.0
        target_gripper_left = 0.0
        target_gripper_right = 0.0
    last_key_states = {}
    most_recent_key_states = {}
    
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
                
                # Map initial positions using the mapper
                print("[INIT] Mapping initial positions...")
                mapped_jl = mapper.map_joints(jl) if jl is not None else None
                mapped_jr = mapper.map_joints(jr) if jr is not None else None
                mapped_gl = mapper.map_gripper(gl) if gl is not None else None
                mapped_gr = mapper.map_gripper(gr) if gr is not None else None

                print("[INIT] Moving to mapped initial positions...")

                if args.single_arm and args.arm_side == 'right':
                    # Right-only initialization
                    if mapped_jr is not None and puppet_right is not None:
                        right_joints_deg = [angle * 180.0 / np.pi * 1000 for angle in mapped_jr]
                        puppet_right.MotionCtrl_2(0x01, 0x01, 100, 0x00)
                        puppet_right.JointCtrl(*[int(angle) for angle in right_joints_deg])
                        if mapped_gr is not None:
                            right_gripper_deg = int(mapped_gr * 180.0 / np.pi * 1000)
                            puppet_right.GripperCtrl(gripper_angle=right_gripper_deg, gripper_effort=1000, gripper_code=0x01)
                        smoothed_joint_right = mapped_jr.copy()
                        smoothed_gripper_right = mapped_gr if mapped_gr is not None else 0.0
                else:
                    # Default to left (single-arm left or dual)
                    if mapped_jl is not None and puppet_left is not None:
                        left_joints_deg = [angle * 180.0 / np.pi * 1000 for angle in mapped_jl]
                        puppet_left.MotionCtrl_2(0x01, 0x01, 100, 0x00)
                        puppet_left.JointCtrl(*[int(angle) for angle in left_joints_deg])
                        if mapped_gl is not None:
                            left_gripper_deg = int(mapped_gl * 180.0 / np.pi * 1000)
                            puppet_left.GripperCtrl(gripper_angle=left_gripper_deg, gripper_effort=1000, gripper_code=0x01)
                        smoothed_joint_left = mapped_jl.copy()
                        smoothed_gripper_left = mapped_gl if mapped_gl is not None else 0.0

                    # If right exists in dual-arm
                    if puppet_right is not None and mapped_jr is not None and not args.single_arm:
                        right_joints_deg = [angle * 180.0 / np.pi * 1000 for angle in mapped_jr]
                        puppet_right.MotionCtrl_2(0x01, 0x01, 100, 0x00)
                        puppet_right.JointCtrl(*[int(angle) for angle in right_joints_deg])
                        if mapped_gr is not None:
                            right_gripper_deg = int(mapped_gr * 180.0 / np.pi * 1000)
                            puppet_right.GripperCtrl(gripper_angle=right_gripper_deg, gripper_effort=1000, gripper_code=0x01)
                        smoothed_joint_right = mapped_jr.copy()
                        smoothed_gripper_right = mapped_gr if mapped_gr is not None else 0.0
                
                # Send confirmation
                confirm_packet = {'mode': 'init_ack'}
                sock.sendto(pickle.dumps(confirm_packet), addr)
                print("[INIT] Sent confirmation. Starting teleoperation...")
                break
                
        except socket.timeout:
            continue
        except Exception as e:
            print(f"[INIT] Error during initialization: {e}")
            continue

    try:
        print("Starting direct UDP teleoperation with joint mapping...")
        print(f"Control mode: {'Single arm' if args.single_arm else 'Dual arm'}")
        while True:
            loop_start = time.time()
            try:
                data, _ = sock.recvfrom(4096)
                packed = pickle.loads(data)
                # Ignore packets that are not teleop packets or from a different session
                # Accept both heartbeat and no-heartbeat packets
                if (('heartbeat' not in packed and not packed.get('no_heartbeat', False)) or 
                    packed.get('session_id') != session_id):
                    continue
                jl, jr, gl, gr, key_states = unpack_data_from_udp(packed)
                
                # Map incoming data using the mapper
                if args.single_arm and args.arm_side == 'right':
                    if jr is not None and joint_commands_right is not None:
                        mapped_jr = mapper.map_joints(jr)
                        for i in range(6):
                            if mapped_jr[i] is not None:
                                joint_commands_right[i] = mapped_jr[i]
                    if gr is not None:
                        target_gripper_right = mapper.map_gripper(gr)
                else:
                    if jl is not None and joint_commands_left is not None:
                        mapped_jl = mapper.map_joints(jl)
                        for i in range(6):
                            if mapped_jl[i] is not None:
                                joint_commands_left[i] = mapped_jl[i]
                    if not args.single_arm and jr is not None and joint_commands_right is not None:
                        mapped_jr = mapper.map_joints(jr)
                        for i in range(6):
                            if mapped_jr[i] is not None:
                                joint_commands_right[i] = mapped_jr[i]
                    if gl is not None:
                        target_gripper_left = mapper.map_gripper(gl)
                    if not args.single_arm and gr is not None:
                        target_gripper_right = mapper.map_gripper(gr)

                if key_states is not None:
                    most_recent_key_states = key_states
            except socket.timeout:
                pass
            
            # Always smooth toward the most recent target at every tick
            if smoothed_joint_left is not None and joint_commands_left is not None:
                for i in range(6):
                    smoothed_joint_left[i] = args.smoothing_alpha * joint_commands_left[i] + (1 - args.smoothing_alpha) * smoothed_joint_left[i]
            if smoothed_joint_right is not None and joint_commands_right is not None:
                for i in range(6):
                    smoothed_joint_right[i] = args.smoothing_alpha * joint_commands_right[i] + (1 - args.smoothing_alpha) * smoothed_joint_right[i]

            if smoothed_gripper_left is not None and target_gripper_left is not None:
                smoothed_gripper_left = args.smoothing_alpha * target_gripper_left + (1 - args.smoothing_alpha) * smoothed_gripper_left
            if smoothed_gripper_right is not None and target_gripper_right is not None:
                smoothed_gripper_right = args.smoothing_alpha * target_gripper_right + (1 - args.smoothing_alpha) * smoothed_gripper_right

            # Send commands to robots
            # Convert radians to degrees * 1000 for PiperX
            if puppet_left is not None and smoothed_joint_left is not None:
                left_joints_deg = [angle * 180.0 / np.pi * 1000 for angle in smoothed_joint_left]
                puppet_left.MotionCtrl_2(0x01, 0x01, 100, 0x00)
                puppet_left.JointCtrl(*[int(angle) for angle in left_joints_deg])
                if smoothed_gripper_left is not None:
                    left_gripper_deg = int(smoothed_gripper_left * 180.0 / np.pi * 1000)
                    puppet_left.GripperCtrl(gripper_angle=left_gripper_deg, gripper_effort=1000, gripper_code=0x01)

            if puppet_right is not None and smoothed_joint_right is not None:
                right_joints_deg = [angle * 180.0 / np.pi * 1000 for angle in smoothed_joint_right]
                puppet_right.MotionCtrl_2(0x01, 0x01, 100, 0x00)
                puppet_right.JointCtrl(*[int(angle) for angle in right_joints_deg])
                if smoothed_gripper_right is not None:
                    right_gripper_deg = int(smoothed_gripper_right * 180.0 / np.pi * 1000)
                    puppet_right.GripperCtrl(gripper_angle=right_gripper_deg, gripper_effort=1000, gripper_code=0x01)
            
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
            puppet_left.JointCtrl(*[int(pos) for pos in neutral_positions])
            puppet_left.GripperCtrl(int(gripper_closed), 1000, 0x01, 0)
            
            if puppet_right is not None:
                puppet_right.JointCtrl(*[int(pos) for pos in neutral_positions])
                puppet_right.GripperCtrl(int(gripper_closed), 1000, 0x01, 0)
            
            time.sleep(1.0)
        except Exception as e:
            print(f"Error during shutdown: {e}")
        
        print("Puppet control terminated.")

if __name__ == '__main__':
    main() 