#!/usr/bin/env python3
"""
PiperX Puppet Robot Control Script (Dedicated Dual-Arm + Full Hardware)
Requires: 2 PiperX arms, stepper motor, and base robot
No optional components - all hardware must be present
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

# Required imports - fail fast if not available
try:
    from piper_sdk import C_PiperInterface_V2
except ImportError:
    print("ERROR: piper_sdk not found. Please install with: pip install piper-sdk")
    sys.exit(1)

try:
    from xemb_scripts.stepper_control.stepper_controller import StepperController
except ImportError:
    print("ERROR: stepper_control not available. Hardware required for dual-full mode.")
    sys.exit(1)

try:
    from xemb_scripts.base_control.base_controller import MDCRobot
except ImportError:
    print("ERROR: base_control not available. Hardware required for dual-full mode.")
    sys.exit(1)

# Hardcoded configuration for optimal performance
CAN_LEFT = "can_left"
CAN_RIGHT = "can_right"
UDP_PORT = 5005
BASE_SPEED = 1
CONTROL_PERIOD = 0.02  # 50Hz control loop
SMOOTHING_ALPHA = 0.8  # More responsive for full system
STEPPER_LARGE_STEP = 100
STEPPER_FINE_STEP = 10
MAX_JOINT_VELOCITY = [2.0, 2.0, 2.0, 3.0, 3.0, 3.0]  # rad/s

def check_hardware_health(puppet_left, puppet_right):
    """Monitor hardware connections and fail if issues detected"""
    if not puppet_left.get_connect_status():
        raise RuntimeError("Lost connection to left PiperX arm")
    if not puppet_right.get_connect_status():
        raise RuntimeError("Lost connection to right PiperX arm")

def clip_joint_velocity(achieved, target, max_vel, dt):
    """Clips the velocity between the achieved and target joint positions."""
    clipped_target = []
    for i in range(len(target)):
        delta = target[i] - achieved[i]
        max_delta = max_vel[i] * dt
        
        if abs(delta) > max_delta:
            clipped_target.append(achieved[i] + (max_delta if delta > 0 else -max_delta))
        else:
            clipped_target.append(target[i])
            
    return clipped_target

def process_key_states_full(key_states, stepper, base, last_states=None):
    """Enhanced key processing for full system control"""
    if last_states is None:
        last_states = {}
    
    # Stepper control with variable step sizes
    # W/S for large steps, Up/Down arrows for fine control
    if key_states.get('w', False) and not last_states.get('w', False):
        stepper.move_steps(STEPPER_LARGE_STEP)
        print(f"Stepper: Large step up ({STEPPER_LARGE_STEP})")
    if key_states.get('s', False) and not last_states.get('s', False):
        stepper.move_steps(-STEPPER_LARGE_STEP)
        print(f"Stepper: Large step down (-{STEPPER_LARGE_STEP})")
    
    if key_states.get('up', False) and not last_states.get('up', False):
        stepper.move_steps(STEPPER_FINE_STEP)
        print(f"Stepper: Fine step up ({STEPPER_FINE_STEP})")
    if key_states.get('down', False) and not last_states.get('down', False):
        stepper.move_steps(-STEPPER_FINE_STEP)
        print(f"Stepper: Fine step down (-{STEPPER_FINE_STEP})")
    
    # Continuous base control
    # A/D for rotation, Left/Right arrows for forward/backward
    base_active = False
    if key_states.get('a', False):
        base.turn_left(strength=0.3, duration=0.1)
        base_active = True
    elif key_states.get('d', False):
        base.turn_right(strength=0.3, duration=0.1)
        base_active = True
    elif key_states.get('left', False):
        base.drive(speed=-BASE_SPEED, turn=0)
        base_active = True
    elif key_states.get('right', False):
        base.drive(speed=BASE_SPEED, turn=0)
        base_active = True
    
    # Stop base if no movement keys are pressed
    if not base_active:
        base.stop()
    
    return key_states

def prep_piperx_robot(piper, neutral_positions, gripper_closed, arm_name):
    """Prepare PiperX robot for operation"""
    try:
        # Initialize gripper to closed position
        piper.GripperCtrl(int(gripper_closed), 1000, 0x01, 0)
        
        # Set initial joint positions to neutral from calibration
        piper.MotionCtrl_2(0x01, 0x01, 100, 0x00)
        piper.JointCtrl(*[int(pos) for pos in neutral_positions])
        
        print(f"PiperX {arm_name} arm prepared successfully with neutral positions")
        return True
    except Exception as e:
        print(f"ERROR preparing PiperX {arm_name} arm: {e}")
        return False

def connect_piperx(can_interface, arm_name):
    """Connect to a PiperX robot - fail fast if unable to connect"""
    try:
        print(f"Connecting to PiperX {arm_name} arm on {can_interface}...")
        piper = C_PiperInterface_V2(can_interface)
        piper.ConnectPort()
        
        # Wait for connection with timeout
        timeout = 5.0
        start_time = time.time()
        while not piper.get_connect_status():
            if time.time() - start_time > timeout:
                raise RuntimeError(f"Connection timeout for {arm_name} arm on {can_interface}")
            time.sleep(0.01)
        
        # Enable the robot
        print(f"Enabling PiperX {arm_name} arm...")
        start_time = time.time()
        while not piper.EnablePiper():
            if time.time() - start_time > timeout:
                raise RuntimeError(f"Enable timeout for {arm_name} arm on {can_interface}")
            time.sleep(0.01)
        
        print(f"PiperX {arm_name} arm on {can_interface} ready for teleoperation")
        return piper
        
    except Exception as e:
        print(f"FATAL ERROR: Failed to connect to PiperX {arm_name} arm on {can_interface}: {e}")
        sys.exit(1)

def emergency_shutdown(puppet_left, puppet_right, stepper, base, neutral_positions, gripper_closed):
    """Emergency shutdown procedure for all hardware"""
    print("Executing emergency shutdown...")
    try:
        # Stop all movement immediately
        stepper.stop()
        base.stop()
        
        # Move arms to neutral position
        puppet_left.JointCtrl(*[int(pos) for pos in neutral_positions])
        puppet_left.GripperCtrl(int(gripper_closed), 1000, 0x01, 0)
        
        puppet_right.JointCtrl(*[int(pos) for pos in neutral_positions])
        puppet_right.GripperCtrl(int(gripper_closed), 1000, 0x01, 0)
        
        time.sleep(1.0)
        print("Emergency shutdown completed")
    except Exception as e:
        print(f"Error during emergency shutdown: {e}")

def print_system_status(puppet_left, puppet_right):
    """Print real-time system status"""
    left_status = "OK" if puppet_left.get_connect_status() else "DISCONNECTED"
    right_status = "OK" if puppet_right.get_connect_status() else "DISCONNECTED"
    print(f"[STATUS] Left arm: {left_status}, Right arm: {right_status}")

def main():
    parser = argparse.ArgumentParser(description='PiperX Dual-Arm Puppet Robot Control (Full Hardware)')
    parser.add_argument('--can-left', default=CAN_LEFT, help=f'CAN interface for left arm (default: {CAN_LEFT})')
    parser.add_argument('--can-right', default=CAN_RIGHT, help=f'CAN interface for right arm (default: {CAN_RIGHT})')
    parser.add_argument('--stepper-port', default='/dev/tty_stepper', help='Stepper motor serial port')
    parser.add_argument('--base-port', default='/dev/tty_base', help='Base robot serial port')
    parser.add_argument('--smoothing-alpha', type=float, default=SMOOTHING_ALPHA, 
                       help=f'Smoothing factor (0.0-1.0, default: {SMOOTHING_ALPHA})')
    args = parser.parse_args()
    
    # Validate smoothing alpha
    if not 0.0 <= args.smoothing_alpha <= 1.0:
        print(f"ERROR: --smoothing-alpha must be between 0.0 and 1.0, got {args.smoothing_alpha}")
        sys.exit(1)
    
    print("Initializing PiperX Dual-Arm Puppet Robot (Full Hardware Mode)...")
    print("Hardware requirements: 2 PiperX arms, stepper motor, base robot")
    print(f"Configuration: Left={args.can_left}, Right={args.can_right}")
    print(f"Stepper port: {args.stepper_port}, Base port: {args.base_port}")
    print(f"Smoothing alpha: {args.smoothing_alpha}")
    
    # Initialize both PiperX robots - fail fast if either fails
    puppet_left = connect_piperx(args.can_left, "left")
    puppet_right = connect_piperx(args.can_right, "right")
    
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
        gripper_closed = piperx_calib['gripper_limits'][0]
        print(f"Loaded neutral positions: {neutral_positions}")
        print(f"Gripper closed position: {gripper_closed}")
    except Exception as e:
        print(f"FATAL ERROR loading PiperX calibration: {e}")
        sys.exit(1)
    
    # Prepare both robots with neutral positions
    if not prep_piperx_robot(puppet_left, neutral_positions, gripper_closed, "left"):
        print("FATAL ERROR: Failed to prepare left PiperX robot")
        sys.exit(1)
    
    if not prep_piperx_robot(puppet_right, neutral_positions, gripper_closed, "right"):
        print("FATAL ERROR: Failed to prepare right PiperX robot")
        sys.exit(1)
    
    # Initialize stepper controller - mandatory hardware
    try:
        stepper = StepperController(port=args.stepper_port)
        stepper.enable()
        print(f"Stepper controller initialized on {args.stepper_port}")
    except Exception as e:
        print(f"FATAL ERROR initializing stepper: {e}")
        sys.exit(1)
    
    # Initialize base controller - mandatory hardware
    try:
        base = MDCRobot(port=args.base_port, default_speed=BASE_SPEED)
        base.connect()
        print(f"Base controller initialized on {args.base_port}")
    except Exception as e:
        print(f"FATAL ERROR initializing base: {e}")
        sys.exit(1)
    
    # Initialize joint position storage - always dual-arm
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
                
                # Validate dual-arm data
                if jl is None or jr is None or gl is None or gr is None:
                    print("[INIT] ERROR: Missing dual-arm data from master")
                    continue
                
                # Map initial positions using the mapper
                print("[INIT] Mapping initial positions for both arms...")
                mapped_jl = mapper.map_joints(jl)
                mapped_jr = mapper.map_joints(jr)
                mapped_gl = mapper.map_gripper(gl)
                mapped_gr = mapper.map_gripper(gr)

                print("[INIT] Moving both arms to mapped initial positions...")
                
                # Convert to degrees and send joint commands (PiperX expects degrees * 1000)
                left_joints_deg = [angle * 180.0 / np.pi * 1000 for angle in mapped_jl]
                right_joints_deg = [angle * 180.0 / np.pi * 1000 for angle in mapped_jr]
                
                # Set motion control parameters and send initial joint positions
                puppet_left.MotionCtrl_2(0x01, 0x01, 100, 0x00)
                puppet_left.JointCtrl(*[int(angle) for angle in left_joints_deg])
                
                puppet_right.MotionCtrl_2(0x01, 0x01, 100, 0x00)
                puppet_right.JointCtrl(*[int(angle) for angle in right_joints_deg])
                
                # Send initial gripper positions
                left_gripper_deg = int(mapped_gl * 180.0 / np.pi * 1000)
                right_gripper_deg = int(mapped_gr * 180.0 / np.pi * 1000)
                puppet_left.GripperCtrl(gripper_angle=left_gripper_deg, gripper_effort=1000, gripper_code=0x01)
                puppet_right.GripperCtrl(gripper_angle=right_gripper_deg, gripper_effort=1000, gripper_code=0x01)
                
                # Update smoothed positions
                smoothed_joint_left = mapped_jl.copy()
                smoothed_joint_right = mapped_jr.copy()
                smoothed_gripper_left = mapped_gl
                smoothed_gripper_right = mapped_gr
                
                # Send confirmation
                confirm_packet = {'mode': 'init_ack'}
                sock.sendto(pickle.dumps(confirm_packet), addr)
                print("[INIT] Dual-arm initialization complete. Starting teleoperation...")
                break
                
        except socket.timeout:
            continue
        except Exception as e:
            print(f"[INIT] Error during initialization: {e}")
            continue

    status_counter = 0
    try:
        print("Starting dual-arm UDP teleoperation with full hardware control...")
        while True:
            loop_start = time.time()
            
            try:
                data, _ = sock.recvfrom(4096)
                packed = pickle.loads(data)
                
                # Only accept valid teleop packets from same session
                if (('heartbeat' not in packed and not packed.get('no_heartbeat', False)) or 
                    packed.get('session_id') != session_id):
                    continue
                    
                jl, jr, gl, gr, key_states = unpack_data_from_udp(packed)
                
                # Validate dual-arm data - both arms required
                if jl is None or jr is None or gl is None or gr is None:
                    continue
                
                # Map incoming data using the mapper
                mapped_jl = mapper.map_joints(jl)
                mapped_jr = mapper.map_joints(jr)
                
                # Update joint commands
                for i in range(6):
                    joint_commands_left[i] = mapped_jl[i]
                    joint_commands_right[i] = mapped_jr[i]
                
                target_gripper_left = mapper.map_gripper(gl)
                target_gripper_right = mapper.map_gripper(gr)

                if key_states is not None:
                    most_recent_key_states = key_states
                    
            except socket.timeout:
                pass
            
            # Always smooth toward the most recent target at every tick
            for i in range(6):
                smoothed_joint_left[i] = args.smoothing_alpha * joint_commands_left[i] + (1 - args.smoothing_alpha) * smoothed_joint_left[i]
                smoothed_joint_right[i] = args.smoothing_alpha * joint_commands_right[i] + (1 - args.smoothing_alpha) * smoothed_joint_right[i]
            
            smoothed_gripper_left = args.smoothing_alpha * target_gripper_left + (1 - args.smoothing_alpha) * smoothed_gripper_left
            smoothed_gripper_right = args.smoothing_alpha * target_gripper_right + (1 - args.smoothing_alpha) * smoothed_gripper_right

            # Send commands to both robots
            # Convert radians to degrees * 1000 for PiperX
            left_joints_deg = [angle * 180.0 / np.pi * 1000 for angle in smoothed_joint_left]
            right_joints_deg = [angle * 180.0 / np.pi * 1000 for angle in smoothed_joint_right]
            
            # Set motion control parameters and send joint commands
            puppet_left.MotionCtrl_2(0x01, 0x01, 100, 0x00)
            puppet_left.JointCtrl(*[int(angle) for angle in left_joints_deg])
            
            puppet_right.MotionCtrl_2(0x01, 0x01, 100, 0x00)
            puppet_right.JointCtrl(*[int(angle) for angle in right_joints_deg])
            
            # Send gripper commands
            left_gripper_deg = int(smoothed_gripper_left * 180.0 / np.pi * 1000)
            right_gripper_deg = int(smoothed_gripper_right * 180.0 / np.pi * 1000)
            puppet_left.GripperCtrl(gripper_angle=left_gripper_deg, gripper_effort=1000, gripper_code=0x01)
            puppet_right.GripperCtrl(gripper_angle=right_gripper_deg, gripper_effort=1000, gripper_code=0x01)
            
            # Handle key-based controls for stepper and base
            if most_recent_key_states != last_key_states:
                last_key_states = process_key_states_full(most_recent_key_states, stepper, base, last_key_states)

            # Periodic hardware health check
            status_counter += 1
            if status_counter % 250 == 0:  # Every 5 seconds at 50Hz
                check_hardware_health(puppet_left, puppet_right)
                status_counter = 0

            # Dynamic sleep to maintain control frequency
            elapsed = time.time() - loop_start
            sleep_time = max(0, CONTROL_PERIOD - elapsed)
            time.sleep(sleep_time)
            
    except KeyboardInterrupt:
        print("\nTeleoperation stopped by user.")
    except Exception as e:
        print(f"\nFATAL ERROR during teleoperation: {e}")
    finally:
        emergency_shutdown(puppet_left, puppet_right, stepper, base, neutral_positions, gripper_closed)
        print("Dual-arm puppet control terminated.")

if __name__ == '__main__':
    main()