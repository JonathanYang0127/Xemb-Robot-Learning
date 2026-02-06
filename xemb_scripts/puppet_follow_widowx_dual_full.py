#!/usr/bin/env python3
"""
WidowX Puppet Robot Control Script (Dedicated Dual-Arm + Full Hardware)
Requires: 2 WidowX puppet arms, stepper motor, and base robot
No optional components - all hardware must be present
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

# Required imports - fail fast if not available
try:
    from stepper_control.stepper_controller import StepperController
except ImportError:
    print("ERROR: stepper_control not available. Hardware required for dual-full mode.")
    sys.exit(1)

try:
    from base_control.base_controller import MDCRobot
except ImportError:
    print("ERROR: base_control not available. Hardware required for dual-full mode.")
    sys.exit(1)

# Hardcoded configuration for optimal performance
PUPPET_LEFT_NAME = "puppet_left"
PUPPET_RIGHT_NAME = "puppet_right"
UDP_PORT = 5005
BASE_SPEED = 1
CONTROL_PERIOD = 0.02  # 50Hz control loop
SMOOTHING_ALPHA = 0.8  # More responsive for full system
STEPPER_LARGE_STEP = 100
STEPPER_FINE_STEP = 10

# Joint limits for safety (can be customized per robot)
JOINT_LIMITS = [[-10, 10] for _ in range(6)]

def enforce_joint_limits(joint_positions, limits):
    """Enforce joint limits on the given joint positions."""
    limited_positions = []
    all_within_limits = True
    
    for i, pos in enumerate(joint_positions):
        min_limit, max_limit = limits[i]
        
        if pos < min_limit or pos > max_limit:
            all_within_limits = False
            limited_pos = max(min_limit, min(max_limit, pos))
            print(f"Joint {i} limited from {pos:.2f} to {limited_pos:.2f}")
        else:
            limited_pos = pos
            
        limited_positions.append(limited_pos)
    
    return limited_positions, all_within_limits

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

def prep_puppet_robot(puppet_bot, arm_type):
    """Prepare the puppet robot for teleoperation"""
    puppet_bot.dxl.robot_set_operating_modes("group", "arm", "position")
    puppet_bot.dxl.robot_set_operating_modes("single", "gripper", "current_based_position")
    
    torque_on(puppet_bot)
    
    # Move arm to starting position based on calibration
    try:
        # Try to load calibration neutral positions
        import json
        with open('piperx_control/widowx_calibration.json', 'r') as f:
            widowx_calib = json.load(f)
        start_arm_qpos = widowx_calib['neutral_positions'][:6]
        gripper_closed = widowx_calib['gripper_limits'][0]
        print(f"Using calibrated {arm_type} arm neutral position: {start_arm_qpos}")
    except Exception as e:
        print(f"Warning: Could not load calibration for {arm_type} arm: {e}")
        # Fallback to default positions
        if arm_type == 'left':
            start_arm_qpos = START_ARM_POSE_LEFT[:6]
        elif arm_type == 'right':
            start_arm_qpos = START_ARM_POSE_RIGHT[:6]
        else:
            start_arm_qpos = START_ARM_POSE[:6]
        gripper_closed = PUPPET_GRIPPER_JOINT_CLOSE
    
    move_arms([puppet_bot], [start_arm_qpos], move_time=1)
    move_grippers([puppet_bot], [gripper_closed], move_time=0.5)
    
    return start_arm_qpos, gripper_closed

def connect_widowx(robot_name, arm_type):
    """Connect to a WidowX robot - fail fast if unable to connect"""
    try:
        print(f"Connecting to WidowX {arm_type} puppet arm ({robot_name})...")
        puppet = InterbotixManipulatorXS(
            robot_model="wx250s",
            group_name="arm",
            gripper_name="gripper",
            robot_name=robot_name,
            init_node=False  # ROS already initialized
        )
        
        # Validate connection
        positions = puppet.dxl.joint_states.position
        if len(positions) < 7:  # 6 joints + gripper
            raise RuntimeError(f"{arm_type} arm has insufficient joints: {len(positions)}")
        
        print(f"WidowX {arm_type} puppet arm connected: {len(positions)} joints detected")
        return puppet
        
    except Exception as e:
        print(f"FATAL ERROR: Failed to connect to WidowX {arm_type} puppet arm: {e}")
        sys.exit(1)

def check_robot_health(puppet_left, puppet_right):
    """Monitor robot connections and fail if issues detected"""
    try:
        left_pos = puppet_left.dxl.joint_states.position
        right_pos = puppet_right.dxl.joint_states.position
        
        if len(left_pos) < 7 or len(right_pos) < 7:
            raise RuntimeError("Lost joint position data from one or both puppet arms")
            
        return True
    except Exception as e:
        print(f"ROBOT HEALTH ERROR: {e}")
        raise e

def emergency_shutdown(puppet_left, puppet_right, stepper, base, neutral_left, neutral_right, gripper_closed):
    """Emergency shutdown procedure for all hardware"""
    print("Executing emergency shutdown...")
    try:
        # Stop all movement immediately
        stepper.stop()
        base.stop()
        
        # Move arms to neutral positions
        move_arms([puppet_left], [neutral_left], move_time=0.5)
        move_arms([puppet_right], [neutral_right], move_time=0.5)
        move_grippers([puppet_left, puppet_right], [gripper_closed, gripper_closed], move_time=0.5)
        
        time.sleep(1.0)
        print("Emergency shutdown completed")
    except Exception as e:
        print(f"Error during emergency shutdown: {e}")

def main():
    parser = argparse.ArgumentParser(description='WidowX Dual-Arm Puppet Robot Control (Full Hardware)')
    parser.add_argument('--stepper-port', default='/dev/tty_stepper', help='Stepper motor serial port')
    parser.add_argument('--base-port', default='/dev/tty_base', help='Base robot serial port')
    parser.add_argument('--smoothing-alpha', type=float, default=SMOOTHING_ALPHA, 
                       help=f'Smoothing factor (0.0-1.0, default: {SMOOTHING_ALPHA})')
    args = parser.parse_args()
    
    # Validate smoothing alpha
    if not 0.0 <= args.smoothing_alpha <= 1.0:
        print(f"ERROR: --smoothing-alpha must be between 0.0 and 1.0, got {args.smoothing_alpha}")
        sys.exit(1)
    
    print("Initializing WidowX Dual-Arm Puppet Robot (Full Hardware Mode)...")
    print("Hardware requirements: 2 WidowX puppet arms, stepper motor, base robot")
    print(f"Stepper port: {args.stepper_port}, Base port: {args.base_port}")
    print(f"Smoothing alpha: {args.smoothing_alpha}")
    
    # Initialize ROS node
    rospy.init_node('widowx_puppet_dual_full', anonymous=True)
    
    # Initialize both WidowX puppet robots - fail fast if either fails
    puppet_left = connect_widowx(PUPPET_LEFT_NAME, "left")
    puppet_right = connect_widowx(PUPPET_RIGHT_NAME, "right")
    
    # Prepare both robots and get neutral positions
    neutral_left, gripper_closed_left = prep_puppet_robot(puppet_left, 'left')
    neutral_right, gripper_closed_right = prep_puppet_robot(puppet_right, 'right')
    
    # Use the same gripper closed position for both arms
    gripper_closed = gripper_closed_left
    
    # Initialize stepper controller - mandatory hardware
    try:
        stepper = StepperController(port=args.stepper_port)
        stepper.enable()
        print(f"Stepper controller initialized on {args.stepper_port}")
    except Exception as e:
        print(f"FATAL ERROR initializing stepper: {e}")
        puppet_left.shutdown()
        puppet_right.shutdown()
        sys.exit(1)
    
    # Initialize base controller - mandatory hardware
    try:
        base = MDCRobot(port=args.base_port, default_speed=BASE_SPEED)
        base.connect()
        print(f"Base controller initialized on {args.base_port}")
    except Exception as e:
        print(f"FATAL ERROR initializing base: {e}")
        stepper.close()
        puppet_left.shutdown()
        puppet_right.shutdown()
        sys.exit(1)
    
    # Initialize joint position storage - always dual-arm
    smoothed_joint_left = neutral_left.copy()
    smoothed_joint_right = neutral_right.copy()
    smoothed_gripper_left = gripper_closed
    smoothed_gripper_right = gripper_closed
    target_gripper_left = gripper_closed
    target_gripper_right = gripper_closed
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
                
                print("[INIT] Moving both arms to initial positions...")
                
                # Apply joint limits for safety
                jl_limited, _ = enforce_joint_limits(jl, JOINT_LIMITS)
                jr_limited, _ = enforce_joint_limits(jr, JOINT_LIMITS)
                
                # Move both arms to initial positions
                move_arms([puppet_left], [jl_limited], move_time=1.0)
                move_arms([puppet_right], [jr_limited], move_time=1.0)
                move_grippers([puppet_left, puppet_right], [gl, gr], move_time=0.5)
                
                # Update smoothed positions
                smoothed_joint_left = jl_limited.copy()
                smoothed_joint_right = jr_limited.copy()
                smoothed_gripper_left = gl
                smoothed_gripper_right = gr
                
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
                
                # Apply joint limits for safety
                jl_limited, jl_safe = enforce_joint_limits(jl, JOINT_LIMITS)
                jr_limited, jr_safe = enforce_joint_limits(jr, JOINT_LIMITS)
                
                if not jl_safe or not jr_safe:
                    print("WARNING: Joint limits applied for safety")
                
                # Update target positions
                smoothed_joint_left = [(args.smoothing_alpha * target + (1 - args.smoothing_alpha) * current) 
                                     for target, current in zip(jl_limited, smoothed_joint_left)]
                smoothed_joint_right = [(args.smoothing_alpha * target + (1 - args.smoothing_alpha) * current) 
                                      for target, current in zip(jr_limited, smoothed_joint_right)]
                
                target_gripper_left = gl
                target_gripper_right = gr
                smoothed_gripper_left = args.smoothing_alpha * target_gripper_left + (1 - args.smoothing_alpha) * smoothed_gripper_left
                smoothed_gripper_right = args.smoothing_alpha * target_gripper_right + (1 - args.smoothing_alpha) * smoothed_gripper_right

                if key_states is not None:
                    most_recent_key_states = key_states
                    
            except socket.timeout:
                pass
            
            # Send commands to both robots
            try:
                # Create joint command messages
                left_joint_cmd = JointSingleCommand()
                left_joint_cmd.joint_positions = smoothed_joint_left
                
                right_joint_cmd = JointSingleCommand()
                right_joint_cmd.joint_positions = smoothed_joint_right
                
                # Send joint commands
                puppet_left.dxl.joint_states_commands.position = smoothed_joint_left
                puppet_right.dxl.joint_states_commands.position = smoothed_joint_right
                
                # Send gripper commands
                puppet_left.dxl.gripper_states_commands.position = smoothed_gripper_left
                puppet_right.dxl.gripper_states_commands.position = smoothed_gripper_right
                
            except Exception as e:
                print(f"Error sending commands to robots: {e}")
            
            # Handle key-based controls for stepper and base
            if most_recent_key_states != last_key_states:
                last_key_states = process_key_states_full(most_recent_key_states, stepper, base, last_key_states)

            # Periodic hardware health check
            status_counter += 1
            if status_counter % 250 == 0:  # Every 5 seconds at 50Hz
                try:
                    check_robot_health(puppet_left, puppet_right)
                except Exception as e:
                    print(f"FATAL: Robot health check failed: {e}")
                    break
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
        emergency_shutdown(puppet_left, puppet_right, stepper, base, 
                         neutral_left, neutral_right, gripper_closed)
        
        # Cleanup
        try:
            stepper.close()
            base.stop()
            puppet_left.shutdown()
            puppet_right.shutdown()
        except Exception as e:
            print(f"Error during cleanup: {e}")
        
        print("Dual-arm WidowX puppet control terminated.")

if __name__ == '__main__':
    main()