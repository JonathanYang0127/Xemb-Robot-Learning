#!/usr/bin/env python3
"""
WidowX Master Controller (Dedicated Dual-Arm + Full Hardware)
Requires: 2 WidowX master arms for controlling 2 PiperX puppets + stepper + base
No optional components - all hardware must be present
"""

import socket
import time
import pickle
import sys
import os
import json
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from pynput import keyboard
import argparse
import uuid

# Import from piperx_control directory
from constants import *
from packing_utils_widowx_piperx import pack_data_for_udp
from robot_utils import torque_on, torque_off, move_arms, move_grippers, get_arm_gripper_positions

# Hardcoded configuration for optimal performance
PUPPET_IP = "100.105.116.25"  # Default puppet Tailscale IP
UDP_PORT = 5005
SLEEP_TIME = 1/30  # 30Hz master control
MASTER_LEFT_NAME = "master_left"
MASTER_RIGHT_NAME = "master_right"

class KeyboardHandler:
    def __init__(self):
        self.key_states = {
            'w': False, 'a': False, 's': False, 'd': False,
            'up': False, 'left': False, 'down': False, 'right': False
        }
        self.running = True
        self.listener = keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release
        )
        self.listener.start()
        print("Keyboard listener started. W/A/S/D and arrow keys are now active. Press ESC to quit.")
    
    def _on_press(self, key):
        try:
            key_char = key.char.lower()
            if key_char in 'wasd':
                self.key_states[key_char] = True
        except (AttributeError, TypeError):
            if key == keyboard.Key.up:
                self.key_states['up'] = True
            elif key == keyboard.Key.down:
                self.key_states['down'] = True
            elif key == keyboard.Key.left:
                self.key_states['left'] = True
            elif key == keyboard.Key.right:
                self.key_states['right'] = True
            elif key == keyboard.Key.esc:
                self.running = False
                print("ESC pressed, exiting...")
                return False
    
    def _on_release(self, key):
        try:
            key_char = key.char.lower()
            if key_char in 'wasd':
                self.key_states[key_char] = False
        except (AttributeError, TypeError):
            if key == keyboard.Key.up:
                self.key_states['up'] = False
            elif key == keyboard.Key.down:
                self.key_states['down'] = False
            elif key == keyboard.Key.left:
                self.key_states['left'] = False
            elif key == keyboard.Key.right:
                self.key_states['right'] = False
    
    def shutdown(self):
        self.listener.stop()
        self.running = False

def load_widowx_calibration(calib_path='widowx_calibration.json'):
    """Load WidowX calibration data from JSON file"""
    try:
        with open(calib_path, 'r') as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError) as e:
        print(f"FATAL ERROR loading calibration file {calib_path}: {e}")
        sys.exit(1)

def prep_master_robot(master_bot, arm_type='left', skip_move_and_torque=False):
    """Prepare the master robot for teleoperation"""
    if not skip_move_and_torque:
        master_bot.dxl.robot_set_operating_modes("group", "arm", "position")
        master_bot.dxl.robot_set_operating_modes("single", "gripper", "position")
        
        # Load calibration data and use neutral positions
        calib_data = load_widowx_calibration()
        neutral_positions = calib_data['neutral_positions']
        
        # Use neutral positions from calibration (first 6 joints)
        start_arm_qpos = neutral_positions[:6]
        print(f"Moving {arm_type} arm to neutral position: {start_arm_qpos}")
        
        move_arms([master_bot], [start_arm_qpos], move_time=1)
        move_grippers([master_bot], [MASTER_GRIPPER_JOINT_OPEN], move_time=0.5)
    else:
        # Just open gripper
        move_grippers([master_bot], [MASTER_GRIPPER_JOINT_OPEN], move_time=0.5)

def press_to_start_dual_arm(master_left, master_right):
    """Wait for user to close both grippers to start teleoperation"""
    # Torque off grippers to allow manual closure
    master_left.dxl.robot_torque_enable("single", "gripper", False)
    master_right.dxl.robot_torque_enable("single", "gripper", False)
    
    print("Close BOTH grippers to start dual-arm teleoperation")
    while True:
        left_gripper_pos = master_left.dxl.joint_states.position[6]
        right_gripper_pos = master_right.dxl.joint_states.position[6]
        threshold = (MASTER_GRIPPER_JOINT_OPEN + MASTER_GRIPPER_JOINT_CLOSE) / 2
        if left_gripper_pos < threshold and right_gripper_pos < threshold:
            break
        time.sleep(0.1)
    print("Both grippers closed, starting dual-arm teleoperation")
    
    # Torque off entire robot after gripper closure
    torque_off(master_left)
    torque_off(master_right)

def create_dual_arm_init_packet(session_id, master_left, master_right):
    """Create initialization packet for dual-arm puppet"""
    init_packet = {
        'mode': 'init',
        'init_session_id': session_id,
        'single_arm': False,  # Always dual-arm
        'arm_side': 'dual'    # Both arms
    }
    
    # Always send both arm data
    init_packet['jl'] = list(master_left.dxl.joint_states.position[:6])
    init_packet['jr'] = list(master_right.dxl.joint_states.position[:6])
    init_packet['gl'] = master_left.dxl.joint_states.position[6]
    init_packet['gr'] = master_right.dxl.joint_states.position[6]
    
    return init_packet

def validate_robot_connection(master_bot, arm_name):
    """Validate robot connection and fail fast if issues"""
    try:
        # Test basic communication
        positions = master_bot.dxl.joint_states.position
        if len(positions) < 7:  # 6 joints + gripper
            raise RuntimeError(f"{arm_name} arm has insufficient joints: {len(positions)}")
        print(f"{arm_name} arm validation passed: {len(positions)} joints detected")
        return True
    except Exception as e:
        print(f"FATAL ERROR: {arm_name} arm validation failed: {e}")
        sys.exit(1)

def monitor_arm_health(master_left, master_right):
    """Monitor both arms for health issues"""
    try:
        # Check for valid joint readings
        left_pos = master_left.dxl.joint_states.position
        right_pos = master_right.dxl.joint_states.position
        
        if len(left_pos) < 7 or len(right_pos) < 7:
            raise RuntimeError("Lost joint position data from one or both arms")
            
        return True
    except Exception as e:
        print(f"ARM HEALTH WARNING: {e}")
        return False

def main():
    parser = argparse.ArgumentParser(description='WidowX Dual-Arm Master Controller (Full Hardware)')
    parser.add_argument('--puppet-ip', default=PUPPET_IP, help=f'IP address of the puppet robot (default: {PUPPET_IP})')
    parser.add_argument('--no-move-left', action='store_true', help='Do not move or torque on the left arm at startup')
    parser.add_argument('--no-move-right', action='store_true', help='Do not move or torque on the right arm at startup')
    parser.add_argument('--gripper-lock-left', action='store_true', help='Use left gripper to lock/unlock left arm during teleop')
    parser.add_argument('--gripper-lock-right', action='store_true', help='Use right gripper to lock/unlock right arm during teleop')
    parser.add_argument('--no-heartbeat', action='store_true', help='Disable heartbeat system and send all joint data in every packet for higher frequency')
    args = parser.parse_args()

    # Use the puppet IP from arguments
    puppet_ip = args.puppet_ip

    # Generate a session_id for this run
    session_id = uuid.uuid4().hex

    print("Initializing WidowX Dual-Arm Master Controller (Full Hardware Mode)...")
    print(f"Puppet IP: {puppet_ip}")
    print(f"Hardware requirements: 2 WidowX master arms")
    print("Control scheme: Dual-arm mandatory, no single-arm fallback")

    # Initialize both master arms - both are mandatory
    print("Connecting to left master arm...")
    try:
        master_left = InterbotixManipulatorXS(
            robot_model="wx250s", 
            group_name="arm", 
            gripper_name="gripper", 
            robot_name=MASTER_LEFT_NAME, 
            init_node=True
        )
        validate_robot_connection(master_left, "Left")
    except Exception as e:
        print(f"FATAL ERROR connecting to left master arm: {e}")
        sys.exit(1)
    
    print("Connecting to right master arm...")
    try:
        master_right = InterbotixManipulatorXS(
            robot_model="wx250s", 
            group_name="arm", 
            gripper_name="gripper", 
            robot_name=MASTER_RIGHT_NAME, 
            init_node=False  # ROS already initialized
        )
        validate_robot_connection(master_right, "Right")
    except Exception as e:
        print(f"FATAL ERROR connecting to right master arm: {e}")
        master_left.shutdown()
        sys.exit(1)
    
    # Prepare both robots - both are mandatory
    try:
        torque_on(master_left)
        torque_on(master_right)
        prep_master_robot(master_left, arm_type='left', skip_move_and_torque=args.no_move_left)
        prep_master_robot(master_right, arm_type='right', skip_move_and_torque=args.no_move_right)
        print("Both master arms prepared successfully")
    except Exception as e:
        print(f"FATAL ERROR preparing master arms: {e}")
        master_left.shutdown()
        master_right.shutdown()
        sys.exit(1)

    # Send initial positions to puppet and wait for confirmation
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    init_packet = create_dual_arm_init_packet(session_id, master_left, master_right)
    data = pickle.dumps(init_packet)
    confirmed = False
    sock.settimeout(0.5)
    print("[INIT] Sending dual-arm initial positions to puppet and waiting for confirmation...")
    
    retry_count = 0
    max_retries = 20
    while not confirmed and retry_count < max_retries:
        sock.sendto(data, (puppet_ip, UDP_PORT))
        try:
            resp, _ = sock.recvfrom(1024)
            resp_packet = pickle.loads(resp)
            if resp_packet.get('mode') == 'init_ack':
                confirmed = True
                print("[INIT] Puppet confirmed dual-arm initial pose.")
        except socket.timeout:
            retry_count += 1
            print(f"[INIT] No confirmation yet, retrying ({retry_count}/{max_retries})...")
        time.sleep(0.5)
    
    if not confirmed:
        print("FATAL ERROR: Could not establish connection with puppet robot")
        master_left.shutdown()
        master_right.shutdown()
        sys.exit(1)

    # Wait for user to start teleoperation
    press_to_start_dual_arm(master_left, master_right)

    # Start UDP sending loop
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    heartbeat = 0
    keyboard_handler = KeyboardHandler()
    
    # Store last sent positions for lock functionality
    last_jl = list(master_left.dxl.joint_states.position[:6])
    last_jr = list(master_right.dxl.joint_states.position[:6])
    
    # Track previous lock state
    prev_locked_left = None
    prev_locked_right = None
    
    health_check_counter = 0
    
    try:
        print("Starting dual-arm teleoperation with full hardware support...")
        while keyboard_handler.running:
            loop_start = time.time()
            
            # Get current joint positions from both arms
            jl_to_send = list(master_left.dxl.joint_states.position[:6])
            jr_to_send = list(master_right.dxl.joint_states.position[:6])
            gl = master_left.dxl.joint_states.position[6]
            gr = master_right.dxl.joint_states.position[6]
            
            # Left arm gripper lock handling
            if args.gripper_lock_left:
                gl_actual = master_left.dxl.joint_states.position[6]
                gripper_left_open = gl_actual > (MASTER_GRIPPER_JOINT_MID + MASTER_GRIPPER_JOINT_CLOSE) / 2
                locked_left = not gripper_left_open
                if prev_locked_left != locked_left:
                    if locked_left:
                        torque_on(master_left)
                        master_left.dxl.robot_torque_enable("single", "gripper", False)
                        print("Left arm LOCKED")
                    else:
                        torque_off(master_left)
                        master_left.dxl.robot_torque_enable("single", "gripper", False)
                        print("Left arm UNLOCKED")
                    prev_locked_left = locked_left
                if locked_left:
                    jl_to_send = last_jl
                    gl = MASTER_GRIPPER_JOINT_CLOSE
                else:
                    last_jl = jl_to_send
                    
            # Right arm gripper lock handling
            if args.gripper_lock_right:
                gr_actual = master_right.dxl.joint_states.position[6]
                gripper_right_open = gr_actual > (MASTER_GRIPPER_JOINT_MID + MASTER_GRIPPER_JOINT_CLOSE) / 2
                locked_right = not gripper_right_open
                if prev_locked_right != locked_right:
                    if locked_right:
                        torque_on(master_right)
                        master_right.dxl.robot_torque_enable("single", "gripper", False)
                        print("Right arm LOCKED")
                    else:
                        torque_off(master_right)
                        master_right.dxl.robot_torque_enable("single", "gripper", False)
                        print("Right arm UNLOCKED")
                    prev_locked_right = locked_right
                if locked_right:
                    jr_to_send = last_jr
                    gr = MASTER_GRIPPER_JOINT_CLOSE
                else:
                    last_jr = jr_to_send
            
            # Pack and send data - always dual-arm
            packed = pack_data_for_udp(jl_to_send, jr_to_send, gl, gr, heartbeat, keyboard_handler.key_states, no_heartbeat=args.no_heartbeat)
            packed['session_id'] = session_id
            packed['single_arm'] = False  # Always dual-arm
            packed['arm_side'] = 'dual'
            
            data = pickle.dumps(packed)
            sock.sendto(data, (puppet_ip, UDP_PORT))
            
            # Only increment heartbeat counter if not in no-heartbeat mode
            if not args.no_heartbeat:
                heartbeat = (heartbeat + 1) % 4
            
            # Print active keys for debugging
            active_keys = [k for k, v in keyboard_handler.key_states.items() if v]
            if active_keys:
                print(f"Active keys: {active_keys}")
            
            # Periodic health monitoring
            health_check_counter += 1
            if health_check_counter % 300 == 0:  # Every 10 seconds at 30Hz
                if not monitor_arm_health(master_left, master_right):
                    print("WARNING: Arm health check failed")
                health_check_counter = 0
            
            # Maintain control frequency
            elapsed = time.time() - loop_start
            sleep_time = max(0, SLEEP_TIME - elapsed)
            time.sleep(sleep_time)
            
    except KeyboardInterrupt:
        print("Dual-arm teleoperation stopped by keyboard interrupt.")
    except Exception as e:
        print(f"FATAL ERROR during teleoperation: {e}")
    finally:
        print("Shutting down dual-arm master controllers...")
        keyboard_handler.shutdown()
        try:
            master_left.shutdown()
            master_right.shutdown()
        except Exception as e:
            print(f"Error during shutdown: {e}")
        sock.close()
        print("Dual-arm master cleanup complete.")

if __name__ == '__main__':
    main()