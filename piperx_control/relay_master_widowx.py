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

PUPPET_IP = "100.105.116.25"  # <-- Set puppet's Tailscale IP here
UDP_PORT = 5005
SLEEP_TIME = 1/30

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
        self.listener.join()
        print("Keyboard listener stopped")

def load_widowx_calibration(calib_path='widowx_calibration.json'):
    """Load WidowX calibration data from JSON file"""
    try:
        with open(calib_path, 'r') as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError) as e:
        print(f"Error loading calibration file {calib_path}: {e}")
        print("Using default neutral positions")
        # Fallback to default neutral positions
        return {
            'neutral_positions': [0, -0.96, 1.16, 0, -0.3, 0]
        }

def prep_master_robot(master_bot, arm_type='left', skip_move_and_torque=False):
    """Prepare the master robot for teleoperation"""
    master_bot.dxl.robot_set_operating_modes("group", "arm", "position")
    master_bot.dxl.robot_set_operating_modes("single", "gripper", "position")
    
    if not skip_move_and_torque:
        torque_on(master_bot)
        
        # Load calibration data and use neutral positions
        calib_data = load_widowx_calibration()
        neutral_positions = calib_data['neutral_positions']
        
        # Use neutral positions from calibration (first 6 joints)
        start_arm_qpos = neutral_positions[:6]
        print(f"Moving {arm_type} arm to neutral position: {start_arm_qpos}")
        
        move_arms([master_bot], [start_arm_qpos], move_time=1)
        move_grippers([master_bot], [MASTER_GRIPPER_JOINT_MID], move_time=0.5)
    else:
        # Just open the gripper
        move_grippers([master_bot], [MASTER_GRIPPER_JOINT_MID], move_time=0.5)

def press_to_start(master_bots):
    print('Close both grippers to start')
    for bot in master_bots:
        bot.dxl.robot_torque_enable("single", "gripper", False)
    close_thresh = (MASTER_GRIPPER_JOINT_MID + 3 * MASTER_GRIPPER_JOINT_CLOSE) / 4
    pressed = [False, False]
    while not all(pressed):
        for i, bot in enumerate(master_bots):
            if not pressed[i]:
                gripper_pos = get_arm_gripper_positions(bot)
                if gripper_pos < close_thresh:
                    pressed[i] = True
                    print(f"{'Left' if i == 0 else 'Right'} gripper closed!")
        time.sleep(DT / 10)
    for bot in master_bots:
        torque_off(bot)
    print('Both grippers closed - Starting teleoperation!')
    # For no-move arms, ensure they start locked and torqued ON
    if args.no_move_left:
        torque_on(master_left)
        master_left.dxl.robot_torque_enable("single", "gripper", False)
    if args.no_move_right:
        torque_on(master_right)
        master_right.dxl.robot_torque_enable("single", "gripper", False)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--no-move-left', action='store_true', help='Do not move or torque on the left arm at startup')
    parser.add_argument('--no-move-right', action='store_true', help='Do not move or torque on the right arm at startup')
    parser.add_argument('--gripper-lock-left', action='store_true', help='Use left gripper to lock/unlock left arm during teleop')
    parser.add_argument('--gripper-lock-right', action='store_true', help='Use right gripper to lock/unlock right arm during teleop')
    args = parser.parse_args()

    # Generate a session_id for this run
    session_id = uuid.uuid4().hex

    # Initialize both master arms immediately
    master_left = InterbotixManipulatorXS(robot_model="wx250s", group_name="arm", gripper_name="gripper", robot_name='master_left', init_node=True)
    master_right = InterbotixManipulatorXS(robot_model="wx250s", group_name="arm", gripper_name="gripper", robot_name='master_right', init_node=False)

    # Prepare both robots (move/torque if not skipped, always open gripper)
    master_bots = {'left': master_left, 'right': master_right}
    prep_flags = {'left': args.no_move_left, 'right': args.no_move_right}
    for arm_type, bot in master_bots.items():
        torque_on(bot)  # Always torque on first
        prep_master_robot(bot, arm_type=arm_type, skip_move_and_torque=prep_flags[arm_type])
    master_bots = [master_left, master_right]

    # Send the actual initial positions to the puppet and wait for confirmation
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    jl = list(master_left.dxl.joint_states.position[:6])
    jr = list(master_right.dxl.joint_states.position[:6])
    gl = master_left.dxl.joint_states.position[6]
    gr = master_right.dxl.joint_states.position[6]
    init_packet = {
        'mode': 'init',
        'jl': jl,
        'jr': jr,
        'gl': gl,
        'gr': gr,
        'init_session_id': session_id
    }
    data = pickle.dumps(init_packet)
    confirmed = False
    sock.settimeout(0.5)
    print("[INIT] Sending initial positions to puppet and waiting for confirmation...")
    while not confirmed:
        sock.sendto(data, (PUPPET_IP, UDP_PORT))
        try:
            resp, _ = sock.recvfrom(1024)
            resp_packet = pickle.loads(resp)
            if resp_packet.get('mode') == 'init_ack':
                confirmed = True
                print("[INIT] Puppet confirmed initial pose.")
        except socket.timeout:
            print("[INIT] No confirmation yet, resending...")
        time.sleep(0.5)  # Reduce handshake sending frequency
    # Wait for the user to close both grippers to start
    press_to_start(master_bots)

    # Start UDP sending loop
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    heartbeat = 0
    keyboard_handler = KeyboardHandler()
    # Store last sent positions for lock functionality
    last_jl = list(master_left.dxl.joint_states.position[:6])
    last_jr = list(master_right.dxl.joint_states.position[:6])
    # Track previous lock state for no-move arms
    prev_locked_left = None
    prev_locked_right = None
    try:
        while keyboard_handler.running:
            # Left arm
            if args.gripper_lock_left:
                gl_actual = master_left.dxl.joint_states.position[6]
                gripper_left_open = gl_actual > (MASTER_GRIPPER_JOINT_MID + MASTER_GRIPPER_JOINT_CLOSE) / 2
                locked_left = not gripper_left_open
                if prev_locked_left != locked_left:
                    if locked_left:
                        torque_on(master_left)
                        master_left.dxl.robot_torque_enable("single", "gripper", False)
                    else:
                        torque_off(master_left)
                        master_left.dxl.robot_torque_enable("single", "gripper", False)
                    prev_locked_left = locked_left
                if locked_left:
                    jl = list(master_left.dxl.joint_states.position[:6])
                    last_jl = jl
                    jl_to_send = last_jl
                else:
                    jl = list(master_left.dxl.joint_states.position[:6])
                    jl_to_send = jl
                gl = MASTER_GRIPPER_JOINT_CLOSE  # Always send closed to puppet
            else:
                jl_to_send = list(master_left.dxl.joint_states.position[:6])
                gl = master_left.dxl.joint_states.position[6]
                last_jl = jl_to_send
            # Right arm
            if args.gripper_lock_right:
                gr_actual = master_right.dxl.joint_states.position[6]
                gripper_right_open = gr_actual > (MASTER_GRIPPER_JOINT_MID + MASTER_GRIPPER_JOINT_CLOSE) / 2
                locked_right = not gripper_right_open
                if prev_locked_right != locked_right:
                    if locked_right:
                        torque_on(master_right)
                        master_right.dxl.robot_torque_enable("single", "gripper", False)
                    else:
                        torque_off(master_right)
                        master_right.dxl.robot_torque_enable("single", "gripper", False)
                    prev_locked_right = locked_right
                if locked_right:
                    jr = list(master_right.dxl.joint_states.position[:6])
                    last_jr = jr
                    jr_to_send = last_jr
                else:
                    jr = list(master_right.dxl.joint_states.position[:6])
                    jr_to_send = jr
                gr = MASTER_GRIPPER_JOINT_CLOSE  # Always send closed to puppet
            else:
                jr_to_send = list(master_right.dxl.joint_states.position[:6])
                gr = master_right.dxl.joint_states.position[6]
                last_jr = jr_to_send
            packed = pack_data_for_udp(jl_to_send, jr_to_send, gl, gr, heartbeat, keyboard_handler.key_states)
            packed['session_id'] = session_id  # Add session_id to every teleop packet
            data = pickle.dumps(packed)
            sock.sendto(data, (PUPPET_IP, UDP_PORT))
            heartbeat = (heartbeat + 1) % 4
            # Print active keys for debugging
            active_keys = [k for k, v in keyboard_handler.key_states.items() if v]
            if active_keys:
                print(f"Active keys: {active_keys}")
            time.sleep(SLEEP_TIME)
    except KeyboardInterrupt:
        print("Teleoperation stopped by keyboard interrupt.")
    finally:
        keyboard_handler.shutdown() 