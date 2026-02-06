import socket
import time
import pickle
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from constants import *
from packing_utils_direct import pack_data_for_udp
from robot_utils import torque_on, torque_off, move_arms, move_grippers, get_arm_gripper_positions
from pynput import keyboard
import sys
import argparse
import uuid
import json

PUPPET_IP = "100.105.116.25"  # <-- Set puppet's Tailscale IP here
UDP_PORT = 5005
SLEEP_TIME = 1/30

def save_joint_states(filename, left_joints, right_joints, left_gripper, right_gripper):
    data = {
        'left_joints': left_joints,
        'right_joints': right_joints,
        'left_gripper': left_gripper,
        'right_gripper': right_gripper
    }
    with open(filename, 'w') as f:
        json.dump(data, f)
    print(f"Joint states saved to {filename}")

def load_joint_states(filename):
    with open(filename, 'r') as f:
        data = json.load(f)
    print(f"Joint states loaded from {filename}")
    return data

class KeyboardHandler:
    def __init__(self):
        self.key_states = {
            'w': False, 'a': False, 's': False, 'd': False,
            'up': False, 'left': False, 'down': False, 'right': False,
            'p': False  # Add 'p' for save
        }
        self.running = True
        self.listener = keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release
        )
        self.listener.start()
        print("Keyboard listener started. W/A/S/D, arrow keys, and 'p' (save) are now active. Press ESC to quit.")
    def _on_press(self, key):
        try:
            key_char = key.char.lower()
            if key_char in 'wasd':
                self.key_states[key_char] = True
            elif key_char == 'p':
                self.key_states['p'] = True
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
            elif key_char == 'p':
                self.key_states['p'] = False
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

def prep_master_robot(master_bot, arm_type='left', skip_move_and_torque=False):
    if not skip_move_and_torque:
        master_bot.dxl.robot_set_operating_modes("group", "arm", "position")
        master_bot.dxl.robot_set_operating_modes("single", "gripper", "position")
        if arm_type == 'left':
            start_arm_qpos = START_ARM_POSE_LEFT[:6]
        elif arm_type == 'right':
            start_arm_qpos = START_ARM_POSE_RIGHT[:6]
        else:
            raise NotImplementedError

        joint_names = master_bot.arm.group_info.joint_names
        print("[DEBUG] Joint names:", joint_names)
        print("[DEBUG] start_arm_qpos before move_arms:", start_arm_qpos)
        for i, name in enumerate(joint_names):
            print(f"Joint {i}: {name}, start_q = {start_arm_qpos[i]}")
        move_arms([master_bot], [start_arm_qpos], move_time=1)
        print("[DEBUG] Called move_arms with:", start_arm_qpos)
    # Always open the gripper
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

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--save-joints', type=str, help='File to save current joint states after initialization')
    parser.add_argument('--load-joints', type=str, help='File to load joint states from for initialization')
    parser.add_argument('--gripper-lock-left', action='store_true', help='Use left gripper to lock/unlock left arm during teleop')
    parser.add_argument('--gripper-lock-right', action='store_true', help='Use right gripper to lock/unlock right arm during teleop')
    args = parser.parse_args()

    # Generate a session_id for this run
    session_id = uuid.uuid4().hex

    # Initialize both master arms immediately
    master_left = InterbotixManipulatorXS(robot_model="wx250s", group_name="arm", gripper_name="gripper", robot_name='master_left', init_node=True)
    master_right = InterbotixManipulatorXS(robot_model="wx250s", group_name="arm", gripper_name="gripper", robot_name='master_right', init_node=False)

    master_bots = {'left': master_left, 'right': master_right}

    # Prepare both robots (move/torque if not skipped, always open gripper)
    if args.load_joints:
        joint_data = load_joint_states(args.load_joints)
        move_arms([master_left], [joint_data['left_joints']], move_time=1)
        move_arms([master_right], [joint_data['right_joints']], move_time=1)
        print("Moved arms to loaded joint states.")
    else:
        for arm_type, bot in master_bots.items():
            torque_on(bot)  # Always torque on first
            prep_master_robot(bot, arm_type=arm_type, skip_move_and_torque=False)
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

    # Save joint states if requested
    if args.save_joints:
        jl = list(master_left.dxl.joint_states.position[:6])
        jr = list(master_right.dxl.joint_states.position[:6])
        gl = master_left.dxl.joint_states.position[6]
        gr = master_right.dxl.joint_states.position[6]
        save_joint_states(args.save_joints, jl, jr, gl, gr)

    # Start UDP sending loop
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    heartbeat = 0
    keyboard_handler = KeyboardHandler()
    # Store last sent positions for lock functionality
    last_jl = list(master_left.dxl.joint_states.position[:6])
    last_jr = list(master_right.dxl.joint_states.position[:6])
    # Track previous lock state for gripper lock arms
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
            # --- Save on 'p' key press ---
            if keyboard_handler.key_states.get('p', False):
                jl = list(master_left.dxl.joint_states.position[:6])
                jr = list(master_right.dxl.joint_states.position[:6])
                gl = master_left.dxl.joint_states.position[6]
                gr = master_right.dxl.joint_states.position[6]
                save_filename = args.save_joints if args.save_joints else 'saved_joints.json'
                save_joint_states(save_filename, jl, jr, gl, gr)
                print(f"[SAVE] Joint states saved to {save_filename} (triggered by 'p' key)")
                keyboard_handler.key_states['p'] = False  # Prevent repeated saves
            time.sleep(SLEEP_TIME)
    except KeyboardInterrupt:
        print("Teleoperation stopped by keyboard interrupt.")
    finally:
        keyboard_handler.shutdown() 