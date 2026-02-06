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
UDP_PORT = 5010
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
        self.running = False

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
    """
    Prepare the master robot for teleoperation
    """
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

def press_to_start(master_bots, single_arm=False):
    """
    Wait for user to close gripper(s) to start teleoperation
    """
    # Torque off grippers to allow manual closure
    if single_arm:
        bot = master_bots[0] if isinstance(master_bots, list) else master_bots
        bot.dxl.robot_torque_enable("single", "gripper", False)
    else:
        for bot in master_bots:
            bot.dxl.robot_torque_enable("single", "gripper", False)
    
    if single_arm:
        print("Close the gripper to start teleoperation")
        bot = master_bots[0] if isinstance(master_bots, list) else master_bots
        while True:
            gripper_pos = bot.dxl.joint_states.position[6]
            if gripper_pos < (MASTER_GRIPPER_JOINT_OPEN + MASTER_GRIPPER_JOINT_CLOSE) / 2:
                break
            time.sleep(0.1)
        print("Gripper closed, starting teleoperation")
    else:
        print("Close both grippers to start teleoperation")
        while True:
            left_gripper_pos = master_bots[0].dxl.joint_states.position[6]
            right_gripper_pos = master_bots[1].dxl.joint_states.position[6]
            threshold = (MASTER_GRIPPER_JOINT_OPEN + MASTER_GRIPPER_JOINT_CLOSE) / 2
            if left_gripper_pos < threshold and right_gripper_pos < threshold:
                break
            time.sleep(0.1)
        print("Both grippers closed, starting teleoperation")
    
    # Torque off entire robot after gripper closure
    if single_arm:
        bot = master_bots[0] if isinstance(master_bots, list) else master_bots
        torque_off(bot)
    else:
        for bot in master_bots:
            torque_off(bot)

def create_init_packet(session_id, master_left=None, master_right=None, single_arm=False, arm_side='left'):
    """
    Create initialization packet for puppet
    """
    init_packet = {
        'mode': 'init',
        'init_session_id': session_id,
        'single_arm': single_arm,
        'arm_side': arm_side
    }
    
    if single_arm:
        if arm_side == 'left' and master_left is not None:
            init_packet['jl'] = list(master_left.dxl.joint_states.position[:6])
            init_packet['gl'] = master_left.dxl.joint_states.position[6]
            init_packet['jr'] = None
            init_packet['gr'] = None
        elif arm_side == 'right' and master_right is not None:
            init_packet['jr'] = list(master_right.dxl.joint_states.position[:6])
            init_packet['gr'] = master_right.dxl.joint_states.position[6]
            init_packet['jl'] = None
            init_packet['gl'] = None
    else:
        # Dual arm mode
        init_packet['jl'] = list(master_left.dxl.joint_states.position[:6]) if master_left else None
        init_packet['jr'] = list(master_right.dxl.joint_states.position[:6]) if master_right else None
        init_packet['gl'] = master_left.dxl.joint_states.position[6] if master_left else None
        init_packet['gr'] = master_right.dxl.joint_states.position[6] if master_right else None
    
    return init_packet

def main():
    parser = argparse.ArgumentParser(description='Flexible WidowX Master Controller')
    parser.add_argument('--single-arm', action='store_true', help='Enable single-arm control mode')
    parser.add_argument('--arm-side', choices=['left', 'right'], default='left', 
                       help='Which arm to use in single-arm mode (default: left)')
    parser.add_argument('--no-move-left', action='store_true', help='Do not move or torque on the left arm at startup')
    parser.add_argument('--no-move-right', action='store_true', help='Do not move or torque on the right arm at startup')
    parser.add_argument('--gripper-lock-left', action='store_true', help='Use left gripper to lock/unlock left arm during teleop')
    parser.add_argument('--gripper-lock-right', action='store_true', help='Use right gripper to lock/unlock right arm during teleop')
    parser.add_argument('--puppet-ip', default=PUPPET_IP, help='IP address of the puppet robot')
    parser.add_argument('--no-heartbeat', action='store_true', help='Disable heartbeat system and send all joint data in every packet for higher frequency')
    args = parser.parse_args()

    # Use the puppet IP from arguments
    puppet_ip = args.puppet_ip

    # Generate a session_id for this run
    session_id = uuid.uuid4().hex

    # Initialize arms based on mode
    master_left = None
    master_right = None
    master_bots = []

    if args.single_arm:
        print(f"Initializing single-arm mode: {args.arm_side} arm")
        if args.arm_side == 'left':
            master_left = InterbotixManipulatorXS(robot_model="wx250s", group_name="arm", gripper_name="gripper", 
                                                robot_name='master_left', init_node=True)
            torque_on(master_left)
            prep_master_robot(master_left, arm_type='left', skip_move_and_torque=args.no_move_left)
            master_bots = [master_left]
        else:  # right
            master_right = InterbotixManipulatorXS(robot_model="wx250s", group_name="arm", gripper_name="gripper", 
                                                 robot_name='master_right', init_node=True)
            torque_on(master_right)
            prep_master_robot(master_right, arm_type='right', skip_move_and_torque=args.no_move_right)
            master_bots = [master_right]
    else:
        print("Initializing dual-arm mode")
        master_left = InterbotixManipulatorXS(robot_model="wx250s", group_name="arm", gripper_name="gripper", 
                                            robot_name='master_left', init_node=True)
        master_right = InterbotixManipulatorXS(robot_model="wx250s", group_name="arm", gripper_name="gripper", 
                                             robot_name='master_right', init_node=False)
        
        # Prepare both robots
        torque_on(master_left)
        torque_on(master_right)
        prep_master_robot(master_left, arm_type='left', skip_move_and_torque=args.no_move_left)
        prep_master_robot(master_right, arm_type='right', skip_move_and_torque=args.no_move_right)
        master_bots = [master_left, master_right]

    # Send initial positions to puppet and wait for confirmation
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    init_packet = create_init_packet(session_id, master_left, master_right, args.single_arm, args.arm_side)
    data = pickle.dumps(init_packet)
    confirmed = False
    sock.settimeout(0.5)
    print("[INIT] Sending initial positions to puppet and waiting for confirmation...")
    
    while not confirmed:
        sock.sendto(data, (puppet_ip, UDP_PORT))
        try:
            resp, _ = sock.recvfrom(1024)
            resp_packet = pickle.loads(resp)
            if resp_packet.get('mode') == 'init_ack':
                confirmed = True
                print("[INIT] Puppet confirmed initial pose.")
        except socket.timeout:
            print("[INIT] No confirmation yet, resending...")
        time.sleep(0.5)

    # Wait for user to start teleoperation
    press_to_start(master_bots, args.single_arm)

    # Start UDP sending loop
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    heartbeat = 0
    keyboard_handler = KeyboardHandler()
    
    # Store last sent positions for lock functionality
    last_jl = list(master_left.dxl.joint_states.position[:6]) if master_left else None
    last_jr = list(master_right.dxl.joint_states.position[:6]) if master_right else None
    
    # Track previous lock state for no-move arms
    prev_locked_left = None
    prev_locked_right = None
    
    try:
        print(f"Starting teleoperation in {'single-arm' if args.single_arm else 'dual-arm'} mode...")
        while keyboard_handler.running:
            # Initialize default values
            jl_to_send = None
            jr_to_send = None
            gl = None
            gr = None
            
            # Left arm handling
            if master_left is not None:
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
                        jl_to_send = last_jl
                        gl = MASTER_GRIPPER_JOINT_CLOSE
                    else:
                        jl_to_send = list(master_left.dxl.joint_states.position[:6])
                        gl = master_left.dxl.joint_states.position[6]
                        last_jl = jl_to_send
                else:
                    jl_to_send = list(master_left.dxl.joint_states.position[:6])
                    gl = master_left.dxl.joint_states.position[6]
                    last_jl = jl_to_send
                    
            # Right arm handling
            if master_right is not None:
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
                        jr_to_send = last_jr
                        gr = MASTER_GRIPPER_JOINT_CLOSE
                    else:
                        jr_to_send = list(master_right.dxl.joint_states.position[:6])
                        gr = master_right.dxl.joint_states.position[6]
                        last_jr = jr_to_send
                else:
                    jr_to_send = list(master_right.dxl.joint_states.position[:6])
                    gr = master_right.dxl.joint_states.position[6]
                    last_jr = jr_to_send
            
            # Pack and send data
            packed = pack_data_for_udp(jl_to_send, jr_to_send, gl, gr, heartbeat, keyboard_handler.key_states, no_heartbeat=args.no_heartbeat)
            packed['session_id'] = session_id
            packed['single_arm'] = args.single_arm
            packed['arm_side'] = args.arm_side
            
            data = pickle.dumps(packed)
            sock.sendto(data, (puppet_ip, UDP_PORT))
            
            # Only increment heartbeat counter if not in no-heartbeat mode
            if not args.no_heartbeat:
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
        for bot in master_bots:
            if bot is not None:
                bot.shutdown()
        sock.close()
        print("Cleanup complete.")

if __name__ == '__main__':
    main() 