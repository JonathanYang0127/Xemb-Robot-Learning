import socket
import pickle
import time
import sys
import os
import numpy as np

# Add the parent directory to the path to import from xemb_scripts
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from xemb_scripts.constants import *
from packing_utils_widowx_piperx import unpack_data_from_udp
from robot_mapper import JointMapper
from xemb_scripts.base_control.base_controller import MDCRobot
from xemb_scripts.stepper_control.stepper_controller import StepperController

# Import PiperX SDK
from piper_sdk import C_PiperInterface_V2

UDP_PORT = 5005
SMOOTH_ALPHA = 0.15
BASE_SPEED = 1
CONTROL_PERIOD = 0.02

def process_key_states(key_states, stepper, base, last_states=None):
    """Process keyboard input for stepper and base control"""
    if last_states is None:
        last_states = {}
    
    # Stepper control (W/S for up/down)
    if key_states.get('w', False) and not last_states.get('w', False):
        stepper.move_steps(100)  # Move up
    elif key_states.get('s', False) and not last_states.get('s', False):
        stepper.move_steps(-100)  # Move down
    
    # Base control (A/D for left/right)
    if key_states.get('a', False):
        base.move_left()
    elif key_states.get('d', False):
        base.move_right()
    else:
        base.stop()
    
    # Arrow keys for additional base control
    if key_states.get('up', False):
        base.move_forward()
    elif key_states.get('down', False):
        base.move_backward()
    elif key_states.get('left', False):
        base.rotate_left()
    elif key_states.get('right', False):
        base.rotate_right()
    
    return key_states

def prep_piperx_robot(piper, neutral_positions, gripper_closed):
    """Prepare the PiperX robot for teleoperation"""
    print("Connecting to PiperX robot...")
    piper.ConnectPort()
    
    # Wait for connection
    while not piper.get_connect_status():
        time.sleep(0.01)
    
    print("Enabling PiperX robot...")
    # Wait for robot to enable (more reliable method)
    while not piper.EnablePiper():
        time.sleep(0.01)
    
    # Initialize gripper to closed position
    piper.GripperCtrl(int(gripper_closed), 1000, 0x01, 0)
    
    # Set initial joint positions to neutral from calibration
    piper.MotionCtrl_2(0x01, 0x01, 100, 0x00)
    piper.JointCtrl(*[int(pos) for pos in neutral_positions])
    
    print("PiperX robot ready for teleoperation")

def main():
    print("Initializing PiperX Puppet Robot...")
    
    # Initialize PiperX robots using the proper SDK
    puppet_left = C_PiperInterface_V2("can0")
    puppet_right = C_PiperInterface_V2("can1")  # Assuming second robot uses can1
    
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

    # Initialize stepper and base controllers
    print("Initializing controllers...")
    stepper = StepperController(port="/dev/tty_stepper")
    stepper.enable()
    base = MDCRobot(port='/dev/tty_base', default_speed=BASE_SPEED)
    base.connect()
    
    # Prepare both puppet robots with neutral positions
    prep_piperx_robot(puppet_left, neutral_positions, gripper_closed)
    prep_piperx_robot(puppet_right, neutral_positions, gripper_closed)
    
    # Initialize joint position storage
    joint_commands_left = [0.0] * 6  # Initialize with zeros
    joint_commands_right = [0.0] * 6
    smoothed_joint_left = joint_commands_left.copy()
    smoothed_joint_right = joint_commands_right.copy()
    smoothed_gripper_left = 0.0
    smoothed_gripper_right = 0.0
    target_gripper_left = 0.0
    target_gripper_right = 0.0
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
                
                # Map initial positions using the mapper
                print("[INIT] Mapping initial positions...")
                mapped_jl = mapper.map_joints(jl)
                mapped_jr = mapper.map_joints(jr)
                mapped_gl = mapper.map_gripper(gl)
                mapped_gr = mapper.map_gripper(gr)

                print("[INIT] Moving to mapped initial positions...")
                
                # Convert to degrees and send joint commands (PiperX expects degrees * 1000)
                left_joints_deg = [angle * 180.0 / np.pi * 1000 for angle in mapped_jl]
                right_joints_deg = [angle * 180.0 / np.pi * 1000 for angle in mapped_jr]
                
                # Set motion control parameters and send initial joint positions
                puppet_left.MotionCtrl_2(0x01, 0x01, 100, 0x00)
                puppet_right.MotionCtrl_2(0x01, 0x01, 100, 0x00)
                puppet_left.JointCtrl(*[int(angle) for angle in left_joints_deg])
                puppet_right.JointCtrl(*[int(angle) for angle in right_joints_deg])
                
                # Send initial gripper positions (convert to degrees * 1000)
                left_gripper_deg = int(mapped_gl * 180.0 / np.pi * 1000)
                right_gripper_deg = int(mapped_gr * 180.0 / np.pi * 1000)
                
                puppet_left.GripperCtrl(gripper_angle=left_gripper_deg, gripper_effort=1000, gripper_code=0x01)
                puppet_right.GripperCtrl(gripper_angle=right_gripper_deg, gripper_effort=1000, gripper_code=0x01)
                
                # Set smoothed joints to initial pose
                joint_commands_left = mapped_jl.copy()
                joint_commands_right = mapped_jr.copy()
                smoothed_joint_left = joint_commands_left.copy()
                smoothed_joint_right = joint_commands_right.copy()
                smoothed_gripper_left = mapped_gl
                smoothed_gripper_right = mapped_gr
                target_gripper_left = mapped_gl
                target_gripper_right = mapped_gr
                
                # Send confirmation
                ack_packet = {'mode': 'init_ack'}
                sock.sendto(pickle.dumps(ack_packet), addr)
                print("[INIT] Sent confirmation to master. Ready for teleop.")
                break
        except Exception as e:
            print(f"[INIT] Waiting for master... ({e})")
            time.sleep(1)

    try:
        print("Starting direct UDP teleoperation with joint mapping...")
        while True:
            loop_start = time.time()
            try:
                data, _ = sock.recvfrom(4096)
                packed = pickle.loads(data)
                # Ignore packets that are not teleop packets or from a different session
                if 'heartbeat' not in packed or packed.get('session_id') != session_id:
                    continue
                jl, jr, gl, gr, key_states = unpack_data_from_udp(packed)
                
                # Map incoming data using the mapper
                if jl is not None:
                    mapped_jl = mapper.map_joints(jl)
                    for i in range(6):
                        if mapped_jl[i] is not None:
                            joint_commands_left[i] = mapped_jl[i]
                if jr is not None:
                    mapped_jr = mapper.map_joints(jr)
                    for i in range(6):
                        if mapped_jr[i] is not None:
                            joint_commands_right[i] = mapped_jr[i]
                if gl is not None:
                    target_gripper_left = mapper.map_gripper(gl)
                if gr is not None:
                    target_gripper_right = mapper.map_gripper(gr)

                if key_states is not None:
                    most_recent_key_states = key_states
            except socket.timeout:
                pass
            
            # Always smooth toward the most recent target at every tick
            for i in range(6):
                smoothed_joint_left[i] = SMOOTH_ALPHA * joint_commands_left[i] + (1 - SMOOTH_ALPHA) * smoothed_joint_left[i]
                smoothed_joint_right[i] = SMOOTH_ALPHA * joint_commands_right[i] + (1 - SMOOTH_ALPHA) * smoothed_joint_right[i]
            smoothed_gripper_left = SMOOTH_ALPHA * target_gripper_left + (1 - SMOOTH_ALPHA) * smoothed_gripper_left
            smoothed_gripper_right = SMOOTH_ALPHA * target_gripper_right + (1 - SMOOTH_ALPHA) * smoothed_gripper_right

            # Send commands to robots
            # Convert radians to degrees * 1000 for PiperX
            left_joints_deg = [angle * 180.0 / np.pi * 1000 for angle in smoothed_joint_left]
            right_joints_deg = [angle * 180.0 / np.pi * 1000 for angle in smoothed_joint_right]
            
            # Set motion control parameters and send joint commands
            puppet_left.MotionCtrl_2(0x01, 0x01, 100, 0x00)
            puppet_right.MotionCtrl_2(0x01, 0x01, 100, 0x00)
            puppet_left.JointCtrl(*[int(angle) for angle in left_joints_deg])
            puppet_right.JointCtrl(*[int(angle) for angle in right_joints_deg])
            
            # Send gripper commands (convert to degrees * 1000)
            left_gripper_deg = int(smoothed_gripper_left * 180.0 / np.pi * 1000)
            right_gripper_deg = int(smoothed_gripper_right * 180.0 / np.pi * 1000)
            
            puppet_left.GripperCtrl(gripper_angle=left_gripper_deg, gripper_effort=1000, gripper_code=0x01)
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
        stepper.stop()
        stepper.disable()
        base.stop()
        base.shutdown()
        
        # Move PiperX robots to neutral position before shutdown
        try:
            puppet_left.JointCtrl(*[int(pos) for pos in neutral_positions])
            puppet_left.GripperCtrl(int(gripper_closed), 1000, 0x01, 0)
            puppet_right.JointCtrl(*[int(pos) for pos in neutral_positions])
            puppet_right.GripperCtrl(int(gripper_closed), 1000, 0x01, 0)
            time.sleep(1.0)
        except Exception as e:
            print(f"Error moving to neutral position: {e}")
        
        # Disable PiperX robots
        puppet_left.DisableArm(motor_num=7, enable_flag=0x01)
        puppet_right.DisableArm(motor_num=7, enable_flag=0x01)
        
        # Disconnect ports
        puppet_left.DisconnectPort()
        puppet_right.DisconnectPort()

if __name__ == '__main__':
    main() 
