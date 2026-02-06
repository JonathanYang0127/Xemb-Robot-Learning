import socket
import pickle
import time
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from interbotix_xs_msgs.msg import JointSingleCommand
from constants import *
from packing_utils_direct import unpack_data_from_udp
from robot_utils import torque_on, torque_off, move_arms, move_grippers
from base_control.base_controller import MDCRobot
from stepper_control.stepper_controller import StepperController
import numpy as np
import rospy
from sensor_msgs.msg import JointState

UDP_PORT = 5005
SMOOTH_ALPHA = 0.15
BASE_SPEED = 1
CONTROL_PERIOD = 0.02
MAX_JOINT_VELOCITY = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0]) * 10   # rad/s, adjust as needed for each joint
# Updated joint limits based on user specification
JOINT_LIMITS_LEFT = [
    [-0.4939418137073517, None],   # waist (left)
    [-1.6, 0.348], # shoulder (left)
    [-0.45, 1.5],                 # elbow (was forearm_roll, left)
    [None, None],                 # wrist_angle (left)
    [None, None],                 # wrist_rotate (left)
    [None, None],                 # gripper or other (left)
]

JOINT_LIMITS_RIGHT = [
    [None, 0.39],  # waist (right)
    [-1.6, 0.348],              # shoulder (right)
    [-0.45, 1.5],                 # elbow (was forearm_roll, right)
    [None, None],                 # wrist_angle (right)
    [None, None],                 # wrist_rotate (right)
    [None, None],                 # gripper or other (right)
]

def enforce_joint_limits(joint_positions, limits):
    limited_positions = []
    all_within_limits = True
    for i, pos in enumerate(joint_positions):
        min_limit, max_limit = limits[i]
        limited_pos = pos
        joint_name = JOINT_NAMES[i] if i < len(JOINT_NAMES) else f"joint_{i}"
        # Only check min if not None
        if min_limit is not None and pos < min_limit:
            all_within_limits = False
            limited_pos = min_limit
            print(f"[LIMIT] Joint {i} ({joint_name}) below min: {pos:.2f} -> {limited_pos:.2f}")
            print(f"  All joint values: {[f'{v:.2f}' for v in joint_positions]}")
        # Only check max if not None
        if max_limit is not None and pos > max_limit:
            all_within_limits = False
            limited_pos = max_limit
            print(f"[LIMIT] Joint {i} ({joint_name}) above max: {pos:.2f} -> {limited_pos:.2f}")
            print(f"  All joint values: {[f'{v:.2f}' for v in joint_positions]}")
        limited_positions.append(limited_pos)
    return limited_positions, all_within_limits

def process_key_states(key_states, stepper, base, last_states=None):
    if last_states is None:
        last_states = {
            'w': False, 's': False,
            'up': False, 'down': False, 'left': False, 'right': False
        }
    if key_states.get('w', False) and not last_states.get('w', False):
        stepper.forward()
    elif key_states.get('s', False) and not last_states.get('s', False):
        stepper.backward()
    elif (not key_states.get('w', False) and last_states.get('w', False)) or \
         (not key_states.get('s', False) and last_states.get('s', False)):
        stepper.stop()
    if key_states.get('up', False):
        base.drive(speed=BASE_SPEED, turn=0)
    elif key_states.get('down', False):
        base.drive(speed=-BASE_SPEED, turn=0)
    elif key_states.get('left', False):
        base.drive(speed=0, turn=BASE_SPEED)
    elif key_states.get('right', False):
        base.drive(speed=0, turn=-BASE_SPEED)
    elif (last_states.get('up', False) or last_states.get('down', False) or
          last_states.get('left', False) or last_states.get('right', False)) and \
         not any(key_states.get(k, False) for k in ['up', 'down', 'left', 'right']):
        base.stop()
    return key_states.copy()

def prep_puppet_robot(puppet_bot, arm_type='left'):
    puppet_bot.dxl.robot_set_operating_modes("group", "arm", "position")
    puppet_bot.dxl.robot_set_operating_modes("single", "gripper", "current_based_position")
    torque_on(puppet_bot)
    if arm_type == 'left':
        start_arm_qpos = START_ARM_POSE_LEFT[:6]
    elif arm_type == 'right':
        start_arm_qpos = START_ARM_POSE_RIGHT[:6]
    else:
        raise NotImplementedError

    move_arms([puppet_bot], [start_arm_qpos], move_time=1)
    move_grippers([puppet_bot], [PUPPET_GRIPPER_JOINT_CLOSE], move_time=0.5)

def left_joint_state_cb(msg):
    global achieved_joint_left
    achieved_joint_left = list(msg.position[:6])

def right_joint_state_cb(msg):
    global achieved_joint_right
    achieved_joint_right = list(msg.position[:6])

def clip_joint_velocity(achieved, target, max_vel, dt):
    clipped = []
    for i in range(len(target)):
        delta = target[i] - achieved[i]
        max_delta = max_vel[i] * dt
        if delta > max_delta:
            clipped.append(achieved[i] + max_delta)
        elif delta < -max_delta:
            clipped.append(achieved[i] - max_delta)
        else:
            clipped.append(target[i])
    return clipped

def main():
    puppet_left = InterbotixManipulatorXS(robot_model="vx300s", group_name="arm", gripper_name="gripper", robot_name='puppet_left', init_node=True)
    puppet_right = InterbotixManipulatorXS(robot_model="vx300s", group_name="arm", gripper_name="gripper", robot_name='puppet_right', init_node=False)
    stepper = StepperController(port="/dev/tty_stepper")
    stepper.enable()
    base = MDCRobot(port='/dev/tty_base', default_speed=BASE_SPEED)
    base.connect()
    # Do NOT call prep_puppet_robot here
    gripper_command_left = JointSingleCommand(name="gripper")
    gripper_command_right = JointSingleCommand(name="gripper")
    joint_commands_left = START_ARM_POSE[:6].copy()
    joint_commands_right = START_ARM_POSE[:6].copy()
    smoothed_joint_left = joint_commands_left.copy()
    smoothed_joint_right = joint_commands_right.copy()
    smoothed_gripper_left = PUPPET_GRIPPER_JOINT_CLOSE
    smoothed_gripper_right = PUPPET_GRIPPER_JOINT_CLOSE
    target_gripper_left = PUPPET_GRIPPER_JOINT_CLOSE
    target_gripper_right = PUPPET_GRIPPER_JOINT_CLOSE
    last_key_states = None
    most_recent_key_states = None
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
                limited_jl, _ = enforce_joint_limits(jl, JOINT_LIMITS_LEFT)
                limited_jr, _ = enforce_joint_limits(jr, JOINT_LIMITS_RIGHT)
                print("[INIT] Setting operating modes and torquing on...")
                puppet_left.dxl.robot_set_operating_modes("group", "arm", "position")
                puppet_left.dxl.robot_set_operating_modes("single", "gripper", "current_based_position")
                puppet_right.dxl.robot_set_operating_modes("group", "arm", "position")
                puppet_right.dxl.robot_set_operating_modes("single", "gripper", "current_based_position")
                torque_on(puppet_left)
                torque_on(puppet_right)
                print("[INIT] Moving to received initial positions...")
                move_arms([puppet_left], [limited_jl], move_time=1.5)
                move_arms([puppet_right], [limited_jr], move_time=1.5)
                move_grippers([puppet_left], [MASTER2PUPPET_JOINT_FN(gl)], move_time=0.7)
                move_grippers([puppet_right], [MASTER2PUPPET_JOINT_FN(gr)], move_time=0.7)
                # Set smoothed joints to initial pose
                joint_commands_left = limited_jl.copy()
                joint_commands_right = limited_jr.copy()
                smoothed_joint_left = limited_jl.copy()
                smoothed_joint_right = limited_jr.copy()
                # Print commanded and achieved initial pose for comparison
                achieved_left = list(puppet_left.dxl.joint_states.position[:6])
                achieved_right = list(puppet_right.dxl.joint_states.position[:6])
                print(f"[INIT] Commanded initial left arm pose: {limited_jl}")
                print(f"[INIT] Achieved initial left arm pose:  {achieved_left}")
                print(f"[INIT] Commanded initial right arm pose: {limited_jr}")
                print(f"[INIT] Achieved initial right arm pose:  {achieved_right}")
                # Send confirmation
                ack_packet = {'mode': 'init_ack'}
                sock.sendto(pickle.dumps(ack_packet), addr)
                print("[INIT] Sent confirmation to master. Ready for teleop.")
                break
        except socket.timeout:
            pass

    try:
        print("Starting direct UDP teleoperation with robust smoothing and correct gripper mapping...")
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
                if jr is not None:
                    for i in range(6):
                        if jr[i] is not None:
                            joint_commands_right[i] = jr[i]
                if gl is not None:
                    target_gripper_left = MASTER2PUPPET_JOINT_FN(gl)
                if gr is not None:
                    target_gripper_right = MASTER2PUPPET_JOINT_FN(gr)
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
            gripper_command_left.cmd = smoothed_gripper_left
            puppet_left.gripper.core.pub_single.publish(gripper_command_left)
            gripper_command_right.cmd = smoothed_gripper_right
            puppet_right.gripper.core.pub_single.publish(gripper_command_right)
            # Stepper/base control: always use the most recent key states
            if most_recent_key_states is not None:
                last_key_states = process_key_states(most_recent_key_states, stepper, base, last_key_states)
            # Get achieved joint positions from Interbotix API
            achieved_joint_left = list(puppet_left.dxl.joint_states.position[:6])
            achieved_joint_right = list(puppet_right.dxl.joint_states.position[:6])
            # --- DEBUG PRINTS ---
            print("[DEBUG] Joint index | Commanded | Clipped | Achieved (left arm)")
            limited_joint_left, _ = enforce_joint_limits(smoothed_joint_left, JOINT_LIMITS_LEFT)
            # Clip velocity based on achieved position
            limited_joint_left = clip_joint_velocity(achieved_joint_left, limited_joint_left, MAX_JOINT_VELOCITY, CONTROL_PERIOD)
            for i in range(6):
                cmd = smoothed_joint_left[i]
                clipped = limited_joint_left[i]
                achieved = achieved_joint_left[i] if i < len(achieved_joint_left) else None
                print(f"[DEBUG] {i:2d} | {cmd: .4f} | {clipped: .4f} | {achieved if achieved is not None else 'N/A'}")
            print("[COMMAND] Sending to left arm:", [f"{x:.4f}" for x in limited_joint_left])
            puppet_left.arm.set_joint_positions(limited_joint_left, blocking=False)

            print("[DEBUG] Joint index | Commanded | Clipped | Achieved (right arm)")
            limited_joint_right, _ = enforce_joint_limits(smoothed_joint_right, JOINT_LIMITS_RIGHT)
            # Clip velocity based on achieved position
            limited_joint_right = clip_joint_velocity(achieved_joint_right, limited_joint_right, MAX_JOINT_VELOCITY, CONTROL_PERIOD)
            for i in range(6):
                cmd = smoothed_joint_right[i]
                clipped = limited_joint_right[i]
                achieved = achieved_joint_right[i] if i < len(achieved_joint_right) else None
                print(f"[DEBUG] {i:2d} | {cmd: .4f} | {clipped: .4f} | {achieved if achieved is not None else 'N/A'}")
            print("[COMMAND] Sending to right arm:", [f"{x:.4f}" for x in limited_joint_right])
            puppet_right.arm.set_joint_positions(limited_joint_right, blocking=False)
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
        torque_off(puppet_left)
        torque_off(puppet_right)

if __name__ == '__main__':
    main() 
