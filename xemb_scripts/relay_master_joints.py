#!/usr/bin/env python3
import time
import sys
import evdev
from evdev import UInput, ecodes as e
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from interbotix_xs_msgs.msg import JointSingleCommand
from constants import (MASTER2PUPPET_JOINT_FN, DT, 
                        START_ARM_POSE, 
                        START_ARM_POSE_LEFT,
                        START_ARM_POSE_RIGHT,
                        MASTER_GRIPPER_JOINT_MID, 
                        PUPPET_GRIPPER_JOINT_CLOSE, MASTER_GRIPPER_JOINT_CLOSE)
from robot_utils import torque_on, torque_off, move_arms, move_grippers, get_arm_gripper_positions
from packing_utils import pack_values_into_axes, unpack_values_from_axes 

PRESS_TO_START_BUTTON = e.BTN_START
FPS = 30
SLEEP_TIME = float(1 / FPS)


def prep_master_robot(master_bot, arm_type='left'):
    # Reboot gripper motors, and set operating modes for all motors
    master_bot.dxl.robot_set_operating_modes("group", "arm", "position")
    master_bot.dxl.robot_set_operating_modes("single", "gripper", "position")
    torque_on(master_bot)

    # Move arm to starting position
    if arm_type == 'left':
        start_arm_qpos = START_ARM_POSE_LEFT[:6]
    elif arm_type == 'right':
        start_arm_qpos = START_ARM_POSE_RIGHT[:6]
    else:
        raise NotImplementedError
    move_arms([master_bot], [start_arm_qpos], move_time=1)
    # Move gripper to starting position
    move_grippers([master_bot], [MASTER_GRIPPER_JOINT_MID], move_time=0.5)

def press_to_start(master_bot):
    master_bot.dxl.robot_torque_enable("single", "gripper", False)
    print('Close the gripper to start')
    close_thresh = (MASTER_GRIPPER_JOINT_MID + 3 * MASTER_GRIPPER_JOINT_CLOSE) / 4
    pressed = False
    while not pressed:
        gripper_pos = get_arm_gripper_positions(master_bot)
        if gripper_pos < close_thresh:
            pressed = True
        time.sleep(DT / 10)
    torque_off(master_bot)
    print('Started!')

def create_virtual_controller():
    """Create a virtual game controller using evdev's UInput with exact Xbox 360 configuration."""
    capabilities = {
        e.EV_KEY: [
            e.BTN_SOUTH,   # A button
            e.BTN_EAST,    # B button
            e.BTN_NORTH,   # Y button
            e.BTN_WEST,    # X button
            e.BTN_TL,      # Left bumper
            e.BTN_TR,      # Right bumper
            e.BTN_SELECT,  # Back button
            e.BTN_START,   # Start button
            e.BTN_MODE,    # Xbox button
            e.BTN_THUMBL,  # Left stick press
            e.BTN_THUMBR   # Right stick press
        ],
        e.EV_ABS: [
            (e.ABS_X, evdev.AbsInfo(
                value=0,
                min=-32768,
                max=32767,
                fuzz=16,
                flat=128,
                resolution=0
            )),
            (e.ABS_Y, evdev.AbsInfo(
                value=0,
                min=-32768,
                max=32767,
                fuzz=16,
                flat=128,
                resolution=0
            )),
            (e.ABS_Z, evdev.AbsInfo(
                value=0,
                min=0,
                max=255,
                fuzz=0,
                flat=0,
                resolution=0
            )),
            (e.ABS_RX, evdev.AbsInfo(
                value=0,
                min=-32768,
                max=32767,
                fuzz=16,
                flat=128,
                resolution=0
            )),
            (e.ABS_RY, evdev.AbsInfo(
                value=0,
                min=-32768,
                max=32767,
                fuzz=16,
                flat=128,
                resolution=0
            )),
            (e.ABS_RZ, evdev.AbsInfo(
                value=0,
                min=0,
                max=255,
                fuzz=0,
                flat=0,
                resolution=0
            )),
            (e.ABS_HAT0X, evdev.AbsInfo(
                value=0,
                min=-1,
                max=1,
                fuzz=0,
                flat=0,
                resolution=0
            )),
            (e.ABS_HAT0Y, evdev.AbsInfo(
                value=0,
                min=-1,
                max=1,
                fuzz=0,
                flat=0,
                resolution=0
            ))
        ],
        e.EV_FF: [
            e.FF_RUMBLE,
            e.FF_PERIODIC,
            e.FF_CONSTANT,
            e.FF_RAMP,
            e.FF_SINE,
            e.FF_GAIN
        ]
    }
    
    try:
        device = UInput(capabilities,
                       name="Microsoft X-Box 360 pad",
                       vendor=0x045e,
                       product=0x028e,
                       version=0x110,
                       bustype=0x3)  # BUS_USB
        return device
    except OSError as err:
        print(f"Failed to create virtual controller: {err}")
        print("Make sure you have the necessary permissions (try running with sudo)")
        raise


def controller_emit(controller, event, value, syn=True):
    """Write an event to the controller."""
    if event in (e.ABS_X, e.ABS_Y, e.ABS_RX, e.ABS_RY, e.ABS_Z, e.ABS_RZ):
        controller.write(e.EV_ABS, event, value)
    else:
        controller.write(e.EV_KEY, event, value)
    if syn:
        controller.syn()


def controller_emit_all(controller, packed_values, packed_values_old, heartbeat):
    """Write all axes and buttons to the controller every cycle."""

    '''
    if heartbeat % 2 == 1:
        for i in (e.ABS_X, e.ABS_Y, e.ABS_RX, e.ABS_RY):
            packed_values[i] = 32767

        for i in (e.ABS_Z, e.ABS_RZ):
            packed_values[i] = 255
    '''
    for event_code in (e.ABS_X, e.ABS_Y, e.ABS_RX, e.ABS_RY, e.ABS_Z):
        if event_code in packed_values.keys():
            controller.write(e.EV_ABS, event_code, packed_values[event_code])
    
    # Single sync after all writes
    controller.syn()

def teleop(robot_side):
    """Runs teleoperation with Parsec-compatible controller input."""
    master_bot = InterbotixManipulatorXS(
        robot_model="wx250s", 
        group_name="arm",
        gripper_name="gripper", 
        robot_name=f'master_{robot_side}', 
        init_node=True
    )

    # Prepare the robot
    prep_master_robot(master_bot)

    # Create the virtual controller
    controller = create_virtual_controller()
    print("Virtual controller created. Waiting for press-to-start...")

    # Wait for the user to close the gripper to start
    press_to_start(master_bot)

    controller_emit(controller, PRESS_TO_START_BUTTON, 1, syn=True)
    time.sleep(0.2)
    controller_emit(controller, PRESS_TO_START_BUTTON, 0, syn=True)

    packed_values_old = [None for i in range(6)]
    heartbeat = 0

    try:
        while True:
            # Read joint positions
            master_state_joints = master_bot.dxl.joint_states.position[:6]
            master_gripper_joint = master_bot.dxl.joint_states.position[6]
           

            # Pack values into axes
            packed_values = pack_values_into_axes(
                master_state_joints, 
                master_gripper_joint,
                heartbeat
            )
            
            # Emit all values every cycle
            controller_emit_all(controller, packed_values, packed_values_old, heartbeat)
            packed_values_old = packed_values.copy()

            heartbeat = (heartbeat + 1) % 2
            time.sleep(SLEEP_TIME)  # Maintain loop timing

    except KeyboardInterrupt:
        print("Teleoperation stopped.")
    finally:
        # Clean up
        controller.close()
        master_bot.shutdown()


if __name__ == '__main__':
    if len(sys.argv) != 2 or sys.argv[1] not in ['left', 'right']:
        print("Usage: sudo python3 relay_master_joints.py <left|right>")
        sys.exit(1)
    
    side = sys.argv[1]
    teleop(side)
