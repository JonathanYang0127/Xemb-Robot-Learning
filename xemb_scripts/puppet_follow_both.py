#!/usr/bin/env python3
import time
import sys
import evdev
from evdev import ecodes as e
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from interbotix_xs_msgs.msg import JointSingleCommand
from constants import DT, START_ARM_POSE, MASTER_GRIPPER_JOINT_CLOSE, PUPPET_GRIPPER_JOINT_CLOSE, MASTER2PUPPET_JOINT_FN
from robot_utils import torque_on, torque_off, move_arms, move_grippers, get_arm_gripper_positions
from packing_utils_both import pack_values_into_axes, unpack_values_from_axes, get_heartbeat


def prep_puppet_robot(puppet_bot):
    """Prepare the puppet robot for teleoperation."""
    puppet_bot.dxl.robot_set_operating_modes("group", "arm", "position")
    puppet_bot.dxl.robot_set_operating_modes("single", "gripper", "current_based_position")
    
    torque_on(puppet_bot)
    
    start_arm_qpos = START_ARM_POSE[:6]
    move_arms([puppet_bot], [start_arm_qpos], move_time=1)
    move_grippers([puppet_bot], [PUPPET_GRIPPER_JOINT_CLOSE], move_time=0.5)


def get_gamepad():
    """Find and configure the virtual Xbox controller."""
    target_name_keywords = ["xbox", "x-box", "360", "gamepad", "controller"]
    devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
    
    print("\nAvailable input devices:")
    for dev in devices:
        print(f"  - {dev.name} (path: {dev.path})")
    
    for device in devices:
        if any(keyword in device.name.lower() for keyword in target_name_keywords):
            print(f"\nFound virtual controller: {device.name}")
            print(f"Device path: {device.path}")
            print("Capabilities:", device.capabilities(verbose=True))
            return device
            
    print("No virtual controller found. Make sure the master robot is running.")
    sys.exit(1)


def teleop_puppet():
    """Control both puppet robots using input from the virtual controller."""
    print("Initializing Puppet Robots...")
    # Initialize both puppet robots
    puppet_left = InterbotixManipulatorXS(
        robot_model="vx300s",
        group_name="arm",
        gripper_name="gripper",
        robot_name='puppet_left',
        init_node=True
    )
    
    puppet_right = InterbotixManipulatorXS(
        robot_model="vx300s",
        group_name="arm",
        gripper_name="gripper",
        robot_name='puppet_right',  # No init_node since already initialized
        init_node=False
    )
    
    puppet_bots = [puppet_left, puppet_right]
    for bot in puppet_bots:
        prep_puppet_robot(bot)
    
    gamepad = get_gamepad()
    print(f"Connected to gamepad: {gamepad.name}")
    
    # Initialize control variables
    gripper_command_left = JointSingleCommand(name="gripper")
    gripper_command_right = JointSingleCommand(name="gripper")
    
    # Initialize joint position storage
    joint_commands_left = START_ARM_POSE[:6].copy()
    joint_commands_right = START_ARM_POSE[:6].copy()
    
    # Dictionaries to store axis state data for each heartbeat
    axis_states = [{}, {}, {}, {}]  # One dict for each heartbeat value
    
    # A temporary buffer to accumulate events in a single SYN_REPORT batch
    batch_events = {}

    old_time = None
    
    try:
        for event in gamepad.read_loop():
            if event.type == e.EV_ABS:
                # Buffer each axis event
                batch_events[event.code] = event.value
            
            elif event.type == e.EV_SYN and event.code == e.SYN_REPORT:
                # Process a complete batch of events
                if e.ABS_Z not in batch_events:
                    # Without heartbeat info, we cannot determine which state this belongs to
                    batch_events.clear()
                    continue
                
                # Determine heartbeat (0-3) from ABS_Z value
                heartbeat = get_heartbeat(batch_events[e.ABS_Z])
                print(heartbeat)

                # Update the appropriate state dictionary
                axis_states[heartbeat] = batch_events.copy()
                
                # Define required axes for each heartbeat
                required_axes = {e.ABS_X, e.ABS_Y, e.ABS_RX, e.ABS_Z}
                if heartbeat == 1 or heartbeat == 3:  # Heartbeats with gripper info need ABS_RY
                    required_axes.add(e.ABS_RY)
                
                # Check if we have this heartbeat's required axes
                if not required_axes.issubset(axis_states[heartbeat].keys()):
                    batch_events.clear()
                    continue
                
                # Check if we have all four heartbeats with required axes
                all_states_complete = True
                for i, state in enumerate(axis_states):
                    req_axes = {e.ABS_X, e.ABS_Y, e.ABS_RX, e.ABS_Z}
                    if i == 1 or i == 3:  # Heartbeats with gripper info need ABS_RY
                        req_axes.add(e.ABS_RY)
                    
                    if not req_axes.issubset(state.keys()):
                        print(i, state.keys())
                        all_states_complete = False
                        break
                
                if not all_states_complete:
                    batch_events.clear()
                    continue

                # All states complete
                new_time = time.time()
                if old_time is not None:
                    print("Update time", new_time - old_time)
                old_time = new_time

                # Now we have complete data for all states, unpack the values
                joints_left, joints_right, gripper_left, gripper_right = unpack_values_from_axes(axis_states[heartbeat])

                # Combine all received joint data (across all heartbeats) to get a complete picture
                for hb in range(4):
                    jl, jr, gl, gr = unpack_values_from_axes(axis_states[hb])
                    
                    # Update left arm joints if values are available
                    if jl is not None:
                        for i in range(6):
                            if jl[i] is not None:
                                joint_commands_left[i] = jl[i]
                    
                    # Update right arm joints if values are available
                    if jr is not None:
                        for i in range(6):
                            if jr[i] is not None:
                                joint_commands_right[i] = jr[i]
                    
                    # Update left gripper if value is available
                    if gl is not None:
                        gripper_left = gl
                    
                    # Update right gripper if value is available
                    if gr is not None:
                        gripper_right = gr
                
                # Send commands to left robot
                puppet_left.arm.set_joint_positions(joint_commands_left, blocking=False)
                if gripper_left is not None:
                    puppet_gripper_left_target = MASTER2PUPPET_JOINT_FN(gripper_left)
                    gripper_command_left.cmd = puppet_gripper_left_target
                    puppet_left.gripper.core.pub_single.publish(gripper_command_left)
                
                # Send commands to right robot
                puppet_right.arm.set_joint_positions(joint_commands_right, blocking=False)
                if gripper_right is not None:
                    puppet_gripper_right_target = MASTER2PUPPET_JOINT_FN(gripper_right)
                    gripper_command_right.cmd = puppet_gripper_right_target
                    puppet_right.gripper.core.pub_single.publish(gripper_command_right)
                
                # Clear batch events
                batch_events.clear()
                    
    except KeyboardInterrupt:
        print("\nTeleoperation stopped by user.")
    finally:
        for bot in puppet_bots:
            torque_off(bot)


if __name__ == '__main__':
    teleop_puppet()

