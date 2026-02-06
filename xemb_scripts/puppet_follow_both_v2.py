#!/usr/bin/env python3
import time
import sys
import evdev
from evdev import ecodes as e
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from interbotix_xs_msgs.msg import JointSingleCommand
from constants import START_ARM_POSE, MASTER_GRIPPER_JOINT_CLOSE, PUPPET_GRIPPER_JOINT_CLOSE, MASTER2PUPPET_JOINT_FN
from robot_utils import torque_on, torque_off, move_arms, move_grippers, get_arm_gripper_positions
from packing_utils_both import pack_values_into_axes, unpack_values_from_axes, get_heartbeat


# Control parameters
SMOOTH_ALPHA = 0.15  # Smoothing factor (lower = smoother)
CONTROL_PERIOD = 0.02  # 50 Hz


def prep_master_robot(master_bot):
    """Prepare a master robot for teleoperation."""
    # Reboot gripper motors, and set operating modes for all motors
    master_bot.dxl.robot_set_operating_modes("group", "arm", "position")
    master_bot.dxl.robot_set_operating_modes("single", "gripper", "position")
    torque_on(master_bot)

    # Move arm to starting position
    start_arm_qpos = START_ARM_POSE[:6]
    move_arms([master_bot], [start_arm_qpos], move_time=1)
    # Move gripper to starting position
    move_grippers([master_bot], [MASTER_GRIPPER_JOINT_MID], move_time=0.5)


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
            device.grab()  # Take exclusive control
            # No need to set non-blocking mode, read_one() already handles this
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
    
    # Initialize smoothed joint and gripper positions
    smoothed_joint_left = joint_commands_left.copy()
    smoothed_joint_right = joint_commands_right.copy()
    smoothed_gripper_left = PUPPET_GRIPPER_JOINT_CLOSE
    smoothed_gripper_right = PUPPET_GRIPPER_JOINT_CLOSE
    
    # Dictionaries to store axis state data for each heartbeat
    axis_states = [{}, {}, {}, {}]  # One dict for each heartbeat value
    
    # Timing variables
    last_update_time = time.time()
    last_print_time = time.time()
    
    # For smoothed gripper state
    current_gripper_left = None
    current_gripper_right = None
    
    try:
        while True:
            # Calculate elapsed time since last update
            now = time.time()
            elapsed = now - last_update_time
            
            # Read all available events from the controller
            events = []
            while True:
                event = gamepad.read_one()
                if event is None:
                    break
                events.append(event)
            
            # Process collected events
            batch_events = {}
            for event in events:
                if event and event.type == e.EV_ABS:
                    batch_events[event.code] = event.value
            
            # Extract heartbeat and update axis states if we have ABS_Z event
            if e.ABS_Z in batch_events:
                heartbeat = get_heartbeat(batch_events[e.ABS_Z])
                axis_states[heartbeat].update(batch_events)
                
                # Check if we have all required axes for this heartbeat
                required_axes = {e.ABS_X, e.ABS_Y, e.ABS_RX, e.ABS_Z}
                if heartbeat == 1 or heartbeat == 3:  # Heartbeats with gripper info
                    required_axes.add(e.ABS_RY)
                
                # Extract joint and gripper data if we have enough information
                all_states_complete = True
                for i, state in enumerate(axis_states):
                    req_axes = {e.ABS_X, e.ABS_Y, e.ABS_RX, e.ABS_Z}
                    if i == 1 or i == 3:  # Heartbeats with gripper info
                        req_axes.add(e.ABS_RY)
                    
                    if not req_axes.issubset(state.keys()):
                        all_states_complete = False
                        break
                
                # If we have all heartbeats with required data, update target positions
                if all_states_complete:
                    # Print update rate approximately once per second
                    if now - last_print_time >= 1.0:
                        print(f"Control update rate: {1.0/elapsed:.1f} Hz")
                        last_print_time = now
                    
                    # Combine all received joint data across all heartbeats
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
                        
                        # Update gripper targets if values are available
                        if gl is not None:
                            current_gripper_left = MASTER2PUPPET_JOINT_FN(gl)
                        
                        if gr is not None:
                            current_gripper_right = MASTER2PUPPET_JOINT_FN(gr)
            
            # Smooth the joint positions (run at fixed 50Hz rate)
            for i in range(6):
                smoothed_joint_left[i] = (
                    SMOOTH_ALPHA * joint_commands_left[i] +
                    (1 - SMOOTH_ALPHA) * smoothed_joint_left[i]
                )
                
                smoothed_joint_right[i] = (
                    SMOOTH_ALPHA * joint_commands_right[i] +
                    (1 - SMOOTH_ALPHA) * smoothed_joint_right[i]
                )
            
            # Smooth the gripper positions
            if current_gripper_left is not None:
                smoothed_gripper_left = (
                    SMOOTH_ALPHA * current_gripper_left +
                    (1 - SMOOTH_ALPHA) * smoothed_gripper_left
                )
                gripper_command_left.cmd = smoothed_gripper_left
                puppet_left.gripper.core.pub_single.publish(gripper_command_left)
            
            if current_gripper_right is not None:
                smoothed_gripper_right = (
                    SMOOTH_ALPHA * current_gripper_right +
                    (1 - SMOOTH_ALPHA) * smoothed_gripper_right
                )
                gripper_command_right.cmd = smoothed_gripper_right
                puppet_right.gripper.core.pub_single.publish(gripper_command_right)
            
            # Send smoothed joint positions to the robots
            puppet_left.arm.set_joint_positions(smoothed_joint_left, blocking=False)
            puppet_right.arm.set_joint_positions(smoothed_joint_right, blocking=False)
            
            # Calculate time to sleep to maintain 50Hz
            process_time = time.time() - now
            sleep_time = max(0, CONTROL_PERIOD - process_time)
            time.sleep(sleep_time)
            
            # Update timing for next iteration
            last_update_time = now
            
    except KeyboardInterrupt:
        print("\nTeleoperation stopped by user.")
    finally:
        for bot in puppet_bots:
            torque_off(bot)


if __name__ == '__main__':
    teleop_puppet()
