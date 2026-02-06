#!/usr/bin/env python3
import time
import sys
import evdev
from evdev import ecodes as e
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from interbotix_xs_msgs.msg import JointSingleCommand
from constants import DT, START_ARM_POSE, MASTER_GRIPPER_JOINT_CLOSE, PUPPET_GRIPPER_JOINT_CLOSE, MASTER2PUPPET_JOINT_FN
from robot_utils import torque_on, torque_off, move_arms, move_grippers, get_arm_gripper_positions
from packing_utils import pack_values_into_axes, unpack_values_from_axes


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
    
    while True:
        devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
        
        print("\nSearching for controller...")
        for dev in devices:
            print(f"  - {dev.name} (path: {dev.path})")
        
        for device in devices:
            if any(keyword in device.name.lower() for keyword in target_name_keywords):
                print(f"\nFound virtual controller: {device.name}")
                print(f"Device path: {device.path}")
                print("Capabilities:", device.capabilities(verbose=True))
                return device
        
        print("No virtual controller found. Waiting for controller to appear...")
        time.sleep(0.5)  # Wait for 0.5 seconds before trying again


def teleop_puppet(robot_side):
    """Control the puppet robot using input from the virtual controller."""
    print("Initializing Puppet Robot...")
    puppet_bot = InterbotixManipulatorXS(
        robot_model="vx300s",
        group_name="arm",
        gripper_name="gripper",
        robot_name=f'puppet_{robot_side}',
        init_node=True
    )
    
    prep_puppet_robot(puppet_bot)
    gamepad = get_gamepad()
    print(f"Connected to gamepad: {gamepad.name}")
    
    # Initialize control variables
    gripper_command = JointSingleCommand(name="gripper")
    joint_commands_old = START_ARM_POSE[:6]
    
    # Dictionaries to hold the latest events for each heartbeat state
    axis_state_0 = {}  # For heartbeat == 0 (e.g., joint values)
    axis_state_1 = {}  # For heartbeat == 1 (e.g., gripper and extra joint values)
    
    # A temporary buffer to accumulate events in a single SYN_REPORT batch.
    batch_events = {}
    
    try:
        for event in gamepad.read_loop():
            if event.type == e.EV_ABS:
                # Buffer each axis event. If multiple events of the same code arrive,
                # the last value will overwrite previous ones.
                batch_events[event.code] = event.value
            
            elif event.type == e.EV_SYN and event.code == e.SYN_REPORT:
                # We now have a complete batch of events.
                if e.ABS_Z not in batch_events:
                    # Without heartbeat info (ABS_Z), we cannot decide which state this batch belongs to.
                    batch_events.clear()
                    continue
                
                # Determine the heartbeat value from ABS_Z.
                # (Here we assume a threshold of 50 to decide between heartbeat 0 and 1.)
                heartbeat = int(batch_events[e.ABS_Z] >= 50)
                
                # Update the appropriate state dictionary with the data from this batch.
                if heartbeat == 0:
                    axis_state_0 = batch_events.copy()
                else:  # heartbeat == 1
                    axis_state_1 = batch_events.copy()
                
                # Define the required axes for each state.
                required_axes_0 = {e.ABS_X, e.ABS_Y, e.ABS_RX, e.ABS_Z}
                required_axes_1 = {e.ABS_X, e.ABS_Y, e.ABS_RX, e.ABS_RY, e.ABS_Z}
                
                # Only process if both states have been updated with all required axes.
                if not (required_axes_0.issubset(axis_state_0.keys()) and 
                        required_axes_1.issubset(axis_state_1.keys())):
                    # Clear the temporary batch buffer and wait for more events.
                    batch_events.clear()
                    continue
                
                # Now that we have complete data for both states, extract joint and gripper values.
                joints_0, _ = unpack_values_from_axes(axis_state_0)
                joints_1, gripper = unpack_values_from_axes(axis_state_1)
                
                if joints_0 is None or joints_1 is None or gripper is None:
                    # If unpacking failed, clear and wait for the next complete batch.
                    batch_events.clear()
                    continue
                
                # Example: update joint commands.
                # (Here, we update joint 0 from the heartbeat-0 data. In practice, you may
                # update other joints from either or both states as needed.)
                for i in range(6):
                    if joints_0[i] is not None:
                        joint_commands_old[i] = joints_0[i]
                    if joints_1[i] is not None:
                        joint_commands_old[i] = joints_1[i]
                
                # Send the new joint positions (non-blocking).
                puppet_bot.arm.set_joint_positions(joint_commands_old, blocking=False)
                
                # Handle gripper command by mapping the master input to the puppet value.
                puppet_gripper_joint_target = MASTER2PUPPET_JOINT_FN(gripper)
                gripper_command.cmd = puppet_gripper_joint_target
                puppet_bot.gripper.core.pub_single.publish(gripper_command)
                
                # Clear both the temporary batch and stored state dictionaries for the next cycle.
                batch_events.clear()
                axis_state_0.clear()
                axis_state_1.clear()
                    
    except KeyboardInterrupt:
        print("\nTeleoperation stopped by user.")
    finally:
        torque_off(puppet_bot)


if __name__ == '__main__':
    if len(sys.argv) != 2 or sys.argv[1] not in ['left', 'right']:
        print("Usage: python3 teleop_puppet.py <left|right>")
        sys.exit(1)
    
    side = sys.argv[1]
    teleop_puppet(side)

