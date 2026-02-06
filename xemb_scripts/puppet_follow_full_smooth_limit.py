#!/usr/bin/env python3
import time
import sys
import evdev
from evdev import ecodes as e
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from interbotix_xs_msgs.msg import JointSingleCommand
from constants import (START_ARM_POSE, 
                       START_ARM_POSE_LEFT,
                       START_ARM_POSE_RIGHT,
                       MASTER_GRIPPER_JOINT_CLOSE, 
                       PUPPET_GRIPPER_JOINT_CLOSE, 
                       MASTER2PUPPET_JOINT_FN)
from robot_utils import torque_on, torque_off, move_arms, move_grippers, get_arm_gripper_positions
from packing_utils_full import pack_values_into_axes, unpack_values_from_axes, get_heartbeat

# Import the controller classes
from base_control.base_controller import MDCRobot
from stepper_control.stepper_controller import StepperController

# Control parameters
SMOOTH_ALPHA = 0.15  # Smoothing factor (lower = smoother)
CONTROL_PERIOD = 0.02  # 50 Hz
BASE_SPEED = 300  # Base movement speed

# Define joint limits for safety
# Format: [min_angle, max_angle] in radians for each joint
#These are example values - replace with actual limits for your robot
'''
JOINT_LIMITS_LEFT = [
    [-2.62, 2.62],   # waist
    [-1.7, 0.64],    # shoulder
    [-1.0, 1.57],    # elbow
    [-1.8, 1.8],     # forearm_roll
    [-1.57, 1.57],   # wrist_angle
    [-1.8, 1.8],     # wrist_rotate
]

JOINT_LIMITS_RIGHT = [
    [-2.62, 2.62],   # waist
    [-1.7, 0.64],    # shoulder
    [-1.0, 1.57],    # elbow
    [-1.8, 1.8],     # forearm_roll
    [-1.57, 1.57],   # wrist_angle
    [-1.8, 1.8],     # wrist_rotate
]
'''
JOINT_LIMITS_LEFT = [[-10, 10] for _ in range(6)]
JOINT_LIMITS_RIGHT = [[-10, 10] for _ in range(6)]


def enforce_joint_limits(joint_positions, limits):
    """
    Enforce joint limits on the given joint positions.
    
    Args:
        joint_positions (list): List of joint positions
        limits (list): List of [min, max] limits for each joint
        
    Returns:
        tuple: (limited_positions, all_within_limits)
        - limited_positions: Joint positions with enforced limits
        - all_within_limits: Boolean indicating if all joints were within limits
    """
    limited_positions = []
    all_within_limits = True
    
    for i, pos in enumerate(joint_positions):
        min_limit, max_limit = limits[i]
        
        # Check if position is outside limits
        if pos < min_limit or pos > max_limit:
            all_within_limits = False
            limited_pos = max(min_limit, min(max_limit, pos))
            print(f"Joint {i} limited from {pos:.2f} to {limited_pos:.2f}")
        else:
            limited_pos = pos
            
        limited_positions.append(limited_pos)
    
    return limited_positions, all_within_limits


def prep_puppet_robot(puppet_bot, arm_type='left'):
    """Prepare the puppet robot for teleoperation."""
    puppet_bot.dxl.robot_set_operating_modes("group", "arm", "position")
    puppet_bot.dxl.robot_set_operating_modes("single", "gripper", "current_based_position")
    
    torque_on(puppet_bot)
    
    # Move arm to starting position
    if arm_type == 'left':
        start_arm_qpos = START_ARM_POSE_LEFT[:6]
    elif arm_type == 'right':
        start_arm_qpos = START_ARM_POSE_RIGHT[:6]
    else:
        raise NotImplementedError
    move_arms([puppet_bot], [start_arm_qpos], move_time=1)
    move_grippers([puppet_bot], [PUPPET_GRIPPER_JOINT_CLOSE], move_time=0.5)


def get_gamepad():
    """Find and configure the virtual Xbox controller. Keeps checking until found."""
    target_name_keywords = ["xbox", "x-box", "360", "gamepad", "controller"]
    
    print("\nLooking for virtual controller...")
    
    while True:  # Keep checking until a controller is found
        devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
        
        print("Available input devices:")
        for dev in devices:
            print(f"  - {dev.name} (path: {dev.path})")
        
        for device in devices:
            if any(keyword in device.name.lower() for keyword in target_name_keywords):
                print(f"\nFound virtual controller: {device.name}")
                print(f"Device path: {device.path}")
                try:
                    device.grab()  # Take exclusive control
                    return device
                except Exception as e:
                    print(f"Error grabbing device: {e}")
                    # Continue searching if we can't grab this device
                    continue
        
        print("No virtual controller found. Will check again in 1 second...")
        time.sleep(1)  # Wait for 1 second before checking again


def process_key_states(key_states, stepper, base, last_states=None):
    """Process key states and control stepper and base accordingly."""
    if last_states is None:
        last_states = {
            'w': False, 's': False,
            'up': False, 'down': False, 'left': False, 'right': False
        }
    
    # Only send commands when key states change to reduce serial traffic
    # W key - stepper forward
    if key_states.get('w', False) and not last_states.get('w', False):
        stepper.forward()
        print("Stepper: Forward")
    # S key - stepper backward
    elif key_states.get('s', False) and not last_states.get('s', False):
        stepper.backward()
        print("Stepper: Backward")
    # Stop stepper if W and S are released
    elif (not key_states.get('w', False) and last_states.get('w', False)) or \
         (not key_states.get('s', False) and last_states.get('s', False)):
        stepper.stop()
        print("Stepper: Stop")
    
    # Up arrow - base forward
    if key_states.get('up', False):
        base.drive(speed=BASE_SPEED, turn=0)
        print("Base: Forward")
    # Down arrow - base backward
    elif key_states.get('down', False):
        base.drive(speed=-BASE_SPEED, turn=0)
        print("Base: Backward")
    # Left arrow - base turn left
    elif key_states.get('left', False):
        base.drive(speed=0, turn=BASE_SPEED)
        print("Base: Turn Left")
    # Right arrow - base turn right
    elif key_states.get('right', False):
        base.drive(speed=0, turn=-BASE_SPEED)
        print("Base: Turn Right")
    # Stop base if no arrow keys are pressed
    elif (last_states.get('up', False) or last_states.get('down', False) or
          last_states.get('left', False) or last_states.get('right', False)) and \
         not any(key_states.get(k, False) for k in ['up', 'down', 'left', 'right']):
        base.stop()
        print("Base: Stop")

    # Return the current key states to use as last_states next time
    return key_states.copy()


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
  
    stepper, base = None, None
    # Initialize stepper motor controller
    print("Initializing Stepper Controller...")
    stepper = StepperController(port="/dev/tty_stepper")
    stepper.enable()
    
    # Initialize base robot
    print("Initializing Base Controller...")
    base = MDCRobot(port='/dev/tty_base', default_speed=BASE_SPEED)
    base.connect()
    
    puppet_bots = {'left': puppet_left, 'right': puppet_right}
    for arm_type, bot in puppet_bots.items():
        prep_puppet_robot(bot, arm_type=arm_type)
    puppet_bots = [puppet_left, puppet_right]

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
    
    # Key states tracking
    last_key_states = None
    
    # For tracking if arms are at their limits
    left_arm_at_limit = False
    right_arm_at_limit = False
    
    try:
        print("Starting teleoperation...")
        print("Use W/S keys to control the stepper motor")
        print("Use arrow keys to control the base")
        print("Joint limits are enforced for safety")
        
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
                    key_states = None
                    for hb in range(4):
                        jl, jr, gl, gr, ks = unpack_values_from_axes(axis_states[hb])
                        
                        # Save key states if available (from heartbeat 0)
                        if ks is not None:
                            key_states = ks
                        
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
                    
                    # Process key states for stepper and base control
                    if key_states:
                        last_key_states = process_key_states(key_states, stepper, base, last_key_states)
            
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
            
            # After checking joint limits, skip updating positions if we're at a limit
            # This way we still send the clamped position once, but don't update anymore until
            # all joints are back within limits
            if left_arm_at_limit:
                # Skip updating left arm since it's at a limit
                limited_joint_left, left_within_limits = enforce_joint_limits(smoothed_joint_left, JOINT_LIMITS_LEFT)
                
                # Only resume control if all joints are back within limits
                if left_within_limits:
                    left_arm_at_limit = False
                    print("Left arm back within limits - resuming control")
                    puppet_left.arm.set_joint_positions(limited_joint_left, blocking=False)
            else:
                # Normal operation - check if we just hit a limit
                limited_joint_left, left_within_limits = enforce_joint_limits(smoothed_joint_left, JOINT_LIMITS_LEFT)
                puppet_left.arm.set_joint_positions(limited_joint_left, blocking=False)
                
                # If we just hit a limit, set the flag for next iteration
                if not left_within_limits:
                    left_arm_at_limit = True
                    print("Left arm joint limit reached - arm stopped until back in range")
            
            # Same logic for right arm
            if right_arm_at_limit:
                limited_joint_right, right_within_limits = enforce_joint_limits(smoothed_joint_right, JOINT_LIMITS_RIGHT)
                
                if right_within_limits:
                    right_arm_at_limit = False
                    print("Right arm back within limits - resuming control")
                    puppet_right.arm.set_joint_positions(limited_joint_right, blocking=False)
            else:
                limited_joint_right, right_within_limits = enforce_joint_limits(smoothed_joint_right, JOINT_LIMITS_RIGHT)
                puppet_right.arm.set_joint_positions(limited_joint_right, blocking=False)
                
                if not right_within_limits:
                    right_arm_at_limit = True
                    print("Right arm joint limit reached - arm stopped until back in range")
            
            # Calculate time to sleep to maintain 50Hz
            process_time = time.time() - now
            sleep_time = max(0, CONTROL_PERIOD - process_time)
            time.sleep(sleep_time)
            
            # Update timing for next iteration
            last_update_time = now
            
    except KeyboardInterrupt:
        print("\nTeleoperation stopped by user.")
    finally:
        # Stop and clean up all controllers
        print("Shutting down controllers...")
        stepper.stop()
        stepper.disable()
        stepper.close()
        
        base.stop()
        base.shutdown()
        
        for bot in puppet_bots:
            torque_off(bot)


if __name__ == '__main__':
    teleop_puppet()
