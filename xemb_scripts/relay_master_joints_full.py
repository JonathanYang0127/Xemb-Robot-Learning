#!/usr/bin/env python3
import time
import sys
import evdev
from evdev import UInput, ecodes as e
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from interbotix_xs_msgs.msg import JointSingleCommand
from constants import MASTER2PUPPET_JOINT_FN, DT, START_ARM_POSE_LEFT, START_ARM_POSE_RIGHT, MASTER_GRIPPER_JOINT_MID, PUPPET_GRIPPER_JOINT_CLOSE, MASTER_GRIPPER_JOINT_CLOSE
from robot_utils import torque_on, torque_off, move_arms, move_grippers, get_arm_gripper_positions
from packing_utils_full import pack_values_into_axes, unpack_values_from_axes 
import threading
import os
from pynput import keyboard

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


def press_to_start(master_bots):
    """Wait for both grippers to be closed to start."""
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


def controller_emit_all(controller, packed_values):
    """Write all axes and buttons to the controller every cycle."""
    # Write all axis values in the packed_values dictionary
    for event_code, value in packed_values.items():
        if event_code in (e.ABS_X, e.ABS_Y, e.ABS_RX, e.ABS_RY, e.ABS_Z):
            controller.write(e.EV_ABS, event_code, value)
    
    # Single sync after all writes
    controller.syn()


class KeyboardHandler:
    """Class to handle keyboard input using pynput."""
    def __init__(self):
        # Dictionary to track key states
        self.key_states = {
            'w': False,
            'a': False, 
            's': False,
            'd': False,
            'up': False,
            'left': False,
            'down': False,
            'right': False
        }
        
        # Flag to control the main loop
        self.running = True
        
        # Start keyboard listener
        self.listener = keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release
        )
        self.listener.start()
        
        print("Keyboard listener started")
        print("W/A/S/D and arrow keys are now active.")
        print("Press ESC to quit.")
    
    def _on_press(self, key):
        """Handle key press events."""
        try:
            # For normal keys
            key_char = key.char.lower()
            if key_char in 'wasd':
                self.key_states[key_char] = True
                print(f"Key {key_char} pressed")
        except (AttributeError, TypeError):
            # For special keys
            if key == keyboard.Key.up:
                self.key_states['up'] = True
                print("Up arrow pressed")
            elif key == keyboard.Key.down:
                self.key_states['down'] = True
                print("Down arrow pressed")
            elif key == keyboard.Key.left:
                self.key_states['left'] = True
                print("Left arrow pressed")
            elif key == keyboard.Key.right:
                self.key_states['right'] = True
                print("Right arrow pressed")
            elif key == keyboard.Key.esc:
                # Exit on ESC
                self.running = False
                print("ESC pressed, exiting...")
                return False
    
    def _on_release(self, key):
        """Handle key release events."""
        try:
            # For normal keys
            key_char = key.char.lower()
            if key_char in 'wasd':
                self.key_states[key_char] = False
                print(f"Key {key_char} released")
        except (AttributeError, TypeError):
            # For special keys
            if key == keyboard.Key.up:
                self.key_states['up'] = False
                print("Up arrow released")
            elif key == keyboard.Key.down:
                self.key_states['down'] = False
                print("Down arrow released")
            elif key == keyboard.Key.left:
                self.key_states['left'] = False
                print("Left arrow released")
            elif key == keyboard.Key.right:
                self.key_states['right'] = False
                print("Right arrow released")
    
    def shutdown(self):
        """Stop the keyboard listener."""
        self.listener.stop()
        self.listener.join()
        print("Keyboard listener stopped")


def teleop():
    """Runs teleoperation with both left and right robot arms."""
    # Initialize both robot arms
    master_left = InterbotixManipulatorXS(
        robot_model="wx250s", 
        group_name="arm",
        gripper_name="gripper", 
        robot_name='master_left', 
        init_node=True
    )
    
    master_right = InterbotixManipulatorXS(
        robot_model="wx250s", 
        group_name="arm",
        gripper_name="gripper", 
        robot_name='master_right',  # No init_node since already initialized
        init_node=False
    )

    # Prepare both robots
    master_bots = {'left': master_left, 'right': master_right}
    for arm_type, bot in master_bots.items():
        prep_master_robot(bot, arm_type=arm_type)
    master_bots = [master_left, master_right]

    # Create the virtual controller
    controller = create_virtual_controller()
    print("Virtual controller created. Waiting for press-to-start...")

    # Wait for the user to close both grippers to start
    press_to_start(master_bots)

    # Signal start with controller button press
    controller.write(e.EV_KEY, PRESS_TO_START_BUTTON, 1)
    controller.syn()
    time.sleep(0.2)
    controller.write(e.EV_KEY, PRESS_TO_START_BUTTON, 0)
    controller.syn()

    # Initialize keyboard handler
    keyboard_handler = KeyboardHandler()
    
    # Main teleoperation loop
    heartbeat = 0
    
    try:
        while keyboard_handler.running:
            # Read joint positions for left arm
            master_left_joints = master_left.dxl.joint_states.position[:6]
            master_left_gripper = master_left.dxl.joint_states.position[6]
            
            # Read joint positions for right arm
            master_right_joints = master_right.dxl.joint_states.position[:6]
            master_right_gripper = master_right.dxl.joint_states.position[6]
            
            # Pack values into axes based on current heartbeat
            packed_values = pack_values_into_axes(
                master_left_joints, 
                master_right_joints,
                master_left_gripper,
                master_right_gripper,
                heartbeat,
                keyboard_handler.key_states  # Pass the current keyboard state
            )
            
            # Emit all values
            controller_emit_all(controller, packed_values)
            
            # Update heartbeat (0-3)
            heartbeat = (heartbeat + 1) % 4
            
            # Maintain loop timing
            time.sleep(SLEEP_TIME)
            
            # Optional: print active keys (for debugging)
            active_keys = [k for k, v in keyboard_handler.key_states.items() if v]
            if active_keys:
                print(f"Active keys: {active_keys}")
                
    except KeyboardInterrupt:
        print("Teleoperation stopped by keyboard interrupt.")
    finally:
        # Clean up
        print("Shutting down...")
        keyboard_handler.shutdown()
        controller.close()
        for bot in master_bots:
            bot.shutdown()
        print("Cleanup complete.")


if __name__ == '__main__':
    teleop()
