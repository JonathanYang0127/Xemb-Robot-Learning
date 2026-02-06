#!/usr/bin/env python3
import time
import sys
import evdev
from evdev import UInput, ecodes as e
import numpy as np
import argparse
import threading
import queue

# Import the packing function - using packing_utils_full as specified
from packing_utils_full import pack_values_into_axes

# Control parameters
CONTROL_PERIOD = 0.02  # 50 Hz
MIN_VAL = -3
MAX_VAL = 3

# Global queue for test values
test_queue = queue.Queue()
exit_event = threading.Event()


def create_virtual_controller():
    """Create a virtual game controller using evdev's UInput with Xbox 360 configuration."""
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


def controller_thread(controller):
    """Thread function to continuously send controller events."""
    heartbeat = 0
    
    while not exit_event.is_set():
        # Check if we have new test values to send
        try:
            # Non-blocking get to check for new values
            test_data = test_queue.get_nowait()
            
            # Extract the values from the test data
            joint_values_left = test_data["joint_left"]
            joint_values_right = test_data["joint_right"]
            gripper_left = test_data["gripper_left"]
            gripper_right = test_data["gripper_right"]
            key_states = test_data["key_states"]
            
            # Send values for all four heartbeats
            for hb in range(4):
                # Pack values into axis states
                packed_values = pack_values_into_axes(
                    joint_values_left, joint_values_right, 
                    gripper_left, gripper_right, 
                    hb, key_states
                )
                
                # Send all events at once
                controller_emit_all(controller, packed_values)
                
                # Small delay between heartbeats
                time.sleep(0.001)
            
            # Mark task as done
            test_queue.task_done()
        except queue.Empty:
            # If no new test data, keep sending the last heartbeat
            # This maintains the connection but doesn't change values
            packed_values = {e.ABS_Z: heartbeat * 50}
            controller_emit_all(controller, packed_values)
            heartbeat = (heartbeat + 1) % 4
        
        # Sleep to maintain desired control rate
        time.sleep(CONTROL_PERIOD)


def run_interactive_test():
    """Run an interactive test sequence letting the user examine each step."""
    print("\nStarting interactive test sequence. Press Enter after each test to continue.")
    print("Press Ctrl+C at any time to exit.\n")
    
    # Set of test cases
    test_cases = [
        {
            "name": "Neutral position",
            "joint_left": [0, 0, 0, 0, 0, 0],
            "joint_right": [0, 0, 0, 0, 0, 0],
            "gripper_left": 0,
            "gripper_right": 0,
            "key_states": {
                'w': False, 'a': False, 's': False, 'd': False,
                'up': False, 'left': False, 'down': False, 'right': False
            }
        },
        {
            "name": "Left arm joint 1 max",
            "joint_left": [3, 0, 0, 0, 0, 0],
            "joint_right": [0, 0, 0, 0, 0, 0],
            "gripper_left": 0,
            "gripper_right": 0,
            "key_states": {
                'w': False, 'a': False, 's': False, 'd': False,
                'up': False, 'left': False, 'down': False, 'right': False
            }
        },
        {
            "name": "Right arm joint 1 max",
            "joint_left": [0, 0, 0, 0, 0, 0],
            "joint_right": [3, 0, 0, 0, 0, 0],
            "gripper_left": 0,
            "gripper_right": 0,
            "key_states": {
                'w': False, 'a': False, 's': False, 'd': False,
                'up': False, 'left': False, 'down': False, 'right': False
            }
        },
        {
            "name": "Grippers test",
            "joint_left": [0, 0, 0, 0, 0, 0],
            "joint_right": [0, 0, 0, 0, 0, 0],
            "gripper_left": 3,
            "gripper_right": 3,
            "key_states": {
                'w': False, 'a': False, 's': False, 'd': False,
                'up': False, 'left': False, 'down': False, 'right': False
            }
        },
        {
            "name": "W key (stepper forward)",
            "joint_left": [0, 0, 0, 0, 0, 0],
            "joint_right": [0, 0, 0, 0, 0, 0],
            "gripper_left": 0,
            "gripper_right": 0,
            "key_states": {
                'w': True, 'a': False, 's': False, 'd': False,
                'up': False, 'left': False, 'down': False, 'right': False
            }
        },
        {
            "name": "Up arrow (base forward)",
            "joint_left": [0, 0, 0, 0, 0, 0],
            "joint_right": [0, 0, 0, 0, 0, 0],
            "gripper_left": 0,
            "gripper_right": 0,
            "key_states": {
                'w': False, 'a': False, 's': False, 'd': False,
                'up': True, 'left': False, 'down': False, 'right': False
            }
        },
        {
            "name": "Multiple joints",
            "joint_left": [1, -1, 0.5, -0.5, 0.25, -0.25],
            "joint_right": [0.8, -0.8, 0.4, -0.4, 0.2, -0.2],
            "gripper_left": 1.5,
            "gripper_right": -1.5,
            "key_states": {
                'w': False, 'a': False, 's': False, 'd': False,
                'up': False, 'left': False, 'down': False, 'right': False
            }
        },
        {
            "name": "All joints max",
            "joint_left": [3, 3, 3, 3, 3, 3],
            "joint_right": [3, 3, 3, 3, 3, 3],
            "gripper_left": 3,
            "gripper_right": 3,
            "key_states": {
                'w': False, 'a': False, 's': False, 'd': False,
                'up': False, 'left': False, 'down': False, 'right': False
            }
        },
        {
            "name": "All joints min",
            "joint_left": [-3, -3, -3, -3, -3, -3],
            "joint_right": [-3, -3, -3, -3, -3, -3],
            "gripper_left": -3,
            "gripper_right": -3,
            "key_states": {
                'w': False, 'a': False, 's': False, 'd': False,
                'up': False, 'left': False, 'down': False, 'right': False
            }
        },
        {
            "name": "Multiple keys",
            "joint_left": [0, 0, 0, 0, 0, 0],
            "joint_right": [0, 0, 0, 0, 0, 0],
            "gripper_left": 0,
            "gripper_right": 0,
            "key_states": {
                'w': True, 'a': False, 's': False, 'd': False,
                'up': True, 'left': True, 'down': False, 'right': False
            }
        }
    ]
    
    try:
        # Run through each test case
        for i, test in enumerate(test_cases):
            print(f"Test {i+1}/{len(test_cases)}: {test['name']}")
            print("  Joint values left:", [f"{v:.2f}" for v in test["joint_left"]])
            print("  Joint values right:", [f"{v:.2f}" for v in test["joint_right"]])
            print(f"  Gripper left: {test['gripper_left']:.2f}")
            print(f"  Gripper right: {test['gripper_right']:.2f}")
            print("  Keys pressed:", [k for k, v in test["key_states"].items() if v])
            
            # Add the test to the queue for the controller thread to send
            test_queue.put(test)
            
            # Wait for user to press Enter before continuing
            input("\nPress Enter to continue to the next test (or Ctrl+C to exit)...")
            
            # Wait for the controller thread to finish sending this test
            test_queue.join()
            
            print("\n" + "-"*50 + "\n")
        
        print("All tests completed!")
        print("Press Ctrl+C to exit...")
        
        # Keep running until user presses Ctrl+C
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\nTest interrupted by user.")


def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Interactive controller test')
    parser.add_argument('--custom', action='store_true',
                        help='Enter custom test values interactively')
    
    args = parser.parse_args()
    
    try:
        # Create virtual controller
        print("Creating virtual controller...")
        controller = create_virtual_controller()
        
        # List all input devices to verify our controller was created
        devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
        print("\nAvailable input devices after creation:")
        for dev in devices:
            print(f"  - {dev.name} (path: {dev.path})")
        
        # Give udev time to set up the device
        print("Waiting 1 second for device setup...")
        time.sleep(1)
        
        # Start controller thread to continuously send events
        control_thread = threading.Thread(target=controller_thread, args=(controller,), daemon=True)
        control_thread.start()
        
        # Run the interactive test
        run_interactive_test()
        
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Signal the thread to exit
        exit_event.set()
        
        # Clean up
        try:
            if 'controller' in locals():
                controller.close()
                print("Virtual controller closed.")
        except:
            pass


if __name__ == "__main__":
    main()
