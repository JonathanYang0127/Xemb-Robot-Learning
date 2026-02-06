#!/usr/bin/env python3
import time
import sys
import evdev
from evdev import ecodes as ec  # Renamed to ec to avoid conflicts
import numpy as np
import argparse

# Import the unpacking function
from packing_utils_full import unpack_values_from_axes, get_heartbeat

def find_virtual_controller():
    """Find the virtual Xbox controller."""
    target_name_keywords = ["xbox", "x-box", "360", "gamepad", "controller"]
    devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
    
    print("\nAvailable input devices:")
    for dev in devices:
        print(f"  - {dev.name} (path: {dev.path})")
    
    for device in devices:
        if any(keyword in device.name.lower() for keyword in target_name_keywords):
            print(f"\nFound virtual controller: {device.name}")
            print(f"Device path: {device.path}")
            return device
            
    print("No virtual controller found. Make sure the sender script is running.")
    sys.exit(1)

def monitor_controller(device, verbose=False):
    """Monitor the virtual controller and print received events."""
    print(f"\nMonitoring controller: {device.name}")
    print("Press Ctrl+C to stop...")
    
    # Dictionaries to store axis state data for each heartbeat
    axis_states = [{}, {}, {}, {}]  # One dict for each heartbeat value
    
    # For raw event display
    last_print_time = time.time()
    event_count = 0
    
    try:
        # Don't grab the device to avoid conflicts
        # device.grab()
        
        while True:
            # Read all available events
            events = []
            try:
                # Use read_one() instead of read() to avoid generator issues
                while True:
                    event = device.read_one()
                    if event is None:
                        break
                    events.append(event)
            except Exception as err:
                print(f"Error reading events: {err}")
                time.sleep(0.1)
                continue
            
            # Count events
            event_count += len(events)
            
            # Print raw event info periodically
            now = time.time()
            if now - last_print_time >= 1.0:
                print(f"Received {event_count} events in the last second")
                last_print_time = now
                event_count = 0
            
            # Process events
            for event in events:
                if verbose:
                    print(f"Event: {event}")
                
                if event.type == ec.EV_ABS:  # Using ec instead of e
                    # Show the actual value
                    if event.code == ec.ABS_X:  # Using ec instead of e
                        print(f"X-axis: {event.value}")
                    elif event.code == ec.ABS_Y:  # Using ec instead of e
                        print(f"Y-axis: {event.value}")
                    elif event.code == ec.ABS_RX:  # Using ec instead of e
                        print(f"RX-axis: {event.value}")
                    elif event.code == ec.ABS_RY:  # Using ec instead of e
                        print(f"RY-axis: {event.value}")
                    elif event.code == ec.ABS_Z:  # Using ec instead of e
                        heartbeat = get_heartbeat(event.value)
                        print(f"Z-axis: {event.value} (Heartbeat: {heartbeat})")
                        
                    # Update axis states for this heartbeat when we get a Z event
                    if event.code == ec.ABS_Z:  # Using ec instead of e
                        heartbeat = get_heartbeat(event.value)
                        axis_states[heartbeat][event.code] = event.value
                    else:
                        # For other axis events, update all heartbeat states
                        # This helps ensure we have the latest values
                        for hb in range(4):
                            axis_states[hb][event.code] = event.value
                
                # Try to unpack after each ABS_Z event which indicates a new heartbeat
                if event.type == ec.EV_ABS and event.code == ec.ABS_Z:  # Using ec instead of e
                    heartbeat = get_heartbeat(event.value)
                    
                    # Make sure we have all the required axes for this heartbeat
                    required_axes = {ec.ABS_X, ec.ABS_Y, ec.ABS_RX, ec.ABS_Z}  # Using ec instead of e
                    if heartbeat == 1 or heartbeat == 3:  # Heartbeats with gripper info
                        required_axes.add(ec.ABS_RY)  # Using ec instead of e
                    
                    # Only try to unpack if we have all required axes
                    if all(axis in axis_states[heartbeat] for axis in required_axes):
                        # Extract joint and gripper data from the current heartbeat
                        jl, jr, gl, gr, ks = unpack_values_from_axes(axis_states[heartbeat])
                        
                        # Display the unpacked values
                        print(f"\nHeartbeat {heartbeat} values:")
                        if jl is not None:
                            left_values = [v if v is not None else "None" for v in jl]
                            print(f"  Left joints: {left_values}")
                        if jr is not None:
                            right_values = [v if v is not None else "None" for v in jr]
                            print(f"  Right joints: {right_values}")
                        if gl is not None:
                            print(f"  Left gripper: {gl}")
                        if gr is not None:
                            print(f"  Right gripper: {gr}")
                        if ks is not None:
                            active_keys = [k for k, v in ks.items() if v]
                            print(f"  Keys pressed: {active_keys}")
                    else:
                        missing = [axis for axis in required_axes if axis not in axis_states[heartbeat]]
                        print(f"Incomplete data for heartbeat {heartbeat}, missing axes: {missing}")
            
            # Avoid hogging the CPU
            time.sleep(0.001)
            
    except KeyboardInterrupt:
        print("\nMonitoring stopped by user.")
    except Exception as err:  # Using err instead of e
        print(f"\nError: {err}")
    finally:
        print("\nMonitoring ended.")

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Simple controller test receiver')
    parser.add_argument('--verbose', action='store_true',
                        help='Print detailed event information')
    
    args = parser.parse_args()
    
    # Find and monitor the virtual controller
    device = find_virtual_controller()
    monitor_controller(device, args.verbose)

if __name__ == "__main__":
    main()
