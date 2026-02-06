#!/usr/bin/env python3
import can
import time

# Set your CAN interface name
CAN_CHANNEL = "can0"

# This is the Piper CAN ID for control mode (usually 0x01 or similar)
CONTROL_MODE_CAN_ID = 0x01

# Control mode value: 0x01 = Position Control
CONTROL_MODE_VALUE = 0x01

def main():
    print("Connecting to CAN bus...")
    try:
        bus = can.interface.Bus(channel=CAN_CHANNEL, bustype='socketcan')
    except Exception as e:
        print(f"❌ Failed to connect to CAN bus: {e}")
        return

    msg = can.Message(
        arbitration_id=CONTROL_MODE_CAN_ID,
        data=[CONTROL_MODE_VALUE],
        is_extended_id=False
    )

    try:
        print(f"Sending control mode enable command: {hex(CONTROL_MODE_VALUE)}")
        bus.send(msg)
        print("✅ Enable command sent!")
    except can.CanError as e:
        print(f"❌ CAN send error: {e}")

if __name__ == "__main__":
    main()
