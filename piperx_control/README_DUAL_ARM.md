# PiperX Dual-Arm VR Control System

This directory contains the dual-arm versions of the VR control system that support **both left and right VR controllers** simultaneously controlling separate robot arms.

## Overview

The dual-arm system consists of three main components:
- **vr_server_dual.py**: Captures data from both left AND right VR controllers
- **vr_relay_dual.py**: Routes VR data to appropriate robot arms based on controller
- **puppet_follow_vr_dual.py**: Controls both robot arms with improved dual-arm support

## Controller Mapping

| VR Controller | Robot ID | Robot Arm | CAN Interface |
|---------------|----------|-----------|---------------|
| Left Controller | 0 | Left Arm | can0 (default) |
| Right Controller | 1 | Right Arm | can1 (default) |

## Quick Start

### 1. Start the VR Server (Dual-Arm)
```bash
cd piperx_control
python vr_server_dual.py
```

### 2. Start the VR Relay (Dual-Arm)
```bash
cd piperx_control
python vr_relay_dual.py --puppet-ip <ROBOT_IP>
```

### 3. Start the Puppet Control (Dual-Arm)
```bash
cd piperx_control
python puppet_follow_vr_dual.py
```

## Key Features

### Dual-Arm Capabilities
- **Independent Control**: Each VR controller operates its corresponding robot arm independently
- **Synchronized Operation**: Both arms can be controlled simultaneously
- **Automatic Routing**: Left controller data automatically routes to left arm, right to right arm
- **Per-Arm Status**: Separate status reporting for each arm

### Enhanced Base Control 🚗 **NEW**
- **Smooth Joystick Control**: Continuous, smooth base movement using VR controller joystick/thumbstick
- **Intelligent Dead Zone**: Configurable dead zone (default 10%) prevents unintended movement
- **Speed Curves**: Quadratic speed curve provides precise control at low speeds
- **Smooth Acceleration**: Gradual acceleration and deceleration for natural movement
- **Real-time Response**: 50Hz control loop for responsive movement
- **Universal Compatibility**: Works with both real and mock base controllers

### Enhanced Debugging
- **Controller-Specific Feedback**: Clear indication of which controller is controlling which robot
- **Per-Arm Packet Tracking**: Separate packet rate monitoring for left and right controllers
- **Improved Error Handling**: Better error messages with controller identification
- **Joystick Debugging**: Real-time joystick input visualization and base movement feedback

### Safety Features
- **Independent Reset**: Each controller can reset its corresponding arm independently
- **Graceful Degradation**: System continues to work if one controller/arm fails
- **Connection Monitoring**: Per-controller connection status tracking
- **Automatic Stop**: Base stops immediately when joystick is centered or VR disconnects

## Control Instructions

### Both Controllers Support:
- **Squeeze Button**: 🎯 Enable movement (hold to move robot)
- **Trigger**: 🤏 Gripper control (press=close, release=open)
- **X Button**: 🔄 Reset arm to neutral position
- **Y Button**: Available for future features

### Auxiliary Controls (Shared):
- **Menu Button**: Stepper motor control
- **🕹️ Joystick/Thumbstick**: 🚗 **NEW** Smooth base movement control
  - **Push forward/back**: Move base forward/backward
  - **Push left/right**: Turn base left/right
  - **Center joystick**: Stop base movement
  - **Features**: Dead zone, speed curves, smooth acceleration/deceleration

## Command Line Options

### vr_server_dual.py
```bash
python vr_server_dual.py [OPTIONS]

Options:
  --vr-port PORT          VR server port (default: 8012)
  --broadcast-port PORT   VR data broadcast port (default: 5006)
  --broadcast-rate HZ     Broadcast frequency (default: 60)
  --debug-prints          Enable debug output
```

### vr_relay_dual.py
```bash
python vr_relay_dual.py [OPTIONS]

Options:
  --puppet-ip IP          Robot IP address
  --frequency HZ          Control frequency (default: 50)
  --vr-timeout SECONDS    VR server timeout (default: 30)
  --debug-poses           Print pose debugging info
  --skip-init             Skip initialization handshake
```

### puppet_follow_vr_dual.py
```bash
python puppet_follow_vr_dual.py [OPTIONS]

Options:
  --single-arm            Use only left arm (fallback mode)
  --can-left INTERFACE    Left arm CAN interface (default: can0)
  --can-right INTERFACE   Right arm CAN interface (default: can1)
  --no-ik                 Disable inverse kinematics
  --debug                 Enable debug output
  --verbose-debug         Enable verbose debugging
```

## Troubleshooting

### Common Issues

1. **Only One Controller Working**
   - Check that both controllers are paired and connected to Meta Quest
   - Verify both controllers appear in the VR server output
   - Use `--debug-poses` to see which controllers are sending data

2. **Controller Mapping Issues**
   - Controllers are automatically mapped: Left→Robot0, Right→Robot1
   - If mapping seems wrong, check the debug output for robot_id values
   - Restart the VR server if controllers get mixed up

3. **One Arm Not Responding**
   - Check CAN interface connections (can0 for left, can1 for right)
   - Use `--single-arm` flag to test with just the left arm
   - Verify robot calibration files are present

4. **Performance Issues**
   - Dual-arm mode requires more processing power
   - Consider reducing `--broadcast-rate` if experiencing latency
   - Monitor per-controller packet rates in status output

### Debug Commands

```bash
# Test VR server with debug output
python vr_server_dual.py --debug-prints

# Test relay with pose debugging
python vr_relay_dual.py --debug-poses --puppet-ip <IP>

# Test puppet with verbose debugging
python puppet_follow_vr_dual.py --verbose-debug

# Single-arm fallback mode
python puppet_follow_vr_dual.py --single-arm
```

## Comparison with Single-Arm System

| Feature | Single-Arm | Dual-Arm |
|---------|------------|----------|
| Controllers | Left only | Left + Right |
| Robots | 1 arm | 2 arms |
| Packet Rate | ~60 Hz | ~120 Hz total |
| Resource Usage | Lower | Higher |
| Complexity | Simple | Advanced |

## Network Setup

1. **VR Server**: Runs on development machine (usually local)
2. **VR Relay**: Can run on development machine or robot
3. **Puppet Control**: Must run on robot with CAN interfaces

Ensure all components can communicate over the network and that the specified ports (5005, 5006) are open.

## File Structure

```
piperx_control/
├── vr_server_dual.py          # Dual-arm VR server
├── vr_relay_dual.py           # Dual-arm VR relay  
├── puppet_follow_vr_dual.py   # Dual-arm puppet control
├── vr_server.py               # Original single-arm VR server
├── vr_relay.py                # Original single-arm VR relay
├── puppet_follow_vr.py        # Original single-arm puppet control
└── README_DUAL_ARM.md         # This file
```

Use the `_dual.py` versions for dual-arm control and the original versions for single-arm control.