# Dual-Arm + Full Hardware Control Scripts

This document describes the dedicated dual-arm control scripts that require all hardware components (2 arms, stepper motor, and base robot). These scripts are optimized versions that remove all flexibility/optional components for maximum performance.

## Overview

The "dual-full" scripts are streamlined versions of the flexible scripts, designed for production use where all hardware is guaranteed to be present. They provide:

- **Better Performance**: No conditional checks or optional component handling
- **Fail-Fast Behavior**: Immediate errors if any hardware is missing
- **Simplified Configuration**: Hardcoded optimizations for dual-arm + full hardware
- **Enhanced Control**: Improved key mappings and hardware integration

## Hardware Requirements

### **MANDATORY COMPONENTS** (All Required)
- 2 × PiperX robotic arms (CAN interfaces: can0, can1)
- 2 × WidowX master arms (for teleoperation)
- 1 × Stepper motor controller (Arduino-based)
- 1 × Mobile base robot (MDC2460-based)
- Network connection between master and puppet computers

### **Software Requirements**
- `piper-sdk` (for PiperX control)
- `interbotix_xs_modules` (for WidowX control)
- All stepper and base control modules
- Valid calibration files

## New Scripts Created

### 1. **PiperX Puppet Control**
**File**: `piperx_control/puppet_follow_piperx_dual_full.py`

Dedicated dual-arm PiperX puppet controller with mandatory stepper and base.

**Features**:
- Dual PiperX arm control (can0 + can1)
- Mandatory stepper motor integration
- Mandatory base robot integration
- Enhanced key controls with variable step sizes
- Hardware health monitoring
- Fail-fast error handling

**Usage**:
```bash
# Basic usage (defaults: can0/can1, /dev/tty_stepper, /dev/tty_base)
python piperx_control/puppet_follow_piperx_dual_full.py

# Custom hardware ports
python piperx_control/puppet_follow_piperx_dual_full.py \
    --stepper-port /dev/ttyACM0 \
    --base-port /dev/ttyACM1 \
    --smoothing-alpha 0.9

# Custom CAN interfaces
python piperx_control/puppet_follow_piperx_dual_full.py \
    --can-left can2 \
    --can-right can3
```

### 2. **WidowX Master Control**
**File**: `piperx_control/relay_master_widowx_dual_full.py`

Dedicated dual-arm WidowX master controller for controlling PiperX puppets.

**Features**:
- Dual WidowX master arm control
- Enhanced keyboard controls for stepper/base
- Gripper-lock functionality for both arms
- Hardware validation and health monitoring
- Session management with puppet

**Usage**:
```bash
# Basic usage
python piperx_control/relay_master_widowx_dual_full.py

# With gripper locks and custom puppet IP
python piperx_control/relay_master_widowx_dual_full.py \
    --puppet-ip 192.168.1.100 \
    --gripper-lock-left \
    --gripper-lock-right

# Skip initial arm movement (if arms already positioned)
python piperx_control/relay_master_widowx_dual_full.py \
    --no-move-left \
    --no-move-right
```

### 3. **WidowX Puppet Control**
**File**: `xemb_scripts/puppet_follow_widowx_dual_full.py`

Dedicated dual-arm WidowX puppet controller (alternative to PiperX puppets).

**Features**:
- Dual WidowX puppet arm control
- Same stepper/base integration as PiperX version
- Joint limit enforcement for safety
- ROS-based control system

**Usage**:
```bash
# Basic usage
python xemb_scripts/puppet_follow_widowx_dual_full.py

# Custom hardware configuration
python xemb_scripts/puppet_follow_widowx_dual_full.py \
    --stepper-port /dev/ttyUSB0 \
    --base-port /dev/ttyUSB1 \
    --smoothing-alpha 0.7
```

### 4. **Hardware Test Script**
**File**: `piperx_control/test_dual_full_hardware.py`

Comprehensive test script to validate all hardware before operation.

**Features**:
- Tests all PiperX and WidowX connections
- Validates stepper and base controllers
- Checks calibration files
- Tests network connectivity
- Provides detailed pass/fail report

**Usage**:
```bash
# Run complete hardware validation
python piperx_control/test_dual_full_hardware.py
```

## Enhanced Control Scheme

### **Keyboard Controls**
The dual-full scripts feature enhanced keyboard controls:

| Key | Function | Notes |
|-----|----------|-------|
| **W** | Stepper large step UP | 100 steps |
| **S** | Stepper large step DOWN | 100 steps |
| **↑** | Stepper fine step UP | 10 steps |
| **↓** | Stepper fine step DOWN | 10 steps |
| **A** | Base turn LEFT | Continuous while held |
| **D** | Base turn RIGHT | Continuous while held |
| **←** | Base move BACKWARD | Continuous while held |
| **→** | Base move FORWARD | Continuous while held |
| **ESC** | Emergency stop | Exits program |

### **Gripper Lock Feature**
When enabled with `--gripper-lock-left` or `--gripper-lock-right`:
- **Closed gripper** = Arm LOCKED (torque on, position held)
- **Open gripper** = Arm UNLOCKED (torque off, free movement)

## Key Differences from Flexible Versions

### **Removed Features**
- ❌ Single-arm mode (`--single-arm`)
- ❌ Optional hardware flags (`--no-stepper`, `--no-base`)
- ❌ Mock hardware classes
- ❌ Hardware availability checks
- ❌ Graceful degradation to mock objects
- ❌ Complex argument parsing for optional components

### **Added/Enhanced Features**
- ✅ Fail-fast hardware validation
- ✅ Enhanced key controls with variable step sizes
- ✅ Continuous base movement (not discrete steps)
- ✅ Hardware health monitoring during operation
- ✅ Emergency shutdown procedures
- ✅ Optimized control loops (no conditional checks)
- ✅ Session management and validation
- ✅ Comprehensive test script

### **Performance Improvements**
- **Faster control loops**: No optional component checks
- **Lower latency**: Streamlined data processing
- **More responsive**: Higher default smoothing alpha (0.8 vs 0.15)
- **Better reliability**: Hardware monitoring and fail-fast behavior

## Setup and Operation

### **1. Hardware Validation**
Always run the test script first:
```bash
python piperx_control/test_dual_full_hardware.py
```

### **2. Start Puppet**
On the puppet computer:
```bash
python piperx_control/puppet_follow_piperx_dual_full.py
```

### **3. Start Master**
On the master computer:
```bash
python piperx_control/relay_master_widowx_dual_full.py --puppet-ip <PUPPET_IP>
```

### **4. Initialization**
1. Both arms will move to neutral positions
2. Close both master grippers to start teleoperation
3. Use keyboard for stepper/base control
4. Press ESC to exit

## Troubleshooting

### **Common Issues**

**"Hardware required for dual-full mode"**
- Missing stepper or base control modules
- Solution: Install required hardware drivers

**"FATAL ERROR: Could not connect to [arm] on [interface]"**
- CAN interface not available or robot not powered
- Check CAN interfaces: `ip link show`
- Verify robot power and connections

**"Connection timeout for [interface]"**
- Robot not responding or wrong CAN interface
- Try different interface (can0, can1, etc.)
- Check robot firmware and connections

**"Lost connection to [arm] arm"**
- Hardware disconnected during operation
- Check cables and power
- Restart both scripts

### **Hardware Health Monitoring**
The scripts include automatic health checks:
- **Every 5 seconds**: Robot connection status
- **Real-time**: Joint position validation  
- **On error**: Emergency shutdown procedure

### **Emergency Procedures**
If something goes wrong:
1. **Press ESC** for immediate exit
2. **Ctrl+C** for keyboard interrupt
3. **Power off** robots if unresponsive
4. Check hardware connections before restarting

## Configuration Files

### **Required Calibration Files**
- `piperx_calibration.json` - PiperX neutral positions and limits
- `widowx_calibration.json` - WidowX neutral positions and limits

### **Example Hardware Configuration**
```bash
# Typical hardware setup
CAN interfaces: can0 (left arm), can1 (right arm)
Stepper port: /dev/tty_stepper
Base port: /dev/tty_base
Network: Tailscale VPN between master/puppet
```

## Performance Tuning

### **Smoothing Alpha**
Controls responsiveness vs smoothness:
- `0.1` = Very smooth, slow response
- `0.5` = Balanced
- `0.8` = **Default** - responsive but stable
- `0.9` = Very responsive, may be jittery

### **Control Frequency**
- Master: 30Hz (hardcoded for stability)
- Puppet: 50Hz (hardcoded for responsiveness)

### **Network Optimization**
- Use wired connections when possible
- Tailscale VPN recommended for remote operation
- Monitor network latency with ping tests

---

**Created**: December 2024  
**Last Updated**: December 2024  
**Version**: 1.0