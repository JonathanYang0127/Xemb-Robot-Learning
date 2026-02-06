# Calibration Guide: WidowX Master to PiperX Puppet

## Overview

This guide explains how to calibrate the WidowX master and PiperX puppet robots for teleoperation. The calibration process creates a mapping between the two robots' joint coordinate systems.

## Important: Understanding Calibration Positions

### What Are "Calibration Positions"?

The calibration process records **two corresponding signed positions** for each joint, NOT minimum and maximum limits. These positions are used to create a mapping between the master and puppet robots.

### Key Points:
- **Position 1** and **Position 2** are arbitrary reference points
- They represent the **same physical pose** on both robots
- They can be in any order (Position 1 doesn't need to be < Position 2)
- They are used to scale movements proportionally between robots

## Calibration Process

### Step 1: Prepare Both Robots

1. **WidowX Master**: Connect and power on
2. **PiperX Puppet**: Connect CAN interfaces and power on
3. **Ensure both robots can move freely** (no obstructions)

### Step 2: Choose Calibration Poses

Select two distinct physical poses that both robots can achieve:

**Recommended Poses:**
1. **Pose A**: Arms extended forward, grippers closed
2. **Pose B**: Arms folded back, grippers open

**Important:** The exact poses don't matter, but they should:
- Be achievable by both robots
- Be significantly different from each other
- Be easily reproducible

### Step 3: Calibrate WidowX Master

```bash
cd piperx_control
python widowx_callibrate.py
```

**Process:**
1. **Neutral Position**: Move robot to natural rest position
2. **Pose A**: Move robot to first calibration pose
3. **Pose B**: Move robot to second calibration pose
4. **Gripper**: Record open and closed positions

### Step 4: Calibrate PiperX Puppet

```bash
cd piperx_control
python piper_callibrate.py
```

**Process:**
1. **Neutral Position**: Move robot to natural rest position
2. **Pose A**: Move robot to **same physical pose** as WidowX Pose A
3. **Pose B**: Move robot to **same physical pose** as WidowX Pose B
4. **Gripper**: Record open and closed positions

### Step 5: Verify Calibration

```bash
cd piperx_control
python test_mapper.py
```

This will verify that:
- Neutral positions map correctly
- Calibration positions correspond properly
- Joint mapping is working as expected

## Example Calibration Session

### WidowX Calibration Output:
```
=== WidowX Calibration Script ===

STEP 1: NEUTRAL POSITION
Move the robot to NEUTRAL position and press Enter...
Neutral position recorded:
  Joint 0 (waist): 0.037 rad
  Joint 1 (shoulder): -1.489 rad
  [...]

STEP 2: JOINT CALIBRATION POSITIONS
For each joint, you will record two corresponding positions that
will be used to map between master and puppet robots.
These are NOT min/max limits, but specific calibration poses.

--- Joint 0: waist ---
Move Joint 0 (waist) to FIRST calibration position and press Enter...
Joint 0 (waist): -1.525 rad

Move Joint 0 (waist) to SECOND calibration position and press Enter...
Joint 0 (waist): 1.546 rad

Joint 0 (waist) calibration positions: [-1.525, 1.546] rad
```

### PiperX Calibration Output:
```
=== PiperX Calibration Script ===

STEP 1: NEUTRAL POSITION
Move the robot to NEUTRAL position and press Enter...
Neutral position recorded:
  Joint 1 (joint_1): -0.003 rad
  Joint 2 (joint_2): 0.166 rad
  [...]

STEP 2: JOINT CALIBRATION POSITIONS
For each joint, you will record two corresponding positions that
will be used to map between master and puppet robots.
These are NOT min/max limits, but specific calibration poses.

--- Joint 1: joint_1 ---
Move Joint 1 (joint_1) to FIRST calibration position and press Enter...
Joint 1 (joint_1): -1.597 rad

Move Joint 1 (joint_1) to SECOND calibration position and press Enter...
Joint 1 (joint_1): 1.601 rad

Joint 1 (joint_1) calibration positions: [-1.597, 1.601] rad
```

## Calibration Tips

### For Consistent Results:
1. **Use the same physical poses** for both robots
2. **Record positions carefully** - small errors affect mapping quality
3. **Choose distinct poses** - similar poses reduce mapping accuracy
4. **Document your poses** - write down what poses you used

### If Calibration Seems Wrong:
1. **Check physical poses match** between robots
2. **Verify neutral positions** are reasonable rest positions
3. **Ensure calibration positions are distinct** from neutral
4. **Re-run test_mapper.py** to verify mapping

## Troubleshooting

### Common Issues:

**Problem**: Mapped positions seem inverted
**Solution**: The calibration positions may be recorded in opposite order. This is normal - the mapping will handle the sign differences.

**Problem**: Robot moves to wrong position
**Solution**: Verify that both robots were in the same physical pose when recording calibration positions.

**Problem**: Mapping seems non-linear
**Solution**: Ensure calibration positions are significantly different from neutral position.

### Validation Commands:

```bash
# Test basic mapping
python test_mapper.py

# Test neutral position movement
python test_piperx_neutral.py

# View calibration data
cat widowx_callibration.json
cat piperx_callibration.json
```

## File Structure

After calibration, you'll have:
- `widowx_callibration.json` - Master robot calibration
- `piperx_callibration.json` - Puppet robot calibration
- `*_backup_*.json` - Backup files (if you re-calibrated)

## Next Steps

Once calibration is complete:
1. **Test the mapping** with `test_mapper.py`
2. **Test neutral movement** with `test_piperx_neutral.py`
3. **Run teleoperation** with master and puppet scripts

The calibration creates a robust mapping that handles different coordinate systems, joint directions, and scaling between the two robots while preserving the kinematic relationship. 