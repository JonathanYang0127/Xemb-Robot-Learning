# Xemb Robot Learning

### Repo Structure
- ``config``: a config for each robot, designating the port they should bind to, more details in quick start guide.
- ``launch``: a ROS launch file for all 4 cameras and all 4 robots.

## Quick start guide

### Hardware selection 

We suggest using a "heavy-duty" computer if possible. 

*In particular, at least 6 USB3 ports are needed. 4 ports for robot connections and 2 ports for cameras.* We have seen cases that a machine was not able to stably connect to all 4 robot arms simultaneously over USB, especially when USB hubs are used.

### Software selection -- OS:

Currently tested and working configurations: 
- :white_check_mark: Ubuntu 18.04 + ROS 1 noetic
- :white_check_mark: Ubuntu 20.04 + ROS 1 noetic

Ongoing testing (compatibility effort underway):
- :construction: ROS 2

### Software installation - ROS:
1. Install ROS and interbotix software following [this guide](https://docs.trossenrobotics.com/interbotix_xsarms_docs/).  
   **(before installing ROS, pip install catkin_msg pyyaml empy==3.3.4)**
2. This will create the directory ``~/interbotix_ws`` which contains ``src``.
3. git clone this repo inside ``~/interbotix_ws/src``
4. ``source /opt/ros/noetic/setup.sh && source ~/interbotix_ws/devel/setup.sh``
5. ``sudo apt-get install ros-noetic-usb-cam && sudo apt-get install ros-noetic-cv-bridge``
6. run ``catkin_make`` inside ``~/interbotix_ws``, make sure the build is successful
7. go to ``~/interbotix_ws/src/interbotix_ros_toolboxes/interbotix_xs_toolbox/interbotix_xs_modules/src/interbotix_xs_modules/arm.py``, find function ``publish_positions``.
   Change ``self.T_sb = mr.FKinSpace(self.robot_des.M, self.robot_des.Slist, self.joint_commands)`` to ``self.T_sb = None``.
   This prevents the code from calculating FK at every step which delays teleoperation.
### Hardware installation:

The goal of this section is to run ``roslaunch aloha 4arms_teleop.launch``, which starts
communication with 4 robots and 4 cameras. It should work after finishing the following steps:

Step 1: Connect 4 robots to the computer via USB, and power on. *Do not use extension cable or usb hub*.
- To check if the robot is connected, install dynamixel wizard [here](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/)
- Dynamixel wizard is a very helpful debugging tool that connects to individual motors of the robot. It allows
things such as rebooting the motor (very useful!), torque on/off, and sending commands.
However, it has no knowledge about the kinematics of the robot, so be careful about collisions.
The robot *will* collapse if motors are torque off i.e. there is no automatically engaged brakes in joints.
- Open Dynamixel wizard, go into ``options`` and select:
  - Protocal 2.0
  - All ports
  - 1000000 bps
  - ID range from 0-10
- Note: repeat above everytime before you scan.
- Then hit ``Scan``. There should be 4 devices showing up, each with 9 motors.


- One issue that arises is the port each robot binds to can change over time, e.g. a robot that
is initially ``ttyUSB0`` might suddenly become ``ttyUSB5``. To resolve this, we bind each robot to a fixed symlink
port with the following mapping:
  - ``ttyDXL_master_right``: right master robot (master: the robot that the operator would be holding)
  - ``ttyDXL_puppet_right``: right puppet robot (puppet: the robot that performs the task)
  - ``ttyDXL_master_left``: left master robot
  - ``ttyDXL_puppet_left``: left puppet robot
- Take ``ttyDXL_master_right``: right master robot as an example:
  1. Find the port that the right master robot is currently binding to, e.g. ``ttyUSB0``
  2. run ``udevadm info --name=/dev/ttyUSB0 --attribute-walk | grep serial`` to obtain the serial number. Use the first one that shows up, the format should look similar to ``FT6S4DSP``.
  3. ``sudo vim /etc/udev/rules.d/99-fixed-interbotix-udev.rules`` and add the following line: 

         SUBSYSTEM=="tty", ATTRS{serial}=="<serial number here>", ENV{ID_MM_DEVICE_IGNORE}="1", ATTR{device/latency_timer}="1", SYMLINK+="ttyDXL_master_right"

  4. This will make sure the right master robot is *always* binding to ``ttyDXL_master_right``
  5. Repeat with the rest of 3 arms.
- To apply the changes, run ``sudo udevadm control --reload && sudo udevadm trigger``
- If successful, you should be able to find ``ttyDXL*`` in your ``/dev``

Step 2: Set max current for gripper motors
- Open Dynamixel Wizard, and select the wrist motor for puppet arms. The name of it should be ```[ID:009] XM430-W350```
- Tip: the LED on the base of robot will flash when it is talking to Dynamixel Wizard. This will help determine which robot is selected. 
- Find ``38 Current Limit``, enter ``200``, then hit ``save`` at the bottom.
- Repeat this for both puppet robots.
- This limits the max current through gripper motors, to prevent overloading errors.


Step 3: Setup 4 cameras
- You may use usb hub here, but *maximum 2 cameras per hub for reasonable latency*.
- To make sure all 4 cameras are binding to a consistent port, similar steps are needed.
- Cameras are by default binding to ``/dev/video{0, 1, 2...}``, while we want to have symlinks ``{CAM_RIGHT_WRIST, CAM_LEFT_WRIST, CAM_LOW, CAM_HIGH}``
- Take ``CAM_RIGHT_WRIST`` as an example, and let's say it is now binding to ``/dev/video0``. run ``udevadm info --name=/dev/video0 --attribute-walk | grep serial`` to obtain it's serial. Use the first one that shows up, the format should look similar to ``0E1A2B2F``.
- Then ``sudo vim /etc/udev/rules.d/99-fixed-interbotix-udev.rules`` and add the following line 

      SUBSYSTEM=="video4linux", ATTRS{serial}=="<serial number here>", ATTR{index}=="0", ATTRS{idProduct}=="085c", ATTR{device/latency_timer}="1", SYMLINK+="CAM_RIGHT_WRIST"

- Repeat this for ``{CAM_LEFT_WRIST, CAM_LOW, CAM_HIGH}`` in additional to ``CAM_RIGHT_WRIST``
- To apply the changes, run ``sudo udevadm control --reload && sudo udevadm trigger``
- If successful, you should be able to find ``{CAM_RIGHT_WRIST, CAM_LEFT_WRIST, CAM_LOW, CAM_HIGH}`` in your ``/dev``

At this point, have a new terminal
    
    conda deactivate # if conda shows up by default
    source /opt/ros/noetic/setup.sh && source ~/interbotix_ws/devel/setup.sh
    roslaunch aloha 4arms_teleop.launch

If no error message is showing up, the computer should be successfully connected to all 4 cameras and all 4 robots.

#### Trouble shooting
- Make sure Dynamixel Wizard is disconnected, and no app is using webcam's stream. It will prevent ROS from connecting to
these devices.

### Software installation - Conda:

    conda create -n aloha python=3.8.10
    conda activate aloha
    pip install torchvision
    pip install torch
    pip install pyquaternion
    pip install pyyaml
    pip install rospkg
    pip install pexpect
    pip install mujoco==2.3.7
    pip install dm_control==1.0.14
    pip install opencv-python
    pip install matplotlib
    pip install einops
    pip install packaging
    pip install h5py

### Testing teleoperation

**Notice**: Before running the commands below, be sure to place all 4 robots in their sleep positions, and open master robot's gripper. 
All robots will rise to a height that is easy for teleoperation.

    # ROS terminal
    conda deactivate
    source /opt/ros/noetic/setup.sh && source ~/interbotix_ws/devel/setup.sh
    roslaunch aloha 4arms_teleop.launch
    
    # Right hand terminal
    conda activate aloha
    cd ~/interbotix_ws/src/aloha/aloha_scripts
    python3 one_side_teleop.py right
    
    # Left hand terminal
    conda activate aloha
    cd ~/interbotix_ws/src/aloha/aloha_scripts
    python3 one_side_teleop.py left

The teleoperation will start when the master side gripper is closed.


## Example Usages

To set up a new terminal, run:

    conda activate aloha
    cd ~/interbotix_ws/src/aloha/aloha_scripts


The ``one_side_teleop.py`` we ran is for testing teleoperation and has no data collection. To collect data for an episode, run:

    python3 record_episodes.py --dataset_dir <data save dir> --episode_idx 0

This will store a hdf5 file at ``<data save dir>``.
To change episode length and other params, edit ``constants.py`` directly.

To visualize the episode collected, run:

    python3 visualize_episodes.py --dataset_dir <data save dir> --episode_idx 0

To replay the episode collected with real robot, run:

    python3 replay_episodes.py --dataset_dir <data save dir> --episode_idx 0

To lower 4 robots before e.g. cutting off power, run:

    python3 sleep.py

## VR Teleoperation with PiperX

New support for Quest 3 VR teleoperation of PiperX robot arms using Cartesian control and inverse kinematics.

### System Architecture

The VR teleoperation system consists of three main components:

1. **`packing_utils_vr.py`** - VR data packing/unpacking utilities
2. **`relay_vr_controller.py`** - VR controller capture & UDP transmission 
3. **`puppet_follow_vr.py`** - PiperX robot control with inverse kinematics

### Hardware Requirements

- **VR Headset**: Meta Quest 3 with controllers
- **Robot**: PiperX arm(s) connected via CAN interface
- **Network**: WiFi connection between VR relay and puppet computers
- **Optional**: Stepper motor and mobile base controllers

### Software Dependencies

Install additional VR dependencies:

```bash
pip install vuer  # VR interface
pip install pinocchio  # Inverse kinematics (if not already installed)
pip install piper-sdk  # PiperX robot control
```

### Setup Instructions

#### 1. Network Setup

Install ngrok for VR connectivity:
```bash
# Install ngrok
curl -s https://ngrok-agent.s3.amazonaws.com/ngrok.snap | sudo snap install --dangerous ngrok

# Start ngrok tunnel
ngrok http 8012
```

#### 2. Robot Calibration

Ensure PiperX calibration files exist:
- `piperx_control/piperx_calibration.json` - Robot joint and gripper limits
- `piperx_control/widowx_calibration.json` - Master arm calibration (if using joint mapping)

#### 3. CAN Interface Setup

Configure CAN interfaces for PiperX robots:
```bash
# Set up CAN interfaces (typically can0, can1)
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0
sudo ip link set can1 type can bitrate 1000000
sudo ip link set up can1
```

### Running VR Teleoperation

#### Quick Start (Single Arm)

1. **Start the puppet robot controller**:
```bash
cd ~/interbotix_ws/src/aloha/piperx_control
python puppet_follow_vr.py --single-arm --can-left can0
```

2. **Start the VR relay controller**:
```bash
python relay_vr_controller.py --puppet-ip <PUPPET_ROBOT_IP>
```

3. **Connect VR headset**:
   - Open Quest browser
   - Navigate to the ngrok URL
   - Point controllers at screen to activate tracking

4. **Start teleoperation**:
   - Pull trigger to enable robot movement
   - Move controller to control robot end-effector
   - Use A/B buttons for gripper control

#### Advanced Usage (Dual Arm with Hardware)

```bash
# Puppet with full hardware support
python puppet_follow_vr.py \
    --can-left can0 \
    --can-right can1 \
    --debug

# Relay with pose debugging
python relay_vr_controller.py \
    --puppet-ip 192.168.1.100 \
    --robot-id 0 \
    --debug-poses \
    --frequency 60
```

### VR Control Interface

| **Input** | **Action** |
|-----------|------------|
| **Left Controller Position** | Robot end-effector position (X, Y, Z) |
| **Left Controller Orientation** | Robot end-effector orientation (Roll, Pitch, Yaw) |
| **Trigger (Analog)** | Enable movement + speed scaling |
| **A Button** | Open gripper |
| **B Button** | Close gripper |
| **Menu Button** | Stepper motor up |
| **Thumbstick** | Base robot turn |
| **Squeeze** | Emergency stop |

### Configuration Options

#### Puppet Robot (`puppet_follow_vr.py`)

```bash
python puppet_follow_vr.py [OPTIONS]

Options:
  --single-arm              Use only one robot arm
  --can-left TEXT          CAN interface for left arm (default: can0)
  --can-right TEXT         CAN interface for right arm (default: can1)
  --no-ik                  Disable inverse kinematics (joint mode only)
  --no-stepper             Disable stepper motor control
  --no-base                Disable mobile base control
  --debug                  Enable debug output
  --verbose-debug          Enable extremely verbose debug output
```

#### VR Relay (`relay_vr_controller.py`)

```bash
python relay_vr_controller.py [OPTIONS]

Options:
  --puppet-ip TEXT         IP address of puppet robot
  --robot-id INTEGER       Robot ID for multi-robot setups
  --frequency FLOAT        Control frequency in Hz (default: 50.0)
  --debug-poses           Print pose data for debugging
  --connection-timeout FLOAT  VR connection timeout in seconds
```

### Technical Features

#### Cartesian Control
- **Real-time inverse kinematics** using Pinocchio
- **Velocity-based control** for smooth motion
- **Joint limit checking** and collision avoidance
- **Reference frame management** with trigger-based activation

#### Communication Protocol
- **UDP networking** with session management
- **No-heartbeat mode** for high-frequency VR data
- **Efficient data compression** using 16-bit encoding
- **Robust error handling** and connection recovery

#### Safety Systems
- **Dead zone filtering** for small movements
- **Velocity limits** and position clamping
- **Emergency stop** via squeeze button
- **Graceful shutdown** with neutral position return

### Debugging and Troubleshooting

#### Common Issues

**VR Controller Not Connecting:**
```bash
# Check ngrok tunnel
ngrok http 8012
# Verify Quest browser can access the URL
# Ensure controllers are paired and tracking
```

**Robot Not Responding:**
```bash
# Check CAN interface
ip link show can0
# Test PiperX connection
python -c "from piper_sdk import C_PiperInterface_V2; robot = C_PiperInterface_V2('can0'); robot.ConnectPort()"
```

**IK Solver Errors:**
```bash
# Run with verbose debugging
python puppet_follow_vr.py --verbose-debug --single-arm
# Check joint limits in calibration file
# Verify robot is in valid starting pose
```

#### Debug Modes

**Basic debugging:**
```bash
python puppet_follow_vr.py --debug --single-arm
```

**Verbose debugging (extensive logging):**
```bash
python puppet_follow_vr.py --verbose-debug --single-arm
```

**VR pose debugging:**
```bash
python relay_vr_controller.py --debug-poses --puppet-ip <IP>
```

### Performance Optimization

- **Control Frequency**: Default 50Hz, increase to 60Hz for more responsive control
- **Dead Zone**: Adjust for controller sensitivity (default: 0.01)
- **Velocity Scaling**: Tune `LINEAR_VELOCITY_SCALE` and `ANGULAR_VELOCITY_SCALE` 
- **Network Latency**: Use wired connection for puppet robot when possible

### Multi-Robot Setup

Control multiple PiperX robots with separate VR controllers:

```bash
# Robot 1 (Left)
python puppet_follow_vr.py --single-arm --can-left can0

# Robot 2 (Right) 
python puppet_follow_vr.py --single-arm --can-left can1

# VR Controller 1
python relay_vr_controller.py --puppet-ip <IP1> --robot-id 0

# VR Controller 2  
python relay_vr_controller.py --puppet-ip <IP2> --robot-id 1
```

This allows independent control of multiple robots for complex bimanual tasks.

