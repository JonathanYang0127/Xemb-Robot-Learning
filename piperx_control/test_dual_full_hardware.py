#!/usr/bin/env python3
"""
Test script for dual-arm + full hardware configuration
Validates all hardware connections before starting teleoperation
"""

import sys
import time
import json
import os

# Add the parent directory to the path
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

def test_piperx_connection(can_interface, arm_name):
    """Test PiperX robot connection"""
    print(f"\n=== Testing PiperX {arm_name} arm on {can_interface} ===")
    try:
        from piper_sdk import C_PiperInterface_V2
        
        print(f"Attempting to connect to {can_interface}...")
        piper = C_PiperInterface_V2(can_interface)
        piper.ConnectPort()
        
        # Wait for connection with timeout
        timeout = 5.0
        start_time = time.time()
        while not piper.get_connect_status():
            if time.time() - start_time > timeout:
                raise RuntimeError(f"Connection timeout for {can_interface}")
            time.sleep(0.01)
        
        print(f"✓ Connected to {can_interface}")
        
        # Try to enable the robot
        print(f"Enabling robot on {can_interface}...")
        start_time = time.time()
        while not piper.EnablePiper():
            if time.time() - start_time > timeout:
                raise RuntimeError(f"Enable timeout for {can_interface}")
            time.sleep(0.01)
        
        print(f"✓ Robot on {can_interface} enabled successfully")
        
        # Test basic joint command
        print(f"Testing joint command on {can_interface}...")
        piper.MotionCtrl_2(0x01, 0x01, 100, 0x00)
        piper.JointCtrl(0, 0, 0, 0, 0, 0)  # Send neutral position
        
        print(f"✓ Joint command sent successfully to {can_interface}")
        
        # Test gripper command
        print(f"Testing gripper command on {can_interface}...")
        piper.GripperCtrl(gripper_angle=0, gripper_effort=1000, gripper_code=0x01)
        
        print(f"✓ Gripper command sent successfully to {can_interface}")
        print(f"✓ PiperX {arm_name} arm test PASSED")
        
        return True, piper
        
    except ImportError:
        print("✗ ERROR: piper_sdk not found. Please install with: pip install piper-sdk")
        return False, None
    except Exception as e:
        print(f"✗ ERROR: PiperX {arm_name} arm test failed: {e}")
        return False, None

def test_stepper_controller(port="/dev/tty_stepper"):
    """Test stepper motor controller"""
    print(f"\n=== Testing Stepper Controller on {port} ===")
    try:
        from xemb_scripts.stepper_control.stepper_controller import StepperController
        
        print(f"Attempting to connect to stepper on {port}...")
        stepper = StepperController(port=port)
        
        print(f"✓ Connected to stepper on {port}")
        
        # Test enable command
        print("Testing stepper enable...")
        stepper.enable()
        print("✓ Stepper enable command sent")
        
        # Test movement command
        print("Testing stepper movement (small step)...")
        stepper.move_steps(10)
        time.sleep(0.5)
        stepper.move_steps(-10)  # Return to original position
        print("✓ Stepper movement test completed")
        
        # Test stop command
        print("Testing stepper stop...")
        stepper.stop()
        print("✓ Stepper stop command sent")
        
        print("✓ Stepper controller test PASSED")
        return True, stepper
        
    except ImportError:
        print("✗ ERROR: stepper_control module not available")
        return False, None
    except Exception as e:
        print(f"✗ ERROR: Stepper controller test failed: {e}")
        return False, None

def test_base_controller(port="/dev/tty_base"):
    """Test base robot controller"""
    print(f"\n=== Testing Base Controller on {port} ===")
    try:
        from xemb_scripts.base_control.base_controller import MDCRobot
        
        print(f"Attempting to connect to base robot on {port}...")
        base = MDCRobot(port=port, default_speed=1)
        base.connect()
        
        print(f"✓ Connected to base robot on {port}")
        
        # Test basic movement commands
        print("Testing base movement commands...")
        
        print("  Testing forward/backward...")
        base.drive(speed=100, turn=0)
        time.sleep(0.2)
        base.stop()
        
        print("  Testing turn left/right...")
        base.drive(speed=0, turn=100)
        time.sleep(0.2)
        base.stop()
        
        print("  Testing stop command...")
        base.stop()
        
        print("✓ Base controller movement test completed")
        print("✓ Base controller test PASSED")
        
        return True, base
        
    except ImportError:
        print("✗ ERROR: base_control module not available")
        return False, None
    except Exception as e:
        print(f"✗ ERROR: Base controller test failed: {e}")
        return False, None

def test_widowx_connection(robot_name, arm_name):
    """Test WidowX robot connection"""
    print(f"\n=== Testing WidowX {arm_name} arm ({robot_name}) ===")
    try:
        from interbotix_xs_modules.arm import InterbotixManipulatorXS
        import rospy
        
        # Initialize ROS if not already done
        if not rospy.get_node_uri():
            rospy.init_node('test_widowx', anonymous=True)
        
        print(f"Attempting to connect to WidowX {arm_name} arm...")
        widowx = InterbotixManipulatorXS(
            robot_model="wx250s",
            group_name="arm", 
            gripper_name="gripper",
            robot_name=robot_name,
            init_node=False
        )
        
        print(f"✓ Connected to WidowX {arm_name} arm")
        
        # Test basic communication
        print("Testing joint position reading...")
        positions = widowx.dxl.joint_states.position
        if len(positions) < 7:
            raise RuntimeError(f"Insufficient joints detected: {len(positions)}")
        
        print(f"✓ Read {len(positions)} joint positions")
        
        # Test basic movement
        print("Testing basic arm movement...")
        widowx.dxl.robot_set_operating_modes("group", "arm", "position")
        widowx.dxl.robot_set_operating_modes("single", "gripper", "position")
        
        # Small movement test
        current_pos = list(widowx.dxl.joint_states.position[:6])
        test_pos = current_pos.copy()
        test_pos[0] += 0.1  # Small movement in first joint
        
        from xemb_scripts.robot_utils import move_arms
        move_arms([widowx], [test_pos], move_time=0.5)
        time.sleep(0.5)
        move_arms([widowx], [current_pos], move_time=0.5)  # Return to original
        
        print("✓ Basic arm movement test completed")
        print(f"✓ WidowX {arm_name} arm test PASSED")
        
        return True, widowx
        
    except ImportError:
        print("✗ ERROR: interbotix_xs_modules not available")
        return False, None
    except Exception as e:
        print(f"✗ ERROR: WidowX {arm_name} arm test failed: {e}")
        return False, None

def test_calibration_files():
    """Test that required calibration files exist and are valid"""
    print("\n=== Testing Calibration Files ===")
    
    required_files = [
        'piperx_calibration.json',
        'widowx_calibration.json'
    ]
    
    all_good = True
    
    for filename in required_files:
        print(f"Checking {filename}...")
        try:
            with open(filename, 'r') as f:
                data = json.load(f)
            
            # Check for required fields
            if 'neutral_positions' not in data:
                print(f"✗ ERROR: {filename} missing 'neutral_positions' field")
                all_good = False
                continue
            
            if len(data['neutral_positions']) < 6:
                print(f"✗ ERROR: {filename} has insufficient neutral positions: {len(data['neutral_positions'])}")
                all_good = False
                continue
            
            print(f"✓ {filename} is valid")
            
        except FileNotFoundError:
            print(f"✗ ERROR: {filename} not found")
            all_good = False
        except json.JSONDecodeError:
            print(f"✗ ERROR: {filename} is not valid JSON")
            all_good = False
        except Exception as e:
            print(f"✗ ERROR: Failed to validate {filename}: {e}")
            all_good = False
    
    if all_good:
        print("✓ All calibration files test PASSED")
    else:
        print("✗ Calibration files test FAILED")
    
    return all_good

def test_network_connectivity(puppet_ip="100.105.116.25"):
    """Test network connectivity to puppet robot"""
    print(f"\n=== Testing Network Connectivity to {puppet_ip} ===")
    
    import subprocess
    try:
        # Test ping
        result = subprocess.run(['ping', '-c', '3', puppet_ip], 
                              capture_output=True, text=True, timeout=10)
        
        if result.returncode == 0:
            print(f"✓ Ping to {puppet_ip} successful")
        else:
            print(f"✗ Ping to {puppet_ip} failed")
            return False
        
        # Test UDP socket creation
        import socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(1.0)
        
        # Try to send a test packet
        test_data = b"test"
        sock.sendto(test_data, (puppet_ip, 5005))
        sock.close()
        
        print(f"✓ UDP socket test to {puppet_ip}:5005 successful")
        print("✓ Network connectivity test PASSED")
        return True
        
    except subprocess.TimeoutExpired:
        print(f"✗ Ping to {puppet_ip} timed out")
        return False
    except Exception as e:
        print(f"✗ Network connectivity test failed: {e}")
        return False

def main():
    """Run comprehensive hardware tests"""
    print("=" * 60)
    print("DUAL-ARM + FULL HARDWARE VALIDATION TEST")
    print("=" * 60)
    
    test_results = {}
    hardware_objects = {}
    
    # Test calibration files first
    test_results['calibration'] = test_calibration_files()
    
    # Test PiperX arms
    test_results['piperx_left'], hardware_objects['piperx_left'] = test_piperx_connection("can0", "left")
    test_results['piperx_right'], hardware_objects['piperx_right'] = test_piperx_connection("can1", "right")
    
    # Test WidowX arms (for master controller)
    test_results['widowx_left'], hardware_objects['widowx_left'] = test_widowx_connection("master_left", "left")
    test_results['widowx_right'], hardware_objects['widowx_right'] = test_widowx_connection("master_right", "right")
    
    # Test stepper controller
    test_results['stepper'], hardware_objects['stepper'] = test_stepper_controller()
    
    # Test base controller
    test_results['base'], hardware_objects['base'] = test_base_controller()
    
    # Test network connectivity
    test_results['network'] = test_network_connectivity()
    
    # Print summary
    print("\n" + "=" * 60)
    print("TEST SUMMARY")
    print("=" * 60)
    
    all_passed = True
    for test_name, result in test_results.items():
        status = "PASS" if result else "FAIL"
        print(f"{test_name.upper():20} : {status}")
        if not result:
            all_passed = False
    
    print("=" * 60)
    if all_passed:
        print("🎉 ALL TESTS PASSED! Hardware is ready for dual-arm + full operation")
        print("\nYou can now run:")
        print("  Master:  python piperx_control/relay_master_widowx_dual_full.py")
        print("  Puppet:  python piperx_control/puppet_follow_piperx_dual_full.py")
    else:
        print("❌ SOME TESTS FAILED! Please address the issues above before proceeding")
        print("\nFailed components need to be fixed before running the full system")
    
    # Cleanup hardware connections
    print("\nCleaning up test connections...")
    try:
        if hardware_objects.get('stepper'):
            hardware_objects['stepper'].stop()
            hardware_objects['stepper'].close()
        if hardware_objects.get('base'):
            hardware_objects['base'].stop()
        if hardware_objects.get('widowx_left'):
            hardware_objects['widowx_left'].shutdown()
        if hardware_objects.get('widowx_right'):
            hardware_objects['widowx_right'].shutdown()
    except Exception as e:
        print(f"Warning: Error during cleanup: {e}")
    
    print("Test completed.")
    return all_passed

if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1)