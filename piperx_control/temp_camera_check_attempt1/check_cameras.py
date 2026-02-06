#!/usr/bin/env python3
"""
Camera availability checker script
"""
import cv2
import subprocess
import sys

def check_opencv_cameras():
    """Check cameras using OpenCV"""
    print("=== OpenCV Camera Check ===")
    available_cameras = []
    
    # Check first 10 camera indices
    for i in range(10):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                height, width = frame.shape[:2]
                print(f"Camera {i}: Available - Resolution: {width}x{height}")
                available_cameras.append(i)
            else:
                print(f"Camera {i}: Device exists but cannot read frames")
            cap.release()
        else:
            # Only print for first few indices to avoid spam
            if i < 3:
                print(f"Camera {i}: Not available")
    
    return available_cameras

def check_v4l2_devices():
    """Check Video4Linux devices"""
    print("\n=== Video4Linux Devices ===")
    try:
        result = subprocess.run(['v4l2-ctl', '--list-devices'], 
                              capture_output=True, text=True)
        if result.returncode == 0:
            print(result.stdout)
        else:
            print("v4l2-ctl not available or failed")
    except FileNotFoundError:
        print("v4l2-ctl not installed")

def check_dev_video():
    """Check /dev/video* devices"""
    print("\n=== /dev/video* Devices ===")
    try:
        result = subprocess.run(['ls', '-la', '/dev/video*'], 
                              capture_output=True, text=True)
        if result.returncode == 0:
            print(result.stdout)
        else:
            print("No /dev/video* devices found")
    except Exception as e:
        print(f"Error checking /dev/video*: {e}")

def test_camera_formats(camera_index):
    """Test different formats for a specific camera"""
    print(f"\n=== Testing Camera {camera_index} Formats ===")
    cap = cv2.VideoCapture(camera_index)
    
    if not cap.isOpened():
        print(f"Cannot open camera {camera_index}")
        return
    
    # Test different formats
    formats_to_test = [
        (640, 480),
        (1280, 720),
        (1920, 1080),
    ]
    
    for width, height in formats_to_test:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        
        actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = cap.get(cv2.CAP_PROP_FPS)
        
        print(f"  Requested: {width}x{height} -> Actual: {actual_width}x{actual_height} @ {fps:.1f}fps")
    
    cap.release()

if __name__ == "__main__":
    print("Camera Availability Checker")
    print("=" * 40)
    
    # Check system devices
    check_dev_video()
    check_v4l2_devices()
    
    # Check OpenCV access
    available_cameras = check_opencv_cameras()
    
    # Test formats for available cameras
    for cam_idx in available_cameras:
        test_camera_formats(cam_idx)
    
    print(f"\nSummary: Found {len(available_cameras)} working cameras: {available_cameras}")