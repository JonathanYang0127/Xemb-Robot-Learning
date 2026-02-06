#!/usr/bin/env python3
"""
Enhanced Base Controller with Smooth Joystick Support
Provides smooth, continuous movement control for robot base using VR joystick input
"""

import time
import math
import numpy as np
from threading import Thread, Lock

class EnhancedBaseController:
    """Enhanced base controller with smooth joystick-based movement"""
    
    def __init__(self, base_controller=None, max_linear_speed=1.0, max_angular_speed=1.0):
        """
        Initialize enhanced base controller
        
        Args:
            base_controller: Existing base controller instance (MDCRobot or MockMDCRobot)
            max_linear_speed: Maximum linear speed (m/s or units/s)
            max_angular_speed: Maximum angular speed (rad/s or units/s)
        """
        self.base_controller = base_controller
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        
        # Movement parameters
        self.dead_zone = 0.1  # Joystick dead zone (0.0 to 1.0)
        self.speed_curve = 2.0  # Speed curve exponent (higher = more precise control at low speeds)
        
        # Current movement state
        self.current_linear = 0.0  # Current linear velocity
        self.current_angular = 0.0  # Current angular velocity
        self.target_linear = 0.0   # Target linear velocity
        self.target_angular = 0.0  # Target angular velocity
        
        # Smoothing parameters
        self.acceleration = 2.0    # Acceleration rate (units/s²)
        self.deceleration = 3.0    # Deceleration rate (units/s²)
        
        # Control thread
        self.control_thread = None
        self.control_running = False
        self.control_lock = Lock()
        self.last_update = time.time()
        
        # Start control thread
        self.start_control_thread()
    
    def start_control_thread(self):
        """Start the control thread for smooth movement"""
        if self.control_thread is None:
            self.control_running = True
            self.control_thread = Thread(target=self._control_loop, daemon=True)
            self.control_thread.start()
            print("🎮 Enhanced base controller started")
    
    def stop_control_thread(self):
        """Stop the control thread"""
        self.control_running = False
        if self.control_thread:
            self.control_thread.join()
            self.control_thread = None
        print("🛑 Enhanced base controller stopped")
    
    def apply_dead_zone(self, value):
        """Apply dead zone to joystick input"""
        if abs(value) < self.dead_zone:
            return 0.0
        # Scale value from dead_zone to 1.0 range to 0.0 to 1.0 range
        if value > 0:
            return (value - self.dead_zone) / (1.0 - self.dead_zone)
        else:
            return (value + self.dead_zone) / (1.0 - self.dead_zone)
    
    def apply_speed_curve(self, value):
        """Apply speed curve for more precise control"""
        sign = 1.0 if value >= 0 else -1.0
        return sign * (abs(value) ** self.speed_curve)
    
    def update_joystick(self, joystick_x, joystick_y):
        """
        Update movement based on joystick input
        
        Args:
            joystick_x: Joystick X axis (-1.0 to 1.0, positive = right/turn right)
            joystick_y: Joystick Y axis (-1.0 to 1.0, positive = forward)
        """
        with self.control_lock:
            # Apply dead zone
            x_clean = self.apply_dead_zone(joystick_x)
            y_clean = self.apply_dead_zone(joystick_y)
            
            # Apply speed curve for more precise control
            x_curved = self.apply_speed_curve(x_clean)
            y_curved = self.apply_speed_curve(y_clean)
            
            # Calculate target velocities
            # Y axis controls forward/backward movement
            self.target_linear = y_curved * self.max_linear_speed
            
            # X axis controls turning (positive = turn right)
            self.target_angular = -x_curved * self.max_angular_speed  # Negative for intuitive control
            
            # Debug output for significant changes
            if abs(x_clean) > 0.05 or abs(y_clean) > 0.05:
                print(f"🕹️ Joystick: X={joystick_x:.2f}→{x_curved:.2f}, Y={joystick_y:.2f}→{y_curved:.2f} | Target: Linear={self.target_linear:.2f}, Angular={self.target_angular:.2f}")
    
    def _smooth_approach(self, current, target, dt):
        """Smoothly approach target value with acceleration/deceleration"""
        diff = target - current
        
        if abs(diff) < 0.01:  # Close enough
            return target
        
        # Choose acceleration or deceleration based on whether we're speeding up or slowing down
        if abs(target) > abs(current):  # Speeding up
            max_change = self.acceleration * dt
        else:  # Slowing down
            max_change = self.deceleration * dt
        
        # Limit the change
        if abs(diff) > max_change:
            return current + max_change * (1.0 if diff > 0 else -1.0)
        else:
            return target
    
    def _control_loop(self):
        """Main control loop for smooth movement"""
        control_rate = 50.0  # 50Hz control rate
        dt = 1.0 / control_rate
        
        while self.control_running:
            start_time = time.time()
            
            with self.control_lock:
                # Smooth approach to target velocities
                self.current_linear = self._smooth_approach(self.current_linear, self.target_linear, dt)
                self.current_angular = self._smooth_approach(self.current_angular, self.target_angular, dt)
                
                # Apply movement if we have a base controller
                if self.base_controller and (abs(self.current_linear) > 0.01 or abs(self.current_angular) > 0.01):
                    self._apply_movement(self.current_linear, self.current_angular, dt)
            
            # Maintain control rate
            elapsed = time.time() - start_time
            sleep_time = max(0, dt - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    def _apply_movement(self, linear_vel, angular_vel, dt):
        """Apply movement to the base controller"""
        try:
            # Calculate movement distances for this control cycle
            linear_distance = linear_vel * dt
            angular_distance = angular_vel * dt
            
            # Apply linear movement
            if abs(linear_distance) > 0.001:  # Minimum movement threshold
                if hasattr(self.base_controller, 'move_forward') and hasattr(self.base_controller, 'move_backward'):
                    if linear_distance > 0:
                        self.base_controller.move_forward(abs(linear_distance))
                    else:
                        self.base_controller.move_backward(abs(linear_distance))
                elif hasattr(self.base_controller, 'set_linear_velocity'):
                    # If base controller supports velocity control
                    self.base_controller.set_linear_velocity(linear_vel)
                else:
                    print(f"🤖 Base linear: {linear_distance:.3f}")
            
            # Apply angular movement
            if abs(angular_distance) > 0.001:  # Minimum movement threshold
                if hasattr(self.base_controller, 'turn_left') and hasattr(self.base_controller, 'turn_right'):
                    if angular_distance > 0:
                        self.base_controller.turn_left(abs(angular_distance))
                    else:
                        self.base_controller.turn_right(abs(angular_distance))
                elif hasattr(self.base_controller, 'set_angular_velocity'):
                    # If base controller supports velocity control
                    self.base_controller.set_angular_velocity(angular_vel)
                else:
                    print(f"🤖 Base angular: {angular_distance:.3f}")
                    
        except Exception as e:
            print(f"❌ Error applying base movement: {e}")
    
    def stop_movement(self):
        """Stop all movement immediately"""
        with self.control_lock:
            self.target_linear = 0.0
            self.target_angular = 0.0
            self.current_linear = 0.0
            self.current_angular = 0.0
        print("🛑 Base movement stopped")
    
    def get_status(self):
        """Get current movement status"""
        with self.control_lock:
            return {
                'current_linear': self.current_linear,
                'current_angular': self.current_angular,
                'target_linear': self.target_linear,
                'target_angular': self.target_angular
            }

# Mock enhanced base controller for testing without real hardware
class MockEnhancedBaseController(EnhancedBaseController):
    """Mock version of enhanced base controller for testing"""
    
    def __init__(self, max_linear_speed=1.0, max_angular_speed=1.0):
        # Create a simple mock base controller
        class MockBase:
            def move_forward(self, distance):
                pass
            def move_backward(self, distance):
                pass
            def turn_left(self, angle):
                pass
            def turn_right(self, angle):
                pass
        
        super().__init__(MockBase(), max_linear_speed, max_angular_speed)
        print("🤖 Mock enhanced base controller initialized")