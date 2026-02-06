import json
import numpy as np

class JointMapper:
    def __init__(self, master_calib_path, puppet_calib_path):
        """
        Initializes the JointMapper with calibration files for master and puppet robots.

        Args:
            master_calib_path (str): Path to the master robot's calibration JSON file.
            puppet_calib_path (str): Path to the puppet robot's calibration JSON file.
        """
        self.master_calib = self._load_calibration(master_calib_path)
        self.puppet_calib = self._load_calibration(puppet_calib_path)
        self._validate_calibration()
        
        print(f"Master robot: {self.master_calib.get('robot_model', 'Unknown')}")
        print(f"Puppet robot: {self.puppet_calib.get('robot_model', 'Unknown')}")
        print(f"Master joints: {self.master_calib['joint_names']}")
        print(f"Puppet joints: {self.puppet_calib['joint_names']}")

    def _load_calibration(self, filepath):
        """Loads a calibration JSON file."""
        try:
            with open(filepath, 'r') as f:
                return json.load(f)
        except (FileNotFoundError, json.JSONDecodeError) as e:
            print(f"Error loading calibration file {filepath}: {e}")
            raise

    def _validate_calibration(self):
        """Validates that the calibration files have the necessary keys."""
        required_keys = ['joint_names', 'joint_limits', 'gripper_limits']
        for key in required_keys:
            if key not in self.master_calib or key not in self.puppet_calib:
                raise ValueError(f"Missing required key '{key}' in one of the calibration files.")
        
        # Check if we have 6 joints for both robots
        if len(self.master_calib['joint_names']) != 6 or len(self.puppet_calib['joint_names']) != 6:
            raise ValueError("Both master and puppet robots must have exactly 6 joints.")
        
        # Check that joint_limits has 2 positions for each joint
        if len(self.master_calib['joint_limits']) != 6 or len(self.puppet_calib['joint_limits']) != 6:
            raise ValueError("Both robots must have exactly 6 joint calibration position pairs.")
        
        for i, positions in enumerate(self.master_calib['joint_limits']):
            if len(positions) != 2:
                raise ValueError(f"Master joint {i} must have exactly 2 calibration positions, got {len(positions)}")
        
        for i, positions in enumerate(self.puppet_calib['joint_limits']):
            if len(positions) != 2:
                raise ValueError(f"Puppet joint {i} must have exactly 2 calibration positions, got {len(positions)}")

    def _normalize_from_neutral(self, value, pos1_val, pos2_val, neutral_val):
        """
        Normalize a value relative to its neutral position.
        
        Note: pos1_val and pos2_val are not "min" and "max" but rather two 
        corresponding signed positions that map consistently between robots.
        
        Returns a value where:
        - 0.0 = neutral position
        - -1.0 = pos1 limit (first calibration position)
        - +1.0 = pos2 limit (second calibration position)
        """
        # Handle None values by returning neutral position (safety measure)
        if value is None:
            return 0.0
            
        if value == neutral_val:
            return 0.0
        
        # Calculate the ranges from neutral to each position
        range_to_pos1 = pos1_val - neutral_val
        range_to_pos2 = pos2_val - neutral_val
        
        # Determine which range the value falls into
        value_offset = value - neutral_val
        
        # If value is on the same side as pos1 from neutral
        if (value_offset * range_to_pos1) > 0:
            # Scale towards -1.0 (pos1 direction)
            if range_to_pos1 == 0:
                return 0.0  # Avoid division by zero
            return value_offset / range_to_pos1 * (-1.0)
        else:
            # Scale towards +1.0 (pos2 direction)
            if range_to_pos2 == 0:
                return 0.0  # Avoid division by zero
            return value_offset / range_to_pos2 * (1.0)

    def _scale_from_neutral(self, norm_value, pos1_val, pos2_val, neutral_val):
        """
        Scale a normalized value (relative to neutral) to a given range.
        
        Note: pos1_val and pos2_val are not "min" and "max" but rather two 
        corresponding signed positions that map consistently between robots.
        
        Input norm_value: -1.0 (pos1) to 0.0 (neutral) to +1.0 (pos2)
        """
        if norm_value == 0.0:
            return neutral_val
        elif norm_value < 0.0:
            # Scale negative range: [-1.0, 0.0] -> [pos1_val, neutral_val]
            return neutral_val + norm_value * (neutral_val - pos1_val)
        else:
            # Scale positive range: [0.0, 1.0] -> [neutral_val, pos2_val]
            return neutral_val + norm_value * (pos2_val - neutral_val)

    def map_joints(self, master_joint_values):
        """
        Maps joint values from the master to the puppet using neutral-based mapping.
        
        WidowX joints are in radians: [waist, shoulder, elbow, forearm_roll, wrist_angle, wrist_rotate]
        PiperX joints are in degrees*1000: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]
        
        The mapping ensures that:
        1. Master neutral position maps to puppet neutral position
        2. Ranges are scaled proportionally from their respective neutral positions
        3. Works regardless of min/max ordering or signs
        4. None values are preserved as None (for heartbeat system)
        """
        if len(master_joint_values) != 6:
            raise ValueError(f"Expected 6 joint values, got {len(master_joint_values)}")
            
        puppet_joint_values = []
        master_neutral = self.master_calib.get('neutral_positions', [0.0] * 6)
        puppet_neutral = self.puppet_calib.get('neutral_positions', [0.0] * 6)
        
        for i in range(6):
            # Preserve None values to maintain heartbeat system behavior
            if master_joint_values[i] is None:
                puppet_joint_values.append(None)
                continue
                
            master_pos1, master_pos2 = self.master_calib['joint_limits'][i]
            puppet_pos1, puppet_pos2 = self.puppet_calib['joint_limits'][i]
            
            # Use neutral positions for proper mid-point mapping
            master_neut = master_neutral[i]
            puppet_neut = puppet_neutral[i]

            # Normalize master value relative to its neutral position
            # pos1 and pos2 are corresponding signed positions, not min/max
            normalized_value = self._normalize_from_neutral(
                master_joint_values[i], master_pos1, master_pos2, master_neut
            )
            
            # Scale to puppet range relative to puppet neutral position
            scaled_value = self._scale_from_neutral(
                normalized_value, puppet_pos1, puppet_pos2, puppet_neut
            )
            
            # Convert from degrees*1000 to radians for internal use
            # The puppet script will convert back to degrees*1000 when sending commands
            puppet_joint_rad = scaled_value / 1000.0 * np.pi / 180.0
            
            puppet_joint_values.append(puppet_joint_rad)
            
        return puppet_joint_values

    def map_gripper(self, master_gripper_value):
        """
        Maps the gripper value from the master to the puppet using simple linear scaling.
        
        WidowX gripper is in radians
        PiperX gripper is in degrees*1000
        
        Note: Grippers do NOT have a neutral position, so we use simple linear scaling
        from [master_min, master_max] to [puppet_min, puppet_max]
        """
        # Handle None values by returning mid-position
        if master_gripper_value is None:
            puppet_min, puppet_max = self.puppet_calib['gripper_limits']
            mid_value = (puppet_min + puppet_max) / 2.0
            return mid_value / 1000.0 * np.pi / 180.0
        
        # Validate gripper limits format
        if not isinstance(self.master_calib['gripper_limits'], list) or len(self.master_calib['gripper_limits']) != 2:
            raise ValueError("Master gripper_limits must be a list of exactly 2 values [min, max]")
        if not isinstance(self.puppet_calib['gripper_limits'], list) or len(self.puppet_calib['gripper_limits']) != 2:
            raise ValueError("Puppet gripper_limits must be a list of exactly 2 values [min, max]")
            
        master_min, master_max = self.master_calib['gripper_limits']
        puppet_min, puppet_max = self.puppet_calib['gripper_limits']
        
        # For grippers, use simple linear scaling since there's NO neutral position
        # This maps the full range: master[min->max] to puppet[min->max]
        if master_max == master_min:
            # Handle degenerate case where gripper has no range
            normalized_value = 0.5
        else:
            # Linear normalization: [master_min, master_max] -> [0, 1]
            normalized_value = (master_gripper_value - master_min) / (master_max - master_min)
        
        # Linear scaling: [0, 1] -> [puppet_min, puppet_max]
        scaled_value = normalized_value * (puppet_max - puppet_min) + puppet_min
        
        # Convert from degrees*1000 to radians for internal use
        # The puppet script will convert back to degrees*1000 when sending commands
        puppet_gripper_rad = scaled_value / 1000.0 * np.pi / 180.0
        
        return puppet_gripper_rad

    def get_neutral_positions(self):
        """
        Returns the neutral positions for both master and puppet robots.
        """
        master_neutral = self.master_calib.get('neutral_positions', [0.0] * 6)
        puppet_neutral = self.puppet_calib.get('neutral_positions', [0.0] * 6)
        return master_neutral, puppet_neutral

    def get_joint_limits_info(self):
        """
        Returns joint calibration position information for debugging.
        """
        print("\n=== Joint Calibration Positions ===")
        print("Note: These are not min/max limits, but corresponding signed positions for mapping")
        print("\nMaster (WidowX) - Radians:")
        for i, (name, positions) in enumerate(zip(self.master_calib['joint_names'], self.master_calib['joint_limits'])):
            print(f"  {name}: [{positions[0]:.3f}, {positions[1]:.3f}] rad")
        
        print("\nPuppet (PiperX) - Degrees*1000:")
        for i, (name, positions) in enumerate(zip(self.puppet_calib['joint_names'], self.puppet_calib['joint_limits'])):
            print(f"  {name}: [{positions[0]}, {positions[1]}] deg*1000")
        
        print(f"\nMaster Gripper: [{self.master_calib['gripper_limits'][0]:.3f}, {self.master_calib['gripper_limits'][1]:.3f}] rad")
        print(f"Puppet Gripper: [{self.puppet_calib['gripper_limits'][0]}, {self.puppet_calib['gripper_limits'][1]}] deg*1000")
        print("Note: Grippers use simple linear scaling (NO neutral position)")

    def test_mapping_range(self):
        """
        Test the mapping with min, neutral, and max values.
        Shows how neutral-based mapping works.
        """
        print("\n=== Testing Joint Mapping Range (Neutral-Based) ===")
        
        master_neutral = self.master_calib.get('neutral_positions', [0.0] * 6)
        puppet_neutral = self.puppet_calib.get('neutral_positions', [0.0] * 6)
        
        print("Expected: Master neutral -> Puppet neutral")
        print(f"Master NEUTRAL: {[f'{x:.3f}' for x in master_neutral]}")
        print(f"Puppet NEUTRAL: {[f'{x:.3f}' for x in puppet_neutral]}")
        
        # Test with neutral values (should map to puppet neutral)
        mapped_neutral = self.map_joints(master_neutral)
        print(f"Mapped NEUTRAL: {[f'{x:.3f}' for x in mapped_neutral]}")
        
        # Convert puppet neutral from degrees*1000 to radians for comparison
        puppet_neutral_rad = [x / 1000.0 * np.pi / 180.0 for x in puppet_neutral]
        print(f"Expected (rad): {[f'{x:.3f}' for x in puppet_neutral_rad]}")
        
        print("\nTesting calibration positions:")
        # Test with pos1 values (first calibration position)
        master_pos1 = [positions[0] for positions in self.master_calib['joint_limits']]
        mapped_pos1 = self.map_joints(master_pos1)
        print(f"Master POS1: {[f'{x:.3f}' for x in master_pos1]}")
        print(f"Mapped POS1: {[f'{x:.3f}' for x in mapped_pos1]}")
        
        # Test with pos2 values (second calibration position)
        master_pos2 = [positions[1] for positions in self.master_calib['joint_limits']]
        mapped_pos2 = self.map_joints(master_pos2)
        print(f"Master POS2: {[f'{x:.3f}' for x in master_pos2]}")
        print(f"Mapped POS2: {[f'{x:.3f}' for x in mapped_pos2]}")
        
        # Show the proportional scaling
        print("\nProportional scaling verification:")
        for i in range(6):
            master_range = master_pos2[i] - master_pos1[i]
            mapped_range = mapped_pos2[i] - mapped_pos1[i]
            print(f"Joint {i+1}: Master range={master_range:.3f}, Mapped range={mapped_range:.3f}")
            
        # Test with some intermediate values
        print("\nTesting intermediate values:")
        test_values = [
            [0.5, -0.5, 1.0, -1.0, 0.25, -0.25],  # Mixed values
            [x * 0.5 for x in master_neutral]       # Half of neutral
        ]
        
        for j, test_val in enumerate(test_values):
            mapped = self.map_joints(test_val)
            print(f"Test {j+1}: {[f'{x:.3f}' for x in test_val]} -> {[f'{x:.3f}' for x in mapped]}")
            
    def test_gripper_mapping(self):
        """
        Test gripper mapping specifically.
        Shows how linear scaling works for grippers (no neutral position).
        """
        print("\n=== Testing Gripper Mapping (Linear Scaling) ===")
        
        master_min, master_max = self.master_calib['gripper_limits']
        puppet_min, puppet_max = self.puppet_calib['gripper_limits']
        
        print(f"Master gripper range: [{master_min:.3f}, {master_max:.3f}] rad")
        print(f"Puppet gripper range: [{puppet_min}, {puppet_max}] deg*1000")
        
        # Test with boundary values
        test_values = [
            master_min,  # Minimum
            master_max,  # Maximum
            (master_min + master_max) / 2,  # Midpoint
        ]
        
        # Add some intermediate values
        test_values.extend([
            master_min + 0.25 * (master_max - master_min),  # 25%
            master_min + 0.75 * (master_max - master_min),  # 75%
        ])
        
        print("\nLinear scaling test:")
        for val in test_values:
            mapped = self.map_gripper(val)
            # Also show what would be sent to PiperX (degrees*1000)
            mapped_deg1000 = mapped * 180.0 / np.pi * 1000
            percentage = (val - master_min) / (master_max - master_min) * 100
            print(f"  Master: {val:.3f} rad ({percentage:.1f}%) -> Puppet: {mapped:.3f} rad ({mapped_deg1000:.0f} deg*1000)")
        
        print("\nExpected behavior:")
        print(f"  Master MIN ({master_min:.3f}) -> Puppet MIN ({puppet_min} deg*1000)")
        print(f"  Master MAX ({master_max:.3f}) -> Puppet MAX ({puppet_max} deg*1000)")
        print("  Linear interpolation for all values in between") 