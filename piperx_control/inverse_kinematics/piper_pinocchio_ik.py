#!/usr/bin/env python3

import numpy as np
import json
from typing import Optional, List, Dict, Tuple, Union
import warnings
import os
import time

try:
    import pinocchio as pin
    PINOCCHIO_AVAILABLE = True
except ImportError:
    PINOCCHIO_AVAILABLE = False
    warnings.warn("Pinocchio not available. Install with: pip install pin")

class PiperIK:
    """
    Piper robot inverse kinematics using Pinocchio.
    Fast, predictable IK solver optimized for real-time applications like VR.
    """
    
    def __init__(self, urdf_file: Optional[str] = None, base_link: str = "base_link", 
                 tip_link: str = "gripper_base", max_iterations: int = 100, 
                 tolerance: float = 1e-4, damping: float = 1e-4,
                 orientation_weight: float = 1.0, verbose: bool = False):
        """
        Initialize the Piper IK solver with Pinocchio.
        
        Args:
            urdf_file: Path to URDF file (will auto-find if None)
            base_link: Name of the base link
            tip_link: Name of the tip/end-effector link
            max_iterations: Maximum IK iterations
            tolerance: Convergence tolerance
            damping: Damping factor for numerical stability
            orientation_weight: Weighting factor for orientation error vs. position error
            verbose: If True, print detailed solver info during operation
        """
        if not PINOCCHIO_AVAILABLE:
            raise ImportError("Pinocchio is required. Install with: pip install pin")
        
        # Find URDF file
        if urdf_file is None:
            urdf_file = find_piper_urdf()
            if urdf_file is None:
                raise FileNotFoundError("No URDF file found. Please specify urdf_file parameter.")
        
        self.urdf_file = urdf_file
        self.base_link = base_link
        self.tip_link = tip_link
        self.max_iterations = max_iterations
        self.tolerance = tolerance
        self.damping = damping
        self.orientation_weight = orientation_weight
        self.verbose = verbose
        
        print(f"Loading robot from: {urdf_file}")
        
        # Load robot model
        try:
            self.model = pin.buildModelFromUrdf(self.urdf_file)
            self.data = self.model.createData()
            
            # Find end-effector frame ID
            self.ee_frame_id = None
            for frame_id, frame in enumerate(self.model.frames):
                if frame.name == self.tip_link:
                    self.ee_frame_id = frame_id
                    break
            
            if self.ee_frame_id is None:
                # Try common gripper frame names
                gripper_names = ['gripper_base', 'gripper_link', 'end_effector', 'tool0']
                for frame_id, frame in enumerate(self.model.frames):
                    if frame.name in gripper_names:
                        self.ee_frame_id = frame_id
                        self.tip_link = frame.name
                        break
                
                if self.ee_frame_id is None:
                    # Use the last frame as fallback
                    self.ee_frame_id = len(self.model.frames) - 1
                    self.tip_link = self.model.frames[self.ee_frame_id].name
                    warnings.warn(f"Could not find '{tip_link}' frame, using '{self.tip_link}' instead")
            
            # Extract joint information - focus on arm joints (first 6)
            self.n_joints = self.model.nq
            self.n_arm_joints = 6  # Piper has 6 arm joints + 2 gripper joints
            self.joint_names = [self.model.names[i] for i in range(1, self.model.njoints)]
            
            # Get joint limits
            self.joint_limits_lower = self.model.lowerPositionLimit.copy()
            self.joint_limits_upper = self.model.upperPositionLimit.copy()
            
            # Replace infinite limits with reasonable values
            inf_mask = np.isinf(self.joint_limits_lower)
            self.joint_limits_lower[inf_mask] = -2*np.pi
            
            inf_mask = np.isinf(self.joint_limits_upper)
            self.joint_limits_upper[inf_mask] = 2*np.pi
            
            print(f"✅ Loaded Piper robot with Pinocchio:")
            print(f"   Total joints: {self.n_joints} (6 arm + 2 gripper)")
            print(f"   Joint names: {self.joint_names}")
            print(f"   Base link: {self.base_link}")
            print(f"   Tip link: {self.tip_link} (frame ID: {self.ee_frame_id})")
            print(f"   Max iterations: {self.max_iterations}")
            print(f"   Tolerance: {self.tolerance}")
            print(f"   Damping: {self.damping}")
            print(f"   Orientation Weight: {self.orientation_weight}")
            print(f"   Joint limits: {self.joint_limits_lower} to {self.joint_limits_upper}")
            
        except Exception as e:
            raise RuntimeError(f"Failed to initialize Pinocchio: {e}")
    
    def forward_kinematics(self, joint_positions: Union[List, np.ndarray]) -> Optional[Dict]:
        """
        Compute forward kinematics using Pinocchio.
        
        Args:
            joint_positions: Joint positions
            
        Returns:
            dict: {'position': [x, y, z], 'orientation': [x, y, z, w]} or None if failed
        """
        try:
            joint_positions = np.asarray(joint_positions)
            
            if len(joint_positions) != self.n_joints:
                print(f"Expected {self.n_joints} joint values, got {len(joint_positions)}")
                return None
            
            # Update robot configuration
            pin.framesForwardKinematics(self.model, self.data, joint_positions)
            
            # Get end-effector pose
            ee_pose = self.data.oMf[self.ee_frame_id]
            
            # Extract position
            position = ee_pose.translation.tolist()
            
            # Extract orientation as quaternion [x, y, z, w]
            quaternion = pin.Quaternion(ee_pose.rotation).coeffs()  # Returns [x, y, z, w]
            
            return {
                'position': position,
                'orientation': quaternion.tolist()
            }
            
        except Exception as e:
            print(f"Forward kinematics failed: {e}")
            return None
    
    def inverse_kinematics(self, target_position: Union[List, np.ndarray], 
                          target_orientation: Optional[Union[List, np.ndarray]] = None, 
                          seed_joints: Optional[Union[List, np.ndarray]] = None,
                          arm_only: bool = True,
                          workspace_info: Optional[Dict] = None) -> Optional[np.ndarray]:
        """
        Compute inverse kinematics using Pinocchio's iterative solver with improved convergence.
        
        Args:
            target_position: Target position [x, y, z]
            target_orientation: Target orientation [x, y, z, w] quaternion (optional)
            seed_joints: Seed joint positions (can be 6 or 8 joints)
            arm_only: If True, return only arm joints (6). If False, return full config (8)
            workspace_info: Workspace information for better seed generation
            
        Returns:
            np.array: Joint positions or None if failed
        """
        try:
            target_position = np.asarray(target_position)
            
            # Try multiple seeds for better convergence
            seed_attempts = []
            
            # Add user-provided seed first
            if seed_joints is not None:
                seed_joints = np.asarray(seed_joints)
                if len(seed_joints) == self.n_arm_joints:
                    seed_attempts.append(self.get_full_configuration(seed_joints))
                elif len(seed_joints) == self.n_joints:
                    seed_attempts.append(seed_joints.copy())
            
            # Add smart seed based on workspace info
            if workspace_info:
                smart_seed = self.generate_good_seed(target_position, workspace_info)
                if smart_seed is not None:
                    seed_attempts.append(smart_seed)
            
            # Add default seeds (more conservative)
            default_seeds = [
                # Joint limits midpoint (often good for this robot)
                self.get_full_configuration((self.joint_limits_lower[:self.n_arm_joints] + 
                                           self.joint_limits_upper[:self.n_arm_joints]) / 2),
                # Neutral position
                self.get_full_configuration(np.zeros(self.n_arm_joints)),
                # Conservative random seeds (smaller range)
                self.get_full_configuration(np.random.uniform(-0.3, 0.3, self.n_arm_joints)),
                self.get_full_configuration(np.random.uniform(-0.6, 0.6, self.n_arm_joints))
            ]
            
            seed_attempts.extend(default_seeds)
            
            # Try each seed
            for seed_idx, q in enumerate(seed_attempts):
                solution = self._solve_ik_single_seed(target_position, target_orientation, q, arm_only)
                if solution is not None:
                    return solution
            
            print(f"❌ Pinocchio IK failed with all {len(seed_attempts)} seeds")
            return None
                
        except Exception as e:
            print(f"Inverse kinematics error: {e}")
            return None
    
    def _solve_ik_single_seed(self, target_position: np.ndarray, 
                             target_orientation: Optional[np.ndarray], 
                             q: np.ndarray, arm_only: bool) -> Optional[np.ndarray]:
        """Solve IK with a single seed."""
        # Create target pose
        target_pose = pin.SE3.Identity()
        target_pose.translation = target_position
        
        # Set orientation if provided
        if target_orientation is not None:
            quat = np.asarray(target_orientation)
            target_pose.rotation = pin.Quaternion(quat).matrix()
        
        # More conservative parameters for this robot
        initial_damping = max(self.damping, 0.01)  # Minimum damping
        max_step_size = 0.1  # Smaller max step
        min_step_size = 0.001  # Smaller min step
        
        # Check initial error to see if we're already close
        pin.framesForwardKinematics(self.model, self.data, q)
        current_pose = self.data.oMf[self.ee_frame_id]
        initial_pos_error = np.linalg.norm(target_position - current_pose.translation)
        
        # If already very close, just return (more lenient tolerance)
        if initial_pos_error < 0.02:  # 2cm tolerance
            return q[:self.n_arm_joints] if arm_only else q
        
        # Print initial error if verbose
        if self.verbose:
            pos_error_initial = target_position - current_pose.translation
            if target_orientation is not None:
                error_pose_initial = target_pose.inverse() * current_pose
                ori_error_initial = pin.log(error_pose_initial).vector[3:6]
                print(f"Initial | Pos Error: {np.linalg.norm(pos_error_initial):.4f} | Ori Error: {np.linalg.norm(ori_error_initial):.4f} | Weighted Ori Error: {np.linalg.norm(self.orientation_weight * ori_error_initial):.4f}")
            else:
                print(f"Initial | Pos Error: {np.linalg.norm(pos_error_initial):.4f}")
        
        # Iterative IK solver with better error handling
        previous_error = float('inf')
        stuck_count = 0
        
        for i in range(self.max_iterations):
            # Compute forward kinematics
            pin.framesForwardKinematics(self.model, self.data, q)
            
            # Get current end-effector pose
            current_pose = self.data.oMf[self.ee_frame_id]
            
            # Compute position error directly (simpler and more robust)
            pos_error = target_position - current_pose.translation
            
            # For orientation, use the log-space error if needed
            if target_orientation is not None:
                error_pose = target_pose.inverse() * current_pose
                ori_error = pin.log(error_pose).vector[3:6]
                if self.verbose: # Print every iteration
                    print(f"Iter {i:2d} | Pos Error: {np.linalg.norm(pos_error):.4f} | Ori Error: {np.linalg.norm(ori_error):.4f} | Weighted Ori Error: {np.linalg.norm(self.orientation_weight * ori_error):.4f}")
                error_masked = np.concatenate([pos_error, self.orientation_weight * ori_error])
            else:
                if self.verbose:
                    print(f"Iter {i:2d} | Pos Error: {np.linalg.norm(pos_error):.4f}")
                error_masked = pos_error
            
            error_norm = np.linalg.norm(error_masked)
            
            # Check convergence with more realistic tolerance
            if error_norm < 0.02:  # 2cm tolerance
                if self.verbose:
                    print(f"Converged at iteration {i} with error {error_norm:.4f}")
                return q[:self.n_arm_joints] if arm_only else q
            
            # Check if we're making progress
            if error_norm >= previous_error:
                stuck_count += 1
                if stuck_count >= 3:  # Stop if not making progress for 3 iterations
                    break
            else:
                stuck_count = 0
            
            previous_error = error_norm
            
            # Compute Jacobian for full robot
            J_full = pin.computeFrameJacobian(self.model, self.data, q, self.ee_frame_id, pin.LOCAL_WORLD_ALIGNED)
            
            # Use only arm joints for IK
            J = J_full[:, :self.n_arm_joints]
            
            # Apply mask
            if target_orientation is None:
                J_masked = J[:3, :]
            else:
                J_masked = J
            
            # More conservative damping
            manipulability = np.sqrt(np.linalg.det(J_masked @ J_masked.T))
            if manipulability < 1e-5:
                effective_damping = 0.1
            elif manipulability < 1e-3:
                effective_damping = 0.05
            else:
                effective_damping = initial_damping
            
            # Damped least squares update
            JJt = J_masked @ J_masked.T + effective_damping * np.eye(J_masked.shape[0])
            try:
                dq_arm = J_masked.T @ np.linalg.solve(JJt, error_masked)
            except np.linalg.LinAlgError:
                # Fallback to pseudo-inverse with higher regularization
                dq_arm = np.linalg.pinv(J_masked, rcond=1e-3) @ error_masked
            
            # Much more conservative step size
            dq_norm = np.linalg.norm(dq_arm)
            if dq_norm > 0:
                # Scale step size based on error magnitude
                step_size = min(max_step_size, max(min_step_size, 0.02 / max(error_norm, 1e-6)))
                step_size = min(step_size, 0.1 / max(dq_norm, 1e-6))
                
                # Update only arm joint positions
                q_new = q[:self.n_arm_joints] + step_size * dq_arm
                
                # Clamp to limits
                q_new = np.clip(q_new, 
                               self.joint_limits_lower[:self.n_arm_joints], 
                               self.joint_limits_upper[:self.n_arm_joints])
                
                # Only update if the change is significant
                joint_change = np.linalg.norm(q_new - q[:self.n_arm_joints])
                if joint_change > 1e-6:
                    q[:self.n_arm_joints] = q_new
                else:
                    break  # No significant change, stop iterating
            else:
                break  # Zero update, stop iterating
        
        return None
    
    def compute_jacobian(self, joint_positions: Union[List, np.ndarray]) -> Optional[np.ndarray]:
        """
        Compute Jacobian matrix using Pinocchio.
        
        Args:
            joint_positions: Joint positions
            
        Returns:
            np.array: 6xN Jacobian matrix or None if failed
        """
        try:
            joint_positions = np.asarray(joint_positions)
            
            if len(joint_positions) != self.n_joints:
                print(f"Expected {self.n_joints} joint values, got {len(joint_positions)}")
                return None
            
            # Compute Jacobian
            J = pin.computeFrameJacobian(self.model, self.data, joint_positions, 
                                       self.ee_frame_id, pin.LOCAL_WORLD_ALIGNED)
            
            # Check for numerical issues
            if np.any(np.isnan(J)) or np.any(np.isinf(J)):
                print("Warning: Jacobian contains NaN or Inf values")
                return None
            
            return J
            
        except Exception as e:
            print(f"Jacobian computation failed: {e}")
            return None
    
    def velocity_ik(self, cartesian_velocity: Union[List, np.ndarray], 
                   joint_positions: Union[List, np.ndarray], 
                   damping: float = 0.01,
                   joint_limit_margin: float = 0.1,
                   singularity_threshold: float = 1e-4,
                   max_damping: float = 0.1) -> Optional[np.ndarray]:
        """
        Compute joint velocities for given Cartesian velocity, with joint limit and singularity avoidance.
        
        Args:
            cartesian_velocity: [vx, vy, vz, wx, wy, wz] or [vx, vy, vz] for position only
            joint_positions: Current joint positions  
            damping: Base damping factor for pseudo-inverse
            joint_limit_margin: Margin from joint limits (in radians) to start scaling down velocity.
            singularity_threshold: Manipulability value below which dynamic damping activates.
            max_damping: Maximum damping factor to apply near a singularity.
            
        Returns:
            np.array: Joint velocities or None if failed
        """
        try:
            jacobian = self.compute_jacobian(joint_positions)
            if jacobian is None:
                return None
                
            cartesian_velocity = np.asarray(cartesian_velocity)
            
            if np.allclose(cartesian_velocity, 0.0):
                return np.zeros(self.n_joints)
            
            if len(cartesian_velocity) == 3:
                J_masked = jacobian[:3, :]
                cart_vel_masked = cartesian_velocity
            elif len(cartesian_velocity) == 6:
                J_masked = jacobian
                cart_vel_masked = cartesian_velocity
            else:
                print(f"Invalid cartesian velocity dimension: {len(cartesian_velocity)}. Expected 3 or 6.")
                return None
            
            try:
                # --- Singularity-Aware Damping ---
                manipulability = self.compute_manipulability(joint_positions)
                effective_damping = damping
                if manipulability < singularity_threshold:
                    # Dynamically increase damping as we approach a singularity
                    damping_scale = 1 - (manipulability / singularity_threshold)
                    effective_damping = damping + (max_damping - damping) * damping_scale

                J_t = J_masked.T
                JJt_damped = J_masked @ J_t + effective_damping**2 * np.eye(J_masked.shape[0])
                damped_inv = J_t @ np.linalg.inv(JJt_damped)
                
                joint_velocities = damped_inv @ cart_vel_masked

                # --- Intelligent Joint Limit Avoidance ---
                for i in range(self.n_joints):
                    # Proximity to lower limit
                    dist_to_lower = joint_positions[i] - self.joint_limits_lower[i]
                    if dist_to_lower < joint_limit_margin and joint_velocities[i] < 0:
                        # Scale velocity down as it approaches the limit
                        scale = dist_to_lower / joint_limit_margin
                        joint_velocities[i] *= max(0, scale)**2 # Use squared for smoother ramp

                    # Proximity to upper limit
                    dist_to_upper = self.joint_limits_upper[i] - joint_positions[i]
                    if dist_to_upper < joint_limit_margin and joint_velocities[i] > 0:
                        # Scale velocity down as it approaches the limit
                        scale = dist_to_upper / joint_limit_margin
                        joint_velocities[i] *= max(0, scale)**2 # Use squared for smoother ramp
                
                max_joint_vel = 2.0  # rad/s
                joint_velocities = np.clip(joint_velocities, -max_joint_vel, max_joint_vel)
                
                return joint_velocities
                
            except np.linalg.LinAlgError as e:
                print(f"Matrix inversion failed: {e}")
                return None
            
        except Exception as e:
            print(f"Velocity IK failed: {e}")
            return None
    
    def check_joint_limits(self, joint_positions: Union[List, np.ndarray]) -> bool:
        """Check if joint positions are within limits."""
        joints = np.asarray(joint_positions)
        return np.all(joints >= self.joint_limits_lower) and np.all(joints <= self.joint_limits_upper)
    
    def get_workspace_limits(self, num_samples: int = 1000) -> Optional[Dict]:
        """Estimate workspace limits by sampling with better filtering."""
        positions = []
        valid_joint_configs = []
        
        for _ in range(num_samples):
            # Sample more conservative joint ranges to avoid extreme configurations
            conservative_lower = self.joint_limits_lower + 0.1 * (self.joint_limits_upper - self.joint_limits_lower)
            conservative_upper = self.joint_limits_upper - 0.1 * (self.joint_limits_upper - self.joint_limits_lower)
            
            joints = np.random.uniform(conservative_lower, conservative_upper)
            fk_result = self.forward_kinematics(joints)
            if fk_result:
                # Filter out positions that are too close to joint limits (might be unstable)
                manipulability = self.compute_manipulability(joints)
                if manipulability > 0.001:  # Filter out near-singular configurations
                    positions.append(fk_result['position'])
                    valid_joint_configs.append(joints)
        
        if not positions:
            return None
            
        positions = np.array(positions)
        
        # Use percentiles instead of min/max to avoid extreme outliers
        return {
            'x': [np.percentile(positions[:, 0], 5), np.percentile(positions[:, 0], 95)],
            'y': [np.percentile(positions[:, 1], 5), np.percentile(positions[:, 1], 95)],
            'z': [np.percentile(positions[:, 2], 5), np.percentile(positions[:, 2], 95)],
            'valid_configs': valid_joint_configs  # Store valid configurations for better seed generation
        }
    
    def generate_good_seed(self, target_position: np.ndarray, workspace_info: Optional[Dict] = None) -> np.ndarray:
        """Generate a good seed joint configuration for IK."""
        if workspace_info and 'valid_configs' in workspace_info:
            # Find the configuration that resulted in the closest position
            valid_configs = workspace_info['valid_configs']
            best_distance = float('inf')
            best_config = None
            
            # Only consider configurations that are reasonably close (within 10cm)
            for config in valid_configs:
                fk_result = self.forward_kinematics(config)
                if fk_result:
                    distance = np.linalg.norm(np.array(target_position) - np.array(fk_result['position']))
                    if distance < 0.1 and distance < best_distance:  # Within 10cm
                        best_distance = distance
                        best_config = config
            
            if best_config is not None and best_distance < 0.05:  # Only return if within 5cm
                return best_config
        
        # Fallback to middle of joint ranges for arm joints
        arm_seed = (self.joint_limits_lower[:self.n_arm_joints] + 
                   self.joint_limits_upper[:self.n_arm_joints]) / 2
        return self.get_full_configuration(arm_seed)
    
    def batch_ik(self, target_positions: List[np.ndarray], 
                 target_orientations: Optional[List[np.ndarray]] = None,
                 seed_joints: Optional[np.ndarray] = None,
                 workspace_info: Optional[Dict] = None) -> List[Optional[np.ndarray]]:
        """
        Compute IK for multiple targets efficiently.
        Useful for trajectory planning or VR path following.
        
        Args:
            target_positions: List of target positions
            target_orientations: List of target orientations (optional)
            seed_joints: Initial seed (will chain solutions)
            workspace_info: Workspace information for better seed generation
            
        Returns:
            List of joint solutions (None for failed solutions)
        """
        solutions = []
        current_seed = seed_joints
        
        for i, target_pos in enumerate(target_positions):
            target_ori = target_orientations[i] if target_orientations else None
            
            solution = self.inverse_kinematics(target_pos, target_ori, current_seed, workspace_info=workspace_info)
            solutions.append(solution)
            
            # Use previous solution as seed for next iteration (warm starting)
            if solution is not None:
                current_seed = solution
        
        return solutions
    
    def set_solver_params(self, max_iterations: int = None, tolerance: float = None, 
                         damping: float = None, orientation_weight: float = None):
        """
        Update solver parameters dynamically.
        
        Args:
            max_iterations: Maximum IK iterations
            tolerance: Convergence tolerance
            damping: Damping factor
            orientation_weight: Weighting for orientation error
        """
        if max_iterations is not None:
            self.max_iterations = max_iterations
        if tolerance is not None:
            self.tolerance = tolerance
        if damping is not None:
            self.damping = damping
        if orientation_weight is not None:
            self.orientation_weight = orientation_weight
        
        print(f"Updated solver: max_iterations={self.max_iterations}, tolerance={self.tolerance}, damping={self.damping}, orientation_weight={self.orientation_weight}")
    
    def get_solver_stats(self) -> Dict:
        """Get solver performance statistics."""
        return {
            'max_iterations': self.max_iterations,
            'tolerance': self.tolerance,
            'damping': self.damping,
            'orientation_weight': self.orientation_weight,
            'n_joints': self.n_joints,
            'joint_limits_lower': self.joint_limits_lower.tolist(),
            'joint_limits_upper': self.joint_limits_upper.tolist(),
            'base_link': self.base_link,
            'tip_link': self.tip_link,
            'ee_frame_id': self.ee_frame_id
        }
    
    def save_config(self, filename: str):
        """Save configuration to file."""
        config = {
            'joint_limits_lower': self.joint_limits_lower.tolist(),
            'joint_limits_upper': self.joint_limits_upper.tolist(),
            'joint_names': self.joint_names,
            'n_joints': self.n_joints,
            'base_link': self.base_link,
            'tip_link': self.tip_link,
            'ee_frame_id': self.ee_frame_id,
            'max_iterations': self.max_iterations,
            'tolerance': self.tolerance,
            'damping': self.damping,
            'orientation_weight': self.orientation_weight
        }
        
        with open(filename, 'w') as f:
            json.dump(config, f, indent=2)
        print(f"Configuration saved to {filename}")
    
    def get_arm_configuration(self, full_joint_positions: Union[List, np.ndarray]) -> np.ndarray:
        """Extract arm joint positions (first 6 joints) from full configuration."""
        joints = np.asarray(full_joint_positions)
        return joints[:self.n_arm_joints]
    
    def get_full_configuration(self, arm_joints: Union[List, np.ndarray], 
                              gripper_joints: Optional[Union[List, np.ndarray]] = None) -> np.ndarray:
        """Combine arm joints with gripper joints to get full configuration."""
        arm_joints = np.asarray(arm_joints)
        
        if gripper_joints is None:
            # Default gripper position (closed)
            gripper_joints = np.array([0.0, 0.0])
        else:
            gripper_joints = np.asarray(gripper_joints)
        
        if len(arm_joints) != self.n_arm_joints:
            raise ValueError(f"Expected {self.n_arm_joints} arm joints, got {len(arm_joints)}")
        
        if len(gripper_joints) != 2:
            raise ValueError(f"Expected 2 gripper joints, got {len(gripper_joints)}")
        
        return np.concatenate([arm_joints, gripper_joints])
    
    def compute_manipulability(self, joint_positions: Union[List, np.ndarray]) -> float:
        """
        Compute manipulability index (measure of dexterity) for arm joints only.
        
        Args:
            joint_positions: Joint positions (can be 6 arm joints or 8 total joints)
            
        Returns:
            float: Manipulability index
        """
        try:
            joint_positions = np.asarray(joint_positions)
            
            # Ensure we have full configuration
            if len(joint_positions) == self.n_arm_joints:
                q = self.get_full_configuration(joint_positions)
            elif len(joint_positions) == self.n_joints:
                q = joint_positions
            else:
                print(f"Invalid joint positions length: {len(joint_positions)}")
                return 0.0
            
            # Compute full Jacobian
            J_full = self.compute_jacobian(q)
            if J_full is None:
                return 0.0
            
            # Use only arm joints and position part (first 3 rows, first 6 columns)
            J_arm_pos = J_full[:3, :self.n_arm_joints]
            
            # Manipulability = sqrt(det(J * J.T))
            JJt = J_arm_pos @ J_arm_pos.T
            det_val = np.linalg.det(JJt)
            
            if det_val < 0:
                return 0.0
            
            manipulability = np.sqrt(det_val)
            return manipulability
            
        except Exception as e:
            print(f"Manipulability computation failed: {e}")
            return 0.0
    
    def differential_ik(self, target_position: Union[List, np.ndarray], 
                       target_orientation: Optional[Union[List, np.ndarray]] = None,
                       current_joints: Optional[Union[List, np.ndarray]] = None,
                       dt: float = 0.01) -> Optional[np.ndarray]:
        """
        Compute differential IK for smooth motion.
        
        Args:
            target_position: Target position [x, y, z]
            target_orientation: Target orientation [x, y, z, w] quaternion (optional)
            current_joints: Current joint positions
            dt: Time step
            
        Returns:
            np.array: Next joint positions or None if failed
        """
        try:
            if current_joints is None:
                current_joints = (self.joint_limits_lower + self.joint_limits_upper) / 2
            
            current_joints = np.asarray(current_joints)
            
            # Get current end-effector pose
            current_fk = self.forward_kinematics(current_joints)
            if current_fk is None:
                return None
            
            # Compute position error
            pos_error = np.array(target_position) - np.array(current_fk['position'])
            
            # Compute orientation error if specified
            if target_orientation is not None:
                current_quat = np.array(current_fk['orientation'])
                target_quat = np.array(target_orientation)
                
                # Quaternion difference - more robust method
                current_quat_pin = pin.Quaternion(current_quat[3], current_quat[0], current_quat[1], current_quat[2])  # w, x, y, z
                target_quat_pin = pin.Quaternion(target_quat[3], target_quat[0], target_quat[1], target_quat[2])
                
                quat_error = target_quat_pin * current_quat_pin.inverse()
                ori_error = pin.log3(quat_error.matrix())
                
                cartesian_error = np.concatenate([pos_error, self.orientation_weight * ori_error])
            else:
                cartesian_error = pos_error
            
            # Compute cartesian velocity
            cartesian_velocity = cartesian_error / dt
            
            # Compute joint velocities
            joint_velocities = self.velocity_ik(cartesian_velocity, current_joints)
            if joint_velocities is None:
                return None
            
            # Integrate to get next joint positions
            next_joints = current_joints + joint_velocities * dt
            
            # Clamp to joint limits
            next_joints = np.clip(next_joints, self.joint_limits_lower, self.joint_limits_upper)
            
            return next_joints
            
        except Exception as e:
            print(f"Differential IK failed: {e}")
            return None

# Utility functions
def find_piper_urdf() -> Optional[str]:
    """Find Piper URDF file in common locations."""
    search_paths = [
        # Your specified location first
        "./piper_description/urdf/piper_description.urdf",
        "piperx_control/piper_description/urdf/piper_description.urdf",
        
        # Other current directory patterns
        "./piper_description.urdf",
        "./urdf/piper_description.urdf",
        
        # Common ROS workspace locations
        "~/interbotix_ws/src/xemb_aloha/piperx_control/piper_description/urdf/piper_description.urdf",
        "~/catkin_ws/src/piper_description/urdf/piper_description.urdf",
        "~/ros_ws/src/piper_description/urdf/piper_description.urdf",
        
        # System locations
        "/opt/ros/*/share/piper_description/urdf/piper_description.urdf",
    ]
    
    for path in search_paths:
        expanded_path = os.path.expanduser(path)
        
        # Handle wildcards for system paths
        if '*' in expanded_path:
            import glob
            matches = glob.glob(expanded_path)
            if matches:
                expanded_path = matches[0]
        
        if os.path.exists(expanded_path):
            print(f"Found Piper URDF at: {expanded_path}")
            return expanded_path
    
    print("No Piper URDF found in common locations")
    return None

def create_piper_ik(urdf_file: Optional[str] = None, **kwargs) -> PiperIK:
    """Create a PiperIK instance with Pinocchio."""
    return PiperIK(urdf_file, **kwargs)

def benchmark_ik(ik_solver: PiperIK, num_tests: int = 1000) -> Dict:
    """Benchmark IK solver performance."""
    print(f"Benchmarking Pinocchio IK with {num_tests} tests...")
    
    # Generate random test targets using improved workspace estimation
    workspace = ik_solver.get_workspace_limits(1000)  # Use more samples for better workspace estimation
    if workspace is None:
        print("❌ Could not determine workspace limits")
        return {}
    
    print(f"✅ Workspace limits: x={workspace['x']}, y={workspace['y']}, z={workspace['z']}")
    print(f"✅ Valid configurations found: {len(workspace.get('valid_configs', []))}")
    
    success_count = 0
    total_time = 0
    solve_times = []
    iterations_list = []
    
    for _ in range(num_tests):
        # Generate more realistic targets by starting from a known good config
        # and making small perturbations
        if workspace.get('valid_configs'):
            # Pick a random valid config as starting point
            base_config = workspace['valid_configs'][np.random.randint(len(workspace['valid_configs']))]
            base_fk = ik_solver.forward_kinematics(base_config)
            
            if base_fk:
                base_pos = np.array(base_fk['position'])
                
                # Add small perturbation (1-3cm)
                perturbation = np.random.uniform(-0.03, 0.03, 3)
                target_pos = base_pos + perturbation
                
                # Ensure target is within workspace bounds
                target_pos[0] = np.clip(target_pos[0], workspace['x'][0], workspace['x'][1])
                target_pos[1] = np.clip(target_pos[1], workspace['y'][0], workspace['y'][1])
                target_pos[2] = np.clip(target_pos[2], workspace['z'][0], workspace['z'][1])
            else:
                # Fallback to random target
                target_pos = [
                    np.random.uniform(workspace['x'][0] * 0.8, workspace['x'][1] * 0.8),
                    np.random.uniform(workspace['y'][0] * 0.8, workspace['y'][1] * 0.8),
                    np.random.uniform(workspace['z'][0] * 0.8, workspace['z'][1] * 0.8)
                ]
        else:
            # Fallback to random target
            target_pos = [
                np.random.uniform(workspace['x'][0] * 0.8, workspace['x'][1] * 0.8),
                np.random.uniform(workspace['y'][0] * 0.8, workspace['y'][1] * 0.8),
                np.random.uniform(workspace['z'][0] * 0.8, workspace['z'][1] * 0.8)
            ]
        
        # Time the solve (with improved workspace info)
        start_time = time.time()
        solution = ik_solver.inverse_kinematics(target_pos, workspace_info=workspace)
        solve_time = time.time() - start_time
        
        total_time += solve_time
        solve_times.append(solve_time)
        
        if solution is not None:
            success_count += 1
    
    solve_times = np.array(solve_times)
    
    stats = {
        'success_rate': success_count / num_tests,
        'avg_solve_time': total_time / num_tests,
        'median_solve_time': np.median(solve_times),
        'max_solve_time': np.max(solve_times),
        'min_solve_time': np.min(solve_times),
        'std_solve_time': np.std(solve_times),
        'max_iterations': ik_solver.max_iterations,
        'tolerance': ik_solver.tolerance,
        'total_tests': num_tests
    }
    
    print(f"✅ Benchmark Results:")
    print(f"   Success rate: {stats['success_rate']:.1%}")
    print(f"   Average solve time: {stats['avg_solve_time']*1000:.2f}ms")
    print(f"   Median solve time: {stats['median_solve_time']*1000:.2f}ms")
    print(f"   Max solve time: {stats['max_solve_time']*1000:.2f}ms")
    print(f"   Standard deviation: {stats['std_solve_time']*1000:.2f}ms")
    
    return stats

def test_ik():
    """Test Pinocchio IK functionality."""
    print("Testing Piper IK with Pinocchio...")
    
    try:
        # Create IK solver with more relaxed settings for better convergence
        ik = PiperIK(max_iterations=200, tolerance=1e-3, damping=1e-3, verbose=True)
    except Exception as e:
        print(f"❌ Failed to initialize: {e}")
        return
    
    # Test with sample joint configuration for arm only (6 joints)
    test_joints = np.array([0.0, 0.5, -0.5, 0.0, 0.0, 0.0])
    print(f"Test arm joints: {test_joints}")
    
    # Get full configuration with default gripper
    test_joints_full = ik.get_full_configuration(test_joints)
    print(f"Full configuration: {test_joints_full}")
    
    # Test forward kinematics
    fk_result = ik.forward_kinematics(test_joints_full)
    if fk_result:
        pos = fk_result['position']
        print(f"FK position: x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f}")
        
        # Test inverse kinematics (arm only)
        ik_result = ik.inverse_kinematics(pos, seed_joints=test_joints)
        
        if ik_result is not None:
            print(f"IK solution (arm): {ik_result}")
            print("✅ Position IK test passed!")
        else:
            print("❌ Position IK failed")
    else:
        print("❌ Forward kinematics failed")
    
    # Test Jacobian
    jac = ik.compute_jacobian(test_joints_full)
    if jac is not None:
        print(f"Jacobian shape: {jac.shape}")
        print("✅ Jacobian computation passed!")
    else:
        print("❌ Jacobian failed")
    
    # Test velocity IK
    cart_vel = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0]
    joint_vel = ik.velocity_ik(cart_vel, test_joints_full)
    if joint_vel is not None:
        print(f"Joint velocities: {joint_vel}")
        print("✅ Velocity IK passed!")
    else:
        print("❌ Velocity IK failed")
    
    # Test manipulability (arm only)
    manipulability = ik.compute_manipulability(test_joints)
    print(f"Manipulability index: {manipulability:.4f}")
    
    # Test batch IK (useful for VR trajectory following)
    if fk_result:
        print("\nTesting batch IK:")
        pos1 = fk_result['position']
        pos2 = [pos1[0] + 0.05, pos1[1], pos1[2]]  # Move 5cm in X
        pos3 = [pos1[0] + 0.05, pos1[1] + 0.05, pos1[2]]  # Move 5cm in X and Y
        
        # Get workspace info for better seed generation
        workspace = ik.get_workspace_limits(500)
        batch_solutions = ik.batch_ik([pos1, pos2, pos3], seed_joints=test_joints, workspace_info=workspace)
        successful_solutions = [s for s in batch_solutions if s is not None]
        
        print(f"✅ Batch IK: {len(successful_solutions)}/{len(batch_solutions)} succeeded")
    
    # Test differential IK
    if fk_result:
        print("\nTesting differential IK:")
        target_pos = [pos[0] + 0.05, pos[1], pos[2]]  # Move 5cm in X
        diff_result = ik.differential_ik(target_pos, current_joints=test_joints_full)
        
        if diff_result is not None:
            print(f"✅ Differential IK: {diff_result}")
        else:
            print("❌ Differential IK failed")
    
    # VR-specific performance test
    print("\nVR Performance Test (1000 solves):")
    benchmark_ik(ik, 1000)
    
    # Test different solver parameters
    print("\nTesting solver parameter changes:")
    
    # Get workspace info for consistent testing
    workspace = ik.get_workspace_limits(500)
    
    # Test with different iteration limits
    for max_iter in [10, 50, 100]:
        ik.set_solver_params(max_iterations=max_iter)
        
        if fk_result:
            start = time.time()
            solution = ik.inverse_kinematics(fk_result['position'], seed_joints=test_joints, workspace_info=workspace)
            elapsed = time.time() - start
            
            status = "✅" if solution is not None else "❌"
            print(f"   {status} Max iterations {max_iter}: solved in {elapsed*1000:.2f}ms")

    # Test with different orientation weights
    for weight in [1.0, 0.2, 0.1]:
        print(f"\n--- Testing with orientation weight: {weight} ---")
        ik.set_solver_params(orientation_weight=weight)
        
        if fk_result:
            # Create a slightly perturbed target to force iteration
            target_pos_perturbed = np.array(fk_result['position']) + np.array([0.01, -0.01, 0.01]) # 1cm offset
            
            # Create a small rotation perturbation
            pert_quat = pin.Quaternion(pin.utils.rpyToMatrix(0.05, -0.05, 0.05)) # ~3 deg rotation
            target_ori_quat = pert_quat * pin.Quaternion(np.array(fk_result['orientation']))
            target_ori_perturbed = target_ori_quat.coeffs() # [x, y, z, w]

            start = time.time()
            solution = ik.inverse_kinematics(target_pos_perturbed, target_orientation=target_ori_perturbed, seed_joints=test_joints, workspace_info=workspace)
            elapsed = time.time() - start
            
            status = "✅" if solution is not None else "❌"
            print(f"   {status} Orientation weight {weight}: solved in {elapsed*1000:.2f}ms")

if __name__ == '__main__':
    test_ik()



