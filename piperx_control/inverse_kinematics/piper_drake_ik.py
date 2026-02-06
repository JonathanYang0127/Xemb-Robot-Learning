#!/usr/bin/env python3

import numpy as np
import json
from typing import Optional, List, Dict, Tuple, Union
import warnings

try:
    from pydrake.multibody.parsing import Parser
    from pydrake.multibody.plant import MultibodyPlant
    from pydrake.systems.framework import Context
    from pydrake.multibody.inverse_kinematics import InverseKinematics
    from pydrake.solvers import Solve
    from pydrake.math import RigidTransform, RotationMatrix
    from pydrake.common import FindResourceOrThrow
    from pydrake.multibody.tree import JacobianWrtVariable
    import pydrake.geometry as geometry
    
    DRAKE_AVAILABLE = True
except ImportError:
    DRAKE_AVAILABLE = False
    warnings.warn("Drake not available. Install with: pip install drake")

class PiperIK:
    """
    Piper robot inverse kinematics using Drake.
    Modern, robust IK with excellent URDF support and singularity handling.
    """
    
    def __init__(self, urdf_file: Optional[str] = None):
        """
        Initialize the Piper IK solver with Drake.
        
        Args:
            urdf_file: Path to URDF file (will auto-find if None)
        """
        if not DRAKE_AVAILABLE:
            raise ImportError("Drake is required. Install with: pip install drake")
        
        # Find URDF file
        if urdf_file is None:
            urdf_file = find_piper_urdf()
            if urdf_file is None:
                raise FileNotFoundError("No URDF file found. Please specify urdf_file parameter.")
        
        print(f"Loading robot from: {urdf_file}")
        
        # Create MultibodyPlant and load URDF
        self.plant = MultibodyPlant(time_step=0.0)  # Continuous plant
        self.parser = Parser(self.plant)
        
        try:
            # Load the robot from URDF (Drake 1.26+ API)
            if hasattr(self.parser, 'AddModels'):
                self.model_instances = self.parser.AddModels(urdf_file)
                self.model_instance = self.model_instances[0] if self.model_instances else None
            elif hasattr(self.parser, 'AddModelFromFile'):
                self.model_instance = self.parser.AddModelFromFile(urdf_file)
            else:
                # Fallback
                self.model_instance = self.parser.AddModel(urdf_file)
            
            self.plant.Finalize()
            
            # Create context
            self.context = self.plant.CreateDefaultContext()
            
            # Get joint info (filter to only arm joints)
            self.joint_names = []
            self.joint_limits_lower = []
            self.joint_limits_upper = []
            self.arm_joint_indices = []
            
            joint_index = 0
            for joint_idx in self.plant.GetJointIndices(self.model_instance):
                joint = self.plant.get_joint(joint_idx)
                if joint.num_positions() == 1:  # Single DOF joint
                    joint_name = joint.name()
                    # Only include arm joints (joint1-joint6), skip gripper joints
                    if joint_name in ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']:
                        self.joint_names.append(joint_name)
                        self.joint_limits_lower.append(joint.position_lower_limits()[0])
                        self.joint_limits_upper.append(joint.position_upper_limits()[0])
                        self.arm_joint_indices.append(joint_index)
                    joint_index += 1
            
            # Sort joints by name to ensure correct order
            joint_order = [(name, idx, lower, upper) for name, idx, lower, upper in 
                          zip(self.joint_names, self.arm_joint_indices, self.joint_limits_lower, self.joint_limits_upper)]
            joint_order.sort(key=lambda x: int(x[0].replace('joint', '')))
            
            self.joint_names = [x[0] for x in joint_order]
            self.arm_joint_indices = [x[1] for x in joint_order]
            self.joint_limits_lower = np.array([x[2] for x in joint_order])
            self.joint_limits_upper = np.array([x[3] for x in joint_order])
            
            self.n_joints = len(self.joint_names)  # Only arm joints
            
            # Get end-effector body
            self.end_effector_body = None
            for body_index in self.plant.GetBodyIndices(self.model_instance):
                body = self.plant.get_body(body_index)
                if 'tool' in body.name().lower() or 'end' in body.name().lower() or 'gripper' in body.name().lower():
                    self.end_effector_body = body
                    break
            
            # If no end-effector found, use the last body
            if self.end_effector_body is None:
                body_indices = self.plant.GetBodyIndices(self.model_instance)
                if body_indices:
                    self.end_effector_body = self.plant.get_body(body_indices[-1])
            
            print(f"✅ Loaded Piper robot with Drake:")
            print(f"   Joints: {self.n_joints}")
            print(f"   Joint names: {self.joint_names}")
            print(f"   End-effector: {self.end_effector_body.name()}")
            print(f"   Joint limits: {self.joint_limits_lower} to {self.joint_limits_upper}")
            
        except Exception as e:
            raise RuntimeError(f"Failed to load URDF with Drake: {e}")
    
    def forward_kinematics(self, joint_positions: Union[List, np.ndarray]) -> Optional[Dict]:
        """
        Compute forward kinematics using Drake with robust singularity handling.
        
        Args:
            joint_positions: Joint positions (6 arm joints only)
            
        Returns:
            dict: {'position': [x, y, z], 'orientation': [x, y, z, w]} or None if failed
        """
        try:
            joint_positions = np.asarray(joint_positions)
            
            if len(joint_positions) != self.n_joints:
                print(f"Expected {self.n_joints} joint values, got {len(joint_positions)}")
                return None
            
            # Check for exact zero configuration (known singularity)
            if np.allclose(joint_positions, 0.0):
                print("Warning: Zero joint configuration detected, using small offset to avoid singularity")
                # Use a very small offset to avoid exact singularity
                joint_positions = joint_positions + 1e-6
            
            # Create full joint state (all joints in the model)
            full_joint_positions = np.zeros(self.plant.num_positions(self.model_instance))
            
            # Set only the arm joint positions
            for i, joint_pos in enumerate(joint_positions):
                full_joint_positions[self.arm_joint_indices[i]] = joint_pos
            
            # Set joint positions in context
            self.plant.SetPositions(self.context, self.model_instance, full_joint_positions)
            
            # Get end-effector pose
            try:
                X_WE = self.plant.CalcRelativeTransform(
                    self.context,
                    self.plant.world_frame(),
                    self.end_effector_body.body_frame()
                )
            except Exception as transform_error:
                print(f"Warning: Transform calculation failed ({transform_error}), trying alternative approach")
                # Try to get pose using different method
                try:
                    X_WE = self.plant.EvalBodyPoseInWorld(self.context, self.end_effector_body)
                except Exception:
                    print("All transform methods failed")
                    return None
            
            # Extract position and orientation
            position = X_WE.translation()
            
            # Handle potential quaternion issues (ROBUST SINGULARITY HANDLING)
            try:
                rotation_matrix = X_WE.rotation()
                
                # Check if rotation matrix is valid
                if not rotation_matrix.IsValid():
                    print("Warning: Invalid rotation matrix, using identity")
                    quaternion = np.array([1.0, 0.0, 0.0, 0.0])  # Identity quaternion
                else:
                    quaternion = rotation_matrix.ToQuaternion().wxyz()  # [w, x, y, z]
                    
                    # Check for zero quaternion (singularity case)
                    if np.allclose(quaternion, 0.0):
                        print("Warning: Zero quaternion detected, using identity rotation")
                        quaternion = np.array([1.0, 0.0, 0.0, 0.0])  # Identity quaternion
                    else:
                        # Normalize quaternion to be safe
                        quat_norm = np.linalg.norm(quaternion)
                        if quat_norm > 1e-8:
                            quaternion = quaternion / quat_norm
                        else:
                            print("Warning: Quaternion norm too small, using identity")
                            quaternion = np.array([1.0, 0.0, 0.0, 0.0])
                
            except Exception as quat_error:
                print(f"Warning: Quaternion extraction failed ({quat_error}), using identity")
                quaternion = np.array([1.0, 0.0, 0.0, 0.0])  # Identity quaternion
            
            return {
                'position': position.tolist(),
                'orientation': [quaternion[1], quaternion[2], quaternion[3], quaternion[0]]  # [x, y, z, w]
            }
            
        except Exception as e:
            print(f"Forward kinematics failed: {e}")
            return None
    
    def inverse_kinematics(self, target_position: Union[List, np.ndarray], 
                          target_orientation: Optional[Union[List, np.ndarray]] = None, 
                          seed_joints: Optional[Union[List, np.ndarray]] = None,
                          position_tolerance: float = 0.001,
                          orientation_tolerance: float = 0.01) -> Optional[np.ndarray]:
        """
        Compute inverse kinematics using Drake's robust solvers.
        
        Args:
            target_position: Target position [x, y, z]
            target_orientation: Target orientation [x, y, z, w] quaternion (optional)
            seed_joints: Seed joint positions (optional)
            position_tolerance: Position tolerance in meters
            orientation_tolerance: Orientation tolerance in radians
            
        Returns:
            np.array: Joint positions or None if failed
        """
        try:
            target_position = np.asarray(target_position)
            
            # Create IK problem
            ik = InverseKinematics(self.plant, self.context)
            
            # Set position constraint
            ik.AddPositionConstraint(
                self.end_effector_body.body_frame(),
                [0, 0, 0],  # Point on end-effector frame
                self.plant.world_frame(),
                target_position - position_tolerance,  # Lower bound
                target_position + position_tolerance   # Upper bound
            )
            
            # Set orientation constraint if provided
            if target_orientation is not None:
                quat = np.asarray(target_orientation)
                # Convert [x, y, z, w] to [w, x, y, z] for Drake
                quat_wxyz = np.array([quat[3], quat[0], quat[1], quat[2]])
                target_rotation = RotationMatrix.MakeFromQuaternion(quat_wxyz)
                
                ik.AddOrientationConstraint(
                    self.end_effector_body.body_frame(),
                    RotationMatrix(),  # Identity rotation in body frame
                    self.plant.world_frame(),
                    target_rotation,
                    orientation_tolerance
                )
            
            # Get the decision variables (only arm joints)
            prog = ik.get_mutable_prog()
            q_vars = ik.q()
            
            # Set seed configuration
            if seed_joints is not None:
                q_seed = np.asarray(seed_joints)
            else:
                # Use middle of joint limits as seed
                q_seed = (self.joint_limits_lower + self.joint_limits_upper) / 2
            
            # Create full state for initial guess
            full_q_seed = np.zeros(self.plant.num_positions(self.model_instance))
            for i, joint_pos in enumerate(q_seed):
                full_q_seed[self.arm_joint_indices[i]] = joint_pos
            
            # Set initial guess
            prog.SetInitialGuess(q_vars, full_q_seed)
            
            # Add joint limit constraints only for arm joints
            for i, joint_idx in enumerate(self.arm_joint_indices):
                prog.AddBoundingBoxConstraint(
                    self.joint_limits_lower[i],
                    self.joint_limits_upper[i],
                    q_vars[joint_idx]
                )
            
            # Solve the optimization problem
            result = Solve(prog)
            
            if result.is_success():
                full_solution = result.GetSolution(q_vars)
                
                # Extract only arm joint solutions
                solution = np.zeros(self.n_joints)
                for i, joint_idx in enumerate(self.arm_joint_indices):
                    solution[i] = full_solution[joint_idx]
                
                # Verify solution
                verify_fk = self.forward_kinematics(solution)
                if verify_fk:
                    pos_error = np.linalg.norm(np.array(target_position) - np.array(verify_fk['position']))
                    print(f"✅ Drake IK solved (position error: {pos_error:.4f}m)")
                    return solution
                else:
                    print("❌ IK solution verification failed")
                    return None
            else:
                print(f"❌ Drake IK optimization failed: {result.get_solver_details()}")
                return None
                
        except Exception as e:
            print(f"Inverse kinematics error: {e}")
            return None
    
    def compute_jacobian(self, joint_positions: Union[List, np.ndarray]) -> Optional[np.ndarray]:
        """
        Compute Jacobian matrix using Drake with robust singularity handling.
        
        Args:
            joint_positions: Joint positions (6 arm joints only)
            
        Returns:
            np.array: 6xN Jacobian matrix or None if failed
        """
        try:
            joint_positions = np.asarray(joint_positions)
            
            # Check for exact zero configuration (known singularity)
            if np.allclose(joint_positions, 0.0):
                print("Warning: Zero joint configuration detected for Jacobian, using small offset")
                # Use a very small offset to avoid exact singularity
                joint_positions = joint_positions + 1e-6
            
            # Create full joint state
            full_joint_positions = np.zeros(self.plant.num_positions(self.model_instance))
            for i, joint_pos in enumerate(joint_positions):
                full_joint_positions[self.arm_joint_indices[i]] = joint_pos
            
            # Set joint positions
            self.plant.SetPositions(self.context, self.model_instance, full_joint_positions)
            
            # Compute full Jacobian with proper arguments
            p_BoBp_B = np.array([0.0, 0.0, 0.0]).reshape(3, 1)  # Point on end-effector as column vector
            
            try:
                J_WE_full = self.plant.CalcJacobianSpatialVelocity(
                    self.context,
                    JacobianWrtVariable.kQDot,
                    self.end_effector_body.body_frame(),
                    p_BoBp_B,
                    self.plant.world_frame(),
                    self.plant.world_frame()
                )
                
                # Extract only the columns corresponding to arm joints
                J_WE = J_WE_full[:, self.arm_joint_indices]
                
                # Check for numerical issues
                if np.any(np.isnan(J_WE)) or np.any(np.isinf(J_WE)):
                    print("Warning: Jacobian contains NaN or Inf values")
                    return None
                
                # Check for near-singular configurations
                try:
                    _, s, _ = np.linalg.svd(J_WE)
                    condition_number = s[0] / s[-1] if s[-1] > 1e-10 else np.inf
                    
                    if condition_number > 1e6:
                        print(f"Warning: Near-singular configuration (condition number: {condition_number:.2e})")
                        # Still return the Jacobian but warn the user
                
                except np.linalg.LinAlgError:
                    print("Warning: SVD failed on Jacobian")
                
                return J_WE
                
            except Exception as jac_error:
                print(f"Jacobian computation failed: {jac_error}")
                return None
            
        except Exception as e:
            print(f"Jacobian computation failed: {e}")
            return None
    
    def velocity_ik(self, cartesian_velocity: Union[List, np.ndarray], 
                   joint_positions: Union[List, np.ndarray], 
                   damping: float = 0.01) -> Optional[np.ndarray]:
        """
        Compute joint velocities for given Cartesian velocity with robust singularity handling.
        
        Args:
            cartesian_velocity: [vx, vy, vz, wx, wy, wz]
            joint_positions: Current joint positions  
            damping: Damping factor for pseudo-inverse
            
        Returns:
            np.array: Joint velocities or None if failed
        """
        try:
            jacobian = self.compute_jacobian(joint_positions)
            if jacobian is None:
                return None
                
            cartesian_velocity = np.asarray(cartesian_velocity)
            
            # Check for zero velocity command
            if np.allclose(cartesian_velocity, 0.0):
                return np.zeros(self.n_joints)
            
            # Adaptive damping based on manipulability
            try:
                # Compute manipulability measure
                manipulability = np.sqrt(np.linalg.det(jacobian @ jacobian.T))
                
                # Increase damping near singularities
                if manipulability < 0.01:
                    adaptive_damping = damping * 10.0
                    print(f"Warning: Near singularity (manipulability: {manipulability:.4f}), using higher damping")
                else:
                    adaptive_damping = damping
                    
            except Exception:
                adaptive_damping = damping
            
            # Damped pseudo-inverse (more robust than regular pseudo-inverse)
            try:
                jac_t = jacobian.T
                jjt_damped = jacobian @ jac_t + adaptive_damping**2 * np.eye(6)
                damped_inv = jac_t @ np.linalg.inv(jjt_damped)
                
                # Compute joint velocities
                joint_velocities = damped_inv @ cartesian_velocity
                
                # Limit joint velocities to reasonable values
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
        """Estimate workspace limits by sampling."""
        positions = []
        
        for _ in range(num_samples):
            joints = np.random.uniform(self.joint_limits_lower, self.joint_limits_upper)
            fk_result = self.forward_kinematics(joints)
            if fk_result:
                positions.append(fk_result['position'])
        
        if not positions:
            return None
            
        positions = np.array(positions)
        return {
            'x': [np.min(positions[:, 0]), np.max(positions[:, 0])],
            'y': [np.min(positions[:, 1]), np.max(positions[:, 1])],
            'z': [np.min(positions[:, 2]), np.max(positions[:, 2])]
        }
    
    def save_config(self, filename: str):
        """Save configuration to file."""
        config = {
            'joint_limits_lower': self.joint_limits_lower.tolist(),
            'joint_limits_upper': self.joint_limits_upper.tolist(),
            'joint_names': self.joint_names,
            'n_joints': self.n_joints
        }
        
        with open(filename, 'w') as f:
            json.dump(config, f, indent=2)
        print(f"Configuration saved to {filename}")

# Utility functions
def find_piper_urdf() -> Optional[str]:
    """Find Piper URDF file in common locations."""
    import os
    
    search_paths = [
        # Your specified location first
        "./piper_description/urdf/piper_description.urdf",
        
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

def create_piper_ik(urdf_file: Optional[str] = None) -> PiperIK:
    """Create a PiperIK instance."""
    return PiperIK(urdf_file)

def test_ik():
    """Test Drake IK functionality."""
    print("Testing Piper IK with Drake...")
    
    try:
        ik = PiperIK()
    except Exception as e:
        print(f"❌ Failed to initialize: {e}")
        return
    
    # Test with sample joint configuration  
    test_joints = np.array([0.0, 0.5, -0.5, 0.0, 0.0, 0.0])
    print(f"Test joints: {test_joints}")
    
    # Test forward kinematics
    fk_result = ik.forward_kinematics(test_joints)
    if fk_result:
        pos = fk_result['position']
        print(f"FK position: x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f}")
        
        # Test inverse kinematics
        ik_result = ik.inverse_kinematics(pos, seed_joints=test_joints)
        
        if ik_result is not None:
            print(f"IK solution: {ik_result}")
            print("✅ Position IK test passed!")
        else:
            print("❌ Position IK failed")
    else:
        print("❌ Forward kinematics failed")
    
    # Test Jacobian
    jac = ik.compute_jacobian(test_joints)
    if jac is not None:
        print(f"Jacobian shape: {jac.shape}")
        print("✅ Jacobian computation passed!")
    else:
        print("❌ Jacobian failed")
    
    # Test velocity IK
    cart_vel = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0]
    joint_vel = ik.velocity_ik(cart_vel, test_joints)
    if joint_vel is not None:
        print(f"Joint velocities: {joint_vel}")
        print("✅ Velocity IK passed!")
    else:
        print("❌ Velocity IK failed")
    
    # Test different IK methods
    if fk_result:
        print("\nTesting zero joint configuration (singularity test):")
        zero_joints = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        
        # Test FK at zero (should handle singularity)
        zero_fk = ik.forward_kinematics(zero_joints)
        if zero_fk:
            print(f"✅ Zero config FK: {zero_fk['position']}")
        else:
            print("❌ Zero config FK failed")
        
        # Test Jacobian at zero
        zero_jac = ik.compute_jacobian(zero_joints)
        if zero_jac is not None:
            print(f"✅ Zero config Jacobian: {zero_jac.shape}")
        else:
            print("❌ Zero config Jacobian failed")
        
        # Test velocity IK at zero
        zero_vel_ik = ik.velocity_ik(cart_vel, zero_joints)
        if zero_vel_ik is not None:
            print(f"✅ Zero config Velocity IK: {zero_vel_ik}")
        else:
            print("❌ Zero config Velocity IK failed")

if __name__ == '__main__':
    test_ik()
