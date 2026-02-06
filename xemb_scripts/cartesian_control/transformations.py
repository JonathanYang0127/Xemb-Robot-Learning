from scipy.spatial.transform import Rotation as R
import numpy as np

# def simplify_angle(angle):
#     angle = angle.copy()
#     for i in range(len(angle)):
#         values = np.array([angle[i], angle[i] - np.pi, angle[i] + np.pi])
#         best_ind = np.argmin(np.abs(values))
#         angle[i] = values[best_ind]
#     return angle

def quat_to_euler(quat, degrees=False):
    """Convert quaternion [x, y, z, w] to Euler angles [roll, pitch, yaw]."""
    euler = R.from_quat(quat).as_euler('xyz', degrees=degrees)
    return euler

def euler_to_quat(euler, degrees=False):
    """Convert Euler angles [roll, pitch, yaw] to quaternion [x, y, z, w]."""
    return R.from_euler('xyz', euler, degrees=degrees).as_quat()

def quat_diff(target, source):
    """Compute the quaternion difference between target and source."""
    result = R.from_quat(target) * R.from_quat(source).inv()
    return result.as_quat()

def angle_diff(target, source):
    """Compute the Euler angle difference between target and source."""
    result = R.from_euler('xyz', target) * R.from_euler('xyz', source).inv()
    return result.as_euler('xyz')

def add_angles(delta, source, degrees=False):
    """Add delta rotation to source rotation."""
    delta_rot = R.from_euler('xyz', delta, degrees=degrees)
    source_rot = R.from_euler('xyz', source, degrees=degrees)
    new_rot = delta_rot * source_rot
    return new_rot.as_euler('xyz', degrees=degrees)

def pose_diff(target, source):
    """Compute pose difference including position and orientation."""
    diff = np.zeros(len(target))
    diff[:3] = target[:3] - source[:3]
    diff[3:6] = angle_diff(target[3:6], source[3:6])
    diff[6] = target[6] - source[6]
    return diff

def rmat_to_euler(rot_mat, degrees=False):
    """Convert rotation matrix to Euler angles."""
    euler = R.from_matrix(rot_mat).as_euler('xyz', degrees=degrees)
    return euler

def euler_to_rmat(euler, degrees=False):
    """Convert Euler angles to rotation matrix."""
    return R.from_euler('xyz', euler, degrees=degrees).as_matrix()

def rmat_to_quat(rot_mat):
    """Convert rotation matrix to quaternion [x, y, z, w]."""
    quat = R.from_matrix(rot_mat).as_quat()
    return quat

def quat_to_rmat(quat):
    """Convert quaternion [x, y, z, w] to rotation matrix."""
    return R.from_quat(quat).as_matrix()

# === VR-specific quaternion utilities (consolidated from VR files) ===

def rotation_matrix_to_quaternion(R):
    """
    Convert 3x3 rotation matrix to quaternion [w, x, y, z].
    
    Note: This returns quaternion in [w, x, y, z] format (w-first),
    which is different from scipy's [x, y, z, w] format.
    This is kept for compatibility with existing VR code.
    """
    try:
        # Use scipy for the conversion, then reorder
        quat_scipy = rmat_to_quat(R)  # [x, y, z, w]
        # Reorder to [w, x, y, z] for VR compatibility
        w = quat_scipy[3]
        x = quat_scipy[0] 
        y = quat_scipy[1]
        z = quat_scipy[2]
        
        quat = np.array([w, x, y, z], dtype=np.float32)
        norm = np.linalg.norm(quat)
        return quat / norm if norm > 0 else np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
    except Exception:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)

def quaternion_to_euler_wxyz(quat):
    """
    Convert quaternion [w, x, y, z] to Euler angles [roll, pitch, yaw].
    
    This function expects quaternion in [w, x, y, z] format (w-first),
    which is used in VR code, and converts to scipy format for processing.
    """
    try:
        w, x, y, z = quat
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = np.arcsin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        return np.array([roll, pitch, yaw])
        
    except Exception as e:
        print(f"Error converting quaternion to Euler: {e}")
        return np.zeros(3)

def normalize_quaternion(quat):
    """Normalize a quaternion to unit length."""
    norm = np.linalg.norm(quat)
    return quat / norm if norm > 0 else np.array([1.0, 0.0, 0.0, 0.0])

def quaternion_multiply(q1, q2):
    """
    Multiply two quaternions in [w, x, y, z] format.
    
    Args:
        q1, q2: quaternions in [w, x, y, z] format
    
    Returns:
        Product quaternion in [w, x, y, z] format
    """
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    
    return np.array([w, x, y, z])

def quaternion_conjugate(quat):
    """Return the conjugate of a quaternion [w, x, y, z] -> [w, -x, -y, -z]."""
    w, x, y, z = quat
    return np.array([w, -x, -y, -z])

# === Compatibility aliases ===
# Keep old function names for backwards compatibility
rotation_matrix_to_quaternion_from_matrix = rotation_matrix_to_quaternion
quaternion_to_euler = quaternion_to_euler_wxyz
