
from .quaternion_math import hamiltonian_product
import numpy as np

# def euler_angles_to_quaternion(roll:float, pitch:float, yaw:float):
#     '''Convert roll, pitch, yaw to quaternion format for rocket coordinate system
    
#     This function converts standard ZYX Euler angles to a quaternion for a rocket where:
#     - Z-axis points up (vertical)
#     - X-axis points east
#     - Y-axis points north
#     - pitch is the standard rotation about the y-axis used by quaternion_to_euler_angles
    
#     Parameters:
#     roll (float): rotation around x axis in radians
#     pitch (float): rotation around y axis in radians
#     yaw (float): rotation around z axis in radians

#     Returns:
#     quaternion (numpy array): [qw, qx, qy, qz] representing the rotation from earth to sensor frame
    
#     Convention: ZYX Euler angles (yaw, then pitch, then roll)
#     '''
    
#     # Calculate half angles for stability
#     hr = roll * 0.5    # half roll
#     hp = pitch * 0.5   # half pitch  
#     hy = yaw * 0.5     # half yaw
    
#     # Calculate trigonometric values
#     chr = np.cos(hr)   # cos(roll/2)
#     shr = np.sin(hr)   # sin(roll/2)
#     chp = np.cos(hp)   # cos(pitch/2)
#     shp = np.sin(hp)   # sin(pitch/2)
#     chy = np.cos(hy)   # cos(yaw/2)
#     shy = np.sin(hy)   # sin(yaw/2)

#     # Quaternion multiplication: q = q_z(yaw) * q_y(pitch) * q_x(roll)
#     # This represents rotating first by yaw around Z, then pitch around Y, then roll around X
#     qw = chr * chp * chy + shr * shp * shy
#     qx = shr * chp * chy - chr * shp * shy  
#     qy = chr * shp * chy + shr * chp * shy
#     qz = chr * chp * shy - shr * shp * chy

#     quaternion = np.array([qw, qx, qy, qz])
    
#     # Ensure unit quaternion (normalize for numerical stability)
#     magnitude = np.linalg.norm(quaternion)
#     if magnitude > 1e-12:  # Avoid division by very small numbers
#         quaternion = quaternion / magnitude
#     else:
#         # Return identity quaternion if magnitude is too small
#         quaternion = np.array([1.0, 0.0, 0.0, 0.0])
#         print(f"Warning: angle_to_quaternion received near-zero magnitude. Returning identity.")
    
#     return quaternion

def rpy_to_matrix(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    Rx = np.array([[1, 0, 0],
                   [0, cr, -sr],
                   [0, sr,  cr]])

    Ry = np.array([[ cp, 0, sp],
                   [  0, 1,  0],
                   [-sp, 0, cp]])

    Rz = np.array([[cy, -sy, 0],
                   [sy,  cy, 0],
                   [ 0,   0, 1]])

    return Rz @ Ry @ Rx


def force_components_world(F_body, roll, pitch, yaw):
    R = rpy_to_matrix(roll, pitch, yaw)
    return R @ np.asarray(F_body)  # [Fx, Fy, Fz] in world frame


def euler_angles_to_quaternion(roll, pitch, yaw):
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return np.array([qw, qx, qy, qz])


def quaternion_derivative(q, omega):
    ''' it is suspected, that this function uses the opposite convention, relative to the ESKF'''
    # q: current quaternion (4,)
    # omega: angular velocity in body frame (3,)
    omega_quat = np.array([0, *omega])

    # dqdt = 0.5 * hamiltonian_product(q, omega_quat)
    dqdt = 0.5 * hamiltonian_product(omega_quat, q)       # <- arguments shifted
    return dqdt
