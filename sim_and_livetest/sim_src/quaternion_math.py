
import numpy as np
from numpy import cos, sin

#------------------------------quaternion math------------------------------------------------------
def hamiltonian_product(q1:iter, q2:iter):
    '''Computes the hamiltonian product between two quaternions
    
    parameters:
    q1 (iterative): quaternion 1. Must be of length 4
    q2 (iterative): quaternion 2. Must be of length 4
    
    returns the hamiltonian product as a new quaternion'''


    a1, b1, c1, d1 = q1
    a2, b2, c2, d2 = q2


    # From the wikipedia page on quaternions. Hamiltonian product
    term1 = a1*a2 - b1*b2 - c1*c2 - d1*d2

    term2 = a1*b2 + b1*a2 + c1*d2 - d1*c2

    term3 = a1*c2 - b1*d2 + c1*a2 + d1*b2

    term4 = a1*d2 + b1*c2 - c1*b2 + d1*a2

    p = np.array([term1, term2, term3, term4])

    return p


def conjugate_quaternion(q:iter):
    '''used for conjugating quaternions
    
    Parameters:
    q (iterative): quaternion. Must be of length 4
    
    returns the conjugate quaternion as a list'''

    # Conjugate is found by negating the vector part
    q_conjugate = np.array([q[0], -q[1], -q[2], -q[3]])
    
    return q_conjugate


def inverse_quaternion(q:iter):
    '''Finds the inverse of a quaternion
    
    Parameters:
    q (iterative): quaternion. Must be of length 4
    
    Returns the inverse quaternion as a list'''

    # Square each entry and sum to get norm
    q_norm = sum([element**2 for element in q])
    
    q_conjugate = conjugate_quaternion(q)

    q_inverse = 1/q_norm * q_conjugate #element for element in q_conjugate]

    return q_inverse


def rotation_product(p:iter, q:iter):
    '''Rotate quaternion p with quaternion q
    
    Parameters:
    p (iterative): quaternion to be rotated
    q (iterative): quaternion to rotate with
    
    returns the rotated quaternion as a list'''

    p_rotated = hamiltonian_product(q, hamiltonian_product(p, inverse_quaternion(q)))
    
    return p_rotated


def rotate_vector(v:iter, q:iter):
    '''Rotate vector v with quaternion q
    
    Parameters:
    v (iterative): vector to be rotated. Must be of length 3
    q (iterative): quaternion to rotate with. Must be of length 4
    
    returns the rotated vector as a list'''

    # Convert vector to quaternion with 0 scalar part


    v_as_quat = np.array([0.0, v[0], v[1], v[2]])

    v_rotated_quat = rotation_product(v_as_quat, q)

    v_rotated = np.array([v_rotated_quat[1],
                          v_rotated_quat[2],
                          v_rotated_quat[3]])

    return v_rotated




#------------------------------quaternion to angles (roll, pitch, yaw)------------------------------
def quaternion_to_euler_angles(q:iter):
    '''Convert quaternion to euler angles (roll, pitch, yaw)
    
    Parameters:
    q (iterative): quaternion. Must be of length 4
    
    returns the euler angles as a tuple (roll, pitch, yaw) in radians'''

    # print(f"{q = }")
    qw, qx, qy, qz = q

    # roll (x-axis rotation)
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = np.sign(sinp) * (np.pi / 2)  # use 90 degrees if out of range
    else:
        pitch = np.arcsin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return np.array([roll, pitch, yaw])



# utility functions
def construct_skew_matrix(u): 
    
    matrix = np.array([
        [0, -u[2], u[1]],
        [u[2], 0, -u[0]],
        [-u[1], u[0], 0]
        ])
    
    return matrix
    


def quaternion_from_rotation_matrix(R):
    """
    Convert a 3x3 rotation matrix to a quaternion [w, x, y, z].
    """
    m = R
    tr = m[0, 0] + m[1, 1] + m[2, 2]

    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2  # S=4*qw
        qw = 0.25 * S
        qx = (m[2, 1] - m[1, 2]) / S
        qy = (m[0, 2] - m[2, 0]) / S
        qz = (m[1, 0] - m[0, 1]) / S
    elif (m[0, 0] > m[1, 1]) and (m[0, 0] > m[2, 2]):
        S = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2  # S=4*qx
        qw = (m[2, 1] - m[1, 2]) / S
        qx = 0.25 * S
        qy = (m[0, 1] + m[1, 0]) / S
        qz = (m[0, 2] + m[2, 0]) / S
    elif m[1, 1] > m[2, 2]:
        S = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2  # S=4*qy
        qw = (m[0, 2] - m[2, 0]) / S
        qx = (m[0, 1] + m[1, 0]) / S
        qy = 0.25 * S
        qz = (m[1, 2] + m[2, 1]) / S
    else:
        S = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2  # S=4*qz
        qw = (m[1, 0] - m[0, 1]) / S
        qx = (m[0, 2] + m[2, 0]) / S
        qy = (m[1, 2] + m[2, 1]) / S
        qz = 0.25 * S

    q = np.array([qw, qx, qy, qz])
    return q / np.linalg.norm(q)    



def rotation_matrix_from_quaternion(quaternion):
    qw, qx, qy, qz = quaternion

    R = np.array([
        [qw**2 + qx**2 - qy**2 - qz**2,  2*(qx*qy - qw*qz),              2*(qx*qz  + qw*qy)            ],
        [2*(qx*qy + qw*qz),              qw**2 - qx**2 + qy**2 - qz**2,  2*(qy*qz - qw*qx)             ],
        [2*(qx*qz - qw*qy),              2*(qw*qx + qy*qz),              qw**2 - qx**2 - qy**2 + qz**2 ]
    ])
    
    return R



def rotation_vector_to_quaternion(rotation_vector):

    phi = np.linalg.norm(rotation_vector)

    if phi < 1e-12:
        # Very small rotation, avoid division by zero
        return np.array([1.0, 0.0, 0.0, 0.0])
    
    u = rotation_vector/phi

    # from phi and u we can construct a quaternion
    q = np.concatenate((
        np.array([cos(phi/2)]),
        sin(phi/2)*u
    ))

    return q



def rotation_vector_to_rotation_matrix(rotation_vector):

    phi = np.linalg.norm(rotation_vector)
    
    if phi < 1e-12:
        # Very small rotation, avoid division by zero
        return np.eye(3)

    u = rotation_vector/phi

    skew_matrix = construct_skew_matrix(u)
    
    R = np.eye(3) + sin(phi) * skew_matrix + (1 - cos(phi)) * skew_matrix@skew_matrix

    return R

