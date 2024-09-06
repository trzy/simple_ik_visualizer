#
# math_helpers/rotation.py
# Bart Trzynadlowski, 2024
#
# Rotation matrices.
#

import numpy as np


def x_rotation_matrix3(degrees: float) -> np.ndarray:
    """
    Returns
    -------
    np.ndarray
        A 3x3 rotation matrix for rotation about the x axis.
    """
    theta = np.deg2rad(degrees)
    return np.array([
        [1, 0, 0],
        [0, np.cos(theta), -np.sin(theta)],
        [0, np.sin(theta), np.cos(theta)]
    ])

def y_rotation_matrix3(degrees: float) -> np.ndarray:
    """
    Returns
    -------
    np.ndarray
        A 3x3 rotation matrix for rotation about the y axis.
    """
    theta = np.deg2rad(degrees)
    return np.array([
        [np.cos(theta), 0, np.sin(theta)],
        [0, 1, 0],
        [-np.sin(theta), 0, np.cos(theta)]
    ])

def z_rotation_matrix3(degrees: float) -> np.ndarray:
    """
    Returns
    -------
    np.ndarray
        A 3x3 rotation matrix for rotation about the z axis.
    """
    theta = np.deg2rad(degrees)
    return np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta), np.cos(theta), 0],
        [0, 0, 1]
    ])

def euler_rotation_matrix3(euler_degrees: np.ndarray) -> np.ndarray:
    """
    Returns
    -------
    np.ndarray
        A 3x3 rotation matrix for rotations in the order: x (roll), y (pitch), and yaw (z). The axes
        and, consequently, the meaning of roll, pitch, and yaw follow the robotics convention of
        Z being "up".
    """
    x_rot = x_rotation_matrix3(euler_degrees[0])
    y_rot = y_rotation_matrix3(euler_degrees[1])
    z_rot = z_rotation_matrix3(euler_degrees[2])
    return np.matmul(z_rot, np.matmul(y_rot, x_rot))

def euler_rotation_matrix4(euler_degrees: np.ndarray) -> np.ndarray:
    """
    Returns
    -------
    np.ndarray
        A 4x4 rotation matrix for rotations in the order: x (roll), y (pitch), and yaw (z). The axes
        and, consequently, the meaning of roll, pitch, and yaw follow the robotics convention of
        Z being "up".
    """
    matrix = np.eye(4)
    matrix[0:3, 0:3] = euler_rotation_matrix3(euler_degrees=euler_degrees)
    return matrix