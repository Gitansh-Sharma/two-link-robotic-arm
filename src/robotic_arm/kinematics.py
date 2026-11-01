"""Forward kinematics for a two-link planar robotic arm."""

import numpy as np

from robotic_arm.parameters import L1, L2


def forward_kinematics(theta1, theta2):
    """
    Calculate the end-effector position.

    Parameters
    ----------
    theta1 : float
        First joint angle in radians, measured from the positive x-axis.
    theta2 : float
        Second joint angle in radians, measured relative to link 1.

    Returns
    -------
    tuple[float, float]
        End-effector coordinates (x, y) in metres.
    """
    x = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)
    y = L1 * np.sin(theta1) + L2 * np.sin(theta1 + theta2)

    return float(x), float(y)


def get_link_positions(theta1, theta2):
    """
    Calculate the base, first joint, and end-effector positions.

    Parameters
    theta1 : float
        First joint angle in radians.
    theta2 : float
        Second joint angle in radians.

    Returns
    numpy.ndarray
        A 3x2 array containing the coordinates of the base,
        first joint, and end effector, in that order.
    """
    joint1_x = L1 * np.cos(theta1)
    joint1_y = L1 * np.sin(theta1)

    end_x, end_y = forward_kinematics(theta1, theta2)

    return np.array([
        [0.0, 0.0],
        [joint1_x, joint1_y],
        [end_x, end_y],
    ])