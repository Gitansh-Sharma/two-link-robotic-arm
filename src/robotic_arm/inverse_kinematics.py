"""Analytical inverse kinematics for a two-link planar robotic arm."""

import numpy as np

from robotic_arm.parameters import L1, L2
from robotic_arm.kinematics import forward_kinematics


def inverse_kinematics(x, y, elbow="down", tolerance=1e-9):
    """
    Calculate joint angles required to reach a target position.

    Parameters
    ----------
    x, y : float
        Desired end-effector coordinates in metres.

    elbow : str
        Either 'up' or 'down'.

    tolerance : float
        Numerical tolerance for reachability checks.

    Returns
    -------
    tuple or None
        Joint angles (theta1, theta2) in radians.
        Returns None if the target is unreachable.
    """

    # Calculate squared distance from the base
    r_squared = x**2 + y**2

    # Calculate cosine of theta2
    D = ( r_squared - L1**2 - L2**2 ) / (2 * L1 * L2)

    # Check reachability with numerical tolerance
    if D < -1 - tolerance or D > 1 + tolerance:
        return None

    # Clamp small numerical errors
    D = np.clip(D, -1.0, 1.0)

    # Determine elbow configuration
    if elbow == "down":
        sign = 1
    elif elbow == "up":
        sign = -1
    else:
        raise ValueError("Elbow must be 'up' or 'down'.")

    sin_theta2 = sign * np.sqrt(1 - D**2)

    theta2 = np.arctan2(sin_theta2, D)

    # Calculate theta1
    phi = np.arctan2(y, x)

    beta = np.arctan2(
        L2 * np.sin(theta2),
        L1 + L2 * np.cos(theta2)
    )

    theta1 = phi - beta

    return float(theta1), float(theta2)