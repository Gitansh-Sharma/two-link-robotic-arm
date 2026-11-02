
import numpy as np
from robotic_arm.inverse_kinematics import inverse_kinematics

def joint_space_trajectory(
    initial_angles,
    final_angles,
    duration,
    num_points=300
):
    """
    Generate a cubic polynomial joint-space trajectory
    for a two-link planar robotic arm.

    Parameters
    ----------
    initial_angles : array-like, shape (2,)
        Initial joint angles in radians.

    final_angles : array-like, shape (2,)
        Final joint angles in radians.

    duration : float
        Total trajectory duration in seconds.

    num_points : int
        Number of time samples.

    Returns
    -------
    time : ndarray
        Time samples in seconds.

    positions : ndarray, shape (num_points, 2)
        Joint positions in radians.

    velocities : ndarray, shape (num_points, 2)
        Joint velocities in rad/s.

    accelerations : ndarray, shape (num_points, 2)
        Joint accelerations in rad/s².
    """

    initial_angles = np.asarray(initial_angles, dtype=float)
    final_angles = np.asarray(final_angles, dtype=float)

    # Input validation
    if initial_angles.shape != (2,) or final_angles.shape != (2,):
        raise ValueError("Both angle inputs must contain exactly two values.")

    if not np.all(np.isfinite(initial_angles)) or not np.all(np.isfinite(final_angles)):
        raise ValueError("Joint angles must be finite numbers.")

    if not np.isfinite(duration) or duration <= 0:
        raise ValueError("Duration must be a positive finite number.")

    if not isinstance(num_points, (int, np.integer)) or isinstance(num_points, bool) or num_points < 2:
        raise ValueError("num_points must be an integer greater than or equal to 2.")

    # Time array
    time = np.linspace(0, duration, num_points)

    # Normalized time: 0 at start, 1 at end
    s = time / duration

    # Joint displacement
    delta_angles = final_angles - initial_angles

    # Cubic polynomial trajectory
    positions = (
        initial_angles
        + delta_angles * (3 * s**2 - 2 * s**3)[:, np.newaxis]
    )

    # First derivative: angular velocity
    velocities = (
        delta_angles
        * (6 * s - 6 * s**2)[:, np.newaxis]
        / duration
    )

    # Second derivative: angular acceleration
    accelerations = (
        delta_angles
        * (6 - 12 * s)[:, np.newaxis]
        / duration**2
    )

    return time, positions, velocities, accelerations


def cartesian_space_trajectory(
    start_point,
    end_point,
    duration,
    num_points=300,
    elbow="down"
):
    """
    Generate a straight-line Cartesian trajectory for a
    two-link planar robotic arm.

    Parameters
    ----------
    start_point : array-like, shape (2,)
        Initial end-effector position [x, y] in metres.

    end_point : array-like, shape (2,)
        Final end-effector position [x, y] in metres.

    duration : float
        Total trajectory duration in seconds.

    num_points : int
        Number of trajectory samples.

    elbow : str
        Inverse kinematics configuration: "down" or "up".

    Returns
    -------
    time : ndarray
        Time samples in seconds.

    cartesian_positions : ndarray, shape (num_points, 2)
        End-effector coordinates [x, y] in metres.

    joint_positions : ndarray, shape (num_points, 2)
        Joint angles [theta1, theta2] in radians.
    """

    start_point = np.asarray(start_point, dtype=float)
    end_point = np.asarray(end_point, dtype=float)

    # Input validation
    if start_point.shape != (2,) or end_point.shape != (2,):
        raise ValueError("Start and end points must each contain two values.")

    if not np.all(np.isfinite(start_point)) or not np.all(np.isfinite(end_point)):
        raise ValueError("Cartesian coordinates must be finite numbers.")

    if not np.isfinite(duration) or duration <= 0:
        raise ValueError("Duration must be a positive finite number.")

    if (
        not isinstance(num_points, (int, np.integer))
        or isinstance(num_points, bool)
        or num_points < 2
    ):
        raise ValueError("num_points must be an integer greater than or equal to 2.")

    if elbow not in ("up", "down"):
        raise ValueError("Elbow must be either 'up' or 'down'.")

    # Generate time samples
    time = np.linspace(0, duration, num_points)

    # Normalized time
    tau = time / duration

    # Cubic time scaling
    s = 3 * tau**2 - 2 * tau**3

    # Generate straight-line Cartesian path
    cartesian_positions = (
        start_point
        + s[:, np.newaxis] * (end_point - start_point)
    )

    # Calculate joint angles using inverse kinematics
    joint_positions = np.zeros((num_points, 2))

    for i, (x, y) in enumerate(cartesian_positions):

        solution = inverse_kinematics(x, y, elbow=elbow)

        if solution is None:
            raise ValueError(
                f"Trajectory contains an unreachable point at sample {i}: "
                f"({x:.4f}, {y:.4f}) m"
            )

        joint_positions[i] = solution

    return time, cartesian_positions, joint_positions
