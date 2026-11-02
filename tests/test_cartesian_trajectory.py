
import numpy as np
import pytest

from robotic_arm.trajectory import cartesian_space_trajectory
from robotic_arm.kinematics import forward_kinematics


def test_cartesian_start_and_end_positions():

    start = [0.5, 0.2]
    end = [0.2, 0.6]

    time, cartesian, joints = cartesian_space_trajectory(
        start, end, 3.0
    )

    assert np.allclose(cartesian[0], start)
    assert np.allclose(cartesian[-1], end)


def test_cartesian_path_is_straight():

    start = [0.5, 0.2]
    end = [0.2, 0.6]

    time, cartesian, joints = cartesian_space_trajectory(
        start, end, 3.0
    )

    direction = np.array(end) - np.array(start)

    # 2D cross product: zero means collinear
    cross_products = (
        direction[0] * (cartesian[:, 1] - start[1])
        - direction[1] * (cartesian[:, 0] - start[0])
    )

    assert np.allclose(cross_products, 0, atol=1e-10)


def test_forward_kinematics_matches_cartesian_path():

    time, cartesian, joints = cartesian_space_trajectory(
        [0.5, 0.2],
        [0.2, 0.6],
        3.0
    )

    calculated_positions = np.array([
        forward_kinematics(theta1, theta2)
        for theta1, theta2 in joints
    ])

    assert np.allclose(
        calculated_positions,
        cartesian,
        atol=1e-8
    )


def test_output_shapes():

    time, cartesian, joints = cartesian_space_trajectory(
        [0.5, 0.2],
        [0.2, 0.6],
        3.0,
        num_points=200
    )

    assert time.shape == (200,)
    assert cartesian.shape == (200, 2)
    assert joints.shape == (200, 2)


def test_unreachable_path_raises_error():

    with pytest.raises(ValueError, match="unreachable"):
        cartesian_space_trajectory(
            [0.5, 0.2],
            [1.0, 0.0],
            3.0
        )


def test_invalid_elbow_configuration():

    with pytest.raises(ValueError):
        cartesian_space_trajectory(
            [0.5, 0.2],
            [0.2, 0.6],
            3.0,
            elbow="invalid"
        )
