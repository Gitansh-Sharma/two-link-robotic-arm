"""Tests for the two-link arm's forward kinematics."""

import numpy as np

from robotic_arm.kinematics import (
    forward_kinematics,
    get_link_positions,
)


def test_both_links_horizontal():
    """Both links point horizontally to the right."""
    x, y = forward_kinematics(0.0, 0.0)

    assert np.isclose(x, 0.9)
    assert np.isclose(y, 0.0, atol=1e-10)


def test_both_links_vertical():
    """Both links point vertically upward."""
    x, y = forward_kinematics(np.pi / 2, 0.0)

    assert np.isclose(x, 0.0, atol=1e-10)
    assert np.isclose(y, 0.9)


def test_second_link_perpendicular():
    """The second link is perpendicular to the first."""
    x, y = forward_kinematics(0.0, np.pi / 2)

    assert np.isclose(x, 0.5)
    assert np.isclose(y, 0.4)


def test_link_positions():
    """The function returns the expected three points."""
    positions = get_link_positions(0.0, 0.0)

    expected = np.array([
        [0.0, 0.0],
        [0.5, 0.0],
        [0.9, 0.0],
    ])

    assert positions.shape == (3, 2)
    assert np.allclose(positions, expected)