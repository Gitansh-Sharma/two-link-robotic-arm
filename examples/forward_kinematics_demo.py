"""Demonstrate forward kinematics for the two-link robotic arm."""

import numpy as np

from robotic_arm.kinematics import (
    forward_kinematics,
    get_link_positions,
)


def main():
    # Specify joint angles in degrees for readability.
    theta1_deg = 30.0
    theta2_deg = 45.0

    # Convert degrees to radians for the mathematical functions.
    theta1 = np.deg2rad(theta1_deg)
    theta2 = np.deg2rad(theta2_deg)

    # Calculate the end-effector position.
    x, y = forward_kinematics(theta1, theta2)

    # Calculate the coordinates of all three points.
    positions = get_link_positions(theta1, theta2)

    print("Two-Link Robotic Arm: Forward Kinematics")
    print("-" * 42)
    print(f"Joint 1 angle: {theta1_deg:.2f} degrees")
    print(f"Joint 2 angle: {theta2_deg:.2f} degrees")
    print(f"Joint 1 position: {positions[1]}")
    print(f"End-effector position: ({x:.4f}, {y:.4f}) m")


if __name__ == "__main__":
    main()