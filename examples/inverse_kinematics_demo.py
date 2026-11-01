"""Demonstrate inverse kinematics."""

import numpy as np

from robotic_arm.inverse_kinematics import inverse_kinematics
from robotic_arm.kinematics import forward_kinematics


def main():

    target_x = 0.5
    target_y = 0.4

    for elbow in ["up", "down"]:

        solution = inverse_kinematics(
            target_x,
            target_y,
            elbow=elbow
        )

        print(f"\nElbow configuration: {elbow}")

        if solution is None:
            print("Target unreachable.")
            continue

        theta1, theta2 = solution

        print(f"Theta 1: {np.rad2deg(theta1):.2f} degrees")
        print(f"Theta 2: {np.rad2deg(theta2):.2f} degrees")

        # Verify using forward kinematics
        x, y = forward_kinematics(theta1, theta2)

        print(f"Calculated position: ({x:.4f}, {y:.4f})")
        print(f"Target position: ({target_x}, {target_y})")


if __name__ == "__main__":
    main()