"""Animate a two-link robotic arm."""

import numpy as np

from robotic_arm.visualization import animate_arm


def main():

    # Generate 300 time samples
    time = np.linspace(0, 2 * np.pi, 300)

    # Joint 1 oscillates between -60 and +60 degrees
    theta1_values = np.deg2rad(
        60 * np.sin(time)
    )

    # Joint 2 oscillates between 45 and 135 degrees
    theta2_values = np.deg2rad(
        90 + 45 * np.sin(2 * time)
    )

    # Start animation
    animate_arm(
        theta1_values,
        theta2_values,
        interval=30
    )


if __name__ == "__main__":
    main()