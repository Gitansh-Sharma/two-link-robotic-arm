"""Draw the robotic arm at a specified configuration."""

import numpy as np

from robotic_arm.visualization import plot_arm


def main():

    theta1 = np.deg2rad(30)
    theta2 = np.deg2rad(45)

    plot_arm(theta1, theta2)


if __name__ == "__main__":
    main()