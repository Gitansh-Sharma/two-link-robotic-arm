"""Visualize the reachable workspace."""

from robotic_arm.visualization import plot_workspace


def main():
    plot_workspace(resolution=150)


if __name__ == "__main__":
    main()