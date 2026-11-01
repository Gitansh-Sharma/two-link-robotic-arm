"""Visualization tools for the two-link robotic arm."""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from robotic_arm.parameters import L1, L2
from robotic_arm.kinematics import get_link_positions


def plot_arm(theta1, theta2, show=True):
    """
    Draw the robotic arm for a given joint configuration.

    Angles must be provided in radians.
    """

    positions = get_link_positions(theta1, theta2)

    x = positions[:, 0]
    y = positions[:, 1]

    fig, ax = plt.subplots(figsize=(8, 7))

    ax.plot(x, y, 'o-', linewidth=4, markersize=9)

    ax.scatter(0, 0, color='black', s=80, label='Base')
    ax.scatter(x[1], y[1], color='blue', s=80, label='Joint 1')
    ax.scatter(x[2], y[2], color='red', s=80, label='End Effector')

    ax.set_title("Two-Link Planar Robotic Arm")
    ax.set_xlabel("X Position (m)")
    ax.set_ylabel("Y Position (m)")

    ax.set_xlim(-1.0, 1.0)
    ax.set_ylim(-1.0, 1.0)

    ax.set_aspect('equal')
    ax.grid(True)
    ax.legend()

    ax.text(
        x[2] + 0.03,
        y[2] + 0.03,
        f"({x[2]:.3f}, {y[2]:.3f})"
    )

    if show:
        plt.show()

    return fig, ax

def plot_workspace(resolution=100):
    """
    Generate and plot the reachable workspace.

    resolution controls the number of sampled
    joint angles in each direction.
    """

    theta1_values = np.linspace(
        0, 2 * np.pi, resolution
    )

    theta2_values = np.linspace(
        -np.pi, np.pi, resolution
    )

    X = []
    Y = []

    for theta1 in theta1_values:
        for theta2 in theta2_values:

            x = (
                L1 * np.cos(theta1)
                + L2 * np.cos(theta1 + theta2)
            )

            y = (
                L1 * np.sin(theta1)
                + L2 * np.sin(theta1 + theta2)
            )

            X.append(x)
            Y.append(y)

    fig, ax = plt.subplots(figsize=(8, 8))

    ax.scatter(
        X,
        Y,
        s=2,
        alpha=0.3,
        label="Reachable positions"
    )

    # Outer and inner workspace boundaries
    outer = plt.Circle(
        (0, 0),
        L1 + L2,
        fill=False,
        linestyle='--',
        label='Maximum reach'
    )

    inner = plt.Circle(
        (0, 0),
        abs(L1 - L2),
        fill=False,
        linestyle='--',
        label='Minimum reach'
    )

    ax.add_patch(outer)
    ax.add_patch(inner)

    ax.scatter(0, 0, color='black', label='Base')

    ax.set_title("Reachable Workspace")
    ax.set_xlabel("X Position (m)")
    ax.set_ylabel("Y Position (m)")

    ax.set_xlim(-1.0, 1.0)
    ax.set_ylim(-1.0, 1.0)

    ax.set_aspect('equal')
    ax.grid(True)
    ax.legend()

    plt.show()

    return fig, 
    
def animate_arm(theta1_values, theta2_values, interval=30):
    """
    Animate the robotic arm over a sequence of joint angles.

    Parameters
    ----------
    theta1_values : array-like
        First joint angles in radians.

    theta2_values : array-like
        Second joint angles in radians.

    interval : int
        Delay between animation frames in milliseconds.
    """

    if len(theta1_values) != len(theta2_values):
        raise ValueError("Both angle arrays must have equal length.")

    if len(theta1_values) == 0:
        raise ValueError("Angle arrays cannot be empty.")

    fig, ax = plt.subplots(figsize=(8, 7))

    arm_line, = ax.plot(
        [],
        [],
        'o-',
        linewidth=4,
        markersize=9
    )

    end_effector, = ax.plot(
        [],
        [],
        'ro',
        markersize=8
    )

    position_text = ax.text(
        0.02, 0.95,
        "",
        transform=ax.transAxes
    )

    ax.set_xlim(-1.0, 1.0)
    ax.set_ylim(-1.0, 1.0)

    ax.set_aspect('equal')
    ax.grid(True)

    ax.set_title("Two-Link Robotic Arm Animation")
    ax.set_xlabel("X Position (m)")
    ax.set_ylabel("Y Position (m)")

    def update(frame):

        positions = get_link_positions(
            theta1_values[frame],
            theta2_values[frame]
        )

        x = positions[:, 0]
        y = positions[:, 1]

        arm_line.set_data(x, y)

        end_effector.set_data(
            [x[-1]],
            [y[-1]]
        )

        position_text.set_text(
            f"End Effector: ({x[-1]:.3f}, {y[-1]:.3f}) m"
        )

        return arm_line, end_effector, position_text

    animation = FuncAnimation(
        fig,
        update,
        frames=len(theta1_values),
        interval=interval,
        blit=True,
        repeat=True
    )

    plt.show()

    return animation