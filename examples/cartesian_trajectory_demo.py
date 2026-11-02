
import numpy as np
import matplotlib.pyplot as plt

from robotic_arm.trajectory import cartesian_space_trajectory
from robotic_arm.kinematics import forward_kinematics


# Cartesian starting and ending positions (metres)
start_point = [0.5, 0.2]
end_point = [0.2, 0.6]

duration = 3.0

# Generate Cartesian trajectory
time, cartesian_positions, joint_positions = cartesian_space_trajectory(
    start_point,
    end_point,
    duration,
    num_points=300,
    elbow="down"
)

# Verify the Cartesian trajectory using forward kinematics
fk_positions = np.array([
    forward_kinematics(theta1, theta2)
    for theta1, theta2 in joint_positions
])

max_error = np.max(
    np.linalg.norm(fk_positions - cartesian_positions, axis=1)
)

print(f"Maximum FK verification error: {max_error:.10f} m")

# Plot Cartesian path and joint angles
fig, axes = plt.subplots(1, 2, figsize=(13, 5))

# End-effector path
axes[0].plot(
    cartesian_positions[:, 0],
    cartesian_positions[:, 1],
    label="Planned path"
)

axes[0].scatter(
    [start_point[0], end_point[0]],
    [start_point[1], end_point[1]],
    marker="o",
    label="Start / End"
)

axes[0].set_xlabel("X position (m)")
axes[0].set_ylabel("Y position (m)")
axes[0].set_title("Cartesian trajectory")
axes[0].axis("equal")
axes[0].grid(True)
axes[0].legend()

# Joint angle trajectories
axes[1].plot(
    time,
    np.rad2deg(joint_positions[:, 0]),
    label="Joint 1"
)

axes[1].plot(
    time,
    np.rad2deg(joint_positions[:, 1]),
    label="Joint 2"
)

axes[1].set_xlabel("Time (s)")
axes[1].set_ylabel("Joint angle (degrees)")
axes[1].set_title("Joint angles during Cartesian motion")
axes[1].grid(True)
axes[1].legend()

plt.tight_layout()
plt.show()
