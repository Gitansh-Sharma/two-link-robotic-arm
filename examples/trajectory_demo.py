
import numpy as np
import matplotlib.pyplot as plt

from robotic_arm.trajectory import joint_space_trajectory


# Initial and final joint configurations
initial_angles = np.deg2rad([0, 0])
final_angles = np.deg2rad([60, 45])

# Movement duration
duration = 3.0

# Generate trajectory
time, positions, velocities, accelerations = joint_space_trajectory(
    initial_angles,
    final_angles,
    duration
)

# Plot position, velocity and acceleration
fig, axes = plt.subplots(3, 1, figsize=(9, 10), sharex=True)

joint_names = ["Joint 1", "Joint 2"]

for i in range(2):
    axes[0].plot(time, np.rad2deg(positions[:, i]), label=joint_names[i])
    axes[1].plot(time, velocities[:, i], label=joint_names[i])
    axes[2].plot(time, accelerations[:, i], label=joint_names[i])

axes[0].set_ylabel("Position (degrees)")
axes[1].set_ylabel("Velocity (rad/s)")
axes[2].set_ylabel("Acceleration (rad/s²)")
axes[2].set_xlabel("Time (seconds)")

for ax in axes:
    ax.grid(True)
    ax.legend()

fig.suptitle("Cubic Joint-Space Trajectory")
plt.tight_layout()
plt.show()
