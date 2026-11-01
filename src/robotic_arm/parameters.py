"""Physical parameters for the two-link robotic arm."""

# Link lengths in metres
L1 = 0.5
L2 = 0.4

# Link masses in kilograms
M1 = 1.0
M2 = 0.8

# Gravitational acceleration in m/s^2
G = 9.81

# Moments of inertia about each link's centre of mass.
I1 = M1 * L1**2 / 12
I2 = M2 * L2**2 / 12