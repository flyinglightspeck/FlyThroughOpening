import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize


def polynomial_trajectory_constraints(coeffs, T, a_max, z_max):
    t = np.linspace(0, T, 100)
    acc = np.polyval([20 * coeffs[5], 12 * coeffs[4], 6 * coeffs[3], 2 * coeffs[2]], t)
    pos_z = np.polyval(coeffs, t)
    return [a_max - np.max(np.abs(acc)), z_max - np.max(pos_z)]


def objective_function(coeffs, p0, v0, a0, pf, vf, af, T):
    t = np.linspace(0, T, 100)
    pos = np.polyval(coeffs, t)
    vel = np.polyval(np.polyder(coeffs), t)
    acc = np.polyval(np.polyder(np.polyder(coeffs)), t)

    error = (
            (pos[0] - p0) ** 2 +
            (vel[0] - v0) ** 2 +
            (acc[0] - a0) ** 2 +
            (pos[-1] - pf) ** 2 +
            (vel[-1] - vf) ** 2 +
            (acc[-1] - af) ** 2
    )

    return error


def initial_guess(p0, v0, a0, pf, vf, af, T):
    A = np.array([
        [1, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 2, 0, 0, 0],
        [1, T, T ** 2, T ** 3, T ** 4, T ** 5],
        [0, 1, 2 * T, 3 * T ** 2, 4 * T ** 3, 5 * T ** 4],
        [0, 0, 2, 6 * T, 12 * T ** 2, 20 * T ** 3]
    ])
    b = np.array([p0, v0, a0, pf, vf, af])
    coeffs = np.linalg.solve(A, b)
    return coeffs


def solve_constrained_polynomial(p0, v0, a0, pf, vf, af, T, a_max, z_max):
    coeffs_init = initial_guess(p0, v0, a0, pf, vf, af, T)
    bounds = [(None, None)] * 6
    constraints = {
        'type': 'ineq',
        'fun': polynomial_trajectory_constraints,
        'args': (T, a_max, z_max)
    }

    result = minimize(
        objective_function, coeffs_init,
        args=(p0, v0, a0, pf, vf, af, T),
        bounds=bounds, constraints=constraints
    )

    if result.success:
        return result.x
    else:
        raise ValueError("Optimization failed")


def polynomial_trajectory(coeffs, T, N):
    t = np.linspace(0, T, N + 1)
    positions = np.polyval(coeffs, t)
    velocities = np.polyval(np.polyder(coeffs), t)
    accelerations = np.polyval(np.polyder(np.polyder(coeffs)), t)

    return positions, velocities, accelerations


# Example usage
# Initial and final states for each axis
p0_x, v0_x, a0_x = 0, 0, 0
pf_x, vf_x, af_x = 10, 0, 0

p0_y, v0_y, a0_y = 0, 0, 0
pf_y, vf_y, af_y = 10, 0, 0

p0_z, v0_z, a0_z = 0, 0, 0
pf_z, vf_z, af_z = 10, 0, 0

T = 10  # Total time
N = 100  # Number of time steps
a_max = 2  # Maximum acceleration
z_max = 5  # Maximum altitude

# Solve for polynomial coefficients for each axis
coeffs_x = solve_constrained_polynomial(p0_x, v0_x, a0_x, pf_x, vf_x, af_x, T, a_max, np.inf)
coeffs_y = solve_constrained_polynomial(p0_y, v0_y, a0_y, pf_y, vf_y, af_y, T, a_max, np.inf)
coeffs_z = solve_constrained_polynomial(p0_z, v0_z, a0_z, pf_z, vf_z, af_z, T, a_max, z_max)

# Generate trajectories for each axis
positions_x, velocities_x, accelerations_x = polynomial_trajectory(coeffs_x, T, N)
positions_y, velocities_y, accelerations_y = polynomial_trajectory(coeffs_y, T, N)
positions_z, velocities_z, accelerations_z = polynomial_trajectory(coeffs_z, T, N)

# Combine trajectories for 3D plotting
positions_3d = np.vstack((positions_x, positions_y, positions_z)).T

# Plot the 3D trajectory
fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection='3d')
ax.plot(positions_3d[:, 0], positions_3d[:, 1], positions_3d[:, 2], label='Trajectory')
ax.set_xlabel('X Position')
ax.set_ylabel('Y Position')
ax.set_zlabel('Z Position')
ax.set_title('3D Trajectory')
ax.legend()
plt.show()

# Plot the position, velocity, and acceleration for each axis
t = np.linspace(0, T, N + 1)

plt.figure(figsize=(12, 12))

plt.subplot(3, 3, 1)
plt.plot(t, positions_x, label='Position X')
plt.title('Position X over Time')
plt.ylabel('Position X')
plt.grid(True)

plt.subplot(3, 3, 2)
plt.plot(t, velocities_x, label='Velocity X', color='orange')
plt.title('Velocity X over Time')
plt.ylabel('Velocity X')
plt.grid(True)

plt.subplot(3, 3, 3)
plt.plot(t, accelerations_x, label='Acceleration X', color='red')
plt.title('Acceleration X over Time')
plt.ylabel('Acceleration X')
plt.grid(True)

plt.subplot(3, 3, 4)
plt.plot(t, positions_y, label='Position Y')
plt.title('Position Y over Time')
plt.ylabel('Position Y')
plt.grid(True)

plt.subplot(3, 3, 5)
plt.plot(t, velocities_y, label='Velocity Y', color='orange')
plt.title('Velocity Y over Time')
plt.ylabel('Velocity Y')
plt.grid(True)

plt.subplot(3, 3, 6)
plt.plot(t, accelerations_y, label='Acceleration Y', color='red')
plt.title('Acceleration Y over Time')
plt.ylabel('Acceleration Y')
plt.grid(True)

plt.subplot(3, 3, 7)
plt.plot(t, positions_z, label='Position Z')
plt.title('Position Z over Time')
plt.ylabel('Position Z')
plt.grid(True)

plt.subplot(3, 3, 8)
plt.plot(t, velocities_z, label='Velocity Z', color='orange')
plt.title('Velocity Z over Time')
plt.ylabel('Velocity Z')
plt.grid(True)

plt.subplot(3, 3, 9)
plt.plot(t, accelerations_z, label='Acceleration Z', color='red')
plt.title('Acceleration Z over Time')
plt.ylabel('Acceleration Z')
plt.grid(True)

plt.tight_layout()
plt.show()
