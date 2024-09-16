import numpy as np
import matplotlib.pyplot as plt

def wall_effect(r_w, theta_w, gamma_w, e_w1, e_w2, l_w):
    O_w = e_w1 * np.cos(theta_w) + e_w2 * np.cos(2 * theta_w)
    f_w = np.exp(-(r_w / l_w) ** 2)
    delta_phi_w = gamma_w * np.sin(theta_w) * (1 + O_w) * f_w
    return delta_phi_w

# Parameters
gamma_w = 1.2
e_w1 = 1.25
e_w2 = 0.0
l_w = 2.5
r_w_fixed = 2.5  # Fixed distance to wall for the theta_w plot
theta_w_fixed = np.pi / 4  # Fixed angle for the r_w plot

# Generate data
theta_w_values = np.linspace(-np.pi, np.pi, 100)
r_w_values = np.linspace(0, 5, 100)

# Calculate delta_phi_w for each theta_w with fixed r_w
delta_phi_w_theta = wall_effect(r_w_fixed, theta_w_values, gamma_w, e_w1, e_w2, l_w)

# Calculate delta_phi_w for each r_w with fixed theta_w
delta_phi_w_r = wall_effect(r_w_values, theta_w_fixed, gamma_w, e_w1, e_w2, l_w)

# Plot delta_phi_w as a function of theta_w
plt.figure(figsize=(12, 5))

plt.subplot(1, 2, 1)
plt.plot(theta_w_values, delta_phi_w_theta)
plt.title(r'$\delta \phi_w$ as a function of $\theta_w$')
plt.xlabel(r'$\theta_w$ (radians)')
plt.ylabel(r'$\delta \phi_w$')
plt.grid(True)

# Plot delta_phi_w as a function of r_w
plt.subplot(1, 2, 2)
plt.plot(r_w_values, delta_phi_w_r)
plt.title(r'$\delta \phi_w$ as a function of $r_w$')
plt.xlabel(r'$r_w$ (distance to wall)')
plt.ylabel(r'$\delta \phi_w$')
plt.grid(True)

plt.tight_layout()
plt.show()
