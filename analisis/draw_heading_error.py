import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D

mpl.use('macosx')
# Define the number of points in theta and phi directions
theta_points = 100
phi_points = 100

# Create arrays for theta and phi
theta = np.linspace(np.pi * 1.85 / 4, np.pi * 2.5 / 4, theta_points)
phi = np.linspace(np.pi * 1.85/ 4, np.pi * 2.5 / 4, phi_points)

# Create a meshgrid
theta, phi = np.meshgrid(theta, phi)

# Calculate x, y, and z coordinates of the sphere
r = 5  # Radius of the sphere
x = r * np.sin(phi) * np.cos(theta)
y = r * np.sin(phi) * np.sin(theta)
z = r * np.cos(phi)

# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the surface
ax.plot_surface(x, y, z, color='y', alpha=0.4)

# Define the number of points in theta and phi directions
theta_points = 100
phi_points = 100

# Create arrays for theta and phi
theta = np.linspace(0, 2 * np.pi, theta_points)
phi = np.linspace(0, 2 * np.pi, phi_points)

# Create a meshgrid
theta, phi = np.meshgrid(theta, phi)

# Calculate x, y, and z coordinates of the sphere
r = 1  # Radius of the sphere
x = r * np.sin(phi) * np.cos(theta)
y = r * np.sin(phi) * np.sin(theta) + 5
z = r * np.cos(phi)


ax.plot_surface(x, y, z, color='r', alpha=0.2)

# Set labels
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

ax.set_axis_off()
ax.set_aspect('equal', adjustable='box')
ax.view_init(elev=0, azim=90)
# Show the plot
plt.show()
