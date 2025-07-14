
#!/usr/bin/env python3

import numpy as np
from scipy.interpolate import splprep, splev
import matplotlib.pyplot as plt
import pandas as pd

# Load corner points
points = np.loadtxt("/home/can-02/Desktop/f1tenth_ws/src/adp_controller/waypoints/wp_corner.csv", delimiter=",", dtype=float)
x = points[:, 0]
y = points[:, 1]

# Generate interpolated points between waypoints
def generate_points(start, end, dist):
    num_points = int(np.linalg.norm(end - start) / dist)
    return np.array([
        np.linspace(start[0], end[0], num_points),
        np.linspace(start[1], end[1], num_points)
    ]).T

dist = 0.5  # gap between points in meters
data = []
for i in range(len(points) - 1):
    data += generate_points(points[i], points[i + 1], dist).tolist()[1:]
data = np.array(data)

# Fit a spline through the interpolated points
tck, u = splprep([data[:, 0], data[:, 1]], s=0)
u_fine = np.linspace(0, 1, len(data)*5)
x_spline, y_spline = splev(u_fine, tck)

# Derivatives for heading and curvature
dx, dy = splev(u_fine, tck, der=1)
ddx, ddy = splev(u_fine, tck, der=2)

# Heading (yaw angle)
heading = np.arctan2(dy, dx)

# Curvature: κ = (x' * y'' - y' * x'') / (x'^2 + y'^2)^(3/2)
curvature = (dx * ddy - dy * ddx) / np.power(dx**2 + dy**2, 1.5)

# Stack the raceline data
raceline = np.vstack([x_spline, y_spline, heading, curvature]).T

# Save to CSV
output_file = "/home/can-02/Desktop/f1tenth_ws/src/adp_controller/waypoints/optimal_raceline.csv"
np.savetxt(output_file, raceline, delimiter=",", header="x,y,heading,curvature", comments='')

# Visualize
plt.figure(figsize=(8, 8))
plt.plot(x_spline, y_spline, 'b-', label='Raceline')
plt.plot(x, y, 'ro', label='Corners')
plt.axis('equal')
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.title("Generated Optimal Raceline")
plt.legend()
plt.grid(True)
plt.show()




