#!/usr/bin/env python3
import numpy as np
from scipy import linalg
import os

# Vehicle parameters
m = 3.47      # kg
Iz = 0.04712  # kg·m²
lf = 0.125    # m
lr = 0.125    # m
Cf = 4.718e1  # N/rad (linearized)
Cr = 5.456e1  # N/rad

# State: [e_y, ė_y, e_ψ, ė_ψ]
# Input: [δ_f] (steering angle)

# Grid setup
vx_grid = np.linspace(1.0, 3.5, 5)           # m/s
kappa_grid = np.linspace(-1.5, 1.5, 5)       # 1/m

K_table = np.zeros((len(vx_grid), len(kappa_grid), 1, 4))  # u = Kx

# ADP approximation using LQR
Q = np.diag([5.0, 0.5, 5.0, 0.5])
R = np.array([[1.0]])

for i, Vx in enumerate(vx_grid):
    for j, kappa in enumerate(kappa_grid):
        try:
            a11 = -2 * (Cf + Cr) / (m * Vx)
            a12 = -Vx - 2 * (Cf * lf - Cr * lr) / (m * Vx)
            a21 = -2 * (Cf * lf - Cr * lr) / (Iz * Vx)
            a22 = -2 * (Cf * lf**2 + Cr * lr**2) / (Iz * Vx)

            A = np.array([
                [0,        1, 0,        0],
                [0,      a11, 0,      a12],
                [0,        0, 0,        1],
                [0,      a21, 0,      a22]
            ])

            B = np.array([
                [0],
                [2 * Cf / m],
                [0],
                [2 * Cf * lf / Iz]
            ])

            # Attempt to solve ARE
            P = linalg.solve_continuous_are(A, B, Q, R)
            K = np.dot(np.linalg.inv(R), np.dot(B.T, P))

            K_table[i, j, :, :] = K

        except np.linalg.LinAlgError:
            print(f"[WARNING] Riccati failed at Vx={Vx:.2f}, kappa={kappa:.2f}. Using zero gain.")

            K_table[i, j, :, :] = np.zeros((1, 4))


# Save results
save_dir = os.path.dirname(__file__)
np.save(os.path.join(save_dir, "adp_gain_table.npy"), K_table)
np.save(os.path.join(save_dir, "vx_grid.npy"), vx_grid)
np.save(os.path.join(save_dir, "kappa_grid.npy"), kappa_grid)

print("Gain table saved:", K_table.shape)

