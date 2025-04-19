#!/usr/bin/env python
import gen
import extract_plane as ep
import matplotlib.pyplot as plt
import numpy as np
from scipy import ndimage

"""
        gen.curved_wall(
            origin=[110, 5, 0],
            length=110,
            height=30,
            yaw=np.pi / 2,
        ),
        """

density = 3
ship = np.concatenate(
    (
        gen.plane(origin=[0, 10, 30], length=110, width=90, density=density),
        gen.tbeam(origin=[0, 30, 30], length=90, density=density),
        gen.tbeam(origin=[0, 90, 30], length=90, density=density),
        gen.bulb_flat(origin=[0, 10, 30], roll=np.pi / 2, yaw=np.pi / 2),
        gen.plane(origin=[0, 0, 0], length=110, width=90, density=density),
        gen.ibeam(origin=[20, 20, 5], roll=np.pi / 2),
    )
)
slice = ep.extract_plane(ship)
grid = ep.voxelize(slice)

image = grid

# Apply convolutions
kernel_x = np.array([[-1, 0, 1], [-2, 0, 2], [-1, 0, 1]])
kernel_y = np.array([[-1, -2, -1], [0, 0, 0], [1, 2, 1]])

g_x = ndimage.convolve(image, kernel_x, mode="constant")
g_y = ndimage.convolve(image, kernel_y, mode="constant")
g = np.sqrt(g_x**2 + g_y**2)  # Gradient magnitude
print("Shape of g: ", g.shape)

# Non-maximum suppression
theta = np.arctan2(g_y, g_x)  # gradient direction

print("gradient direction theta: \n", theta)
rows, cols = g.shape
g_suppressed = np.copy(g)

for i in range(1, rows - 1):
    for j in range(1, cols - 1):
        # Get the current pixel's gradient magnitude and direction
        curr_mag = g[i, j]
        curr_dir = theta[i, j]
        if (0 <= curr_dir < np.pi / 8) or (15 * np.pi / 8 <= curr_dir <= 2 * np.pi):
            neighbors = [g[i, j - 1], g[i, j + 1]]
        elif (np.pi / 8 <= curr_dir < 3 * np.pi / 8) or (
            9 * np.pi / 8 <= curr_dir < 11 * np.pi / 8
        ):
            neighbors = [g[i - 1, j - 1], g[i + 1, j + 1]]
        elif (3 * np.pi / 8 <= curr_dir < 5 * np.pi / 8) or (
            11 * np.pi / 8 <= curr_dir < 13 * np.pi / 8
        ):
            neighbors = [g[i - 1, j], g[i + 1, j]]
        elif (5 * np.pi / 8 <= curr_dir < 7 * np.pi / 8) or (
            13 * np.pi / 8 <= curr_dir < 15 * np.pi / 8
        ):
            neighbors = [g[i - 1, j + 1], g[i + 1, j - 1]]
        # Check if the current pixel has the maximum gradient magnitude
        if curr_mag < np.max(neighbors):
            print(
                "At coordinates (",
                i,
                j,
                ")curr_mag is less than the maximum ggradient of its neighbors",
            )
            g_suppressed[i, j] = 50
        else:
            print("NOT")


# plt.pcolor(g.T, cmap="binary", edgecolors="b")
# plt.show()
print("G:\n", g)
print("g_suppressed:\n", g_suppressed)
plt.pcolor(g_suppressed.T, edgecolors="b")
plt.show()
