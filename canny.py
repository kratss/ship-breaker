#!/usr/bin/env python
import gen
import extract_plane as ep
import matplotlib.pyplot as plt
import numpy as np
import cv2

# import torch.nn.functional as F
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

# Non-maximum suppression
theta = np.arctan2(g_y, g_x)  # gradient direction
theta = np.round(theta / (np.pi / 4)) * (np.pi / 4)  # quantize
theta = np.abs(theta)  # simplifies comparison logic

rows, cols = g.shape
g_suppressed = np.copy(g)
nms_mask = np.zeros_like(g)
print("Theta: \n:", theta)
print("Shape of g: ", g.shape)
print("gradient direction theta: \n", theta)

"""
for i in range(1, g.shape[0] - 1):
    for j in range(1, g.shape[1] - 1):
        if theta[i, j] == 0 or theta[i, j] == np.pi:
            neighbors = g[i, j - 1], g[i, j + 1]
        elif theta[i, j] == np.pi / 4:
            neighbors = g[i - 1, j - 1], g[i + 1, j + 1]
        elif theta[i, j] <= np.pi / 2:
            neighbors = g[i - 1, j], g[i + 1, j]
        elif theta[i, j] <= 3 * np.pi / 4:
            neighbors = g[i - 1, j - 1], g[i, j + 1]
        if g[i, j] >= max(neighbors):
            nms_mask[i, j] = 1
g_suppressed = g * nms_mask"""
image_uint8 = (image * 255).astype(np.uint8)
edges = cv2.Canny(image_uint8, 50, 150)
# plt.pcolor(g.T, cmap="binary", edgecolors="b")
# plt.show()
print("G:\n", g)
print("edges:\n", edges)
plt.pcolor(image.T, edgecolors="b")
plt.show()
