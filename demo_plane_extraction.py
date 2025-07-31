#!/usr/bin/env python
"""
Demo plane extraction
"""

import planner as pln
import open3d as o3d
import open3d.t as o3dt
import open3d.core as o3c
import numpy as np
import matplotlib.pyplot as plt

DENSITY = 13
NOISE_STD = 0.09
Z_PLANE = 3
TOLERANCE = 0.1
DENSITY_GRID = 5

cloud = pln.gen_ship(DENSITY, NOISE_STD)
slice = pln.extract_plane(cloud, Z_PLANE, TOLERANCE)
cloud_pcd = o3d.t.geometry.PointCloud(cloud)
slice_pcd = o3d.t.geometry.PointCloud(slice)
slice = slice[::-1]  # Ensure matplotlib is right side up
grid_dim = np.array(
    [
        [slice[:, 0].min(), slice[:, 1].min()],
        [slice[:, 0].max(), slice[:, 1].max()],
    ]
)

o3d.visualization.draw_geometries([cloud_pcd.to_legacy()])
o3d.visualization.draw_geometries([slice_pcd.to_legacy()])

grid = pln.voxelize(slice, DENSITY_GRID, grid_dim)
plt.imshow(grid, cmap="binary")
plt.show()
