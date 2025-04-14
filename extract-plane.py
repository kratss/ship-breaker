#!/usr/bin/env python
# %% imports
import open3d as o3d
import open3d.t as o3dt
import numpy as np
import open3d.core as o3c
import gen
import matplotlib.pyplot as plt

minmax = np.zeros([3, 2])
# %% setup
density = 5
ship = np.concatenate(
    (
        gen.plane(origin=[5, 5, 5], length=10, width=2, density=density),
        gen.plane(
            origin=[5, 5, 5], length=10, width=2, roll=np.pi / 2, density=density
        ),
        gen.plane(
            origin=[5, 5, 5], length=10, width=2, pitch=np.pi / 2, density=density
        ),
        gen.plane(origin=[0, 10, 30], length=110, width=90, density=density),
        gen.tbeam(origin=[0, 30, 30], length=90, density=density),
        gen.tbeam(origin=[0, 60, 30], length=90, density=density),
        gen.tbeam(origin=[0, 90, 30], length=90, density=density),
        gen.curved_wall(
            origin=[110, 5, 0],
            length=110,
            height=30,
            yaw=np.pi / 2,
        ),
        gen.bulb_flat(origin=[0, 10, 30], roll=np.pi / 2, yaw=np.pi / 2),
        gen.plane(origin=[0, 0, 0], length=110, width=90, density=density),
        gen.ibeam(origin=[20, 20, 5], roll=np.pi / 2),
    )
)
print("shape of pcd: \n", ship.shape)
plane = 3  # plane perpendiculat to the y axis at 3 along it
dist = 1.4
slice = ship[np.abs(ship[:, 0] - plane) < dist]
slice[:, 0] = 0
# projecter = np.array([[0, 1, 0], [0, 0, 1]])
# slice = np.dot(ship_plane_intersection, projecter.T).T

# %% voxelize
print("slice.max is ", slice[:, 0].max())
print("slice.max is ", slice[:, 1].max())
print("slice.max is ", slice[:, 2].max())


min_y = slice[:, 1].min()
min_z = slice[:, 2].min()
max_y = slice[:, 1].max()
max_z = slice[:, 2].max()

res = 5
grid_y = int((max_y - min_y) * res)
grid_z = int((max_z - min_z) * res)
grid = np.zeros([grid_y, grid_z])
grid_idx_y = np.linspace(min_y, max_y, int((max_y - min_y) * res))
grid_idx_z = np.linspace(min_z, max_z, int((max_z - min_z) * res))
for point in slice:
    # put point in closest grid point
    # when a grid point has a pcd point, we set that grid pixel to 1
    idx_y = np.argmin(np.abs(point[1] - grid_idx_y))
    idx_z = np.argmin(np.abs(point[2] - grid_idx_z))
    grid[idx_y, idx_z] = 1
    print("changing grid at", idx_z, ", ", idx_y, " to 1")
print("grid:\n", grid)
print("shape of grid:\n", np.shape(grid))
print("slice[2]: ", slice[2])
plt.pcolor(grid, cmap="binary", edgecolors="r")
plt.show()
"""
for point in slice:
    idx_x = np.argmin(np.abs(grid_x - point[0]))
    idx_y = np.argmin(np.abs(grid_y - point[1]))
    grid[idx_x, idx_y] = 255
    cv2.circle(grid, (x, y), 1, 255, -1)
"""

slice_pcd = o3d.geometry.PointCloud()
slice_pcd.points = o3d.utility.Vector3dVector(slice)
slice_voxel = o3d.geometry.VoxelGrid.create_from_point_cloud(slice_pcd, 0.3)

# df  = x +
print("shape of pcd: \n", slice.shape)
pcd = o3d.t.geometry.PointCloud(slice)
pcd2 = o3d.t.geometry.PointCloud(ship)
o3d.visualization.draw_geometries([slice_voxel])
