#!/usr/bin/env python
import gen
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
import open3d.t as o3dt
import open3d.core as o3c
from icecream import ic


def extract_plane(scene, plane=3, dist=0.1):
    """
    Extract the portion of the scene point cloud that is near the
        cutting plane

    Args:
        scene: Array of x,y,z points describing point cloud of the
            scene
        plane: Location of cutting plane along z axis. The plane is
            perpendicular to the z axis
        dist: perpendicular distance between the plane and scene points.
            Scene points may not fall exactly on the plane, so this
            sensitivity must be tuned by the user

    Returns:
        numpy array: narrow cross section of the scene
    """

    print("shape of scene pcd: \n", scene.shape)
    slice = scene[np.abs(scene[:, 2] - plane) < dist]
    slice[:, 2] = 0
    return slice


def voxelize(slice, res=1):
    """
    Voxelize a two dimensional point cloud projection to create a binary image
    Assumes projected cloud lies on a yz plane

    Args:
        slice (numpy array): two dimensional point cloud projection
        res (int): number of squares per unit distance in the image

    Returns:
        grid (numpy array): two dimensional binary image
    """
    min_x = slice[:, 0].min()
    min_y = slice[:, 1].min()
    max_x = slice[:, 0].max()
    max_y = slice[:, 1].max()
    grid_x = max(0, int((max_x - min_x) * res))
    grid_y = max(1, int((max_y - min_y) * res))
    x_points = max(2, int((max_x - min_x) * res))
    y_points = max(2, int((max_y - min_y) * res))
    grid = np.zeros([grid_x, grid_y])
    grid_idx_x = np.linspace(min_x, max_x, int((max_x - min_x) * res))
    grid_idx_y = np.linspace(min_y, max_y, int((max_y - min_y) * res))

    # Edge case logic
    if min_x == max_x:
        grid_idx_x = np.array([min_x])
    else:
        grid_idx_x = np.linspace(min_x, max_x, x_points)

    if min_y == max_y:
        grid_idx_y = np.array([min_y])
    else:
        grid_idx_y = np.linspace(min_y, max_y, y_points)

    # Check if either dimension is empty
    if len(grid_idx_y) == 0 or len(grid_idx_y) == 0:
        return np.zeros([1, 1])  # Return minimal grid if dimensions are empty

    for point in slice:
        idx_x = np.argmin(np.abs(point[1] - grid_idx_x))
        idx_y = np.argmin(np.abs(point[2] - grid_idx_y))
        grid[idx_x, idx_y] = 1
    return grid


def cloud_to_grid(cloud):
    cloud = gen.noise(cloud)
    slice = extract_plane(cloud)
    grid = voxelize(slice)
    return grid


if __name__ == "__main__":
    density = 3
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
    slice = extract_plane(ship)
    print("shape of pcd: \n", slice.shape)
    slice_pcd = o3d.geometry.PointCloud()
    slice_pcd.points = o3d.utility.Vector3dVector(slice)
    o3d.visualization.draw_geometries([slice_pcd])
    grid = voxelize(slice)
    plt.pcolor(grid.T, cmap="binary", edgecolors="r")
    plt.show()

    # Delete later
    """
    slice_voxel = o3d.geometry.VoxelGrid.create_from_point_cloud(slice_pcd, 0.3)
    pcd = o3d.t.geometry.PointCloud(slice)
    pcd2 = o3d.t.geometry.PointCloud(ship)
    o3d.visualization.draw_geometries([slice_voxel])
    """
