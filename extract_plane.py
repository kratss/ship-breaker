#!/usr/bin/env python
"""
Extract a 2D slice from a point cloud
"""
import gen
import matplotlib.pyplot as plt
import model
import numpy as np
import open3d as o3d
import open3d.t as o3dt
import open3d.core as o3c
from icecream import ic


def extract_plane(scene, z_plane, tolerance):
    """
    Extract the portion of the scene point cloud that is near the
        cutting plane

    Args:
        scene: Array of x,y,z points describing point cloud of the
            scene
        z_plane: Location of cutting plane along z axis. The plane is
            perpendicular to the z axis
        tolerance: perpendicular distance between the plane and scene points.
            Scene points may not fall exactly on the plane, so this
            sensitivity must be tuned by the user

    Returns:
        numpy array: narrow cross section of the scene
    """

    print("shape of scene pcd: \n", scene.shape)
    slice = scene[np.abs(scene[:, 2] - z_plane) < tolerance]
    slice[:, 2] = 0
    print(f"Z range: {scene[:, 2].min()} to {scene[:, 2].max()}")
    ic(slice)
    return slice


def voxelize(slice, res):
    """
    Voxelize a two dimensional point cloud projection to create a binary image

    Note: Input assumed to be projected onto an XY plane

    Args:
        slice (numpy array): 2D point cloud projection
        res (int): Grid cells per unit distance in the cloud

    Returns:
        grid (numpy array): 2D binary image (Y,X ordering)
    """
    if len(slice) == 0:
        ic("Voxelize: Warning, empty slice. Returning minimal grid")
        return np.zeros((1, 1))  # Return minimal grid
    min_x = 0  # slice[:, 0].min()
    min_y = 0  # slice[:, 1].min()
    ic(slice)
    max_x = slice[:, 0].max()
    max_y = slice[:, 1].max()
    grid_x = max(2, int((max_x - min_x) * res))
    grid_y = max(2, int((max_y - min_y) * res))
    x_points = max(2, int((max_x - min_x) * res))
    y_points = max(2, int((max_y - min_y) * res))
    grid = np.zeros([grid_y, grid_x])  # Note that the y value goes FIRST
    ic(max_y)
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
        idx_x = np.argmin(np.abs(point[0] - grid_idx_x))
        idx_y = np.argmin(np.abs(point[1] - grid_idx_y))
        idx_y = grid_y - 1 - idx_y  # flip upside down because
        # point clouds have 0 at the bottom, images have it at thetop
        grid[idx_y, idx_x] = 1
    return grid


def cloud_to_grid(cloud, res, z_plane, tolerance):
    slice = extract_plane(cloud, z_plane, tolerance)
    grid = voxelize(slice, res)
    return grid


def get_3d(coords2d, res, z_plane):
    """
    Turn 2D image coordinates into 3D point cloud coordinates
    """
    coords3d = np.zeros([coords2d.shape[0], 3])
    coords3d[:, 0:2] = coords2d
    coords3d = coords3d / res
    coords3d[:, 2] = z_plane
    return coords3d


if __name__ == "__main__":
    density = 3
    noise_std = 0.1

    tbeams = model.gen_tbeams(noise_std=noise_std)
    planes = model.gen_planes(noise_std=noise_std)
    curved_walls = model.gen_curved_walls(origin=[0, 0, 20], density=5, noise_std=0)
    tbeams_pcd = o3d.t.geometry.PointCloud(tbeams)
    planes_pcd = o3d.t.geometry.PointCloud(planes)
    curved_walls_pcd = o3d.t.geometry.PointCloud(curved_walls)
    slice = extract_plane(curved_walls)
    # slice_pcd = o3d.geometry.PointCloud()
    # slice_pcd.points = o3d.utility.Vector3dVector(slice)
    slice_pcd = o3d.t.geometry.PointCloud(slice)
    axes = gen.draw_axes()
    o3d.visualization.draw_geometries(
        [axes, slice_pcd.to_legacy(), tbeams_pcd.to_legacy(), planes_pcd.to_legacy()]
    )
    grid = voxelize(slice, res=1)
    plt.pcolor(grid, cmap="binary", edgecolors="r")
    plt.show()
