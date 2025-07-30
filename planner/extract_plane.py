#!/usr/bin/env python
"""
Extract a 2D slice from a point cloud

For a given point cloud and cutting plane, a cross section can be extracted within a specified tolerance. The cross section is then converted to a binary image to facilitate analysis.
"""
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
import open3d.t as o3dt
import open3d.core as o3c
from icecream import ic


def extract_plane(
    scene,
    z_plane,
    tolerance,
):
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

    print("Shape of scene point cloud: \n", scene.shape)
    slice = scene[np.abs(scene[:, 2] - z_plane) < tolerance]
    slice[:, 2] = 0
    print(f"Z range: {scene[:, 2].min()} to {scene[:, 2].max()}")
    return slice


def voxelize(slice, density, grid_dim):
    """
    Voxelize a two dimensional point cloud projection to create a binary image

    Note: Input assumed to be projected onto an XY plane

    Args:
        slice (numpy array): 2D point cloud projection
        density (int): Grid cells per unit distance in the cloud

    Returns:
        grid (numpy array): 2D binary image (Y,X ordering)
    """
    if len(slice) == 0:
        ic("Voxelize: Warning, empty slice. Returning minimal grid")
        return np.zeros((1, 1))  # Return minimal grid
    min_x = grid_dim[0, 0]
    min_y = grid_dim[0, 1]
    max_x = grid_dim[1, 0]
    max_y = grid_dim[1, 1]

    grid_x = max(2, int((max_x - min_x) * density))
    grid_y = max(2, int((max_y - min_y) * density))

    x_points = max(2, int((max_x - min_x) * density))
    y_points = max(2, int((max_y - min_y) * density))

    grid = np.zeros([grid_y, grid_x])  # Note that the y value goes FIRST
    grid_idx_x = np.linspace(min_x, max_x, int((max_x - min_x) * density))

    # Edge case: when grid length is zero
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


def cloud_to_grid(cloud, density, z_plane, tolerance, grid_dim):
    """
    Returns the binary image of the cross section for a given cloud

    This wrapper function simplifies the point cloud extraction and image conversion process

    Args:
        cloud (numpy.ndarray): Array of x,y,z points describing point cloud of the scene
        density (float): Number of pixels in image space per unit length in cloud space
        z_plane (float): Location of the cutting plane along the z-axis
        tolerance (float): Points that lie within this perpendicular distance from the cutting plane will be included in the cross-section

    Returns:
        numpy.ndarray: A (2,n) array giving a binary image of the cross-section
    """
    slice = extract_plane(cloud, z_plane, tolerance)
    grid = voxelize(slice, density, grid_dim)
    return grid


def get_3d(coords2d, density, z_plane):
    """
    Recover 3D cloud scale and dimensionality from 2D image space

    Args:
        coords2d (numpy.ndarray): Ordered list of coordinates in image space with dimensions (n,2)
        density (float): Number of pixels in image space per unit length in cloud space
        z_plane (float): Location of the cutting plane along the z-axis

    Returns:
        np.ndarray (float): Ordered list of coordinates in cloud space with dimensions (n,3)
    """
    coords3d = np.zeros([coords2d.shape[0], 3])
    coords3d[:, 0:2] = coords2d
    coords3d = coords3d / density
    coords3d[:, 2] = z_plane
    return coords3d


if __name__ == "__main__":
    """
    Demo plane extraction
    """

    import primitives.gen
    import primitives.model

    DENSITY = 33
    NOISE_STD = 0.05
    Z_PLANE = 3
    TOLERANCE = 0.1
    DENSITY_GRID = 5

    cloud = model.gen_tbeams_many(density=DENSITY, noise_std=NOISE_STD)
    cloud_pcd = o3d.t.geometry.PointCloud(cloud)
    slice = extract_plane(cloud, Z_PLANE, TOLERANCE)
    slice_pcd = o3d.t.geometry.PointCloud(slice)
    slice = slice[::-1]  # Ensure matplotlib is right side up
    axes = gen.draw_axes()
    o3d.visualization.draw_geometries([slice_pcd.to_legacy()])
    grid = voxelize(slice, density=DENSITY_GRID)
    plt.imshow(grid, cmap="binary")
    plt.show()
