#!/usr/bin/env python
# %% imports
import open3d as o3d
import open3d.t as o3dt
import numpy as np
import open3d.core as o3c
import gen
import matplotlib.pyplot as plt


# %% setup
def extract_plane(scene, plane=3, dist=0.1):
    """
    Extract the portion of the scene point cloud that is near the
        cutting plane

    Args:
        scene (numpy array): numpy array of describing a point cloud of the scene
        plane (double): location of cutting plane along y axis. The plane is perpendicular to the y axis
        dist (double): perpendicular distance between the plane and scene points.
        Scene points may not fall exactly on the plane, and this sensitivity
        must be tuned by the user

    Returns:
        numpy array: narrow cross section of the scene
    """

    print("shape of scene pcd: \n", scene.shape)
    slice = scene[np.abs(ship[:, 0] - plane) < dist]
    slice[:, 0] = 0
    return slice


def voxelize(slice, res=5):
    """
    Voxelize a two dimensional point cloud projection to create a binary image
    Assumes projected cloud lies on a yz plane

    Args:
        slice (numpy array): two dimensional point cloud projection
        res (int): number of squares per unit distance in the image

    Returns:
        grid (numpy array): two dimensional binary image
    """

    min_y = slice[:, 1].min()
    min_z = slice[:, 2].min()
    max_y = slice[:, 1].max()
    max_z = slice[:, 2].max()
    grid_y = int((max_y - min_y) * res)
    grid_z = int((max_z - min_z) * res)
    grid = np.zeros([grid_y, grid_z])
    grid_idx_y = np.linspace(min_y, max_y, int((max_y - min_y) * res))
    grid_idx_z = np.linspace(min_z, max_z, int((max_z - min_z) * res))
    for point in slice:
        idx_y = np.argmin(np.abs(point[1] - grid_idx_y))
        idx_z = np.argmin(np.abs(point[2] - grid_idx_z))
        grid[idx_y, idx_z] = 1
        print("changing grid at", idx_z, ", ", idx_y, " to 1")
    return grid


# %% main
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
    grid = voxelize(slice)
    plt.pcolor(grid.T, cmap="binary", edgecolors="r")
    plt.show()

    slice_pcd = o3d.geometry.PointCloud()
    slice_pcd.points = o3d.utility.Vector3dVector(slice)
    slice_voxel = o3d.geometry.VoxelGrid.create_from_point_cloud(slice_pcd, 0.3)

    print("shape of pcd: \n", slice.shape)
    pcd = o3d.t.geometry.PointCloud(slice)
    pcd2 = o3d.t.geometry.PointCloud(ship)
    o3d.visualization.draw_geometries([slice_voxel])
