#!/usr/bin/env python

# %% Cell 1
import open3d as o3d
import numpy as np
import open3d.core as o3c
import gen

# Load data
# use .t for tensor version. legacy verion breaks gen libray
environment_pcd = o3d.t.geometry.PointCloud(gen.disc_gen(resolution=20))
# to_legacy for compute_point_cloud_distance compatibiltiy
environment_pcd = environment_pcd.to_legacy()
plane = np.loadtxt("assets/current_asset.xyz")
plane_pcd = o3d.geometry.PointCloud()
plane_pcd.points = o3d.utility.Vector3dVector(plane)

# %% Cell 2
# Remove points not intersecting with plane
dists = environment_pcd.compute_point_cloud_distance(plane_pcd)
dists = np.asarray(dists)
ind = np.where(dists < 0.01)[0]
environment_plane_intersection = environment_pcd.select_by_index(ind)

# %% Cell 3
# Compute the normal to the plane
# TODO: take the cutting plane as input to define this
# point on the plane. With this plus the normal we have a plane definition
normal = np.array([0, 0, 1])
point_on_plane = np.array([0, 0, 0])

distances = np.dot(environment_plane_intersection.points - point_on_plane, normal)

# Translate the points along the normal vector of the plane
translated_points = environment_plane_intersection.points - distances[
    :, np.newaxis
] * normal / np.linalg.norm(normal)

# Create a new point cloud with the translated points
projected_pcd = o3d.geometry.PointCloud()
projected_pcd.points = o3d.utility.Vector3dVector(translated_points)


# %% Cell 3
o3d.visualization.draw_plotly([environment_pcd, plane_pcd])
o3d.visualization.draw_plotly([environment_plane_intersection, projected_pcd, line_set])
