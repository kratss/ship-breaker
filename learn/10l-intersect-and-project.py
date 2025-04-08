#!/usr/bin/env python
import open3d as o3d
import numpy as np
import open3d.core as o3c

"""
"""

"""
# Load data
ply_point_cloud = o3d.data.PLYPointCloud()
pcd = o3d.io.read_point_cloud(ply_point_cloud.path)

plane = np.loadtxt("plane.xyz")
plane_pcd = o3d.geometry.PointCloud()
plane_pcd.points = o3d.utility.Vector3dVector(plane)

dists = pcd.compute_point_cloud_distance(plane_pcd)
dists = np.asarray(dists)
ind = np.where(dists < 0.01)[0]
pcd_without_chair = pcd.select_by_index(ind)
o3d.visualization.draw_geometries(
    [pcd_without_chair],
    zoom=0.3412,
    front=[0.4257, -0.2125, -0.8795],
    lookat=[2.6172, 2.0475, 1.532],
    up=[-0.0694, -0.9768, 0.2024],
)

print(dists)
print("\nind: \n", ind)
"""

# Load data
ply_point_cloud = o3d.data.PLYPointCloud()
pcd = o3d.io.read_point_cloud(ply_point_cloud.path)
plane = np.loadtxt("plane.xyz")
plane_pcd = o3d.geometry.PointCloud()
plane_pcd.points = o3d.utility.Vector3dVector(plane)

# Remove points not intersecting with plane
dists = pcd.compute_point_cloud_distance(plane_pcd)
dists = np.asarray(dists)
ind = np.where(dists < 0.01)[0]
pcd_plane_intersection = pcd.select_by_index(ind)

#
normal = np.array([0, 0, 1])  # normal vector of the plane
point_on_plane = np.array([0, 0, 0])  # point on the plane

distances = np.dot(pcd_plane_intersection.points - point_on_plane, normal)

# Translate the points along the normal vector of the plane
translated_points = pcd_plane_intersection.points - distances[
    :, np.newaxis
] * normal / np.linalg.norm(normal)

# Create a new point cloud with the translated points
translated_pcd = o3d.geometry.PointCloud()
translated_pcd.points = o3d.utility.Vector3dVector(translated_points)


o3d.visualization.draw_geometries([pcd_plane_intersection, translated_pcd])
distances = np.dot(pcd_plane_intersection.points - point_on_plane, normal)

# Translate the points along the normal vector of the plane
translated_points = pcd_plane_intersection.points - distances[
    :, np.newaxis
] * normal / np.linalg.norm(normal)

# Create a new point cloud with the translated points
translated_pcd = o3d.geometry.PointCloud()
translated_pcd.points = o3d.utility.Vector3dVector(translated_points)


o3d.visualization.draw_geometries([pcd, translated_pcd])
