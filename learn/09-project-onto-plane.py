#!/usr/bin/env python
import open3d as o3d
import numpy as np
import open3d.core as o3c

"""
# Load data
ply_point_cloud = o3d.data.PLYPointCloud()
pcd = o3d.io.read_point_cloud(ply_point_cloud.path)

plane = np.loadtxt("plane.xyz")
plane_pcd = o3d.geometry.PointCloud()
plane_pcd.points = o3d.utility.Vector3dVector(plane)

# Create line set object
points = np.array([[0, 0, 0], [1, 1, 1]])  # Define the points of the line
lines = np.array([[0, 1]])  # Define the lines (each line is defined by two points)
line_set = o3d.geometry.LineSet()
line_set.points = o3d.utility.Vector3dVector(points)
line_set.lines = o3d.utility.Vector2iVector(lines)
o3d.visualization.draw_plotly([pcd, line_set])

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

print("KDTree demo")
print("load a point cloud and paint it gray")

sample_pcd_data = o3d.data.PCDPointCloud()
pcd = o3d.io.read_point_cloud(sample_pcd_data.path)

normal = np.array([0, 0, 1])  # normal vector of the plane
point_on_plane = np.array([0, 0, 0])  # point on the plane


distances = np.dot(pcd.points - point_on_plane, normal)

# Translate the points along the normal vector of the plane
translated_points = pcd.points - distances[:, np.newaxis] * normal / np.linalg.norm(
    normal
)

# Create a new point cloud with the translated points
translated_pcd = o3d.geometry.PointCloud()
translated_pcd.points = o3d.utility.Vector3dVector(translated_points)


o3d.visualization.draw_geometries([pcd, translated_pcd])
