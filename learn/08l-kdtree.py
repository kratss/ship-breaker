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
pcd.paint_uniform_color([0.5, 0.5, 0.5])
pcd_tree = o3d.geometry.KDTreeFlann(pcd)

print("Paint the 1501st point red.")
pcd.colors[1500] = [1, 0, 0]


print("Find its 200 nearest neighbors, and paint them blue.")
[k, idx, _] = pcd_tree.search_knn_vector_3d(pcd.points[1500], 200)
np.asarray(pcd.colors)[idx[1:], :] = [0, 0, 1]

print("Find its neighbors with distance less than 0.2, and paint them green.")
[k, idx, _] = pcd_tree.search_radius_vector_3d(pcd.points[1500], 0.2)
np.asarray(pcd.colors)[idx[1:], :] = [0, 1, 0]

# KNN func
distances = []
indices = []
for point in pcd.points:
    [dist, idx, _] = pcd_tree.search_knn_vector_3d(point, 10)
    distances.append(dist)
    indices.append(idx)

# Cluster points using DBSCAN
from sklearn.cluster import DBSCAN
import numpy as np

points = np.array(pcd.points)
db = DBSCAN(eps=0.1, min_samples=10).fit(points)
labels = db.labels_

# Color each cluster
colors = np.zeros((len(points), 3))
for i in range(len(points)):
    if labels[i] == -1:
        colors[i] = [0, 0, 0]  # black for noise points
    else:
        colors[i] = np.random.rand(3)  # random color for each cluster

# Visualize the clusters
pcd.colors = o3d.utility.Vector3dVector(colors)
o3d.visualization.draw_geometries([pcd])
print("Visualize the point cloud.")
