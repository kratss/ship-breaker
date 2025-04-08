#!/usr/bin/env python
# %% Cell 1
import open3d as o3d
import numpy as np
import open3d.core as o3c


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
normal = np.array([0, 0, 1])  # TODO: take the cutting plane as input to define this
point_on_plane = np.array(
    [0, 0, 0]
)  # point on the plane. With this plus the normal we have a plane definition

# distance from each point to the plane
# get the projection of the vector from a given point to the plane along the normal of the plain
distances = np.dot(pcd_plane_intersection.points - point_on_plane, normal)

# Translate the points along the normal vector of the plane
translated_points = pcd_plane_intersection.points - distances[
    :, np.newaxis
] * normal / np.linalg.norm(normal)

# Create a new point cloud with the translated points
translated_pcd = o3d.geometry.PointCloud()
translated_pcd.points = o3d.utility.Vector3dVector(translated_points)


distances = np.dot(pcd_plane_intersection.points - point_on_plane, normal)

# Translate the points along the normal vector of the plane
translated_points = pcd_plane_intersection.points - distances[
    :, np.newaxis
] * normal / np.linalg.norm(normal)

# Create a new point cloud with the translated points
translated_pcd = o3d.geometry.PointCloud()
translated_pcd.points = o3d.utility.Vector3dVector(translated_points)


### Nearest neighbors stuff
# create a kdtree of the pcd to facilitate knn ops
kdtree = o3d.geometry.KDTreeFlann(translated_pcd)
dist = []
idx = []
# get a list of the 1 nearest point to each of the first 50 points in translated_pcd
for i in range(50):
    point = translated_pcd.points[i]
    [d, i_idx, _] = kdtree.search_hybrid_vector_3d(point, 0.1, 1)
    dist.append(d)
    idx.append(i_idx)

for i in range(5):
    point = translated_pcd.points[i]
    nearest_neighbor_idx = idx
    nearest_neighbor = np.asarray(translated_pcd.points)[nearest_neighbor_idx]
    print(f"Point {i}: {point}")
    print(f"nearest_neighbor_idx: {nearest_neighbor_idx}")
    print(f"Nearest Neighbor: {nearest_neighbor}")
    print(f"Distance: {dist[i]}")
    print("--------------------")


# %% Cell 2
### Draw a line
# lineset is a collection of lines
print("=================")
translated_pcd_points = np.asarray(translated_pcd.points[0:10])
nearest_neighbor = np.asarray(nearest_neighbor[0:10])
nearest_neighbor = np.squeeze(nearest_neighbor)
print(translated_pcd_points.shape)
print(nearest_neighbor.shape)
points = np.stack([translated_pcd.points[0:10], nearest_neighbor[0:10]])

lines = np.array([[i, i + 10] for i in range(10)])
lines = np.array([[0, 1]])
line_set = o3d.geometry.LineSet()
line_set.points = o3d.utility.Vector3dVector(points)
# lines here indexes the points in line_set.points, telling it which lines to
# form a point from
line_set.lines = o3d.utility.Vector2iVector(lines)

# %% Cell 3
o3d.visualization.draw_geometries([pcd, plane_pcd])
o3d.visualization.draw_geometries([pcd_plane_intersection, translated_pcd, line_set])
