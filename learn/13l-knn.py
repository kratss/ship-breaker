#!/usr/bin/env python
import open3d as o3d
import numpy as np
import open3d.core as o3c

# %%
# Load data
environment = o3d.data.PLYPointCloud()
environment_pcd = o3d.io.read_point_cloud(environment.path)
plane = np.loadtxt("plane.xyz")
plane_pcd = o3d.geometry.PointCloud()
plane_pcd.points = o3d.utility.Vector3dVector(plane)

# Remove points not intersecting with plane
dists = environment_pcd.compute_point_cloud_distance(plane_pcd)
dists = np.asarray(dists)
ind = np.where(dists < 0.01)[0]
environment_plane_intersection = environment_pcd.select_by_index(ind)
# %%
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


# Draw line
print("points", np.asarray(projected_pcd.points))
projected_pcd_points = np.asarray(projected_pcd.points)
points = np.array([projected_pcd_points[0], projected_pcd_points[1]])
lines = np.array([[0, 1]])
line_set = o3d.geometry.LineSet()
line_set.points = o3d.utility.Vector3dVector(points)
line_set.lines = o3d.utility.Vector2iVector(lines)
line_set.colors = o3d.utility.Vector3dVector(np.array([[1, 0, 0]]))

# Get normalsW
projected_pcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
)
print(np.asarray(projected_pcd.normals))
lines = []
for i in range(len(projected_pcd.points)):
    lines.append([i, len(projected_pcd.points) + i])

normal_lines = o3d.geometry.LineSet()
normal_lines.points = o3d.utility.Vector3dVector(
    np.vstack(
        (
            np.asarray(projected_pcd.points),
            np.asarray(projected_pcd.points) + np.asarray(projected_pcd.normals),
        )
    )
)
normal_lines.lines = o3d.utility.Vector2iVector(lines)

# Visualize
o3d.visualization.draw_geometries([environment_pcd, plane_pcd])
o3d.visualization.draw_geometries(
    [environment_plane_intersection, projected_pcd, line_set, normal_lines]
)
