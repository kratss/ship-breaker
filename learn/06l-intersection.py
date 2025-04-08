import open3d as o3d
import numpy as np
import open3d.core as o3c


def plane_gen(x_range=[0, 5], y_range=[0, 5], num_points=[500], a=1, b=2, c=3, d=4):
    # returns a point cloud lying on the defined plane
    # ax + by + cz + d = 0
    # a, b, c are given by plane defintion
    # plug in any x and y to generate a valid z
    y_range = [0, 5]
    x_range = [0, 5]
    num_points = 1000
    x = np.random.uniform(x_range[0], x_range[1], num_points)
    y = np.random.uniform(y_range[0], y_range[1], num_points)
    a = 4
    b = 3
    c = 1
    d = 2
    z = -1 * (a * x + b * y + d) / c
    plane = np.column_stack((x, y, z))
    return plane


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
