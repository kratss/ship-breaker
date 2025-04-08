import open3d as o3d
import numpy as np


def plane_gen(x_range=[0, 5], y_range=[0, 5], num_points=[500], a=1, b=2, c=3, d=4):
    # returns a point cloud lying on the defined plane
    # ax + by + cz + d = 0
    # a, b, c are given by plane defintion
    # plug in any x and y to generate a valid z
    x = np.random.uniform(x_range[0], x_range[1], num_points)
    y = np.random.uniform(y_range[0], y_range[1], num_points)
    z = -1 * (a * x + b * y + d) / c
    plane = np.column_stack((x, y, z))
    return plane


# pcd = o3d.t.geometry.PointCloud(plane_gen(a=0, c=1, d=-1.8))
# o3d.io.write_point_cloud(filename="plane.pcd", pointcloud=pcd)
plane = plane_gen(a=0, b=0, c=1, d=-1.9, num_points=5000000)
print(plane)
np.savetxt("plane.xyz", plane)
