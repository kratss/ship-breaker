import open3d as o3d
import numpy as np


# basic point cloud creation
# https://www.open3d.org/html/tutorial/t_geometry/pointcloud.html

# create point cloud var with some [x,y,z] data in PointCloud()
plane = np.loadtxt("plane.xyz")
pcd = o3d.t.geometry.PointCloud(plane)
ply_point_cloud = o3d.data.PLYPointCloud()
pcd2 = o3d.t.io.read_point_cloud(ply_point_cloud.path)
print(pcd)

# display point cloud in browser window (as opposed to locally with draw_geometry)
o3d.visualization.draw_plotly([pcd.to_legacy(), pcd2.to_legacy()])
