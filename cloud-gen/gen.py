#!/usr/bin/env python3
import open3d as o3d
import numpy as np


# basic point cloud creation
# https://www.open3d.org/html/tutorial/t_geometry/pointcloud.html


def plane_gen(x_range=[0, 5], y_range=[0, 5], num_points=[500], a=1, b=2, c=3, d=4):
    # Wall

    # returns a point cloud lying on the defined plane
    # ax + by + cz + d = 0
    # a, b, c are given by plane defintion
    # plug in any x and y to generate a valid z
    y_range = [0, 5]
    x_range = [0, 5]
    num_points = 1000
    x = np.random.uniform(x_range[0], x_range[1], num_points)
    y = np.random.uniform(y_range[0], y_range[1], num_points)
    b = 3
    c = 1
    d = 2
    z = -1 * (a * x + b * y + d) / c
    plane = np.column_stack((x, y, z))
    return plane


def circle_gen(resolution=360, radius=5):
    angle = np.linspace(0, 2 * np.pi, resolution)
    x = radius * np.cos(angle)
    y = radius * np.sin(angle)
    z = np.zeros(resolution)
    print("y is:", y)
    print("\nx is: ", x)
    return np.column_stack((x, y, z))


def disc_gen(resolution=6, radius_inner=2, radius_outer=5, yaw=0.4):
    radius = np.linspace(radius_inner, radius_outer, resolution)
    print("\nradius[1] is: ", radius[1])
    x = []
    y = []
    z = []
    for i in range(radius.shape[0]):
        angle = np.linspace(0, 2 * np.pi, int(resolution * radius[i]))
        xt = radius[i] * np.cos(angle)
        yt = radius[i] * np.sin(angle)
        zt = np.zeros(yt.shape[0])
        x = np.concatenate((x, xt))
        y = np.concatenate((y, yt))
        z = np.concatenate((z, zt))
    coordinates = np.column_stack((x, y, z))
    rotation_yaw = np.array(
        [
            [np.cos(yaw), 0, np.sin(yaw)],
            [0, 1, 0],
            [-np.sin(yaw), 0, np.cos(yaw)],
        ]
    )
    coordinates = coordinates @ rotation_yaw.T
    return coordinates


if __name__ == "__main__":
    # create point cloud var with some [x,y,z] data in PointCloud()
    pcd = o3d.t.geometry.PointCloud(disc_gen(resolution=20))
    pcd2 = o3d.t.geometry.PointCloud(disc_gen(resolution=20, yaw=0.1))
    print(pcd)

    # display point cloud in browser window (as opposed to locally with draw_geometry)
    o3d.visualization.draw_plotly([pcd.to_legacy(), pcd2.to_legacy()])
