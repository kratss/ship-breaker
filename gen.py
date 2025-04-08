#!/usr/bin/env python3
import open3d as o3d
import numpy as np


# basic point cloud creation
# https://www.open3d.org/html/tutorial/t_geometry/pointcloud.html


def plane_gen1(x_range=[0, 5], y_range=[0, 5], num_points=[500], a=0, b=0, c=0, d=0):
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
    z = -1 * (a * x + b * y + d) / c
    plane = np.column_stack((x, y, z))
    print("\nplane: ", np.shape(plane))
    return plane


def plane_gen2(x_range=[0, 5], y_range=[0, 5], num_points=[500], a=1, b=2, c=3, d=4):
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
    print("\nplane: ", np.shape(plane))
    return plane


def plane_gen3(
    origin=[0, 0, 0], roll=0, pitch=0, yaw=0, width=5, length=5, num_points=500
):
    # Origin specifies the location of the southwest corner
    # Roll pitch yaw correspond to phi theta psi by convention and rotate on
    # the x, y, and z axes respectively

    # Generate points lying on a plane of the given size
    x_range = np.array([0, width])
    y_range = np.array([0, length])
    z = np.full(num_points, 0)
    x = np.random.uniform(x_range[0], x_range[1], num_points)
    y = np.random.uniform(y_range[0], y_range[1], num_points)

    # Rotate the plane
    R_roll = np.array(
        [
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)],
        ]
    )
    R_pitch = np.array(
        [
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)],
        ]
    )
    R_yaw = np.array(
        [
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1],
        ]
    )
    plane = np.array([x, y, z]).T
    plane = R_roll @ R_pitch @ R_yaw @ plane.T

    # Translate plane to specified origin
    plane[0, :] = plane[0, :] + origin[0]
    plane[1, :] = plane[1, :] + origin[1]
    plane[2, :] = plane[2, :] + origin[2]
    plane = plane.T
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


def ibeam_gen(
    length=5, x_range=[0, 5], y_range=[0, 5], num_points=[500], a=0, b=0, c=0, d=0
):
    flange1 = plane_gen(
        x_range=[0 - length, 5],
        y_range=[0 - length, 5],
        num_points=[500],
        a=1,
        b=2,
        c=3,
        d=4,
    )

    # web
    flange2 = plane_gen(
        x_range=[0 + length, 5 + length],
        y_range=[0 + length, 5 + length],
        num_points=[500],
    )
    print("\n ibeam_gen: \n", np.vstack((flange1, flange2)))
    return np.vstack((flange1, flange2))


def plane_gen2(
    origin=[0, 0, 0], roll=0, pitch=0, yaw=0, width=5, length=5, num_points=1000
):

    # a(x-x0) + b(y-y0) + c(z-z0) where abc is the normal to the plane, and x0y0z0 is the "northeast" corner
    # roll pitch yaw correspond to phi theta psi by convention and rotate on
    # the x, y, and z axes respectively
    x_range = np.array([origin[0], origin[0] + width])
    y_range = np.array([origin[1], origin[1] + length])
    z = np.full(num_points, origin[2])
    x = np.random.uniform(x_range[0], x_range[1], num_points)
    y = np.random.uniform(y_range[0], y_range[1], num_points)
    R_roll = np.array(
        [
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)],
        ]
    )
    R_pitch = np.array(
        [
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)],
        ]
    )

    R_yaw = np.array(
        [
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1],
        ]
    )
    plane = np.array([x, y, z]).T
    print("\nplane:\n", np.shape(plane))
    plane = R_roll @ R_pitch @ R_yaw @ plane.T
    print("(x,y,z)^T R", np.shape(plane))
    return plane.T


class Plane:
    def __init__(self, p1, p2, p3):
        self.p1 = np.array(p1)
        self.p2 = np.array(p2)
        self.p3 = np.array(p3)
        self.normal = self.get_normal()
        self.plane_def = self.get_plane_def()

    def get_normal(self):
        vectors = np.array([self.p2 - self.p1, self.p3 - self.p1])
        normal = np.cross(vectors[0], vectors[1])
        return normal

    def get_plane_def(self):
        print("self.normal", self.normal)
        a = b = c = 0
        x0, y0, z0 = self.p1
        d = -a * x0 - b * y0 - c * z0
        return a, b, c, d


if __name__ == "__main__":
    # note that the new tensor .t is required. legacy version will break this
    # pcd.paint_uniform_color(np.array([0, 0.7, 0], dtype=np.float32))
    pcd = o3d.t.geometry.PointCloud(plane_gen2())
    pcd2 = o3d.t.geometry.PointCloud(plane_gen3(origin=[0, 9, 0], roll=np.pi / 2))

    # display point cloud in browser window (as opposed to locally with draw_geometry)
    # o3d.visualization.draw_plotly([pcd.to_legacy()])
    o3d.visualization.draw_plotly([pcd.to_legacy(), pcd2.to_legacy()])
