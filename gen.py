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
    origin=[0, 0, 0], roll=0, pitch=0, yaw=0, length=5, width=20, num_points=500
):
    # Origin specifies the location of the southwest corner
    # Roll pitch yaw correspond to phi theta psi by convention and rotate on
    # the x, y, and z axes respectively

    # Generate points lying on a plane of the given size
    x_range = np.array([0, length])
    y_range = np.array([0, width])
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


def circle_gen(resolution=360, radius=5, gap=0, origin=[0, 0, 0]):
    # Gap is size of the gap in radians
    print("Generating circle...")
    origin = np.array(origin)
    angle = np.linspace(0 + gap / 2, 2 * np.pi - gap / 2, resolution)
    x = radius * np.cos(angle)
    y = radius * np.sin(angle)
    z = np.zeros(resolution)
    plane = np.column_stack((x, y, z))
    plane = plane + origin
    return plane


def disc_gen(resolution=6, radius_inner=2, radius_outer=5, yaw=0):
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


def prism(
    origin=[0, 0, 0],
    length=6,
    width=12,
    height=18,
    x_range=[0, 5],
    y_range=[0, 5],
    num_points=3000,
):

    print("\nGenerating prism at", origin)
    # prism_x is the length of the prism along the x axis
    prism_x = length
    prism_y = width
    prism_z = height
    edge_pts = int(num_points / 6)
    side = np.zeros([6, edge_pts, 3])
    side[0] = plane_gen3(
        origin=origin,
        num_points=edge_pts,
        length=prism_x,
        width=prism_y,
    )
    side[1] = plane_gen3(
        origin=(origin + np.array([0, 0, prism_z])),
        num_points=edge_pts,
        length=prism_x,
        width=prism_y,
    )
    side[2] = plane_gen3(
        origin=origin,
        num_points=edge_pts,
        pitch=3 * np.pi / 2,
        length=prism_z,
        width=prism_y,
    )
    side[3] = plane_gen3(
        origin=(origin + np.array([prism_x, 0, 0])),
        pitch=3 * np.pi / 2,
        num_points=edge_pts,
        length=prism_z,
        width=prism_y,
    )
    side[4] = plane_gen3(
        origin=origin,
        num_points=edge_pts,
        roll=np.pi / 2,
        length=prism_x,
        width=prism_z,
    )
    side[5] = plane_gen3(
        origin=(origin + np.array([0, prism_y, 0])),
        num_points=edge_pts,
        roll=np.pi / 2,
        length=prism_x,
        width=prism_z,
    )
    return np.vstack((side))


def ibeam_gen(
    origin=[0, 0, 0],
    length=500,
    height=35,
    width=15,
    thickness=5,
    skip=[False] * 12,
):
    print("Generating I-beam...")
    origin = np.array(origin)
    l = length
    w = width
    h = height
    t = thickness
    widths = np.array(
        [
            w,
            t,
            (w - t) / 2,
            h - 2 * t,
            (w - t) / 2,
            t,
            w,
            t,
            (w - t) / 2,
            h - 2 * t,
            (w - t) / 2,
            t,
        ]
    )
    origins = np.array(
        [
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, t],
            [0, (w - t) / 2, t],
            [0, 0, h - t],
            [0, 0, h - t],
            [0, 0, h],
            [0, w, h - t],
            [0, (w - t) / 2 + t, h - t],
            [0, (w - t) / 2 + t, t],
            [0, (w - t) / 2 + t, t],
            [0, w, 0],
        ]
    )
    rotations = np.array(
        [
            [0, 0, 0],
            [np.pi / 2, 0, 0],
            [0, 0, 0],
            [np.pi / 2, 0, 0],
            [0, 0, 0],
            [np.pi / 2, 0, 0],
            [0, 0, 0],
            [np.pi / 2, 0, 0],
            [0, 0, 0],
            [np.pi / 2, 0, 0],
            [0, 0, 0],
            [np.pi / 2, 0, 0],
            [0, 0, 0],
        ]
    )
    origins = origins + origin
    cloud = np.empty((1, 3))
    for i in range(np.shape(origins)[0]):
        if skip[i] == False:
            cloudt = plane_gen3(
                origin=origins[i, :],
                roll=rotations[i, 0],
                pitch=0,
                yaw=0,
                length=length,
                width=widths[i],
            )
            cloud = np.concatenate((cloud, cloudt))
    print("I-beam: ", cloud)
    return cloud


def cylinder_gen(
    origin=[0, 0, 0],
    gap=np.pi / 4,
    length=5,
    density=50,
    radius=20,
):
    # Needs to be able to have arbitrary open space
    origin = np.array(origin)
    z = np.linspace(0, length, int(density))
    cloud = np.empty((1, 3))
    for i in z:
        cloudt = circle_gen(origin=[0, 0, i], gap=gap)
        cloud = np.concatenate((cloud, cloudt))
    return cloud


def tbeam_gen(
    origin=[0, 0, 0],
    length=500,
    height=35,
    width=15,
    thickness=5,
    skip=[False] * 12,
):
    print("Generating I-beam...")
    origin = np.array(origin)
    l = length
    w = width
    h = height
    t = thickness
    widths = np.array(
        [
            w,
            t,
            (w - t) / 2,
            h - 2 * t,
            (w - t) / 2,
            h - 2 * t,
            (w - t) / 2,
            t,
        ]
    )
    origins = np.array(
        [
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, t],
            [0, (w - t) / 2, t],
            [0, (w - t) / 2, h - t],
            [0, (w - t) / 2 + t, t],
            [0, (w - t) / 2 + t, t],
            [0, w, 0],
        ]
    )
    rotations = np.array(
        [
            [0, 0, 0],
            [np.pi / 2, 0, 0],
            [0, 0, 0],
            [np.pi / 2, 0, 0],
            [0, 0, 0],
            [np.pi / 2, 0, 0],
            [0, 0, 0],
            [np.pi / 2, 0, 0],
            [0, 0, 0],
        ]
    )
    origins = origins + origin
    cloud = np.empty((1, 3))
    for i in range(np.shape(origins)[0]):
        if skip[i] == False:
            cloudt = plane_gen3(
                origin=origins[i, :],
                roll=rotations[i, 0],
                pitch=0,
                yaw=0,
                length=length,
                width=widths[i],
            )
            cloud = np.concatenate((cloud, cloudt))
    print("T-beam: \n", cloud)
    return cloud


def stiffener_bulb_flat(a=-2, b=1, c=0, num_points=300, width=5):
    print("Generating bulb flat stiffener...")
    # Generate curved section
    # Basic eq: (-2)x^2 + (1)*x + 0 = y with x=[0,1] graph
    z = np.linspace(0, width, int(num_points))
    cloud = np.empty((1, 3))
    for i in z:
        x = np.linspace(0, 1, num_points)
        y = a * x**2 + b * x + c
        z = np.full(num_points, i)
        print("z[", i, "]: ", z)
        cloudt = np.column_stack([x, y, z])
        cloud = np.concatenate((cloud, cloudt))
    print("shape is: ", np.column_stack([x, y, z]).shape)
    cloudt = plane_gen3(
        origin=[1, -1, 0],
        roll=np.pi / 2,
        pitch=0,
        yaw=0,
        length=2,
        width=width,
        num_points=500,
    )
    cloud = np.concatenate((cloud, cloudt))
    # print("Bulb flat stiffiner:\n", cloud)
    return cloud


def rot_mat(roll=0, pitch=0, yaw=0):
    # returns appropiate rotation matrix for a roll pitch and yaw
    print("Generating rotation matrix...")
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
    return R_yaw @ R_pitch @ R_roll


def stiffener_unkown_name_gen(
    origin=[0, 0, 0],
    length=15,
    stem_width=3,
    stem_height=9,
    bulb_radius=3,
    radius=4,
):
    cloud = np.empty((1, 3))

    # generate stem
    cloudt = plane_gen3(length=length, width=stem_height, roll=np.pi / 2)
    cloud = np.concatenate((cloud, cloudt))
    cloudt = plane_gen3(length=length, width=stem_width)
    cloud = np.concatenate((cloud, cloudt))
    cloudt = plane_gen3(
        length=length,
        width=stem_height,
        roll=np.pi / 2,
        origin=[0, stem_width, 0],
    )
    cloud = np.concatenate((cloud, cloudt))

    # generate bulb
    cloudt = cylinder_gen(
        origin=[0, 0, 0],
        gap=stem_width / bulb_radius,
        length=length,
        density=50,
        radius=bulb_radius,
    )
    R = rot_mat(pitch=np.pi / 2)
    cloudt = cloudt @ R.T
    cloudt = cloudt + np.array([0, stem_width / 2, stem_height + radius])
    cloud = np.concatenate((cloud, cloudt))
    return cloud


def curved_wall_gen(x_range=[0, 25], a=3, b=4, c=1, width=90, num_points=800):
    x = np.random.uniform(x_range[0], x_range[1], num_points)
    z = (x * a) ** 2 + (x * b) + c
    y = np.linspace(0, width, int(num_points))
    return np.column_stack((x, y, z))


if __name__ == "__main__":
    # note that the new tensor .t is required. legacy version will break this
    # pcd.paint_uniform_color(np.array([0, 0.7, 0], dtype=np.float32))
    pcd1 = o3d.t.geometry.PointCloud(stiffener_bulb_flat())

    # display point cloud in browser window (as opposed to locally with draw_geometry)
    o3d.visualization.draw_plotly([pcd1.to_legacy()])
#    o3d.visualization.draw_geometries([pcd1.to_legacy(), pcd2.to_legacy()])
#
