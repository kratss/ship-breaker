#! /usr/bin/env python3
"""Programmatically generate point clouds"""
import open3d as o3d
import numpy as np
from icecream import ic


def rot_mat(roll=0, pitch=0, yaw=0):
    """Returns rotation matrix
    Return rotation matrix described by a given roll, pitch, and yaw
    to rotate a point cloud about the x, y, and z axes by matrix
    multiplying it by the array coordinates

    Roll, pitch, and yaw are rotations about the x, y, and z axes respectively

    Args:
        roll: rotation about x axis in radians
        pitch: rotation about y axis in radians
        yaw: rotation about z axis in radians

    Returns:
        A numpy array of shape (3,3) representing a rotation matrix
    """
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
    R = R_yaw @ R_pitch @ R_roll
    return R


def noise(cloud, std=0.01):
    print("Adding noise...")
    cloud_noisy = np.random.normal(cloud, std)
    return cloud_noisy


def plane(
    origin=[0, 0, 0],
    roll=0,
    pitch=0,
    yaw=0,
    length=5,
    width=20,
    density=20,
):
    """
    Generate points lying on a plane of the given size
    Origin specifies the location of the southwest corner
    Roll pitch yaw correspond to phi theta psi by convention and rotate on
    the x, y, and z axes respectively
    """
    print("\nGenerating plane...")
    origin = np.array(origin)
    Z = 0
    X = np.linspace(0, length, int(length * density))
    Y = np.linspace(0, width, int(width * density))
    cloud = np.array([[0, 0, 0]])
    for x in X:
        a = np.full(np.shape(Y), x)
        b = Y
        c = np.full(np.shape(Y), Z)
        cloudt = np.column_stack([a, b, c])
        cloud = np.concatenate([cloud, cloudt])
    R = rot_mat(roll, pitch, yaw)
    # print("plane cloud: \n", cloud)
    cloud = (R @ cloud.T).T
    # print("plane cloud after rotation: \n", np.round(cloud, decimals=0))
    cloud = cloud + origin

    # print("Shape of plane cloud: ", np.shape(cloud))
    return cloud


def circle(origin=[0, 0, 0], resolution=360, radius=5, gap=0):
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


def disc(resolution=6, radius_inner=2, radius_outer=5, yaw=0):
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


def cylinder(
    origin=[0, 0, 0],
    gap=np.pi / 4,
    length=5,
    density=50,
    radius=20,
):
    # Needs to be able to have arbitrary open space
    origin = np.array(origin)
    z = np.linspace(0, length, int(density))
    cloud = np.array([[0, 0, 0]])
    for i in z:
        cloudt = circle(origin=[0, 0, i], gap=gap)
        cloud = np.concatenate((cloud, cloudt))
    return cloud


def prism(
    origin=[0, 0, 0],
    length=6,
    width=12,
    height=18,
    density=15,
):

    print("\nGenerating prism at", origin)
    # prism_x is the length of the prism along the x axis
    prism_x = length
    prism_y = width
    prism_z = height
    cloud = np.array([[0, 0, 0]])
    cloudt = plane(
        origin=origin,
        density=density,
        length=prism_x,
        width=prism_y,
    )
    cloud = np.concatenate([cloud, cloudt])
    cloudt = plane(
        origin=(origin + np.array([0, 0, prism_z])),
        density=density,
        length=prism_x,
        width=prism_y,
    )
    cloud = np.concatenate([cloud, cloudt])
    cloudt = plane(
        origin=origin,
        density=density,
        pitch=3 * np.pi / 2,
        length=prism_z,
        width=prism_y,
    )
    cloud = np.concatenate([cloud, cloudt])
    cloudt = plane(
        origin=(origin + np.array([prism_x, 0, 0])),
        pitch=3 * np.pi / 2,
        density=density,
        length=prism_z,
        width=prism_y,
    )
    cloud = np.concatenate([cloud, cloudt])
    cloudt = plane(
        origin=origin,
        density=density,
        roll=np.pi / 2,
        length=prism_x,
        width=prism_z,
    )
    cloud = np.concatenate([cloud, cloudt])
    cloudt = plane(
        origin=(origin + np.array([0, prism_y, 0])),
        density=density,
        roll=np.pi / 2,
        length=prism_x,
        width=prism_z,
    )
    cloud = np.concatenate([cloud, cloudt])
    return cloud


def ibeam(
    origin=[0, 0, 0],
    length=10,
    height=8,
    width=5,
    thickness=1,
    roll=0,
    pitch=0,
    yaw=0,
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
    cloud = np.array([[0, 0, 0]])
    for i in range(np.shape(origins)[0]):
        if skip[i] == False:
            cloudt = plane(
                origin=origins[i, :],
                roll=rotations[i, 0],
                pitch=0,
                yaw=0,
                length=length,
                width=widths[i],
            )
            cloud = np.concatenate((cloud, cloudt))
    R = rot_mat(roll, pitch, yaw)
    cloud = (R @ cloud.T).T
    origins = origins + origin
    print("I-beam: ", cloud)
    return cloud


def tbeam(
    origin=[0, 0, 0],
    length=10,
    width=5,
    height=10,
    thickness=2,
    roll=0,
    pitch=0,
    yaw=0,
    skip=[False] * 12,
    density=20,
):
    # Origin is measured from TOP of t-beam as defined by its typical orientation
    print("Generating T-beam...")
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
            h - t,
            t,
            h - t,
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
            [0, (w - t) / 2, h],
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
    cloud = np.array([[0, 0, 0]])
    for i in range(np.shape(origins)[0]):
        if skip[i] == False:
            cloudt = plane(
                origin=origins[i, :],
                roll=rotations[i, 0],
                pitch=0,
                yaw=0,
                length=length,
                width=widths[i],
                density=density,
            )
            cloud = np.concatenate((cloud, cloudt))
    R = rot_mat(np.pi, 0, 0)
    cloud = (R @ cloud.T).T
    R = rot_mat(roll, pitch, yaw)
    cloud = (R @ cloud.T).T
    # print("shape of tbeam cloud is :", np.shape(cloud))
    cloud = cloud + origin
    # print("T-beam: \n", cloud)

    return cloud


def bulb_flat(
    origin=[0, 0, 0],
    a=-2,
    b=1,
    c=0,
    num_points=300,
    length=15,
    width=5,
    scale=1,
    roll=0,
    pitch=0,
    yaw=0,
    density=15,
):
    """
    Generate a bulb flat (aka Holland profile) stiffener
    """
    # Generate curved section
    # Basic eq: (-2)x^2 + (1)*x + 0 = y with x=[0,1] graph
    print("Generating bulb flat stiffener...")
    origin = np.array([origin])

    # Generate bulb
    z = np.linspace(0, width, int(num_points))
    cloud = np.array([[0, 0, 0]])
    for i in z:
        x = np.linspace(0, 1, num_points)
        y = a * x**2 + b * x + c
        z = np.full(num_points, i)
        cloudt = np.column_stack([x, y, z])
        cloud = np.concatenate((cloud, cloudt))

    # Generate flat
    cloudt = plane(
        origin=[1, -1, 0] * scale,
        roll=np.pi / 2,
        pitch=0,
        yaw=0,
        length=length,
        width=width,
        density=density,
    )
    cloud = np.concatenate((cloud, cloudt))
    cloudt = plane(
        origin=[1, 0, 0],
        roll=np.pi / 2,
        pitch=0,
        yaw=0,
        length=length,
        width=width,
        density=density,
    )

    # Transform
    cloud = cloud * scale
    R = rot_mat(roll, pitch, yaw)
    cloud = R @ cloud.T
    cloud = cloud.T
    cloud = origin + cloud
    print("Bulb flat stiffener:\n", cloud)
    return cloud


def curved_wall(
    a=1 / 2,
    b=0,
    c=0,
    origin=[0, 0, 0],
    roll=0,
    pitch=0,
    yaw=0,
    length=20,
    height=50,
    density=15,
):
    """
    Create a curved wall given by a second degree polynomial

    Args:
        a: as in a*x^2 + b*x + c
        b: as in a*x^2 + b*x + c
        c: as in a*x^2 + b*x + c
        origin: position of the object


    """
    print("\nGenerating curved wall...")
    origin = np.array(origin)
    Y = np.linspace(0, length, int(length * density))
    Z = np.linspace(0, 1, int(height * density))
    cloud = np.array([[0, 0, 0]])
    for y in Y:
        x = np.full(np.shape(Z), (Z * a) ** 2 + (Z * b) + c)
        y = np.full(np.shape(Z), y)
        z = Z
        cloudt = np.column_stack([x, y, z])
        cloud = np.concatenate([cloud, cloudt])
    cloud[:, 2] = np.multiply(cloud[:, 2], height)
    cloud[:, 0] = np.multiply(cloud[:, 0], height)
    R = rot_mat(roll, pitch, yaw)
    cloud = (R @ cloud.T).T
    cloud = cloud + origin
    # print("cloud shape: ", np.shape(cloud))
    # print("cloud:\n", cloud)
    return cloud


def draw_axes(size=1.0, origin=[0, 0, 0]):
    """
    Draw and color-code the origin
    X axis is red
    Y axis is green
    Z axis is blue

    Arguments:
        size: int representing length of axis lines
    """

    points = [
        origin,  # Origin
        [origin[0] + size, origin[1], origin[2]],  # X-axis end
        [origin[0], origin[1] + size, origin[2]],  # Y-axis end
        [origin[0], origin[1], origin[2] + size],  # Z-axis end
    ]

    lines = [[0, 1], [0, 2], [0, 3]]  # X-axis  # Y-axis  # Z-axis

    colors = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]  # Red for X  # Green for Y  # Blue for Z

    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)

    return line_set


def ship():
    """
    Generates a pre configured point cloud representing a ship

    Args:
        None

    Returns:
        Numpy array representing X,Y,Z coordinates of point cloud
        points
    """
    return noise(
        np.concatenate(
            (
                plane(origin=[0, 30, 0], length=10, width=10, roll=np.pi / 2),
                tbeam(
                    origin=[10, 30, 10],
                    length=10,
                    width=5,
                    roll=-np.pi / 2,
                    pitch=np.pi / 2,
                    skip=[
                        True,
                        False,
                        False,
                        False,
                        False,
                        False,
                        False,
                        False,
                        False,
                        False,
                        False,
                        False,
                    ],
                ),
                plane(origin=[15, 30, 0], length=10, width=10, roll=np.pi / 2),
                tbeam(
                    origin=[25, 30, 10],
                    length=10,
                    width=5,
                    roll=-np.pi / 2,
                    pitch=np.pi / 2,
                    skip=[
                        True,
                        False,
                        False,
                        False,
                        False,
                        False,
                        False,
                        False,
                        False,
                        False,
                        False,
                        False,
                    ],
                ),
            )
        )
    )


if __name__ == "__main__":
    # Note: the new tensor .t is required. Legacy version breaks code
    # Note: opengl backend doesn't work with wayland
    #   Use another draw_* to draw in browser window when required
    pcd = o3d.t.geometry.PointCloud(ship())
    axes = draw_axes()
    o3d.visualization.draw_geometries([pcd.to_legacy(), axes])
