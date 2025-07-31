#!/usr/bin/env python3
"""
Generates pre-configured point cloud models of ships and components

A unit of length in the point cloud equals one centimeter

Functions are separated into gen_planes, gen_tbeams, and so on to mimcic
how tagged groups of elements will be passed to the path finding algo, while
gen_ship() generates a more complete model for visualization purposes
"""
from planner import gen
import numpy as np
import open3d as o3d
from icecream import ic


def gen_curved_walls(density, noise_std):
    origin = [55, 0, 20]
    cloud = np.concatenate(
        (
            gen.curved_wall(
                origin=origin, roll=np.pi / 2, pitch=np.pi, yaw=np.pi, height=30
            ),
            gen.curved_wall(
                origin=[100, 0, 0], roll=np.pi / 2, pitch=0, yaw=np.pi, height=30
            ),
        )
    )

    cloud = gen.noise(cloud, std=noise_std)
    z_min = cloud[:, 2].min()
    z_max = cloud[:, 2].max()
    return cloud


def gen_planes(density, noise_std):
    cloud = np.concatenate(
        (
            gen.plane(origin=[16, 30, 0], length=8, width=8, roll=np.pi / 2),
            # gen.plane(origin=[31, 30, 0], length=8, width=8, roll=np.pi / 2),
            # gen.plane(origin=[46, 30, 0], length=8, width=8, roll=np.pi / 2),
        )
    )
    cloud = gen.noise(cloud, std=noise_std)
    return cloud


def gen_tbeams(density, noise_std):
    LENGTH = 10
    WIDTH = 20
    HEIGHT = 50
    THICKNESS = 0.1
    cloud = np.concatenate(
        (
            gen.tbeam(
                origin=[30, 30, 0],
                length=10,
                width=20,
                height=50,
                thickness=0.1,
                roll=-np.pi / 2,
                pitch=-np.pi / 2,
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
                density=density,
            ),
            gen.tbeam(
                origin=[45, 30, 0],
                length=10,
                width=5,
                roll=-np.pi / 2,
                pitch=-np.pi / 2,
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
                density=density,
            ),
        )
    )
    z_min = cloud[:, 2].min()
    z_max = cloud[:, 2].max()
    cloud = gen.noise(cloud, std=noise_std)
    return cloud


def gen_tbeams_many(density, noise_std):
    LENGTH = 10
    WIDTH = 20
    HEIGHT = 50
    THICKNESS = 0.1cloud = np.concatenate(
        (
            gen.tbeam(
                origin=[70, 30, 0],
                length=10,
                width=5,
                height=6,
                thickness=0.7,
                roll=-np.pi / 2,
                pitch=-np.pi / 2,
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
                density=density,
            ),
            gen.tbeam(
                origin=[90, 30, 0],
                length=10,
                width=5,
                height=6,
                thickness=0.7,
                roll=-np.pi / 2,
                pitch=-np.pi / 2,
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
                density=density,
            ),
            gen.tbeam(
                origin=[110, 30, 0],
                length=10,
                width=5,
                height=6,
                thickness=0.7,
                roll=-np.pi / 2,
                pitch=-np.pi / 2,
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
                density=density,
            ),
            gen.tbeam(
                origin=[130, 30, 0],
                length=10,
                width=5,
                height=6,
                thickness=0.7,
                roll=-np.pi / 2,
                pitch=-np.pi / 2,
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
                density=density,
            ),
            gen.tbeam(
                origin=[150, 30, 0],
                length=10,
                width=5,
                height=6,
                thickness=0.7,
                roll=-np.pi / 2,
                pitch=-np.pi / 2,
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
                density=density,
            ),
        )
    )
    z_min = cloud[:, 2].min()
    z_max = cloud[:, 2].max()
    cloud = gen.noise(cloud, std=noise_std)
    return cloud


def gen_floor(density, noise_std):
    cloud = gen.plane(origin=[57, 0, 0], length=60, width=30, roll=np.pi / 2)
    cloud = gen.noise(cloud, std=noise_std)
    return cloud


def gen_ship(density, noise_std):
    """
    Generates a pre configured point cloud representing a ship

    Args:
        density (float)
        noise_std (float)

    Returns:
        Numpy array representing X,Y,Z coordinates of point cloud
        points
    """
    return gen.noise(
        np.concatenate(
            (
                gen_curved_walls(density, noise_std),
                gen_planes(density, noise_std),
                gen_floor(density, noise_std),
                gen_tbeams(density, noise_std),
            )
        )
    )


if __name__ == "__main__":
    """
    Demo premade models
    """
    DENSITY = 33
    NOISE_STD = 0.05
    pcd = o3d.t.geometry.PointCloud(gen_tbeams_many(DENSITY, NOISE_STD))
    axes = gen.draw_axes()
    o3d.visualization.draw_geometries([pcd.to_legacy(), axes])
