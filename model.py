#!/usr/bin/env python3
"""
Generates pre-configured point cloud models of a ship separate components

Functions are separated into gen_planes, gen_tbeams, and so on to mimcic
how tagged groups of elements will be passed to the final program
"""
import gen
import numpy as np
import open3d as o3d
from icecream import ic


def gen_ship():
    """
    Generates a pre configured point cloud representing a ship

    Args:
        None

    Returns:
        Numpy array representing X,Y,Z coordinates of point cloud
        points
    """
    return gen.noise(
        np.concatenate(
            (
                gen.curved_wall(origin=[2, 0, 10], roll=-np.pi / 2, height=30),
                gen.plane(origin=[10, 30, 0], length=10, width=10, roll=np.pi / 2),
                gen.tbeam(
                    origin=[20, 30, 10],
                    length=10,
                    width=5,
                    thickness=1,
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
                gen.plane(origin=[25, 30, 0], length=10, width=10, roll=np.pi / 2),
                gen.tbeam(
                    origin=[35, 30, 10],
                    length=10,
                    width=5,
                    thickness=1,
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
                gen.plane(origin=[40, 30, 0], length=10, width=10, roll=np.pi / 2),
                gen.plane(origin=[60, 0, 0], length=10, width=30, pitch=-np.pi / 2),
                gen.bulb_flat(origin=[60, 10, 0]),
            )
        )
    )


def gen_curved_walls(density, noise_std):
    origin = [5, 0, 20]
    cloud = gen.curved_wall(origin=origin, roll=np.pi / 2, pitch=np.pi, yaw=np.pi)
    cloud = gen.noise(cloud, std=noise_std)
    z_min = cloud[:, 2].min()
    z_max = cloud[:, 2].max()
    return cloud


def gen_planes(density, noise_std):
    cloud = np.concatenate(
        (
            gen.plane(origin=[16, 30, 0], length=8, width=8, roll=np.pi / 2),
            gen.plane(origin=[31, 30, 0], length=8, width=8, roll=np.pi / 2),
            gen.plane(origin=[46, 30, 0], length=8, width=8, roll=np.pi / 2),
        )
    )
    cloud = gen.noise(cloud, std=noise_std)
    return cloud


def gen_tbeams(density, noise_std):
    cloud = np.concatenate(
        (
            gen.tbeam(
                origin=[30, 30, 0],
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


def gen_floor(desnity, noise_std):
    cloud = np.concatenate(
        (
            gen.plane(origin=[15, 0, 0], length=30, width=30, roll=np.pi / 2),
            gen.plane(origin=[45, 0, 0], length=30, width=30, roll=np.pi / 2),
        )
    )
    cloud = gen.noise(cloud, std=noise_std)
    return cloud


if __name__ == "__main__":
    density = 15
    pcd = o3d.t.geometry.PointCloud(gen_ship())
    axes = gen.draw_axes()
    o3d.visualization.draw_geometries([pcd.to_legacy(), axes])
