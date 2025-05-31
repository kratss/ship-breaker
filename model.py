#!/usr/bin/env python3
"""
Generates pre-configured point cloud models of a ship separate components

Functions are separated into gen_planes, gen_tbeams, and so on to mimcic
how tagged groups of elements will be passed to the final program
"""
import gen
import numpy as np
import open3d as o3d


def gen_ship():
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


def gen_planes(density=5, noise_std=0.01):
    cloud = np.concatenate(
        (
            gen.plane(origin=[15, 10, 0], length=10, width=10, roll=np.pi / 2),
            gen.plane(origin=[30, 10, 0], length=10, width=10, roll=np.pi / 2),
            gen.plane(origin=[45, 10, 0], length=10, width=10, roll=np.pi / 2),
        )
    )
    cloud = gen.noise(cloud, std=noise_std)
    return cloud


def gen_tbeams(density=5, noise_std=0.01):
    cloud = np.concatenate(
        (
            gen.tbeam(
                origin=[30, 10, 0],
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
                origin=[45, 10, 0],
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
    cloud = gen.noise(cloud, std=noise_std)
    return cloud


if __name__ == "__main__":
    density = 15
    pcd = o3d.t.geometry.PointCloud(gen_tbeams(density=5))
    axes = gen.draw_axes()
    o3d.visualization.draw_geometries([pcd.to_legacy(), axes])
