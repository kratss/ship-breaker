#!/usr/bin/env python
import open3d as o3d
import gen
import numpy as np


origin = o3d.geometry.LineSet()
density = 5
points = np.vstack(
    [
        [0, 0, 0],
        [10, 0, 0],
        [0, 20, 0],
        [0, 0, 30],
    ]
)
lines = np.array([[0, 1], [0, 2], [0, 3]])
origin.points = o3d.utility.Vector3dVector(points)
origin.lines = o3d.utility.Vector2iVector(lines)
density = 2
np.set_printoptions(precision=1, floatmode="fixed")
# roof, roof tbeam 1, root tbeam 2
pcd_points = np.concatenate(
    (
        gen.plane(origin=[5, 5, 5], length=10, width=2, density=density),
        gen.plane(
            origin=[5, 5, 5], length=10, width=2, roll=np.pi / 2, density=density
        ),
        gen.plane(
            origin=[5, 5, 5], length=10, width=2, pitch=np.pi / 2, density=density
        ),
        gen.plane(origin=[0, 10, 30], length=110, width=90, density=density),
        gen.tbeam(origin=[0, 30, 30], length=90, density=density),
        gen.tbeam(origin=[0, 60, 30], length=90, density=density),
        gen.tbeam(origin=[0, 90, 30], length=90, density=density),
        gen.curved_wall(
            origin=[110, 5, 0],
            length=110,
            height=30,
            yaw=np.pi / 2,
        ),
        gen.bulb_flat(origin=[0, 10, 30], roll=np.pi / 2, yaw=np.pi / 2),
        gen.plane(origin=[0, 0, 0], length=110, width=90, density=density),
        gen.ibeam(origin=[20, 20, 5], roll=np.pi / 2),
    )
)
pcd = o3d.t.geometry.PointCloud(pcd_points)
o3d.visualization.draw_geometries([origin, pcd.to_legacy()])
