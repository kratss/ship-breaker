#!/usr/bin/env python
import open3d as o3d
import gen
import numpy as np

pcd = o3d.t.geometry.PointCloud(
    gen.ibeam(
        origin=[3, 5, 1],
        skip=[
            False,
            False,
            False,
            False,
            False,
            False,
            True,
            False,
            False,
            False,
            False,
            False,
        ],
        length=15,
    )
)
pcd2 = o3d.t.geometry.PointCloud(
    gen.tbeam(
        origin=[30, 40, 0],
        skip=[
            False,
            False,
            False,
            False,
            True,
            False,
            False,
            False,
            False,
            False,
            False,
            False,
        ],
        width=4,
        height=12,
        thickness=1,
        length=15,
    )
)
pcd3 = o3d.t.geometry.PointCloud(gen.curved_wall(a=3) + np.array([80, 0, 0]))
o3d.visualization.draw_plotly([pcd.to_legacy(), pcd2.to_legacy()])
o3d.visualization.draw_plotly([pcd3.to_legacy()])
