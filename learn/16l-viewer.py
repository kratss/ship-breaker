#!/usr/bin/env python
import open3d as o3d
import numpy as np
import open3d.core as o3c

# write name of desired file in quotes
pcd = o3d.io.read_point_cloud("shipcloud.svo2")
o3d.visualization.draw_geometries([pcd])
