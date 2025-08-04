#!/usr/bin/env python
from icecream import ic
import open3d as o3d
import planner as pln

# Generated data
DENSITY = 1
NOISE_STD = 0.1
clouds = {
    "curved_walls": pln.gen_curved_walls(DENSITY, NOISE_STD),
    "floors": pln.gen_floor(DENSITY, NOISE_STD),
    "tbeams": pln.gen_tbeams(DENSITY, NOISE_STD),
}

# Chosen parameters
Z_PLANE = 5
GRID_DENSITY = 1
TOLERANCE = 1

my_cloud = pln.Cloud(clouds, Z_PLANE, TOLERANCE)

my_cloud.visualize()
