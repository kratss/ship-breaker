#!/usr/bin/env python
import primitives as pt


DENSITY = 35
NOISE_STD = 0.01
clouds = {
    "curved_walls": pt.gen_curved_walls(DENSITY, NOISE_STD),
    "planes": pt.gen_planes(DENSITY, NOISE_STD),
    "tbeams": pt.gen_tbeams_many(DENSITY, NOISE_STD),
}

my_cloud = pt.Cloud(clouds, 4.8, 1.1)
