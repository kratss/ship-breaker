#!/usr/bin/env python
import gen
import model
import extract_plane as ep
from icecream import ic

DENSITY = 56
NOISE_STD = 0.01
Z_PLANE = 5
GRID_RES = 5
TOLERANCE = 1
ic(DENSITY)
my_model = model.gen_curved_walls(DENSITY, NOISE_STD)
ic(ep.cloud_to_grid(my_model, 5, 5, 1))
