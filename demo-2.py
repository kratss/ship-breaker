#!/usr/bin/env python
from icecream import ic
import open3d as o3d
import primitives as pt

"""
Terminology:
"component" refers to a specific structure in the vessel interior, such
as a beam at a given (X,Y,Z) coordinate.

"component class" refers to all structures of the same type, such as
all T-beams in the environment
"""

# Generated data
DENSITY = 65
NOISE_STD = 0.00
clouds = {
    "curved_walls": pt.gen_curved_walls(DENSITY, NOISE_STD),
    # "planes": pt.gen_planes(DENSITY, NOISE_STD),
    "floors": pt.gen_floor(DENSITY, NOISE_STD),
    "tbeams": pt.gen_tbeams_many(DENSITY, NOISE_STD),
}

# Chosen parameters
Z_PLANE = 5
GRID_DENSITY = 10
TOLERANCE = 1

# Collect input pcd into Cloud object
my_cloud = pt.Cloud(clouds, Z_PLANE, TOLERANCE)

# Create ComponentGroup objects for each component class
# All T-beams are in a ComponentGroup named tbeams,
# All planes are in a ComponentGroup named planes, etc
component_groups = []
for key, value in clouds.items():
    component_groups.append(
        pt.ComponentGroup(
            key,
            value,
            Z_PLANE,
            GRID_DENSITY,
            TOLERANCE,
            my_cloud.grid_dim,
        )
    )


# Create pt.object
#   Creates a component object for each structure in the ship. Store all
#   in a list as the "component" attribute
#
#   Creates components_ordered which orders the
#   the primtives by calling self.stich_primtives()
#
#   Creates ordered list of 2D coordinates that create the calculated
#   path
#
#   Converts ordered list of 2D coordinates to 3D coordinates, which is
#   the final result of the program
my_path = pt.Path(component_groups, GRID_DENSITY, Z_PLANE)
# ic(my_pathcoords2d) # Entire cutting path in grid space
ic(my_path.components[2].get_info())
my_path.visualize()
