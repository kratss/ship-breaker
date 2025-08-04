#!/usr/bin/env python
from icecream import ic
import open3d as o3d
import planner as pln

# Generated data
DENSITY = 0.8
NOISE_STD = 0.01
clouds = {
    "curved_walls": pln.gen_curved_walls(DENSITY, NOISE_STD),
    "floors": pln.gen_floor(DENSITY, NOISE_STD),
    "tbeams": pln.gen_tbeams(DENSITY, NOISE_STD),
}

# Chosen parameters
Z_PLANE = 5
GRID_DENSITY = 1
TOLERANCE = 1

# Collect input pcd into Cloud object
my_cloud = pln.Cloud(clouds, Z_PLANE, TOLERANCE)

# Create ComponentGroup objects for each component class
# All T-beams are in a ComponentGroup named tbeams,
# All planes are in a ComponentGroup named planes, etc
component_groups = []
for key, value in clouds.items():
    component_groups.append(
        pln.ComponentGroup(
            key,
            value,
            Z_PLANE,
            GRID_DENSITY,
            TOLERANCE,
            my_cloud.grid_dim,
        )
    )


# Create pln.object
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
my_path = pln.Path(component_groups, GRID_DENSITY, Z_PLANE)

# Get information and visualizations
ic(my_path.coords2d)  # Entire cutting path in grid space
ic(my_path.components[2].get_info())
my_cloud.visualize()
my_path.visualize()
