#!/usr/bin/env python
"""
Interface that brings together library path planning functionality

grid refers to the 2D projection of the point cloud slice onto a grid
such that it can be viewed as an image
"""
import cv2
import contour
import extract_plane as ep
import gen
from icecream import ic
import matplotlib.pyplot as plt
import model
import numpy as np
from skimage.morphology import skeletonize


class Path:
    def __init__(self, component_groups, grid_res, z_plane):
        """
        Initializes Path

        Args:
            component_groups: list of component groups, i.g. group of
                T-beams, group of I-beams
            grid_res: number of squares in grid per unit in the point
                cloud
            max_grid: dimensions of the grid
            components: list of individual component objects
            components_ordered: list of components ordered to form a
                path for the plasma torch
            coords2d: numpy array of 2d coordinates describing the
                cutting path. NOT resized
            coords3d: numpy array of 3d coordinates describing the
                cutting path. Resized to original point cloud scale
            grid_path: 2D representation of the calculated cutting path
                in grid scale, not cloud scale
        """
        self.component_groups = component_groups
        self.grid_res = grid_res
        self.z_plane = z_plane
        self.max_grid = self.get_grid_size()
        self.components = self.get_components()
        self.components_ordered = self.order_components()
        self.coords2d = np.concatenate(
            [comp.cntr for comp in self.components_ordered], axis=0
        )
        self.coords3d = ep.get_3d(self.coords2d, self.grid_res, self.z_plane)
        self.grid_path = self.get_grid_path()

    def get_grid_size(self):
        """
        Find the size of the image grid
        """
        max_grid = [0, 0]
        for group in self.component_groups:
            if group.grid.shape[0] > max_grid[0]:
                max_grid[0] = group.grid.shape[0]
            if group.grid.shape[1] > max_grid[1]:
                max_grid[1] = group.grid.shape[1]
        return max_grid

    def get_components(self):
        """
        Sort tagged point cloud into individual objects
        """
        components = []
        for group in self.component_groups:
            for i, cntr in enumerate(group.cntrs):
                components.append(
                    contour.Component(group.name[:-1] + str(i), cntr, self.max_grid)
                )
        return components

    def order_components(self):
        """
        Order components based on which component.first_point is closest
        to the current components .last_point
        """
        components_ordered = []
        remaining = self.components.copy()
        ic(f"Starting with {len(remaining)} components")
        current_component = remaining[0]
        del remaining[0]
        components_ordered.append(current_component)
        while remaining:
            ic(f"Remaining: {[c.name for c in remaining]}")
            ic(f"Ordered so far: {[c.name for c in components_ordered]}")
            distances = [
                (idx, np.sum((comp.first_point - current_component.last_point) ** 2))
                for idx, comp in enumerate(remaining)
            ]
            nearest_idx = min(distances, key=lambda x: x[1])[0]
            current_component = remaining[nearest_idx]
            del remaining[nearest_idx]
            components_ordered.append(current_component)

        ic(f"Final ordered: {[c.name for c in components_ordered]}")
        return components_ordered

    def get_grid_path(self):
        """
        Create 2D representation of the calculated cutting path
        in grid scale, not cloud scale
        """
        grid_path = np.zeros(self.max_grid)
        grid_path[self.coords2d[:, 1], self.coords2d[:, 0]] = 1
        return grid_path

    def visualize(self):
        """
        Show information about the Path object
        """
        plt.imshow(self.grid_path, cmap="gray")
        plt.show()


if __name__ == "__main__":

    ic(dir(contour))
    ic(hasattr(contour, "Component"))
    # Generated data
    DENSITY = 35
    NOISE_STD = 0.05
    Z_PLANE = 5
    GRID_RES = 5
    TOLERANCE = 1
    clouds = {
        "curved_walls": model.gen_curved_walls(DENSITY, NOISE_STD),
        "planes": model.gen_planes(DENSITY, NOISE_STD),
        "tbeams": model.gen_tbeams(DENSITY, NOISE_STD),
    }

    # Collect clouds and create objects
    component_groups = []
    for key, value in clouds.items():
        component_groups.append(
            contour.ComponentGroup(key, value, Z_PLANE, GRID_RES, TOLERANCE)
        )

    my_path = Path(component_groups, GRID_RES, Z_PLANE)
    ### Sort tagged point cloud into individual objects
    ic("Components detected:")
    for comp in my_path.components:
        ic(comp.name)
    ### Choose the order of the components
    ic("Calculated component order")
    for comp in my_path.components_ordered:
        ic(comp.name)
    # Collect all coordinates in order
    ic(my_path.coords2d.shape)
    ic(my_path.coords3d.shape)
    ic(my_path.grid_path.shape)

    ### Visualize and print info
    my_path.visualize()
