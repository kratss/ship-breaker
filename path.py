#!/usr/bin/env python
"""
Determine the cutting path for a plasma torch

This module provides the primary interface for the library
"""
# Note:
#    grid refers to the 2D projection of the point cloud slice onto a grid
#    such that it can be viewed as an image

import cv2
import contour
import extract_plane as ep
import gen
from icecream import ic
import matplotlib.pyplot as plt
import model
import numpy as np
import open3d as o3d
from skimage.morphology import skeletonize


class Path:
    """
    Contains the path for the cutting torch to follow

    Attributes:
        component_groups: list of component groups, i.g. group of
            T-beams, group of I-beams
        grid_res: number of squares in grid per unit in the point cloud
        z_plane: location of the cutting plane alnog the z-axis
        max_grid: dimensions of the grid
        components: list of individual component objects
        components_ordered: list of components ordered to form a path for the plasma torch
        coords2d: numpy array of 2d coordinates describing the cutting path. NOT resized
        coords3d: numpy array of 3d coordinates describing the cutting path. Resized to original point cloud scale
        grid_path: 2D representation of the calculated cutting path in grid scale, not cloud scale
    """

    def __init__(self, component_groups, grid_res, z_plane):
        """
        Initializes Path

        Args:
            component_groups:a
            grid_res: a
            z_plane: a
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
        to the current component's .last_point
        """
        components_ordered = []
        remaining = self.components.copy()
        print(f"Starting with {len(remaining)} components")

        first_component_idx = min(
            range(len(self.components)), key=lambda i: self.components[i].first_point[0]
        )
        first_component = self.components[first_component_idx]
        current_component = first_component
        ic(current_component.cntr)
        del remaining[first_component_idx]
        components_ordered.append(current_component)
        while remaining:
            print(f"Remaining: {[c.name for c in remaining]}")
            print(f"Ordered so far: {[c.name for c in components_ordered]}")
            distances = [
                (idx, np.sum((comp.first_point - current_component.last_point) ** 2))
                for idx, comp in enumerate(remaining)
            ]
            nearest_idx = min(distances, key=lambda x: x[1])[0]
            current_component = remaining[nearest_idx]
            del remaining[nearest_idx]
            components_ordered.append(current_component)

        # If first_point is below the midpoint, move leftware to acheive clockwise motion

        print(f"Final ordered: {[c.name for c in components_ordered]}")
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

        Notes:
            o3d visualization *must* go before matplot display to avoid QT thread-handling issue

            o3d visualization does not work on wlroots even with x-wayland. Use X11 environment instead
        """

        axes = gen.draw_axes()
        pcd = o3d.t.geometry.PointCloud(self.coords3d)
        pcd_legacy = o3d.geometry.PointCloud()
        pcd_legacy.points = o3d.utility.Vector3dVector(pcd.point.positions.numpy())

        lines = [[i, i + 1] for i in range(len(pcd_legacy.points) - 1)]
        line_set = o3d.geometry.LineSet()
        line_set.points = pcd_legacy.points
        line_set.lines = o3d.utility.Vector2iVector(lines)

        o3d.visualization.draw_geometries([pcd_legacy, line_set])

        plt.imshow(self.grid_path, cmap="gray")
        plt.show()


class Cloud:
    def __init__(self, clouds):
        self.clouds = clouds
        self.overall_cloud = self.get_overall_cloud()

    def get_overall_cloud(self):
        overall_cloud = []
        for key, value in clouds.items():
            overall_cloud.append(value)
        overall_cloud = np.concatenate(overall_cloud, axis=0)
        return overall_cloud

    def visualize(self):
        axes = gen.draw_axes()
        pcd = o3d.t.geometry.PointCloud(self.overall_cloud)
        pcd_legacy = o3d.geometry.PointCloud()
        pcd_legacy.points = o3d.utility.Vector3dVector(pcd.point.positions.numpy())
        ic(np.asarray(pcd_legacy.points[0:3]))
        o3d.visualization.draw_geometries([pcd_legacy])


if __name__ == "__main__":

    # Generated data
    DENSITY = 35
    NOISE_STD = 0.00
    clouds = {
        # "curved_walls": model.gen_curved_walls(DENSITY, NOISE_STD),
        # "planes": model.gen_planes(DENSITY, NOISE_STD),
        # "floors": model.gen_floor(DENSITY, NOISE_STD),
        "tbeams": model.gen_tbeams(DENSITY, NOISE_STD),
    }

    # Chosen parameters
    Z_PLANE = 5
    GRID_RES = 5
    TOLERANCE = 1

    # Collect clouds and create objects
    my_cloud = Cloud(clouds)

    component_groups = []
    for key, value in clouds.items():
        component_groups.append(
            contour.ComponentGroup(key, value, Z_PLANE, GRID_RES, TOLERANCE)
        )

    my_path = Path(component_groups, GRID_RES, Z_PLANE)
    ### Sort tagged point cloud into individual objects
    """
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
    ic(my_cloud.overall_cloud)
    ic(my_path.components_ordered[1].cntr)
    ic(my_path.components_ordered[1].cntr[0])
    ic(my_path.components_ordered[1].cntr[-1])
    """
    ic(my_path.components_ordered[1].first_point)
    ic(my_path.components_ordered[1].last_point)
    ic(my_path.components_ordered[1].cntr)
    # my_cloud.visualize()
    my_path.visualize()
