#!/usr/bin/env python
"""
Find the path the robot must follow
"""
import cv2
import extract_plane as ep
import gen
from icecream import ic
import matplotlib.pyplot as plt
import model
import numpy as np
from skimage.morphology import skeletonize


class ComponentGroup:
    """
    Container class that holds multiple members of a single component type.


    This class groups together all components of a particular type (i.g. an object named tbeams that holds all tbeams from the scene). It contains both the three-dimensional point cloud and two dimensional slice, and the key points of each component

    Attributes:
        name (str): the component type. If the type is known, type-specific code will be used to provide the key points from the primtive. Else, a fallback function will be used
        cloud (3xn numpy array): point cloud of all constituent components
        plane (float): desired cutting plane
        grid_density (int): number of squares in image space per square unit in point cloud space

    """

    def __init__(self, name, cloud, plane, grid_density, tolerance):
        self.name = name
        self.cloud = cloud
        self.plane = plane
        self.tolerance = tolerance
        self.grid_density = grid_density
        self.grid = self.process_grid()
        self.cntrs = self.get_contours(self.grid)

    def get_contours(self, img):
        if img.dtype == bool:
            img = img.astype(np.uint8) * 255
        else:
            img = np.uint8(img)

        if self.name == "planes":
            """KNN Approach. Find end point with only one neighbor"""
            skeleton = cv2.ximgproc.thinning(img)
            return skeleton

        img_cntrs, hrrchy = cv2.findContours(
            img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        return img_cntrs

    def process_grid(self):
        grid = ep.cloud_to_grid(
            self.cloud, self.grid_density, self.plane, self.tolerance
        )
        grid = self.denoise(grid)
        grid = self.thin(grid)
        return grid

    def denoise(self, grid):
        return cv2.GaussianBlur(grid, (5, 5), 0)

    def thin(self, grid):
        grid = skeletonize(grid)
        return grid


class Component:
    """
    An individual structural component. E.g. I-beam
    """

    def __init__(self, name, cntr, grid_size):
        self.name = name
        self.grid_size = grid_size
        self.cntr = self.fix_contour(cntr)
        self.first_pt = self.get_first_pt()
        self.last_pt = self.get_last_pt()

    def get_info(self):
        return f"Component is a {self.name}"

    def get_first_pt(self):
        return self.cntr[0]

    def get_last_pt(self):
        return self.cntr[-1]

    def fix_contour(self, cntr):
        """
        Squeeze out the extra dimension that cv.contours produces.
        Ensure cntrs containing only one point are still 2D
        """
        cntr = np.squeeze(cntr)
        if cntr.ndim < 2:
            cntr = cntr.reshape(1, -1)  # Ensure arrays are 2D

        # Remove duplicate points, keeping first occurrence
        unique_points, indices = np.unique(cntr, axis=0, return_index=True)

        return cntr

    def visualize(self):
        print("Visualizing...")
        cntr = self.cntr.squeeze()
        grid_size = self.grid_size
        w = grid_size[0]
        h = grid_size[1]
        viz = np.zeros((w, h))
        for point in cntr:
            x, y = point[0], point[1]
            viz[y, x] = 1
        plt.figure(figsize=(8, 6))
        plt.imshow(viz, cmap="binary")
        plt.colorbar()
        plt.tight_layout()
        plt.show()


if __name__ == "__main__":

    # Generated data
    DENSITY = 35
    NOISE_STD = 0.05
    Z_PLANE = 5
    grid_density = 5
    TOLERANCE = 1
    curved_walls = ComponentGroup(
        "curved_walls",
        model.gen_curved_walls(DENSITY, NOISE_STD),
        Z_PLANE,
        grid_density,
        TOLERANCE,
    )
    planes = ComponentGroup(
        "planes", model.gen_planes(DENSITY, NOISE_STD), Z_PLANE, grid_density, TOLERANCE
    )
    tbeams = ComponentGroup(
        "tbeams", model.gen_tbeams(DENSITY, NOISE_STD), Z_PLANE, grid_density, TOLERANCE
    )

    # List of NON-EMPTY component groups
    component_groups = [tbeams, planes, curved_walls]

    ### Sort tagged point cloud into individual objects
    # Find the size of the image grid
    max_grid = [0, 0]
    for group in component_groups:
        if group.grid.shape[0] > max_grid[0]:
            max_grid[0] = group.grid.shape[0]
        if group.grid.shape[1] > max_grid[1]:
            max_grid[1] = group.grid.shape[1]
    components = []
    for group in component_groups:
        for i, cntr in enumerate(group.cntrs):
            components.append(Component(group.name[:-1] + str(i), cntr, max_grid))

    ### Choose the order of the components
    components_ordered = []
    remaining = components.copy()
    current_component = remaining[0]
    del remaining[0]
    components_ordered.append(current_component)
    while remaining:
        distances = [
            (idx, np.sum((comp.first_pt - current_component.last_pt) ** 2))
            for idx, comp in enumerate(remaining)
        ]
        nearest_idx = min(distances, key=lambda x: x[1])[0]
        current_component = remaining[nearest_idx]
        del remaining[nearest_idx]
        components_ordered.append(current_component)

    # Collect all coordinates in order
    coords2D = np.concatenate([comp.cntr for comp in components_ordered], axis=0)
    coords3D = ep.get_3d(coords2D, grid_density, Z_PLANE)

    ### Visualize and print info
    component_instances = [
        name for name, obj in globals().items() if isinstance(obj, Component)
    ]
    grid_slice = np.zeros(max_grid)
    for comp in components_ordered:
        grid_slice[comp.cntr[:, 1], comp.cntr[:, 0]] = 1
    plt.imshow(grid_slice, cmap="gray")
    plt.show()
