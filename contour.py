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
    def __init__(self, name, cloud, plane, grid_res, tolerance):
        self.name = name
        self.cloud = cloud
        self.plane = plane
        self.tolerance = tolerance
        self.grid_res = grid_res
        self.grid_noisy = ep.cloud_to_grid(
            self.cloud, self.grid_res, self.plane, self.tolerance
        )
        self.grid_denoised = self.denoise()
        self.grid_thinned = self.thin()
        self.grid = self.grid_thinned
        self.cntrs = self.get_contours(self.grid)

    def get_contours(self, img):
        if img.dtype == bool:
            img = img.astype(np.uint8) * 255
        else:
            img = np.uint8(img)
        img_cntrs, hrrchy = cv2.findContours(
            img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        return img_cntrs

    def denoise(self):
        return cv2.GaussianBlur(self.grid_noisy, (5, 5), 0)

    def thin(self):
        grid = self.grid_denoised
        grid = skeletonize(grid)
        return grid


class Component:
    """
    An individual structural component. E.g. I-beam
    """

    def __init__(self, name, cntr, grid_size):
        self.name = name
        self.cntr_fixed = self.fix_contour(cntr)
        self.cntr = self.cntr_fixed
        self.grid_size = grid_size
        self.first_point = self.get_first_point()
        self.last_point = self.get_last_point()

    def get_info(self):
        return f"Component is a {self.name}"

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
        # Sort by original indices to maintain order
        sorted_indices = np.sort(indices)
        cntr = cntr[sorted_indices]
        if cntr[0][0] > cntr[-1][0]:
            cntr = np.flip(cntr, axis=0)  # Ensure the cntr runs left to right
        return cntr[sorted_indices]

    def get_first_point(self):
        return self.cntr[0]
        """
        cntr = self.cntr.squeeze()  # simplify ugly cv2 formatting
        if cntr.ndim == 1:
            cntr = cntr.reshape(1, -1)  # ensure array is 2D
        max_y = np.max(cntr[:, 1])
        max_y_points = cntr[cntr[:, 1] == max_y]
        min_x_idx = np.argmin(max_y_points[:, 0])
        return max_y_points[min_x_idx]
        """

    def get_last_point(self):
        return self.cntr[-1]
        """
        cntr = self.cntr.squeeze()  # simplify ugly cv2 formatting
        if cntr.ndim == 1:
            cntr = cntr.reshape(1, -1)  # ensure array is 2D
        max_y = np.max(cntr[:, 1])
        max_y_points = cntr[cntr[:, 1] == max_y]
        max_x_idx = np.argmax(max_y_points[:, 0])
        return max_y_points[max_x_idx]
        """

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

    def get_closest(self, components):
        # Searches for the component closest to the current component
        dists = [
            (
                component,
                (
                    (self.last_point[0] - component.first_point[0]) ** 2
                    + (self.last_point[1] - component.first_point[1]) ** 2
                )
                ** 0.5,
            )
            for component in components
        ]
        return min(dists, key=lambda x: x[1])[0]


if __name__ == "__main__":

    # Generated data
    DENSITY = 35
    NOISE_STD = 0.05
    Z_PLANE = 5
    GRID_RES = 5
    TOLERANCE = 1
    curved_walls = ComponentGroup(
        "curved_walls",
        model.gen_curved_walls(DENSITY, NOISE_STD),
        Z_PLANE,
        GRID_RES,
        TOLERANCE,
    )
    planes = ComponentGroup(
        "planes", model.gen_planes(DENSITY, NOISE_STD), Z_PLANE, GRID_RES, TOLERANCE
    )
    tbeams = ComponentGroup(
        "tbeams", model.gen_tbeams(DENSITY, NOISE_STD), Z_PLANE, GRID_RES, TOLERANCE
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
    # ic(f"Starting with {len(remaining)} components")
    current_component = remaining[0]
    del remaining[0]
    components_ordered.append(current_component)
    while remaining:
        # ic(f"Remaining: {[c.name for c in remaining]}")
        # ic(f"Ordered so far: {[c.name for c in components_ordered]}")
        distances = [
            (idx, np.sum((comp.first_point - current_component.last_point) ** 2))
            for idx, comp in enumerate(remaining)
        ]
        nearest_idx = min(distances, key=lambda x: x[1])[0]
        current_component = remaining[nearest_idx]
        del remaining[nearest_idx]
        components_ordered.append(current_component)

    # ic(f"Final ordered: {[c.name for c in components_ordered]}")

    # Collect all coordinates in order
    coords2D = np.concatenate([comp.cntr for comp in components_ordered], axis=0)
    coords3D = ep.get_3d(coords2D, GRID_RES, Z_PLANE)
    ### Visualize and print info
    component_instances = [
        name for name, obj in globals().items() if isinstance(obj, Component)
    ]
    grid_slice = np.zeros(max_grid)
    for comp in components_ordered:
        # ic(comp.name)
        grid_slice[comp.cntr[:, 1], comp.cntr[:, 0]] = 1
    #    grid_slice[components[1].cntr[:, 1], components[1].cntr[:, 0]] = 1
    plt.imshow(grid_slice, cmap="gray")
    plt.show()
