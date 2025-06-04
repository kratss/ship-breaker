#!/usr/bin/env python
import cv2
import extract_plane as ep
import gen
from icecream import ic
import matplotlib.pyplot as plt
import model
import numpy as np
from skimage.morphology import skeletonize


class ComponentGroup:
    def __init__(self, name, cloud, plane, grid_res):
        self.name = name
        self.cloud = cloud
        self.plane = plane
        self.grid_res = grid_res
        self.grid_noisy = ep.cloud_to_grid(self.cloud, self.grid_res, self.plane)
        self.grid_denoised = self.denoise()
        self.grid_thinned = self.thin()
        self.grid = self.grid_thinned
        self.cntrs = self.get_contours(self.grid)

    def get_contours(self, img):
        if img.dtype == bool:
            img = img.astype(np.uint8) * 255
        else:
            img = np.uint8(img)
        filled = img.copy()
        cntrs_temp, _ = cv2.findContours(
            img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        cv2.drawContours(filled, cntrs_temp, -1, 255, -1)
        print("Drawing filled")
        img_cntrs, hrrchy = cv2.findContours(
            filled, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        return img_cntrs

    def denoise(self):
        return cv2.GaussianBlur(self.grid_noisy, (5, 5), 0)

    def thin(self):
        grid = self.grid_denoised
        grid = skeletonize(grid)
        return grid


class Component:
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
        Squeeze out extra dimension that cv.contours produces
        Ensure cntrs containing only one point are still 2D
        """
        cntr = np.squeeze(cntr)
        if cntr.ndim < 2:
            cntr = cntr.reshape(1, -1)  # Ensure arrays are 2D
        # Remove duplicate points, keeping first occurrence
        unique_points, indices = np.unique(cntr, axis=0, return_index=True)
        # Sort by original indices to maintain order
        sorted_indices = np.sort(indices)
        return cntr[sorted_indices]

    def get_contours(self):
        grid = self.grid
        grid = np.uint8(self.grid)
        _, grid = cv2.threshold(grid, 0, 255, cv2.THRESH_BINARY)
        cntrs, hrrchy = cv2.findContours(grid, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        return cntrs

    def get_first_point(self):
        cntr = self.cntr.squeeze()  # simplify ugly cv2 formatting
        if cntr.ndim == 1:
            cntr = cntr.reshape(1, -1)  # ensure array is 2D
        max_y = np.max(cntr[:, 1])
        max_y_points = cntr[cntr[:, 1] == max_y]
        min_x_idx = np.argmin(max_y_points[:, 0])
        return max_y_points[min_x_idx]

    def get_last_point(self):
        cntr = self.cntr.squeeze()  # simplify ugly cv2 formatting
        if cntr.ndim == 1:
            cntr = cntr.reshape(1, -1)  # ensure array is 2D
        max_y = np.max(cntr[:, 1])
        max_y_points = cntr[cntr[:, 1] == max_y]
        max_x_idx = np.argmax(max_y_points[:, 0])
        return max_y_points[max_x_idx]

    def visualize(self):
        ic("Visualizing...")
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
    Z_PLANE = 1
    GRID_RES = 5
    curved_walls = ComponentGroup(
        "curved_walls", model.gen_planes(DENSITY, NOISE_STD), Z_PLANE, GRID_RES
    )
    planes = ComponentGroup(
        "planes", model.gen_planes(DENSITY, NOISE_STD), Z_PLANE, GRID_RES
    )
    tbeams = ComponentGroup(
        "tbeams", model.gen_tbeams(NOISE_STD, DENSITY), Z_PLANE, GRID_RES
    )

    # List of NON-EMPTY component groups
    component_groups = [tbeams]

    # Sort tagged point cloud into individual objects
    ## Find the size of the image grid
    max_grid = [0, 0]
    for group in component_groups:
        if group.grid.shape[0] > max_grid[0]:
            max_grid[0] = group.grid.shape[0]
        if group.grid.shape[1] > max_grid[1]:
            max_grid[1] = group.grid.shape[1]
    ### TEMP ###
    """
    grid_slice = np.zeros(max_grid)
    ic(tbeams.cntrs)
    ic("noisy")
    plt.imshow(tbeams.grid_denoised)
    plt.show()
    ic("thinned")
    plt.imshow(tbeams.grid_thinned, cmap="gray")
    plt.show()
    ic("====")
        """
    ### #### ###

    components = []
    for group in component_groups:
        for i, cntr in enumerate(group.cntrs):
            components.append(Component(group.name[:-1] + str(i), cntr, max_grid))

    current = components[0]
    old = []  # components already cut
    grid_cntr = np.zeros(max_grid)
    for i, current in enumerate(components):
        if np.any(current.cntr[:, 0] >= max_grid[1]) or np.any(
            current.cntr[:, 1] >= max_grid[0]
        ):
            ic("Out of bounds!", current.cntr.max(axis=0), max)
        grid_cntr[current.cntr[:, 1], current.cntr[:, 0]] = 1  # TODO: Very broken
        # exclude old items from list of candidates
        candidates = [item for j, item in enumerate(components) if j not in old]
        if not candidates:  # check if candidates is empty
            break
        next = min(
            candidates,
            key=lambda x: (x.first_point[0] - current.last_point[0]) ** 2
            + (x.first_point[1] - current.last_point[1]) ** 2,
        )

    component_instances = [
        name for name, obj in globals().items() if isinstance(obj, Component)
    ]
    print(component_instances)
    ic(components[0].cntr.shape)
    grid_slice = np.zeros(max_grid)
    grid_slice[components[0].cntr[:, 1], components[0].cntr[:, 0]] = 1
    plt.imshow(grid_slice, cmap="gray")
    # plt.imshow(tbeams.grid_denoised, cmap="gray")
    # plt.imshow(grid_cntrs, cmap="gray")
    plt.show()
