#!/usr/bin/env python
import cv2
import extract_plane as ep
from icecream import ic
import matplotlib
import model
import numpy as np
from skimage.morphology import skeletonize


def get_contours(img):
    img = np.uint8(img)
    _, img = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY)
    img_cntrs, hrrchy = cv2.findContours(
        img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    return img_cntrs


def thinning(img):
    """
    Thin out thick lines
    """
    skeleton = skeletonize(img > 0).astype(np.uint8) * 255
    return skeleton


def denoise(img):
    """
    Simple noise removal images.
    """
    if img.dtype != np.uint8:
        img = (img * 255).astype(np.uint8)
    # Median filter is excellent for salt-and-pepper noise
    denoised = cv2.medianBlur(img, 3)
    # Small opening to remove remaining specs
    kernel = np.ones((2, 2), np.uint8)
    cleaned = cv2.morphologyEx(denoised, cv2.MORPH_OPEN, kernel)

    return cleaned


class Component:
    def __init__(self, id, cntr, grid_size):
        self.id = id
        self.cntr = cntr  # 2D slice represented as an image
        self.grid_size = grid_size
        self.first_point = self.get_first_point()
        self.last_point = self.get_last_point()

    def get_info(self):
        return f"Component is a {self.name}"

    def get_contours(self):
        grid = self.grid
        grid = np.uint8(self.grid)
        _, grid = cv2.threshold(grid, 0, 255, cv2.THRESH_BINARY)
        cntrs, hrrchy = cv2.findContours(
            grid, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
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
    import matplotlib.pyplot as plt
    import gen
    import extract_plane as ep

    density = 55
    noise_std = 0.11

    cloud_curved_walls = model.gen_planes(density, noise_std=noise_std)
    cloud_ibeams = model.gen_planes(density, noise_std=noise_std)
    cloud_planes = model.gen_planes(density, noise_std=noise_std)
    cloud_tbeams = model.gen_planes(density, noise_std=noise_std)

    clouds = ["curved_walls":cloud_curved_walls, "ibeams":cloud_ibeams, "planes":cloud_planes, "tbeams":cloud_tbeams]

    grids[name]=["curved_walls":None, "ibeams":None, "planes":None, "tbeams":None]
    for name in clouds:
    grid_curved_walls = np.empty([])
    grid_ibeams = np.empty([])
    grid_planes = ep.cloud_to_grid(model.gen_planes(density, noise_std=noise_std))
    grid_tbeams = ep.cloud_to_grid(model.gen_tbeams(density, noise_std=noise_std))
    grids = {
        "curved_walls": grid_curved_walls,
        "ibeams": grid_ibeams,
        "planes": grid_planes,
        "tbeams": grid_tbeams,
    }
    # get size of overall grid

    if grid_tbeams.size > 1:
        grid_size = np.array((grid_tbeams.shape))
        cntrs_tbeams = get_contours(denoise(grid_tbeams))
    if grid_planes.size > 1:
        cntrs_planes = get_contours(grid_planes)
        grid_size = np.maximum(grid_size, grid_planes.shape)
    if grid_curved_walls.size > 1:
        cntrs_curved_walls = get_contours(grid_curved_walls)
        grid_size = np.maximum(grid_size, grid_curved_walls.shape)
    if grid_ibeams.size > 1:
        cntrs_ibeams = get_contours(grid_ibeams)
        grid_size = np.maximum(grid_size, grid_ibeams.shape)

    cntrs = []
    for name in grids:
        if grids[name].size > 1:
            ic(name)
            ic(grids[name].size)

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    # Original image
    axes[0].imshow(grid_tbeams, cmap="gray")
    axes[0].set_title("Original Binary Image")
    axes[0].axis("off")

    # Denoised image
    denoised_img = denoise(grid_tbeams)
    denoised_img = thinning(denoised_img)
    axes[1].imshow(denoised_img, cmap="gray")
    axes[1].set_title("Denoised Image")
    axes[1].axis("off")

    plt.tight_layout()
    plt.show()

    tbeams = []
    for i in range(len(cntrs_tbeams)):
        tbeams.append(Component(f"tbeam{i}", cntrs_tbeams[i], grid_size))
    planes = []
    if grid_planes.size > 1:
        for i in range(len(cntrs_planes)):
            planes.append(Component(f"planes{i}", cntrs_planes[i], grid_size))

    curved_walls = []
    if grid_curved_walls.size > 1:
        for i in range(len(cntrs_curved_walls)):
            curved_walls.append(
                Component(f"curved_wall{i}", cntrs_curved_walls[i], grid_size)
            )

    ibeams = []
    if grid_ibeams.size > 1:
        for i in range(len(cntrs_ibeams)):
            ibeams.append(Component(f"ibeam{i}", cntrs_ibeams[i], grid_size))

    # make sure not to include the current component
    components = tbeams + planes[1:] + ibeams + curved_walls
    tbeams[0].visualize()
