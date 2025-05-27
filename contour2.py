#!/usr/bin/env python
import numpy as np
import matplotlib
from icecream import ic
import cv2


def gen_cloud():
    density = 10
    ship = np.concatenate(
        [
            gen.plane(),
            gen.tbeam(
                origin=[0, 30, 30],
                length=90,
                density=density,
                skip=[
                    False,
                    False,
                    False,
                    False,
                    False,
                    False,
                    False,
                    True,
                    True,
                    True,
                    False,
                    False,
                ],
            ),
            gen.tbeam(origin=[0, 80, 30], length=90, density=density),
        ]
    )
    ship = np.concatenate(
        (
            gen.plane(origin=[5, 5, 5], length=10, width=2, density=density),
            gen.plane(
                origin=[5, 5, 5], length=10, width=2, roll=np.pi / 2, density=density
            ),
            gen.plane(
                origin=[5, 5, 5], length=10, width=2, pitch=np.pi / 2, density=density
            ),
            gen.plane(origin=[0, 10, 30], length=110, width=90, density=density),
            gen.tbeam(origin=[0, 30, 30], length=90, density=density),
            gen.tbeam(origin=[0, 60, 30], length=90, density=density),
            gen.tbeam(origin=[0, 90, 30], length=90, density=density),
            gen.curved_wall(
                origin=[110, 5, 0],
                length=110,
                height=30,
                yaw=np.pi / 2,
            ),
            gen.bulb_flat(origin=[0, 10, 30], roll=np.pi / 2, yaw=np.pi / 2),
            gen.plane(origin=[0, 0, 0], length=110, width=90, density=density),
            gen.ibeam(origin=[20, 20, 5], roll=np.pi / 2),
        )
    )
    slice = ep.extract_plane(ship)
    grid = ep.voxelize(slice)
    return grid


def gen_cloud2():
    density = 10
    ship = np.concatenate(
        [
            gen.plane(origin=[0, 10, 30], length=10, width=9, density=density),
            gen.plane(origin=[0, 10, 31], length=10, width=9, density=density),
        ]
    )
    slice = ep.extract_plane(ship)
    grid = ep.voxelize(slice)
    return grid


def gen_cloud3():
    density = 10
    ship = np.concatenate(
        [
            gen.tbeam(origin=[0, 520, 90], length=90, density=density),
            gen.tbeam(origin=[0, 600, 30], length=90, density=density),
            gen.tbeam(origin=[0, 900, 30], length=90, density=density),
        ]
    )
    slice = ep.extract_plane(ship)
    grid = ep.voxelize(slice)
    return grid


def gen_cloud4():
    density = 10
    ship = np.concatenate([gen.curved_wall(), gen.curved_wall(origin=[0, 30, 0])])
    slice = ep.extract_plane(ship)
    grid = ep.voxelize(slice)
    return grid


def gen_cloud5():
    density = 10
    ship = np.concatenate(
        [gen.ibeam(origin=[0, 40, 49]), gen.curved_wall(origin=[0, 30, 0])]
    )
    slice = ep.extract_plane(ship)
    grid = ep.voxelize(slice)
    return grid


def get_contours(img):
    img = np.uint8(img)
    _, img = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY)
    cntrs, hrrchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return cntrs


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
        cntr = self.cntr.squeeze()
        max_y = np.max(cntr[:, 1])
        max_y_points = cntr[cntr[:, 1] == max_y]
        min_x_idx = np.argmin(max_y_points[:, 0])
        return max_y_points[min_x_idx]

    def get_last_point(self):
        cntr = self.cntr.squeeze()
        max_y = np.max(cntr[:, 1])
        max_y_points = cntr[cntr[:, 1] == max_y]
        max_x_idx = np.argmax(max_y_points[:, 0])
        return max_y_points[max_x_idx]

    def visualize(self):
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

    grid_tbeams = gen_cloud3().T
    grid_planes = gen_cloud2().T
    grid_curved_walls = gen_cloud4().T
    grid_ibeams = gen_cloud5().T

    # get size of overal slice grid
    grid_size = np.array((grid_tbeams.shape))
    grid_size = np.maximum(grid_size, grid_planes.shape)
    grid_size = np.maximum(grid_size, grid_curved_walls.shape)
    grid_size = np.maximum(grid_size, grid_ibeams.shape)
    ic(grid_size)

    # get tagged list of point cloud structures and assign them to
    # appropriate category
    cntrs_tbeams = get_contours(grid_tbeams)
    cntrs_planes = get_contours(grid_planes)
    cntrs_ibeams = get_contours(grid_ibeams)
    ic(cntrs_ibeams)
    cntrs_curved_walls = get_contours(grid_curved_walls)
    #
    planes = []
    for i in range(len(cntrs_planes)):
        planes.append(Component(f"plane{i}", cntrs_planes[i], grid_size))

    tbeams = []
    for i in range(len(cntrs_tbeams)):
        tbeams.append(Component(f"tbeam{i}", cntrs_tbeams[i], grid_size))

    ibeams = []
    for i in range(len(cntrs_ibeams)):
        ibeams.append(Component(f"ibeam{i}", cntrs_ibeams[i], grid_size))

    curved_walls = []
    for i in range(len(cntrs_curved_walls)):
        curved_walls.append(
            Component(f"curved_wall{i}", cntrs_curved_walls[i], grid_size)
        )

    # make sure not to include the current component
    components = tbeams + planes[1:] + ibeams + curved_walls
    closest = planes[0].get_closest(components)
    ic(closest.id)
    ic(closest.first_point)
    ic(planes[0].last_point)
