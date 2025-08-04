#!/usr/bin/env python
"""
Apply primitives to the point cloud
"""
import cv2
from . import extract_plane as ep
from . import gen
from icecream import ic
import matplotlib.pyplot as plt
from . import model
import numpy as np
import pandas as pd
from skimage.morphology import skeletonize


class ComponentGroup:
    """
    Container class that holds multiple members of a single component type.


    This class groups together all components of a particular type (i.g. an object named tbeams that holds all tbeams from the scene). It contains both the three-dimensional point cloud and two dimensional slice, and the key points of each component

    Attributes:
        name (str): the component type. If the type is known, type-specific code will be used to provide the key points from the primtive. Else, a fallback function will be used
        cloud (numpy.ndarray): point cloud of shape (n,3) of all constituent components
        plane (float): desired cutting plane
        tolerance (float): amount of deviation from the cutting plane permitted
        grid_density (int): number of squares in image space per square unit in point cloud space
        grid_dim (numpy.ndarray): an array of size (2,2) describing the location of the bottom-left and top-right corners of the grid in grid space
        grid (numpy.ndarray): a (2,n) array giving a binary image of the cross-section of the component group. Values are 1 or 0.
        cntrs (list): list of lists of key points that describe the entire component_group
    """

    def __init__(self, name, cloud, plane, grid_density, tolerance, grid_dim):
        self.name = name
        print("Initializing component group", self.name)
        self.cloud = cloud
        self.plane = plane
        self.tolerance = tolerance
        self.grid_density = grid_density
        self.grid_dim = grid_dim
        self.grid = self.process_grid()
        self.cntrs = self.get_contours(self.grid)

    def get_contours(self, img):
        """
        Apply the primitives to the binary image to obtain the key points

        Args:
            img (np.ndarray): An array of size (n,3) giving a binary image of the cross-section of the component group. Values are 1 or 0.

        Returns:
            list: A list of lists, with each list containing the ordered set of key points for a specific component
        """
        print("Obtaining primitives for", self.name)
        if img.dtype == bool:
            img = img.astype(np.uint8) * 255
        else:
            img = np.uint8(img)

        if self.name == "planes":
            """KNN Approach. Find end point with only one neighbor"""
            skeleton = cv2.ximgproc.thinning(img)
            max = 0
            extremes = (0, 1)
            for i in range(len(skeleton)):
                for j in range(i + 1, len(skeleton)):
                    dist = np.linalg.norm(skeleton[i] - skeleton[j])
                    if dist > max:
                        max = dist
                        extremes = (i, j)
            return skeleton[list(extremes)]
        if self.name == "tbeams":
            """
            Find the points with the greatest y value, and move them
            to the top and bottom of the array. This trick ensures the
            path has navigable 45 degree angles.

            Without this, the contour will have a -45 degree angle
            """
            img_cntrs, hrrchy = cv2.findContours(
                img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            img_cntrs = list(img_cntrs)  # make mutable
            for i in range(len(img_cntrs)):
                cntr = img_cntrs[i].reshape(-1, 2)
                ic("tbeam")
                ic(cntr.shape)
                if cntr.shape[0] < 6:
                    print(
                        "Warning: T-beams are possibly being parsed incorrectly. Adjust the density of the grid or point cloud"
                    )
                cntr = pd.DataFrame(cntr).drop_duplicates().values
                ic(cntr.shape)
                median_x = np.median(cntr[:, 0])
                """
                left_side_idx = np.where(cntr[:, 0] < median_x)[0]
                max_y_idx = left_side_idx[np.argmin(cntr[left_side_idx, 1])]
                cntr = np.vstack([cntr[max_y_idx], np.delete(cntr, max_y_idx, axis=0)])

                right_side_idx = np.where(cntr[:, 0] > median_x)[0]
                max_y_idx = left_side_idx[np.argmin(cntr[right_side_idx, 1])]
                cntr = np.vstack([cntr[max_y_idx], np.delete(cntr, max_y_idx, axis=0)])
                """
                cntr[0, 0] = cntr[0, 0] - 2 * abs(cntr[0, 0] - cntr[1, 0])
                cntr = cntr.reshape(-1, 1, 2)
                img_cntrs[i] = cntr
            return img_cntrs
        if self.name == "curved_walls":
            img_cntrs, hrrchy = cv2.findContours(
                img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            # unique_points, indices = np.unique(img_cntrs, axis=0, return_index=True)
            # img_cntrs = unique_points
            return img_cntrs

        img_cntrs, hrrchy = cv2.findContours(
            img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        return img_cntrs

    def process_grid(self):
        """
        Obtain the binary image of the cross section from the point cloud

        Returns:
            np.ndarray: An array of size (n,3) giving a binary image of the cross-section of the component group. Values are 1 or 0.
        """
        grid = ep.cloud_to_grid(
            self.cloud,
            self.grid_density,
            self.plane,
            self.tolerance,
            self.grid_dim,
        )
        grid = self.denoise(grid)
        grid = self.thin(grid)
        return grid

    def denoise(self, grid):
        """
        Removes noise

        Returns:
            np.ndarray: An array of size (n,3) giving a binary image of the cross-section of the component group. Values are 1 or 0.
        """
        return cv2.GaussianBlur(grid, (5, 5), 0)

    def thin(self, grid):
        """
        Thins lines with mutliple pixels in width

        Returns:
            np.ndarray: An array of size (n,3) giving a binary image of the cross-section of the component group. Values are 1 or 0.
        """
        grid = skeletonize(grid)
        return grid


class Component:
    """
    An individual structural component. E.g. I-beam

    Attributes:
        name (str): name of the specific component at a specific location in space. E.g. plane0, plane1, plane2 ...
        grid_size (np.ndarray): length and width of the image grid. Size is 2
        cntr (np.ndarray): ordered list of (x,y) points the robot must follow to cut the component
        first_pt (np.ndarray): the first pair of (x,y) coordinates in the primitive path. Improves code legibility
        last_pt (np.ndarray): the last pair of (x,y) coordinates in the primitive path. Improves code legibility
    """

    def __init__(self, name, cntr, grid_size):
        self.name = name
        self.grid_size = grid_size
        self.cntr = self.fix_contour(cntr)
        self.first_pt = self.get_first_pt()
        self.last_pt = self.get_last_pt()

    def get_info(self):
        """
        Prints useful info about the component
        """
        print(f"Component is {self.name}")
        ic(self.cntr)
        ic(self.first_pt)
        ic(self.last_pt)
        return

    def get_first_pt(self):
        """Returns the first point in the primitive path

        Returns:
            np.ndarray: x,y coordinate
        """

        if self.name.startswith("curved_wall"):
            max = 0
            extremes = (0, 1)
            for i in range(len(self.cntr)):
                for j in range(i + 1, len(self.cntr)):
                    dist = np.linalg.norm(self.cntr[i] - self.cntr[j])
                    if dist > max:
                        max = dist
                        extremes = (i, j)
            return self.cntr[list(extremes)][0]
        return self.cntr[0]

    def get_last_pt(self):
        """Returns the last point in the primitive path

        Returns:
            np.ndarray: x,y coordinate
        """
        if self.name.startswith("curved_wall"):
            max = 0
            extremes = (0, 1)
            for i in range(len(self.cntr)):
                for j in range(i + 1, len(self.cntr)):
                    dist = np.linalg.norm(self.cntr[i] - self.cntr[j])
                    if dist > max:
                        max = dist
                        extremes = (i, j)
            return self.cntr[list(extremes)][1]
        return self.cntr[-1]

    def fix_contour(self, cntr):
        """
        Squeeze out the extra dimension that cv.contours produces.
        Ensure cntrs containing only one point are still 2D

        Args:
            cntr (np.ndarray): Ordered list of (x,y) points

        Returns
            cntr (np.ndarray): Ordered list of (x,y) points
        """
        cntr = np.squeeze(cntr)
        if cntr.ndim < 2:
            cntr = cntr.reshape(1, -1)  # Ensure arrays are 2D

        # Remove duplicate points, keep first occurrence
        cntr = pd.DataFrame(cntr).drop_duplicates().values
        # unique_points, indices = np.unique(cntr, axis=0, return_index=True)
        # cntr = unique_points
        return cntr

    def visualize(self):
        """
        Prints 2D plot of the component path.

        Note: Wayland only
        """
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
    GRID_DENSITY = 5
    TOLERANCE = 1
    GRID_DIM = np.array([[0, 0], [300, 300]])
    curved_walls = ComponentGroup(
        "curved_walls",
        model.gen_curved_walls(DENSITY, NOISE_STD),
        Z_PLANE,
        GRID_DENSITY,
        TOLERANCE,
        GRID_DIM,
    )
    planes = ComponentGroup(
        "planes",
        model.gen_planes(DENSITY, NOISE_STD),
        Z_PLANE,
        GRID_DENSITY,
        TOLERANCE,
        GRID_DIM,
    )
    tbeams = ComponentGroup(
        "tbeams",
        model.gen_tbeams(DENSITY, NOISE_STD),
        Z_PLANE,
        GRID_DENSITY,
        TOLERANCE,
        GRID_DIM,
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
    coords3D = ep.get_3d(coords2D, GRID_DENSITY, Z_PLANE)

    ### Visualize and print info
    component_instances = [
        name for name, obj in globals().items() if isinstance(obj, Component)
    ]
    grid_slice = np.zeros(max_grid)
    for comp in components_ordered:
        grid_slice[comp.cntr[:, 1], comp.cntr[:, 0]] = 1
    plt.imshow(grid_slice, cmap="gray")
    plt.show()
