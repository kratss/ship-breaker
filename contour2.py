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


def get_contour(img):
    img = np.uint8(img)
    _, img = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY)
    cntrs, hrrchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    ic(cntrs[0])
    return cntrs


if __name__ == "__main__":
    import matplotlib.pyplot as plt
    import gen
    import extract_plane as ep

    img = gen_cloud()
    cntrs = get_contour(img)

    # Visualization. Not required
    w, h = img.shape
    viz = np.zeros((w, h))
    cntr = cntrs[0].reshape(-1, 2)
    for point in cntr:
        x, y = point[0], point[1]
        if 0 <= y < w and 0 <= x < h:
            viz[y, x] = 1
    cntr = cntrs[0].reshape(-1, 2)
    for point in cntr:
        x, y = point[0], point[1]
        if 0 <= y < w and 0 <= x < h:
            viz[y, x] = 1
    plt.figure(figsize=(8, 6))
    plt.imshow(viz, cmap="binary")
    plt.colorbar()
    plt.tight_layout()
    plt.show()
