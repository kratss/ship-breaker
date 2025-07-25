#!/usr/bin/env python
import numpy as np


def hough(image):
    # image means EDGE image
    rho_max = int(np.ceil(np.sqrt(image.shape[0] ** 2 + image.shape[1] ** 2)))
    rho_min = -rho_max
    theta = np.linspace(-np.pi, np.pi, 180)
    rho = np.linspace(rho_min, rho_max, int(rho_max) * 2)
    accumulator = np.zeros([rho.shape[0], theta.shape[0]])
    for x, y in zip(*np.where(image == 1)):
        # remember this asterisk trick
        if image[x, y] == 1:
            for t_idx in range(theta.shape[0]):
                r = x * np.cos(theta[t_idx]) + y * np.sin(theta[t_idx])
                r_idx = np.argmin(np.abs(r - rho))
                accumulator[r_idx, t_idx] += 1
    plt.figure(figsize=(10, 8))
    plt.imshow(accumulator, cmap="viridis", norm=None)
    plt.colorbar()
    plt.title("Hough Space - Raw Values")
    plt.show()
    return accumulator


def threshold(hough_space):
    print("sadg")


def gen_cloud():
    density = 15
    ship = np.concatenate(
        [
            gen.tbeam(origin=[0, 30, 30], length=90, density=density),
            gen.tbeam(origin=[0, 80, 30], length=90, density=density),
        ]
    )
    slice = ep.extract_plane(ship)
    grid = ep.voxelize(slice)
    return grid


def contour(image):
    # Error checks
    if image.dtype != np.uint8:  # Convert to uint8 if not already
        image = image.astype(np.uint8)
    if image.max() == 1:  # Ensure values are 0 or 255
        image = image * 255

    contours, hierarchy = cv2.findContours(
        image,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )

    image_contoured = np.zeros_like(image)
    image_contoured = cv2.cvtColor(image_contoured, cv2.COLOR_GRAY2RGB)
    cv2.drawContours(image_contoured, contours, -1, (0, 0, 255), 1)

    total_points = sum(len(contour) for contour in contours)
    all_points = np.zeros((total_points, 2), dtype=np.float32)
    idx = 0
    for contour in contours:
        points = contour.reshape(-1, 2)
        n_points = len(points)
        all_points[idx : idx + n_points] = points
        idx += n_points

    return all_points, image_contoured


def prob_hough(image):
    # preprocess image
    if image is None or image.size == 0:
        print("Error: Image is empty or not loaded properly")
        return  # Exit the function # Convert to uint8 if not already (required by OpenCV)
    if image.dtype != np.uint8:
        image = image.astype(np.uint8)
    if image.max() == 1:  # change image values from 1 and 0 to 255 and 0
        image = image * 255

    # Probabilistic Hough transform
    lines = cv2.HoughLinesP(
        image,
        rho=1,  # Distance resolution in pixels
        theta=np.pi / 180,  # Angle resolution in radians
        threshold=9,  # Minimum number of votes
        minLineLength=5,  # Minimum line length
        maxLineGap=58,  # Maximum allowed gap between line segments
    )

    # Create a color image to draw lines on
    # Convert grayscale to RGB if needed
    if len(image.shape) == 2:
        display_img = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
    else:
        display_img = image.copy()

    # Draw the detected line segments
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(display_img, (x1, y1), (x2, y2), (0, 0, 255), 1)

    print("display_img:\n", display_img)
    # Display the result
    plt.figure(figsize=(10, 8))
    plt.imshow(cv2.cvtColor(display_img, cv2.COLOR_BGR2RGB))
    plt.title(f"{len(lines) if lines is not None else 0} Line Segments Detected")
    plt.axis("off")
    plt.show()


if __name__ == "__main__":
    import cv2
    import matplotlib.pyplot as plt
    import gen
    import extract_plane as ep

    image = gen_cloud()
    prob_hough(image)
