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
    density = 5
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

    plt.figure(figsize=(8, 10))

    # For each contour, connect the points
    for i, contour in enumerate(contours_simplified):
        # Extract x and y coordinates
        x = contour[:, 0]
        y = contour[:, 1]

        # Add first point at the end to close the shape
        x = np.append(x, x[0])
        y = np.append(y, y[0])

        # Plot with different colors for each contour
        color = "blue" if i == 0 else "red"
        plt.plot(x, y, "-o", color=color, label=f"Contour {i+1}")

        plt.grid(True, linestyle="--", alpha=0.7)
        plt.title("Connected Contour Points")
        plt.legend()
        plt.gca().invert_yaxis()  # Invert y-axis to match image coordinates
        plt.show()
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


def prob_hough(binary_img):
    if binary_img is None or binary_img.size == 0:
        print("Error: Image is empty or not loaded properly")
        return  # Exit the function # Convert to uint8 if not already (required by OpenCV)
    if binary_img.dtype != np.uint8:
        binary_img = binary_img.astype(np.uint8)

    # If your binary image has values other than 0 and 255, normalize it
    if binary_img.max() == 1:
        binary_img = binary_img * 255

    # Apply Probabilistic Hough Transform
    lines = cv2.HoughLinesP(
        binary_img,
        rho=1,  # Distance resolution in pixels
        theta=np.pi / 180,  # Angle resolution in radians
        threshold=9,  # Minimum number of votes
        minLineLength=5,  # Minimum line length
        maxLineGap=28,  # Maximum allowed gap between line segments
    )

    # Create a color image to draw lines on
    # Convert grayscale to RGB if needed
    if len(binary_img.shape) == 2:
        display_img = cv2.cvtColor(binary_img, cv2.COLOR_GRAY2RGB)
    else:
        display_img = binary_img.copy()

    # Draw the detected line segments
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(display_img, (x1, y1), (x2, y2), (0, 0, 255), 1)

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
    contours, out_image = contour(image)
    lines_v = np.zeros([1, 2])
    lines_h = np.zeros([1, 2])

    for i in range(np.shape(contours)[0] - 1):
        slope = (contours[i + 1, 1] - contours[i, 1]) / (
            contours[i + 1, 0] - contours[i, 0] + 0.0001
        )
        print("slope: \n", slope)
        slope_angle = np.arctan(slope)
        if slope_angle < np.pi / 4:
            lines_v = np.vstack((contours[i], contours[i + 1], lines_v))
        else:
            lines_h = np.vstack((contours[i], contours[i + 1], lines_h))

    print("vertical lines:\n", lines_v, "\nhorizontal lines:\n", lines_h)
    print("contours are:\n", contours)

    top_idx = np.argmax(contours[:, 1])
    lines_h = np.delete(lines_h, top_idx, axis=0)

    blank = np.zeros((50, 50, 3))
    plt.imshow(cv2.cvtColor(blank, contours, cv2.COLOR_BGR2RGB))
    plt.show()
