import cv2 as cv
import numpy as np

# Detected the red color , then traingle and then the biggest traning to find the lower most corner.

# Load the cone_image
cone_image = cv.imread('resource/cone_x40cm.png')

# Convert BGR to HSV
hsv = cv.cvtColor(cone_image, cv.COLOR_BGR2HSV)

# Define range of red color in HSV
lower_red = np.array([0, 100, 100])
upper_red = np.array([10, 255, 255])

# Threshold the HSV cone_image to get only red colors
mask = cv.inRange(hsv, lower_red, upper_red)

# Find contours
contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

# Filter contours based on area and shape (triangle approximation)
min_contour_area = 100
triangle_contours = []
for contour in contours:
    area = cv.contourArea(contour)
    if area > min_contour_area:
        perimeter = cv.arcLength(contour, True)
        approx = cv.approxPolyDP(contour, 0.03 * perimeter, True)  # Adjust epsilon for more or less approximation
        if len(approx) == 3:  # Ensure it's a triangle
            triangle_contours.append(approx)

# Find the largest triangle
if triangle_contours:
    largest_triangle = max(triangle_contours, key=cv.contourArea)

    # Find the lowermost right corner point of the largest triangle
    lowest_point = tuple(largest_triangle[largest_triangle[:, :, 1].argmax()][0])
    rightmost_point = tuple(largest_triangle[largest_triangle[:, :, 0].argmax()][0])
    lowermost_right_point = (max(lowest_point[0], rightmost_point[0]), lowest_point[1])

    # Draw the largest triangle and the lowermost right corner point on the original cone_image
    cv.drawContours(cone_image, [largest_triangle], 0, (0, 255, 0), 2)
    cv.circle(cone_image, lowermost_right_point, 5, (255, 0, 0), -1)
    print(lowermost_right_point)

    # Display the result
    cv.imshow('Result', cone_image)
    cv.waitKey(0)
    cv.destroyAllWindows()
else:
    print("No triangle found in the cone_image.")
