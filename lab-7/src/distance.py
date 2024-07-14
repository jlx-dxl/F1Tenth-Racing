import numpy as np
import cv2 as cv
import glob

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((6*8, 3), np.float32)
objp[:, :2] = np.mgrid[0:8, 0:6].T.reshape(-1, 2) * (25/8)  # Each square is 25/8 cm

objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

images = glob.glob('calibration/*.png')

for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    ret, corners = cv.findChessboardCorners(gray, (8,6), None)

    if ret == True:
        objpoints.append(objp)

        corners2 = cv.cornerSubPix(gray, corners, (11,1), (-1,-1), criteria)
        imgpoints.append(corners2)

        cv.drawChessboardCorners(img, (8,6), corners2, ret)
        # cv.imshow('img', img)
        

ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
print(mtx)


image_width = 960
image_height = 540

def cone_bottom_corner(cone_image):

    hsv = cv.cvtColor(cone_image, cv.COLOR_BGR2HSV)

    lower_red = np.array([0, 100, 100])
    upper_red = np.array([10, 255, 255])

    mask = cv.inRange(hsv, lower_red, upper_red)

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
    cv.drawContours(cone_image, [largest_triangle], 0, (0, 255, 0), 2)
    cv.circle(cone_image, lowermost_right_point, 5, (255, 0, 0), -1)
    print(lowermost_right_point)
    cv.imshow('Result', cone_image)
    cv.waitKey(0)
    # cv.destroyAllWindows()

    return lowermost_right_point


cone_image = cv.imread('resource/cone_x40cm.png')
corner_point = cone_bottom_corner(cone_image)
cone_corner_x = corner_point[0]
cone_corner_y = corner_point[1]

x_car_distance = 40  # cm

pixel_coord = np.array([[corner_point[0], corner_point[1], 1]]).T #x, y coordinates of bottom right cone corner
camera_coord = np.linalg.inv(mtx)@pixel_coord*x_car_distance # camera coordinate 
camera_height = camera_coord[1] #13.8 cm

print("Mounting height of the camera:", camera_height, "cm")


cone_image_2 = cv.imread('resource/cone_unknown.png')
corner_point_2 = cone_bottom_corner(cone_image_2)
cone_corner_x = corner_point_2[0]
cone_corner_y = corner_point_2[1]

def pixel_to_xy_car(pixel_x, pixel_y, image_width, image_height):

    fx = mtx[0][0]
    fy = mtx[1][1]
    cx = mtx[0][2]
    cy = mtx[1][2]
    x_car = camera_height*fy/(pixel_y-cy)
    y_car = (pixel_x-cx)*x_car/fx

    return x_car, y_car

x_car_distance, y_car_distance = pixel_to_xy_car(cone_corner_x, cone_corner_y, image_width, image_height)
print("x_car distance:", x_car_distance, "cm")
print("y_car distance:", y_car_distance, "cm")


cv.destroyAllWindows()
