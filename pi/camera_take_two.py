import numpy as np
import cv2 as cv
import glob
import os
import matplotlib as plt
from point import Point
import json

# Functionality
# 1. Convert pixel coords to World Coords [done]
# 2. Calibrate camera with set images [done]
# 3. Find center of ball
#    a. get u,v coordinates [done]
#    b. generate real world coordinates [pending]
# 4. Set ball type based on color scheme [done]


class Camera:
    def __init__(self, u, v, port, debug=False):
        self.debug = debug

        # Frame size of camera
        self.u = u
        self.v = v

        # Set camera object and port
        self.port = port
        self.cam = cv.VideoCapture(self.port)

        # Check if camera has been opened before
        if not (self.cam).isOpened():
            print("Camera could not be opened, try again")
            exit()

        # Camera params
        self.OPEN_CV_DELAY = 100
        self.frameSize = (u, v)
        self.cameraMatrix = np.zeros((3, 3), dtype=np.float32)
        # Ball type and colors
        # ballType:
        #   0 -> ping pong
        #   1 -> golf ball
        #   2 -> steel ball
        self.lower_color = np.array([[0, 0, 255], [0, 0, 0], [0, 0, 0]])
        self.upper_color = np.array([[255, 255, 255], [0, 0, 0], [0, 0, 0]])

    def calibrate_cam_from_images(self, dir='pi/calibration/test'):
        # Get images from calibration folder
        images = glob.glob(f'{dir}/*.png')

        if not images:
            print("No images found in the folder")
            return

        # Termination criteria
        criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # Prepare object points
        objp = np.zeros((6*7, 3), np.float32)
        objp[:, :2] = np.mgrid[0:7, 0:6].T.reshape(-1, 2)

        # Arrays to store object points and image points from all images
        objpoints = []
        imgpoints = []

        # Loop through all images
        for fname in images:
            img = cv.imread(fname)
            gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

            # Find the chess board corners
            ret, corners = cv.findChessboardCorners(gray, (7, 6), None)

            # If found, add object points, image points
            if ret:
                objpoints.append(objp)

                corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                imgpoints.append(corners2)

                # Draw and display the corners
                img = cv.drawChessboardCorners(img, (7, 6), corners2, ret)
                cv.imshow('img', img)
                cv.waitKey(self.OPEN_CV_DELAY)
        
        # Calibrate camera
        ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

        # convert rvecs and tvecs to list
        rvecs = [rvec.tolist() for rvec in rvecs]
        tvecs = [tvec.tolist() for tvec in tvecs]

        # Save the calibration data to a json file, pi/camera_calibration_data.json
        data = {
            'cameraMatrix': mtx.tolist(),
            'dist': dist.tolist(),
            'rvecs': rvecs,
            'tvecs': tvecs
        }

        self.cameraMatrix = mtx
        self.dist = dist # unused for now?

        with open('pi/camera_calibration_data.json', 'w') as f:
            json.dump(data, f)

        print("Camera calibrated successfully")

    def show_camera_feed(self):
        # Show camera feed
        while True:
            ret, frame = self.cam.read()
            coords = self.detect_ball(frame)
            if coords:
                self.pixel_to_world_coords(coords.x, coords.y)

            if not ret:
                print("Cannot receive frame (stream end?). Exiting ...")
                break

            if cv.waitKey(1) == ord('q'):
                break

    def detect_ball(self, image):
        # Resize the image to desired dimensions
        frame = cv.resize(image, (self.u, self.v))
        
        # Convert the image to HSV color space
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        
        # Define the radius of the circle in pixels and the center of the image
        ball_platform_radius_px = 150
        center_x, center_y = frame.shape[1] // 2, frame.shape[0] // 2
        
        # Create a circular mask with the specified radius and center
        circular_mask = np.zeros((frame.shape[0], frame.shape[1]), dtype=np.uint8)
        cv.circle(circular_mask, (center_x, center_y), ball_platform_radius_px, (255), -1)
        
        # Apply the color threshold to create a mask for the target color
        mask = cv.inRange(hsv, self.lower_color[0], self.upper_color[0])
        
        # Combine the color mask with the circular mask
        combined_mask = cv.bitwise_and(mask, mask, mask=circular_mask)
        
        # Apply the combined mask to the original frame
        res = cv.bitwise_and(frame, frame, mask=combined_mask)
        
        # Convert to grayscale for further processing if needed
        gray = cv.cvtColor(res, cv.COLOR_BGR2GRAY)
        
        # Blur the image to reduce noise
        blurred = cv.GaussianBlur(gray, (9, 9), 0)
        
        # Use Hough Circles to detect circular shapes
        circles = cv.HoughCircles(blurred, cv.HOUGH_GRADIENT, dp=1.2, minDist=30, param1=50, param2=30, minRadius=20, maxRadius=50)
        
        # Draw detected circles on the original image

        circle_coords = None
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for (x, y, r) in circles:
                cv.circle(frame, (x, y), r, (0, 255, 0), 4)
                cv.rectangle(frame, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)

                # Get the coordinates of the detected circle
                circle_coords = Point(x, y)
        
        # Show the masked and result image
        cv.imshow('Masked Frame', gray)
        cv.imshow('Detected Ball', frame)

        # wait for 1ms
        cv.waitKey(1)

        return circle_coords
    
    def pixel_to_world_coords(self, u, v):
              
        # Convert the pixel coordinates to world coordinates
        pixel_coords = np.array([[u, v]], dtype=np.float32)

        # Convert to camera frame using the camera matrix
        camera_coords = cv.undistortPoints(pixel_coords, self.cameraMatrix, None)

        height_m = 0.4 # height of the camera from the ground in meters

        # scale up the camera coordinates by this amount
        camera_coords = camera_coords * height_m
        obj_coords_cam_frame = np.array([camera_coords[0][0][0], camera_coords[0][0][1], height_m])

        rad_to_rotate_z = np.pi / 2
        rot_Z = np.array([[np.cos(rad_to_rotate_z), -np.sin(rad_to_rotate_z), 0],
                        [np.sin(rad_to_rotate_z), np.cos(rad_to_rotate_z), 0],
                        [0, 0, 1]])
        
        rad_to_rotate_X = np.pi
        rot_X = np.array([[1, 0, 0],
                        [0, np.cos(rad_to_rotate_X), -np.sin(rad_to_rotate_X)],
                        [0, np.sin(rad_to_rotate_X), np.cos(rad_to_rotate_X)]])

        obj_coords_platform_frame = np.dot(rot_Z, obj_coords_cam_frame)
        obj_coords_platform_frame = np.dot(rot_X, obj_coords_platform_frame)

        print(f"Pixel coordinates: ({u}, {v}), camera_coords: {camera_coords}, obj_coords_platform_frame: {obj_coords_platform_frame}")



if __name__ == '__main__':
    # from pi/camera_params.json get the data 
    with open('pi/camera_params.json') as f:
        data = json.load(f)
    
    # Create camera object
    cam = Camera(data['u'], data['v'], "/dev/video0", debug=True)
    print(cam.u, cam.v, cam.port)
    cam.calibrate_cam_from_images()
    cam.show_camera_feed()
