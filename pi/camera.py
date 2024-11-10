import numpy as np
import cv2 as cv
import glob
from point import Point
import json
import subprocess
from kalman_filter import KalmanFilter

import os


class Camera:
    def __init__(self, u, v, port, debug=False):
        self.debug = debug

        # Frame size of camera
        self.u = u
        self.v = v

        self.static_height_m = (
            (15.5 - 1.4) * 2.54 / 100
        )  # height of the camera from the ground in meters

        # Used to a mask where we reject any input that is not within the circular platform
        self.ball_platform_radius_px = 200

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
        self.lower_color = np.array([35, 50, 100], dtype=np.uint8)
        self.upper_color = np.array([85, 255, 255], dtype=np.uint8)

        self.set_camera_brightness(25)
        self.set_camera_contrast(50)
        self.set_camera_saturation(99)
        self.set_auto_white_balance(1)
        self.set_camera_hue(100)

    def set_camera_brightness(self, brightness):
        # Set camera brightness
        subprocess.run(["v4l2-ctl", "-d", self.port, "-c", f"brightness={brightness}"])

    def set_camera_contrast(self, contrast):
        # Set camera contrast
        subprocess.run(["v4l2-ctl", "-d", self.port, "-c", f"contrast={contrast}"])

    def set_camera_saturation(self, saturation):
        # Set camera saturation
        subprocess.run(["v4l2-ctl", "-d", self.port, "-c", f"saturation={saturation}"])

    def set_auto_white_balance(self, auto):
        # Set camera white balance
        subprocess.run(
            ["v4l2-ctl", "-d", self.port, "-c", f"white_balance_automatic={auto}"]
        )

    def set_camera_hue(self, hue):
        # Set camera hue
        subprocess.run(["v4l2-ctl", "-d", self.port, "-c", f"hue={hue}"])

    def calibrate_cam_from_images(self, dir="pi/calibration/test"):
        # Get images from calibration folder
        images = glob.glob(f"{dir}/*.png")

        if not images:
            print("No images found in the folder")
            return

        # Termination criteria
        criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # Prepare object points
        objp = np.zeros((6 * 7, 3), np.float32)
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
                if self.debug:
                    img = cv.drawChessboardCorners(img, (7, 6), corners2, ret)
                    cv.imshow("img", img)
                    cv.waitKey(self.OPEN_CV_DELAY)

        # Calibrate camera
        ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(
            objpoints, imgpoints, gray.shape[::-1], None, None
        )

        # convert rvecs and tvecs to list
        rvecs = [rvec.tolist() for rvec in rvecs]
        tvecs = [tvec.tolist() for tvec in tvecs]

        # Save the calibration data to a json file, pi/camera_calibration_data.json
        data = {
            "cameraMatrix": mtx.tolist(),
            "dist": dist.tolist(),
            "rvecs": rvecs,
            "tvecs": tvecs,
        }

        self.cameraMatrix = mtx
        self.dist = dist

        with open("pi/camera_calibration_data.json", "w") as f:
            json.dump(data, f)

        print("Camera calibrated successfully")

    def load_camera_params(self, filepath: str):
        # Load camera params from json file
        with open(filepath) as f:
            data = json.load(f)

        self.cameraMatrix = np.array(data["cameraMatrix"])
        self.dist = np.array(data["dist"])

    # def show_camera_feed(self):
        # Show camera feed
        # while True:
            # print(self.get_ball_coordinates())

    def get_ball_coordinates(self):
        # Get the coordinates of the ball
        ret, frame = self.cam.read()
        coords = self.detect_ball(frame)
        if coords:
            return self.pixel_to_world_coords(coords.x, coords.y, self.static_height_m)
        else:
            if self.debug:
                print("Ball not detected!")

            return None

    def detect_ball(self, image):
        frame = cv.resize(image, (self.u, self.v))
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        center_x, center_y = frame.shape[1] // 2, frame.shape[0] // 2
        circular_mask = np.zeros((frame.shape[0], frame.shape[1]), dtype=np.uint8)
        cv.circle(
            circular_mask, (center_x, center_y), self.ball_platform_radius_px, (255), -1
        )

        # Now use the HSV values in cv.inRange
        mask = cv.inRange(hsv, self.lower_color, self.upper_color)

        # Combine the color mask with the circular mask
        combined_mask = cv.bitwise_and(mask, mask, mask=circular_mask)
        # Apply the combined mask to the original frame
        res = cv.bitwise_and(frame, frame, mask=combined_mask)

        # image show the masked frame
        if self.debug:
            cv.imshow("Masked Frame Colour", res)
            cv.waitKey(1)

        # Greyscale the mask to use with filters
        gray = cv.cvtColor(res, cv.COLOR_BGR2GRAY)
        blurred = cv.GaussianBlur(gray, (15, 15), 0)  # noise reduction

        circles = cv.HoughCircles(
            blurred,
            cv.HOUGH_GRADIENT,
            dp=1.2,
            minDist=30,
            param1=50,
            param2=30,
            minRadius=5,
            maxRadius=50,
        )
        # Draw detected circles on the original image
        circle_coords = None
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for x, y, r in circles:
                cv.circle(frame, (x, y), r, (0, 255, 0), 4)
                cv.rectangle(frame, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)

                # Get the coordinates of the detected circle
                circle_coords = Point(x, y)

        if self.debug:
            # Show the masked and result image
            cv.imshow("Masked Frame", gray)
            cv.imshow("Detected Ball", frame)

            # wait for 1ms
            cv.waitKey(1)

        return circle_coords

    def pixel_to_world_coords(self, u, v, height_m):
        # Convert the pixel coordinates to world coordinates
        pixel_coords = np.array([[u, v]], dtype=np.float32)

        # Convert to camera frame using the camera matrix
        camera_coords = cv.undistortPoints(pixel_coords, self.cameraMatrix, self.dist)

        # scale up the camera coordinates by this amount
        camera_coords = camera_coords * height_m
        obj_coords_cam_frame = np.array(
            [camera_coords[0][0][0], camera_coords[0][0][1], height_m]
        )

        rad_to_rotate_z = np.pi / 2
        rot_Z = np.array(
            [
                [np.cos(rad_to_rotate_z), -np.sin(rad_to_rotate_z), 0],
                [np.sin(rad_to_rotate_z), np.cos(rad_to_rotate_z), 0],
                [0, 0, 1],
            ]
        )

        rad_to_rotate_X = np.pi
        rot_X = np.array(
            [
                [1, 0, 0],
                [0, np.cos(rad_to_rotate_X), -np.sin(rad_to_rotate_X)],
                [0, np.sin(rad_to_rotate_X), np.cos(rad_to_rotate_X)],
            ]
        )

        obj_coords_platform_frame = np.dot(rot_Z, obj_coords_cam_frame)
        obj_coords_platform_frame = np.dot(rot_X, obj_coords_platform_frame)

        # if self.debug:
        #     print(
        #         f"Pixel coordinates: ({u}, {v}), camera_coords: {camera_coords}, obj_coords_platform_frame: {obj_coords_platform_frame}"
        #     )

        obj_coords_2d_point = Point(
            obj_coords_platform_frame[0], obj_coords_platform_frame[1]
        )

        return obj_coords_2d_point


if __name__ == "__main__":
    # from pi/camera_params.json get the data
    with open("pi/camera_params.json") as f:
        data = json.load(f)

    current_dir = os.path.dirname(os.path.realpath(__file__))
    # get the file camera.json with the current dir
    cv_params_file_path = os.path.join(current_dir, "camera_params.json")

    with open(cv_params_file_path, "r") as file:
        data = json.load(file)  # Load JSON data as a dictionary

    # Create camera object
    cv_system = Camera(data["u"], data["v"], "/dev/video0", debug=True)

    cv_system.load_camera_params("pi/camera_calibration_data.json")

    kalman_filter = KalmanFilter()  # Use default params for now

    for i in range(1000):
        current_measurement = cv_system.get_ball_coordinates()
        # print(f"Current position is: {current_measurement}")

        filtered_state = kalman_filter.predict()

        if current_measurement is not None:
            filtered_state = kalman_filter.update(current_measurement)
            camera_valid = True
        else:
            print("Ball not detected!!! Using old value for now")

    kalman_filter.visualize_data()
