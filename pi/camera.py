# imports
import numpy as np
import cv2 as cv
import glob
import os
import matplotlib as plt
from point import Point

# Functionality
# 1. Convert pixel coords to World Coords [done]
# 2. Calibrate camera with set images [done]
# 3. Find center of ball
#    a. get u,v coordinates [done]
#    b. generate real world coordinates [pending]
# 4. Set ball type based on color scheme [done]


class Camera:
    def __init__(self, u, v, num):
        # focal length, currently unknown
        self.u = u
        self.v = v

        # pixel resolution of camera

        self.num = num

        self.cam = cv.VideoCapture(self.num)

        self.scale = 400

        # self.cam.open(self.num, cv.CAP_DSHOW)
        if not (self.cam).isOpened():
            print("Camera could not be opened, try again")
            exit()

        # Camera params
        self.frameSize = (u, v)
        self.cameraMatrix = np.zeros((3, 3), dtype=np.float32)
        self.newCameraMatrix = np.zeros((3, 3), dtype=np.float32)
        self.dist = None
        self.rvec = None
        self.tvec = None
        self.ball_loc = Point(0, 0)

        # Ball type and colors
        # ballType:
        #   0 -> ping pong
        #   1 -> golf ball
        #   2 -> steel ball
        self.lower_color = np.array([[58, 0, 205], [0, 0, 0], [0, 0, 0]])
        self.upper_color = np.array([[191, 195, 255], [0, 0, 0], [0, 0, 0]])

        # (self.cam).release()
        # cv.destroyAllWindows()

    def open_camera(self):
        (self.cam).open(self.num)

    def close_camera(self):
        (self.cam).release()
        cv.destroyAllWindows()

    def get_ball_coordinates(self):
        _, image = (self.cam).read()
        # cv.imshow("Frame:", image)
        [u, v] = self.detectBall(0, image)
        xyz_values = self.scale * self.detect_xyz(u, v)
        self.ball_loc.x = xyz_values[0]
        self.ball_loc.y = xyz_values[1]
        return self.ball_loc

    def calibrate(self, ncorner_w, ncorner_h):
        # set up for files
        root = os.getcwd()
        calibrationDir = os.path.join(root, r"\\calibration\\images")
        chessboardSize = (ncorner_w, ncorner_h)  # undetermined value

        criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # set up points arrays
        objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0 : chessboardSize[0], 0 : chessboardSize[1]].T.reshape(
            -1, 2
        )

        objPoints = []
        imgPoints = []

        # recursively find all images to test
        images = glob.glob(r"calibrationimages/*.png")
        for image in images:
            img = cv.imread(image)
            gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

            ret, corners = cv.findChessboardCorners(gray, chessboardSize, None)
            if ret == True:
                objPoints.append(objp)
                corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                imgPoints.append(corners)

                cv.drawChessboardCorners(img, chessboardSize, corners2, ret)
                cv.imshow("img", img)
                cv.waitKey(100)
        # (self.cam).release()
        cv.destroyAllWindows()

        # generate matrices
        ret, cameraMatrix, dist, rvecs, tvecs = cv.calibrateCamera(
            objPoints, imgPoints, self.frameSize, None, None
        )

        # Set class objects
        self.cameraMatrix = cameraMatrix
        self.dist = dist
        self.rvec = rvecs
        self.tvec = tvecs
        # distortion

        # images = glob.glob(r'C:\Users\aksha\Desktop\University\4A\MTE 380\mte-380\pi\calibration\images\*.png')
        counter = 0
        for image in images:
            img = cv.imread(image)  # file name will prolly change
            h, w = img.shape[:2]

            newCameraMatrix, roi = cv.getOptimalNewCameraMatrix(
                cameraMatrix, dist, (w, h), 1, (w, h)
            )

            self.newCameraMatrix = newCameraMatrix

            dst = cv.undistort(img, cameraMatrix, dist, None, newCameraMatrix)
            x, y, w, h = roi
            dst = dst[y : y + h, x : x + w]
            img_name = f"calibration/cali/caliResults_{counter}.png"
            path = str(os.getcwd()) + img_name
            cv.imwrite(path, dst)
            counter = counter + 1

    def calibration_images(self):
        #  Run function to collect images
        img_counter = 0
        while True:
            ret, frame = (self.cam).read()
            if not ret:
                print("Can't receive frame (stream end?). Exiting ...")
                break

            cv.imshow("test", frame)
            k = cv.waitKey(1)
            if k % 256 == 27:  # ESC
                print("escape hit, closing the app")
                break
            elif k % 256 == 32:  # SPACEBAR
                # the format for storing the images screenshotted
                img_name = f"calibration/images/cali_{img_counter}.png"  # set relative path for images
                path = str(os.getcwd()) + (
                    img_name
                )  # combine user root path to image path
                # saves the image as a png file
                cv.imwrite(path, frame)
                print("screenshot taken")
                # the number of images automaticallly increases by 1
                img_counter += 1
        (self.cam).release()

        # stops the camera window
        (self.cam).destoryAllWindows()

    def detect_xyz(self, u, v):
        uv_1 = np.array([u, v, 1], dtype=np.float32)
        uv_1 = uv_1.T
        if self.rvec is None:
            print("Rotation Matrix is none")
            exit()

        R_mtx, jac = cv.Rodrigues((self.rvec)[0])
        inv_R = np.linalg.inv(R_mtx)
        inv_newcam = np.linalg.inv(self.cameraMatrix)
        xyz_c = inv_newcam.dot(uv_1)
        xyz = inv_R.dot(xyz_c)
        # #Camera Matrices Set Up
        # #EQN for [X,Y,Z] = R^-1 (sA^-1[u;v;1] - t_vect)
        return xyz

    def list_ports(self):
        """
        Test the ports and returns a tuple with the available ports and the ones that are working.
        """
        non_working_ports = []
        dev_port = 0
        working_ports = []
        available_ports = []
        while (
            len(non_working_ports) < 6
        ):  # if there are more than 5 non working ports stop the testing.
            camera = cv.VideoCapture(dev_port)
            if not camera.isOpened():
                non_working_ports.append(dev_port)
                print("Port %s is not working." % dev_port)
            else:
                is_reading, img = camera.read()
                w = camera.get(3)
                h = camera.get(4)
                if is_reading:
                    print(
                        "Port %s is working and reads images (%s x %s)"
                        % (dev_port, h, w)
                    )
                    working_ports.append(dev_port)
                else:
                    print(
                        "Port %s for camera ( %s x %s) is present but does not reads."
                        % (dev_port, h, w)
                    )
                    available_ports.append(dev_port)
            dev_port += 1
        return available_ports, working_ports, non_working_ports

    def detectBall(self, balltype, image):
        x = -100
        y = -100
        frame = image
        # if not ret:
        #     print("Could not grab frame")
        #     return [None,None]
        frame = cv.resize(frame, (self.u, self.v))
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        # Setup ball color array, need a function
        lower_ball_color = self.lower_color[0]
        higher_ball_color = self.upper_color[0]

        mask = cv.inRange(hsv, lower_ball_color, higher_ball_color)
        _, contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        # # Find the index of the largest contour
        if contours:
            # Determines the largest contour size using the cv.contour Area function
            largest_contour = max(contours, key=cv.contourArea)
            # Computes the minimum enclosing circle aroudn the largest contour
            ((x, y), radius) = cv.minEnclosingCircle(largest_contour)
            # * 4 Only consider large enough objects. If it only detects a small portion of your ball, you can test higher radius values to capture more of the ball
            if radius > 10 and radius < 100:
                # Draw a yellow circle around the ball
                cv.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                # Draw a red dot in the center of the ball
                cv.circle(
                    frame, (int(x), int(y)), 2, (0, 0, 255), -1
                )  # (image to draw dot on, x,y pixel coordinates, radius in pixels, RGB values in this case red, -1 indicates to fill the circle)
                # Display the position of the ball
        # Display the resulting frame
        cv.imshow("frame", frame)
        # Release the capture when everything is done
        # (self.cam).release()
        # # Close all windows
        # cv.destroyAllWindows()
        return [int(x), int(y)]

    def nothing(x):
        pass

    def colorMaskDetect(self):
        self.cam.open(self.num)
        Winname = "Frame:"
        cv.namedWindow("Frame:")
        # H, S,V are for Lower Boundaries
        # H2,S2,V2 are for Upper Boundaries
        cv.createTrackbar("H", Winname, 0, 255, self.nothing)
        cv.createTrackbar("S", Winname, 0, 255, self.nothing)
        cv.createTrackbar("V", Winname, 0, 255, self.nothing)
        cv.createTrackbar("H2", Winname, 0, 255, self.nothing)
        cv.createTrackbar("S2", Winname, 0, 255, self.nothing)
        cv.createTrackbar("V2", Winname, 0, 255, self.nothing)

        while (self.cam).isOpened():
            _, frame = (self.cam).read()
            H = cv.getTrackbarPos("H", "Frame:")
            S = cv.getTrackbarPos("S", "Frame:")
            V = cv.getTrackbarPos("V", "Frame:")
            H2 = cv.getTrackbarPos("H2", "Frame:")
            S2 = cv.getTrackbarPos("S2", "Frame:")
            V2 = cv.getTrackbarPos("V2", "Frame:")
            hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
            lower_boundary = np.array([H, S, V])
            upper_boundary = np.array([H2, S2, V2])
            mask = cv.inRange(hsv, lower_boundary, upper_boundary)
            final = cv.bitwise_and(frame, frame, mask=mask)
            cv.imshow("Frame:", final)

            if cv.waitKey(1) == ord("q"):
                break

        (self.cam).release()
        cv.destroyAllWindows()
