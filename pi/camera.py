#imports
import numpy as np
import cv2 as cv
import glob
import os
import matplotlib as plt

    # Functionality
    # 1. Convert pixel coords to World Coords [done]
    # 2. Calibrate camera with set images [done]
    # 3. Find center of ball 
    #    a. get u,v coordinates [done]
    #    b. generate real world coordinates [pending]
    # 4. Set ball type based on color scheme [done]


class Camera():
    def __init__(self, fx, fy, u, v):
        #focal length, currently unknown
        self.fx = fx
        self.fy = fy

        # pixel resolution of camera
        self.u = u 
        self.v = v

        #Camera params
        self.frameSize = (u,v)
        self.cameraMatrix = np.zeros((3,3),dtype = np.float32)
        self.newCameraMatrix = np.zeros((3,3),dtype = np.float32)
        self.dist 
        self.rvec
        self.tvec

        #Ball type and colors
        # ballType: 
        #   0 -> ping pong
        #   1 -> golf ball
        #   2 -> steel ball
        self.upper_color = [[0,0,0],[0,0,0],[0,0,0]]
        self.lower_color = [[0,0,0],[0,0,0],[0,0,0]]
    
    def calibrate(self,ncorner_w,ncorner_h):
        #set up for files
        root = os.getcwd()
        calibrationDir = os.path.join(root,r'\\calibration\\images')
        chessboardSize = (ncorner_w,ncorner_h) #undetermined value
        criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        #set up points arrays 
        objp = np.zeros((chessboardSize[0]*chessboardSize[1],3),np.float32)
        objp[:,:2] = np.mgrid[0:chessboardSize[0],0:chessboardSize[1]].T.reshape(-1,2)

        objPoints = []
        imgPoints = []

    #recursively find all images to test
        images = glob.glob('*.png')
        for image in images:
            img = cv.imread(image)
            gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

            ret, corners = cv.findChessboardCorners(gray, chessboardSize, None)

            if ret == True:

                objPoints.append(objp)
                corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
                imgPoints.append(corners)

                cv.drawChessboardCorners(img, chessboardSize, corners2, ret)
                cv.imshow('img',img)
                cv.waitkey(1000)

        cv.destroyAllWindows()

        #generate matrices
        ret, cameraMatrix, dist, rvecs, tvecs = cv.calibrateCamera(objPoints,imgPoints, self.frameSize, None, None) 

        #Set class objects
        self.cameraMatrix = cameraMatrix
        self.dist = dist
        self.rvec = rvecs
        self.tvec = tvecs

        #distortion
        img = cv.imread('cali5.png') #file name will prolly change
        h, w =img.shape[:2]
        newCameraMatrix, roi = cv.getOptimalNewCameraMatrix(cameraMatrix, dist, (w,h), 1, (w,h))

        self.newCameraMatrix = newCameraMatrix

        dst = cv.undistort(img, cameraMatrix, dist, None, newCameraMatrix)
        x,y,w,h = roi
        dst = dst[y:y+h,x:x+w]
        cv.imwrite('caliResults.png',dst)
        return newCameraMatrix

    def calibrationImages(self):
    #  Run function to collect images
    # 
        cam = cv.VideoCapture(0)
        img_counter = 0
        while True:
            ret, frame = cam.read()
            cv.imshow('test', frame)
            k = cv.waitKey(1)
            if k%256 == 27:
                print('escape hit, closing the app')
                break
                # if the spacebar key is been pressed
                # screenshots will be taken
            elif k%256  == 32:
                # the format for storing the images screenshotted
                img_name = f'\pi\calibration\images\\cali_{img_counter}.jpg' #set relative path for images
                path = str(os.getcwd()) + (img_name) #combine user root path to image path 
                # saves the image as a png file
                cv.imwrite(path, frame)
                print('screenshot taken')
                # the number of images automaticallly increases by 1
                img_counter += 1
        cam.release()

        # stops the camera window
        cam.destoryAllWindows()

    def detectXYZ(self, u,v):
        uv_1 = np.array([u,v,1],dtype=np.float32)
        uv_1 = uv_1.T

        #Camera Matrices Set Up
        R_mtx, jac = cv.Rodrigues(self.rvec) #rotation matrix
        Rt = np.column_stack((R_mtx,self.tvec)) #Create [R|t]
        P_mtx = self.newCameraMatrix.dot(Rt)

        #EQN for [X,Y,Z] = R^-1 (sA^-1[u;v;1] - t_vect)
        xyz = (np.linalg.inv(R_mtx)).dot((np.linalg.inv(self.newCameraMatrix)).dot(uv_1) - self.tvec)
        return xyz

    def detectBall(self,balltype):
    cap = cv.VideoCapture(0)
    cap.set(cv.CAP_PROP_FPS, 30)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Could not grab frame")
            break
        frame = cv.resize(frame,(self.u,self.v))
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        #Setup ball color array, need a function 
        lower_ball_color = self.upper_color[balltype]
        higher_ball_color = self.lower_color[balltype]

        mask = cv.inRange(hsv,lower_ball_color,higher_ball_color)
        contours, _ =  cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        # Find the index of the largest contour
        if contours:
            # Determines the largest contour size using the cv.contour Area function
            largest_contour = max(contours, key=cv.contourArea)
            # Computes the minimum enclosing circle aroudn the largest contour
            ((x, y), radius) = cv.minEnclosingCircle(largest_contour)
            # * 4 Only consider large enough objects. If it only detects a small portion of your ball, you can test higher radius values to capture more of the ball
            if radius > 10:
                # Draw a yellow circle around the ball
                cv.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                # Draw a red dot in the center of the ball
                cv.circle(frame, (int(x), int(y)), 2, (0, 0, 255), -1)  # (image to draw dot on, x,y pixel coordinates, radius in pixels, RGB values in this case red, -1 indicates to fill the circle)
                # Display the position of the ball
                print(f"Yellow ball detected at position: ({int(x)}, {int(y)})")
        # Display the resulting frame
        cv.imshow('frame', frame)

        # Break the loop when 'q' is pressed
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

        return x,y
    # Release the capture when everything is done
    cap.release()
    # Close all windows
    cv.destroyAllWindows()






