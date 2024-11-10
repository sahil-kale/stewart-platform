import numpy as np
import matplotlib as plt
from point import Point


class KalmanFilter:
    def __init__(self, K, dt=1.0):
        self.K = K  # Kalman gain
        self.dt = dt
        self.X = np.zeros(
            (4, 1)
        )  # [x, y, vx, vy]^T - current estimate of the ball state
        self.P = np.eye(4) * 1000  # Large initial uncertainty

        self.A = np.array(
            [[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]]
        )  # Transition matrix to get state estimate given previous estimate

        self.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])  # Measurement matrix

        self.Q = np.eye(4) * 0.01  # Process noise covariance
        self.R = np.eye(2) * 5.0  # Measurement noise covariance

    def predict(self):
        self.X = (
            self.A @ self.X
        )  # Get predicted next state based on the transition matrix
        self.P = (
            self.A @ self.P @ self.A.T + self.Q
        )  # Update the sensor covariance matrix

    def update(self, measured_point):
        # Use the new measurements to update the prediction

        Z = np.array([measured_point.x, measured_point.y]).reahape(2, 1)

        # Update error vector Y
        Y = Z - self.H @ self.X
        S = self.H @ self.P @ self.H.T + self.R
        self.K = self.P @ self.H.T @ np.linalg.inv(S)

        self.P = (np.eye(len(self.P)) - self.K @ self.H) @ self.P

    def kalman_filter(self, measured_point):
        self.predict()
        self.update(measured_point)
        estimated_position = Point(self.X[0, 0], self.X[1, 0])
        estimated_velocity = Point(self.X[2, 0], self.X[3, 0])
        new_state = [estimated_position, estimated_velocity]
        return (
            new_state  # Return the most recent state estimate as an array of two points
        )
