import numpy as np
import matplotlib.pyplot as plt
from point import Point
import argparse
import os
import json


class KalmanFilter:
    def __init__(self, K=1.0, Q=0.01, R=5.0, dt=1.0):
        self.K = K  # Initial kalman gain
        self.dt = dt
        self.X = np.zeros(
            (6, 1)
        )  # [x, y, vx, vy, ax, ay]^T - current estimate of the ball state
        self.P = np.eye(6) * 1000  # Large initial uncertainty

        self.A = np.array(
            [
                [1, 0, self.dt, 0, 0.5 * self.dt**2, 0],
                [0, 1, 0, self.dt, 0, 0.5 * self.dt**2],
                [0, 0, 1, 0, self.dt, 0],
                [0, 0, 0, 1, 0, self.dt],
                [0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 1],
            ]
        )  # Transition matrix to get state estimate given previous estimate

        self.H = np.array(
            [[1, 0, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0]]
        )  # Measurement matrix

        self.Q = np.eye(6) * Q  # Process noise covariance
        self.R = np.eye(2) * R  # Measurement noise covariance

        # Arrays for visualization of data
        self.noisy_positions_x = []
        self.noisy_positions_y = []
        self.filtered_positions_x = []
        self.filtered_positions_y = []
        self.filtered_velocity_x = []
        self.filtered_velocity_y = []
        self.filtered_acceleration_x = []
        self.filtered_acceleration_y = []

    def predict(self):
        self.X = (
            self.A @ self.X
        )  # Get predicted next state based on the transition matrix
        self.P = (
            self.A @ self.P @ self.A.T + self.Q
        )  # Update the sensor covariance matrix

        estimated_position = Point(self.X[0, 0], self.X[1, 0])
        estimated_velocity = Point(self.X[2, 0], self.X[3, 0])
        estimated_acceleration = Point(self.X[4, 0], self.X[5, 0])
        new_state = [estimated_position, estimated_velocity, estimated_acceleration]

        self.filtered_positions_x.append(new_state[0].x)
        self.filtered_positions_y.append(new_state[0].y)

        self.filtered_velocity_x.append(new_state[1].x)
        self.filtered_velocity_y.append(new_state[1].y)

        self.filtered_acceleration_x.append(new_state[2].x)
        self.filtered_acceleration_y.append(new_state[2].y)

        return new_state

    def update(self, measured_point):
        # Use the new measurements to update the prediction

        Z = np.array([measured_point.x, measured_point.y]).reshape(2, 1)

        # Update error vector Y
        Y = Z - self.H @ self.X
        S = self.H @ self.P @ self.H.T + self.R
        self.K = self.P @ self.H.T @ np.linalg.inv(S)

        self.P = (np.eye(len(self.P)) - self.K @ self.H) @ self.P

        self.X = self.X + self.K @ Y  # Update value based on the new kalman gain
        estimated_position = Point(self.X[0, 0], self.X[1, 0])
        estimated_velocity = Point(self.X[2, 0], self.X[3, 0])
        estimated_acceleration = Point(self.X[4, 0], self.X[5, 0])

        new_state = [estimated_position, estimated_velocity, estimated_acceleration]

        self.noisy_positions_x.append(measured_point.x)
        self.noisy_positions_y.append(measured_point.y)

        self.filtered_positions_x.append(new_state[0].x)
        self.filtered_positions_y.append(new_state[0].y)

        self.filtered_velocity_x.append(new_state[1].x)
        self.filtered_velocity_y.append(new_state[1].y)

        self.filtered_acceleration_x.append(new_state[2].x)
        self.filtered_acceleration_y.append(new_state[2].y)

        return new_state

    def visualize_data(self):
        plt.figure(figsize=(10, 6))

        # Plot noisy measurements
        plt.scatter(
            self.noisy_positions_x,
            self.noisy_positions_y,
            color="red",
            label="Noisy Data",
            s=10,
            alpha=0.5,
        )

        # Plot filtered estimates
        # plt.plot(
        #     self.filtered_positions_x,
        #     self.filtered_positions_y,
        #     color="blue",
        #     label="Filtered Position Data (Kalman)",
        #     linewidth=2,
        # )

        # Labeling
        plt.title("Kalman Filter: Noisy vs. Filtered Position")
        plt.xlabel("X Position")
        plt.ylabel("Y Position")
        plt.legend()
        plt.grid(True)

        # Show plot
        plt.show()

def main(Q_val, R_val, dt_val):
    current_dir = os.path.dirname(os.path.realpath(__file__))
    measured_points_file_path = os.path.join(current_dir, "measured_points.json")
    with open(measured_points_file_path, "r") as file:
        data = json.load(file)  # Load JSON data as a dictionary
    
    measured_points_x = data["x"]
    measured_points_y = data["y"]

    kalman_filter = KalmanFilter(Q_val, R_val, dt_val)

    for i in range(len(measured_points_x)):
        noisy_x = measured_points_x[i]
        noisy_y = measured_points_y[i]

        current_measurement = Point(noisy_x, noisy_y)

        filtered_state = kalman_filter.predict()

        filtered_state = kalman_filter.update(current_measurement)

    kalman_filter.visualize_data()



if __name__ == "__main__":
    # Set up argument parser
    parser = argparse.ArgumentParser(
        description="Run camera and Kalman filter with specified parameters."
    )
    parser.add_argument(
        "--Q", type=float, default=0.01, help="Process noise covariance"
    )
    parser.add_argument(
        "--R", type=float, default=5.0, help="Measurement noise covariance"
    )
    parser.add_argument("--dt", type=float, default=1.0, help="Time step interval")

    args = parser.parse_args()

    # Call main function with parsed arguments
    main(args.Q, args.R, args.dt)
