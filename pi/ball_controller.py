from point import Point
import numpy as np


class BallController:
    def __init__(self, kp_x, ki_x, kd_x, kp_y, ki_y, kd_y, dt, saturation_angle):
        # X-axis PID parameters
        self.kp_x = kp_x
        self.ki_x = ki_x
        self.kd_x = kd_x

        # Y-axis PID parameters
        self.kp_y = kp_y
        self.ki_y = ki_y
        self.kd_y = kd_y

        # Time step
        self.dt = dt

        # Saturation angle
        self.saturation_angle = saturation_angle

        # Initialize errors for x and y
        self.integral_error_x = 0
        self.integral_error_y = 0

        self.previous_error_x = 0
        self.previous_error_y = 0

    def set_gains(self, kp_x, ki_x, kd_x, kp_y, ki_y, kd_y):
        self.kp_x = kp_x
        self.ki_x = ki_x
        self.kd_x = kd_x

        self.kp_y = kp_y
        self.ki_y = ki_y
        self.kd_y = kd_y

    def update(self, error, integral_error, previous_error, kp, ki, kd):
        # Calculate integral and derivative errors
        integral_error += error * self.dt
        derivative_error = (error - previous_error) / self.dt

        # PID formula
        output = kp * error + ki * integral_error + kd * derivative_error

        # Return the output and updated integral/previous errors for future use
        return output, integral_error, error

    def run_control_loop(self, desired_position: Point, current_position: Point):
        # Calculate errors
        error_x = desired_position.x - current_position.x
        error_y = desired_position.y - current_position.y

        # error for y is inverted
        error_y = -error_y

        # Update X control
        output_x, self.integral_error_x, self.previous_error_x = self.update(
            error_x,
            self.integral_error_x,
            self.previous_error_x,
            self.kp_x,
            self.ki_x,
            self.kd_x,
        )

        # Update Y control
        output_y, self.integral_error_y, self.previous_error_y = self.update(
            error_y,
            self.integral_error_y,
            self.previous_error_y,
            self.kp_y,
            self.ki_y,
            self.kd_y,
        )

        output_x = np.clip(output_x, -self.saturation_angle, self.saturation_angle)
        output_y = np.clip(output_y, -self.saturation_angle, self.saturation_angle)

        return output_x, output_y
