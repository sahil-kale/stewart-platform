from point import Point
import numpy as np
import math
import time
from anti_stiction import AntiStictionController


class BallController:
    def __init__(
        self,
        kp_x,
        ki_x,
        kd_x,
        kp_y,
        ki_y,
        kd_y,
        dt,
        saturation_angle,
        max_euclidean_error,
        integral_windup_clear_threshold,
        stiction_compensation_deadband,
        stiction_compensation_feedforward,
        integral_clear_threshold,
    ):
        # X-axis PID parameter range
        self.kp_range_x = kp_x
        self.ki_range_x = ki_x
        self.kd_range_x = kd_x

        # Y-axis PID parameter range
        self.kp_range_y = kp_y
        self.ki_range_y = ki_y
        self.kd_range_y = kd_y

        # Time step
        self.dt = dt

        # Saturation angle
        self.saturation_angle = saturation_angle

        # Max Euclidean error for gain scheduling
        self.max_euclidean_error = max_euclidean_error

        # Max integral error before clearing, to prevent against windup
        self.integral_windup_clear_threshold = integral_windup_clear_threshold

        self.integral_clear_threshold = integral_clear_threshold

        # Initialize errors for x and y
        self.integral_error_x = 0
        self.integral_error_y = 0

        self.previous_error_x = 0
        self.previous_error_y = 0

        self.current_kp_x = 0
        self.current_ki_x = 0
        self.current_kd_x = 0

        self.current_kp_y = 0
        self.current_ki_y = 0
        self.current_kd_y = 0

        self.stiction_compensation_deadband = stiction_compensation_deadband
        self.stiction_compensation_feedforward = stiction_compensation_feedforward

        num_delay_steps = 3

        self.anti_stiction_controller_x = AntiStictionController(
            self.stiction_compensation_deadband,
            self.stiction_compensation_feedforward,
            0,
            self.dt * num_delay_steps,
        )
        self.anti_stiction_controller_y = AntiStictionController(
            self.stiction_compensation_deadband,
            self.stiction_compensation_feedforward,
            0,
            self.dt * num_delay_steps,
        )

    # Use gains as determined by euclidean error magnitude
    def update(
        self,
        error,
        integral_error,
        previous_error,
        kp,
        ki,
        kd,
        anti_stiction_controller,
    ):
        # Calculate integral and derivative errors
        integral_error += error * self.dt
        derivative_error = (error - previous_error) / self.dt

        integral_control_output = ki * integral_error

        # PID formula
        output = kp * error + kd * derivative_error + integral_control_output

        # Anti-stiction compensation
        anti_stiction_output = anti_stiction_controller.run(output, time.time())
        # Return the output and updated integral/previous errors for future use
        return output + anti_stiction_output, integral_error, error

    def run_control_loop(self, desired_position: Point, current_position: Point):
        # Calculate errors
        error_x = desired_position.x - current_position.x
        error_y = desired_position.y - current_position.y

        # error for y is inverted
        error_y = -error_y

        error_euclidean = math.sqrt(error_x**2 + error_y**2)
        if error_euclidean > self.integral_clear_threshold:
            self.integral_error_x = 0
            self.integral_error_y = 0

        # Interpolate between inner and outer gains to get current gains
        self.update_instantaneous_gains(error_euclidean)

        # Update X control
        output_x, self.integral_error_x, self.previous_error_x = self.update(
            error_x,
            self.integral_error_x,
            self.previous_error_x,
            self.current_kp_x,
            self.current_ki_x,
            self.current_kd_x,
            self.anti_stiction_controller_x,
        )

        # Update Y control
        output_y, self.integral_error_y, self.previous_error_y = self.update(
            error_y,
            self.integral_error_y,
            self.previous_error_y,
            self.current_kp_y,
            self.current_ki_y,
            self.current_kd_y,
            self.anti_stiction_controller_y,
        )

        if np.abs(self.integral_error_x) > self.integral_windup_clear_threshold:
            self.integral_error_x = 0

        if np.abs(self.integral_error_y) > self.integral_windup_clear_threshold:
            self.integral_error_y = 0

        output_x = np.clip(output_x, -self.saturation_angle, self.saturation_angle)
        output_y = np.clip(output_y, -self.saturation_angle, self.saturation_angle)

        return output_x, output_y

    def update_instantaneous_gains(self, euclidean_error):
        self.current_kp_x = np.interp(
            euclidean_error, [0, self.max_euclidean_error], self.kp_range_x
        )
        self.current_ki_x = np.interp(
            euclidean_error, self.ki_range_x[1], self.ki_range_x[0]
        )
        self.current_kd_x = np.interp(
            euclidean_error, [0, self.max_euclidean_error], self.kd_range_x
        )

        self.current_kp_y = np.interp(
            euclidean_error, [0, self.max_euclidean_error], self.kp_range_y
        )
        self.current_ki_y = np.interp(
            euclidean_error, self.ki_range_y[1], self.ki_range_y[0]
        )
        self.current_kd_y = np.interp(
            euclidean_error, [0, self.max_euclidean_error], self.kd_range_y
        )
