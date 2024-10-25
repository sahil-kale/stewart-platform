class BallController:
    def __init__(self, kp_x, ki_x, kd_x, kp_y, ki_y, kd_y, dt):
        self.kp_x = kp_x
        self.ki_x = ki_x
        self.kd_x = kd_x

        self.kp_y = kp_y
        self.ki_y = ki_y
        self.kd_y = kd_y

        self.dt = dt

        self.integral_error_x = 0
        self.integral_error_y = 0

        self.previous_error_x = 0
        self.previous_error_y = 0

    def update_x(self, error):
        self.integral_error_x += error * self.dt
        current_derivative_error_x = (error - self.previous_error_x) / self.dt

        output_x = (
            self.kd_x * error
            + self.ki_x * self.integral_error_x
            + self.kd_x * current_derivative_error_x
        )
        return output_x

    def update_y(self, error):
        self.integral_error_y += error * self.dt
        current_derivative_error_y = (error - self.previous_error_y) / self.dt

        output_y = (
            self.kd_y * error
            + self.ki_y * self.integral_error_y
            + self.kd_y * current_derivative_error_y
        )
        return output_y

    def run_control_loop(self, desired_position, current_position):
        error_x = desired_position[0] - current_position[0]
        error_y = desired_position[1] - current_position[1]

        output_x = self.update_x(error_x)
        output_y = self.update_y(error_y)

        return output_x, output_y
