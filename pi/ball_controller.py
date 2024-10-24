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

    def update_x(self, error):
        self.integral_error_x += error * self.dt
        
