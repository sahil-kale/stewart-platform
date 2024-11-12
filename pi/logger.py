import time
import json

import os, pty


class Logger:
    def __init__(self, file_path):
        self.file_path = file_path

    # Format used is this:
    # data = [
    #     {
    #         "time": time_since_start,
    #         "camera_measured_x": current_measurement_x,
    #         "camera_measured_y": current_measurement_y,
    #         "camera_filtered_x": self.current_position.x,
    #         "camera_filtered_y": self.current_position.y,
    #         "desired_position_x": desired_position.x,
    #         "desired_position_y": desired_position.y,
    #         "pitch_rad": pitch_rad,
    #         "roll_rad": roll_rad,
    #         "height": height,
    #         "servo_1_angle": servo_angle[0][0],
    #         "servo_2_angle": servo_angle[1][0],
    #         "servo_3_angle": servo_angle[2][0]
    #     }
    # ]
    def log_new_data(self, data):
        with open(self.file_path, "w") as file:
            json.dump(data, file, indent=4)
