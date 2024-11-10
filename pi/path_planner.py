from point_3d import Point3D
import numpy as np


class PathPlanner:
    def __init__(self, platform_radius):
        self.platform_radius = platform_radius
        self.platform_height = 0.06  # TODO - add dynamic height changes

    def get_square_trajectory(self):
        bottom_left_corner_point = Point3D(
            0 - self.platform_radius / 2,
            0 - self.platform_radius / 2,
            self.platform_height,
        )
        bottom_right_corner_point = Point3D(
            self.platform_radius / 2, 0 - self.platform_radius / 2, self.platform_height
        )
        top_left_corner_point = Point3D(
            0 - self.platform_radius / 2, self.platform_radius / 2, self.platform_height
        )
        top_right_corner_point = Point3D(
            self.platform_radius / 2, self.platform_radius / 2, self.platform_height
        )

        return [
            bottom_left_corner_point,
            bottom_right_corner_point,
            top_left_corner_point,
            top_right_corner_point,
        ]
