import numpy as np
from point import Point


class PlatformKinematicsFeeder:
    def __init__(self, debug=False):
        self.debug = debug

    def calculate_desired_platform_height(
        self, ball_height, ball_coords: Point, theta_x, theta_y
    ):
        # theta_x is the angle of the platform in the x direction in rad (roll)
        # theta_y is the angle of the platform in the y direction in rad (pitch)

        vector_x = np.array([ball_coords.x, 0, ball_coords.x * -np.tan(theta_x)])
        vector_y = np.array([0, ball_coords.y, ball_coords.y * np.tan(theta_y)])

        vector = vector_x + vector_y

        # define a vector straight up, representing a flat plane
        vector_up = np.array([0, 0, 1])

        # calculate the angle between the two vectors
        # cos theta = (a . b) / (|a| |b|)
        angle = np.arccos(
            np.dot(vector, vector_up)
            / (np.linalg.norm(vector) * np.linalg.norm(vector_up))
        )

        angle = (
            np.pi / 2 - angle
        )  # subtract from 90 degrees to get the angle between plane and vector

        # calculate the height of the platform using a simple trigonometric relation
        # the norm of vector is the hypotenuse of the triangle
        # the height is the opposite side from the angle

        height = np.sin(angle) * np.linalg.norm(vector)

        platform_height = ball_height - height

        if self.debug:
            print(
                f"Ball height: {ball_height}, Ball coords: {ball_coords}, Theta x: {theta_x}, Theta y: {theta_y}, Platform height: {platform_height}"
            )  # clip the platform height to a minimum of 0
        return max(platform_height, 0)


if __name__ == "__main__":
    # Test the platform kinematics feeder
    pkf = PlatformKinematicsFeeder()

    height = 0.18

    ball_coords = Point(0.08, 0)
    theta_x = np.deg2rad(-5.7)
    theta_y = np.deg2rad(0)

    platform_height = pkf.calculate_desired_platform_height(
        height, ball_coords, theta_x, theta_y
    )
