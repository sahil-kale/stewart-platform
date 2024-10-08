import numpy as np


class ServoKinematicsFeeder:
    """
    Class responsible for determining the desired servo length request based on the platform point orientation in platform frame

    It essentially takes in both the platform point, base point, and determines the desired servo lengths for the servo kinematics module
    """

    def __init__(self, platform_to_base_height: float, base_attachment_points: list):
        """
        Initialize the servo kinematics feeder with the height of the platform from the base. Note that this assumes that each point is 1:1 with the base attachment points
        """
        self.platform_to_base_height = platform_to_base_height

        self.platform_T_base = np.array(
            [
                [1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, platform_to_base_height],
                [0, 0, 0, 1],
            ]
        )

        self.base_attachment_points = base_attachment_points

    def compute_servo_vector(self, desired_platform_points: list):
        """
        Compute the servo lengths required to reach the desired platform points

        NOTE: assumption is that the platform points are 1:1 with the previously defined base attachment points
        """

        # convert the platform points to base frame
        desired_platform_points_in_base_frame = []

        servo_lengths = []

        for i in range(len(desired_platform_points)):
            platform_point = desired_platform_points[i]
            # add an extra 1 to the end to make it a 4x1 vector
            platform_point = np.append(platform_point, 1)
            desired_platform_point_in_base_frame = np.matmul(
                self.platform_T_base, platform_point
            )
            # remove the 1 at the end
            desired_platform_point_in_base_frame = desired_platform_point_in_base_frame[
                :3
            ]
            desired_platform_points_in_base_frame.append(
                desired_platform_point_in_base_frame
            )

            # compute the servo lengths by simply taking the difference between the desired platform point and the base attachment point at that index
            base_point = np.array(self.base_attachment_points[i])
            servo_lengths.append(desired_platform_point_in_base_frame - base_point)

        return servo_lengths, desired_platform_points_in_base_frame
