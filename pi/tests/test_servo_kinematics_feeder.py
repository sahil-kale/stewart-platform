import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../")))

import servo_kinematics_feeder

import numpy as np


def test_point_translation():
    test_height = 1
    test_points = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]

    skf = servo_kinematics_feeder.ServoKinematicsFeeder(test_height, test_points)

    # expect that the points are translated by the height
    expected_points = [[1, 0, 1], [0, 1, 1], [0, 0, 2]]

    servo_vectors, desired_platform_points_in_base_frame = skf.compute_servo_vector(
        test_points
    )

    assert np.allclose(desired_platform_points_in_base_frame, expected_points)
