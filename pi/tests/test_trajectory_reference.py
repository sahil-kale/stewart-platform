import sys
import os
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../")))

from trajectory_reference import TrajectoryReference
from point import Point, Point3D
from trajectory_reference import PathPoint

square_path = [
    PathPoint(Point3D(0, 0, 0), 0),
    PathPoint(Point3D(0, 1, 1), 1),
    PathPoint(Point3D(1, 1, 2), 2),
    PathPoint(Point3D(1, 0, 3), 3),
    PathPoint(Point3D(0, 0, 4), 4),
]


def test_trajectory_reference_interpolation():
    trajectory_reference = TrajectoryReference(square_path)
    assert trajectory_reference.get_desired_point(-1) == Point3D(0, 0, 0)
    assert trajectory_reference.get_desired_point(0) == Point3D(0, 0, 0)
    assert trajectory_reference.get_desired_point(0.5) == Point3D(0, 0.5, 0.5)
    assert trajectory_reference.get_desired_point(1) == Point3D(0, 1, 1)
    assert trajectory_reference.get_desired_point(1.5) == Point3D(0.5, 1, 1.5)
    assert trajectory_reference.get_desired_point(2) == Point3D(1, 1, 2)
    assert trajectory_reference.get_desired_point(2.5) == Point3D(1, 0.5, 2.5)
    assert trajectory_reference.get_desired_point(3) == Point3D(1, 0, 3)
    assert trajectory_reference.get_desired_point(3.5) == Point3D(0.5, 0, 3.5)
    assert trajectory_reference.get_desired_point(4) == Point3D(0, 0, 4)
    assert trajectory_reference.get_desired_point(5) == Point3D(0, 0, 4)
