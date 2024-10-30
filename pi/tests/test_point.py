import sys
import os
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../")))
from point import Point


def test_point_distance():
    p1 = Point(0, 0)
    p2 = Point(3, 4)
    assert p1.distance(p2) == 5


def test_vector_to():
    p1 = Point(0, 0)
    p2 = Point(3, 4)
    assert np.allclose(p1.get_vector_to(p2), [3, 4])
