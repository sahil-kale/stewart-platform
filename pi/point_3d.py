# Implement a new class Point that represents a point in 2D space.
import numpy as np


class Point3D:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def get_vector_to(self, other):
        return np.array([other.x - self.x, other.y - self.y, other.z - self.z])

    def distance(self, other):
        return (
            (self.x - other.x) ** 2 + (self.y - other.y) ** 2 + (self.z - other.z) ** 2
        ) ** 0.5

    def __repr__(self):
        return f"Point({self.x}, {self.y}, {self.z})"
