# Implement a new class Point that represents a point in 2D space.
import numpy as np


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def get_vector_to(self, other):
        return np.array([other.x - self.x, other.y - self.y])

    def distance(self, other):
        return ((self.x - other.x) ** 2 + (self.y - other.y) ** 2) ** 0.5

    def __repr__(self):
        return f"Point({self.x}, {self.y})"
