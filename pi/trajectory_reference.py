import numpy as np
from point import Point3D


class PathPoint:
    def __init__(self, point: Point3D, time: float):
        self.point = point
        self.time = time


class TrajectoryReference:
    def __init__(self, path: list[PathPoint]):
        self.path = path

    def get_desired_point(self, time: float) -> Point3D:
        time = np.clip(time, 0, self.path[-1].time)
        # find the index of the path point that is closest to the desired time
        closest_index = 0
        for i in range(len(self.path)):
            if self.path[i].time >= time:
                break
            else:
                closest_index = i

        # interpolate between the two closest points
        current_point = self.path[closest_index].point
        next_point = self.path[closest_index + 1].point
        time_diff = self.path[closest_index + 1].time - self.path[closest_index].time
        ratio = (time - self.path[closest_index].time) / time_diff

        # get the vector between the current and next point
        vector = current_point.get_vector_to(next_point)
        vector = vector * ratio

        # add the vector to the current point to get the desired point
        desired_point = Point3D(
            current_point.x + vector[0],
            current_point.y + vector[1],
            current_point.z + vector[2],
        )

        return desired_point
