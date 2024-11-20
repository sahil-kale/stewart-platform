from point import Point3D
from trajectory_reference import PathPoint


def generate_square_path(
    platform_height: float, side_length: float, time_per_side: float
):
    return [
        PathPoint(Point3D(0, 0, platform_height), 0),
        PathPoint(Point3D(0, side_length, platform_height), 1 * time_per_side),
        PathPoint(
            Point3D(side_length, side_length, platform_height), 2 * time_per_side
        ),
        PathPoint(Point3D(side_length, 0, platform_height), 3 * time_per_side),
        PathPoint(Point3D(0, 0, platform_height), 4 * time_per_side),
    ]
