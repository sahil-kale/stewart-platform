import numpy as np
import matplotlib.pyplot as plt

MAX_PLATFORM_DISTANCE = 0.15


def get_gain(x, y):
    max_gain = 2.2
    distance = np.sqrt(x**2 + y**2)

    # do a linear interpolation between 0 and max_platform_distance. 0 maps to max_gain, max_platform_distance maps to 0
    ki = np.interp(distance, [0, MAX_PLATFORM_DISTANCE], [max_gain, 0])
    return ki


def plot_gain():
    x = np.linspace(-MAX_PLATFORM_DISTANCE, MAX_PLATFORM_DISTANCE, 100)
    y = np.linspace(-MAX_PLATFORM_DISTANCE, MAX_PLATFORM_DISTANCE, 100)
    X, Y = np.meshgrid(x, y)
    Z = get_gain(X, Y)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.plot_surface(X, Y, Z, cmap="viridis")

    ax.set_xlabel("X Position (m)")
    ax.set_ylabel("Y Position (m)")
    ax.set_zlabel("Gain")
    plt.show()


if __name__ == "__main__":
    plot_gain()
