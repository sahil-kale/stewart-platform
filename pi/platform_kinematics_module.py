import numpy as np
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


class PlatformKinematicsModule:
    def __init__(self, platform_attachment_points: list):
        """
        It is assumed the platform attachment points are defined in the platform's local coordinate system, with 0,0 being the centre.
        Points are given in format [x, y, z] where movement around x is defined as "roll" and movement around y is defined as "pitch".
        """
        self.platform_attachment_points = platform_attachment_points

    def compute_transformed_platform_attachment_points(self, roll, pitch):
        """
        Compute the transformed platform attachment points given the pitch and roll angles

        :param pitch: The pitch angle in radians
        :param roll: The roll angle in radians
        """

        # create the rotation matrix for roll
        roll_rotation_matrix = np.array(
            [
                [1, 0, 0],
                [0, np.cos(roll), -np.sin(roll)],
                [0, np.sin(roll), np.cos(roll)],
            ]
        )

        # create the rotation matrix for pitch
        pitch_rotation_matrix = np.array(
            [
                [np.cos(pitch), 0, np.sin(pitch)],
                [0, 1, 0],
                [-np.sin(pitch), 0, np.cos(pitch)],
            ]
        )

        # create the combined rotation matrix
        rotation_matrix = np.matmul(roll_rotation_matrix, pitch_rotation_matrix)

        # transform the attachment points
        transformed_attachment_points = []
        for attachment_point in self.platform_attachment_points:
            transformed_attachment_points.append(
                np.matmul(rotation_matrix, attachment_point)
            )

        return transformed_attachment_points


def update_plot(roll, pitch):
    """Update the 3D plot with the new roll and pitch values."""
    # Clear the plot
    ax.cla()

    # Transform the attachment points
    transformed_points = pkm.compute_transformed_platform_attachment_points(
        np.radians(roll), np.radians(pitch)
    )

    # Plot the original and transformed attachment points as 3D planes (triangles)
    original_points = np.array(attachment_points)
    transformed_points = np.array(transformed_points)

    # Create triangular planes for both original and transformed points
    original_plane = [original_points[0], original_points[1], original_points[2]]
    transformed_plane = [
        transformed_points[0],
        transformed_points[1],
        transformed_points[2],
    ]

    # Add the original triangle (blue)
    ax.add_collection3d(
        Poly3DCollection(
            [original_plane], color="blue", alpha=0.5, linewidths=1, edgecolors="b"
        )
    )

    # Add the transformed triangle (red)
    ax.add_collection3d(
        Poly3DCollection(
            [transformed_plane], color="red", alpha=0.5, linewidths=1, edgecolors="r"
        )
    )

    # Set labels and limits
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    # Plot the vectors between the original and transformed attachment points
    for i in range(len(attachment_points)):
        ax.quiver(
            original_points[i, 0],
            original_points[i, 1],
            original_points[i, 2],
            transformed_points[i, 0] - original_points[i, 0],
            transformed_points[i, 1] - original_points[i, 1],
            transformed_points[i, 2] - original_points[i, 2],
            color="g",
        )

    # Redraw the canvas
    canvas.draw()


def on_slider_change(value):
    """Callback for when the slider value changes."""
    roll = roll_slider.get()
    pitch = pitch_slider.get()
    update_plot(roll, pitch)


if __name__ == "__main__":
    # Define the attachment points of the platform, attach them in a "triangle" with the center of the platform at the origin
    attachment_points = [
        np.array([1.0, 0.0, 0.0]),
        np.array([-0.5, 0.86602540378, 0.0]),
        np.array([-0.5, -0.86602540378, 0.0]),
    ]

    # Create the platform kinematics module
    pkm = PlatformKinematicsModule(attachment_points)

    # Set up the tkinter window
    root = tk.Tk()
    root.title("Live Platform Kinematics")

    # Set up the matplotlib figure and 3D axis
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    # Create a canvas to embed the matplotlib figure into the tkinter window
    canvas = FigureCanvasTkAgg(fig, master=root)
    canvas_widget = canvas.get_tk_widget()
    canvas_widget.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

    # Create sliders for roll and pitch
    roll_slider = tk.Scale(
        root,
        from_=-180,
        to=180,
        orient="horizontal",
        label="Roll (degrees)",
        command=on_slider_change,
    )
    roll_slider.pack(side=tk.LEFT, fill=tk.X, expand=True)

    pitch_slider = tk.Scale(
        root,
        from_=-180,
        to=180,
        orient="horizontal",
        label="Pitch (degrees)",
        command=on_slider_change,
    )
    pitch_slider.pack(side=tk.RIGHT, fill=tk.X, expand=True)

    # Initialize the plot with zero roll and pitch
    update_plot(0, 0)

    # Start the tkinter main loop
    root.mainloop()
