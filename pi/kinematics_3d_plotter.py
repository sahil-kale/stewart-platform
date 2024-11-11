import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class Kinematics3dPlotter:
    def __init__(self, base_attachment_points, lh, la):
        self.base_attachment_points = base_attachment_points
        self.lh = lh
        self.la = la

        # Create the figure and axes for 3D plotting
        self.fig, self.ax = plt.subplots(subplot_kw={"projection": "3d"})

        # Set initial plot parameters
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_zlabel("Z (m)")

        # Set the axis limits and store them to lock the axes
        self.x_lim = [-0.1, 0.1]
        self.y_lim = [-0.1, 0.1]
        self.z_lim = [0, 0.1]

        self.ax.set_xlim(self.x_lim)
        self.ax.set_ylim(self.y_lim)
        self.ax.set_zlim(self.z_lim)

    def compute_linkage_points(self, base_attachment_points, servo_angles, lh):
        """
        Computes the 3D linkage points for each servo based on the servo angles and horn length.
        """
        linkage_points = []
        for i, attachment_point in enumerate(base_attachment_points):
            theta1, _ = servo_angles[i]

            # Make a servo horn vector
            servo_horn_vector = np.array([lh * np.cos(theta1), 0, lh * np.sin(theta1)])

            # Compute the direction vector for the attachment point
            direction_vector = attachment_point / np.linalg.norm(attachment_point)

            # Project the direction vector onto the XY-plane
            direction_xy = direction_vector[:2]  # Take only the X and Y components
            direction_xy = direction_xy / np.linalg.norm(direction_xy)  # Normalize

            # Find the angle between the x-axis and the projected direction vector in the XY-plane
            angle = np.arctan2(direction_xy[1], direction_xy[0])  # Angle in radians

            # Define a rotation matrix to rotate around the z-axis by the computed angle
            rotation_matrix_z = np.array(
                [
                    [np.cos(angle), -np.sin(angle), 0],
                    [np.sin(angle), np.cos(angle), 0],
                    [0, 0, 1],
                ]
            )

            # Apply the rotation matrix to the servo horn vector
            rotated_servo_horn_vector = np.dot(rotation_matrix_z, servo_horn_vector)

            # Compute the linkage point
            linkage_point = attachment_point + rotated_servo_horn_vector
            linkage_points.append(linkage_point)

        return np.array(linkage_points)

    def plot_kinematics(self, desired_platform_points_in_base_frame, linkage_points):
        """
        Plots the kinematic setup: base points, transformed platform points, servo vectors, and servo legs.
        """
        self.ax.clear()

        # Plot the base attachment_points plane
        attachment_x, attachment_y, attachment_z = self.base_attachment_points.T
        self.ax.scatter(attachment_x, attachment_y, attachment_z, color="blue", s=100)
        self.ax.plot_trisurf(
            attachment_x, attachment_y, attachment_z, color="blue", alpha=0.3
        )

        # Plot the transformed platform points (after pitch and roll)
        platform_x, platform_y, platform_z = np.array(
            desired_platform_points_in_base_frame
        ).T
        self.ax.scatter(platform_x, platform_y, platform_z, color="red", s=100)
        self.ax.plot_trisurf(platform_x, platform_y, platform_z, color="red", alpha=0.3)

        # Plot the servo vectors (from base to platform)
        for i in range(len(self.base_attachment_points)):
            self.ax.plot(
                [attachment_x[i], platform_x[i]],
                [attachment_y[i], platform_y[i]],
                [attachment_z[i], platform_z[i]],
                color="green",
                linestyle="--",
            )

        # Plot the linkage points and legs
        linkage_x, linkage_y, linkage_z = linkage_points.T
        self.ax.scatter(linkage_x, linkage_y, linkage_z, color="purple", s=100)

        # Plot servo arms (between base and linkage, and linkage and platform)
        for i in range(len(self.base_attachment_points)):
            self.ax.plot(
                [attachment_x[i], linkage_x[i]],
                [attachment_y[i], linkage_y[i]],
                [attachment_z[i], linkage_z[i]],
                color="cyan",
                linestyle="-",
            )
            self.ax.plot(
                [linkage_x[i], platform_x[i]],
                [linkage_y[i], platform_y[i]],
                [linkage_z[i], platform_z[i]],
                color="magenta",
                linestyle=":",
            )

        self.ax.set_title("Platform Kinematics Visualization")

    def update(self, platform_points, servo_angles):
        """
        This method is intended to be called in every control loop iteration
        to update the plot.
        """
        # Compute the servo angles and linkage points
        desired_platform_points_in_base_frame = platform_points
        linkage_points = self.compute_linkage_points(
            self.base_attachment_points, servo_angles, self.lh
        )

        # Update the plot with new values
        self.plot_kinematics(
            desired_platform_points_in_base_frame,
            linkage_points,
        )

        # Reapply the axis limits to lock the axes
        self.ax.set_xlim(self.x_lim)
        self.ax.set_ylim(self.y_lim)
        self.ax.set_zlim(self.z_lim)
