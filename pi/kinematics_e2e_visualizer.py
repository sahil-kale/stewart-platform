import platform_kinematics_module
import servo_kinematics_module
import servo_kinematics_feeder
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider

# Define constants for pitch and roll maximums
PITCH_MAX = 14  # degrees
ROLL_MAX = 14  # degrees
PLATFORM_ATTACHMENT_RADIUS = 80 / 1000  # meters


def compute_servo_angles(
    attachment_points, platform_points, platform_height_from_base_m, lh, la
):
    """
    Computes the servo angles given the attachment points, platform points, and kinematic parameters.
    """
    # Create the servo kinematics feeder module
    sf = servo_kinematics_feeder.ServoKinematicsFeeder(
        platform_height_from_base_m, attachment_points
    )

    # Compute the servo vectors and desired platform points in the base frame
    servo_vector, desired_platform_points_in_base_frame = sf.compute_servo_vector(
        platform_points
    )

    # Create the servo kinematics module
    skm = servo_kinematics_module.ServoKinematicsModule(lh, la)

    # Compute the servo angles for each servo vector
    servo_angles = []
    for i in range(len(servo_vector)):
        servo_angle = skm.compute_servo_angle(np.linalg.norm(servo_vector[i]))
        servo_angles.append(servo_angle)

    return servo_angles, desired_platform_points_in_base_frame


def compute_linkage_points(attachment_points, servo_angles, lh):
    """
    Computes the 3D linkage points for each servo based on the servo angles and horn length.
    """
    linkage_points = []
    for i, attachment_point in enumerate(attachment_points):
        theta1, _ = servo_angles[i]

        # Make a servo horn vector. For now, specify the frame as x being forward, y being left (0), and z being height
        servo_horn_vector = np.array([lh * np.cos(theta1), 0, lh * np.sin(theta1)])

        # Compute the direction vector for the attachment point
        direction_vector = attachment_point / np.linalg.norm(attachment_point)

        # Project the direction vector onto the XY-plane by ignoring the z component
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


def plot_kinematics(
    attachment_points, desired_platform_points_in_base_frame, linkage_points, ax
):
    """
    Plots the kinematic setup: base points, transformed platform points, servo vectors, and servo legs.
    """
    ax.clear()

    # Plot the base (attachment_points) plane
    attachment_x, attachment_y, attachment_z = attachment_points.T
    ax.scatter(
        attachment_x,
        attachment_y,
        attachment_z,
        color="blue",
        label="Base Points (Attachment)",
        s=100,
    )
    ax.plot_trisurf(attachment_x, attachment_y, attachment_z, color="blue", alpha=0.3)

    # Plot the transformed platform points (after pitch and roll)
    platform_x, platform_y, platform_z = np.array(
        desired_platform_points_in_base_frame
    ).T
    ax.scatter(
        platform_x,
        platform_y,
        platform_z,
        color="red",
        label="Transformed Platform Points",
        s=100,
    )
    ax.plot_trisurf(platform_x, platform_y, platform_z, color="red", alpha=0.3)

    # Plot the servo vectors (from base to platform)
    for i in range(len(attachment_points)):
        ax.plot(
            [attachment_x[i], platform_x[i]],
            [attachment_y[i], platform_y[i]],
            [attachment_z[i], platform_z[i]],
            color="green",
            label="Servo Vector" if i == 0 else "",
            linestyle="--",
        )

    # Plot the linkage points and legs
    linkage_x, linkage_y, linkage_z = linkage_points.T
    ax.scatter(
        linkage_x, linkage_y, linkage_z, color="purple", label="Linkage Points", s=100
    )

    # Plot servo arms (between base and linkage, and linkage and platform)
    for i in range(len(attachment_points)):
        # From base to linkage point
        ax.plot(
            [attachment_x[i], linkage_x[i]],
            [attachment_y[i], linkage_y[i]],
            [attachment_z[i], linkage_z[i]],
            color="cyan",
            linestyle="-",
            label="Servo Leg" if i == 0 else "",
        )

        # From linkage point to platform point
        ax.plot(
            [linkage_x[i], platform_x[i]],
            [linkage_y[i], platform_y[i]],
            [linkage_z[i], platform_z[i]],
            color="magenta",
            linestyle=":",
            label="Linkage to Platform" if i == 0 else "",
        )

    # Set plot labels and title
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("Platform Kinematics Visualization")

    # Adjust plot limits for better visibility
    ax.set_xlim([-0.1, 0.1])
    ax.set_ylim([-0.1, 0.1])
    ax.set_zlim([0, 0.1])

    # Show the legend
    # ax.legend()


def update(val):
    """Updates the plot when sliders are changed."""
    pitch_angle = slider_pitch.val
    roll_angle = slider_roll.val

    # Compute the transformed platform points after applying pitch and roll
    platform_points = pk.compute_transformed_platform_attachment_points(
        np.deg2rad(roll_angle), np.deg2rad(pitch_angle)
    )

    # Compute the servo angles
    servo_angles, desired_platform_points_in_base_frame = compute_servo_angles(
        attachment_points, platform_points, platform_height_from_base_m, lh, la
    )

    # Compute the linkage points
    linkage_points = compute_linkage_points(attachment_points, servo_angles, lh)

    # Update the plot with new values
    plot_kinematics(
        attachment_points, desired_platform_points_in_base_frame, linkage_points, ax
    )
    fig.canvas.draw_idle()


if __name__ == "__main__":
    # Define the attachment points of the platform in a triangular configuration, centered at the origin
    attachment_points = [
        [1.0, 0.0, 0.0],
        [-0.5, 0.86602540378, 0.0],
        [-0.5, -0.86602540378, 0.0],
    ]

    # Convert to np array
    attachment_points = np.array(attachment_points)

    # Define servo attachment length and scale the attachment points (point at which it attaches on platform)
    servo_attachment_length_m = PLATFORM_ATTACHMENT_RADIUS
    attachment_points = attachment_points * servo_attachment_length_m

    # Create the platform kinematics module
    pk = platform_kinematics_module.PlatformKinematicsModule(attachment_points)

    # Define the initial pitch and roll angles (in degrees) and convert them to radians
    initial_pitch = 0
    initial_roll = 0

    # Define the height of the platform from the base
    platform_height_from_base_m = 60 / 1000

    # Kinematic parameters for the servos
    lh = 45 / 1000  # Length of servo horn in meters
    la = 51 / 1000  # Length of arm linkage in meters

    # Initial servo angle computation
    platform_points = pk.compute_transformed_platform_attachment_points(
        np.deg2rad(initial_roll), np.deg2rad(initial_pitch)
    )

    servo_angles, desired_platform_points_in_base_frame = compute_servo_angles(
        attachment_points, platform_points, platform_height_from_base_m, lh, la
    )

    linkage_points = compute_linkage_points(attachment_points, servo_angles, lh)

    # Create the plot
    fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
    plt.subplots_adjust(left=0.1, bottom=0.35)  # Adjust space for sliders

    # Plot the initial kinematics
    plot_kinematics(
        attachment_points, desired_platform_points_in_base_frame, linkage_points, ax
    )

    # Create sliders for pitch and roll
    ax_pitch = plt.axes([0.1, 0.15, 0.8, 0.03], facecolor="lightgoldenrodyellow")
    ax_roll = plt.axes([0.1, 0.05, 0.8, 0.03], facecolor="lightgoldenrodyellow")

    slider_pitch = Slider(
        ax_pitch, "Pitch", -PITCH_MAX, PITCH_MAX, valinit=initial_pitch
    )
    slider_roll = Slider(ax_roll, "Roll", -ROLL_MAX, ROLL_MAX, valinit=initial_roll)

    # Call update function when sliders change
    slider_pitch.on_changed(update)
    slider_roll.on_changed(update)

    # Show the plot with sliders
    plt.show()
