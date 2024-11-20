import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider


class ServoKinematicsModule:
    def __init__(self, lh: float, la: float, radial_displacement: float):
        """
        Initialize the servo kinematics module with the lengths of the upper arm and lower arm.
        :param lh: The length of the servo horn (m)
        :param la: The length of the arm (m)
        :param radial_displacement: The radial displacement of the arm (m) - positive is outward displacement
        """
        self.lh = lh  # Length of the servo horn
        self.la = la  # Length of the arm
        self.radial_displacement = radial_displacement

        self.height_dict = {}

        max_height = lh + la
        self.num_points = 1000
        for height in np.linspace(0, max_height, self.num_points):
            self.height_dict[height] = self.compute_servo_angle_optimizer(height)

    def compute_servo_angle(self, desired_height: float):
        height_keys = list(self.height_dict.keys())
        closest_height = min(height_keys, key=lambda x: abs(x - desired_height))
        return self.height_dict[closest_height]

    def compute_servo_angle_optimizer(self, desired_height: float):
        """
        Compute the servo angle required to reach the desired height.
        Assumes the servo horn and arm linkage produce a strictly "upward" vector.

        :param desired_height: The desired height (linear displacement) to reach.
        :return: Servo angle theta1 (in radians) and linkage angle theta2 (in radians).
        """

        def cost(angles):
            theta1, theta2 = angles

            # Compute height error
            height = self.lh * np.sin(theta1) + self.la * np.sin(theta2)
            height_error = (height - desired_height) ** 2
            height_error_weight = 1

            # Geometric constraint - must travel vertically
            constraint_violation = (
                self.lh * np.cos(theta1)
                - self.la * np.cos(theta2)
                - self.radial_displacement
            ) ** 2
            constraint_violation_weight = 1000

            # Combine the two terms in the cost function
            return (
                height_error * height_error_weight
                + constraint_violation_weight * constraint_violation
            )

        bounds = [(np.pi / 2, np.pi * 4 / 4), (np.pi / 2, np.pi * 4 / 4)]

        # Initial guess for the angles
        initial_guess = [np.pi * 3 / 4, np.pi * 3 / 4]

        # Solve the minimization problem with the constraint
        result = minimize(
            cost,
            initial_guess,
            bounds=bounds,
        )

        theta1_solution, theta2_solution = result.x

        return theta1_solution, theta2_solution

    def visualize_linkage(self, theta1, theta2, ax):
        """
        Visualizes the 2D linkage of the servo and arm system using forward kinematics.
        :param theta1: The angle of the servo horn (rad)
        :param theta2: The angle of the arm (rad)
        :param ax: The axis to plot on
        """
        # Compute the (x, y) positions of each joint
        joint1_x = 0  # Servo base is at the origin
        joint1_y = 0

        joint2_x = self.lh * np.cos(np.pi - theta1)  # End of the servo horn
        joint2_y = self.lh * np.sin(np.pi - theta1)

        end_effector_x = joint2_x + self.la * np.cos(theta2)  # End of the arm
        end_effector_y = joint2_y + self.la * np.sin(theta2)

        # Clear the previous plot
        ax.clear()

        # Plot the linkage
        ax.plot([joint1_x, joint2_x], [joint1_y, joint2_y], "ro-", label="Servo Horn")
        ax.plot(
            [joint2_x, end_effector_x],
            [joint2_y, end_effector_y],
            "bo-",
            label="Arm Linkage",
        )
        ax.scatter(
            [joint1_x, joint2_x, end_effector_x],
            [joint1_y, joint2_y, end_effector_y],
            c="k",
        )  # Joints
        ax.set_xlim([-0.08, 0.08])
        ax.set_ylim([0, 0.1])
        ax.set_xlabel("X Position (m)")
        ax.set_ylabel("Y Position (m)")
        ax.set_title("2D Visualization of Servo and Arm Linkage")
        ax.legend()
        ax.grid(True)


def update(val):
    """Update the plot when the slider is moved."""
    desired_height = slider.val
    theta1, theta2 = skm.compute_servo_angle(desired_height)
    skm.visualize_linkage(theta1, theta2, ax)
    fig.canvas.draw_idle()


if __name__ == "__main__":
    # Create the servo kinematics module
    skm = ServoKinematicsModule(
        lh=67.5 / 1000, la=51 / 1000, radial_displacement=-30 / 1000
    )

    # Create the plot
    fig, ax = plt.subplots()
    plt.subplots_adjust(left=0.1, bottom=0.25)  # Adjust plot to make room for slider

    # Initial height and initial angles
    initial_height = 59 / 1000
    theta1, theta2 = skm.compute_servo_angle(initial_height)

    # Visualize the initial linkage
    skm.visualize_linkage(theta1, theta2, ax)

    # Create the slider
    ax_slider = plt.axes([0.1, 0.1, 0.8, 0.03], facecolor="lightgoldenrodyellow")
    slider = Slider(
        ax_slider, "Height", 0.01, 0.12, valinit=initial_height, valstep=0.001
    )

    # Call update function when slider value is changed
    slider.on_changed(update)

    # Show the plot with the slider
    plt.show()
