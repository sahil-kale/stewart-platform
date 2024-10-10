import numpy as np
from platform_kinematics_module import PlatformKinematicsModule
from servo_kinematics_module import ServoKinematicsModule
from servo_kinematics_feeder import ServoKinematicsFeeder
from kinematics_3d_plotter import Kinematics3dPlotter
import argparse
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider


class MainControlLoop:
    def __init__(self, run_visualizer: bool = False, run_controller: bool = False):
        self.params = {
            "lh": 41 / 1000,
            "la": 51 / 1000,
            "platform_attachment_radius": 100 / 1000,
            "control_loop_frequency": 100,
        }

        attachment_points_structure = [
            [1.0, 0.0, 0.0],
            [-0.5, 0.86602540378, 0.0],
            [-0.5, -0.86602540378, 0.0],
        ]

        self.attachment_points = np.array(attachment_points_structure)
        self.attachment_points = (
            self.attachment_points * self.params["platform_attachment_radius"]
        )

        self.pk = PlatformKinematicsModule(self.attachment_points)
        self.sk = ServoKinematicsModule(self.params["lh"], self.params["la"])

        self.run_visualizer = run_visualizer
        self.run_controller = run_controller

        if self.run_visualizer:
            self.visualizer = Kinematics3dPlotter(
                self.attachment_points, self.params["lh"], self.params["la"]
            )
            self.create_sliders()

    def create_sliders(self):
        """Create sliders for pitch, roll, and height"""
        ax_pitch = plt.axes([0.15, 0.02, 0.65, 0.03], facecolor="lightgoldenrodyellow")
        ax_roll = plt.axes([0.15, 0.06, 0.65, 0.03], facecolor="lightgoldenrodyellow")
        ax_height = plt.axes([0.15, 0.10, 0.65, 0.03], facecolor="lightgoldenrodyellow")

        self.slider_pitch = Slider(ax_pitch, "Pitch", -14.0, 14.0, valinit=0.0)
        self.slider_roll = Slider(ax_roll, "Roll", -14.0, 14.0, valinit=0.0)
        self.slider_height = Slider(ax_height, "Height", 0.02, 0.1, valinit=0.06)

    def run(self):
        while True:
            # Get pitch, roll, and height from the sliders
            if self.run_controller:
                # TODO: integrate real control logic
                pass
            else:
                pitch_rad = np.deg2rad(self.slider_pitch.val)
                roll_rad = np.deg2rad(self.slider_roll.val)
                height = self.slider_height.val

            # Compute the platform's new attachment points
            platform_points = self.pk.compute_transformed_platform_attachment_points(
                roll_rad, pitch_rad
            )
            skf = ServoKinematicsFeeder(height, self.attachment_points)
            servo_vectors, platform_points_in_base_frame = skf.compute_servo_vector(
                platform_points
            )

            # Compute the servo angles
            servo_angles = []
            for i in range(len(servo_vectors)):
                desired_servo_length = np.linalg.norm(servo_vectors[i])
                servo_angle = self.sk.compute_servo_angle(desired_servo_length)
                servo_angles.append(servo_angle)

            # TODO: send servo angles to servo controller

            # Update visualization if enabled
            if self.run_visualizer:
                self.visualizer.update(platform_points_in_base_frame, servo_angles)
                # Redraw the figure
                plt.pause(0.01)  # Adjust the pause to suit real-time control loop speed


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--visualize", action="store_true", help="Run the 3D visualization"
    )
    args = parser.parse_args()

    mcl = MainControlLoop(run_visualizer=args.visualize)
    mcl.run()
