import numpy as np
from platform_controller import PlatformController
from platform_kinematics_module import PlatformKinematicsModule
from camera import Camera
from point import Point, lpf_point
from servo_kinematics_module import ServoKinematicsModule
from servo_kinematics_feeder import ServoKinematicsFeeder
from kinematics_3d_plotter import Kinematics3dPlotter
from ball_controller import BallController
import argparse
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from kalman_filter import KalmanFilter
import time
import json
from platform_kinematics_feeder import PlatformKinematicsFeeder
from trajectory_reference import TrajectoryReference, PathPoint
from paths import generate_square_path
from point import Point3D

import os, pty


def create_fake_serial():
    master, slave = pty.openpty()
    s_name = os.ttyname(slave)
    return s_name, master


class MainControlLoop:
    def __init__(
        self,
        serial_port: str,
        camera_port: str,
        servo_offsets: list[float] = [0, 0, 0],  # degrees
        platform_offset: list[float] = [0, 0],  # degrees
        virtual: bool = False,
        camera_debug: bool = False,
        run_visualizer: bool = False,
        run_controller: bool = True,
        print_time_profile: bool = False,
        path=None,
    ):
        self.dt = 0.02
        self.saturate_angle = 12
        self.params = {
            "lh": 67.5 / 1000,
            "la": 122.2 / 1000,
            "platform_attachment_radius": 70 / 1000,
            "base_attachment_radius": 100 / 1000,
            "resting_height": 0.15,
            "max_euclidean_distance": 0.15,
        }

        self.servo_offset_radians = [
            np.radians(servo_offsets[0]),
            np.radians(servo_offsets[1]),
            np.radians(servo_offsets[2]),
        ]

        self.platform_offset_radians = [
            np.radians(platform_offset[0]),
            np.radians(platform_offset[1]),
        ]

        attachment_points_structure = [
            [1.0, 0.0, 0.0],
            [-0.5, 0.86602540378, 0.0],
            [-0.5, -0.86602540378, 0.0],
        ]

        self.platform_attachment_points = np.array(attachment_points_structure)
        self.platform_attachment_points = (
            self.platform_attachment_points * self.params["platform_attachment_radius"]
        )

        self.base_attachment_points = np.array(attachment_points_structure)
        self.base_attachment_points = (
            self.base_attachment_points * self.params["base_attachment_radius"]
        )

        self.pk = PlatformKinematicsModule(self.platform_attachment_points)
        self.sk = ServoKinematicsModule(
            self.params["lh"],
            self.params["la"],
            self.params["base_attachment_radius"]
            - self.params["platform_attachment_radius"],
        )

        kp_range = [1.0, 1.0]
        ki_range = [[2.2, 0.0], [0, self.params["max_euclidean_distance"]]]
        kd_range = [0.55, 0.55]
        self.ball_controller = BallController(
            kp_range,
            ki_range,
            kd_range,
            kp_range,
            ki_range,
            kd_range,
            self.dt,
            np.deg2rad(self.saturate_angle),
            max_euclidean_error=0.15,
            integral_windup_clear_threshold=10,
            stiction_compensation_deadband=0.005,
            stiction_compensation_feedforward=0.00,
            integral_clear_threshold=0.20,
        )

        self.run_visualizer = run_visualizer
        self.run_controller = run_controller

        if self.run_visualizer:
            self.visualizer = Kinematics3dPlotter(
                self.base_attachment_points, self.params["lh"], self.params["la"]
            )
            self.create_kinematic_sliders()

        self.virtual = virtual

        if self.virtual:
            serial_port = create_fake_serial()[0]

        self.pc = PlatformController(serial_port, debug=False)

        # get the current directory
        current_dir = os.path.dirname(os.path.realpath(__file__))
        # get the file camera.json with the current dir
        cv_params_file_path = os.path.join(current_dir, "camera_params.json")

        self.camera_debug = camera_debug

        with open(cv_params_file_path, "r") as file:
            data = json.load(file)  # Load JSON data as a dictionary

        if self.virtual is not True:
            self.cv_system = Camera(data["u"], data["v"], camera_port, camera_debug)
            self.cv_system.load_camera_params("pi/camera_calibration_data.json")

        self.kalman_filter = KalmanFilter(self.dt, Q=50, R=50)

        self.current_position = Point(0, 0)

        self.print_time_profile = print_time_profile

        self.path = path
        if self.path is not None:
            self.trajectory_reference = TrajectoryReference(self.path)

    def create_kinematic_sliders(self):
        """Create sliders for pitch, roll, and height"""
        ax_pitch = plt.axes([0.15, 0.02, 0.65, 0.03], facecolor="lightgoldenrodyellow")
        ax_roll = plt.axes([0.15, 0.06, 0.65, 0.03], facecolor="lightgoldenrodyellow")
        ax_height = plt.axes([0.15, 0.10, 0.65, 0.03], facecolor="lightgoldenrodyellow")

        self.slider_pitch = Slider(
            ax_pitch, "Pitch", -self.saturate_angle, self.saturate_angle, valinit=0.0
        )
        self.slider_roll = Slider(
            ax_roll, "Roll", -self.saturate_angle, self.saturate_angle, valinit=0.0
        )
        self.slider_height = Slider(
            ax_height, "Height", 0.02, 0.2, valinit=self.params["resting_height"]
        )

    def run(self):
        # create a new log file path for the current run with the current time
        # make a new directory "logs" if it doesn't exist
        if not os.path.exists("logs"):
            os.makedirs("logs")

        logfile = f"logs/log_{time.strftime('%Y-%m-%d_%H-%M-%S')}.txt"

        # open the log file in write mode
        f = open(logfile, "w")
        header_written = False

        boot_time = time.time()

        while True:
            # Get pitch, roll, and height from the sliders
            time_since_loop_iteration_start = time.time()
            camera_valid = False
            desired_position = Point(0, 0)

            if self.path is not None:
                time_since_boot = time.time() - boot_time
                desired_position = self.trajectory_reference.get_desired_point(
                    time_since_boot
                )
                desired_position = Point(desired_position.x, desired_position.y)

            if self.run_controller:
                if self.virtual is False:
                    current_measurement = self.cv_system.get_ball_coordinates()
                    time_ball_detected = time.time()

                    self.current_position = self.kalman_filter.predict()

                    if current_measurement is not None:
                        self.current_position = self.kalman_filter.update(
                            current_measurement
                        )
                        self.current_position = current_measurement
                        camera_valid = True
                    else:
                        if self.camera_debug:
                            print("Ball not detected")

                else:
                    self.current_position = Point(0, 0)

                output_angles = self.ball_controller.run_control_loop(
                    desired_position, self.current_position
                )
                time_after_ball_controller = time.time()

                pitch_rad = output_angles[0] + self.platform_offset_radians[0]
                roll_rad = output_angles[1] + self.platform_offset_radians[1]
                pkf = PlatformKinematicsFeeder()
                height = pkf.calculate_desired_platform_height(
                    self.params["resting_height"],
                    self.current_position,
                    pitch_rad,
                    roll_rad,
                )
                time_after_platform_kinematics = time.time()
            else:
                # set the pitch and roll sliders to negative whatever it was before
                # self.slider_pitch.set_val(-self.slider_pitch.val)
                # self.slider_roll.set_val(-self.slider_roll.val)

                pitch_rad = (
                    np.deg2rad(self.slider_pitch.val) + self.platform_offset_radians[0]
                )
                roll_rad = (
                    np.deg2rad(self.slider_roll.val) + self.platform_offset_radians[1]
                )
                height = self.slider_height.val

            # Compute the platform's new attachment points
            platform_points = self.pk.compute_transformed_platform_attachment_points(
                roll_rad, pitch_rad
            )
            skf = ServoKinematicsFeeder(height, self.platform_attachment_points)
            servo_vectors, platform_points_in_base_frame = skf.compute_servo_vector(
                platform_points
            )
            time_after_servo_kinematics = time.time()

            # Compute the servo angles
            servo_angles = []
            duty_cycles = []
            for i in range(len(servo_vectors)):
                desired_servo_length = np.linalg.norm(servo_vectors[i])
                servo_angle = self.sk.compute_servo_angle(desired_servo_length)
                servo_angles.append(servo_angle)
                duty_cycle = self.pc.compute_duty_cycle_from_angle(
                    servo_angle[0] + self.servo_offset_radians[i]
                )
                duty_cycles.append(duty_cycle)
            time_after_duty_cycle_computation = time.time()

            # Send the servo angles to the platform
            if not self.virtual:
                self.pc.write_duty_cycles(
                    duty_cycles[0], duty_cycles[1], duty_cycles[2]
                )
                time_after_serial_write = time.time()

            time_elapsed = time.time() - time_since_loop_iteration_start
            pause_time = self.dt - time_elapsed

            if self.print_time_profile:
                print(
                    f"Time elapsed: {time_elapsed} seconds | Time since ball detected: {time_ball_detected - time_since_loop_iteration_start} seconds | Time after platform kinematics: {time_after_platform_kinematics - time_ball_detected} seconds | Time after servo kinematics: {time_after_servo_kinematics - time_after_platform_kinematics} seconds | Time after duty cycle computation: {time_after_duty_cycle_computation - time_after_servo_kinematics} seconds | Time after serial write: {time_after_serial_write - time_after_duty_cycle_computation} seconds"
                )

            if pause_time < 0:
                pause_time = 0
            # Update visualization if enabled
            if self.run_visualizer:
                self.visualizer.update(platform_points_in_base_frame, servo_angles)
                # Redraw the figure
                plt.pause(pause_time)
            else:
                time.sleep(pause_time)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--visualize", action="store_true", help="Run the 3D visualization"
    )

    parser.add_argument(
        "--port",
        type=str,
        default="/dev/ttyACM0",
        help="The serial port for the platform controller",
    )

    parser.add_argument(
        "--camera_port",
        type=str,
        default="/dev/video0",
        help="The port for the camera",
    )

    # add an argument for virtual
    parser.add_argument(
        "--virtual",
        action="store_true",
        help="Run the control loop in virtual mode",
    )

    # add argument for CV/Camera debug
    parser.add_argument(
        "--camera_debug",
        action="store_true",
        help="Flag for showing more camera info for debugging",
    )

    parser.add_argument(
        "--inhibit_controller",
        action="store_true",
        help="Run the control loop",
    )

    parser.add_argument(
        "--cam_color_mask_detect",
        action="store_true",
        help="Run the camera calibration",
    )

    parser.add_argument(
        "--print_time_profile",
        action="store_true",
        help="Print time profile",
    )

    parser.add_argument("--path", type=str, default="none")

    args = parser.parse_args()

    run_controller = args.inhibit_controller == False

    servo_offsets = [-3, 0, 0]
    platform_offset = [-6, -4]  # degrees

    path = None

    if args.path == "square":
        path = generate_square_path(0.15, 0.1, 5)

    mcl = MainControlLoop(
        args.port,
        args.camera_port,
        virtual=args.virtual,
        camera_debug=args.camera_debug,
        run_visualizer=args.visualize,
        servo_offsets=servo_offsets,
        platform_offset=platform_offset,
        run_controller=run_controller,
        print_time_profile=args.print_time_profile,
        path=path,
    )
    mcl.run()
