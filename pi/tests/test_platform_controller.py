import sys
import os
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../")))

import platform_controller

import os, pty


def create_fake_serial():
    master, slave = pty.openpty()
    s_name = os.ttyname(slave)
    return s_name, master


def test_platform_controller_init():
    # Test the PlatformController class

    fakeser_name = create_fake_serial()

    pc = platform_controller.PlatformController(fakeser_name[0])
    assert pc.ser.baudrate == 9600
    assert pc.ser.port == fakeser_name[0]


def test_platform_controller_write():
    # Test the PlatformController class
    fakeser = create_fake_serial()
    pc = platform_controller.PlatformController(fakeser[0])
    pc.write_raw(b"Hello")
    assert os.read(fakeser[1], 5) == b"Hello"


def test_platform_controller_write_duty_cycles_encoded():
    # we're going to expect the platform controller to encode the duty cycles, which are in microseconds, as 3x 2-byte unsigned integers, little-endian
    fakeser = create_fake_serial()

    pc = platform_controller.PlatformController(fakeser[0])
    pc.write_duty_cycles(1000, 2000, 3000)

    bytes = bytearray()
    dutycycle_1_bytes = (1000).to_bytes(2, byteorder="little", signed=False)
    dutycycle_2_bytes = (2000).to_bytes(2, byteorder="little", signed=False)
    dutycycle_3_bytes = (3000).to_bytes(2, byteorder="little", signed=False)
    bytes.extend(dutycycle_1_bytes)
    bytes.extend(dutycycle_2_bytes)
    bytes.extend(dutycycle_3_bytes)

    # expect a checksum to be written, which is a uint32_t little endian sum of the 3 duty cycles
    checksum = sum([1000, 2000, 3000])
    checksum_bytes = checksum.to_bytes(4, byteorder="little", signed=False)
    bytes.extend(checksum_bytes)

    # get the number of bytes
    num_bytes_expected = len(bytes)

    assert os.read(fakeser[1], num_bytes_expected) == bytes


def test_duty_cycle_resolver_from_angle():
    fakeser = create_fake_serial()
    pc = platform_controller.PlatformController(fakeser[0])

    angle_ninety_degrees_duty_cycle = pc.compute_duty_cycle_from_angle(np.pi / 2)

    assert angle_ninety_degrees_duty_cycle == 1400

    angle_zero_duty_cycle = pc.compute_duty_cycle_from_angle(0)

    assert angle_zero_duty_cycle == 500

    angle_full_duty_cycle = pc.compute_duty_cycle_from_angle(np.pi)

    assert angle_full_duty_cycle == 2300
