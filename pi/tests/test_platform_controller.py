import sys
import os

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

    assert os.read(fakeser[1], 6) == bytes