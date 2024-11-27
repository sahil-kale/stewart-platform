import sys
import os
import numpy as np
import pytest

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../")))
from anti_stiction import AntiStictionController


def test_anti_stiction_controller_init():
    anti_stiction_controller = AntiStictionController(0.1, 0.1, 0.05, 0.1)
    assert anti_stiction_controller.state == AntiStictionController.State.STATIC


def test_anti_stiction_controller_fire_when_over_threshold():
    anti_stiction_controller = AntiStictionController(0.1, 0.1, 0.05, 0.1)
    asc_result = anti_stiction_controller.run(0.09, 0.1)
    assert anti_stiction_controller.state == AntiStictionController.State.STATIC
    assert asc_result == 0
    asc_result = anti_stiction_controller.run(0.2, 0.2)
    assert anti_stiction_controller.state == AntiStictionController.State.FIRED
    assert asc_result == 0.1


def test_anti_stiction_controller_decay():
    anti_stiction_controller = AntiStictionController(0.1, 0.1, 0.05, 0.1)
    anti_stiction_controller.run(0.2, 0.2)
    assert anti_stiction_controller.state == AntiStictionController.State.FIRED
    asc_result = anti_stiction_controller.run(0.2, 0.25)
    assert asc_result == pytest.approx(0.075)
    asc_result = anti_stiction_controller.run(0.2, 0.3)
    assert asc_result == pytest.approx(0.05)
    asc_result = anti_stiction_controller.run(0.2, 0.4)
    assert asc_result == pytest.approx(0.05)


def test_anti_stiction_static_if_output_below_threshold():
    anti_stiction_controller = AntiStictionController(0.1, 0.1, 0.05, 0.1)
    anti_stiction_controller.run(0.2, 0.2)
    assert anti_stiction_controller.state == AntiStictionController.State.FIRED
    asc_result = anti_stiction_controller.run(0.09, 0.25)
    assert asc_result == pytest.approx(0)
    assert anti_stiction_controller.state == AntiStictionController.State.STATIC
