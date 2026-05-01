import importlib.util
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT / "scripts" / "move.py"


def load_module():
    spec = importlib.util.spec_from_file_location("move", SCRIPT_PATH)
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class FakeFloat32MultiArray:
    def __init__(self):
        self.data = []


def test_parse_key_value_args_supports_robot_axis_probe_command():
    module = load_module()

    command = module.parse_key_value_args(
        ["x_vel=0.1", "y_vel=-0.2", "t=0.15", "topic=/test_cmd", "rate_hz=25"]
    )

    assert command.x_vel == 0.1
    assert command.y_vel == -0.2
    assert command.angular_z == 0.0
    assert command.duration_sec == 0.15
    assert command.topic == "/test_cmd"
    assert command.rate_hz == 25.0


def test_parse_key_value_args_defaults_to_robotfetch_electric_topic():
    module = load_module()

    command = module.parse_key_value_args(["x_vel=0.1"])

    assert command.topic == "/t0x0101_robotfetch"


def test_parse_key_value_args_rejects_unknown_keys():
    module = load_module()

    try:
        module.parse_key_value_args(["x_vel=0.1", "seconds=1"])
    except ValueError as exc:
        assert "unknown key" in str(exc)
        assert "seconds" in str(exc)
    else:
        raise AssertionError("expected ValueError")


def test_parse_key_value_args_rejects_unsupported_linear_z_velocity():
    module = load_module()

    try:
        module.parse_key_value_args(["z_vel=0.1"])
    except ValueError as exc:
        assert "unknown key" in str(exc)
        assert "z_vel" in str(exc)
    else:
        raise AssertionError("expected ValueError")


def test_parse_key_value_args_rejects_empty_command():
    module = load_module()

    try:
        module.parse_key_value_args([])
    except ValueError as exc:
        assert "at least one" in str(exc)
    else:
        raise AssertionError("expected ValueError")


def test_build_float32_multi_array_sets_requested_velocity_and_stop_message():
    module = load_module()
    command = module.MoveCommand(x_vel=0.1, y_vel=-0.2, angular_z=-0.4)

    message = module.build_float32_multi_array(FakeFloat32MultiArray, command)
    stop = module.build_float32_multi_array(
        FakeFloat32MultiArray,
        command,
        stop=True,
    )

    assert message.data == [0.1, -0.2, -0.4]
    assert stop.data == [0.0, 0.0, 0.0]
