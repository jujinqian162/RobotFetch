from types import SimpleNamespace

from adapters.robot_adapter import RobotAdapter
from workflow.types import EnvStatus, Phase


def test_robot_adapter_marks_status_align_ready_and_passthroughs_cmd_vel():
    adapter = RobotAdapter(env_status_publisher=lambda _: None)

    env_status = adapter.on_phase(Phase.STATUS_ALIGN)
    command = adapter.on_cmd_vel(
        SimpleNamespace(
            linear=SimpleNamespace(x=0.0, y=0.12, z=0.0),
            angular=SimpleNamespace(x=0.0, y=0.0, z=0.05),
        )
    )

    assert env_status is EnvStatus.READY
    assert command.linear_y == 0.12
    assert command.angular_z == 0.05
