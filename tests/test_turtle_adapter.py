from dataclasses import dataclass, field
from types import SimpleNamespace

from adapters.turtle_adapter import TurtleAdapter, TurtleAdapterNode, update_env_status_from_phase
from workflow.types import EnvStatus, Phase


@dataclass
class FakePublisher:
    messages: list[object] = field(default_factory=list)

    def publish(self, message: object) -> None:
        self.messages.append(message)



def test_update_env_status_from_phase_returns_ready_for_status_align():
    assert update_env_status_from_phase(Phase.STATUS_ALIGN) is EnvStatus.READY



def test_update_env_status_from_phase_returns_idle_for_non_running_phase():
    assert update_env_status_from_phase(Phase.READY) is EnvStatus.IDLE
    assert update_env_status_from_phase(Phase.DONE) is EnvStatus.IDLE



def test_turtle_adapter_updates_and_publishes_env_status_from_phase():
    publisher = FakePublisher()
    adapter = TurtleAdapter(env_status_publisher=publisher.publish)

    adapter.on_phase(Phase.STATUS_ALIGN)
    adapter.on_phase(Phase.DONE)

    assert adapter.current_env_status is EnvStatus.IDLE
    assert publisher.messages == [EnvStatus.READY.value, EnvStatus.IDLE.value]



def test_turtle_adapter_maps_linear_y_to_forward_velocity():
    cmd_publisher = FakePublisher()
    adapter = TurtleAdapter(
        env_status_publisher=FakePublisher().publish,
        turtle_cmd_publisher=cmd_publisher.publish,
    )

    adapter.on_cmd_vel(SimpleNamespace(linear=SimpleNamespace(y=0.18), angular=SimpleNamespace(z=0.0)))

    assert cmd_publisher.messages == [
        {"linear_x": 0.18, "angular_z": 0.0}
    ]



def test_turtle_adapter_node_forwards_phase_updates_to_adapter():
    node = TurtleAdapterNode.__new__(TurtleAdapterNode)
    node._env_status_pub = FakePublisher()
    node._adapter = TurtleAdapter(env_status_publisher=node._env_status_pub.publish)

    node.on_phase_message(Phase.STATUS_ALIGN.value)

    assert node._adapter.current_env_status is EnvStatus.READY
    assert node._env_status_pub.messages == [EnvStatus.READY.value]



def test_turtle_adapter_node_forwards_cmd_vel_to_adapter():
    node = TurtleAdapterNode.__new__(TurtleAdapterNode)
    node._cmd_pub = FakePublisher()
    node._adapter = TurtleAdapter(
        env_status_publisher=FakePublisher().publish,
        turtle_cmd_publisher=node._cmd_pub.publish,
    )

    node.on_cmd_vel_message(
        SimpleNamespace(linear=SimpleNamespace(y=-0.12), angular=SimpleNamespace(z=0.25))
    )

    assert node._cmd_pub.messages == [
        {"linear_x": -0.12, "angular_z": 0.25}
    ]
