from dataclasses import dataclass, field

from adapters.turtle_adapter import TurtleAdapter, TurtleAdapterNode, update_env_status_from_phase
from workflow.types import EnvStatus, Phase


@dataclass
class FakePublisher:
    messages: list[str] = field(default_factory=list)

    def publish(self, message: str) -> None:
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



def test_turtle_adapter_node_forwards_phase_updates_to_adapter():
    node = TurtleAdapterNode.__new__(TurtleAdapterNode)
    node._env_status_pub = FakePublisher()
    node._adapter = TurtleAdapter(env_status_publisher=node._env_status_pub.publish)

    node.on_phase_message(Phase.STATUS_ALIGN.value)

    assert node._adapter.current_env_status is EnvStatus.READY
    assert node._env_status_pub.messages == [EnvStatus.READY.value]
