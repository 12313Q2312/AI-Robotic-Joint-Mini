"""In-process stand-in for the ROS 2 topic contract.

The headless loop publishes the same topic names the ROS nodes use.
Nothing here opens a socket.
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class TopicBus:
    messages: list[tuple[str, object]] = field(default_factory=list)

    def publish(self, topic: str, payload: object) -> None:
        self.messages.append((topic, payload))

    def topics(self) -> set[str]:
        return {topic for topic, _payload in self.messages}
