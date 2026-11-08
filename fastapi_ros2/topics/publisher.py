"""Publish JSON messages to any topic."""

from typing import Any, final

from rclpy.exceptions import InvalidTopicNameException
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.validate_full_topic_name import validate_full_topic_name

from fastapi_ros2.topics.messages import message_type, to_message


class PublishError(Exception):
    """The request can't be published, for the reason in the message."""


# Publishers of every message type share the dict, so only `Any` fits them all.
type _AnyPublisher = Publisher[Any]  # pyright: ignore[reportExplicitAny]


@final
class TopicPublisher:
    """Publish JSON messages to any topic, as `ros2 topic pub --once` does."""

    def __init__(self, node: Node):
        self.node = node
        # ponytail: a publisher per topic and type ever published to, never
        # removed; evict the least recently used if clients invent many topics.
        self._publishers: dict[tuple[str, str], _AnyPublisher] = {}

    def publish(
        self,
        topic: str,
        message: dict[str, object],
        type_name: str | None,
    ) -> str:
        """Publish a message to a fully qualified topic, returning its type.

        Without a type, the topic's type in the graph is used.
        """
        try:
            validate_full_topic_name(topic)
        except InvalidTopicNameException:
            raise PublishError(f'"{topic}" is not a valid topic name') from None
        type_name = self._type(topic, type_name)
        cls = message_type(type_name)
        ros_message = to_message(cls, message)
        publisher = self._publishers.get((topic, type_name))
        if publisher is None:
            publisher = self.node.create_publisher(cls, topic, 10)
            self._publishers[(topic, type_name)] = publisher
        publisher.publish(ros_message)
        return type_name

    def _type(self, topic: str, type_name: str | None) -> str:
        """The message type to publish, checked against the topic's in the graph."""
        types = dict(self.node.get_topic_names_and_types()).get(topic, [])
        if type_name is None:
            if len(types) == 1:
                return types[0]
            if not types:
                raise PublishError(
                    f'Topic "{topic}" is not in the ROS graph; give its type',
                )
            raise PublishError(
                f'Topic "{topic}" has several types; give one of {types}',
            )
        if types and type_name not in types:
            raise PublishError(
                f'Topic "{topic}" has type {", ".join(types)}, not {type_name}',
            )
        return type_name
