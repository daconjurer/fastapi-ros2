"""What a node sees of the ROS graph, named as ros2cli shows it."""

from rclpy.node import Node
from rclpy.topic_or_service_is_hidden import topic_or_service_is_hidden


def node_names(node: Node) -> list[str]:
    """The fully qualified names of the nodes in the graph, sorted."""
    return sorted(
        f"{namespace.rstrip('/')}/{name}"
        for name, namespace in node.get_node_names_and_namespaces()
    )


def visible(names_and_types: list[tuple[str, list[str]]]) -> dict[str, list[str]]:
    """Map each name to its types, leaving out hidden names as ros2cli does.

    Hidden names include the topics and services each action is built on.
    """
    return {
        name: types
        for name, types in names_and_types
        if not topic_or_service_is_hidden(name)
    }
