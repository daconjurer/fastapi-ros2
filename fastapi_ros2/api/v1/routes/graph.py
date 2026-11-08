from fastapi import APIRouter
from pydantic import BaseModel
from rclpy.topic_or_service_is_hidden import topic_or_service_is_hidden

from fastapi_ros2.nodes import Ros


router = APIRouter()


class GraphResponse(BaseModel):
    """What the API's node sees of the ROS graph, as `ros2 <kind> list -t` shows it.

    Names are fully qualified, and each maps to its types (usually one).
    """

    nodes: list[str]
    topics: dict[str, list[str]]
    services: dict[str, list[str]]
    actions: dict[str, list[str]]


def _visible(names_and_types: list[tuple[str, list[str]]]) -> dict[str, list[str]]:
    # Hidden names, such as the topics and services each action is built on, are
    # left out, as ros2cli does by default.
    return {
        name: types
        for name, types in names_and_types
        if not topic_or_service_is_hidden(name)
    }


@router.get("/graph")
async def graph(ros: Ros) -> GraphResponse:
    """List the nodes, topics, services, and actions in the ROS graph.

    Discovery is asynchronous, so an entity that just appeared or disappeared may
    take a moment to show.
    """
    node = ros.node
    return GraphResponse(
        nodes=sorted(
            f"{namespace.rstrip('/')}/{name}"
            for name, namespace in node.get_node_names_and_namespaces()
        ),
        topics=_visible(node.get_topic_names_and_types()),
        services=_visible(node.get_service_names_and_types()),
        actions=_visible(node.get_action_names_and_types()),
    )
