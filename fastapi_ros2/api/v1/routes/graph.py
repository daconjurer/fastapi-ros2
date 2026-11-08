from fastapi import APIRouter
from pydantic import BaseModel

from fastapi_ros2.graph import node_names, visible
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


@router.get("/graph")
async def graph(ros: Ros) -> GraphResponse:
    """List the nodes, topics, services, and actions in the ROS graph.

    Discovery is asynchronous, so an entity that just appeared or disappeared may
    take a moment to show.
    """
    node = ros.node
    return GraphResponse(
        nodes=node_names(node),
        topics=visible(node.get_topic_names_and_types()),
        services=visible(node.get_service_names_and_types()),
        actions=visible(node.get_action_names_and_types()),
    )
