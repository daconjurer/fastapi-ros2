from typing import Literal

from fastapi import APIRouter, Response

from pydantic import BaseModel

from fastapi_ros2.nodes import Ros


router = APIRouter()


class NodeHealth(BaseModel):
    name: str
    spinning: bool


class HealthResponse(BaseModel):
    status: Literal["ok", "unavailable"]
    node: NodeHealth
    # Every service and action the API depends on, by fully qualified name.
    services: dict[str, bool]
    actions: dict[str, bool]


@router.get("/health", responses={503: {"model": HealthResponse}})
async def health(ros: Ros, response: Response) -> HealthResponse:
    """Report whether the API can talk to ROS: `200` if so, `503` otherwise."""
    # Without the executor thread no ROS response or message ever arrives, and
    # once rclpy is shut down the node is destroyed.
    spinning = ros.node.context.ok() and ros.spin_thread.is_alive()
    services = {service.name: service.is_ready() for service in ros.services}
    actions = {action.name: action.is_ready() for action in ros.actions}
    ready = spinning and all(services.values()) and all(actions.values())
    response.status_code = 200 if ready else 503
    return HealthResponse(
        status="ok" if ready else "unavailable",
        node=NodeHealth(name=ros.node_name, spinning=spinning),
        services=services,
        actions=actions,
    )
