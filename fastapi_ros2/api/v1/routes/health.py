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
    # Every service the API's node has a client for, by fully qualified name.
    services: dict[str, bool]


@router.get("/health", responses={503: {"model": HealthResponse}})
async def health(ros: Ros, response: Response) -> HealthResponse:
    node = ros.node
    # Once rclpy is shut down, the node and its clients are destroyed.
    context_ok = node.context.ok()
    node_health = NodeHealth(
        name=ros.node_name,
        # Without the executor thread no ROS response or message ever arrives.
        spinning=context_ok and ros.spin_thread.is_alive(),
    )
    services = dict.fromkeys(ros.service_names, False)
    if context_ok:
        services = {c.service_name: c.service_is_ready() for c in node.clients}

    ready = node_health.spinning and all(services.values())
    if not ready:
        response.status_code = 503
    return HealthResponse(
        status="ok" if ready else "unavailable", node=node_health, services=services
    )
