from typing import Literal

from fastapi import APIRouter, Request, Response

from pydantic import BaseModel


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
async def health(request: Request, response: Response) -> HealthResponse:
    node = request.state.node
    # Once rclpy is shut down, the node and its clients are destroyed.
    context_ok = node.context.ok()
    node_health = NodeHealth(
        name=request.state.node_name,
        # Without the executor thread no ROS response or message ever arrives.
        spinning=context_ok and request.state.spin_thread.is_alive(),
    )
    services = dict.fromkeys(request.state.service_names, False)
    if context_ok:
        services = {c.service_name: c.service_is_ready() for c in node.clients}

    ready = node_health.spinning and all(services.values())
    if not ready:
        response.status_code = 503
    return HealthResponse(
        status="ok" if ready else "unavailable", node=node_health, services=services
    )
