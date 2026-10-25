from typing import Annotated

from fastapi import APIRouter, Body, Request
from turtlesim.srv import TeleportAbsolute

from fastapi_ros2.api.v1.models import SERVICE_ERRORS, ros_model


router = APIRouter(responses=SERVICE_ERRORS)


# Finite values off the canvas (about 0 to 11 on each axis) are clamped by turtlesim.
class MoveRequest(ros_model(TeleportAbsolute.Request)):  # type: ignore[misc]
    pass


# TeleportAbsolute has no response fields, so success is an empty object.
class MoveResponse(ros_model(TeleportAbsolute.Response)):  # type: ignore[misc]
    pass


@router.post("/services/move_turtle")
async def move_turtle_service(
    request: Request,
    body: Annotated[
        MoveRequest,
        Body(
            openapi_examples={
                "pose": {
                    "summary": "A pose inside the canvas",
                    "value": {"x": 3.0, "y": 4.0, "theta": 1.5},
                }
            }
        ),
    ],
) -> MoveResponse:
    result = await request.state.turtle_controller.move_absolute(
        body.x, body.y, body.theta
    )
    return MoveResponse.model_validate(result)
