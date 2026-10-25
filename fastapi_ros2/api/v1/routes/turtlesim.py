from typing import Annotated

from fastapi import APIRouter, Body
from turtlesim_msgs.srv import TeleportAbsolute

from fastapi_ros2.api.v1.models import SERVICE_ERRORS, Float32, RosModel, mirrors
from fastapi_ros2.nodes import Ros


router = APIRouter(responses=SERVICE_ERRORS)


# Finite values off the canvas (about 0 to 11 on each axis) are clamped by turtlesim.
@mirrors(TeleportAbsolute.Request)
class MoveRequest(RosModel):
    x: Float32
    y: Float32
    theta: Float32


# TeleportAbsolute has no response fields, so success is an empty object.
@mirrors(TeleportAbsolute.Response)
class MoveResponse(RosModel):
    pass


MoveBody = Annotated[
    MoveRequest,
    Body(
        openapi_examples={
            "pose": {
                "summary": "A pose inside the canvas",
                "value": {"x": 3.0, "y": 4.0, "theta": 1.5},
            }
        }
    ),
]


@router.post("/services/move_turtle")
async def move_turtle_service(ros: Ros, body: MoveBody) -> MoveResponse:
    result = await ros.turtle_controller.move_absolute(body.x, body.y, body.theta)
    return MoveResponse.model_validate(result)
