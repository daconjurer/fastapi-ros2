from typing import Annotated

from fastapi import APIRouter, Body, WebSocket
from pydantic import BaseModel
from turtlesim_msgs.action import RotateAbsolute

from fastapi_ros2.actions.client import Feedback, GoalNotFoundError
from fastapi_ros2.api.v1.models import (
    SERVICE_ERRORS,
    Float32,
    RosModel,
    ServiceError,
    mirrors,
)
from fastapi_ros2.nodes import Ros


router = APIRouter()


@mirrors(RotateAbsolute.Goal)
class RotateRequest(RosModel):
    """The heading to rotate the turtle to, in radians."""

    theta: Float32


@mirrors(RotateAbsolute.Feedback)
class RotateFeedback(RosModel):
    """How far the turtle still has to rotate, in radians."""

    remaining: Float32


@mirrors(RotateAbsolute.Result)
class RotateResult(RosModel):
    """How far the turtle rotated from where it started, in radians."""

    delta: Float32


class GoalResponse(BaseModel):
    """The id to follow or cancel a goal by."""

    goal_id: str


class FeedbackEvent(BaseModel):
    """A WebSocket message with the goal's latest feedback."""

    feedback: RotateFeedback


class OutcomeEvent(BaseModel):
    """The last WebSocket message: how the goal finished, and its result."""

    status: str
    result: RotateResult


RotateBody = Annotated[
    RotateRequest,
    Body(
        openapi_examples={
            "half turn": {"summary": "Face west", "value": {"theta": 3.14}},
        },
    ),
]
"""The `POST` body, with an example."""

GOAL_ERRORS: dict[int | str, dict[str, object]] = {
    404: {"model": ServiceError, "description": "No such goal"},
    409: {"model": ServiceError, "description": "The goal can't be canceled"},
}
"""How a request about an existing goal can fail."""


@router.post(
    "/actions/rotate_turtle",
    status_code=202,
    responses={
        **SERVICE_ERRORS,
        409: {"model": ServiceError, "description": "The server rejected the goal"},
    },
)
async def rotate_turtle(ros: Ros, body: RotateBody) -> GoalResponse:
    """Rotate the turtle to a heading, with turtlesim's `RotateAbsolute` action.

    Returns once the goal is accepted. Follow its feedback and outcome on the
    WebSocket at `/actions/rotate_turtle/{goal_id}`. A new goal aborts the one
    the turtle is rotating to.
    """
    return GoalResponse(goal_id=await ros.rotate_controller.rotate(body.theta))


@router.post(
    "/actions/rotate_turtle/{goal_id}/cancel",
    status_code=202,
    responses={**SERVICE_ERRORS, **GOAL_ERRORS},
)
async def cancel_rotate_turtle(ros: Ros, goal_id: str) -> GoalResponse:
    """Ask turtlesim to cancel a goal.

    The goal's outcome, on its WebSocket, says whether it was canceled.
    """
    await ros.rotate_controller.cancel(goal_id)
    return GoalResponse(goal_id=goal_id)


@router.websocket("/actions/rotate_turtle/{goal_id}")
async def follow_rotate_turtle(ros: Ros, websocket: WebSocket, goal_id: str):
    """Send the goal's feedback as it arrives, then its outcome, and close.

    Messages are `FeedbackEvent`s, then one `OutcomeEvent`. A goal that has
    finished sends only its outcome. An unknown goal closes with code 4404.
    """
    await websocket.accept()
    try:
        events = ros.rotate_controller.events(goal_id)
    except GoalNotFoundError as error:
        await websocket.close(code=4404, reason=str(error))
        return
    async for event in events:
        if isinstance(event, Feedback):
            message = FeedbackEvent(
                feedback=RotateFeedback.model_validate(event.feedback),
            )
        else:
            message = OutcomeEvent(
                status=event.status,
                result=RotateResult.model_validate(event.result),
            )
        await websocket.send_text(message.model_dump_json())
    await websocket.close()
