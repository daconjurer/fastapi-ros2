from typing import Annotated

from fastapi import APIRouter, Body, Path
from pydantic import BaseModel, Field, JsonValue

from fastapi_ros2.api.v1.models import ServiceError
from fastapi_ros2.nodes import Ros


router = APIRouter()

TopicName = Annotated[
    str,
    Path(
        description="Fully qualified, without the leading slash",
        examples=["turtle1/cmd_vel"],
    ),
]
"""A topic's name in the path, which may contain slashes."""


class PublishRequest(BaseModel):
    """A message to publish, as JSON, and its type if the topic is new."""

    type: str | None = Field(
        default=None,
        description="Defaults to the topic's type in the ROS graph",
        examples=["geometry_msgs/msg/Twist"],
    )
    message: dict[str, JsonValue] = Field(
        default_factory=dict,
        description="The message's fields; those left out keep their defaults",
    )


class PublishResponse(BaseModel):
    """Where the message was published, and as which type."""

    topic: str
    type: str


PublishBody = Annotated[
    PublishRequest,
    Body(
        openapi_examples={
            "turn": {
                "summary": "Drive turtlesim's turtle forward and turning",
                "value": {"message": {"linear": {"x": 2.0}, "angular": {"z": 1.8}}},
            },
            "new topic": {
                "summary": "A topic not in the graph yet needs its type",
                "value": {"type": "std_msgs/msg/String", "message": {"data": "hello"}},
            },
        },
    ),
]
"""The `POST` body, with examples for turtlesim and a new topic."""


@router.post(
    "/topics/{name:path}",
    responses={
        422: {"model": ServiceError, "description": "The message can't be published"},
    },
)
async def publish(ros: Ros, name: TopicName, body: PublishBody) -> PublishResponse:
    """Publish a message to any topic, as `ros2 topic pub --once` does.

    Every value is checked against its field's type, so that what arrives is what
    was sent.
    """
    topic = "/" + name.strip("/")
    # Publishing hands the message to DDS and returns at once, so it doesn't
    # block the event loop.
    type_name = ros.topic_controller.publish(
        topic,
        dict(body.message),
        body.type,
    )
    return PublishResponse(topic=topic, type=type_name)
