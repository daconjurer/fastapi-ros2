from typing import Annotated

from fastapi import APIRouter, Body, Path
from pydantic import BaseModel, Field

from fastapi_ros2.api.v1.models import SERVICE_ERRORS, Int64, ServiceError
from fastapi_ros2.nodes import Ros


router = APIRouter(
    responses={
        **SERVICE_ERRORS,
        404: {"model": ServiceError, "description": "No such node or parameter"},
    },
)

Double = Annotated[float, Field(allow_inf_nan=False)]
"""A finite float, since JSON can't carry NaN or infinity."""

type ParameterJson = (
    bool | Int64 | Double | str | list[bool] | list[Int64] | list[Double] | list[str]
)
"""The parameter values JSON can carry, validated to fit their ROS types."""

NodeName = Annotated[
    str,
    Path(
        description="Fully qualified, without the leading slash",
        examples=["turtlesim"],
    ),
]
"""A node's name in the path, which may contain slashes for its namespace."""

ParameterName = Annotated[str, Path(examples=["background_r"])]
"""A parameter's name in the path, which may contain slashes."""


class ParameterResponse(BaseModel):
    """A parameter and its value."""

    name: str
    value: ParameterJson


class ParameterUpdate(BaseModel):
    """The value to set a parameter to."""

    value: ParameterJson


ParameterBody = Annotated[
    ParameterUpdate,
    Body(
        openapi_examples={
            "red": {
                "summary": "Turtlesim's background red channel, 0 to 255",
                "value": {"value": 255},
            },
        },
    ),
]
"""The `PUT` body, with an example for turtlesim."""


def _fully_qualified(node: NodeName) -> str:
    """The node's name as ROS writes it, with the leading slash."""
    return "/" + node.strip("/")


@router.get("/nodes/{node:path}/parameters/{name:path}")
async def get_parameter(
    ros: Ros,
    node: NodeName,
    name: ParameterName,
) -> ParameterResponse:
    """Read a parameter of any node in the graph."""
    value = await ros.parameter_controller.get(_fully_qualified(node), name)
    return ParameterResponse(name=name, value=value)


@router.put(
    "/nodes/{node:path}/parameters/{name:path}",
    responses={
        422: {"model": ServiceError, "description": "The node refused the value"},
    },
)
async def set_parameter(
    ros: Ros,
    node: NodeName,
    name: ParameterName,
    body: ParameterBody,
) -> ParameterResponse:
    """Set a parameter the node has, as `ros2 param set` does.

    The node checks the value's type and range, and refuses it with a reason.
    """
    value = await ros.parameter_controller.set(_fully_qualified(node), name, body.value)
    return ParameterResponse(name=name, value=value)
