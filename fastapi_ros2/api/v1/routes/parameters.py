from collections.abc import Iterable
from typing import Annotated, cast

from fastapi import APIRouter, Body, HTTPException, Path
from pydantic import BaseModel, Field
from rcl_interfaces.msg import ParameterType, ParameterValue
from rclpy.parameter import Parameter, parameter_value_to_python

from fastapi_ros2.api.v1.models import SERVICE_ERRORS, Int64, ServiceError
from fastapi_ros2.api.v1.routes.graph import node_names
from fastapi_ros2.nodes import Ros


router = APIRouter(
    responses={
        **SERVICE_ERRORS,
        404: {"model": ServiceError, "description": "No such node or parameter"},
    }
)

Double = Annotated[float, Field(allow_inf_nan=False)]

# The parameter types JSON can carry, by their JSON form.
# ponytail: byte arrays read as lists of ints and can't be written; add a bytes
# encoding if a node needs them.
type ParameterJson = (
    bool | Int64 | Double | str | list[bool] | list[Int64] | list[Double] | list[str]
)

NodeName = Annotated[
    str,
    Path(
        description="Fully qualified, without the leading slash", examples=["turtlesim"]
    ),
]
ParameterName = Annotated[str, Path(examples=["background_r"])]


class ParameterResponse(BaseModel):
    name: str
    value: ParameterJson


class ParameterUpdate(BaseModel):
    value: ParameterJson


ParameterBody = Annotated[
    ParameterUpdate,
    Body(
        openapi_examples={
            "red": {
                "summary": "Turtlesim's background red channel, 0 to 255",
                "value": {"value": 255},
            }
        }
    ),
]


def _node(ros: Ros, node: str) -> str:
    name = "/" + node.strip("/")
    if name not in node_names(ros.node):
        raise HTTPException(404, f'Node "{name}" is not in the ROS graph')
    return name


async def _current(ros: Ros, node: str, name: str) -> ParameterValue:
    value = await ros.parameter_controller.get(node, name)
    if value is None:
        raise HTTPException(404, f'Node "{node}" has no parameter "{name}"')
    return value


def _to_json(value: ParameterValue) -> ParameterJson:
    python = parameter_value_to_python(value)
    if value.type == ParameterType.PARAMETER_BYTE_ARRAY:
        return list(b"".join(cast(list[bytes], python)))
    if isinstance(python, bool | int | float | str):
        return python
    # Arrays may come as `array.array`, which the response model does not take.
    return cast(ParameterJson, list(cast(Iterable[object], python)))


def _to_ros(value: ParameterJson, current: ParameterValue) -> ParameterValue:
    """The value as the ROS type its JSON form implies; the node checks the type."""
    # JSON has one number type, so clients send 1 for 1.0.
    if current.type == ParameterType.PARAMETER_DOUBLE and type(value) is int:
        value = float(value)
    if (
        current.type == ParameterType.PARAMETER_DOUBLE_ARRAY
        and isinstance(value, list)
        and all(type(x) is int for x in value)
    ):
        value = [float(x) for x in value]
    # An empty list has no element type of its own.
    if value == [] and current.type in _ARRAY_TYPES:
        return Parameter("", Parameter.Type(current.type), []).get_parameter_value()
    return Parameter("", value=value).get_parameter_value()


_ARRAY_TYPES = {
    ParameterType.PARAMETER_BOOL_ARRAY,
    ParameterType.PARAMETER_INTEGER_ARRAY,
    ParameterType.PARAMETER_DOUBLE_ARRAY,
    ParameterType.PARAMETER_STRING_ARRAY,
}


@router.get("/nodes/{node:path}/parameters/{name:path}")
async def get_parameter(
    ros: Ros, node: NodeName, name: ParameterName
) -> ParameterResponse:
    """Read a parameter of any node in the graph."""
    node = _node(ros, node)
    value = await _current(ros, node, name)
    return ParameterResponse(name=name, value=_to_json(value))


@router.put(
    "/nodes/{node:path}/parameters/{name:path}",
    responses={422: {"description": "Invalid value, or the node refused it"}},
)
async def set_parameter(
    ros: Ros, node: NodeName, name: ParameterName, body: ParameterBody
) -> ParameterResponse:
    """Set a parameter the node has, as `ros2 param set` does.

    The node checks the value's type and range, and refuses it with a reason.
    """
    node = _node(ros, node)
    value = _to_ros(body.value, await _current(ros, node, name))
    result = await ros.parameter_controller.set(node, name, value)
    if not result.successful:
        raise HTTPException(422, result.reason)
    return ParameterResponse(name=name, value=_to_json(value))
