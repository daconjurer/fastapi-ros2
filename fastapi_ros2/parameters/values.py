"""Conversion between ROS parameter values and plain Python values."""

from collections.abc import Iterable
from typing import cast

from rcl_interfaces.msg import ParameterType, ParameterValue
from rclpy.parameter import Parameter, parameter_value_to_python

type PythonValue = (
    bool | int | float | str | list[bool] | list[int] | list[float] | list[str]
)
"""A parameter value as JSON can carry it.

ponytail: byte arrays read as lists of ints and can't be written; add a bytes
encoding if a node needs them.
"""


def to_python(value: ParameterValue) -> PythonValue:
    """The value of a parameter that is set, with arrays as lists."""
    python = parameter_value_to_python(value)
    if value.type == ParameterType.PARAMETER_BYTE_ARRAY:
        return list(b"".join(cast(list[bytes], python)))
    if isinstance(python, bool | int | float | str):
        return python
    # Arrays may come as `array.array`.
    return cast(PythonValue, list(cast(Iterable[object], python)))


def to_ros(value: PythonValue, current: ParameterValue) -> ParameterValue:
    """The value as the ROS type its Python type implies, given the current value.

    Whole numbers for a float parameter become floats, since JSON clients often
    send 1.0 as 1, and an empty list takes the current array type, having no
    element type of its own. Otherwise the node checks the type.
    """
    array_types = {
        ParameterType.PARAMETER_BOOL_ARRAY,
        ParameterType.PARAMETER_INTEGER_ARRAY,
        ParameterType.PARAMETER_DOUBLE_ARRAY,
        ParameterType.PARAMETER_STRING_ARRAY,
    }
    if current.type == ParameterType.PARAMETER_DOUBLE and type(value) is int:
        value = float(value)
    if (
        current.type == ParameterType.PARAMETER_DOUBLE_ARRAY
        and isinstance(value, list)
        and all(type(x) is int for x in value)
    ):
        value = [float(x) for x in value]
    if value == [] and current.type in array_types:
        return Parameter("", Parameter.Type(current.type), []).get_parameter_value()
    return Parameter("", value=value).get_parameter_value()
