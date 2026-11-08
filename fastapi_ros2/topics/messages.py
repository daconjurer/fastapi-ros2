"""Build ROS messages from JSON, checked against the message's field types."""

import math
import re
from typing import cast

from rosidl_parser.definition import (
    AbstractGenericString,
    AbstractSequence,
    AbstractType,
    Array,
    BasicType,
    BoundedSequence,
    BoundedString,
    BoundedWString,
    NamespacedType,
)
from rosidl_pycommon.interface_base_classes import BaseMessage
from rosidl_runtime_py import set_message_fields
from rosidl_runtime_py.utilities import get_message

_MESSAGE_TYPE = re.compile(r"^\w+/msg/\w+$")
"""A message type's full name, as `ros2 topic list -t` shows it."""

_INTEGER_TYPES = {
    "int8": (-(2**7), 2**7 - 1),
    "int16": (-(2**15), 2**15 - 1),
    "int32": (-(2**31), 2**31 - 1),
    "int64": (-(2**63), 2**63 - 1),
    "uint8": (0, 2**8 - 1),
    "uint16": (0, 2**16 - 1),
    "uint32": (0, 2**32 - 1),
    "uint64": (0, 2**64 - 1),
    "short": (-(2**15), 2**15 - 1),
    "long": (-(2**31), 2**31 - 1),
    "long long": (-(2**63), 2**63 - 1),
    "unsigned short": (0, 2**16 - 1),
    "unsigned long": (0, 2**32 - 1),
    "unsigned long long": (0, 2**64 - 1),
}
"""The range of each integer type, by its name in the message definition."""

_FLOAT32_MAX = 3.4028234663852886e38


class MessageError(ValueError):
    """The JSON does not fit the message type, for the reason in the message."""


def message_type(name: str) -> type[BaseMessage]:
    """The message class for a full type name, such as `std_msgs/msg/String`."""
    if not _MESSAGE_TYPE.match(name):
        raise MessageError(f'"{name}" is not a message type like "std_msgs/msg/String"')
    try:
        return cast(type[BaseMessage], get_message(name))
    except ModuleNotFoundError, AttributeError, ValueError:
        raise MessageError(f'Unknown message type "{name}"') from None


def to_message[M: BaseMessage](cls: type[M], data: object) -> M:
    """Build a message from a JSON object, leaving out fields at their defaults.

    rosidl converts values without checking them: it truncates 1.5 to 1 for an
    integer and publishes integers and floats beyond their type's range, which
    then arrive changed. So every value is checked first, against its field's
    type, and anything that would not arrive as sent is refused.
    """
    _check_message(cls, data, "message")
    message = cls()
    # rosidl annotates the values as strings, but takes any JSON value.
    set_message_fields(message, cast(dict[str, str], data))
    return message


def _check_message(cls: type[BaseMessage], data: object, path: str) -> None:
    if not isinstance(data, dict):
        raise MessageError(f"{path} should be an object")
    # Every generated message class has SLOT_TYPES, though BaseMessage does not
    # declare it: the type of each field, in the order of the field names.
    slot_types = cast(list[AbstractType], getattr(cls, "SLOT_TYPES"))
    fields = dict(zip(cls.get_fields_and_field_types(), slot_types, strict=True))
    for key, value in cast(dict[str, object], data).items():
        if key not in fields:
            raise MessageError(f'{path} has no field "{key}"')
        _check(fields[key], value, f"{path}.{key}")


def _check(field_type: AbstractType, value: object, path: str) -> None:
    match field_type:
        case NamespacedType():
            name = "/".join([*field_type.namespaces, field_type.name])
            _check_message(cast(type[BaseMessage], get_message(name)), value, path)
        case Array() | AbstractSequence():
            if not isinstance(value, list):
                raise MessageError(f"{path} should be a list")
            items = cast(list[object], value)
            if isinstance(field_type, Array) and len(items) != field_type.size:
                raise MessageError(f"{path} should have {field_type.size} items")
            if (
                isinstance(field_type, BoundedSequence)
                and len(items) > field_type.maximum_size
            ):
                raise MessageError(
                    f"{path} should have at most {field_type.maximum_size} items",
                )
            for i, item in enumerate(items):
                _check(field_type.value_type, item, f"{path}[{i}]")
        case AbstractGenericString():
            if not isinstance(value, str):
                raise MessageError(f"{path} should be a string")
            if isinstance(field_type, BoundedString | BoundedWString):
                # Typed as possibly a string, for sizes given as constant names.
                maximum = int(field_type.maximum_size)
                if len(value) > maximum:
                    raise MessageError(
                        f"{path} should have at most {maximum} characters",
                    )
        case BasicType():
            _check_basic(field_type.typename, value, path)
        case _:
            raise MessageError(f"{path} has a type JSON can't set")


def _check_basic(typename: str, value: object, path: str) -> None:
    # bool is a subclass of int, so it is ruled out for numbers explicitly.
    if typename == "boolean":
        if not isinstance(value, bool):
            raise MessageError(f"{path} should be a boolean")
    elif typename in _INTEGER_TYPES:
        low, high = _INTEGER_TYPES[typename]
        if type(value) is not int:
            raise MessageError(f"{path} should be an integer ({typename})")
        if not low <= value <= high:
            raise MessageError(f"{path} should fit in {typename}, {low} to {high}")
    elif typename in ("float", "double", "long double"):
        if type(value) not in (int, float):
            raise MessageError(f"{path} should be a number ({typename})")
        number = cast(float, value)
        if not math.isfinite(number):
            raise MessageError(f"{path} should be finite")
        if typename == "float" and abs(number) > _FLOAT32_MAX:
            raise MessageError(f"{path} should fit in a 32-bit float")
    else:
        # ponytail: octet, char, and wchar fields are refused; add their JSON
        # forms if a message needs them.
        raise MessageError(f"{path} has a type JSON can't set ({typename})")
