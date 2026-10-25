import importlib
from typing import Annotated, Any

from pydantic import AfterValidator, BaseModel, ConfigDict, Field, create_model
from pydantic_core import PydanticCustomError


INT64_MIN, INT64_MAX = -(2**63), 2**63 - 1
FLOAT32_MAX = 3.4028234663852886e38


def _fits_int64(value: int) -> int:
    if not INT64_MIN <= value <= INT64_MAX:
        raise PydanticCustomError(
            "int64_overflow", "Input should fit in a signed 64-bit integer"
        )
    return value


def _fits_float32(value: float) -> float:
    if abs(value) > FLOAT32_MAX:
        raise PydanticCustomError(
            "float32_overflow", "Input should fit in a 32-bit float"
        )
    return value


# Values beyond a field's ROS type fail to convert to the ROS message.
# The int64 range is documented as the OpenAPI int64 format rather than minimum
# and maximum, which JSON Schema would round to the nearest float.
Int64 = Annotated[
    int, Field(json_schema_extra={"format": "int64"}), AfterValidator(_fits_int64)
]

# NaN, infinity, or values beyond float32 corrupt the receiving node's state.
Float32 = Annotated[
    float,
    Field(
        allow_inf_nan=False,
        description="Finite, within the 32-bit float range",
        json_schema_extra={"format": "float"},
    ),
    AfterValidator(_fits_float32),
]

# ROS field types, as `get_fields_and_field_types()` names them.
# ponytail: only the types our services use; add others here as they come up.
_FIELD_TYPES: dict[str, Any] = {
    "boolean": bool,
    "int64": Int64,
    "float": Float32,
    "double": float,
    "string": str,
}


def ros_model(ros_type: type) -> type[BaseModel]:
    """Derive a Pydantic model from a generated ROS message class.

    Fields keep the message's names, order, and types, and nested messages
    become nested models. The model validates ROS messages by their attributes,
    so a route can return `Model.model_validate(ros_message)`.

    The name drops the generated class's underscore: `AddTwoInts.Response`
    (`AddTwoInts_Response`) becomes `AddTwoIntsResponse`.
    """
    fields: dict[str, Any] = {}
    for name, ros_field_type in ros_type.get_fields_and_field_types().items():
        if ros_field_type in _FIELD_TYPES:
            fields[name] = (_FIELD_TYPES[ros_field_type], ...)
        elif ros_field_type.count("/") == 1:
            # A nested message, e.g. "geometry_msgs/Vector3".
            package, message = ros_field_type.split("/")
            nested = getattr(importlib.import_module(f"{package}.msg"), message)
            fields[name] = (ros_model(nested), ...)
        else:
            raise TypeError(
                f"{ros_type.__name__}.{name}: unsupported ROS type {ros_field_type!r}"
            )

    return create_model(
        ros_type.__name__.replace("_", ""),
        __config__=ConfigDict(from_attributes=True),
        **fields,
    )


class ServiceError(BaseModel):
    detail: str


# Every service endpoint can fail this way, besides invalid input (422).
SERVICE_ERRORS: dict[int | str, dict[str, Any]] = {
    503: {"model": ServiceError, "description": "The ROS service is not available"},
    504: {
        "model": ServiceError,
        "description": "The ROS service did not respond in time",
    },
}
