from collections.abc import Callable
from typing import Annotated, ClassVar, cast

from pydantic import AfterValidator, BaseModel, ConfigDict, Field
from pydantic.fields import FieldInfo
from pydantic_core import PydanticCustomError
from rosidl_pycommon.interface_base_classes import BaseMessage


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
# ponytail: only the types our services use, and no nested messages; add others
# here as they come up.
_FIELD_TYPES: dict[str, object] = {
    "boolean": bool,
    "int64": Int64,
    "float": Float32,
    "double": float,
    "string": str,
}


class RosModel(BaseModel):
    """Base for models that mirror a ROS message, checked with `@mirrors`.

    It validates ROS messages by their attributes, so a route can return
    `Model.model_validate(ros_message)`.
    """

    model_config: ClassVar[ConfigDict] = ConfigDict(from_attributes=True)


def mirrors[M: type[BaseModel]](ros_type: type[BaseMessage]) -> Callable[[M], M]:
    """Check that a model declares the fields of a generated ROS message class.

    The fields must match the message's names, order, and types (`Int64` for an
    int64, `Float32` for a float), so a model that drifts from its message fails
    at import instead of sending ROS what it can't take.
    """
    ros_fields = ros_type.get_fields_and_field_types()
    unsupported = set(ros_fields.values()) - _FIELD_TYPES.keys()
    if unsupported:
        raise TypeError(f"{ros_type.__name__}: unsupported ROS types {unsupported}")
    # FieldInfo has no equality, but its repr shows everything that affects
    # validation and the schema: the type, validators, constraints, and default.
    # Pydantic takes `Annotated` types here, though it types the argument as a class.
    expected = [
        (name, repr(FieldInfo.from_annotation(cast(type, _FIELD_TYPES[t]))))
        for name, t in ros_fields.items()
    ]

    def check(model: M) -> M:
        declared = [(name, repr(f)) for name, f in model.model_fields.items()]
        if declared != expected:
            fields = ", ".join(f"{name}: {t}" for name, t in ros_fields.items())
            raise TypeError(
                f"{model.__name__} should declare the fields of "
                f"{ros_type.__name__} ({fields or 'none'})"
            )
        return model

    return check


class ServiceError(BaseModel):
    detail: str


# Every service endpoint can fail this way, besides invalid input (422).
SERVICE_ERRORS: dict[int | str, dict[str, object]] = {
    503: {"model": ServiceError, "description": "The ROS service is not available"},
    504: {
        "model": ServiceError,
        "description": "The ROS service did not respond in time",
    },
}
