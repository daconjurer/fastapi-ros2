from typing import Annotated

from example_interfaces.srv import AddTwoInts
from fastapi import APIRouter, Body

from pydantic import model_validator
from pydantic_core import PydanticCustomError

from fastapi_ros2.api.v1.models import (
    INT64_MAX,
    INT64_MIN,
    SERVICE_ERRORS,
    Int64,
    RosModel,
    mirrors,
)
from fastapi_ros2.nodes import Ros


router = APIRouter(responses=SERVICE_ERRORS)


@mirrors(AddTwoInts.Request)
class AddRequest(RosModel):
    a: Int64
    b: Int64

    @model_validator(mode="after")
    def sum_fits_int64(self) -> "AddRequest":
        # The add_two_ints server would wrap an overflowing sum around.
        if not INT64_MIN <= self.a + self.b <= INT64_MAX:
            raise PydanticCustomError(
                "sum_overflow", "a + b should fit in a signed 64-bit integer"
            )
        return self


@mirrors(AddTwoInts.Response)
class AddResponse(RosModel):
    sum: Int64


AddBody = Annotated[
    AddRequest,
    Body(openapi_examples={"sum": {"summary": "2 + 3", "value": {"a": 2, "b": 3}}}),
]


@router.post("/services/sum")
async def sum_service(ros: Ros, body: AddBody) -> AddResponse:
    result = await ros.add_controller.add_two_ints(body.a, body.b)
    return AddResponse.model_validate(result)
