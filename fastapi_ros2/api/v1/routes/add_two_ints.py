from typing import Annotated

from fastapi import APIRouter, Request

from pydantic import AfterValidator, BaseModel, Field, model_validator
from pydantic_core import PydanticCustomError


router = APIRouter()

INT64_MIN, INT64_MAX = -(2**63), 2**63 - 1


def _fits_int64(value: int) -> int:
    if not INT64_MIN <= value <= INT64_MAX:
        raise PydanticCustomError(
            "int64_overflow", "Input should fit in a signed 64-bit integer"
        )
    return value


# AddTwoInts fields are int64; larger values fail to convert to the ROS message.
# The range is documented as the OpenAPI int64 format rather than minimum and
# maximum, which JSON Schema would round to the nearest float.
Int64 = Annotated[
    int, Field(json_schema_extra={"format": "int64"}), AfterValidator(_fits_int64)
]


class AddRequest(BaseModel):
    a: Int64
    b: Int64

    @model_validator(mode="after")
    def sum_fits_int64(self) -> "AddRequest":
        # An overflowing sum crashes the add_two_ints server when it responds.
        if not INT64_MIN <= self.a + self.b <= INT64_MAX:
            raise PydanticCustomError(
                "sum_overflow", "a + b should fit in a signed 64-bit integer"
            )
        return self


class AddResponse(BaseModel):
    sum: int


@router.post("/services/sum", response_model=AddResponse)
async def sum_service(request: Request, body: AddRequest):
    response = AddResponse(sum=-1)
    result_sum = await request.state.add_controller.add_two_ints(body.a, body.b)

    response.sum = result_sum
    return response
