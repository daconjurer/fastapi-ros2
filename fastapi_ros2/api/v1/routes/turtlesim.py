from typing import Annotated

from fastapi import APIRouter, Request

from pydantic import AfterValidator, BaseModel, Field
from pydantic_core import PydanticCustomError


router = APIRouter()

FLOAT32_MAX = 3.4028234663852886e38

# TeleportAbsolute fields are float32. NaN, infinity, or values beyond float32
# corrupt the turtle's pose; finite values off the canvas are clamped by turtlesim.


def _fits_float32(value: float) -> float:
    if abs(value) > FLOAT32_MAX:
        raise PydanticCustomError(
            "float32_overflow", "Input should fit in a 32-bit float"
        )
    return value


Float32 = Annotated[
    float,
    Field(
        allow_inf_nan=False,
        description="Finite, within the 32-bit float range",
        json_schema_extra={"format": "float"},
    ),
    AfterValidator(_fits_float32),
]


class MoveRequest(BaseModel):
    x: Float32
    y: Float32
    theta: Float32


class MoveResponse(BaseModel):
    done: bool


@router.post("/services/move_turtle", response_model=MoveResponse)
async def move_turtle_service(request: Request, body: MoveRequest):
    response = MoveResponse(done=False)
    result_move = await request.state.turtle_controller.move_absolute(
        body.x, body.y, body.theta
    )

    response.done = result_move
    return response
