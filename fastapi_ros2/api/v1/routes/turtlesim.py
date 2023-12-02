from fastapi import APIRouter

from pydantic import BaseModel

from fastapi_ros2.nodes import turtle_controller


router = APIRouter()


class MoveResponse(BaseModel):
    done: bool


@router.get("/services/move_turtle", response_model=MoveResponse)
async def move_turtle_service(x: float, y: float, theta: float):
    response = MoveResponse(done=False)
    result_move = await turtle_controller.move_absolute(x, y, theta)

    response.done = result_move
    return response
