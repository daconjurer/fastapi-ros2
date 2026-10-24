from fastapi import APIRouter, Request

from pydantic import BaseModel


router = APIRouter()


class MoveResponse(BaseModel):
    done: bool


@router.get("/services/move_turtle", response_model=MoveResponse)
async def move_turtle_service(request: Request, x: float, y: float, theta: float):
    response = MoveResponse(done=False)
    result_move = await request.state.turtle_controller.move_absolute(x, y, theta)

    response.done = result_move
    return response
