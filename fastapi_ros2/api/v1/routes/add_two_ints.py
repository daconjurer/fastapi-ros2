from fastapi import APIRouter

from pydantic import BaseModel

from fastapi_ros2.nodes import add_controller


router = APIRouter()


class AddResponse(BaseModel):
    sum: int


@router.get("/services/sum", response_model=AddResponse)
async def sum_service(a: int, b: int):
    response = AddResponse(sum=-1)
    result_sum = await add_controller.add_two_ints(a, b)

    response.sum = result_sum
    return response
