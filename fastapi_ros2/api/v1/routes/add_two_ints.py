from fastapi import APIRouter, Request

from pydantic import BaseModel


router = APIRouter()


class AddResponse(BaseModel):
    sum: int


@router.get("/services/sum", response_model=AddResponse)
async def sum_service(request: Request, a: int, b: int):
    response = AddResponse(sum=-1)
    result_sum = await request.state.add_controller.add_two_ints(a, b)

    response.sum = result_sum
    return response
