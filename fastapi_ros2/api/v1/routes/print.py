from fastapi import APIRouter, Request

from pydantic import BaseModel


router = APIRouter()


class PrintResponse(BaseModel):
    count: int


@router.get("/topics/print", response_model=PrintResponse)
async def sum_service(request: Request):
    response = PrintResponse(count=-1)
    result_print = await request.state.print_controller.publish_print()

    response.count = result_print
    return response
