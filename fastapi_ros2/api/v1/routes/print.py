from fastapi import APIRouter, Request

from pydantic import BaseModel


router = APIRouter()


class PrintResponse(BaseModel):
    count: int


@router.post("/topics/print")
async def print_topic(request: Request) -> PrintResponse:
    response = PrintResponse(count=-1)
    result_print = await request.state.print_controller.publish_print()

    response.count = result_print
    return response
