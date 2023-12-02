from fastapi import APIRouter

from pydantic import BaseModel

from fastapi_ros2.nodes import print_controller


router = APIRouter()


class PrintResponse(BaseModel):
    count: int


@router.get("/topics/print", response_model=PrintResponse)
async def sum_service():
    response = PrintResponse(count=-1)
    result_print = await print_controller.publish_print()

    response.count = result_print
    return response
