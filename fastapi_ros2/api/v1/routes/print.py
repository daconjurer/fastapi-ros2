from fastapi import APIRouter

from pydantic import BaseModel

from fastapi_ros2.nodes import Ros


router = APIRouter()


class PrintResponse(BaseModel):
    count: int


@router.post("/topics/print")
async def print_topic(ros: Ros) -> PrintResponse:
    response = PrintResponse(count=-1)
    result_print = await ros.print_controller.publish_print()

    response.count = result_print
    return response
