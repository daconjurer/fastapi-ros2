import rclpy

import uvicorn
from fastapi import FastAPI

from fastapi_ros2.api.v1.routes.add_two_ints import router as add_two_ints_router
from fastapi_ros2.api.v1.routes.turtlesim import router as turtle_router
from fastapi_ros2.api.v1.routes.print import router as print_router

app = FastAPI()
app.include_router(add_two_ints_router)
app.include_router(turtle_router)
app.include_router(print_router)


def main(args=None):
    uvicorn.run(app, port=5000, log_level='warning')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
