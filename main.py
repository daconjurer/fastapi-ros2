import uvicorn
from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse

from fastapi_ros2.api.v1.routes.add_two_ints import router as add_two_ints_router
from fastapi_ros2.api.v1.routes.turtlesim import router as turtle_router
from fastapi_ros2.api.v1.routes.print import router as print_router
from fastapi_ros2.nodes import ros_lifespan
from fastapi_ros2.services.call import ServiceTimeoutError, ServiceUnavailableError

app = FastAPI(lifespan=ros_lifespan)
app.include_router(add_two_ints_router)
app.include_router(turtle_router)
app.include_router(print_router)


@app.exception_handler(ServiceUnavailableError)
async def service_unavailable_handler(request: Request, exc: ServiceUnavailableError):
    return JSONResponse(status_code=503, content={"detail": str(exc)})


@app.exception_handler(ServiceTimeoutError)
async def service_timeout_handler(request: Request, exc: ServiceTimeoutError):
    return JSONResponse(status_code=504, content={"detail": str(exc)})


def main(args=None):
    uvicorn.run(app, port=8000, log_level='warning')


if __name__ == '__main__':
    main()
