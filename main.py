from collections.abc import Sequence
from typing import cast

import uvicorn
from fastapi import FastAPI, Request
from fastapi.encoders import jsonable_encoder
from fastapi.exception_handlers import request_validation_exception_handler
from fastapi.exceptions import RequestValidationError
from fastapi.responses import JSONResponse

from fastapi_ros2.api.v1.routes.add_two_ints import router as add_two_ints_router
from fastapi_ros2.api.v1.routes.turtlesim import router as turtle_router
from fastapi_ros2.api.v1.routes.print import router as print_router
from fastapi_ros2.api.v1.routes.health import router as health_router
from fastapi_ros2.nodes import ros_lifespan
from fastapi_ros2.services.call import ServiceTimeoutError, ServiceUnavailableError

app = FastAPI(lifespan=ros_lifespan)
app.include_router(add_two_ints_router)
app.include_router(turtle_router)
app.include_router(print_router)
app.include_router(health_router)


@app.exception_handler(RequestValidationError)
async def validation_error_handler(request: Request, exc: RequestValidationError):
    try:
        return await request_validation_exception_handler(request, exc)
    except ValueError:
        # The default 422 echoes the invalid input, which cannot be encoded as
        # JSON when it is NaN or infinity; leave the input out in that case.
        details = cast(Sequence[dict[str, object]], exc.errors())
        errors = [{k: v for k, v in e.items() if k != "input"} for e in details]
        return JSONResponse(
            status_code=422, content={"detail": jsonable_encoder(errors)}
        )


@app.exception_handler(ServiceUnavailableError)
async def service_unavailable_handler(_request: Request, exc: ServiceUnavailableError):
    return JSONResponse(status_code=503, content={"detail": str(exc)})


@app.exception_handler(ServiceTimeoutError)
async def service_timeout_handler(_request: Request, exc: ServiceTimeoutError):
    return JSONResponse(status_code=504, content={"detail": str(exc)})


def main():
    uvicorn.run(app, port=8000, log_level="warning")


if __name__ == "__main__":
    main()
