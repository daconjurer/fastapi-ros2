#! /bin/bash

ruff check --diff fastapi_ros2/
ruff format --diff fastapi_ros2/
# mypy fastapi_ros2/
