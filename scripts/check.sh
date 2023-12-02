#! /bin/bash
echo "ruff check --diff fastapi_ros2/"
ruff check --diff fastapi_ros2/
echo "DONE!"
echo "ruff format --diff fastapi_ros2/"
ruff format --diff fastapi_ros2/
echo "DONE!"
# mypy fastapi_ros2/
