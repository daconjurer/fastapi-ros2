FROM ros:lyrical-ros-base

COPY --from=ghcr.io/astral-sh/uv:0.11.21 /uv /uvx /usr/local/bin/
COPY --from=ghcr.io/casey/just:1.59.0 /just /usr/local/bin/

RUN apt-get update \
    && apt-get install -y --no-install-recommends ros-lyrical-example-interfaces ros-lyrical-turtlesim \
    && rm -rf /var/lib/apt/lists/*

# The app venv uses the system Python and sees its site-packages, since rclpy and
# the ROS message packages are installed there and built against that Python.
ENV UV_PYTHON_PREFERENCE=only-system UV_PROJECT_ENVIRONMENT=/opt/venv UV_LINK_MODE=copy \
    VIRTUAL_ENV=/opt/venv PATH=/opt/venv/bin:$PATH
RUN uv venv --system-site-packages /opt/venv
# Interactive shells (devcontainer terminals, debugger) need the ROS environment too.
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc

WORKDIR /app
COPY pyproject.toml uv.lock ./
RUN uv sync --locked --no-default-groups --no-cache
COPY . .

ENTRYPOINT ["/ros_entrypoint.sh"]
