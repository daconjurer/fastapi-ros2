# Tool images as named stages, so that Dependabot sees and updates them.
FROM ghcr.io/astral-sh/uv:0.11.21 AS uv
FROM ghcr.io/casey/just:1.59.0 AS just

FROM ros:lyrical-ros-base@sha256:3224ee150cb02eb89a5e0811bc267b57ed3a91b9c10191df57810726a7e916d7

COPY --from=uv /uv /uvx /usr/local/bin/
COPY --from=just /just /usr/local/bin/

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
# Dependencies first, so that code changes don't reinstall them.
RUN uv sync --locked --no-default-groups --no-cache --no-install-project
COPY . .
RUN uv sync --locked --no-default-groups --no-cache

ENTRYPOINT ["/ros_entrypoint.sh"]
