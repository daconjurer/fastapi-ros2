FROM ros:jazzy-ros-base

RUN apt-get update \
    && apt-get install -y --no-install-recommends python3-venv ros-jazzy-example-interfaces ros-jazzy-turtlesim \
    && rm -rf /var/lib/apt/lists/*

# Poetry gets its own venv; the app venv sees the system site-packages so rclpy
# and the ROS message packages stay importable.
RUN python3 -m venv /opt/poetry && /opt/poetry/bin/pip install --no-cache-dir poetry==2.1.1 \
    && python3 -m venv --system-site-packages /opt/venv
ENV VIRTUAL_ENV=/opt/venv PATH=/opt/venv/bin:/opt/poetry/bin:$PATH
# Interactive shells (devcontainer terminals, debugger) need the ROS environment too.
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc

WORKDIR /app
COPY pyproject.toml poetry.lock ./
RUN poetry install --only main --no-root --no-interaction
COPY . .

ENTRYPOINT ["/ros_entrypoint.sh"]
