import threading
from contextlib import asynccontextmanager
from types import SimpleNamespace

import pytest
from fastapi.testclient import TestClient
from rclpy.qos import QoSProfile
from rclpy.task import Future

from fastapi_ros2.nodes import ros_state
from main import app


class FakeClient:
    """An rclpy service client whose server answers every call with `response`.

    Set `response` to None for a server that never answers, or to an exception
    for a call that fails.
    """

    def __init__(self, srv_type, srv_name: str, depth: int = 10):
        self.service_name = "/" + srv_name.lstrip("/")
        self.qos_profile = QoSProfile(depth=depth)
        self.ready = True
        self.response = srv_type.Response()
        self.requests: list = []
        self.removed: list[Future] = []

    def service_is_ready(self) -> bool:
        return self.ready

    def call_async(self, request) -> Future:
        self.requests.append(request)
        future = Future()
        if isinstance(self.response, Exception):
            outcome = future.set_exception
        else:
            outcome = future.set_result
        if self.response is not None:
            # Like rclpy, complete the future on another thread than the event loop's.
            threading.Thread(target=outcome, args=(self.response,)).start()
        return future

    def remove_pending_request(self, future: Future) -> None:
        self.removed.append(future)


class FakePublisher:
    def __init__(self, topic: str):
        self.topic_name = "/" + topic
        self.published: list = []

    def publish(self, msg) -> None:
        self.published.append(msg)


class FakeNode:
    """The API's rclpy node, with fake clients and publishers."""

    def __init__(self):
        self.context = SimpleNamespace(ok=lambda: True)
        self.clients: list[FakeClient] = []
        self.publishers: list[FakePublisher] = []

    def get_fully_qualified_name(self) -> str:
        return "/fastapi_ros2"

    def create_client(self, srv_type, srv_name: str) -> FakeClient:
        self.clients.append(FakeClient(srv_type, srv_name))
        return self.clients[-1]

    def create_publisher(self, msg_type, topic: str, qos_depth: int) -> FakePublisher:
        self.publishers.append(FakePublisher(topic))
        return self.publishers[-1]

    def client(self, service_name: str) -> FakeClient:
        return next(c for c in self.clients if c.service_name == service_name)


@pytest.fixture
def node() -> FakeNode:
    return FakeNode()


@pytest.fixture
def spin_thread() -> SimpleNamespace:
    return SimpleNamespace(is_alive=lambda: True)


@pytest.fixture
def api(node, spin_thread, monkeypatch):
    """A client for the app, running on the fake node instead of ROS."""

    @asynccontextmanager
    async def fake_lifespan(app):
        yield ros_state(node, spin_thread)

    monkeypatch.setattr(app.router, "lifespan_context", fake_lifespan)
    with TestClient(app) as client:
        yield client
