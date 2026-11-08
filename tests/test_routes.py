from array import array
import json
import os
import signal
from typing import cast

import pytest
import rclpy
from fastapi.testclient import TestClient
from example_interfaces.srv import AddTwoInts
from turtlesim_msgs.srv import TeleportAbsolute

from fastapi_ros2.api.v1.routes.graph import GraphResponse
from fastapi_ros2.services.call import call_service
from tests.conftest import Servers, api_client, ros_state, wait_until

INT64_MAX = 2**63 - 1


@pytest.mark.usefixtures("servers")
def test_sum(api: TestClient):
    response = api.post("/services/sum", json={"a": 2, "b": 3})

    assert response.status_code == 200
    assert response.json() == {"sum": 5}


@pytest.mark.parametrize(
    "body, error",
    [
        ({"a": INT64_MAX + 1, "b": 0}, "int64_overflow"),
        ({"a": INT64_MAX, "b": 1}, "sum_overflow"),
        ({"a": "two", "b": 3}, "int_parsing"),
        ({"a": 2}, "missing"),
    ],
)
def test_sum_rejects_invalid_input(
    api: TestClient, body: dict[str, object], error: str
):
    response = api.post("/services/sum", json=body)

    assert response.status_code == 422
    assert response.json()["detail"][0]["type"] == error


def test_move_turtle(api: TestClient, servers: Servers):
    response = api.post("/services/move_turtle", json={"x": 3, "y": 4, "theta": 1.5})

    assert response.status_code == 200
    assert response.json() == {}
    [request] = servers.teleports
    assert (request.x, request.y, request.theta) == (3, 4, 1.5)


@pytest.mark.parametrize(
    "x, error",
    [
        (1e39, "float32_overflow"),
        (float("nan"), "finite_number"),
        (float("inf"), "finite_number"),
    ],
)
def test_move_turtle_rejects_invalid_input(
    api: TestClient, servers: Servers, x: float, error: str
):
    # NaN and infinity are not valid JSON, but clients send them anyway. httpx
    # refuses to encode them, so encode the body with Python's json module.
    response = api.post(
        "/services/move_turtle",
        content=json.dumps({"x": x, "y": 4, "theta": 1.5}),
        headers={"Content-Type": "application/json"},
    )

    assert response.status_code == 422
    assert response.json()["detail"][0]["type"] == error
    assert servers.teleports == []


def test_service_unavailable(api: TestClient):
    response = api.post("/services/sum", json={"a": 2, "b": 3})

    assert response.status_code == 503
    assert response.json() == {"detail": 'Service "/add_two_ints" is not available'}


def test_service_timeout(
    api: TestClient, servers: Servers, monkeypatch: pytest.MonkeyPatch
):
    client = api_client(api, TeleportAbsolute)
    servers.node.destroy_service(servers.teleport)
    wait_until(lambda: not client.service_is_ready())
    servers.unresponsive_node.create_service(
        TeleportAbsolute, "/turtle1/teleport_absolute", lambda req, res: res
    )
    wait_until(client.service_is_ready)
    # Shorten the default timeout, which the clients rely on.
    monkeypatch.setattr(call_service, "__defaults__", (0.1,))

    response = api.post("/services/move_turtle", json={"x": 3, "y": 4, "theta": 1.5})

    assert response.status_code == 504
    assert response.json() == {
        "detail": 'Service "/turtle1/teleport_absolute" did not respond within 0.1s'
    }


def test_print(api: TestClient, servers: Servers):
    assert api.post("/topics/print").json() == {"count": 1}
    assert api.post("/topics/print").json() == {"count": 2}

    wait_until(lambda: len(servers.prints) == 2)
    assert servers.prints == ["Hello World: 0", "Hello World: 1"]


@pytest.mark.usefixtures("servers")
def test_health(api: TestClient):
    response = api.get("/health")

    assert response.status_code == 200
    assert response.json() == {
        "status": "ok",
        "node": {"name": "/fastapi_ros2", "spinning": True},
        "services": {"/add_two_ints": True, "/turtle1/teleport_absolute": True},
    }


def test_health_with_a_service_down(api: TestClient, servers: Servers):
    client = api_client(api, AddTwoInts)
    servers.add_two_ints.destroy_service(servers.add_two_ints.srv)
    wait_until(lambda: not client.service_is_ready())

    response = api.get("/health")

    assert response.status_code == 503
    assert response.json()["status"] == "unavailable"
    assert response.json()["services"] == {
        "/add_two_ints": False,
        "/turtle1/teleport_absolute": True,
    }


@pytest.mark.usefixtures("servers")
def test_a_failed_callback_stops_the_api(
    api: TestClient, monkeypatch: pytest.MonkeyPatch
):
    signals: list[tuple[int, int]] = []

    def kill(pid: int, sig: int) -> None:
        signals.append((pid, sig))

    monkeypatch.setattr(os, "kill", kill)

    def fail() -> None:
        raise RuntimeError("A callback failed")

    # An exception in any callback ends the executor's spin thread.
    ros_state(api).node.create_timer(0.001, fail)
    wait_until(lambda: not ros_state(api).spin_thread.is_alive())

    # The API stops itself, as on `docker compose stop`, so Docker restarts it.
    assert signals == [(os.getpid(), signal.SIGTERM)]
    # Until it has stopped, /health reports it.
    response = api.get("/health")
    assert response.status_code == 503
    assert response.json()["node"]["spinning"] is False


# Shutting rclpy down from outside, as its SIGINT/SIGTERM handler does, ends
# every executor's spin.
@pytest.mark.usefixtures("servers")
def test_health_after_rclpy_shutdown(api: TestClient):
    rclpy.shutdown()

    response = api.get("/health")

    assert response.status_code == 503
    assert response.json()["node"]["spinning"] is False
    assert response.json()["services"] == {
        "/add_two_ints": False,
        "/turtle1/teleport_absolute": False,
    }


@pytest.mark.parametrize("path", ["/services/sum", "/services/move_turtle"])
@pytest.mark.parametrize("status", ["503", "504"])
def test_service_errors_are_documented(api: TestClient, path: str, status: str):
    openapi = api.get("/openapi.json")

    assert status in openapi.json()["paths"][path]["post"]["responses"]


@pytest.mark.usefixtures("servers")
def test_graph(api: TestClient):
    def graph() -> GraphResponse:
        return GraphResponse.model_validate(api.get("/graph").json())

    # Discovery is asynchronous, so wait for the action server to show up.
    wait_until(lambda: "/turtle1/rotate_absolute" in graph().actions)
    body = graph()

    assert {"/fastapi_ros2", "/test_servers", "/add_two_ints_service"} <= set(
        body.nodes
    )
    assert body.topics["/print"] == ["std_msgs/msg/String"]
    assert body.services["/add_two_ints"] == ["example_interfaces/srv/AddTwoInts"]
    assert body.actions == {
        "/turtle1/rotate_absolute": ["turtlesim_msgs/action/RotateAbsolute"]
    }
    # The topics and services the action is built on are hidden.
    assert [n for n in [*body.topics, *body.services] if "/_action/" in n] == []


@pytest.mark.usefixtures("servers")
def test_get_parameter(api: TestClient):
    response = api.get("/nodes/test_servers/parameters/background_r")

    assert response.status_code == 200
    assert response.json() == {"name": "background_r", "value": 69}


@pytest.mark.usefixtures("servers")
@pytest.mark.parametrize(
    "path, detail",
    [
        ("/nodes/nowhere/parameters/x", 'Node "/nowhere" is not in the ROS graph'),
        (
            "/nodes/test_servers/parameters/nothing",
            'Node "/test_servers" has no parameter "nothing"',
        ),
    ],
)
def test_parameter_not_found(api: TestClient, path: str, detail: str):
    assert api.get(path).status_code == 404
    response = api.put(path, json={"value": 1})

    assert response.status_code == 404
    assert response.json() == {"detail": detail}


@pytest.mark.parametrize(
    "name, value, stored",
    [
        ("background_r", 255, 255),
        # JSON clients send whole doubles as integers.
        ("scale", 2, 2.0),
        ("names", ["b", "c"], ["b", "c"]),
        ("names", [], []),
        ("ids", [4, 5], [4, 5]),
        # JSON clients send whole doubles as integers, in arrays too.
        ("weights", [1, 2.5], [1.0, 2.5]),
        ("weights", [3], [3.0]),
    ],
)
def test_set_parameter(
    api: TestClient, servers: Servers, name: str, value: object, stored: object
):
    path = f"/nodes/test_servers/parameters/{name}"

    response = api.put(path, json={"value": value})

    assert response.status_code == 200
    assert response.json() == {"name": name, "value": stored}
    actual = cast(object, servers.node.get_parameter(name).value)
    assert (actual.tolist() if isinstance(actual, array) else actual) == stored
    assert api.get(path).json()["value"] == stored


@pytest.mark.parametrize(
    "value, reason",
    [
        (256, "out of range"),
        ("red", "Wrong parameter type"),
    ],
)
def test_set_parameter_refused(
    api: TestClient, servers: Servers, value: object, reason: str
):
    response = api.put(
        "/nodes/test_servers/parameters/background_r", json={"value": value}
    )

    assert response.status_code == 422
    assert reason in response.json()["detail"]
    assert cast(object, servers.node.get_parameter("background_r").value) == 69


@pytest.mark.usefixtures("servers")
def test_set_parameter_rejects_invalid_input(api: TestClient):
    response = api.put(
        "/nodes/test_servers/parameters/background_r", json={"value": {"r": 1}}
    )

    assert response.status_code == 422


def test_parameter_timeout(
    api: TestClient, servers: Servers, monkeypatch: pytest.MonkeyPatch
):
    # The node is in the graph with its parameter services, but never answers.
    _ = servers
    monkeypatch.setattr(call_service, "__defaults__", (0.1,))

    response = api.get("/nodes/test_unresponsive_servers/parameters/x")

    assert response.status_code == 504


@pytest.mark.usefixtures("servers")
def test_health_leaves_out_parameter_services(api: TestClient):
    _ = api.get("/nodes/test_servers/parameters/background_r")

    services = cast(dict[str, bool], api.get("/health").json()["services"])
    assert services.keys() == {
        "/add_two_ints",
        "/turtle1/teleport_absolute",
    }
