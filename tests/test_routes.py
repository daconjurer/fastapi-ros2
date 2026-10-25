import json

import pytest
import rclpy
from example_interfaces.srv import AddTwoInts
from turtlesim_msgs.srv import TeleportAbsolute

from fastapi_ros2.services.call import call_service
from tests.conftest import api_client, wait_until

INT64_MAX = 2**63 - 1


def test_sum(api, servers):
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
def test_sum_rejects_invalid_input(api, body, error):
    response = api.post("/services/sum", json=body)

    assert response.status_code == 422
    assert response.json()["detail"][0]["type"] == error


def test_move_turtle(api, servers):
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
def test_move_turtle_rejects_invalid_input(api, servers, x, error):
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


def test_service_unavailable(api):
    response = api.post("/services/sum", json={"a": 2, "b": 3})

    assert response.status_code == 503
    assert response.json() == {"detail": 'Service "/add_two_ints" is not available'}


def test_service_timeout(api, servers, monkeypatch):
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


def test_print(api, servers):
    assert api.post("/topics/print").json() == {"count": 1}
    assert api.post("/topics/print").json() == {"count": 2}

    wait_until(lambda: len(servers.prints) == 2)
    assert servers.prints == ["Hello World: 0", "Hello World: 1"]


def test_health(api, servers):
    response = api.get("/health")

    assert response.status_code == 200
    assert response.json() == {
        "status": "ok",
        "node": {"name": "/fastapi_ros2", "spinning": True},
        "services": {"/add_two_ints": True, "/turtle1/teleport_absolute": True},
    }


def test_health_with_a_service_down(api, servers):
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


@pytest.mark.filterwarnings("ignore::pytest.PytestUnhandledThreadExceptionWarning")
def test_health_with_the_executor_stopped(api, servers):
    def fail():
        raise RuntimeError("A callback failed")

    # An exception in any callback ends the executor's spin thread.
    api.app_state["node"].create_timer(0.001, fail)
    wait_until(lambda: not api.app_state["spin_thread"].is_alive())

    response = api.get("/health")

    assert response.status_code == 503
    assert response.json()["node"]["spinning"] is False


# Shutting rclpy down from outside ends every executor's spin thread with an error.
@pytest.mark.filterwarnings("ignore::pytest.PytestUnhandledThreadExceptionWarning")
def test_health_after_rclpy_shutdown(api, servers):
    rclpy.shutdown()

    response = api.get("/health")

    assert response.status_code == 503
    assert response.json()["node"]["spinning"] is False
    assert set(response.json()["services"].values()) == {False}


def test_service_errors_are_documented(api):
    paths = api.get("/openapi.json").json()["paths"]

    for path in ("/services/sum", "/services/move_turtle"):
        assert {"503", "504"} <= paths[path]["post"]["responses"].keys()
