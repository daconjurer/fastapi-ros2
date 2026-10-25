from example_interfaces.srv import AddTwoInts
import pytest

from fastapi_ros2.services.call import call_service

INT64_MAX = 2**63 - 1


def test_sum(api, node):
    node.client("/add_two_ints").response = AddTwoInts.Response(sum=5)

    response = api.post("/services/sum", json={"a": 2, "b": 3})

    assert response.status_code == 200
    assert response.json() == {"sum": 5}
    [request] = node.client("/add_two_ints").requests
    assert (request.a, request.b) == (2, 3)


@pytest.mark.parametrize(
    "body, error",
    [
        ({"a": INT64_MAX + 1, "b": 0}, "int64_overflow"),
        ({"a": INT64_MAX, "b": 1}, "sum_overflow"),
        ({"a": "two", "b": 3}, "int_parsing"),
        ({"a": 2}, "missing"),
    ],
)
def test_sum_rejects_invalid_input(api, node, body, error):
    response = api.post("/services/sum", json=body)

    assert response.status_code == 422
    assert response.json()["detail"][0]["type"] == error
    assert node.client("/add_two_ints").requests == []


def test_move_turtle(api, node):
    response = api.post("/services/move_turtle", json={"x": 3, "y": 4, "theta": 1.5})

    assert response.status_code == 200
    assert response.json() == {}
    [request] = node.client("/turtle1/teleport_absolute").requests
    assert (request.x, request.y, request.theta) == (3, 4, 1.5)


@pytest.mark.parametrize(
    "x, error",
    [
        (1e39, "float32_overflow"),
        (float("nan"), "finite_number"),
        (float("inf"), "finite_number"),
    ],
)
def test_move_turtle_rejects_invalid_input(api, node, x, error):
    # NaN and infinity are not valid JSON, but Python's encoder sends them anyway.
    response = api.post("/services/move_turtle", json={"x": x, "y": 4, "theta": 1.5})

    assert response.status_code == 422
    assert response.json()["detail"][0]["type"] == error
    assert node.client("/turtle1/teleport_absolute").requests == []


def test_service_unavailable(api, node):
    node.client("/add_two_ints").ready = False

    response = api.post("/services/sum", json={"a": 2, "b": 3})

    assert response.status_code == 503
    assert response.json() == {"detail": 'Service "/add_two_ints" is not available'}


def test_service_timeout(api, node, monkeypatch):
    node.client("/turtle1/teleport_absolute").response = None
    # Shorten the default timeout, which the clients rely on.
    monkeypatch.setattr(call_service, "__defaults__", (0.01,))

    response = api.post("/services/move_turtle", json={"x": 3, "y": 4, "theta": 1.5})

    assert response.status_code == 504
    assert response.json() == {
        "detail": 'Service "/turtle1/teleport_absolute" did not respond within 0.01s'
    }


def test_print(api, node):
    assert api.post("/topics/print").json() == {"count": 1}
    assert api.post("/topics/print").json() == {"count": 2}

    [publisher] = node.publishers
    assert publisher.topic_name == "/print"
    assert [msg.data for msg in publisher.published] == [
        "Hello World: 0",
        "Hello World: 1",
    ]


def test_health(api):
    response = api.get("/health")

    assert response.status_code == 200
    assert response.json() == {
        "status": "ok",
        "node": {"name": "/fastapi_ros2", "spinning": True},
        "services": {"/add_two_ints": True, "/turtle1/teleport_absolute": True},
    }


def test_health_with_a_service_down(api, node):
    node.client("/add_two_ints").ready = False

    response = api.get("/health")

    assert response.status_code == 503
    assert response.json()["status"] == "unavailable"
    assert response.json()["services"] == {
        "/add_two_ints": False,
        "/turtle1/teleport_absolute": True,
    }


def test_health_with_the_executor_stopped(api, spin_thread):
    spin_thread.is_alive = lambda: False

    response = api.get("/health")

    assert response.status_code == 503
    assert response.json()["node"]["spinning"] is False


def test_health_after_rclpy_shutdown(api, node):
    node.context.ok = lambda: False

    response = api.get("/health")

    assert response.status_code == 503
    assert response.json()["node"]["spinning"] is False
    assert set(response.json()["services"].values()) == {False}


def test_service_errors_are_documented(api):
    paths = api.get("/openapi.json").json()["paths"]

    for path in ("/services/sum", "/services/move_turtle"):
        assert {"503", "504"} <= paths[path]["post"]["responses"].keys()
