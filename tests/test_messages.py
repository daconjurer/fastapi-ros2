import pytest
from rosidl_pycommon.interface_base_classes import BaseMessage
from geometry_msgs.msg import Point32, Polygon, Twist, Vector3
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32, Int8, String, UInt64

from fastapi_ros2.topics.messages import MessageError, message_type, to_message


def test_message_type():
    assert message_type("geometry_msgs/msg/Twist") is Twist


@pytest.mark.parametrize(
    "name",
    [
        "geometry_msgs/Twist",
        "geometry_msgs/srv/Twist",
        "nope/msg/X",
        "std_msgs/msg/Nope",
    ],
)
def test_message_type_rejects(name: str):
    with pytest.raises(MessageError):
        _ = message_type(name)


def test_to_message():
    message = to_message(Twist, {"linear": {"x": 2}, "angular": {"z": -1.5}})

    assert message == Twist(linear=Vector3(x=2.0), angular=Vector3(z=-1.5))


def test_to_message_lists():
    message = to_message(Polygon, {"points": [{"x": 1, "y": 2}, {"z": 3}]})

    assert message == Polygon(points=[Point32(x=1.0, y=2.0), Point32(z=3.0)])


@pytest.mark.parametrize(
    "cls, data, error",
    [
        (String, [], "message should be an object"),
        (Twist, {"nope": 1}, 'message has no field "nope"'),
        (Twist, {"linear": 5}, "message.linear should be an object"),
        (Twist, {"linear": {"x": "a"}}, "message.linear.x should be a number"),
        (Twist, {"linear": {"x": True}}, "message.linear.x should be a number"),
        (Float32, {"data": 1e39}, "message.data should fit in a 32-bit float"),
        (Int8, {"data": 1.5}, "message.data should be an integer (int8)"),
        (Int8, {"data": 128}, "message.data should fit in int8, -128 to 127"),
        (UInt64, {"data": -1}, "message.data should fit in uint64"),
        (Bool, {"data": 1}, "message.data should be a boolean"),
        (String, {"data": 1}, "message.data should be a string"),
        (Polygon, {"points": {"x": 1}}, "message.points should be a list"),
        (Polygon, {"points": [{"x": "a"}]}, "message.points[0].x should be a number"),
        (Imu, {"orientation_covariance": [0.0]}, "should have 9 items"),
    ],
)
def test_to_message_rejects(cls: type[BaseMessage], data: object, error: str):
    with pytest.raises(MessageError) as raised:
        _ = to_message(cls, data)

    assert error in str(raised.value)
