import signal

from example_interfaces.srv import AddTwoInts  # type: ignore

import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore


def wrap_int64(value: int) -> int:
    """Wrap an integer into the int64 range, like two's complement arithmetic.

    >>> wrap_int64(2**63)
    -9223372036854775808
    >>> wrap_int64(-(2**63) - 1)
    9223372036854775807
    >>> wrap_int64(5)
    5
    """
    return (value + 2**63) % 2**64 - 2**63


class AddTwoIntsService(Node):
    def __init__(self):
        super().__init__("add_two_ints_service")
        self.srv = self.create_service(
            AddTwoInts, "add_two_ints", self.add_two_ints_callback
        )

    def add_two_ints_callback(self, request, response):
        self.get_logger().info(f"Incoming request: a = {request.a}, b = {request.b}")
        # `sum` is an int64, and an out-of-range value fails to serialize and stops
        # the server. Wrap around instead, like ROS's C++ demo server
        # (demo_nodes_cpp), whose int64 addition wraps silently.
        response.sum = wrap_int64(request.a + request.b)
        self.get_logger().info(f"Response: {response}")

        return response


def main():
    # On SIGTERM (`docker compose stop`), Lyrical's rclpy shuts the context down
    # but leaves `spin` blocked, so the node never leaves the ROS graph cleanly.
    # Stop like on Ctrl+C instead: rclpy forwards the signal to this handler.
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    rclpy.init()
    minimal_service = AddTwoIntsService()
    try:
        rclpy.spin(minimal_service)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_service.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
