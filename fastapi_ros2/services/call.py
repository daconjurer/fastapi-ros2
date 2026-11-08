import asyncio
import weakref
from typing import Any

from rclpy.client import Client
from rclpy.type_support import SrvRequestT, SrvResponseT

from fastapi_ros2.futures import ros_result


def service_ready(client: Client[Any, Any]) -> bool:  # pyright: ignore[reportExplicitAny]
    """Whether the client's service is available.

    Shutting rclpy down destroys the client, after which only False is safe to
    report, so the context is checked first.
    """
    return client.context.ok() and client.service_is_ready()


class ServiceUnavailableError(Exception):
    pass


class ServiceTimeoutError(Exception):
    pass


# Requests beyond the service QoS history depth are silently dropped by the
# middleware, so each client only gets that many calls in flight; the rest wait.
# ponytail: burst protection for this process only, not added capacity; replicas
# each get their own limit, so raise the server's QoS depth if the API scales out.
# Clients of every service type share the dict. rclpy's `Client` is invariant in its
# request and response types, so only `Any` accepts them all.
type _AnyClient = Client[Any, Any]  # pyright: ignore[reportExplicitAny]
_in_flight: weakref.WeakKeyDictionary[_AnyClient, asyncio.Semaphore] = (
    weakref.WeakKeyDictionary()
)


async def call_service(
    client: Client[SrvRequestT, SrvResponseT],
    request: SrvRequestT,
    timeout_sec: float = 5.0,
) -> SrvResponseT:
    """Await a ROS service call from asyncio, without blocking the event loop.

    The response arrives on the executor thread that spins the client's node.
    The timeout includes time spent waiting for an in-flight slot.
    """
    if not client.service_is_ready():
        raise ServiceUnavailableError(
            f'Service "{client.service_name}" is not available',
        )

    try:
        return await asyncio.wait_for(_call(client, request), timeout_sec)
    except TimeoutError:
        raise ServiceTimeoutError(
            f'Service "{client.service_name}" did not respond within {timeout_sec}s',
        ) from None


async def _call(
    client: Client[SrvRequestT, SrvResponseT],
    request: SrvRequestT,
) -> SrvResponseT:
    if client not in _in_flight:
        _in_flight[client] = asyncio.Semaphore(client.qos_profile.depth)

    async with _in_flight[client]:
        ros_future = client.call_async(request)
        try:
            return await ros_result(ros_future)
        finally:
            if not ros_future.done():
                client.remove_pending_request(ros_future)
