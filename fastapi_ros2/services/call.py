import asyncio
import weakref
from typing import Any, cast

from rclpy.client import Client
from rclpy.task import Future
from rclpy.type_support import SrvRequestT, SrvResponseT


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
            f'Service "{client.service_name}" is not available'
        )

    try:
        return await asyncio.wait_for(_call(client, request), timeout_sec)
    except TimeoutError:
        raise ServiceTimeoutError(
            f'Service "{client.service_name}" did not respond within {timeout_sec}s'
        ) from None


async def _call(
    client: Client[SrvRequestT, SrvResponseT], request: SrvRequestT
) -> SrvResponseT:
    if client not in _in_flight:
        _in_flight[client] = asyncio.Semaphore(client.qos_profile.depth)

    async with _in_flight[client]:
        loop = asyncio.get_running_loop()
        result: asyncio.Future[SrvResponseT] = loop.create_future()

        def on_done(done: Future[SrvResponseT]) -> None:
            loop.call_soon_threadsafe(_transfer, done, result)

        ros_future = client.call_async(request)
        ros_future.add_done_callback(on_done)
        try:
            return await result
        finally:
            if not ros_future.done():
                client.remove_pending_request(ros_future)


def _transfer(
    ros_future: Future[SrvResponseT], result: asyncio.Future[SrvResponseT]
) -> None:
    """Copy a completed rclpy future's outcome into an asyncio future.

    Runs on the event loop thread, scheduled from the executor thread with
    `loop.call_soon_threadsafe`, because asyncio futures are not thread-safe.
    Does nothing if `result` is already done, which happens when the call
    timed out and `asyncio.wait_for` cancelled it before the response arrived.
    """
    if result.done():
        return
    exception = ros_future.exception()
    if exception is not None:
        result.set_exception(exception)
    else:
        # rclpy types the result as optional, but a future done without an
        # exception has one.
        result.set_result(cast(SrvResponseT, ros_future.result()))
