import asyncio
import weakref

from rclpy.client import Client
from rclpy.task import Future


class ServiceUnavailableError(Exception):
    pass


class ServiceTimeoutError(Exception):
    pass


# Requests beyond the service QoS history depth are silently dropped by the
# middleware, so each client only gets that many calls in flight; the rest wait.
_in_flight: weakref.WeakKeyDictionary = weakref.WeakKeyDictionary()


async def call_service(client: Client, request, timeout_sec: float = 5.0):
    """Await a ROS service call from asyncio, without blocking the event loop.

    The response arrives on the executor thread that spins the client's node.
    The timeout includes time spent waiting for an in-flight slot.
    """
    if not client.service_is_ready():
        raise ServiceUnavailableError(f'Service "{client.srv_name}" is not available')

    try:
        return await asyncio.wait_for(_call(client, request), timeout_sec)
    except asyncio.TimeoutError:  # Not an alias of TimeoutError before Python 3.11.
        raise ServiceTimeoutError(
            f'Service "{client.srv_name}" did not respond within {timeout_sec}s'
        ) from None


async def _call(client: Client, request):
    slots = _in_flight.setdefault(client, asyncio.Semaphore(client.qos_profile.depth))
    async with slots:
        loop = asyncio.get_running_loop()
        result = loop.create_future()

        def transfer(ros_future: Future):
            if result.done():
                return
            if ros_future.exception() is not None:
                result.set_exception(ros_future.exception())
            else:
                result.set_result(ros_future.result())

        ros_future = client.call_async(request)
        ros_future.add_done_callback(lambda f: loop.call_soon_threadsafe(transfer, f))
        try:
            return await result
        finally:
            if not ros_future.done():
                client.remove_pending_request(ros_future)
