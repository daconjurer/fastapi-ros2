"""Await rclpy futures from asyncio."""

import asyncio
from typing import cast

from rclpy.task import Future


async def ros_result[T](ros_future: Future[T]) -> T:
    """Await an rclpy future's result from the event loop.

    The future completes on the executor thread that spins its node, so the
    outcome is handed to the event loop with `loop.call_soon_threadsafe`.
    """
    loop = asyncio.get_running_loop()
    result: asyncio.Future[T] = loop.create_future()

    def on_done(done: Future[T]) -> None:
        loop.call_soon_threadsafe(_transfer, done, result)

    ros_future.add_done_callback(on_done)
    return await result


def _transfer[T](ros_future: Future[T], result: asyncio.Future[T]) -> None:
    """Copy a completed rclpy future's outcome into an asyncio future.

    Runs on the event loop thread, because asyncio futures are not thread-safe.
    Does nothing if `result` is already done, which happens when the awaiting
    task was cancelled, e.g. by a timeout, before the outcome arrived.
    """
    if result.done():
        return
    exception = ros_future.exception()
    if exception is not None:
        result.set_exception(exception)
    else:
        # rclpy types the result as optional, but a future done without an
        # exception has one.
        result.set_result(cast(T, ros_future.result()))
