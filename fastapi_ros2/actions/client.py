"""Send goals to a ROS action server and follow them from asyncio."""

import asyncio
import uuid
from collections.abc import AsyncIterator
from dataclasses import dataclass, field
from typing import Any, final

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node
from rclpy.type_support import Action, FeedbackMessage
from rosidl_pycommon.interface_base_classes import BaseImpl, BaseMessage
from unique_identifier_msgs.msg import UUID

from fastapi_ros2.futures import ros_result


class ActionUnavailableError(Exception):
    """The action server is not available."""


class ActionTimeoutError(Exception):
    """The action server did not answer in time."""


class GoalRejectedError(Exception):
    """The action server rejected the goal."""


class GoalNotFoundError(Exception):
    """No goal has this id, or it finished too long ago."""


class CancelRejectedError(Exception):
    """The goal can't be canceled, for the reason in the message."""


_STATUSES = {
    GoalStatus.STATUS_SUCCEEDED: "succeeded",
    GoalStatus.STATUS_CANCELED: "canceled",
    GoalStatus.STATUS_ABORTED: "aborted",
}
"""The terminal goal statuses, by name."""

_CANCEL_ERRORS = {
    CancelGoal.Response.ERROR_REJECTED: "The action server refused to cancel the goal",
    CancelGoal.Response.ERROR_UNKNOWN_GOAL_ID: "The action server has no such goal",
    CancelGoal.Response.ERROR_GOAL_TERMINATED: "The goal has already finished",
}
"""Why a cancel request failed, by its return code."""


@dataclass(frozen=True)
class Feedback[F]:
    """Feedback the action server published for a goal."""

    feedback: F


@dataclass(frozen=True)
class Outcome[R]:
    """How a goal finished, and its result."""

    status: str
    result: R


type Event[R, F] = Feedback[F] | Outcome[R]
"""What a goal's listeners receive: feedback, then exactly one outcome."""


@dataclass
class _Goal[R: BaseMessage, F: BaseMessage]:
    """A goal sent from this process, and who is listening to it."""

    handle: ClientGoalHandle[Any, R, F, Any] | None = None  # pyright: ignore[reportExplicitAny]
    outcome: Outcome[R] | None = None
    listeners: set[asyncio.Queue[Event[R, F]]] = field(default_factory=set)

    def notify(self, event: Event[R, F]) -> None:
        # ponytail: unbounded queues, fine for turtlesim's ~60 feedback messages
        # a second; keep only the latest feedback if listeners fall behind.
        for queue in self.listeners:
            queue.put_nowait(event)


@final
class ActionClientAsync[
    G: BaseMessage,
    R: BaseMessage,
    F: BaseMessage,
    I: BaseImpl[Any, Any, Any],
]:
    """Send goals to an action server, and follow their feedback and outcome.

    Goals are tracked by id, so that one request can send a goal and others can
    follow or cancel it. Calls to the server fail like `call_service` does.
    """

    def __init__(
        self,
        node: Node,
        action_type: type[Action[G, R, F, I]],
        name: str,
        keep_finished: float = 60.0,
    ):
        self.node = node
        self.client = ActionClient(node, action_type, name)
        self.name = name
        # How long a finished goal's outcome stays available to new listeners.
        self._keep_finished = keep_finished
        self._goals: dict[str, _Goal[R, F]] = {}
        self._tasks: set[asyncio.Task[None]] = set()

    def is_ready(self) -> bool:
        """Whether the action server is available, False once rclpy is shut down."""
        return self.node.context.ok() and self.client.server_is_ready()

    async def send(self, goal: G, timeout_sec: float = 5.0) -> str:
        """Send a goal and return its id once the server accepts it."""
        if not self.client.server_is_ready():
            raise ActionUnavailableError(f'Action "{self.name}" is not available')
        loop = asyncio.get_running_loop()
        goal_id = uuid.uuid4()
        key = str(goal_id)
        tracked: _Goal[R, F] = _Goal()
        self._goals[key] = tracked

        def on_feedback(message: FeedbackMessage[F]) -> None:
            # Runs on the executor thread.
            _ = loop.call_soon_threadsafe(tracked.notify, Feedback(message.feedback))

        future = self.client.send_goal_async(
            goal,
            feedback_callback=on_feedback,
            goal_uuid=UUID(uuid=list(goal_id.bytes)),
        )
        try:
            handle = await asyncio.wait_for(ros_result(future), timeout_sec)
        except TimeoutError:
            del self._goals[key]
            raise ActionTimeoutError(
                f'Action "{self.name}" did not answer the goal within {timeout_sec}s',
            ) from None
        if not handle.accepted:
            del self._goals[key]
            raise GoalRejectedError(f'Action "{self.name}" rejected the goal')
        tracked.handle = handle
        task = loop.create_task(self._follow(key, tracked, handle))
        self._tasks.add(task)
        task.add_done_callback(self._tasks.discard)
        return key

    def events(self, goal_id: str) -> AsyncIterator[Event[R, F]]:
        """The goal's feedback from now on, then its outcome.

        A goal that has finished yields only its outcome.
        """
        tracked = self._goal(goal_id)
        if tracked.outcome is not None:
            return _once(tracked.outcome)
        # Registered before the first await, so no event is missed.
        queue: asyncio.Queue[Event[R, F]] = asyncio.Queue()
        tracked.listeners.add(queue)
        return _until_outcome(tracked, queue)

    async def cancel(self, goal_id: str, timeout_sec: float = 5.0) -> None:
        """Ask the server to cancel a goal; its outcome then says if it did."""
        tracked = self._goal(goal_id)
        if tracked.outcome is not None or tracked.handle is None:
            raise CancelRejectedError(
                _CANCEL_ERRORS[CancelGoal.Response.ERROR_GOAL_TERMINATED],
            )
        try:
            response = await asyncio.wait_for(
                ros_result(tracked.handle.cancel_goal_async()),
                timeout_sec,
            )
        except TimeoutError:
            raise ActionTimeoutError(
                f'Action "{self.name}" did not answer the cancel request within {timeout_sec}s',
            ) from None
        if response.return_code != CancelGoal.Response.ERROR_NONE:
            raise CancelRejectedError(
                _CANCEL_ERRORS.get(response.return_code, "The goal can't be canceled"),
            )

    def destroy(self) -> None:
        """Stop following goals and destroy the client, which `destroy_node` skips."""
        for task in self._tasks:
            _ = task.cancel()
        self.client.destroy()

    def _goal(self, goal_id: str) -> _Goal[R, F]:
        if goal_id not in self._goals:
            raise GoalNotFoundError(f'No goal "{goal_id}" for action "{self.name}"')
        return self._goals[goal_id]

    async def _follow(
        self,
        key: str,
        tracked: _Goal[R, F],
        handle: ClientGoalHandle[Any, R, F, Any],  # pyright: ignore[reportExplicitAny]
    ) -> None:
        """Wait for the goal's result, tell the listeners, and forget it later.

        ponytail: a goal whose server dies before answering is never forgotten;
        give up after a deadline if servers come and go.
        """
        response = await ros_result(handle.get_result_async())
        status = _STATUSES.get(response.status, "unknown")
        tracked.outcome = Outcome(status, response.result)
        tracked.notify(tracked.outcome)
        tracked.listeners.clear()
        _ = asyncio.get_running_loop().call_later(
            self._keep_finished,
            self._goals.pop,
            key,
            None,
        )


async def _once[R, F](outcome: Outcome[R]) -> AsyncIterator[Event[R, F]]:
    yield outcome


async def _until_outcome[R: BaseMessage, F: BaseMessage](
    tracked: _Goal[R, F],
    queue: asyncio.Queue[Event[R, F]],
) -> AsyncIterator[Event[R, F]]:
    try:
        while True:
            event = await queue.get()
            yield event
            if isinstance(event, Outcome):
                return
    finally:
        tracked.listeners.discard(queue)
