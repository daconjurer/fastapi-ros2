"""Get and set the parameters of any node in the ROS graph."""

import asyncio
from typing import cast, final

from rcl_interfaces.msg import (
    Parameter,
    ParameterType,
    ParameterValue,
    SetParametersResult,
)
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.client import Client
from rclpy.node import Node
from rosidl_pycommon.interface_base_classes import BaseMessage

from fastapi_ros2.graph import node_names
from fastapi_ros2.parameters.values import PythonValue, to_python, to_ros
from fastapi_ros2.services.call import call_service


class NodeNotFoundError(Exception):
    """The node is not in the ROS graph."""


class ParameterNotFoundError(Exception):
    """The node has no such parameter, or it is not set."""


class ParameterRefusedError(Exception):
    """The node refused the value, for the reason in the message."""


@final
class ParametersClientAsync:
    """Get and set the parameters of any node, through its parameter services.

    Calls fail as `call_service` does when the node does not answer.
    """

    def __init__(self, node: Node):
        self.node = node
        # ponytail: a pair of clients per node ever asked about, never removed;
        # remove them when their node leaves the graph if nodes come and go a lot.
        self._clients: dict[
            str,
            tuple[
                Client[GetParameters.Request, GetParameters.Response],
                Client[SetParameters.Request, SetParameters.Response],
            ],
        ] = {}

    async def get(self, node_name: str, name: str) -> PythonValue:
        """The parameter's value, by the node's fully qualified name."""
        return to_python(await self._get(node_name, name))

    async def set(self, node_name: str, name: str, value: PythonValue) -> PythonValue:
        """Set a parameter the node has, returning the value as it was sent.

        The value is converted to the parameter's type where JSON can't tell
        them apart (see `to_ros`); otherwise the node checks it.
        """
        ros_value = to_ros(value, await self._get(node_name, name))
        _, client = self._clients_for(node_name)
        await _discovered(client)
        request = SetParameters.Request(
            parameters=[Parameter(name=name, value=ros_value)]
        )
        response = await call_service(client, request)
        result = cast(list[SetParametersResult], response.results)[0]
        if not result.successful:
            raise ParameterRefusedError(result.reason)
        return to_python(ros_value)

    async def _get(self, node_name: str, name: str) -> ParameterValue:
        client, _ = self._clients_for(node_name)
        await _discovered(client)
        response = await call_service(client, GetParameters.Request(names=[name]))
        # C++ nodes answer an unknown name with a value that is not set, rclpy
        # nodes with no value at all.
        values = cast(list[ParameterValue], response.values)
        if not values or values[0].type == ParameterType.PARAMETER_NOT_SET:
            raise ParameterNotFoundError(
                f'Node "{node_name}" has no parameter "{name}"'
            )
        return values[0]

    def _clients_for(self, node_name: str):
        """The node's get and set parameter clients, created on first use.

        Only for nodes in the graph, so that requests can't create clients for
        any name they like.
        """
        if node_name not in node_names(self.node):
            raise NodeNotFoundError(f'Node "{node_name}" is not in the ROS graph')
        if node_name not in self._clients:
            self._clients[node_name] = (
                self.node.create_client(GetParameters, f"{node_name}/get_parameters"),
                self.node.create_client(SetParameters, f"{node_name}/set_parameters"),
            )
        return self._clients[node_name]


async def _discovered[Req: BaseMessage, Res: BaseMessage](
    client: Client[Req, Res], timeout: float = 1.0
) -> None:
    """Give a new client a moment to discover its service, which it does not
    see at once; `call_service` reports it unavailable if it still doesn't."""
    loop = asyncio.get_running_loop()
    deadline = loop.time() + timeout
    while not client.service_is_ready() and loop.time() < deadline:
        await asyncio.sleep(0.01)
