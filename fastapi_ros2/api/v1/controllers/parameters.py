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

from fastapi_ros2.services.call import call_service


@final
class ParameterController:
    """Get and set the parameters of any node, through its parameter services."""

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

    def _clients_for(self, node_name: str):
        if node_name not in self._clients:
            self._clients[node_name] = (
                self.node.create_client(GetParameters, f"{node_name}/get_parameters"),
                self.node.create_client(SetParameters, f"{node_name}/set_parameters"),
            )
        return self._clients[node_name]

    async def get(self, node_name: str, name: str) -> ParameterValue | None:
        """The parameter's value, or None if the node has no such parameter."""
        client, _ = self._clients_for(node_name)
        await _discovered(client)
        response = await call_service(client, GetParameters.Request(names=[name]))
        # C++ nodes answer an unknown name with a value that is not set, rclpy
        # nodes with no value at all.
        values = cast(list[ParameterValue], response.values)
        if not values or values[0].type == ParameterType.PARAMETER_NOT_SET:
            return None
        return values[0]

    async def set(
        self, node_name: str, name: str, value: ParameterValue
    ) -> SetParametersResult:
        _, client = self._clients_for(node_name)
        await _discovered(client)
        request = SetParameters.Request(parameters=[Parameter(name=name, value=value)])
        response = await call_service(client, request)
        return cast(list[SetParametersResult], response.results)[0]


async def _discovered[Req: BaseMessage, Res: BaseMessage](
    client: Client[Req, Res], timeout: float = 1.0
) -> None:
    """Give a new client a moment to discover its service, which it does not
    see at once; `call_service` reports it unavailable if it still doesn't."""
    loop = asyncio.get_running_loop()
    deadline = loop.time() + timeout
    while not client.service_is_ready() and loop.time() < deadline:
        await asyncio.sleep(0.01)
