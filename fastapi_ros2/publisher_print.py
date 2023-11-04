import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import uvicorn
from fastapi import FastAPI
from pydantic import BaseModel


app = FastAPI()


class Response(BaseModel):
    msg: str


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'move', 10)
        self.i = 0

        @app.get('/publish', response_model=Response)
        async def publish():
            msg = String()
            msg.data = f"Hello World: {self.i}"
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing: "{msg.data}"')
            self.i += 1

            response = {"msg": ""}

            response["msg"] = "Message published"
            return response


def main(args=None):
    rclpy.init()
    minimal_publisher = MinimalPublisher()
    spin_thread = threading.Thread(target=rclpy.spin, args=(minimal_publisher,))
    spin_thread.start()
    uvicorn.run(app, port=5000, log_level='warning')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
