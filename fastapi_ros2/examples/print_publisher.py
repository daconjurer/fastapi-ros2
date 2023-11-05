import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import uvicorn
from fastapi import FastAPI
from pydantic import BaseModel


app = FastAPI()


class PrintResponse(BaseModel):
    msg: str


class PrintPublisher(Node):
    def __init__(self):
        super().__init__('print_publisher')
        self.publisher_ = self.create_publisher(String, 'print', 10)
        self.i = 0

        @app.get('/topics/print', response_model=PrintResponse)
        async def publish_print():
            response = PrintResponse(msg="")

            msg = String()
            msg.data = f"Hello World: {self.i}"
            self.get_logger().info(
                f'Publishing "{msg.data}" to topic "{self.publisher_.topic_name}"'
            )
            self.publisher_.publish(msg)
            self.i += 1

            message = f'Message "{msg.data}" published to topic "{self.publisher_.topic_name}"'
            self.get_logger().info(message)

            response.msg = message
            return response


def main(args=None):
    rclpy.init()
    print_publisher = PrintPublisher()
    spin_thread = threading.Thread(target=rclpy.spin, args=(print_publisher,))
    spin_thread.start()
    uvicorn.run(app, port=5000, log_level='warning')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
