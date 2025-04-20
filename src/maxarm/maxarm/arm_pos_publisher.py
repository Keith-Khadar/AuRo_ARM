import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

import asyncio
import websockets
import json

class Arm_Pos_Publisher(Node):

    def __init__(self):
        super().__init__('arm_position')
        self.publisher_ = self.create_publisher(Point, 'odom', 10)


ws_url = "ws://192.168.4.1/ws"

async def read_websocket(ws_url, ros_node):
    async with websockets.connect(ws_url) as websocket:
        ros_node.get_logger().info("Connected to WebSocket")

        while rclpy.ok():
            try:
                message_json = await websocket.recv()
                message = json.loads(message_json)

                ros_node.get_logger().debug(message)
                
                msg = Point()
                msg.x = message["x"]
                msg.y = message["y"]
                msg.z = message["z"]
                
                ros_node.publisher_.publish(msg)
            except websockets.exceptions.ConnectionClosed as e:
                ros_node.get_logger().error("WebSocket Closed")
                break

def main(args=None):
    rclpy.init(args=args)

    arm_pos_publisher = Arm_Pos_Publisher()

    # start the WebSocket listner async
    loop = asyncio.get_event_loop()
    loop.run_until_complete(read_websocket(ws_url, arm_pos_publisher))

    rclpy.spin(arm_pos_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    arm_pos_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
