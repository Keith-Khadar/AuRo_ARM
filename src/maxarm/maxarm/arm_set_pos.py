import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

import asyncio
import websockets
import json
import threading

class Arm_Set_Pos(Node):

    def __init__(self):
        super().__init__('arm_set_pos')
        self.subscription = self.create_subscription(Point, 'goal_pose', self.write_websocket, 10)
        self.ws_url = "ws://192.168.4.1/ws"
        self.loop = asyncio.new_event_loop()
        threading.Thread(target=self._run_loop, daemon=True).start()
        
    def _run_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def write_websocket(self, msg: Point):
        data = {
            'x': msg.x,
            'y': msg.y,
            'z': msg.z,
        }
        json_str = json.dumps(data)
        self.get_logger().info(f'Sending to WebSocket: {json_str}')
        asyncio.run_coroutine_threadsafe(self.send_ws_message(json_str), self.loop)

    async def send_ws_message(self, message):
        try:
            async with websockets.connect(self.ws_url) as websocket:
                await websocket.send(message)
        except Exception as e:
            self.get_logger().warn(f"WebSocket error: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    arm_set_pos = Arm_Set_Pos()

    rclpy.spin(arm_set_pos)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    arm_set_pos.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
