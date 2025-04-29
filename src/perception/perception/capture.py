import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import ament_index_python
from .detectCircles import detectCircles
from .grid import toBitmapGrid
from .arm_scaling import homography
from .pathing import pathing_bfs
from geometry_msgs.msg import Point
import time


class ImageSubscriber(Node):
    def __init__(self):
        print("Init")
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(Image, '/camera1/image_raw', self.listener_callback, 3)
        self.subscription
        
        self.ImOut = self.create_publisher(Image, '/out/image', 3)
        self.armOut = self.create_publisher(Point, '/set_pos/goal_pose', 1)

        self.bridge = CvBridge()
        self.start_center_prev = []
        self.dest_center_prev = []
        self.count = 0

        self.isMoving = False


        loc = Point()
        loc.z = float(0)
        loc.x = float(0)
        loc.y = float(0)

        print(f"START MOVING YA BUM! {loc}")
        self.armOut.publish(loc)                        
        print("hellow?")

    def listener_callback(self, data):
        if(self.isMoving):
            return

        imCV = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8") 
    
        # Get the default frame width and height
        frame_width = int(data.width)
        frame_height = int(data.height)


        # Add Homography preprocessing
        img_scaled, worked = homography(imCV)
        
        if (worked == False):
            # Convert back to Image from Cv
            outCV = self.bridge.cv2_to_imgmsg(imCV, encoding="bgr8")
            self.ImOut.publish(outCV)
            return
        

        # Find start and dest points
        frame, start_center, dest_center, start_radius, dest_radius = detectCircles(img_scaled)

        # Convert back to Image from Cv
        outCV = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.ImOut.publish(outCV)

        # This code checks for green and blue circles that havent moved for twenty frames
        # Which will indicate user is done drawing and arm should start moving
        if(len(self.start_center_prev) > 0 and len(self.dest_center_prev) > 0):
            if(len(start_center) > 0 and len(dest_center)>0):
                if(abs(self.start_center_prev[0]-start_center[0]) <= 10 and abs(self.start_center_prev[1]-start_center[1]) <= 10 and 
                abs(self.dest_center_prev[0]-dest_center[0]) <= 10 and abs(self.dest_center_prev[1]-dest_center[1]) <= 10):
                    self.count = self.count + 1
                    #print(count)
                    if(self.count > 10):

                        self.isMoving = True
                        ## Convert image to bitmap/path - edit to add return value
                        bitmask, start, end = toBitmapGrid(img_scaled, start_center, dest_center)
                        print(f"Start: {start}, End: {end}")
                        _, directions = pathing_bfs(bitmask, start, end)
                        print(_, directions)
                        ## Add arm movement/publications here
                        current_x = start[1]
                        current_y = start[0]

                        new_y = (4.75)*(current_y - 14)
                        print(new_y)
                        new_x = (-5.428)*(current_x - 18)
                        loc = Point()
                        loc.z = float(-132)
                        loc.x = float(new_x)
                        loc.y = float(new_y)

                        print(f"START MOVING YA BUM! {loc}")
                        self.armOut.publish(loc)                        
                        
                        time.sleep(2)

                        for direction in directions:
                            current_x += direction[0]
                            current_y += direction[1]
                            
                            ## Add arm movement/publications here
                            new_y = (4.75)*(current_y - 14)
                            print(new_y)
                            new_x = (-5.428)*(current_x - 18)
                            loc = Point()
                            loc.z = float(-132)
                            loc.x = float(new_x)
                            loc.y = float(new_y)
                            self.armOut.publish(loc)                        
                            time.sleep(1)
   
                        current_x = end[1]
                        current_y = end[0]

                        new_y = (4.75)*(current_y - 14)
                        print(new_y)
                        new_x = (-5.428)*(current_x - 18)
                        loc = Point()
                        loc.z = float(-132)
                        loc.x = float(new_x)
                        loc.y = float(new_y)

                        print(f"START MOVING YA BUM! {loc}")
                        self.armOut.publish(loc)                        
                        time.sleep(2)
                        #self.isMoving = False
                else:
                    self.count = 0
                    #print("reset")
                    #print(abs(start_center_prev[0]-start_center[0]), abs(start_center_prev[1]-start_center[1]),
                    #      abs(dest_center_prev[0]-dest_center[0]),abs(dest_center_prev[1]-dest_center[1]))
                self.start_center_prev = start_center
                self.dest_center_prev = dest_center

        if(len(start_center) > 0 and len(dest_center) > 0):
            self.start_center_prev = start_center
            self.dest_center_prev = dest_center

    
def main():
    print("PLS!")
    rclpy.init()
    
    print("Creating the Image Subscriber")
    image_subscriber = ImageSubscriber()

    rclpy.spin(image_subscriber)

    # Destroy the node explicitly
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
