## Team 2 - Maze Solving with Arm
Thomas Chang, Keith Khadar, Emily Namm

This project uses homography and image detection to scale a printed-out maze into a grid, 
BFS or DFS to find a path from a given start point to a given end point, and translates 
this path into movements for a robotic arm to draw the solution to the maze.

### Setup Instructions

Repo: https://github.com/Keith-Khadar/AuRo_ARM.git

Using the Arduino IDE flash the ESP32 on the MaxArm with the code located in the firmware directory.

`./scripts/install.sh` to install ROS2 and its dependencies.

Connect to the Wifi provided by the ESP32.

On separate terminals, run `ros2 launch perception perception_launch.py` and 
`ros2 launch maxarm maxarm_launch.py` from the main workspace directory.

Then run RQT and open the image viewer to see the output image.

### User Instructions

- The maze paper must have ArUco markers on each of its corners and placed directly in 
front of the arm. The maze itself can be either printed or drawn in black.
- Angle the camera so that each of the ArUco markers are within frame.
- Draw a blue dot on the maze to indicate the desired starting position.
- Draw a green dot on the maze to indicate the desired ending position.

## Link to Demo
https://youtu.be/UKz4wESF0mw
