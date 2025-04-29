#Team 2 - Maze Drawing with Arm#
Thomas Chang, Keith Khadar, Emily Namm

This project uses homography and image detection to scale a printed-out maze into a grid, 
BFS or DFS to find a path from a given start point to a given end point, and translates 
this path into movements for a robotic arm to draw the solution to the maze.

###Setup Instructions###

Repo: https://github.com/Keith-Khadar/AuRo_ARM.git

`./scripts/install.sh` to install ROS2 and its dependencies.

On separate terminals, run `ros2 launch perception perception_launch.py` and 
`ros2 launch maxarm maxarm_launch.py` from the main workspace directory.

###User Instructions###

- The maze paper must have ArUco markers on each of its corners and placed directly in 
front of the arm. The maze itself can be either printed or drawn in black.
- Angle the camera so that each of the ArUco markers are within frame.
- Draw a blue dot on the maze to indicate the desired starting position.
- Draw a green dot on the maze to indicate the desired ending position.
