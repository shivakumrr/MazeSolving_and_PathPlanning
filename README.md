# MazeSolving_and_PathPlanning

The goal of this project was to develop a solution to solve 4x4 rectangular mazes with the
Mycobot Pro 600. The robot's end effector had to navigate through the maze in straight-line
sections. A maze was printed on a plastic board and placed within the AI Kit's camera zone. The
algorithm was designed to automatically detect the maze's entrance, calculate the solution path,
and determine its exit based on an image captured by the AI Kit's camera. The solution path
consisted of consecutive straight lines from the maze entrance to the exit, and the robot's path
planning was visualized in its digital twin before execution. The system had to work for any 4x4
maze generated via a maze generation tool and enable the robot's end effector to autonomously
navigate the solution path.
Methodology
1. System Architecture
The project was broken down into three main components:
1. Path Planning and Simulation
o OpenCV was utilized to process the maze image, extract path points, and solve
the maze.
o Interpolation points were generated and stored in a CSV file for subsequent
stages.
o MATLAB Simulink simulated the robot's motion, computed joint angles, and
validated kinematics.
2. Data Processing
o Joint angle data were derived from inverse kinematics (IK) calculations in
Simulink.
o Errors between simulated and desired positions were minimized to improve
accuracy.
3. TCP Communication
o Python scripts enabled real-time transmission of joint angle commands to the
robotic arm via a TCP protocol.
o A server-client model facilitated seamless communication.
