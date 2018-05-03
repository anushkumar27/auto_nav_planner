# auto_nav_planner
Planner node is responsible for Dynamic Path Planning (DPP) and visualising

## Description
This module/node is responsible for taking input from the user via the ROS Framework’s 
inter-node communication mechanisms. It also collaborates with the OA Node to detect 
obstacles and avoid them. Its main responsibility is to plan a collision free path from the 
aerial robot’s initial position to the desired destination. It should also be able to handle the 
case of the destination changing before the robot has reached its destination. If the robot 
reaches the destination, it shall hover at the destination point requested by the client. These 
responsibilities are implemented through a set of published and subscribed topics as well as 
through “ROS Services” which act like an API for other nodes/modules to interact and 
invoke behavior in other nodes. It shall interact with the Bridge module to send commands to 
the robot, and the OA Node for obstacle avoidance. 
