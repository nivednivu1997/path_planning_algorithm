# path_planning_algorithm
A* Algorithm


The chosen path planning algorithm for this task is the A* (A-star) algorithm, which is one of the most widely used algorithms for pathfinding and graph traversal. A* is an informed search algorithm that combines the strengths of Dijkstra's algorithm and greedy best-first search. It uses a heuristic to estimate the cost to reach the goal from a given node, making it more efficient in finding the shortest path compared to uninformed search algorithms.

Suitability for the Task:

Optimality: A* is guaranteed to find the shortest path if the heuristic used is admissible (i.e., it never overestimates the cost to reach the goal). This makes it ideal for applications where the optimal path is required, such as in robotics navigation.
Efficiency: By using a heuristic, A* can significantly reduce the number of nodes it needs to explore, especially in large maps. This efficiency is crucial for real-time applications like autonomous navigation.
Flexibility: The A* algorithm can be adapted to different environments and constraints by modifying the heuristic function. This makes it versatile for various types of maps and obstacle configurations.
Deterministic: Unlike probabilistic algorithms, A* will always produce the same path for the same input, which is important for consistency in navigation tasks.
Given these properties, A* is a suitable choice for path planning between two points in a known environment represented by a map.

Implementation Details of the Path Planning Node
The path planning node is implemented using ROS 2, and it leverages the A* algorithm to compute the optimal path between two specified points. Here are the key implementation details:

Node Initialization
Node Creation: The node is initialized by inheriting from Node, which is the base class for all ROS 2 nodes.
Parameters: The node declares a parameter map_yaml_path which specifies the path to the map's YAML file containing metadata about the map image, resolution, and origin.
Map Loading: The node reads the YAML file to get the path of the map image and other map parameters. The map image is then loaded using the PIL (Pillow) library and converted to a grayscale NumPy array for further processing.
Coordinate Conversion
World to Map Coordinates: The node includes functions to convert world coordinates to map coordinates and vice versa. This is crucial for accurately placing the start and goal points on the map grid.
Map to World Coordinates: This function converts grid coordinates back to world coordinates to publish the path in a format suitable for visualization in RViz.
Path Planning
Start and Goal Points: The start and goal points are defined in world coordinates and converted to map coordinates for processing.
A Algorithm*:
Heuristic Function: The Euclidean distance is used as the heuristic function, which estimates the cost from the current node to the goal.
Neighbor Nodes: The algorithm considers four possible movements (up, down, left, right) for each node.
Cost Calculation: The algorithm calculates the cost for each neighbor node and selects the path with the lowest estimated total cost.
Path Reconstruction: Once the goal is reached, the path is reconstructed from the goal back to the start using the recorded predecessors.
Path Publishing
ROS 2 Path Message: The computed path is converted into a nav_msgs/Path message. Each point in the path is converted to a geometry_msgs/PoseStamped message.
Publishing: The path is published on a dedicated topic /computed_path for visualization in RViz.
Visualization
RViz Configuration: An RViz configuration file is used to set up the visualization environment. This file specifies the map and the path topic to visualize the computed path.
Launch File: The launch file starts the map server to publish the map data, the path planning node to compute and publish the path, and RViz to visualize the path.




Steps to run
1) git clone https://github.com/nivednivu1997/path_planning_algorithm.git
2) cd ros2_ws
3) colcon build
4) source install setup.bash
5) ros2 launch path_planning path_planner.launch.xml

