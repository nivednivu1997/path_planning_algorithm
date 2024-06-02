import os
import rclpy
from rclpy.node import Node
import numpy as np
import yaml
from PIL import Image
import heapq
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')
        
        self.declare_parameter('map_yaml_path', 'maps/map.yaml')
        map_yaml_path = self.get_parameter('map_yaml_path').get_parameter_value().string_value
        
        # Load map parameters
        with open(map_yaml_path, 'r') as file:
            self.map_info = yaml.safe_load(file)
        
        # Get the directory of the yaml file
        map_dir = os.path.dirname(map_yaml_path)
        image_path = os.path.join(map_dir, self.map_info['image'])
        
        self.map_image = self.load_map_image(image_path)

        if self.map_image is None:
            self.get_logger().error(f"Failed to load map image from path: {image_path}")
            return
        else:
            self.get_logger().info(f"Map image loaded successfully from path: {image_path}")

        self.map_resolution = self.map_info['resolution']
        self.map_origin = np.array(self.map_info['origin'])

        self.start = np.array([-6.29, -2.96])
        self.goal = np.array([0.477, 1.55])

        self.path_pub = self.create_publisher(Path, '/computed_path', 10)

        self.plan_path()

    def load_map_image(self, image_path):
        try:
            with Image.open(image_path) as img:
                map_image = np.array(img.convert('L'))
                return map_image
        except Exception as e:
            self.get_logger().error(f"Error loading map image: {e}")
            return None

    def world_to_map(self, world_point):
        map_point = ((world_point - self.map_origin[:2]) / self.map_resolution).astype(int)
        self.get_logger().info(f"Converted world point {world_point} to map point {map_point}")
        return map_point

    def map_to_world(self, map_point):
        world_point = map_point * self.map_resolution + self.map_origin[:2]
        self.get_logger().info(f"Converted map point {map_point} to world point {world_point}")
        return world_point

    def plan_path(self):
        self.get_logger().info("Starting to plan path...")
        start_map = tuple(self.world_to_map(self.start))
        goal_map = tuple(self.world_to_map(self.goal))
        self.get_logger().info(f"Start map coordinates: {start_map}")
        self.get_logger().info(f"Goal map coordinates: {goal_map}")

        # Perform A* search
        path = self.a_star(start_map, goal_map)

        if path:
            path_msg = Path()
            path_msg.header.frame_id = "map"
            for point in path:
                pose = PoseStamped()
                pose.header.frame_id = "map"
                world_point = self.map_to_world(np.array(point))
                pose.pose.position.x = world_point[0]
                pose.pose.position.y = world_point[1]
                path_msg.poses.append(pose)
            self.path_pub.publish(path_msg)
            self.get_logger().info("Path published successfully")
        else:
            self.get_logger().info("Path not found")

    def a_star(self, start, goal):
        self.get_logger().info("Starting A* algorithm...")
        def heuristic(a, b):
            return np.linalg.norm(np.array(a) - np.array(b))

        neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0)]

        close_set = set()
        came_from = {}
        gscore = {start: 0}
        fscore = {start: heuristic(start, goal)}
        oheap = []

        heapq.heappush(oheap, (fscore[start], start))
        
        while oheap:
            current = heapq.heappop(oheap)[1]
            self.get_logger().info(f"Current node: {current}")

            if current == goal:
                self.get_logger().info("Goal reached!")
                data = []
                while current in came_from:
                    data.append(current)
                    current = came_from[current]
                data.append(start)
                self.get_logger().info(f"Path found: {data[::-1]}")
                return data[::-1]

            close_set.add(current)
            for i, j in neighbors:
                neighbor = current[0] + i, current[1] + j
                tentative_g_score = gscore[current] + heuristic(current, neighbor)
                
                if 0 <= neighbor[0] < self.map_image.shape[0]:
                    if 0 <= neighbor[1] < self.map_image.shape[1]:                
                        if self.map_image[neighbor[0]][neighbor[1]] == 0:
                            continue
                    else:
                        continue
                else:
                    continue
                
                if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                    continue

                if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in oheap]:
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g_score
                    fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    heapq.heappush(oheap, (fscore[neighbor], neighbor))

        self.get_logger().info("No path found")
        return False

def main(args=None):
    rclpy.init(args=args)
    path_planner = PathPlanner()
    rclpy.spin(path_planner)
    path_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
