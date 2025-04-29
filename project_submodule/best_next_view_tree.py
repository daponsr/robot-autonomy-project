#!/usr/bin/env python3
import rclpy
from enum import Enum
from typing import List

import numpy as np

from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.parameter import Parameter

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult

from tf2_ros.buffer import Buffer
from tf2_ros import LookupException, TransformException
from tf2_ros.transform_listener import TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
# import the Map
# from nav_msgs.msg import Map
# import the MarkerArray
from visualization_msgs.msg import MarkerArray, Marker
# from nav_msgs.msg import Map
import random

from typing import Tuple

from rclpy.time import Time

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion


from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
import math
import time



from tf2_ros.buffer import Buffer
from tf2_ros import LookupException, TransformException
from tf2_ros.transform_listener import TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class GridNode:
    def __init__(self, x, y):
        self.x = x
        self.y = y

class TreeNode:
    def __init__(self, x, y, parent=None, gain=0):
        self.x = x
        self.y = y
        self.parent = parent
        self.gain = gain
def calculate_total_gain(node):
    total_gain = 0
    while node is not None:
        total_gain += node.gain
        node = node.parent
    return total_gain
def visualize_tree(tree):
    markers = MarkerArray()
    for node in tree:
        if node.parent is not None:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.05
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.points = [
                Point(x=float(node.x), y=float(node.y), z=0),
                Point(x=float(node.parent.x), y=float(node.parent.y), z=0)
            ]
            markers.markers.append(marker)
    self.publisher.publish(markers)



class VisibilityComputation:
    def __init__(self, map_data, resolution):
        """
        Args:
            map_data (np.array): 2D occupancy grid.
            resolution (float): Size of each cell in meters.
        """
        self.map_data = map_data
        self.resolution = resolution
        self.map_height = map_data.shape[0]
        self.map_width = map_data.shape[1]

    def compute_visible_unknown(self, node, max_distance=3):
        """
        For a given node, count how many unknown cells (value -1)
        are visible by casting rays in 36 directions up to max_distance.
        
        Args:
            node: An object with attributes 'x' and 'y' (meters).
            max_distance (float): Maximum distance (in meters) for each ray.
        
        Returns:
            int: Total number of unknown cells encountered.
        """
        directions = 36
        max_cells = int(max_distance / self.resolution)
        total_unknown = 0
        for d in range(directions):
            angle = 2 * math.pi * d / directions
            total_unknown += self.ray_cast_unknown(node, angle, max_cells)
        return total_unknown

    def ray_cast_unknown(self, node, angle, max_cells):
        """
        Cast a ray from the node and count unknown cells (-1) until an obstacle (100)
        is encountered or the ray goes out of bounds.
        
        Args:
            node: An object with attributes 'x' and 'y' (meters).
            angle (float): Angle in radians.
            max_cells (int): Maximum steps along the ray.
        
        Returns:
            int: Count of unknown cells seen along this ray.
        """
        origin_x, origin_y = node.x, node.y
        count = 0
        for step in range(1, max_cells + 1):
            rx = origin_x + step * self.resolution * math.cos(angle)
            ry = origin_y + step * self.resolution * math.sin(angle)
            col = int(rx / self.resolution)
            row = int(ry / self.resolution)
            if not (0 <= row < self.map_height and 0 <= col < self.map_width):
                break
            val = self.map_data[row, col]
            if val == -1:
                count += 1
            elif val == 100:  # obstacle encountered
                break
        return count

    def get_rays(self, node, max_distance=3):
        """
        Compute the endpoint of each ray cast from the node for visualization.
        
        Args:
            node: An object with attributes 'x' and 'y' (meters).
            max_distance (float): Maximum distance for each ray.
        
        Returns:
            list of tuples: Each tuple is (x_start, y_start, x_end, y_end).
        """
        directions = 36
        max_cells = int(max_distance / self.resolution)
        rays = []
        for d in range(directions):
            angle = 2 * math.pi * d / directions
            origin_x, origin_y = node.x, node.y
            last_valid = (origin_x, origin_y)
            for step in range(1, max_cells + 1):
                rx = origin_x + step * self.resolution * math.cos(angle)
                ry = origin_y + step * self.resolution * math.sin(angle)
                col = int(rx / self.resolution)
                row = int(ry / self.resolution)
                if not (0 <= row < self.map_height and 0 <= col < self.map_width):
                    break
                val = self.map_data[row, col]
                if val == 100:
                    break
                last_valid = (rx, ry)
            rays.append((origin_x, origin_y, last_valid[0], last_valid[1]))
        return rays


class ExploratorNode(Node):
    def __init__(self):
        print("function", "__init__")
        self.node = rclpy.create_node("update_map")
        # subscribe to the topic /map
        self.subscription = self.node.create_subscription(OccupancyGrid, "/map", self.callback_map, 10)
        self.subscription
        # create a number of random nodes in the free area
        self.amount_of_nodes = 5
        self.nodes_init = False
        self.random_nodes = []

        self._tf_buffer = Buffer()                                          # set the transform buffer
        self._tf_listener = TransformListener(self._tf_buffer, self.node)        # set the transform listener
        # add a static transform broadcaster
        self._tf_publisher = StaticTransformBroadcaster(self.node)      # set the transform broadcaster
        tf = TransformStamped()
        tf.header.stamp = self.node.get_clock().now().to_msg()
        tf.header.frame_id = 'map'
        tf.child_frame_id = 'odom'
        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0
        self._tf_publisher.sendTransform(tf)


        # publisher to "prm_markers"
        self.publisher = self.node.create_publisher(MarkerArray, "prm_markers", 10)
        self.publisher

        # publisher for the tree
        self.tree_publisher = self.node.create_publisher(MarkerArray, "tree_markers", 10)
        self.tree_publisher

        # subscribe to the pose
        self.odom_subscription = self.node.create_subscription(Odometry, "/odom", self.callback_pose, 10)
        self.odom_subscription

        self.resolution = 0.05

        self.map_height = None
        self.map_width = None
        self.map_data = None
        self.prev_map_data = None

        self.last_map_msg = None

        self.information_gains = None
        self.to_generate = True

        # Action client for navigation
        self.action_client = ActionClient(self.node, NavigateToPose, '/navigate_to_pose')
        self.action_client

        # set the pose of the robot
        self.robot_x = None
        self.robot_y = None


        self.goal_reached = True

        self.gain_prev = [0] * self.amount_of_nodes
        self.best_node_index_prev = None

        # add a timer in case the robot gets stuck so we can regenerate the nodes
        self.timer = self.node.create_timer(1.0, self.timer_callback)
        self.timeout = time.time() + 3600 # init to 1 hour

    def timer_callback(self):
        if(time.time() > self.timeout):
            print("timeout")
            self.to_generate = True
            self.goal_reached = True
            # update the timeout
            self.timeout = time.time() + 30

    def callback_pose(self, msg):
        # set the pose of the robot
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def reconstruct_path(self, node):
        path = []
        while node is not None:
            path.append((node.x, node.y))
            node = node.parent
        return path[::-1]  # Reverse the path to start from the root


    def callback_map(self, msg):
        if(self.robot_x is None or self.robot_y is None):
            print("robot pose not set")
            return
        print("map received")
        self.latest_map_msg = msg
        self.resolution = msg.info.resolution
        self.map_height = msg.info.height
        self.map_width = msg.info.width
        self.map_data = np.array(msg.data).reshape((self.map_height, self.map_width))

        if not self.goal_reached:
            return

        # Use RRT-based information gain computation
        root_node = TreeNode(self.robot_x, self.robot_y)
        print("Root node:", (root_node.x, root_node.y))
        # best_node = self.compute_information_gain_rrt(root_node)
        # best_node = self.compute_information_gain_rrt_frontier(root_node)
        best_node = self.compute_information_gain_rrt_easy(root_node)
        path_to_best_node = self.reconstruct_path(best_node)
        print("Path to best node:", path_to_best_node)

        print("Best node selected:", (best_node.x, best_node.y))
        self.goal_reached = False
        self.timeout = time.time() + 30
        self.navigate_to_node((best_node.x, best_node.y))
        self.publish_nodes()


    def compute_visible_unknown(self, node, max_distance=3):
        """
        For a given node, count how many unknown cells (value -1)
        are visible by casting rays in 36 directions up to max_distance.
        
        Args:
            node: An object with attributes 'x' and 'y' (meters).
            max_distance (float): Maximum distance (in meters) for each ray.
        
        Returns:
            int: Total number of unknown cells encountered.
        """
        directions = 36
        max_cells = int(max_distance / self.resolution)
        total_unknown = 0
        for d in range(directions):
            angle = 2 * math.pi * d / directions
            total_unknown += self.ray_cast_unknown(node, angle, max_cells)
        return total_unknown

    def ray_cast_unknown(self, node, angle, max_cells):
        """known or occupied re
        Cast a ray from the node and count unknown cells (-1) until an obstacle (100)
        is encountered or the ray goes out of bounds.
        
        Args:
            node: An object with attributes 'x' and 'y' (meters).
            angle (float): Angle in radians.
            max_cells (int): Maximum steps along the ray.
        
        Returns:
            int: Count of unknown cells seen along this ray.
        """
        origin_x, origin_y = node.x, node.y
        count = 0
        for step in range(1, max_cells + 1):
            rx = origin_x + step * self.resolution * math.cos(angle)
            ry = origin_y + step * self.resolution * math.sin(angle)
            col = int(rx / self.resolution)
            row = int(ry / self.resolution)
            if not (0 <= row < self.map_height and 0 <= col < self.map_width):
                break
            val = self.map_data[row, col]
            if val == -1:
                count += 1
            elif val == 100:  # obstacle encountered
                break
        return count

    def get_rays(self, node, max_distance=3):
        print("function", "get_rays")
        """
        Compute the endpoint of each ray cast from the node for visualization.
        
        Args:
            node: An object with attributes 'x' and 'y' (meters).
            max_distance (float): Maximum distance for each ray.
        
        Returns:
            list of tuples: Each tuple is (x_start, y_start, x_end, y_end).
        """
        directions = 36
        max_cells = int(max_distance / self.resolution)
        rays = []
        for d in range(directions):
            angle = 2 * math.pi * d / directions
            origin_x, origin_y = node.x, node.y
            last_valid = (origin_x, origin_y)
            for step in range(1, max_cells + 1):
                rx = origin_x + step * self.resolution * math.cos(angle)
                ry = origin_y + step * self.resolution * math.sin(angle)
                col = int(rx / self.resolution)
                row = int(ry / self.resolution)
                if not (0 <= row < self.map_height and 0 <= col < self.map_width):
                    break
                val = self.map_data[row, col]
                if val == 100:
                    break
                last_valid = (rx, ry)
            rays.append((origin_x, origin_y, last_valid[0], last_valid[1]))
        return rays
    def find_frontier_cells(self):
        """
        Identify frontier cells in the map.
        
        Returns:
            list of tuples: List of (x, y) coordinates of frontier cells.
        """
        frontier_cells = []
        for row in range(1, self.map_height - 1):
            for col in range(1, self.map_width - 1):
                if self.map_data[row, col] == 0:  # Free cell
                    # Check if any neighbor is unknown (-1)
                    neighbors = [
                        self.map_data[row - 1, col],
                        self.map_data[row + 1, col],
                        self.map_data[row, col - 1],
                        self.map_data[row, col + 1],
                    ]
                    if -1 in neighbors:
                        # Convert grid indices to world coordinates
                        x = self.latest_map_msg.info.origin.position.x + col * self.resolution
                        y = self.latest_map_msg.info.origin.position.y + row * self.resolution
                        frontier_cells.append((x, y))
        return frontier_cells
    def compute_information_gain_rrt_frontier(self, root_node, max_depth=2, max_distance=3, lambda_factor=1, frontier_bias=0.7):
        """
        Compute information gain using a tree-based exploration (RRT) with a bias toward frontier cells.
        
        Args:
            root_node (TreeNode): The root node of the RRT.
            max_depth (int): Maximum depth of the tree.
            max_distance (float): Maximum distance for each ray.
            lambda_factor (float): Tuning factor for the discounting factor.
            frontier_bias (float): Probability of biasing node generation toward frontier cells.

        Returns:
            TreeNode: The node with the highest cumulative information gain.
        """
        tree = [root_node]
        best_node = root_node

        # Get the map origin and resolution
        origin_x = self.latest_map_msg.info.origin.position.x
        origin_y = self.latest_map_msg.info.origin.position.y
        resolution = self.latest_map_msg.info.resolution

        # Find frontier cells
        frontier_cells = self.find_frontier_cells()

        for depth in range(2):
            new_nodes = []
            for node in tree:
                # Generate random child nodes
                # for _ in range(3):  # Number of children per node
                i = 0
                attempts = 0
                while((i < 2) and (attempts < 100)):
                    attempts += 1
                    if random.random() < frontier_bias and frontier_cells:
                        # Bias toward frontier: Select a random frontier cell
                        frontier_x, frontier_y = random.choice(frontier_cells)
                        new_x = frontier_x + random.uniform(-resolution, resolution)
                        new_y = frontier_y + random.uniform(-resolution, resolution)
                    else:
                        # Generate random coordinates within the map bounds
                        new_x = random.uniform(origin_x, origin_x + self.map_width * resolution)
                        new_y = random.uniform(origin_y, origin_y + self.map_height * resolution)

                    # Convert to grid indices
                    col = int((new_x - origin_x) / resolution)
                    row = int((new_y - origin_y) / resolution)

                    # Check if the cell is free
                    if not (0 <= row < self.map_height and 0 <= col < self.map_width):
                        continue
                    if self.map_data[row, col] != 0:  # Skip if the cell is not free
                        continue

                    i += 1

                    # Calculate the cost (Euclidean distance) from the parent node
                    cost = math.sqrt((new_x - node.x) ** 2 + (new_y - node.y) ** 2)

                    # Calculate the visible unknown cells
                    grid_node = GridNode(new_x, new_y)
                    visible_unknown = self.compute_visible_unknown(grid_node, max_distance)

                    # Apply the gain formula with the discounting factor
                    gain = node.gain + visible_unknown * math.exp(-lambda_factor * cost)

                    # Create a new tree node
                    new_node = TreeNode(new_x, new_y, parent=node, gain=gain)
                    new_nodes.append(new_node)

                    # Update the best node
                    if gain > best_node.gain:
                        best_node = new_node

            print("Adding new nodes to the tree", len(new_nodes))
            tree.extend(new_nodes)

        # compute 



        # Visualize the tree
        self.visualize_tree(tree)

        return best_node

    def generate_random_nodes_valid(self, root, amount, max_distance = 3.0, angle_limit = [0, 2*math.pi]):
        generate_nodes = []
        while(len(generate_nodes) < amount):
            # generate a random node
            # new_x = random.uniform(root.x - max_distance, root.x + max_distance)
            # new_y = random.uniform(root.y - max_distance, root.y + max_distance)

            angle = random.uniform(angle_limit[0], angle_limit[1])
            distance = random.uniform(0, max_distance)
            print("angle", angle)

            if(angle < angle_limit[0] or angle > angle_limit[1]):
                    continue

            # convert to next_x and next_y
            new_x = root.x + distance * math.cos(angle)
            new_y = root.y + distance * math.sin(angle)

            # check if the node is valid
            col = int((new_x - self.latest_map_msg.info.origin.position.x) / self.latest_map_msg.info.resolution)
            row = int((new_y - self.latest_map_msg.info.origin.position.y) / self.latest_map_msg.info.resolution)

            if not (0 <= row < self.map_height and 0 <= col < self.map_width):
                continue
            if self.map_data[row, col] != 0:
                continue
            append_node = TreeNode(new_x, new_y, parent=root)
            generate_nodes.append(append_node)
        print("generated nodes", len(generate_nodes))
        return generate_nodes

    def compute_information_gain_rrt_easy(self, root_node, max_depth=2, max_distance=3, lambda_factor=1, frontier_bias=0.7):
        
        AMOUNT_LEVELS = 3
        AMOUNT_BRANCH = 2

        # generate the tree based on the amount of levels --> depth
        # and th eamount of branch --> new nodes from each of the nodes
        # the root_node is the root node of the tree
        tree = []
        tree.append([root_node])

        best_node = root_node
        best_gain = 0

        for i in range(AMOUNT_LEVELS):
            print("tree level", i, "nodes", len(tree[i]), "vals", tree[i])
            for parent in tree[i]:
                print("Parent node:", parent.x, parent.y)

                limit_angle = [0, 2*math.pi]
                # if(i > 0):
                #     limit_angle = [0, math.pi]

                generated_valid_nodes = self.generate_random_nodes_valid(parent, AMOUNT_BRANCH, angle_limit=limit_angle)
                print("generated valid nodes", len(generated_valid_nodes))

                for node in generated_valid_nodes:
                    # Calculate the cost (Euclidean distance) from the parent node
                    cost = math.sqrt((node.x - parent.x) ** 2 + (node.y - parent.y) ** 2)
                    # Calculate the visible unknown cells
                    grid_node = GridNode(node.x, node.y)
                    visible_unknown = self.compute_visible_unknown(grid_node, max_distance)
                    # Apply the gain formula with the discounting factor
                    gain = parent.gain + visible_unknown * math.exp(-lambda_factor * cost)
                    node.gain = gain
                    node.parent = parent
                    if(node.gain > best_gain):
                        best_gain = node.gain
                        best_node = node

                tree.append(generated_valid_nodes)

        # opbtain the parents all the way to the root node from the "best_node"

        parents_list = []
        parent = best_node.parent
        actual_node = best_node
        while(actual_node.parent != root_node):
            parents_list.append(actual_node.parent)
            best_node = actual_node
            actual_node = actual_node.parent
            print("actual node", actual_node.x, actual_node.y)
        print("found")

        # compute the gain of every path

        # convert the tree to a 1D list
        tree_1d = []
        for i in range(len(tree)):
            for j in range(len(tree[i])):
                tree_1d.append(tree[i][j])
    
        #visualize the tree
        self.visualize_tree(tree_1d)
        return best_node


    def compute_information_gain_rrt(self, root_node, max_depth=2, max_distance=3, lambda_factor=1):
        """
        Compute information gain using a tree-based exploration (RRT) with a discounting factor.
        
        Args:
            root_node (TreeNode): The root node of the RRT.
            max_depth (int): Maximum depth of the tree.
            max_distance (float): Maximum distance for each ray.
            lambda_factor (float): Tuning factor for the discounting factor.

        Returns:
            TreeNode: The node with the highest cumulative information gain.
        """
        tree = [root_node]
        best_node = root_node

        # Get the map origin and resolution
        origin_x = self.latest_map_msg.info.origin.position.x
        origin_y = self.latest_map_msg.info.origin.position.y
        resolution = self.latest_map_msg.info.resolution

        for depth in range(max_depth):
            new_nodes = []
            for node in tree:
                # Generate random child nodes anywhere in the map
                for _ in range(5):  # Number of children per node
                    # Generate random coordinates within the map bounds
                    new_x = random.uniform(origin_x, origin_x + self.map_width * resolution)
                    new_y = random.uniform(origin_y, origin_y + self.map_height * resolution)

                    # Convert to grid indices
                    col = int((new_x - origin_x) / resolution)
                    row = int((new_y - origin_y) / resolution)

                    # Check if the cell is free
                    if not (0 <= row < self.map_height and 0 <= col < self.map_width):
                        continue
                    if self.map_data[row, col] != 0:  # Skip if the cell is not free
                        continue

                    # Calculate the cost (Euclidean distance) from the parent node
                    cost = math.sqrt((new_x - node.x) ** 2 + (new_y - node.y) ** 2)

                    # Calculate the visible unknown cells
                    grid_node = GridNode(new_x, new_y)
                    visible_unknown = self.compute_visible_unknown(grid_node, max_distance)

                    # Apply the gain formula with the discounting factor
                    gain = node.gain + visible_unknown * math.exp(-lambda_factor * cost)

                    # Create a new tree node
                    new_node = TreeNode(new_x, new_y, parent=node, gain=gain)
                    new_nodes.append(new_node)

                    # Update the best node
                    if gain > best_node.gain:
                        best_node = new_node

            print("Adding new nodes to the tree", len(new_nodes))
            tree.extend(new_nodes)

        # Visualize the tree
        self.visualize_tree(tree)

        return best_node

    def visualize_tree(self, tree):
        """
        Visualize the tree in RViz using a MarkerArray.
        
        Args:
            tree (list of TreeNode): The tree to visualize.
        """
        markers = MarkerArray()
        marker_id = 0

        # Add markers for the current tree
        for node in tree:
            if node.parent is not None:
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = self.node.get_clock().now().to_msg()
                marker.id = marker_id
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD
                marker.scale.x = 0.05  # Line width
                marker.color.a = 1.0  # Alpha
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0  # Blue color

                # Add points for the line (parent to child)
                start_point = Point(x=float(node.x), y=float(node.y), z=0.0)
                end_point = Point(x=float(node.parent.x), y=float(node.parent.y), z=0.0)
                marker.points.append(start_point)
                marker.points.append(end_point)

                markers.markers.append(marker)
                marker_id += 1

        # Add delete markers for any unused IDs
        for i in range(marker_id, len(tree) * 5):  # Assuming a safe upper bound for marker IDs
            delete_marker = Marker()
            delete_marker.header.frame_id = "map"
            delete_marker.id = i
            delete_marker.action = Marker.DELETE
            markers.markers.append(delete_marker)

        # Publish the markers
        self.tree_publisher.publish(markers)


    def navigate_to_node(self, node):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = node[0]
        goal_msg.pose.pose.position.y = node[1]
        goal_msg.pose.pose.orientation.w = 1.0  # Facing forward

        # Send the goal to the action server
        self.action_client.wait_for_server()
        self._send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            print("Goal rejected")
            return

        print("Goal accepted")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        print("function", "result_callback")
        result = future.result().result
        print(f"Result: {result}")
        if(len(self.random_nodes) == 0):
            self.to_generate = True
        self.goal_reached = True


    def publish_nodes(self):
        # create the markers
        markers = MarkerArray()
        for i in range(len(self.random_nodes)):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.node.get_clock().now().to_msg()
            marker.id = i
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.pose.position.x = self.random_nodes[i][0]
            marker.pose.position.y = self.random_nodes[i][1]
            marker.pose.position.z = 0.5
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            markers.markers.append(marker)

        # publish the markers
        self.publisher.publish(markers)

    def generate_random_nodes(self, msg):
        random_nodes = []

        # Get the origin of the map
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        

        while(len(random_nodes) < self.amount_of_nodes):
            x = random.uniform(origin_x, origin_x + width * resolution)
            y = random.uniform(origin_y, origin_y + height * resolution)

            grid_x = int((x - origin_x) / resolution)
            grid_y = int((y - origin_y) / resolution)

            index = grid_y * width + grid_x
            if(0 <= index < len(msg.data) and msg.data[index] == 0):
                random_nodes.append((x, y))

        return random_nodes
            


def main():
    rclpy.init()
    node = ExploratorNode()
    rclpy.spin(node.node)
    rclpy.shutdown()



if __name__ == "__main__":
    main()