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
import math

class GridNode:
    def __init__(self, x, y):
        print("function", "__init__")
        self.x = x
        self.y = y


class Lecture0(Node):
    def __init__(self):
        print("function", "__init__")
        self.node = rclpy.create_node("update_map")
        # subscribe to the topic /map
        self.subscription = self.node.create_subscription(OccupancyGrid, "/map", self.callback_map, 10)
        self.subscription
        # create a number of random nodes in the free area
        self.amount_of_nodes = 10
        self.nodes_init = False
        self.random_nodes = []

        # publisher to "prm_markers"
        self.publisher = self.node.create_publisher(MarkerArray, "prm_markers", 10)
        self.publisher

        self.resolution = 0.05

        self.map_height = None
        self.map_width = None
        self.map_data = None

        self.information_gains = None

        # Action client for navigation
        self.action_client = ActionClient(self.node, NavigateToPose, '/navigate_to_pose')
        self.action_client
        print("out")



    def callback_map(self, msg):
        print("function", "callback_map")
        print("map received")
        if(not self.nodes_init):
            self.nodes_init = True

            # printe what I get in the map
            print("map data")
            # print a list of all the elements in the "msg" object
            print(dir(msg))
            print("info", msg.info)
            # prin the shape of the map
            print("shape", np.array(msg.data).shape)
            # convert the data to a matrix
            print("data", np.array(msg.data))


            self.resolution = msg.info.resolution
            self.map_height = msg.info.height
            self.map_width = msg.info.width
            self.map_data = np.array(msg.data).reshape((self.map_height, self.map_width))

            self.random_nodes = self.generate_random_nodes(msg)
            # self.random_nodes = self.generate_random_nodes()
            # compute the information gain for each node
            self.information_gains = self.compute_information_gain()
            # Select the node with the highest gain
            best_node_index = self.select_best_node()
            best_node = self.random_nodes[best_node_index]

            # Navigate to the selected node
            self.navigate_to_node(best_node)



            # reshape the data to a 3D matrix with the shape (height, width, 1)
            # publish the nodes
        self.publish_nodes()

    def compute_visible_unknown(self, node, max_distance=3):
        print("function", "compute_visible_unknown")
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
        print("function", "ray_cast_unknown")
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

    def compute_information_gain(self):
        print("function", "compute_information_gain")
        print("Computing information gain for each node")
        information_gains = []
        for node_coords in self.random_nodes:
            # Create a Node object for the current node
            node = GridNode(node_coords[0], node_coords[1])

            # Compute the visible unknown cells
            visible_unknown = self.compute_visible_unknown(node, max_distance=3)

            # Apply the exponential decay factor
            distance = 0  # Replace with the actual distance if needed
            gain = visible_unknown * math.exp(-distance)

            information_gains.append(gain)
        return information_gains

    def select_best_node(self):
        print("function", "select_best_node")
        # Find the index of the node with the highest information gain
        best_node_index = int(np.argmax(self.information_gains))
        print(f"Best node selected: {self.random_nodes[best_node_index]} with gain {self.information_gains[best_node_index]}")
        return best_node_index

    def navigate_to_node(self, node):
        print("function", "navigate_to_node")
        print(f"Navigating to node: {node}")
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
        print("function", "feedback_callback")
        feedback = feedback_msg.feedback
        # print(f"Feedback: {feedback}")

    def goal_response_callback(self, future):
        print("function", "goal_response_callback")
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
        print("Reevaluating nodes...")
        self.nodes_init = False 
        # Reevaluate the nodes and select the next one
        # self.callback_map(self.latest_map_msg)


    def publish_nodes(self):
        print("function", "publish_nodes")
        print("publishing nodes")
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
            print("node", marker.pose.position.x, marker.pose.position.y)
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
        print("function", "generate_random_nodes")
        print("Generating random nodes")
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
    print("week 9 exercises")
    rclpy.init()
    node = Lecture0()
    rclpy.spin(node.node)
    rclpy.shutdown()



if __name__ == "__main__":
    main()