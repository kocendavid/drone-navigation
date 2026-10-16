"""ROS map, route and pose for RViz on the controller's Fast DDS network."""

import math

import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def map_message(snapshot, z):
    message = OccupancyGrid()
    message.header.frame_id = "world"
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(int(snapshot.stamp), 10**9)
    message.info.resolution = float(snapshot.model.resolution)
    message.info.height, message.info.width = map(int, snapshot.model.grid.shape)
    message.info.origin.position.x, message.info.origin.position.y = map(
        float, snapshot.model.origin
    )
    message.info.origin.position.z = float(z)
    message.info.origin.orientation.w = 1.0
    # Preserve assumed footprint as unknown in a standard occupancy map.
    message.data = np.array([-1, 0, 100, -1], dtype=np.int8)[snapshot.model.grid].ravel().tolist()
    return message


def pose_message(header, xy, z, yaw=0.0):
    message = PoseStamped()
    message.header = header
    message.pose.position.x, message.pose.position.y = map(float, xy)
    message.pose.position.z = float(z)
    message.pose.orientation.z = math.sin(yaw / 2)
    message.pose.orientation.w = math.cos(yaw / 2)
    return message


class RosMapPublisher:
    def __init__(self, node, z):
        self.z = z
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_pub = node.create_publisher(OccupancyGrid, "/navigation/map", qos)
        self.path_pub = node.create_publisher(Path, "/navigation/path", qos)
        self.pose_pub = node.create_publisher(PoseStamped, "/navigation/pose", qos)
        self.trail_pub = node.create_publisher(Path, "/navigation/trail", qos)

    def publish(self, snapshot, path, trail=()):
        if snapshot is None or not snapshot.valid:
            return
        occupancy = map_message(snapshot, self.z)
        self.map_pub.publish(occupancy)
        route = Path()
        route.header = occupancy.header
        route.poses = [pose_message(route.header, point, self.z) for point in path]
        self.path_pub.publish(route)
        history = Path()
        history.header = occupancy.header
        history.poses = [pose_message(history.header, point, self.z) for point in trail]
        self.trail_pub.publish(history)
        self.pose_pub.publish(
            pose_message(occupancy.header, snapshot.pose[:2], snapshot.pose[3], snapshot.pose[2])
        )
