import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry

import message_filters

import numpy as np


class RecorderNode(Node):
    def __init__(self):
        super().__init__("my_node")
        self.get_logger().info("Initializing my_node")

        # Create the subscribers and synchronize the messages
        self.odom_sub = message_filters.Subscriber(self, Odometry, "/diff_drive_controller/odom")
        self.filter_sub = message_filters.Subscriber(self, Odometry, "/odometry/filtered")
        self.ground_truth_sub = message_filters.Subscriber(self, Odometry, "/ground_truth")
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.odom_sub, self.filter_sub, self.ground_truth_sub], 10, 0.01
        )
        self.ts.registerCallback(self.store_data)

        self.odom = []
        self.filter = []
        self.ground_truth = []

    def store_data(self, odom, filter, ground_truth):
        self.get_logger().info(
            f"Storing data... Current lenght {len(self.odom)}", throttle_duration_sec=5.0
        )
        self.odom.append([])  # Insert the data that you want to save in the lists
        self.filter.append([])  # Insert the data that you want to save in the lists
        self.ground_truth.append([])  # Insert the data that you want to save in the lists

    def save_data(self):
        np.save("odom.npy", np.array(self.odom))
        np.save("filter.npy", np.array(self.filter))
        np.save("ground_truth.npy", np.array(self.ground_truth))


def main(args=None):
    rclpy.init(args=args)
    node = RecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_data()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
