#!/usr/bin/env python3
"""Broadcast odom→base_footprint TF from /odom nav_msgs/Odometry.

Workaround for ros_gz_bridge Pose_V→TFMessage producing empty frame IDs
(gazebosim/ros_gz#172, #410). Subscribes to the already-bridged /odom topic
and re-publishes the pose as a proper TF broadcast.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class OdomTfBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_tf_broadcaster')
        self.br = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 50)

    def odom_cb(self, msg: Odometry):
        t = TransformStamped()
        t.header = msg.header                    # frame_id = "odom"
        t.child_frame_id = msg.child_frame_id    # "base_footprint"
        # Fallbacks if Gz bridge leaves them empty
        if not t.header.frame_id:
            t.header.frame_id = 'odom'
        if not t.child_frame_id:
            t.child_frame_id = 'base_footprint'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t)


def main():
    rclpy.init()
    rclpy.spin(OdomTfBroadcaster())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
