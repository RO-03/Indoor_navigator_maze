#!/usr/bin/env python3
"""
amcl_initializer.py  —  Bootstrap node for ROS 2 Jazzy + Gazebo Harmonic

Problem:
    AMCL only publishes the map→odom TF AFTER it processes a laser scan with
    a valid initial pose.  Due to a clock-race on node activation and DDS
    subscription delivery timing, AMCL can take many seconds to publish its
    first map→odom TF at start-up.  Nav2's global_costmap checks for the
    map frame immediately on activation and fails if it isn't there.

Solution:
    This node starts BEFORE the navigation stack (at t=28 s, not t=32 s) and:
      1. Publishes map→odom TF at 5 Hz (robot spawn position = -5.18, -6.58).
      2. Publishes /initialpose at 1 Hz so AMCL gets the pose reliably.
      3. Stops publishing /initialpose once /amcl_pose is received (AMCL ack).
      4. Keeps broadcasting map→odom TF until shutdown; AMCL's higher-frequency
         TF updates will simply override this "seed" in every lookup that falls
         after AMCL's first broadcast.

Usage (from launch, fires at t=28 s):
    Node(package='autonomous_tb3', executable='amcl_initializer', ...)
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSDurabilityPolicy,
    QoSReliabilityPolicy, QoSHistoryPolicy,
)
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from tf2_ros import TransformBroadcaster

# ── Robot initial position (must match spawn args in maze_navigation.launch.py) ─
ROBOT_X   = -5.18
ROBOT_Y   = -6.58
ROBOT_YAW =  0.0     # radians

# Send /initialpose only for the first MAX_POSE_TICKS ticks (0.2 s each = 2 s).
# AMCL resets its particle filter on EVERY /initialpose it receives — flooding
# it prevents the filter from ever converging or publishing /amcl_pose.
MAX_POSE_TICKS = 10


class AmclInitializer(Node):
    def __init__(self):
        super().__init__('amcl_initializer')
        # Force sim-time (Gazebo clock)
        self.set_parameters([
            rclpy.parameter.Parameter(
                'use_sim_time',
                rclpy.Parameter.Type.BOOL,
                True,
            )
        ])

        # TRANSIENT_LOCAL so AMCL receives /initialpose even if it isn't yet
        # subscribed at the moment this node first publishes.
        qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', qos)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, qos)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.amcl_acked = False
        self._tick = 0

        # 5 Hz: every other tick we also send /initialpose (1 Hz effective)
        self.timer = self.create_timer(0.2, self._pub)
        self.get_logger().info(
            f'amcl_initializer started — broadcasting map→odom @ '
            f'({ROBOT_X}, {ROBOT_Y}) and publishing /initialpose '
            f'until AMCL acknowledges')

    # ──────────────────────────────────────────────────────────────────────────
    def _pub(self):
        now = self.get_clock().now()
        if now.nanoseconds == 0:
            return  # sim clock not yet available; skip

        # ── 1. Broadcast map → odom TF ────────────────────────────────────────
        t = TransformStamped()
        t.header.stamp    = now.to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id  = 'odom'
        t.transform.translation.x = ROBOT_X
        t.transform.translation.y = ROBOT_Y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(ROBOT_YAW / 2.0)
        t.transform.rotation.w = math.cos(ROBOT_YAW / 2.0)
        self.tf_broadcaster.sendTransform(t)

        # ── 2. Publish /initialpose for first MAX_POSE_TICKS ticks only ─────────
        # Sending /initialpose resets AMCL's particle filter every time.
        # Stop after 2 s so the filter can converge and publish /amcl_pose.
        if self._tick < MAX_POSE_TICKS:
            pose = PoseWithCovarianceStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp    = now.to_msg()
            pose.pose.pose.position.x    = ROBOT_X
            pose.pose.pose.position.y    = ROBOT_Y
            pose.pose.pose.orientation.z = math.sin(ROBOT_YAW / 2.0)
            pose.pose.pose.orientation.w = math.cos(ROBOT_YAW / 2.0)
            pose.pose.covariance[0]  = 0.25   # x
            pose.pose.covariance[7]  = 0.25   # y
            pose.pose.covariance[35] = 0.0685  # yaw
            self.pose_pub.publish(pose)
            self.get_logger().info(
                f'[tick {self._tick}/{MAX_POSE_TICKS}] /initialpose '
                f'@ sim-time {now.nanoseconds / 1e9:.2f} s')
        elif self._tick == MAX_POSE_TICKS:
            self.get_logger().info(
                'Sent /initialpose for 2 s — stopping now so AMCL can converge. '
                'map→odom TF broadcast continues...')

        self._tick += 1

    # ──────────────────────────────────────────────────────────────────────────
    def _amcl_cb(self, _msg):
        if not self.amcl_acked:
            self.get_logger().info(
                'AMCL acknowledged the pose — '
                'stopping /initialpose; map→odom TF will transition to AMCL')
            self.amcl_acked = True


def main(args=None):
    rclpy.init(args=args)
    node = AmclInitializer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
