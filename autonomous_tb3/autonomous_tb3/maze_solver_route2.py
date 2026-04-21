#!/usr/bin/env python3
"""
maze_solver_route2.py — Route 2 (upper-right quadrant)

Start  : (-5.18, -6.58)  ← robot spawn position
Goal 1 : (-2.50, -5.00)  ← mid-corridor, heading right
Goal 2 : ( 0.00, -2.50)  ← upper-right section of maze

Run AFTER the launch is up and RViz shows Navigation: active:
  ros2 run autonomous_tb3 maze_solver_route2
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# ── Spawn / initial-pose coordinates (must match launch + amcl_initializer) ─
SPAWN_X = -5.18
SPAWN_Y = -6.58

# ── Route 2 waypoints ────────────────────────────────────────────────────────
# working WAYPOINTS = [
#     (-1.29, -1.39, 0.0),   # (x, y, yaw_rad) — mid-corridor
#     ( -3.42, -1.06, 0.0),   # upper-right section
# ]

# WAYPOINTS = [
#     (-1.46, 1.7, 0.0),
#     ( -3.22, 1.42, 0.0),
#     ( -3.66, 1.57, 0.0),
#     ( -5.49, 1.98, 0.0),
#     ( -6.98, -1.02, 0.0),

# ]

WAYPOINTS = [
    (-5.83, -5.17, 0.0),   # (x, y, yaw_rad) — mid-corridor
    ( -7.92, -5.23, 0.0),   # upper-right section
    ( -7.87, -3.75, 0.0),
    ( -6.19, -3.69, 0.0),  
]


class InitialPosePub(Node):
    """Publishes /initialpose at 0.5 Hz until AMCL acknowledges."""

    def __init__(self, x: float, y: float):
        super().__init__('initial_pose_helper')
        self.set_parameters([
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
        ])
        qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', qos)
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._cb, qos)
        self.received = False
        self._x = x
        self._y = y
        self.timer = self.create_timer(0.5, self._publish)

    def _publish(self):
        if self.received:
            self.timer.cancel()
            return
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = self._x
        msg.pose.pose.position.y = self._y
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance[0]  = 0.25
        msg.pose.covariance[7]  = 0.25
        msg.pose.covariance[35] = 0.0685
        self.get_logger().info(
            f'[Route 2] Publishing initial pose ({self._x}, {self._y}) '
            f'sim-time {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}')
        self.pub.publish(msg)

    def _cb(self, _msg):
        self.get_logger().info('[Route 2] AMCL acknowledged pose!')
        self.received = True


def publish_initial_pose(x: float, y: float, timeout: float = 15.0) -> bool:
    node = InitialPosePub(x, y)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    start = time.time()
    while not node.received and (time.time() - start) < timeout:
        executor.spin_once(timeout_sec=0.1)
    success = node.received
    executor.remove_node(node)
    node.destroy_node()
    return success


def make_pose(navigator: BasicNavigator, x: float, y: float, yaw: float) -> PoseStamped:
    import math
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def main():
    rclpy.init()

    print('[Route 2] Publishing initial pose at spawn position...')
    ok = publish_initial_pose(SPAWN_X, SPAWN_Y, timeout=15.0)
    if not ok:
        print('[Route 2] WARN: AMCL ack timed out — amcl_initializer seeds TF, continuing.')
    else:
        print('[Route 2] AMCL acknowledged.')

    print('[Route 2] Starting BasicNavigator...')
    navigator = BasicNavigator()
    navigator.initial_pose_received = True
    navigator.waitUntilNav2Active(localizer='robot_localization')

    print('[Route 2] Nav2 active! Building waypoints...')
    goals = [make_pose(navigator, x, y, yaw) for x, y, yaw in WAYPOINTS]

    print(f'[Route 2] Navigating through {len(goals)} goal(s):')
    for i, (x, y, _) in enumerate(WAYPOINTS):
        print(f'  Goal {i+1}: ({x:.2f}, {y:.2f})')

    navigator.goThroughPoses(goals)

    while not navigator.isTaskComplete():
        pass

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('[Route 2] ✅ Goal SUCCEEDED — robot reached destination!')
    elif result == TaskResult.CANCELED:
        print('[Route 2] ⚠️  Goal was CANCELED.')
    elif result == TaskResult.FAILED:
        print('[Route 2] ❌ Goal FAILED — try adjusting waypoint coordinates.')
    else:
        print(f'[Route 2] Unknown result: {result}')

    navigator.lifecycleShutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
