#!/usr/bin/env python3
"""
maze_solver_route3.py — Route 3 (middle of maze → very top passage)

Start  : (-5.18, -6.58)  ← robot spawn position
Goal 1 : ( 0.804,  0.577) ← middle of maze  (picked with RViz Publish Point)
Goal 2 : ( 4.660, -1.410) ← top/right passage (picked with RViz Publish Point)

Run AFTER the launch is up and RViz shows Navigation: active:
  ros2 run autonomous_tb3 maze_solver_route3
"""

import math
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

# ── Route 3 waypoints  (x, y, yaw_radians) ──────────────────────────────────
#   yaw: 0.0=east  1.57=north  3.14=west  -1.57=south
#   Coordinates obtained from RViz → Publish Point tool (z ignored for 2-D nav)
WAYPOINTS = [
    (-1.94,  2.54,  0.0),  # p1
    ( 1.61,  3.02,  0.0),  # p2
    ( 2.15,  2.31,  0.0),
    ( 2.24,  1.62,  0.0),  # p3
    ( 0.121,-0.179, 0.0),  # p4
    ( 0.27, -4.09,  0.0),  # p5
    ( 4.49, -3.52,  0.0),  # p6
    ( 4.48, -1.49,  0.0),  # p7
]




class InitialPosePub(Node):
    """Publishes /initialpose until AMCL acknowledges (or timeout)."""

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
            f'[Route 3] Publishing initial pose ({self._x:.2f}, {self._y:.2f}) '
            f'sim-time {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}')
        self.pub.publish(msg)

    def _cb(self, _msg):
        self.get_logger().info('[Route 3] AMCL acknowledged pose!')
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

    print('[Route 3] Publishing initial pose at spawn position...')
    ok = publish_initial_pose(SPAWN_X, SPAWN_Y, timeout=15.0)
    if not ok:
        print('[Route 3] WARN: AMCL ack timed out — amcl_initializer seeds TF, continuing.')
    else:
        print('[Route 3] AMCL acknowledged.')

    print('[Route 3] Starting BasicNavigator...')
    navigator = BasicNavigator()
    navigator.initial_pose_received = True
    navigator.waitUntilNav2Active(localizer='robot_localization')

    print('[Route 3] Nav2 active! Building 3-waypoint route...')
    goals = [make_pose(navigator, x, y, yaw) for x, y, yaw in WAYPOINTS]

    print(f'[Route 3] Navigating through {len(goals)} goal(s):')
    for i, (x, y, yaw) in enumerate(WAYPOINTS):
        print(f'  Goal {i+1}: ({x:.2f}, {y:.2f})  yaw={math.degrees(yaw):.0f}°')

    navigator.goThroughPoses(goals)

    while not navigator.isTaskComplete():
        pass

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('[Route 3] ✅ Goal SUCCEEDED — all 3 waypoints reached!')
    elif result == TaskResult.CANCELED:
        print('[Route 3] ⚠️  Goal was CANCELED.')
    elif result == TaskResult.FAILED:
        print('[Route 3] ❌ Goal FAILED — try adjusting waypoint coordinates.')
    else:
        print(f'[Route 3] Unknown result: {result}')

    navigator.lifecycleShutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
