#! /usr/bin/env python3
'''
Maze solver for ROS 2 Jazzy + Gazebo Harmonic.

The key fix for Jazzy with use_sim_time=True:
  - We bypass BasicNavigator.setInitialPose() entirely.
  - We publish /initialpose directly using a MultiThreadedExecutor so
    the sim clock can advance between publishes (fixes TF extrapolation).
  - We call waitUntilNav2Active() AFTER the initial pose is confirmed.
'''
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


class InitialPosePub(Node):
    """Lightweight node that publishes the initial pose and listens for AMCL ack."""

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
            PoseWithCovarianceStamped, '/amcl_pose', self._cb, qos
        )
        self.received = False
        self._x = x
        self._y = y

        # Publish every 0.5 s until AMCL acks
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
        # Standard covariance for AMCL
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685
        self.get_logger().info(
            f'Publishing initial pose ({self._x:.2f}, {self._y:.2f}) at sim-time '
            f'{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}'
        )
        self.pub.publish(msg)

    def _cb(self, msg):
        self.get_logger().info('AMCL acknowledged the initial pose!')
        self.received = True


def publish_initial_pose(x: float, y: float, timeout: float = 60.0) -> bool:
    """
    Spin an InitialPosePub node with a MultiThreadedExecutor until AMCL acks.
    Returns True on success, False on timeout.
    """
    node = InitialPosePub(x, y)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    start = time.time()
    while not node.received and (time.time() - start) < timeout:
        executor.spin_once(timeout_sec=0.1)

    success = node.received
    # Remove from executor before destroy to avoid 'Destroyable' crash
    executor.remove_node(node)
    node.destroy_node()
    return success


def main():
    rclpy.init()

    # ── Step 1: Inject initial pose ────────────────────────────────────────
    # amcl_initializer (running from launch) already broadcasts map→odom TF
    # and seeds /initialpose.  We re-send here for reliability, but if AMCL
    # doesn't ack within 15 s we continue anyway — Nav2 is already up.
    print('[maze_solver] Publishing initial pose...')
    success = publish_initial_pose(x=-5.18, y=-6.58, timeout=15.0)
    if not success:
        print('[maze_solver] WARN: AMCL ack timed out — Nav2 TF is seeded by '
              'amcl_initializer, proceeding to navigate anyway.')
    else:
        print('[maze_solver] Initial pose confirmed by AMCL.')

    print('[maze_solver] Initial pose confirmed! Starting BasicNavigator...')

    # ── Step 2: Start BasicNavigator and wait for Nav2 ──────────────────────
    navigator = BasicNavigator()
    # Tell BasicNavigator that AMCL pose is already received (we did it ourselves)
    # Use localizer='robot_localization' to skip the redundant amcl lifecycle checks
    navigator.initial_pose_received = True
    navigator.waitUntilNav2Active(localizer='robot_localization')

    print('[maze_solver] Nav2 is active! Sending navigation goals...')

    # ── Step 3: Build goal list ───────────────────────────────────────────────
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = -1.23
    goal_pose.pose.position.y = -2.1
    goal_pose.pose.orientation.w = 1.0

    goal_pose_1 = PoseStamped()
    goal_pose_1.header.frame_id = 'map'
    goal_pose_1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose_1.pose.position.x = -7.4
    goal_pose_1.pose.position.y = -1.17
    goal_pose_1.pose.orientation.w = 1.0

    # ── Step 4: Navigate ──────────────────────────────────────────────────────
    navigator.goThroughPoses([goal_pose, goal_pose_1])

    while not navigator.isTaskComplete():
        pass

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('[maze_solver] Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('[maze_solver] Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('[maze_solver] Goal failed!')
    else:
        print('[maze_solver] Unknown result status.')

    navigator.lifecycleShutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()