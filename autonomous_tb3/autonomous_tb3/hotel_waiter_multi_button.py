#!/usr/bin/env python3
'''
Hotel Waiter for ROS 2 Jazzy + Gazebo Harmonic.

Uses a dedicated MultiThreadedExecutor node to publish the initial pose
so that the sim clock can advance between publishes, resolving the
Jazzy TF extrapolation deadlock in BasicNavigator.setInitialPose().
'''
import time
import threading
import subprocess
import os

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
import tkinter as tk
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from ament_index_python.packages import get_package_share_directory


class InitialPosePub(Node):
    """Lightweight node: publishes /initialpose on a timer until AMCL acks."""

    def __init__(self, x: float, y: float):
        super().__init__('hotel_pose_helper')
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
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685
        self.get_logger().info(f'[hotel_waiter] Publishing initial pose ({self._x:.2f}, {self._y:.2f})')
        self.pub.publish(msg)

    def _cb(self, msg):
        self.get_logger().info('[hotel_waiter] AMCL acknowledged the initial pose!')
        self.received = True


def publish_initial_pose(x: float, y: float, timeout: float = 60.0) -> bool:
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


def _make_pose(navigator, x: float, y: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.w = 1.0
    return pose


class NavigatorApp:
    def __init__(self):
        print('[hotel_waiter] Publishing initial pose...')
        success = publish_initial_pose(x=-4.36, y=0.51, timeout=60.0)
        if not success:
            raise RuntimeError('AMCL did not acknowledge initial pose within 60s.')

        print('[hotel_waiter] Initializing BasicNavigator...')
        self.navigator = BasicNavigator()
        self.navigator.initial_pose_received = True
        self.navigator.waitUntilNav2Active(localizer='robot_localization')
        print('[hotel_waiter] Nav2 is active!')

        self.beer_path = os.path.join(
            get_package_share_directory('autonomous_tb3'),
            'models', 'beer', 'model.sdf'
        )
        self.waiter_active_flag = False
        self.return_from_table = False

        # Drive to starting counter position
        self._go(-4.37, -3.38)
        self.waiter_active_flag = True

        # Build GUI
        self.tk_button = tk.Tk()
        self.tk_button.title('Hotel Waiter Control')
        self._create_button('Table 1', -0.6,  2.44)
        self._create_button('Table 2',  4.29, 2.64)
        self._create_button('Table 3',  4.33, -1.74)
        self._create_button('Table 4', -0.6, -1.99)

    def _create_button(self, text, x, y):
        tk.Button(
            self.tk_button, text=text,
            command=lambda: self.go_to_pose(x, y)
        ).pack(pady=4)

    def _go(self, x, y):
        pose = _make_pose(self.navigator, x, y)
        self.navigator.goToPose(pose)
        while not self.navigator.isTaskComplete():
            pass
        result = self.navigator.getResult()
        print(f'[hotel_waiter] Navigation result: {result}')

    def go_to_pose(self, x, y):
        table_serving = _make_pose(self.navigator, x, y)
        counter_return = _make_pose(self.navigator, -4.37, -3.38)

        if self.waiter_active_flag:
            print('[hotel_waiter] Heading to table...')
            cmd = ['ros2', 'run', 'autonomous_tb3', 'sdf_spawner',
                   self.beer_path, 'beer', '-4.46', '-3.46']
            subprocess.Popen(cmd).wait()

            self.navigator.followWaypoints([table_serving, counter_return])
            self.return_from_table = True
        else:
            print('[hotel_waiter] Heading to counter...')
            self.navigator.goToPose(counter_return)

        while not self.navigator.isTaskComplete():
            pass

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('[hotel_waiter] Goal succeeded!')
        else:
            print('[hotel_waiter] Goal failed.')

        if self.return_from_table:
            print('[hotel_waiter] Removing beer model...')
            subprocess.Popen([
                'gz', 'service',
                '-s', '/world/default/remove',
                '--reqtype', 'gz.msgs.Entity',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '2000',
                '--req', 'name: "beer" type: MODEL'
            ]).wait()
            self.return_from_table = False

    def run(self):
        self.tk_button.mainloop()
        self.navigator.lifecycleShutdown()


def start_app():
    rclpy.init()
    app = NavigatorApp()
    app.run()
    rclpy.shutdown()