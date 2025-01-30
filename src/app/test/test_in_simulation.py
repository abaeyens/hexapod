import cmath
import math
import os
import time
import unittest

import numpy as np
import tf_transformations

import rclpy
import geometry_msgs.msg
import nav_msgs.msg
import sensor_msgs.msg
import std_msgs.msg

import launch
import launch_ros
import launch_testing
import launch_testing_ros
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


@launch_testing.parametrize(
        'walk_type',
        (('forward', 'leftward', 'reverse_clockwise')))
def generate_test_description(walk_type):
    return (
        launch.LaunchDescription(
            [
                # Config
                launch_ros.actions.SetParameter(name='use_sim_time', value=True),
                # Nodes under test
                ## Simulator
                IncludeLaunchDescription(
                    XMLLaunchDescriptionSource([os.path.join(
                        get_package_share_directory('simulation'),
                        'launch/simulator.launch.xml')]),
                    launch_arguments={
                        # TODO set to True
                        'run_headless': 'false',
                        'verbosity': '5',
                    }.items(),
                ),
                ## Robot description
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(
                        get_package_share_directory('robot_description'),
                        'robot_description.launch.py'))),
                ## Twist to gait
                launch_ros.actions.Node(
                    package='app',
                    namespace='',
                    executable='follow_velocity_rectangle',
                    name='follow_velocity_rectangle',
                    parameters=[
                        ('use_sim_time', 'true'),
                    ],
                ),
                # Launch tests
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        # context
        {
        },
    )


# Active tests
class TestHexapod(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_hexapod')

    def tearDown(self):
        self.node.destroy_node()

    def test_gazebo_started(self, proc_output):
        """Did Gazebo start properly?"""
        timeout = 10
        proc_output.assertWaitFor(
            'JointPositionController subscribing to Actuator messages',
            timeout=timeout, stream='stdout')
        proc_output.assertWaitFor(
            'OdometryPublisher publishing odometry on',
            timeout=timeout, stream='stdout')

    def test_messages_published(self, proc_output):
        """Does the simulator publish odometry and IMU?"""
        wait_for_topics = launch_testing_ros.WaitForTopics(
            [('odom', nav_msgs.msg.Odometry),
             ('imu', sensor_msgs.msg.Imu),
             ], timeout=10.0)
        assert wait_for_topics.wait()
        wait_for_topics.shutdown()

    def test_walk_around(self, proc_output, walk_type):
        """Does our pet walk around sufficiently quickly?"""
        # Settings
        duration = 10
        goal_velocity_linear = 0.08
        goal_velocity_angular = 0.15
        velocity_publishing_rate = 10
        allowed_relative_error = 0.2
        ## simulation must run at least at 25% of real time
        minimum_allowed_simulation_speed = 0.25

        # Store received poses
        poses = []
        sub = self.node.create_subscription(
            nav_msgs.msg.Odometry, 'odom',
            lambda m: poses.append(m.pose.pose), 10)

        # Publish velocity reference
        pub = self.node.create_publisher(
            geometry_msgs.msg.TwistStamped, 'cmd_vel', 10)
        def publish_cmd_vel():
            msg = geometry_msgs.msg.TwistStamped()
            msg.header = std_msgs.msg.Header()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            match walk_type:
                case 'forward':
                    msg.twist.linear.x = goal_velocity_linear
                case 'leftward':
                    msg.twist.linear.y = goal_velocity_linear
                case 'reverse_clockwise':
                    msg.twist.angular.z = goal_velocity_angular
                case _:
                    raise RuntimeError()
            pub.publish(msg)
        timer = self.node.create_timer(
            1.0/velocity_publishing_rate, publish_cmd_vel)

        try:
            # Spin until desired simulation duration elapsed
            end_time = self.node.get_clock().now() \
                + rclpy.duration.Duration(seconds=duration)
            # ... or too much wall duration elapsed
            wall_end_time = time.time() + duration/minimum_allowed_simulation_speed
            while self.node.get_clock().now() < end_time:
                if time.time() > wall_end_time:
                    raise Exception('Simulation running too slow, aborting.')
                rclpy.spin_once(self.node, timeout_sec=1)
            # Compare walked path of poses with given reference
            assert len(poses) >= 2, 'Must have received at least two poses.'
            match walk_type:
                case 'forward':
                    goal_distance_linear = duration * goal_velocity_linear
                    self.assertAlmostEqual(
                        poses[-1].position.x, goal_distance_linear,
                        delta=goal_distance_linear*allowed_relative_error)
                    self.assertAlmostEqual(
                        poses[-1].position.y, 0.0,
                        delta=goal_distance_linear*allowed_relative_error)
                case 'leftward':
                    goal_distance_linear = duration * goal_velocity_linear
                    self.assertAlmostEqual(
                        poses[-1].position.y, goal_distance_linear,
                        delta=goal_distance_linear*allowed_relative_error)
                    self.assertAlmostEqual(
                        poses[-1].position.x, 0.0,
                        delta=goal_distance_linear*allowed_relative_error)
                case 'reverse_clockwise':
                    yaws = [tf_transformations.euler_from_quaternion(
                        [p.orientation.x, p.orientation.y,
                         p.orientation.z, p.orientation.w])[2]
                    for p in poses]
                    traveled_yaw = np.unwrap(yaws)[-1]
                    goal_distance_angular = duration * goal_velocity_angular
                    self.assertAlmostEqual(
                        traveled_yaw, goal_distance_angular,
                        delta=goal_distance_angular*allowed_relative_error,
                    )
                case _:
                    raise RuntimeError()
        finally:
            # Cleanup, whether tests succeeded or failed
            self.node.destroy_subscription(sub)
            self.node.destroy_publisher(pub)
            self.node.destroy_timer(timer)

# Post-shutdown tests
@launch_testing.post_shutdown_test()
class TestHexapodSimShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        """Check if the processes exited normally."""
        launch_testing.asserts.assertExitCodes(proc_info)
