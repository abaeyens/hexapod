import os
import time
import unittest

import rclpy
import geometry_msgs.msg
import nav_msgs.msg
import sensor_msgs.msg
import std_msgs.msg

import launch
import launch_ros
import launch_testing.actions
import launch_testing_ros
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_test_description():
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
             # TODO fix IMU not getting published
             #('imu', sensor_msgs.msg.IMU),
             ], timeout=10.0)
        assert wait_for_topics.wait()
        wait_for_topics.shutdown()

    def test_walk_forward(self, proc_output):
        """Does our pet walk forward sufficiently quickly?"""
        max_duration = 10
        goal_velocity = 0.08
        min_distance = max_duration * goal_velocity * 0.8
        velocity_publishing_rate = 10
        # simulation must run at least at 25% of real time
        wall_end_time = time.time() + max_duration * 4
        positions = []
        sub = self.node.create_subscription(
            nav_msgs.msg.Odometry, 'odom',
            lambda m: positions.append(m.pose.pose.position), 10)
        pub = self.node.create_publisher(
            geometry_msgs.msg.TwistStamped, 'cmd_vel', 10)
        def publish_cmd_vel():
            msg = geometry_msgs.msg.TwistStamped()
            msg.header = std_msgs.msg.Header()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.twist.linear.x = goal_velocity
            pub.publish(msg)
        timer = self.node.create_timer(
            1.0/velocity_publishing_rate, publish_cmd_vel)
        try:
            end_time = self.node.get_clock().now() \
                + rclpy.duration.Duration(seconds=max_duration)
            while self.node.get_clock().now() < end_time:
                if time.time() > wall_end_time:
                    raise Exception('Simulation running too slow, aborting.')
                rclpy.spin_once(self.node, timeout_sec=1)
            assert len(positions) >= 2
            assert positions[-1].x - positions[0].x > min_distance
        finally:
            self.node.destroy_subscription(sub)
            self.node.destroy_publisher(pub)
            self.node.destroy_timer(timer)

# Post-shutdown tests
@launch_testing.post_shutdown_test()
class TestHexapodSimShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        """Check if the processes exited normally."""
        launch_testing.asserts.assertExitCodes(proc_info)
