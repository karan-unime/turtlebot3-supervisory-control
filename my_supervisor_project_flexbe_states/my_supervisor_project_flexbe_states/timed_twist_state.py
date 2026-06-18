#!/usr/bin/env python
"""TimedTwistState: drive the robot with a fixed Twist command for a set duration."""

from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from flexbe_core import EventState, Logger


class TimedTwistState(EventState):
    """
    Publish a fixed Twist command for a given duration, then return done.

    -- linear_x     float   Forward speed in m/s
    -- angular_z    float   Turning rate in rad/s
    -- duration     float   How long to publish, in seconds

    <= done         Finished publishing for the given duration
    """

    def __init__(self, linear_x, angular_z, duration):
        super().__init__(outcomes=['done'])

        self._linear_x = linear_x
        self._angular_z = angular_z
        self._target_duration = Duration(seconds=duration)

        # IMPORTANT:
        # Do NOT use self._pub because FlexBE uses it internally.
        self._cmd_pub = None

        self._enter_time = None
        self._return = None

    def on_start(self):
        """Called once when the behavior starts."""
        self._cmd_pub = self._node.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

    def on_enter(self, userdata):
        """Called every time this state becomes active."""
        self._enter_time = self._node.get_clock().now()
        self._return = None

        Logger.loginfo(
            f"TimedTwistState: lin={self._linear_x} "
            f"ang={self._angular_z} "
            f"for {self._target_duration.nanoseconds / 1e9:.1f}s"
        )

    def execute(self, userdata):
        """Runs repeatedly while the state is active."""

        if self._return is not None:
            return self._return

        msg = Twist()
        msg.linear.x = self._linear_x
        msg.angular.z = self._angular_z

        Logger.loginfo(
            f"Publishing: lin={self._linear_x}, ang={self._angular_z}"
        )

        self._cmd_pub.publish(msg)

        elapsed = self._node.get_clock().now() - self._enter_time

        if elapsed >= self._target_duration:
            self._return = 'done'
            return 'done'

        return None

    def on_exit(self, userdata):
        """Stop the robot when leaving the state."""
        self._cmd_pub.publish(Twist())
