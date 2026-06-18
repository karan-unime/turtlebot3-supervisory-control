#!/usr/bin/env python

import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from flexbe_core import EventState, Logger


class MoveDistanceState(EventState):
    """
    Move the robot forward for a given distance using odometry.

    -- distance      float   Distance to travel (meters)
    -- speed         float   Forward speed (m/s)

    <= done                  Target distance reached
    """

    def __init__(self, distance, speed):
        super().__init__(outcomes=['done'])

        self._distance = distance
        self._speed = speed

        self._cmd_pub = None
        self._odom_sub = None

        self._x = 0.0
        self._y = 0.0

        self._x0 = None
        self._y0 = None

    def odom_callback(self, msg):
        """Continuously update robot position."""
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y

    def on_start(self):
        """Create publisher and subscriber once."""

        self._cmd_pub = self._node.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self._odom_sub = self._node.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        Logger.loginfo('MoveDistanceState initialized.')

    def on_enter(self, userdata):
        """Save starting position."""

        self._x0 = self._x
        self._y0 = self._y

        Logger.loginfo(
            f'Start position: '
            f'x={self._x0:.2f}, y={self._y0:.2f}'
        )

    def execute(self, userdata):
        """Move until target distance is reached."""

        dx = self._x - self._x0
        dy = self._y - self._y0

        travelled = math.sqrt(dx ** 2 + dy ** 2)

        Logger.loginfo(
            f'Travelled: {travelled:.2f} / '
            f'{self._distance:.2f} m'
        )

        if travelled >= self._distance:
            Logger.loginfo('Target distance reached.')
            return 'done'

        msg = Twist()
        msg.linear.x = self._speed
        msg.angular.z = 0.0

        self._cmd_pub.publish(msg)

        return None

    def on_exit(self, userdata):
        """Stop robot when leaving state."""

        stop_msg = Twist()
        self._cmd_pub.publish(stop_msg)

        Logger.loginfo('Robot stopped.')
