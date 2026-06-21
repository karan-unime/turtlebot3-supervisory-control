#!/usr/bin/env python3

import math

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from flexbe_core import EventState, Logger


class MoveUntilClearState(EventState):
    """
    Move forward until the front path is clear.

    -- speed       float    Forward speed (m/s)
    -- threshold   float    Distance considered clear (m)

    <= done
    """

    def __init__(self,
                 speed=0.15,
                 threshold=0.8):
        super().__init__(
            outcomes=['done']
        )

        self._speed = speed
        self._threshold = threshold

        self._scan = None
        self._scan_sub = None
        self._cmd_pub = None

    def on_start(self):

        self._cmd_pub = self._node.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self._scan_sub = self._node.create_subscription(
            LaserScan,
            '/scan',
            self._scan_callback,
            10
        )

        Logger.loginfo(
            f'MoveUntilClearState initialized '
            f'(threshold={self._threshold:.2f} m)'
        )

    def _scan_callback(self, msg):
        self._scan = msg

    def execute(self, userdata):

        if self._scan is None:
            return None

        ranges = list(self._scan.ranges)

        n = len(ranges)

        # Front sector
        front_sector = (
            ranges[0:15] +
            ranges[n-15:n]
        )

        front_sector = [
            x for x in front_sector
            if not math.isinf(x)
            and not math.isnan(x)
        ]

        if front_sector:
            front_distance = min(front_sector)
        else:
            front_distance = self._scan.range_max

        Logger.loginfo(
            f'Front distance: '
            f'{front_distance:.2f} m'
        )

        # Path is clear
        if front_distance > self._threshold:

            self._cmd_pub.publish(
                Twist()
            )

            Logger.loginfo(
                'Path is clear.'
            )

            return 'done'

        # Keep moving
        msg = Twist()
        msg.linear.x = self._speed

        self._cmd_pub.publish(msg)

        return None

    def on_exit(self, userdata):

        if self._cmd_pub is not None:
            self._cmd_pub.publish(
                Twist()
            )

        Logger.loginfo(
            'Robot stopped.'
        )