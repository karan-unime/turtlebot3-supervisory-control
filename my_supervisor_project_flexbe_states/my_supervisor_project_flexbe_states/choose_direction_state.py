#!/usr/bin/env python3

import math

from sensor_msgs.msg import LaserScan
from flexbe_core import EventState, Logger


class ChooseDirectionState(EventState):
    """
    Choose avoidance direction using LiDAR.

    <= left
    <= right
    """

    def __init__(self):
        super().__init__(
            outcomes=['left', 'right']
        )

        self._scan = None
        self._scan_sub = None

    def on_start(self):
        self._scan_sub = self._node.create_subscription(
            LaserScan,
            '/scan',
            self._scan_callback,
            10
        )

        Logger.loginfo(
            'ChooseDirectionState initialized.'
        )

    def _scan_callback(self, msg):
        self._scan = msg

    def execute(self, userdata):

        if self._scan is None:
            return None

        ranges = list(self._scan.ranges)

        # Left sector (45° to 135°)
        left_sector = ranges[45:135]

        # Right sector (225° to 315°)
        right_sector = ranges[225:315]

        # Remove invalid values
        left_sector = [
            x for x in left_sector
            if not math.isinf(x) and not math.isnan(x)
        ]

        right_sector = [
            x for x in right_sector
            if not math.isinf(x) and not math.isnan(x)
        ]

        # Compute average free space
        if left_sector:
            left_distance = (
                sum(left_sector) /
                len(left_sector)
            )
        else:
            left_distance = self._scan.range_max

        if right_sector:
            right_distance = (
                sum(right_sector) /
                len(right_sector)
            )
        else:
            right_distance = self._scan.range_max

        Logger.loginfo(
            f'Left: {left_distance:.2f} m, '
            f'Right: {right_distance:.2f} m'
        )

        # Choose side with more free space
        if left_distance >= right_distance:
            Logger.loginfo('Choosing LEFT')
            return 'left'

        Logger.loginfo('Choosing RIGHT')
        return 'right'