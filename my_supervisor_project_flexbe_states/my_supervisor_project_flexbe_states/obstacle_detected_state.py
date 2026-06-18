#!/usr/bin/env python

from sensor_msgs.msg import LaserScan
from flexbe_core import EventState, Logger


class ObstacleDetectedState(EventState):
    """
    Check whether an obstacle is present in front of the robot.

    -- threshold    float   Distance threshold in meters.

    <= clear                No obstacle detected.
    <= obstacle             Obstacle detected.
    """

    def __init__(self, threshold=0.4):
        super().__init__(
            outcomes=['clear', 'obstacle']
        )

        self._threshold = threshold
        self._scan = None
        self._scan_sub = None

    def on_start(self):
        """Create LaserScan subscriber."""
        self._scan_sub = self._node.create_subscription(
            LaserScan,
            '/scan',
            self._scan_callback,
            10
        )

        Logger.loginfo(
            f"ObstacleDetectedState initialized "
            f"(threshold = {self._threshold:.2f} m)"
        )

    def _scan_callback(self, msg):
        """Store latest scan message."""
        self._scan = msg

    def execute(self, userdata):
        """Check front obstacle distance."""

        if self._scan is None:
            return None

        ranges = self._scan.ranges

        if len(ranges) == 0:
            return None

        window = list(ranges[:15]) + list(ranges[-15:])

        valid_ranges = [
            r for r in window
            if r != float('inf')
        ]

        if len(valid_ranges) == 0:
            front_distance = float('inf')
        else:
            front_distance = min(valid_ranges)

        Logger.loginfo(
            f"Front distance: {front_distance:.2f} m"
        )

        if front_distance < self._threshold:
            Logger.logwarn(
                f"Obstacle detected at "
                f"{front_distance:.2f} m"
            )
            return 'obstacle'

        return 'clear'