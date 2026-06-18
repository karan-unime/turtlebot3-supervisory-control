#!/usr/bin/env python

from rclpy.duration import Duration
from flexbe_core import EventState, Logger


class WaitState(EventState):
    """
    Wait for a specified amount of time.

    -- wait_time     float   Time to wait in seconds

    <= done                  Waiting finished
    """

    def __init__(self, wait_time):
        super().__init__(outcomes=['done'])

        self._wait_time = Duration(seconds=wait_time)
        self._start_time = None

    def on_enter(self, userdata):

        self._start_time = self._node.get_clock().now()

        Logger.loginfo(
            f'Waiting for {self._wait_time.nanoseconds / 1e9:.1f} seconds'
        )

    def execute(self, userdata):

        elapsed = (
            self._node.get_clock().now()
            - self._start_time
        )

        if elapsed >= self._wait_time:
            Logger.loginfo('Wait finished.')
            return 'done'

        return None