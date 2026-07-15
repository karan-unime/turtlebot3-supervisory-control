#!/usr/bin/env python3

from rclpy.duration import Duration

from flexbe_core import EventState, Logger
from my_supervisor_project_flexbe_states.mission_config import WAIT_TIME


class WaitState(EventState):
    """
    Wait State

    Reads the waiting time from mission_config.py

    <= done
    """

    def __init__(self):

        super().__init__(
            outcomes=['done']
        )

        self._wait_time = Duration(
            seconds=WAIT_TIME
        )

        self._start_time = None


    ########################################################

    def on_enter(self, userdata):

        self._start_time = self._node.get_clock().now()

        Logger.loginfo(
            f"Waiting {WAIT_TIME:.1f} second(s)"
        )


    ########################################################

    def execute(self, userdata):

        elapsed = (
            self._node.get_clock().now()
            - self._start_time
        )

        if elapsed >= self._wait_time:

            Logger.loginfo("Wait finished.")

            return 'done'

        return None