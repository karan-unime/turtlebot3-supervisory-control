#!/usr/bin/env python3

from flexbe_core import EventState, Logger

# Import mission configuration
from my_supervisor_project_flexbe_states.mission_config  import WAYPOINTS


class WaypointManagerState(EventState):

    """
    Waypoint Mission Manager

    Outcomes:
        waypoint
        completed

    Output Keys:
        target_x
        target_y
    """

    def __init__(self):

        super().__init__(
            outcomes=[
                'waypoint',
                'completed'
            ],
            output_keys=[
                'target_x',
                'target_y'
            ]
        )

        # Current waypoint index
        self._index = 0

        # Load waypoints from configuration
        self._points = WAYPOINTS

    #####################################################

    def on_enter(self, userdata):

        # Reset only when a completely new mission starts
        if self._index >= len(self._points):
            self._index = 0

        Logger.loginfo("Waypoint Manager Started")

    #####################################################

    def execute(self, userdata):

        if self._index >= len(self._points):

            Logger.loginfo("All waypoints completed.")

            return 'completed'

        x, y = self._points[self._index]

        userdata.target_x = x
        userdata.target_y = y

        Logger.loginfo(
            f"Waypoint {self._index + 1}/{len(self._points)}"
            f" -> ({x:.2f}, {y:.2f})"
        )

        self._index += 1

        return 'waypoint'