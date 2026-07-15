#!/usr/bin/env python3

from flexbe_core import EventState, Logger


class InitialState(EventState):

    """
    Robot initialization check

    <= ready
    <= failed
    """

    def __init__(self):

        super().__init__(
            outcomes=[
                'ready',
                'failed'
            ]
        )


    def on_enter(self, userdata):

        Logger.loginfo(
            "Initializing robot systems..."
        )


    def execute(self, userdata):

        # later:
        # check scan
        # check odom
        # check battery

        Logger.loginfo(
            "System check completed"
        )

        return 'ready'