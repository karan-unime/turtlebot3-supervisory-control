#!/usr/bin/env python3

import time

from flexbe_core import EventState, Logger


class ChargingState(EventState):


    def __init__(self):

        super().__init__(
            outcomes=[
                'charged',
                'failed'
            ]
        )


        self.start = None



    def on_enter(self, userdata):


        Logger.loginfo(
            "Moving to charging station"
        )


        self.start = time.time()



    def execute(self, userdata):


        if time.time()-self.start > 5:


            Logger.loginfo(
                "Charging completed"
            )


            return 'charged'


        return None