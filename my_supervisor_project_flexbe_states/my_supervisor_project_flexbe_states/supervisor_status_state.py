#!/usr/bin/env python3

from flexbe_core import EventState, Logger


class SupervisorStatusState(EventState):

    """
    HFSM Supervisor decision state

    <= navigate
    <= recovery
    <= mission_complete
    """


    def __init__(self):

        super().__init__(

            outcomes=[
                'navigate',
                'recovery',
                'mission_complete'
            ]
        )


        self._mission_finished = False
        self._failure_counter = 0



    def on_enter(self, userdata):

        Logger.loginfo(
            "HFSM Supervisor active"
        )



    def execute(self, userdata):


        # Future:
        # check battery
        # check failures
        # check mission status
        # LTL events enter here


        if self._mission_finished:


            Logger.loginfo(
                "Mission completed"
            )


            return 'mission_complete'



        if self._failure_counter >= 3:


            Logger.logwarn(
                "Supervisor switching to recovery"
            )


            return 'recovery'



        Logger.loginfo(
            "Supervisor selected Navigation Mode"
        )


        return 'navigate'