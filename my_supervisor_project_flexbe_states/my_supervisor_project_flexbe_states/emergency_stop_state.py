#!/usr/bin/env python3

from geometry_msgs.msg import Twist

from flexbe_core import EventState, Logger


class EmergencyStopState(EventState):


    def __init__(self):

        super().__init__(
            outcomes=[
                'stopped'
            ]
        )


        self.pub=None



    def on_enter(
        self,
        userdata
    ):


        Logger.logwarn(
            "EMERGENCY STOP"
        )


        if self.pub is None:


            self.pub = self._node.create_publisher(
                Twist,
                '/cmd_vel',
                10
            )


        self.pub.publish(
            Twist()
        )



    def execute(
        self,
        userdata
    ):

        return 'stopped'