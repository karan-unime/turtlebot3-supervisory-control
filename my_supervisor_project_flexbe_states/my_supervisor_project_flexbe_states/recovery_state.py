#!/usr/bin/env python3

import time

from geometry_msgs.msg import Twist

from flexbe_core import EventState, Logger


class RecoveryState(EventState):


    def __init__(self):

        super().__init__(
            outcomes=[
                'recovered',
                'failed'
            ]
        )


        self.pub = None
        self.start = None



    def on_enter(self, userdata):


        Logger.logwarn(
            "Recovery mode activated"
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


        self.start = time.time()



    def execute(self, userdata):


        if time.time()-self.start > 2:


            return 'recovered'


        return None