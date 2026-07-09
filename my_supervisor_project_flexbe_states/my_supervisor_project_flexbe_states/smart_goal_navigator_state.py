#!/usr/bin/env python3

import math

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from flexbe_core import EventState, Logger


class SmartGoalNavigatorState(EventState):

    """
    Autonomous Goal Navigation

    <= reached
    <= failed
    """


    def __init__(self):

        super().__init__(
            outcomes=[
                'reached',
                'failed'
            ]
        )


        self.scan = None
        self.odom = None


        self.goal_x = 2.0
        self.goal_y = 2.0


        self.cmd_pub = None
        self.scan_sub = None
        self.odom_sub = None


        self.safe_distance = 0.7



    def on_enter(self, userdata):


        node = self._node


        if self.cmd_pub is None:

            self.cmd_pub = node.create_publisher(
                Twist,
                "/cmd_vel",
                10
            )


        if self.scan_sub is None:

            self.scan_sub = node.create_subscription(
                LaserScan,
                "/scan",
                self.scan_cb,
                10
            )


        if self.odom_sub is None:

            self.odom_sub = node.create_subscription(
                Odometry,
                "/odom",
                self.odom_cb,
                10
            )


        Logger.loginfo(
            "Smart Goal Navigator Started"
        )



    def scan_cb(self,msg):

        self.scan = msg



    def odom_cb(self,msg):

        self.odom = msg



    def stop(self):

        t = Twist()

        self.cmd_pub.publish(t)



    def execute(self,userdata):


        if self.scan is None or self.odom is None:

            Logger.loginfo(
                "Waiting sensors..."
            )

            return None



        # ============================
        # current robot position
        # ============================

        x = self.odom.pose.pose.position.x
        y = self.odom.pose.pose.position.y


        dx = self.goal_x - x
        dy = self.goal_y - y


        distance_goal = math.sqrt(
            dx*dx + dy*dy
        )


        if distance_goal < 0.2:

            self.stop()

            Logger.loginfo(
                "Goal reached"
            )

            return "reached"



        goal_angle = math.atan2(
            dy,
            dx
        )


        # ============================
        # LIDAR front check
        # ============================

        front = min(
            min(self.scan.ranges[0:20]),
            min(self.scan.ranges[-20:])
        )


        cmd = Twist()



        if front < self.safe_distance:


            # obstacle
            # rotate until free


            Logger.loginfo(
                "Obstacle detected -> searching path"
            )


            cmd.angular.z = 0.5
            cmd.linear.x = 0.0



        else:


            # move toward goal


            Logger.loginfo(
                f"Moving to goal distance {distance_goal:.2f}"
            )


            cmd.linear.x = 0.15


            cmd.angular.z = goal_angle * 0.3



        self.cmd_pub.publish(
            cmd
        )


        return None
