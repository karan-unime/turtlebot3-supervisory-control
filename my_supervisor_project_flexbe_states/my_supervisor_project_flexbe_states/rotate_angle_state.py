#!/usr/bin/env python3

import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

from flexbe_core import EventState, Logger


class RotateAngleState(EventState):
    """
    Rotate robot by a desired angle.

    -- angle_deg       float    Rotation angle in degrees
    -- angular_speed   float    Maximum angular speed (rad/s)

    <= done                     Rotation completed
    """

    def __init__(self, angle_deg, angular_speed=0.3):
        super().__init__(outcomes=['done'])

        self._target_angle = math.radians(abs(angle_deg))
        self._direction = 1.0 if angle_deg >= 0 else -1.0
        self._angular_speed = angular_speed

        self._cmd_pub = None
        self._odom_sub = None
        self._odom = None

        self._start_yaw = None
        self._return = None

    def on_start(self):
        self._cmd_pub = self._node.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self._odom_sub = self._node.create_subscription(
            Odometry,
            '/odom',
            self._odom_callback,
            10
        )

        Logger.loginfo('RotateAngleState initialized.')

    def _odom_callback(self, msg):
        self._odom = msg

    def _get_yaw(self):
        if self._odom is None:
            return None

        q = self._odom.pose.pose.orientation

        quaternion = [
            q.x,
            q.y,
            q.z,
            q.w
        ]

        (_, _, yaw) = euler_from_quaternion(quaternion)

        return yaw

    def _normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    def on_enter(self, userdata):
        self._return = None

        self._start_yaw = self._get_yaw()

        if self._start_yaw is None:
            Logger.logwarn('Waiting for odometry...')
            return

        Logger.loginfo(
            f'Start yaw: {math.degrees(self._start_yaw):.1f} deg'
        )

    def execute(self, userdata):
        if self._return is not None:
            return self._return

        current_yaw = self._get_yaw()

        if current_yaw is None:
            return None

        diff = self._normalize_angle(
            current_yaw - self._start_yaw
        )

        rotated = abs(diff)

        Logger.loginfo(
            f'Rotated: {math.degrees(rotated):.1f} / '
            f'{math.degrees(self._target_angle):.1f} deg'
        )

        if rotated >= self._target_angle:
            Logger.loginfo('Target angle reached.')
            self._return = 'done'
            return 'done'

        remaining = self._target_angle - rotated

        msg = Twist()

        # Slow down near target
        if remaining > 0.3:
            speed = self._angular_speed
        elif remaining > 0.1:
            speed = self._angular_speed * 0.5
        else:
            speed = 0.08

        msg.angular.z = self._direction * speed

        self._cmd_pub.publish(msg)

        return None

    def on_exit(self, userdata):
        if self._cmd_pub is not None:
            self._cmd_pub.publish(Twist())

        Logger.loginfo('Rotation stopped.')