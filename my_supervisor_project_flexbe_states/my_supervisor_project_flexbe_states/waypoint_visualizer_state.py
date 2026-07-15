#!/usr/bin/env python3

from flexbe_core import EventState, Logger

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


class WaypointVisualizerState(EventState):

    def __init__(self):

        super().__init__(
            outcomes=['done']
        )

        self.pub = None

    def on_enter(self, userdata):

        if self.pub is None:

            self.pub = self._node.create_publisher(
                MarkerArray,
                "/waypoints",
                10
            )

        points = [

            (0.0, 0.0),   # WP1
            (2.0, 0.0),   # WP2
            (2.0, 2.0),   # WP3
            (0.0, 2.0)    # WP4

        ]

        colors = [

            (1.0, 1.0, 0.0),   # Yellow
            (0.0, 0.0, 1.0),   # Blue
            (0.0, 1.0, 0.0),   # Green
            (1.0, 0.0, 0.0)    # Red

        ]

        array = MarkerArray()

        for i, (x, y) in enumerate(points):

            marker = Marker()

            marker.header.frame_id = "odom"
            marker.header.stamp = self._node.get_clock().now().to_msg()

            marker.ns = "waypoints"
            marker.id = i

            # Flat ground marker
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD

            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.005

            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

            # 10 cm diameter, 1 cm thick
            marker.scale.x = 0.10
            marker.scale.y = 0.10
            marker.scale.z = 0.01

            marker.color.r = colors[i][0]
            marker.color.g = colors[i][1]
            marker.color.b = colors[i][2]
            marker.color.a = 1.0

            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 0

            array.markers.append(marker)

        self.pub.publish(array)

        Logger.loginfo("Waypoint markers published.")

    def execute(self, userdata):

        return 'done'