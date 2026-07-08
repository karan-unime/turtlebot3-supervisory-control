#!/usr/bin/env python3

import math
from dataclasses import dataclass

from sensor_msgs.msg import LaserScan
from flexbe_core import EventState, Logger


# ==================================================
# GAP STRUCTURE
# ==================================================

@dataclass
class Gap:

    start_index: int
    end_index: int

    center_index: int

    width: int

    min_distance: float
    avg_distance: float

    angle: float

    score: float = 0.0



# ==================================================
# SMART NAVIGATOR
# ==================================================

class SmartNavigatorState(EventState):

    """
    LiDAR based trajectory planner

    <= path_found
    <= no_path

    ># target_angle
    ># free_distance
    ># gap_quality
    """


    def __init__(self):

        super().__init__(

            outcomes=[
                'path_found',
                'no_path'
            ],

            output_keys=[
                'target_angle',
                'free_distance',
                'gap_quality'
            ]
        )


        self._scan = None
        self._scan_sub = None

        self._safe_distance = 0.8



    # ==================================================
    # START
    # ==================================================

    def on_enter(self, userdata):


        if self._scan_sub is None:


            self._scan_sub = (
                self._node.create_subscription(
                    LaserScan,
                    '/scan',
                    self._scan_callback,
                    10
                )
            )


        Logger.loginfo(
            "SmartNavigatorState started - waiting LiDAR"
        )



    # ==================================================
    # CALLBACK
    # ==================================================

    def _scan_callback(self, msg):

        self._scan = msg



    # ==================================================
    # CLEAN LIDAR
    # ==================================================

    def _clean_ranges(self):

        clean = []


        for r in self._scan.ranges:


            if math.isinf(r):

                r = self._scan.range_max


            elif math.isnan(r):

                r = 0.0


            clean.append(r)


        return clean



    # ==================================================
    # CONVERT TURTLEBOT3 LDS ANGLE
    # ==================================================

    def _robot_angle(self, index):


        raw_angle = (
            self._scan.angle_min
            +
            index *
            self._scan.angle_increment
        )


        # TurtleBot3 LDS mapping:
        #
        # raw 180 deg = FRONT
        #
        # convert:
        # front = 0 deg
        # left  = +90 deg
        # right = -90 deg


        angle = raw_angle - math.pi


        if angle > math.pi:

            angle -= 2 * math.pi


        if angle < -math.pi:

            angle += 2 * math.pi


        return angle


    # ==================================================
    # CREATE GAP
    # ==================================================

    def _create_gap(
        self,
        ranges,
        start,
        end
    ):


        if end <= start:

            return None



        gap_ranges = ranges[start:end+1]


        if len(gap_ranges) == 0:

            return None



        center = int(
            (start + end) / 2
        )


        angle = self._robot_angle(
            center
        )


        Logger.loginfo(
            f"Gap center={center}, "
            f"angle={math.degrees(angle):.1f}"
        )



        return Gap(

            start_index=start,

            end_index=end,

            center_index=center,

            width=end-start,

            min_distance=min(
                gap_ranges
            ),

            avg_distance=
                sum(gap_ranges)
                /
                len(gap_ranges),

            angle=angle
        )



    # ==================================================
    # FIND GAPS
    # ==================================================

    def _detect_gaps(
        self,
        ranges
    ):


        gaps = []

        start = None



        for i, distance in enumerate(ranges):


            angle = self._robot_angle(i)



            # only front hemisphere
            # -90 to +90

            if abs(angle) > math.radians(90):

                continue



            if distance > self._safe_distance:


                if start is None:

                    start = i


            else:


                if start is not None:


                    gap = self._create_gap(
                        ranges,
                        start,
                        i-1
                    )


                    if gap:

                        gaps.append(gap)


                    start = None



        if start is not None:


            gap = self._create_gap(
                ranges,
                start,
                len(ranges)-1
            )


            if gap:

                gaps.append(gap)



        return gaps



    # ==================================================
    # SCORE PATHS
    # ==================================================

    def _score_gaps(
        self,
        gaps
    ):


        for gap in gaps:


            width_score = min(
                gap.width / 100,
                1
            )


            distance_score = (
                gap.min_distance
                /
                self._scan.range_max
            )


            forward_score = (
                1
                -
                abs(gap.angle)
                /
                math.radians(90)
            )


            gap.score = (

                0.3 * width_score

                +

                0.5 * distance_score

                +

                0.2 * forward_score
            )


        return gaps



    # ==================================================
    # SELECT BEST
    # ==================================================

    def _select_best_gap(
        self,
        gaps
    ):


        if len(gaps) == 0:

            return None



        return max(
            gaps,
            key=lambda g: g.score
        )



    # ==================================================
    # EXECUTE
    # ==================================================

    def execute(
        self,
        userdata
    ):


        if self._scan is None:


            Logger.loginfo(
                "Waiting for LiDAR data..."
            )


            return None



        ranges = self._clean_ranges()



        gaps = self._detect_gaps(
            ranges
        )



        if len(gaps) == 0:


            Logger.logwarn(
                "No path found"
            )


            return 'no_path'



        self._score_gaps(
            gaps
        )


        best = self._select_best_gap(
            gaps
        )


        angle_deg = math.degrees(
            best.angle
        )



        Logger.loginfo(
            f"Gaps found: {len(gaps)}"
        )


        Logger.loginfo(
            f"Chosen trajectory: {angle_deg:.1f} deg"
        )


        Logger.loginfo(
            f"Clear distance: {best.min_distance:.2f} m"
        )


        Logger.loginfo(
            f"Score: {best.score:.2f}"
        )



        userdata.target_angle = angle_deg


        # move small step,
        # then rescan

        userdata.free_distance = min(
            best.min_distance,
            0.3
        )


        userdata.gap_quality = best.score



        return 'path_found'
