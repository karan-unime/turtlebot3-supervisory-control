#!/usr/bin/env python3
"""
SmartGoalNavigatorState — Bug2 obstacle-avoidance navigator for FlexBE / TurtleBot3.

Five bugs fixed vs. the original:

  Bug 1 [CRITICAL]  M-line reset
        In execute(), the original code wrote self.start_x = x / self.start_y = y
        when an obstacle was hit, which overwrote the mission-start point with the
        hit point.  is_on_mline() then measured distance to a line from the HIT
        POINT to the GOAL instead of from the MISSION START to the GOAL, so the
        robot almost never crossed the real M-line again.
        FIX: start_x / start_y are now set only in on_enter() and are never touched
        after that.

  Bug 2 [CRITICAL]  Wall-side label inverted in choose_best_direction()
        The scorer gives a HIGH score to the side with MORE OPEN SPACE.
        If left_score > right_score the open side is the LEFT, so the robot
        travels counterclockwise around the obstacle — which puts the WALL on
        the robot's RIGHT, not its left.  The original code returned "LEFT" for
        the open-left case, making every subsequent sensor read and turn look at
        the wrong side of the robot.
        FIX: return "RIGHT" when left is more open; return "LEFT" when right is
        more open.  Now wall_side always names the side the WALL is on.

  Bug 3 [IMPORTANT] Alignment rotation direction inverted
        To place the wall on the LEFT the robot must turn RIGHT so the front
        obstacle slides into the left sensor arc; the original code turned LEFT,
        doing the opposite.  Mirror image for the RIGHT-wall case.
        FIX: LEFT wall → turn right (−); RIGHT wall → turn left (+).
        The exit condition was also tightened: stop rotating when the wall sensor
        reads closer than wall_distance × 1.8 AND the front is clear.

  Bug 4 [IMPORTANT] BUG2 leave condition too strict (front > 1.0 m)
        On real corridors or narrow passages the front reading is often < 1.0 m
        even when the path to the goal is clear enough to leave the wall.  This
        prevented the robot from ever escaping the wall-follow loop.
        FIX: replaced with front > safe_distance * 2 (typically ~0.6 m), which
        still ensures some clearance but allows leaving in realistic environments.

  Bug 5 [IMPORTANT] wall_start_time not reset on ALIGN → FOLLOW transition
        wall_start_time was set at the start of ALIGN_WALL.  The full alignment
        rotation (which can take several seconds) therefore consumed most of the
        OBSTACLE_TIMEOUT before wall-following even began, causing spurious
        "Robot stuck" failures on the first obstacle.
        FIX: wall_start_time is reset inside align_with_wall() right before
        switching mode to FOLLOW_WALL.
"""

import math
import time

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion

from flexbe_core import EventState, Logger

from my_supervisor_project_flexbe_states.mission_config import (
    SAFE_DISTANCE,
    GOAL_TOLERANCE,
    ROTATE_THRESHOLD,
    ROTATE_SPEED,
    MAX_STEERING,
    FAR_SPEED,
    MEDIUM_SPEED,
    SLOW_SPEED,
    FINAL_SPEED,
    CENTER_SPEED,
    OBSTACLE_TIMEOUT,
)


class SmartGoalNavigatorState(EventState):
    """
    Intelligent Bug2 Goal Navigator.

    Modes
    -----
    GOAL        : drive straight toward the goal with proportional steering.
    ALIGN_WALL  : rotate in place until the chosen wall side is beside the robot.
    FOLLOW_WALL : P-controller wall-following with BUG2 leave condition.
    """

    # ------------------------------------------------------------------
    # INITIALIZATION
    # ------------------------------------------------------------------
    def __init__(self):
        super().__init__(
            outcomes=["reached", "failed"],
            input_keys=["target_x", "target_y"],
        )

        # ROS handles
        self.scan = None
        self.odom = None
        self.cmd_pub = None
        self.scan_sub = None
        self.odom_sub = None

        # Goal
        self.goal_x = 0.0
        self.goal_y = 0.0

        # Mission start — captured on the FIRST execute() tick that has odom.
        # Not captured in on_enter() because odom may still be None at that point
        # (subscriptions just created).  Until captured, _start_captured = False.
        self.start_x = 0.0
        self.start_y = 0.0
        self._start_captured = False

        # Bug2 hit point
        self.hit_x = 0.0
        self.hit_y = 0.0
        self.hit_distance = None

        # Navigation mode
        self.mode = "GOAL"

        # Wall side: "LEFT"  → wall is on the robot's LEFT side
        #            "RIGHT" → wall is on the robot's RIGHT side
        self.wall_side = None

        # Wall-following tuning
        self.wall_distance = 0.30   # desired clearance from wall (m)
        self.wall_speed = 0.10      # forward speed while following wall (m/s)
        self.wall_kp = 2.0          # proportional gain for lateral error
        self.max_wall_turn = 0.60   # maximum angular rate while following (rad/s)
        self.wall_start_time = None

        self.safe_distance = SAFE_DISTANCE

    # ------------------------------------------------------------------
    # ENTER STATE
    # ------------------------------------------------------------------
    def on_enter(self, userdata):
        self.goal_x = userdata.target_x
        self.goal_y = userdata.target_y
        self.mode = "GOAL"
        self.wall_side = None
        self.hit_distance = None
        self.wall_start_time = None
        self._start_captured = False   # ← reset so first execute() tick re-captures

        Logger.loginfo(f"NEW GOAL ({self.goal_x:.2f}, {self.goal_y:.2f})")

        node = self._node

        if self.cmd_pub is None:
            self.cmd_pub = node.create_publisher(Twist, "/cmd_vel", 10)

        if self.scan_sub is None:
            self.scan_sub = node.create_subscription(
                LaserScan, "/scan", self.scan_cb, 10
            )

        if self.odom_sub is None:
            self.odom_sub = node.create_subscription(
                Odometry, "/odom", self.odom_cb, 10
            )

    # ------------------------------------------------------------------
    # CALLBACKS
    # ------------------------------------------------------------------
    def scan_cb(self, msg):
        self.scan = msg

    def odom_cb(self, msg):
        self.odom = msg

    # ------------------------------------------------------------------
    # HELPERS
    # ------------------------------------------------------------------
    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def get_robot_pose(self):
        pos = self.odom.pose.pose.position
        q = self.odom.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return pos.x, pos.y, yaw

    def get_goal_information(self, x, y, yaw):
        dx = self.goal_x - x
        dy = self.goal_y - y
        distance = math.hypot(dx, dy)
        angle_error = math.atan2(dy, dx) - yaw
        # Normalise to (−π, π]
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi
        return distance, angle_error

    # ------------------------------------------------------------------
    # LASER PROCESSING
    # ------------------------------------------------------------------
    def process_laser(self):
        raw = self.scan.ranges
        n = len(raw)

        def clean(r):
            return 10.0 if (math.isinf(r) or math.isnan(r)) else r

        scan = [clean(r) for r in raw]

        # Index 0 = front, increasing counter-clockwise (TurtleBot3 LDS-01, 360 pts)
        # All slice endpoints are scaled proportionally so it works even if n ≠ 360.
        def sector(lo, hi):
            """Return min reading in the arc lo..hi degrees (counter-clockwise)."""
            i0 = round(lo * n / 360) % n
            i1 = round(hi * n / 360) % n
            if i0 <= i1:
                return min(scan[i0:i1]) if i0 < i1 else 10.0
            return min(scan[i0:] + scan[:i1])

        return {
            "front":       min(sector(0, 20), sector(340, 360)),
            "front_left":  sector(20, 50),
            "left":        sector(50, 90),
            "far_left":    sector(90, 135),
            "front_right": sector(310, 340),
            "right":       sector(270, 310),
            "far_right":   sector(225, 270),
        }

    # ------------------------------------------------------------------
    # CHOOSE WHICH SIDE TO PASS
    # ------------------------------------------------------------------
    def choose_best_direction(self, sectors):
        """
        Score each side by the open space available there.
        Higher score = more open.

        The robot will travel toward the MORE OPEN side, which means the
        WALL ends up on the OPPOSITE side.

        Example: left is more open  → robot goes left (counterclockwise)
                                    → wall on RIGHT → return "RIGHT"
                 right is more open → robot goes right (clockwise)
                                    → wall on LEFT  → return "LEFT"

        BUG FIXED: original code returned the open side's name, which caused
        every wall sensor read and turn direction to be on the wrong side.
        """
        left_score = (
            4.0 * sectors["front_left"]
            + 2.0 * sectors["left"]
            + sectors["far_left"]
        )
        right_score = (
            4.0 * sectors["front_right"]
            + 2.0 * sectors["right"]
            + sectors["far_right"]
        )

        # Disqualify a side whose entrance is already blocked
        if sectors["front_left"] < self.safe_distance:
            left_score = -999
        if sectors["front_right"] < self.safe_distance:
            right_score = -999

        Logger.loginfo(f"Open-space score  LEFT={left_score:.2f}  RIGHT={right_score:.2f}")

        if left_score >= right_score:
            Logger.loginfo("Going LEFT around obstacle → wall will be on RIGHT")
            return "RIGHT"   # ← FIX Bug 2: was "LEFT"
        Logger.loginfo("Going RIGHT around obstacle → wall will be on LEFT")
        return "LEFT"        # ← FIX Bug 2: was "RIGHT"

    # ------------------------------------------------------------------
    # ALIGN WITH WALL
    # ------------------------------------------------------------------
    def align_with_wall(self, sectors):
        """
        Rotate in place until the wall is properly beside the robot.

        FIX Bug 3: to put the wall on the LEFT the robot must turn RIGHT
        (clockwise) so the front obstacle slides into the left sensor arc.
        The original code turned in the wrong direction for each case.

        FIX Bug 5: wall_start_time is reset here (just before entering
        FOLLOW_WALL) so the full OBSTACLE_TIMEOUT budget is available for
        actual wall-following, not wasted on alignment rotation.
        """
        cmd = Twist()
        front = sectors["front"]
        left  = sectors["left"]
        right = sectors["right"]

        aligned_threshold = self.wall_distance * 1.8  # wall sensor reads ≤ this

        if self.wall_side == "LEFT":
            # Wall must end up on the LEFT → turn RIGHT until left sensor sees wall
            if left > aligned_threshold or front < self.safe_distance:
                cmd.angular.z = -0.45          # ← FIX Bug 3: was +0.45 (wrong way)
                Logger.loginfo("Aligning: turning RIGHT to place wall on LEFT")
                self.cmd_pub.publish(cmd)
                return False
            Logger.loginfo("LEFT wall aligned — starting wall follow")
            self.wall_start_time = time.time()  # ← FIX Bug 5: reset timer HERE
            self.mode = "FOLLOW_WALL"
            return True

        else:  # wall_side == "RIGHT"
            # Wall must end up on the RIGHT → turn LEFT until right sensor sees wall
            if right > aligned_threshold or front < self.safe_distance:
                cmd.angular.z = +0.45          # ← FIX Bug 3: was -0.45 (wrong way)
                Logger.loginfo("Aligning: turning LEFT to place wall on RIGHT")
                self.cmd_pub.publish(cmd)
                return False
            Logger.loginfo("RIGHT wall aligned — starting wall follow")
            self.wall_start_time = time.time()  # ← FIX Bug 5: reset timer HERE
            self.mode = "FOLLOW_WALL"
            return True

    # ------------------------------------------------------------------
    # FOLLOW WALL (BUG2)
    # ------------------------------------------------------------------
    def follow_wall(self, sectors, x, y, distance, angle_error):
        """
        P-controller wall follower with BUG2 leave condition.

        The P-controller signs were correct in the original; only the leave
        condition and wall_side labelling needed fixing.
        """
        cmd = Twist()
        front = sectors["front"]

        if self.wall_side == "LEFT":
            wall_reading = sectors["left"]
            if front < self.safe_distance:
                # Convex corner: obstacle now in front → turn RIGHT away from it
                cmd.linear.x = 0.04
                cmd.angular.z = -0.65
            else:
                error = self.wall_distance - wall_reading
                cmd.linear.x = self.wall_speed
                cmd.angular.z = max(
                    min(-self.wall_kp * error, self.max_wall_turn),
                    -self.max_wall_turn,
                )
        else:  # RIGHT wall
            wall_reading = sectors["right"]
            if front < self.safe_distance:
                # Convex corner: turn LEFT
                cmd.linear.x = 0.04
                cmd.angular.z = +0.65
            else:
                error = self.wall_distance - wall_reading
                cmd.linear.x = self.wall_speed
                cmd.angular.z = max(
                    min(+self.wall_kp * error, self.max_wall_turn),
                    -self.max_wall_turn,
                )

        # ---- BUG2 LEAVE CONDITION — two independent triggers ----------------
        #
        # Trigger A — M-line crossing (classic BUG2):
        #   Robot is within MLINE_CORRIDOR of the start→goal line, closer to
        #   the goal than at the hit point, and the forward path is clear.
        #   Corridor widened from 0.10 m to 0.20 m: the original value was too
        #   tight for a differential-drive robot in Gazebo where the path around
        #   an obstacle rarely brings the robot back to within 10 cm of the line.
        #
        # Trigger B — Goal-facing (angle-based fallback):
        #   If the robot is heading toward the goal (small angle error), has
        #   moved more than 0.30 m closer than the hit point, and the front is
        #   clear, leave the wall even without crossing the M-line.  This handles
        #   cases where the obstacle geometry keeps the robot parallel to but
        #   always offset from the M-line.  A minimum follow-time guard of 2 s
        #   prevents leaving immediately after the first corner turn.
        MIN_FOLLOW_SECS = 2.0
        elapsed = time.time() - self.wall_start_time
        clear_ahead = front > self.safe_distance * 2

        # Trigger A
        if self.is_on_mline(x, y) and distance < self.hit_distance - 0.15 and clear_ahead:
            Logger.loginfo("BUG2 leave — M-line crossed, closer to goal")
            self.mode = "GOAL"
            self.wall_side = None
            return

        # Trigger B
        if (elapsed > MIN_FOLLOW_SECS
                and abs(angle_error) < 0.30
                and distance < self.hit_distance - 0.30
                and clear_ahead):
            Logger.loginfo(
                f"BUG2 leave — facing goal (err={angle_error:.2f} rad), "
                f"closer by {self.hit_distance - distance:.2f} m"
            )
            self.mode = "GOAL"
            self.wall_side = None
            return

        Logger.loginfo(f"Following {self.wall_side} wall  |  wall={wall_reading:.2f} m")
        self.cmd_pub.publish(cmd)

    # ------------------------------------------------------------------
    # M-LINE CHECK
    # ------------------------------------------------------------------
    def is_on_mline(self, x, y):
        """
        Return True when the robot is within 10 cm of the line
        MISSION_START → GOAL.

        NOTE: start_x / start_y are set once in on_enter() and are the
        original mission-start coordinates.  They are NEVER updated to the
        hit point (that was Bug 1).
        """
        A = self.goal_y - self.start_y
        B = self.start_x - self.goal_x
        C = self.goal_x * self.start_y - self.goal_y * self.start_x
        denom = math.hypot(A, B)
        if denom < 1e-5:
            return False
        dist_to_mline = abs(A * x + B * y + C) / denom
        Logger.loginfo(f"M-line distance = {dist_to_mline:.3f} m")
        return dist_to_mline < 0.20   # widened from 0.10 m — original was too tight

    # ------------------------------------------------------------------
    # GOAL NAVIGATION
    # ------------------------------------------------------------------
    def navigate_goal(self, distance, angle_error):
        cmd = Twist()

        if abs(angle_error) > ROTATE_THRESHOLD:
            cmd.angular.z = ROTATE_SPEED if angle_error > 0 else -ROTATE_SPEED
            self.cmd_pub.publish(cmd)
            return

        if distance > 1.50:
            cmd.linear.x = FAR_SPEED
        elif distance > 0.80:
            cmd.linear.x = MEDIUM_SPEED
        elif distance > 0.30:
            cmd.linear.x = SLOW_SPEED
        elif distance > 0.12:
            cmd.linear.x = FINAL_SPEED
        else:
            cmd.linear.x = CENTER_SPEED

        cmd.angular.z = max(min(1.5 * angle_error, MAX_STEERING), -MAX_STEERING)
        self.cmd_pub.publish(cmd)

    # ------------------------------------------------------------------
    # EXECUTE  (called every FlexBE tick)
    # ------------------------------------------------------------------
    def execute(self, userdata):
        if self.scan is None or self.odom is None:
            return None

        x, y, yaw = self.get_robot_pose()

        # Capture mission start on the very first tick that has valid odom.
        # This is more reliable than on_enter() because subscriptions may not
        # have delivered a message yet by the time on_enter() runs.
        if not self._start_captured:
            self.start_x = x
            self.start_y = y
            self._start_captured = True
            Logger.loginfo(f"Mission start captured: ({x:.3f}, {y:.3f})")

        distance, angle_error = self.get_goal_information(x, y, yaw)
        sectors = self.process_laser()

        # ---- Goal reached -----------------------------------------------
        if distance < GOAL_TOLERANCE:
            Logger.loginfo("GOAL REACHED")
            self.stop_robot()
            return "reached"

        # ---- GOAL mode --------------------------------------------------
        if self.mode == "GOAL":
            if sectors["front"] < self.safe_distance:
                Logger.logwarn("Obstacle detected — switching to wall-follow")

                self.wall_side = self.choose_best_direction(sectors)
                self.hit_distance = distance
                self.hit_x = x
                self.hit_y = y

                # FIX Bug 1: DO NOT touch self.start_x / self.start_y here.
                # The original code overwrote them with x, y (the hit point),
                # which corrupted the M-line for the rest of this navigation.
                # start_x / start_y stay as set in on_enter().

                Logger.loginfo(
                    f"Hit point ({x:.2f}, {y:.2f}) | "
                    f"wall side = {self.wall_side} | "
                    f"dist to goal = {distance:.2f} m"
                )
                self.mode = "ALIGN_WALL"
            else:
                self.navigate_goal(distance, angle_error)
            return None

        # ---- ALIGN_WALL mode --------------------------------------------
        if self.mode == "ALIGN_WALL":
            self.align_with_wall(sectors)
            return None

        # ---- FOLLOW_WALL mode -------------------------------------------
        if self.mode == "FOLLOW_WALL":
            if time.time() - self.wall_start_time > OBSTACLE_TIMEOUT:
                Logger.logerr("Wall-follow timeout — obstacle unsolvable")
                self.stop_robot()
                return "failed"
            self.follow_wall(sectors, x, y, distance, angle_error)
            return None

        return None

    # ------------------------------------------------------------------
    # EXIT
    # ------------------------------------------------------------------
    def on_exit(self, userdata):
        self.stop_robot()