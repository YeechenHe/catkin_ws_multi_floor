#!/usr/bin/env python3
"""
Trajectory recorder node for multi-floor navigation.

Subscribes to /amcl_pose and records two separate paths:
  - /trajectory_floor0  (nav_msgs/Path, blue in RViz)
  - /trajectory_floor1  (nav_msgs/Path, red  in RViz)

Floor switching is detected by monitoring the /change_map service
responses via a dedicated subscriber on /floor_level topic published
by change_map_node, or by directly watching the map frame change.
Since change_map_node does not publish a floor topic, we detect the
floor switch by listening to the same IntTrigger service call result
through a lightweight topic /current_floor_level that this node
itself manages via a subscriber on /initialpose (which multi_floor_nav
publishes each time it re-localizes on a new floor).

Detection strategy:
  - Floor 0 initial pose is near (4.0, -5.0)
  - Floor 1 initial pose is near (3.0, -0.5)
  We watch /initialpose and compare position to known floor poses.
"""

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Path
import math


FLOOR0_INIT_X = 4.0
FLOOR0_INIT_Y = -5.0
FLOOR1_INIT_X = 3.0
FLOOR1_INIT_Y = -0.5
FLOOR_SWITCH_THRESHOLD = 1.5  # metres radius to classify which floor


def distance_2d(x1: float, y1: float, x2: float, y2: float) -> float:
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


class TrajectoryRecorder:
    def __init__(self) -> None:
        rospy.init_node("trajectory_recorder", anonymous=False)

        self._current_floor: int = 0

        self._path_floor0 = Path()
        self._path_floor0.header.frame_id = "map"

        self._path_floor1 = Path()
        self._path_floor1.header.frame_id = "map"

        self._pub_floor0 = rospy.Publisher(
            "/trajectory_floor0", Path, queue_size=1, latch=True
        )
        self._pub_floor1 = rospy.Publisher(
            "/trajectory_floor1", Path, queue_size=1, latch=True
        )

        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self._amcl_callback)
        rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self._init_pose_callback)

        rospy.loginfo("[trajectory_recorder] Started. Recording floor 0 trajectory.")

    def _detect_floor_from_pose(self, x: float, y: float) -> int:
        dist0 = distance_2d(x, y, FLOOR0_INIT_X, FLOOR0_INIT_Y)
        dist1 = distance_2d(x, y, FLOOR1_INIT_X, FLOOR1_INIT_Y)
        if dist0 < dist1:
            return 0
        return 1

    def _init_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        """Detect floor switch when multi_floor_nav publishes a new initial pose."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        detected_floor = self._detect_floor_from_pose(x, y)
        if detected_floor != self._current_floor:
            rospy.loginfo(
                "[trajectory_recorder] Floor switch detected: %d -> %d",
                self._current_floor,
                detected_floor,
            )
            self._current_floor = detected_floor

    def _amcl_callback(self, msg: PoseWithCovarianceStamped) -> None:
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose = msg.pose.pose

        if self._current_floor == 0:
            self._path_floor0.header.stamp = rospy.Time.now()
            self._path_floor0.poses.append(pose_stamped)
            self._pub_floor0.publish(self._path_floor0)
        else:
            self._path_floor1.header.stamp = rospy.Time.now()
            self._path_floor1.poses.append(pose_stamped)
            self._pub_floor1.publish(self._path_floor1)

    def spin(self) -> None:
        rospy.spin()


if __name__ == "__main__":
    recorder = TrajectoryRecorder()
    recorder.spin()
