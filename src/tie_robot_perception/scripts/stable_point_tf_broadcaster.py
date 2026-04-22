#!/usr/bin/env python3

import copy
import math
from collections import defaultdict

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler

try:
    from tie_robot_msgs.msg import AreaProgress
    from tie_robot_msgs.msg import PointsArray
except ImportError:  # pragma: no cover - helps unit tests import helpers in isolation
    AreaProgress = None
    PointsArray = None


def _is_finite_number(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _point_to_dict(point):
    if isinstance(point, dict):
        return point

    return {
        "idx": getattr(point, "idx", None),
        "Pix_coord": list(getattr(point, "Pix_coord", [0, 0])),
        "World_coord": list(getattr(point, "World_coord", [0.0, 0.0, 0.0])),
        "Angle": getattr(point, "Angle", 0.0),
        "is_shuiguan": getattr(point, "is_shuiguan", False),
    }


def _area_progress_to_dict(msg):
    if isinstance(msg, dict):
        return msg

    return {
        "current_area_index": getattr(msg, "current_area_index", 0),
        "total_area_count": getattr(msg, "total_area_count", 0),
        "just_finished_area_index": getattr(msg, "just_finished_area_index", 0),
        "ready_for_next_area": getattr(msg, "ready_for_next_area", False),
        "all_done": getattr(msg, "all_done", False),
    }


def _build_transform_spec(parent_frame, child_frame, world_coord_mm):
    x_mm, y_mm, z_mm = world_coord_mm
    translation_m = [float(x_mm) / 1000.0, float(y_mm) / 1000.0, float(z_mm) / 1000.0]
    return {
        "header_frame_id": parent_frame,
        "child_frame_id": child_frame,
        "translation_m": translation_m,
        "rotation_rpy": [0.0, 0.0, 0.0],
    }


def _spec_to_transform(spec):
    transform = TransformStamped()
    transform.header.frame_id = spec["header_frame_id"]
    transform.child_frame_id = spec["child_frame_id"]
    transform.transform.translation.x = spec["translation_m"][0]
    transform.transform.translation.y = spec["translation_m"][1]
    transform.transform.translation.z = spec["translation_m"][2]

    quat = quaternion_from_euler(*spec["rotation_rpy"])
    transform.transform.rotation.x = quat[0]
    transform.transform.rotation.y = quat[1]
    transform.transform.rotation.z = quat[2]
    transform.transform.rotation.w = quat[3]
    return transform


class StablePointTracker:
    def __init__(self, stable_frame_count=2, stable_z_tolerance_mm=4.0, parent_frame="cabin_frame"):
        if stable_frame_count < 1:
            raise ValueError("stable_frame_count must be at least 1")
        if stable_z_tolerance_mm < 0.0:
            raise ValueError("stable_z_tolerance_mm must be non-negative")

        self.stable_frame_count = int(stable_frame_count)
        self.stable_z_tolerance_mm = float(stable_z_tolerance_mm)
        self.parent_frame = parent_frame
        self.active_area_index = 0
        self._streaks = defaultdict(list)
        self._active_specs = {}
        self._archived_specs = {}
        self._last_point_specs = {}

    def ingest_points(self, points):
        published = []
        for raw_point in points:
            point = _point_to_dict(raw_point)
            idx = point.get("idx")
            world_coord = point.get("World_coord") or [0.0, 0.0, 0.0]
            if idx in (None, "") or len(world_coord) < 3:
                continue

            try:
                idx = int(idx)
            except (TypeError, ValueError):
                continue

            z_mm = float(world_coord[2])
            if not _is_finite_number(z_mm):
                continue

            streak = self._streaks[idx]
            if streak and abs(streak[-1] - z_mm) > self.stable_z_tolerance_mm:
                streak = []
            streak.append(z_mm)
            if len(streak) > self.stable_frame_count:
                streak = streak[-self.stable_frame_count :]
            self._streaks[idx] = streak

            self._last_point_specs[idx] = _build_transform_spec(
                self.parent_frame,
                f"bind_point_{idx}",
                world_coord,
            )

            if len(streak) < self.stable_frame_count:
                continue

            if max(streak[-self.stable_frame_count :]) - min(streak[-self.stable_frame_count :]) > self.stable_z_tolerance_mm:
                continue

            spec = self._last_point_specs[idx]
            previous = self._active_specs.get(idx)
            if previous == spec:
                continue

            self._active_specs[idx] = spec
            published.append(spec)

        return published

    def ingest_area_progress(self, area_progress):
        progress = _area_progress_to_dict(area_progress)
        current_area_index = int(progress.get("current_area_index") or 0)
        archived = []

        if current_area_index != self.active_area_index:
            archived.extend(self._archive_active_points(self.active_area_index))
            self.active_area_index = current_area_index

        if progress.get("all_done") and self._active_specs:
            archived.extend(self._archive_active_points(self.active_area_index))

        return archived

    def snapshot_transforms(self):
        transforms = list(self._archived_specs.values()) + list(self._active_specs.values())
        transforms.sort(key=lambda spec: spec["child_frame_id"])
        return transforms

    def _archive_active_points(self, area_index):
        archived = []
        if area_index in (None, 0):
            self._active_specs = {}
            self._streaks = defaultdict(list)
            self._last_point_specs = {}
            return archived

        for idx, spec in sorted(self._active_specs.items()):
            archived_spec = copy.deepcopy(spec)
            archived_spec["child_frame_id"] = f"area_{area_index}_point_{idx}"
            self._archived_specs[archived_spec["child_frame_id"]] = archived_spec
            archived.append(archived_spec)

        self._active_specs = {}
        self._streaks = defaultdict(list)
        self._last_point_specs = {}
        return archived


class StablePointTFBroadcaster:
    def __init__(self):
        stable_frame_count = max(1, int(rospy.get_param("~stable_frame_count", 2)))
        stable_z_tolerance_mm = float(rospy.get_param("~stable_z_tolerance_mm", 4.0))
        publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 20.0))
        if not math.isfinite(publish_rate_hz) or publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be a positive finite number")

        self.tracker = StablePointTracker(
            stable_frame_count=stable_frame_count,
            stable_z_tolerance_mm=stable_z_tolerance_mm,
            parent_frame="cabin_frame",
        )
        self.publish_rate_hz = publish_rate_hz
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.rate = rospy.Rate(publish_rate_hz)
        self._subscribers = []
        self._subscribers.append(
            rospy.Subscriber("/coordinate_point", PointsArray, self.coordinate_callback)
        )
        self._subscribers.append(
            rospy.Subscriber("/cabin/area_progress", AreaProgress, self.area_progress_callback)
        )

    def coordinate_callback(self, msg):
        points = getattr(msg, "PointCoordinatesArray", []) or []
        if len(points) != 4:
            return
        published = self.tracker.ingest_points(points)
        self._publish_specs(published)

    def area_progress_callback(self, msg):
        archived = self.tracker.ingest_area_progress(msg)
        self._publish_specs(archived)

    def spin_once(self):
        self._publish_specs(self.tracker.snapshot_transforms())

    def spin(self):
        while not rospy.is_shutdown():
            self.spin_once()
            self.rate.sleep()

    def _publish_specs(self, specs):
        for spec in specs:
            transform = _spec_to_transform(spec)
            transform.header.stamp = rospy.Time.now()
            self.broadcaster.sendTransform(transform)
def main():
    rospy.init_node("stable_point_tf_broadcaster")
    try:
        node = StablePointTFBroadcaster()
    except Exception as exc:
        rospy.logfatal("Failed to start stable_point_tf_broadcaster: %s", exc)
        raise SystemExit(1) from exc

    rospy.loginfo(
        "stable_point_tf_broadcaster started: parent=%s stable_frame_count=%d stable_z_tolerance_mm=%.3f publish_rate_hz=%.3f",
        node.tracker.parent_frame,
        node.tracker.stable_frame_count,
        node.tracker.stable_z_tolerance_mm,
        node.publish_rate_hz,
    )
    node.spin()


if __name__ == "__main__":
    main()
