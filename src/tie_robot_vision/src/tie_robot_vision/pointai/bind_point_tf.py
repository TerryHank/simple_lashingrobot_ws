"""Publish current-frame bind points in the raw camera frame as TF."""

import math

import rospy
from geometry_msgs.msg import TransformStamped


RAW_CAMERA_BIND_POINT_SOURCE_FRAME = "Scepter_depth_frame"
RAW_CAMERA_BIND_POINT_CHILD_PREFIX = "surface_dp_bind_point"


def build_raw_camera_bind_point_transforms(
    points_array_msg,
    stamp,
    source_frame=RAW_CAMERA_BIND_POINT_SOURCE_FRAME,
    child_prefix=RAW_CAMERA_BIND_POINT_CHILD_PREFIX,
):
    transforms = []
    points = getattr(points_array_msg, "PointCoordinatesArray", []) or []

    for fallback_index, point in enumerate(points, start=1):
        raw_coord = list(getattr(point, "World_coord", []))
        if len(raw_coord) < 3:
            continue

        x_mm, y_mm, z_mm = (float(raw_coord[0]), float(raw_coord[1]), float(raw_coord[2]))
        if not all(math.isfinite(value) for value in (x_mm, y_mm, z_mm)):
            continue

        point_index = int(getattr(point, "idx", 0) or fallback_index)
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = source_frame
        transform.child_frame_id = f"{child_prefix}_{point_index}"
        transform.transform.translation.x = x_mm / 1000.0
        transform.transform.translation.y = y_mm / 1000.0
        transform.transform.translation.z = z_mm / 1000.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0
        transforms.append(transform)

    return transforms


def publish_raw_camera_bind_point_transforms(self, points_array_msg):
    broadcaster = getattr(self, "raw_bind_point_tf_broadcaster", None)
    if broadcaster is None:
        return

    transforms = build_raw_camera_bind_point_transforms(
        points_array_msg,
        rospy.Time.now(),
        source_frame=getattr(self, "raw_bind_point_tf_source_frame", RAW_CAMERA_BIND_POINT_SOURCE_FRAME),
        child_prefix=getattr(self, "raw_bind_point_tf_child_prefix", RAW_CAMERA_BIND_POINT_CHILD_PREFIX),
    )
    self.latest_raw_bind_point_transforms = transforms
    if not transforms:
        return

    broadcaster.sendTransform(transforms)


def republish_latest_raw_camera_bind_point_transforms(self, _event=None):
    broadcaster = getattr(self, "raw_bind_point_tf_broadcaster", None)
    transforms = getattr(self, "latest_raw_bind_point_transforms", None)
    if broadcaster is None or not transforms:
        return

    stamp = rospy.Time.now()
    for transform in transforms:
        transform.header.stamp = stamp
    broadcaster.sendTransform(transforms)
