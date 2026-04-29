#!/usr/bin/env python3
"""Export a compact ROS visual snapshot for offline perception replay."""

import argparse
import json
import os
import re
import sys
from datetime import datetime
from pathlib import Path

import numpy as np
import rosbag
import rospy
import rostopic
from sensor_msgs.msg import Image

try:
    import cv2
except ImportError:  # pragma: no cover - preview images are optional.
    cv2 = None


DEFAULT_TOPICS = [
    "/Scepter/color/image_raw",
    "/Scepter/color/camera_info",
    "/Scepter/depth/image_raw",
    "/Scepter/depth/camera_info",
    "/Scepter/ir/image_raw",
    "/Scepter/ir/camera_info",
    "/Scepter/transformedColor/image_raw",
    "/Scepter/transformedDepth/image_raw",
    "/Scepter/worldCoord/raw_world_coord",
    "/Scepter/worldCoord/world_coord",
    "/Scepter/worldCoord/camera_info",
    "/pointAI/result_image_raw",
    "/pointAI/line_image",
    "/perception/lashing/result_image",
    "/perception/lashing/points_camera",
    "/perception/lashing/workspace/quad_pixels",
    "/coordinate_point",
    "/cabin/cabin_data_upload",
    "/moduan/moduan_gesture_data",
    "/robot/binding_gun_status",
    "/robot/moduan_status",
    "/tf",
    "/tf_static",
]


ENCODINGS = {
    "mono8": (np.uint8, 1),
    "8uc1": (np.uint8, 1),
    "8uc3": (np.uint8, 3),
    "bgr8": (np.uint8, 3),
    "rgb8": (np.uint8, 3),
    "bgra8": (np.uint8, 4),
    "rgba8": (np.uint8, 4),
    "mono16": (np.uint16, 1),
    "16uc1": (np.uint16, 1),
    "16uc3": (np.uint16, 3),
    "32fc1": (np.float32, 1),
    "32fc3": (np.float32, 3),
    "64fc1": (np.float64, 1),
}


def parse_args():
    parser = argparse.ArgumentParser(
        description="Capture one message per visual/robot-state topic into a bag, "
        "PNG previews, NPY arrays and metadata."
    )
    parser.add_argument(
        "--output-dir",
        default="docs/releases/slam_v30/visual_modalities",
        help="Directory for the exported bag, previews and metadata.",
    )
    parser.add_argument(
        "--bag-name",
        default="slam_v30_visual_modalities.bag",
        help="Output bag filename under --output-dir.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=5.0,
        help="Seconds to wait for each topic before marking it unavailable.",
    )
    parser.add_argument(
        "--topic",
        action="append",
        default=[],
        help="Additional topic to export. Can be passed multiple times.",
    )
    parser.add_argument(
        "--no-default-topics",
        action="store_true",
        help="Only export topics passed through --topic.",
    )
    return parser.parse_args()


def safe_topic_name(topic):
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", topic.strip("/")) or "root"


def msg_stamp(msg):
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is not None and stamp.to_sec() > 0:
        return stamp
    return rospy.Time.now()


def image_to_array(msg):
    encoding = msg.encoding.lower()
    if encoding not in ENCODINGS:
        raise ValueError("unsupported image encoding: {}".format(msg.encoding))

    dtype, channels = ENCODINGS[encoding]
    item_size = np.dtype(dtype).itemsize
    data = np.frombuffer(msg.data, dtype=dtype)
    row_items = msg.step // item_size

    if msg.is_bigendian != (sys.byteorder == "big"):
        data = data.byteswap()

    if channels == 1:
        array = data.reshape((msg.height, row_items))[:, : msg.width]
    else:
        row_pixels = row_items // channels
        array = data.reshape((msg.height, row_pixels, channels))[:, : msg.width, :]
    return array


def finite_stats(array):
    scalar = array
    if array.ndim == 3:
        scalar = array[..., 2] if array.shape[2] >= 3 else array[..., 0]
    finite = np.isfinite(scalar)
    if not finite.any():
        return {"finite_count": 0, "min": None, "max": None, "mean": None}
    values = scalar[finite]
    return {
        "finite_count": int(values.size),
        "min": float(values.min()),
        "max": float(values.max()),
        "mean": float(values.mean()),
    }


def normalize_to_u8(array):
    scalar = array
    if array.ndim == 3:
        scalar = array[..., 2] if array.shape[2] >= 3 else array[..., 0]

    scalar = scalar.astype(np.float32, copy=False)
    finite = np.isfinite(scalar)
    if not finite.any():
        return np.zeros(scalar.shape, dtype=np.uint8)

    values = scalar[finite]
    low = np.percentile(values, 1.0)
    high = np.percentile(values, 99.0)
    if high <= low:
        high = low + 1.0
    normalized = np.clip((scalar - low) / (high - low), 0.0, 1.0)
    normalized[~finite] = 0.0
    return (normalized * 255.0).astype(np.uint8)


def image_preview(array, encoding):
    encoding = encoding.lower()
    if encoding == "rgb8" and cv2 is not None:
        return cv2.cvtColor(array, cv2.COLOR_RGB2BGR)
    if encoding == "rgba8" and cv2 is not None:
        return cv2.cvtColor(array, cv2.COLOR_RGBA2BGRA)
    if encoding in ("bgr8", "bgra8"):
        return array
    if array.dtype == np.uint8 and array.ndim in (2, 3):
        return array

    preview = normalize_to_u8(array)
    if cv2 is not None:
        return cv2.applyColorMap(preview, cv2.COLORMAP_TURBO)
    return preview


def write_preview(path, array, encoding):
    if cv2 is None:
        return False
    preview = image_preview(array, encoding)
    return bool(cv2.imwrite(str(path), preview))


def write_text(path, msg):
    text = str(msg)
    if len(text) > 400000:
        text = text[:400000] + "\n\n... truncated by export_visual_modalities_snapshot.py\n"
    path.write_text(text, encoding="utf-8")


def resolve_topic(topic):
    msg_class, real_topic, _ = rostopic.get_topic_class(topic, blocking=False)
    return msg_class, real_topic or topic


def main():
    args = parse_args()
    out_dir = Path(args.output_dir)
    image_dir = out_dir / "images"
    array_dir = out_dir / "arrays"
    message_dir = out_dir / "messages"
    for directory in (out_dir, image_dir, array_dir, message_dir):
        directory.mkdir(parents=True, exist_ok=True)

    topics = [] if args.no_default_topics else list(DEFAULT_TOPICS)
    topics.extend(args.topic)
    topics = list(dict.fromkeys(topics))

    rospy.init_node("slam_v30_visual_modalities_exporter", anonymous=True)
    bag_path = out_dir / args.bag_name

    records = []
    with rosbag.Bag(str(bag_path), "w", compression="bz2") as bag:
        for topic in topics:
            msg_class, real_topic = resolve_topic(topic)
            record = {
                "topic": topic,
                "resolved_topic": real_topic,
                "captured": False,
            }

            if msg_class is None:
                record["error"] = "topic type unavailable"
                records.append(record)
                print("[skip] {}: topic type unavailable".format(topic))
                continue

            record["type"] = "{}.{}".format(msg_class.__module__, msg_class.__name__)
            try:
                msg = rospy.wait_for_message(real_topic, msg_class, timeout=args.timeout)
            except Exception as exc:  # rospy.ROSException has no stable public fields.
                record["error"] = str(exc)
                records.append(record)
                print("[miss] {}: {}".format(topic, exc))
                continue

            stamp = msg_stamp(msg)
            bag.write(real_topic, msg, t=stamp)
            record["captured"] = True
            record["stamp"] = stamp.to_sec()

            base = safe_topic_name(real_topic)
            if isinstance(msg, Image):
                try:
                    array = image_to_array(msg)
                    array_path = array_dir / "{}.npy".format(base)
                    np.save(str(array_path), array)
                    record["array_path"] = str(array_path.relative_to(out_dir))
                    record["encoding"] = msg.encoding
                    record["height"] = int(msg.height)
                    record["width"] = int(msg.width)
                    record["dtype"] = str(array.dtype)
                    record["shape"] = [int(v) for v in array.shape]
                    record["stats"] = finite_stats(array)

                    preview_path = image_dir / "{}.png".format(base)
                    if write_preview(preview_path, array, msg.encoding):
                        record["preview_path"] = str(preview_path.relative_to(out_dir))
                except Exception as exc:
                    record["image_export_error"] = str(exc)
            else:
                message_path = message_dir / "{}.txt".format(base)
                write_text(message_path, msg)
                record["message_path"] = str(message_path.relative_to(out_dir))

            records.append(record)
            print("[ok] {} ({})".format(real_topic, record["type"]))

    metadata = {
        "release": "slam/v30",
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "ros_master_uri": os.environ.get("ROS_MASTER_URI"),
        "bag_path": str(bag_path),
        "bag_name": bag_path.name,
        "topic_count": len(records),
        "captured_count": sum(1 for record in records if record.get("captured")),
        "records": records,
    }
    metadata_path = out_dir / "metadata.json"
    metadata_path.write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )

    print("wrote {}".format(bag_path))
    print("wrote {}".format(metadata_path))
    print(
        "captured {}/{} topics".format(
            metadata["captured_count"], metadata["topic_count"]
        )
    )


if __name__ == "__main__":
    main()
