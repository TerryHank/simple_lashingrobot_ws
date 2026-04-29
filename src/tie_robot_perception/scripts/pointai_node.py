#!/usr/bin/env python3
"""ROS executable wrapper for the structured pointAI node package."""
import os
import sys


PERCEPTION_PACKAGE_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "src")
)
if os.path.isdir(PERCEPTION_PACKAGE_DIR) and PERCEPTION_PACKAGE_DIR not in sys.path:
    sys.path.insert(0, PERCEPTION_PACKAGE_DIR)

from tie_robot_perception.pointai.node import ImageProcessor, main  # noqa: E402,F401


if __name__ == "__main__":
    main()
