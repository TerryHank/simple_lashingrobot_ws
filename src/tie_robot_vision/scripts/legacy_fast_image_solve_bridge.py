#!/usr/bin/env python3
"""Expose 20260403 fast_image_solve services backed by tie_robot_vision."""

import rospy

from tie_robot_vision.srv import ProcessImage as VisionProcessImage
from tie_robot_vision.srv import ProcessImageRequest as VisionProcessImageRequest

try:
    from fast_image_solve.msg import PointCoords as LegacyPointCoords
    from fast_image_solve.srv import ProcessImage, ProcessImageResponse
except ImportError as exc:  # pragma: no cover - only hit when used outside legacy workspace.
    raise SystemExit(
        "legacy_fast_image_solve_bridge requires the old fast_image_solve package. "
        "Use vision_stack.launch in a pure tie_robot_vision workspace."
    ) from exc


def copy_point_to_legacy(point):
    legacy_point = LegacyPointCoords()
    legacy_point.idx = int(getattr(point, "idx", 0))
    legacy_point.Pix_coord = list(getattr(point, "Pix_coord", [0, 0]))[:2]
    legacy_point.World_coord = list(getattr(point, "World_coord", [0.0, 0.0, 0.0]))[:3]
    legacy_point.Angle = float(getattr(point, "Angle", 0.0))
    legacy_point.is_shuiguan = bool(getattr(point, "is_shuiguan", False))
    return legacy_point


class LegacyFastImageSolveBridge:
    def __init__(self):
        self.backend_service_name = rospy.get_param(
            "~backend_service_name",
            "/tie_robot_vision/process_image",
        )
        self.backend_request_mode = int(rospy.get_param(
            "~backend_request_mode",
            VisionProcessImageRequest.MODE_EXECUTION_REFINE,
        ))
        public_services = rospy.get_param(
            "~public_service_names",
            ["/pointAI/process_image", "/Moduan/process_image"],
        )
        if isinstance(public_services, str):
            public_services = [item.strip() for item in public_services.split(",") if item.strip()]
        if not public_services:
            raise ValueError("~public_service_names must contain at least one service name")

        self.backend = rospy.ServiceProxy(self.backend_service_name, VisionProcessImage)
        self.servers = [
            rospy.Service(service_name, ProcessImage, self.handle_legacy_request)
            for service_name in public_services
        ]
        rospy.loginfo(
            "tie_robot_vision legacy bridge started: %s -> %s",
            ", ".join(public_services),
            self.backend_service_name,
        )

    def handle_legacy_request(self, _request):
        try:
            backend_request = VisionProcessImageRequest()
            backend_request.request_mode = self.backend_request_mode
            response = self.backend(backend_request)
        except Exception as exc:
            rospy.logerr("tie_robot_vision legacy bridge backend call failed: %s", exc)
            return ProcessImageResponse(count=0, PointCoordinatesArray=[])

        points = [
            copy_point_to_legacy(point)
            for point in getattr(response, "PointCoordinatesArray", [])
        ]
        legacy_response = ProcessImageResponse()
        legacy_response.count = int(getattr(response, "count", len(points)))
        legacy_response.PointCoordinatesArray = points
        return legacy_response


def main():
    rospy.init_node("tie_robot_vision_legacy_fast_image_solve_bridge")
    LegacyFastImageSolveBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
