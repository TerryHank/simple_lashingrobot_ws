#!/usr/bin/env python3
import re
import unittest
from pathlib import Path


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
PROTOCOL_CPP = (
    WORKSPACE_ROOT
    / "src"
    / "tie_robot_hw"
    / "src"
    / "driver"
    / "linear_module_protocol.cpp"
)


class LinearModuleProtocolAngleScalingTest(unittest.TestCase):
    def test_point_payload_scales_xyz_but_keeps_rotation_angle_raw(self):
        source = PROTOCOL_CPP.read_text(encoding="utf-8")

        self.assertRegex(
            source,
            r"int32_t\s+scaled\s*=\s*static_cast<int32_t>\(value\s*\*\s*100\.0f\)",
            "x/y/z 坐标仍应按旧线模协议乘 100 后写入寄存器",
        )
        self.assertRegex(
            source,
            r"int32_t\s+raw\s*=\s*static_cast<int32_t>\(value\)",
            "旋转角应按旧 Set_Motor_Angle 协议原样写入 int32，不乘 100",
        )

        payload_body = re.search(
            r"buildPointRegisterPayload\(const LinearModulePoint& point\)\s*\{(?P<body>.*?)\n\}",
            source,
            re.DOTALL,
        )
        self.assertIsNotNone(payload_body, "未找到 buildPointRegisterPayload 实现")
        body = payload_body.group("body")

        for field in ("x_mm", "y_mm", "z_mm"):
            self.assertIn(f"lowerScaledWord(point.{field})", body)
            self.assertIn(f"upperScaledWord(point.{field})", body)
        self.assertIn("lowerRawWord(point.angle_deg)", body)
        self.assertIn("upperRawWord(point.angle_deg)", body)
        self.assertNotIn("lowerScaledWord(point.angle_deg)", body)
        self.assertNotIn("upperScaledWord(point.angle_deg)", body)


if __name__ == "__main__":
    unittest.main()
