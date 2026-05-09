export const STATUS_MONITORS = [
  { id: "ros", label: "ROS连接", kind: "connection" },
  {
    id: "chassis",
    label: "索驱",
    diagnosticHardwareId: "tie_robot/chassis_driver",
    diagnosticStaleMs: 12000,
  },
  {
    id: "moduan",
    label: "末端",
    diagnosticHardwareId: "tie_robot/moduan_driver",
    diagnosticStaleMs: 12000,
  },
  { id: "visual", label: "视觉", diagnosticHardwareId: "tie_robot/visual_algorithm" },
];
