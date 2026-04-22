export const STATUS_MONITORS = [
  { id: "ros", label: "ROS", kind: "connection" },
  { id: "chassis", label: "索驱", topic: "/robot/chassis_status", messageType: "std_msgs/Float32" },
  { id: "moduan", label: "末端", topic: "/robot/moduan_status", messageType: "std_msgs/Float32" },
  { id: "bindingGun", label: "绑扎枪", topic: "/robot/binding_gun_status", messageType: "std_msgs/Float32" },
];
