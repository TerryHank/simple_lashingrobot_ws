import { TOPICS } from "./topicRegistry.js";

export const LOG_TOPIC_OPTIONS = [
  {
    id: "all",
    label: "全部终端",
    topic: TOPICS.logs.all,
  },
  {
    id: "algorithm",
    label: "算法层",
    topic: TOPICS.logs.all,
    nodeNames: ["pointAINode", "scepter_world_coord_processor"],
  },
  {
    id: "cabin",
    label: "索驱",
    topic: TOPICS.logs.cabin,
    nodeNames: ["suoqu_driver_node", "suoquNode", "cabin_motion_controller"],
  },
  {
    id: "moduan",
    label: "线性模组",
    topic: TOPICS.logs.moduan,
    nodeNames: ["moduan_driver_node", "moduanNode", "moduan_motion_controller"],
  },
  {
    id: "camera",
    label: "相机驱动",
    topic: TOPICS.logs.camera,
    nodeNames: ["scepter_manager"],
  },
  {
    id: "bridge",
    label: "Web桥接",
    topic: TOPICS.logs.all,
    nodeNames: ["webActionBridgeNode", "web_action_bridge_node", "rosbridge_websocket", "rosapi", "system_log_mux"],
  },
];

export const DEFAULT_LOG_TOPIC = "all";

export function getLogTopicOption(topicId) {
  return LOG_TOPIC_OPTIONS.find((option) => option.id === topicId) || LOG_TOPIC_OPTIONS[0];
}

export function getLogTopicLabel(topicId) {
  return getLogTopicOption(topicId).label;
}

export function matchesLogTopicFilter(entry, topicId) {
  const option = getLogTopicOption(topicId);
  if (!option.nodeNames?.length) {
    return true;
  }
  return option.nodeNames.includes(entry?.nodeName || "");
}
