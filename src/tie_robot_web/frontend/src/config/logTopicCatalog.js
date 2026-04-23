export const LOG_TOPIC_OPTIONS = [
  {
    id: "all",
    label: "全部终端",
    topic: "/system_log/all",
  },
  {
    id: "algorithm",
    label: "算法层",
    topic: "/system_log/all",
    nodeNames: ["pointAINode", "stable_point_tf_broadcaster", "scepter_world_coord_processor"],
  },
  {
    id: "cabin",
    label: "索驱",
    topic: "/system_log/suoquNode",
    nodeNames: ["suoquNode"],
  },
  {
    id: "moduan",
    label: "线性模组",
    topic: "/system_log/moduanNode",
    nodeNames: ["moduanNode"],
  },
  {
    id: "camera",
    label: "相机驱动",
    topic: "/system_log/scepter_manager",
    nodeNames: ["scepter_manager"],
  },
  {
    id: "bridge",
    label: "Web桥接",
    topic: "/system_log/all",
    nodeNames: ["topicTransNode", "rosbridge_websocket", "rosapi", "system_log_mux"],
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
