export const IMAGE_TOPIC_OPTIONS = [
  {
    id: "/Scepter/ir/image_raw",
    label: "红外图像",
    overlayCompatible: true,
  },
  {
    id: "/pointAI/result_image_raw",
    label: "识别结果图",
    overlayCompatible: false,
  },
  {
    id: "/Scepter/color/image_raw",
    label: "彩色图像",
    overlayCompatible: false,
  },
  {
    id: "/Scepter/depth/image_raw",
    label: "深度图像",
    overlayCompatible: false,
  },
  {
    id: "/Scepter/worldCoord/world_coord",
    label: "滤波世界点云图",
    overlayCompatible: false,
  },
  {
    id: "/Scepter/worldCoord/raw_world_coord",
    label: "原始世界点云图",
    overlayCompatible: false,
  },
];

export const DEFAULT_IMAGE_TOPIC = "/Scepter/ir/image_raw";

export function getImageTopicLabel(topicId) {
  return IMAGE_TOPIC_OPTIONS.find((option) => option.id === topicId)?.label || topicId;
}

export function isOverlayCompatibleImageTopic(topicId) {
  return IMAGE_TOPIC_OPTIONS.find((option) => option.id === topicId)?.overlayCompatible ?? false;
}
