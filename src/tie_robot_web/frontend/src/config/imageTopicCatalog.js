import {
  FRONTEND_DIRECT_TOPIC_REGISTRY,
  TOPICS,
  getTopicRegistryEntry,
} from "./topicRegistry.js";

function getRegistryLabel(topicName, fallback) {
  return getTopicRegistryEntry(topicName)?.label || fallback || topicName;
}

export const IMAGE_TOPIC_OPTIONS = [
  {
    id: TOPICS.camera.irImage,
    label: getRegistryLabel(TOPICS.camera.irImage, "红外图像"),
    overlayCompatible: true,
  },
  {
    id: TOPICS.camera.colorImage,
    label: getRegistryLabel(TOPICS.camera.colorImage, "彩色图像"),
    overlayCompatible: false,
  },
  {
    id: TOPICS.camera.depthImage,
    label: getRegistryLabel(TOPICS.camera.depthImage, "深度图像"),
    overlayCompatible: false,
  },
  {
    id: TOPICS.camera.filteredWorldCoord,
    label: getRegistryLabel(TOPICS.camera.filteredWorldCoord, "滤波世界点云图"),
    overlayCompatible: false,
  },
  {
    id: TOPICS.camera.rawWorldCoord,
    label: getRegistryLabel(TOPICS.camera.rawWorldCoord, "原始世界点云图"),
    overlayCompatible: false,
  },
  {
    id: TOPICS.algorithm.resultImageRaw,
    label: getRegistryLabel(TOPICS.algorithm.resultImageRaw, "识别结果图"),
    overlayCompatible: true,
  },
];

export const DEFAULT_IMAGE_TOPIC = TOPICS.camera.irImage;
export const IMAGE_TOPIC_REGISTRY = FRONTEND_DIRECT_TOPIC_REGISTRY.filter((entry) => (
  entry.messageType === "sensor_msgs/Image"
));

export function getImageTopicLabel(topicId) {
  return IMAGE_TOPIC_OPTIONS.find((option) => option.id === topicId)?.label || topicId;
}

export function isOverlayCompatibleImageTopic(topicId) {
  return IMAGE_TOPIC_OPTIONS.find((option) => option.id === topicId)?.overlayCompatible ?? false;
}
