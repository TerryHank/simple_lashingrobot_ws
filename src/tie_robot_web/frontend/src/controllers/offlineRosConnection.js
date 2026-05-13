export function createOfflineRosConnection() {
  return {
    isReady() {
      return false;
    },
    getResources() {
      return {};
    },
    updateDisplayedImageSubscription(topic) {
      return { success: false, changed: false, topic };
    },
    updateLogSubscription(topicId) {
      return { success: false, changed: false, topicId };
    },
    updatePointCloudSubscription({ enabled = false, source = "filteredWorldCoord" } = {}) {
      return { success: false, changed: false, enabled: Boolean(enabled), source };
    },
    updateImageHoverCoordinateSubscription({ enabled = false } = {}) {
      return { success: false, changed: false, enabled: Boolean(enabled) };
    },
  };
}
