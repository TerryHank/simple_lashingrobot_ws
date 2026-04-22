export class SceneAdapter {
  normalizePointCloud(message, context = {}) {
    return {
      message,
      context,
    };
  }

  normalizeTiePoints(message, context = {}) {
    return {
      message,
      context,
    };
  }

  normalizePlanningMarkers(message, context = {}) {
    return {
      message,
      context,
    };
  }
}
