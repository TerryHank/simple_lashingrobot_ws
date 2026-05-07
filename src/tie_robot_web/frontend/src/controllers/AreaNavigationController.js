const DEFAULT_LINEAR_MODULE_ZERO_SETTLE_MS = 800;

function normalizeNumber(value) {
  const numberValue = Number(value);
  return Number.isFinite(numberValue) ? numberValue : null;
}

function normalizeArea(area, fallbackOrdinal) {
  const pose = area?.cabin_pose;
  const x = normalizeNumber(pose?.x);
  const y = normalizeNumber(pose?.y);
  const z = normalizeNumber(pose?.z);
  if (x === null || y === null || z === null) {
    return null;
  }
  const areaIndex = normalizeNumber(area?.area_index);
  return {
    areaIndex: areaIndex === null ? fallbackOrdinal + 1 : Math.round(areaIndex),
    ordinal: fallbackOrdinal,
    pose: { x, y, z },
  };
}

function normalizeAreas(bindPath) {
  const areas = Array.isArray(bindPath?.areas) ? bindPath.areas : [];
  return areas.map((area, index) => normalizeArea(area, index)).filter(Boolean);
}

function defaultWaitAfterLinearModuleZero() {
  return new Promise((resolve) => {
    window.setTimeout(resolve, DEFAULT_LINEAR_MODULE_ZERO_SETTLE_MS);
  });
}

async function defaultLoadBindPath() {
  const response = await fetch("/api/planning/bind-path", { cache: "no-store" });
  let payload = null;
  try {
    payload = await response.json();
  } catch (_error) {
    payload = null;
  }
  if (!response.ok || !payload?.success) {
    throw new Error(payload?.message || "读取 pseudo_slam_bind_path.json 失败。");
  }
  return payload.bind_path || null;
}

export class AreaNavigationController {
  constructor({
    rosConnection,
    loadBindPath = defaultLoadBindPath,
    waitAfterLinearModuleZero = defaultWaitAfterLinearModuleZero,
    getCurrentCabinPosition = () => null,
    getCabinSpeed = () => 300,
    callbacks = {},
  }) {
    this.rosConnection = rosConnection;
    this.loadBindPath = loadBindPath;
    this.waitAfterLinearModuleZero = waitAfterLinearModuleZero;
    this.getCurrentCabinPosition = getCurrentCabinPosition;
    this.getCabinSpeed = getCabinSpeed;
    this.callbacks = callbacks;
    this.latestAreaProgress = null;
  }

  handleAreaProgressMessage(message) {
    this.latestAreaProgress = message || null;
  }

  async moveRelative(direction) {
    const normalizedDirection = Number(direction) < 0 ? -1 : 1;
    const resources = this.rosConnection.getResources();
    if (
      !resources?.cabinSingleMoveService ||
      !resources?.manualAreaTakeoverPublisher ||
      !resources?.moduanMoveZeroPublisher
    ) {
      return this.reportResult("ROS 还没连好，暂时不能切换工作区域。", "warn");
    }

    let bindPath = null;
    try {
      bindPath = await this.loadBindPath();
    } catch (error) {
      return this.reportResult(error?.message || "读取规划区域失败。", "error");
    }

    const areas = normalizeAreas(bindPath);
    if (areas.length === 0) {
      return this.reportResult("pseudo_slam_bind_path.json 没有可切换的工作区域。", "warn");
    }

    const currentOrdinal = this.resolveCurrentAreaOrdinal(areas);
    if (currentOrdinal < 0) {
      return this.reportResult("无法判断当前工作区域，请先确认索驱当前位置或等待区域进度上报。", "warn");
    }

    const targetOrdinal = currentOrdinal + normalizedDirection;
    if (targetOrdinal < 0) {
      return this.reportResult("已经是第一个区域，不能再切换到上一个区域。", "warn");
    }
    if (targetOrdinal >= areas.length) {
      return this.reportResult("已经是最后一个区域，不能再切换到下一个区域。", "warn");
    }

    const targetArea = areas[targetOrdinal];
    const takeoverResult = this.rosConnection.publishManualAreaTakeover();
    if (!takeoverResult?.success) {
      return this.reportResult(takeoverResult?.message || "人工接管信号发送失败，已阻止切换区域。", "error");
    }

    const zeroResult = this.rosConnection.publishModuanMoveZero();
    if (!zeroResult?.success) {
      return this.reportResult(zeroResult?.message || "线性模组回零信号发送失败，已阻止切换区域。", "error");
    }
    this.callbacks.onResultMessage?.("已接管自动执行链，线性模组正在回零，随后切换工作区域。");
    await this.waitAfterLinearModuleZero();

    const payload = { ...targetArea.pose, speed: this.resolveCabinSpeed() };
    const moveResult = await this.rosConnection.callCabinSingleMoveService(payload);
    if (!moveResult?.success) {
      return this.reportResult(moveResult?.message || "索驱切换工作区域失败。", "error");
    }

    const message =
      `已切换到区域${targetArea.areaIndex}: x=${Math.round(payload.x)}, ` +
      `y=${Math.round(payload.y)}, z=${Math.round(payload.z)}`;
    this.callbacks.onCabinMoved?.(payload, targetArea);
    return this.reportResult(moveResult.message || message, "success", {
      targetArea,
      payload,
    });
  }

  resolveCurrentAreaOrdinal(areas) {
    const progressAreaIndex = normalizeNumber(this.latestAreaProgress?.current_area_index);
    if (progressAreaIndex !== null) {
      const matchedArea = areas.find((area) => area.areaIndex === Math.round(progressAreaIndex));
      if (matchedArea) {
        return matchedArea.ordinal;
      }
    }
    return this.resolveNearestAreaOrdinal(areas);
  }

  resolveNearestAreaOrdinal(areas) {
    const position = this.getCurrentCabinPosition?.();
    const currentX = normalizeNumber(position?.x);
    const currentY = normalizeNumber(position?.y);
    if (currentX === null || currentY === null) {
      return -1;
    }

    let bestOrdinal = -1;
    let bestDistanceSq = Number.POSITIVE_INFINITY;
    for (const area of areas) {
      const dx = area.pose.x - currentX;
      const dy = area.pose.y - currentY;
      const distanceSq = dx * dx + dy * dy;
      if (distanceSq < bestDistanceSq) {
        bestDistanceSq = distanceSq;
        bestOrdinal = area.ordinal;
      }
    }
    return bestOrdinal;
  }

  resolveCabinSpeed() {
    const speed = Number(this.getCabinSpeed?.());
    return Number.isFinite(speed) && speed > 0 ? speed : 300;
  }

  reportResult(message, level = "info", extra = {}) {
    this.callbacks.onResultMessage?.(message);
    this.callbacks.onLog?.(message, level);
    return { success: level === "success", message, ...extra };
  }
}
