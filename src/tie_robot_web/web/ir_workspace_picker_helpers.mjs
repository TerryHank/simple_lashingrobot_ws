export function resolveRosbridgeUrl({
  pathname = '/',
  savedPreferences = null,
  currentHost = 'localhost',
} = {}) {
  if (pathname && pathname.length > 1) {
    try {
      const decoded = decodeURIComponent(pathname.substring(1));
      if (decoded.startsWith('ws://') || decoded.startsWith('wss://')) {
        return decoded;
      }
    } catch {
      // ignore invalid encoded pathname and fall through
    }
  }

  if (currentHost) {
    return `ws://${currentHost}:9090`;
  }

  if (savedPreferences && savedPreferences.ip) {
    return `ws://${savedPreferences.ip}:${savedPreferences.port || '9090'}`;
  }

  return `ws://${currentHost || 'localhost'}:9090`;
}

export function clampImagePixel(point, imageWidth, imageHeight) {
  return {
    x: Math.min(Math.max(Math.round(point.x), 0), Math.max(imageWidth - 1, 0)),
    y: Math.min(Math.max(Math.round(point.y), 0), Math.max(imageHeight - 1, 0)),
  };
}

export function mapCanvasClickToImagePixel({
  clientX,
  clientY,
  rect,
  imageWidth,
  imageHeight,
}) {
  const width = Math.max(rect?.width || 0, 1);
  const height = Math.max(rect?.height || 0, 1);
  const relativeX = ((clientX - (rect?.left || 0)) / width) * imageWidth;
  const relativeY = ((clientY - (rect?.top || 0)) / height) * imageHeight;
  return clampImagePixel({ x: relativeX, y: relativeY }, imageWidth, imageHeight);
}

export function buildWorkspaceQuadPayload(points) {
  if (!Array.isArray(points) || points.length !== 4) {
    throw new Error('Workspace quad requires exactly four points');
  }

  return points.flatMap((point) => [
    Math.round(point.x),
    Math.round(point.y),
  ]);
}

export function parseWorkspaceQuadPayload(payload) {
  if (!Array.isArray(payload) || payload.length !== 8) {
    throw new Error('Workspace quad payload requires exactly eight values');
  }

  const points = [];
  for (let index = 0; index < payload.length; index += 2) {
    points.push({
      x: Number(payload[index]),
      y: Number(payload[index + 1]),
    });
  }
  return points;
}

export function findDraggablePointIndex(points, candidatePoint, hitRadius = 12) {
  if (!Array.isArray(points) || points.length === 0 || !candidatePoint) {
    return -1;
  }

  let bestIndex = -1;
  let bestDistanceSquared = Number.POSITIVE_INFINITY;
  const hitRadiusSquared = hitRadius * hitRadius;

  points.forEach((point, index) => {
    const dx = Number(point.x) - Number(candidatePoint.x);
    const dy = Number(point.y) - Number(candidatePoint.y);
    const distanceSquared = dx * dx + dy * dy;
    if (distanceSquared <= hitRadiusSquared && distanceSquared < bestDistanceSquared) {
      bestDistanceSquared = distanceSquared;
      bestIndex = index;
    }
  });

  return bestIndex;
}

export function replaceWorkspacePoint(points, index, nextPoint) {
  if (!Array.isArray(points)) {
    return [];
  }

  return points.map((point, pointIndex) => {
    if (pointIndex !== index) {
      return point;
    }
    return {
      x: Number(nextPoint.x),
      y: Number(nextPoint.y),
    };
  });
}

export function findPercentileIntensity(bytes, percentile) {
  if (!(bytes instanceof Uint8ClampedArray) || bytes.length === 0) {
    return 0;
  }

  const histogram = new Uint32Array(256);
  for (let index = 0; index < bytes.length; index += 1) {
    histogram[bytes[index]] += 1;
  }

  const clampedPercentile = Math.min(Math.max(percentile, 0), 100);
  const targetCount = Math.max(Math.ceil((clampedPercentile / 100) * bytes.length), 1);
  let cumulative = 0;
  for (let intensity = 0; intensity < histogram.length; intensity += 1) {
    cumulative += histogram[intensity];
    if (cumulative >= targetCount) {
      return intensity;
    }
  }

  return 255;
}

export function stretchMono8Contrast(
  bytes,
  { lowClipPercent = 1, highClipPercent = 99 } = {},
) {
  if (!(bytes instanceof Uint8ClampedArray) || bytes.length === 0) {
    return new Uint8ClampedArray();
  }

  const low = findPercentileIntensity(bytes, lowClipPercent);
  const high = findPercentileIntensity(bytes, highClipPercent);
  if (high <= low) {
    return Uint8ClampedArray.from(bytes);
  }

  const stretched = new Uint8ClampedArray(bytes.length);
  const scale = 255 / (high - low);
  for (let index = 0; index < bytes.length; index += 1) {
    const normalized = Math.round((bytes[index] - low) * scale);
    stretched[index] = Math.min(Math.max(normalized, 0), 255);
  }
  return stretched;
}

export function applyGammaToMono8(bytes, gamma = 1.0) {
  if (!(bytes instanceof Uint8ClampedArray) || bytes.length === 0) {
    return new Uint8ClampedArray();
  }

  const safeGamma = Number.isFinite(gamma) && gamma > 0 ? gamma : 1.0;
  const corrected = new Uint8ClampedArray(bytes.length);
  for (let index = 0; index < bytes.length; index += 1) {
    corrected[index] = Math.round(255 * ((bytes[index] / 255) ** safeGamma));
  }
  return corrected;
}

export function equalizeMono8(bytes) {
  if (!(bytes instanceof Uint8ClampedArray) || bytes.length === 0) {
    return new Uint8ClampedArray();
  }

  const histogram = new Uint32Array(256);
  for (let index = 0; index < bytes.length; index += 1) {
    histogram[bytes[index]] += 1;
  }

  let cdfMin = 0;
  const cdf = new Uint32Array(256);
  let cumulative = 0;
  for (let intensity = 0; intensity < histogram.length; intensity += 1) {
    cumulative += histogram[intensity];
    cdf[intensity] = cumulative;
    if (cdfMin === 0 && histogram[intensity] > 0) {
      cdfMin = cumulative;
    }
  }

  if (cdfMin === bytes.length) {
    return Uint8ClampedArray.from(bytes);
  }

  const equalized = new Uint8ClampedArray(bytes.length);
  const denominator = Math.max(bytes.length - cdfMin, 1);
  for (let index = 0; index < bytes.length; index += 1) {
    equalized[index] = Math.round(((cdf[bytes[index]] - cdfMin) / denominator) * 255);
  }
  return equalized;
}

export function enhanceMono8(
  bytes,
  {
    mode = 'auto',
    gamma = 0.85,
    lowClipPercent = 1,
    highClipPercent = 99,
  } = {},
) {
  if (!(bytes instanceof Uint8ClampedArray) || bytes.length === 0) {
    return new Uint8ClampedArray();
  }

  if (mode === 'raw') {
    return Uint8ClampedArray.from(bytes);
  }

  let enhanced = stretchMono8Contrast(bytes, { lowClipPercent, highClipPercent });
  if (mode === 'strong') {
    enhanced = equalizeMono8(enhanced);
  }
  return applyGammaToMono8(enhanced, gamma);
}
