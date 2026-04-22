export function clampImagePixel(point, imageWidth, imageHeight) {
  return {
    x: Math.min(Math.max(Math.round(point.x), 0), Math.max(imageWidth - 1, 0)),
    y: Math.min(Math.max(Math.round(point.y), 0), Math.max(imageHeight - 1, 0)),
  };
}

export function mapCanvasClickToImagePixel({ clientX, clientY, rect, imageWidth, imageHeight }) {
  const width = Math.max(rect?.width || 0, 1);
  const height = Math.max(rect?.height || 0, 1);
  const relativeX = ((clientX - (rect?.left || 0)) / width) * imageWidth;
  const relativeY = ((clientY - (rect?.top || 0)) / height) * imageHeight;
  return clampImagePixel({ x: relativeX, y: relativeY }, imageWidth, imageHeight);
}

export function buildWorkspaceQuadPayload(points) {
  if (!Array.isArray(points) || points.length !== 4) {
    throw new Error("Workspace quad requires exactly four points");
  }
  return points.flatMap((point) => [Math.round(point.x), Math.round(point.y)]);
}

export function parseWorkspaceQuadPayload(payload) {
  if (!Array.isArray(payload) || payload.length !== 8) {
    throw new Error("Workspace quad payload requires exactly eight values");
  }
  const points = [];
  for (let index = 0; index < payload.length; index += 2) {
    points.push({ x: Number(payload[index]), y: Number(payload[index + 1]) });
  }
  return points;
}

export function findDraggablePointIndex(points, candidatePoint, hitRadius = 12) {
  if (!Array.isArray(points) || !candidatePoint) {
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
  return (points || []).map((point, pointIndex) => {
    if (pointIndex !== index) {
      return point;
    }
    return { x: Number(nextPoint.x), y: Number(nextPoint.y) };
  });
}

export function decodeImageBytes(data) {
  if (Array.isArray(data)) {
    return Uint8ClampedArray.from(data);
  }
  if (typeof data === "string") {
    const binary = atob(data);
    const bytes = new Uint8ClampedArray(binary.length);
    for (let index = 0; index < binary.length; index += 1) {
      bytes[index] = binary.charCodeAt(index);
    }
    return bytes;
  }
  if (data instanceof Uint8Array || data instanceof Uint8ClampedArray) {
    return new Uint8ClampedArray(data);
  }
  if (data?.buffer instanceof ArrayBuffer) {
    return new Uint8ClampedArray(data.buffer);
  }
  return null;
}

export function findPercentileIntensity(bytes, percentile) {
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

export function stretchMono8Contrast(bytes, { lowClipPercent = 1, highClipPercent = 99 } = {}) {
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
  const safeGamma = Number.isFinite(gamma) && gamma > 0 ? gamma : 1.0;
  const corrected = new Uint8ClampedArray(bytes.length);
  for (let index = 0; index < bytes.length; index += 1) {
    corrected[index] = Math.round(255 * ((bytes[index] / 255) ** safeGamma));
  }
  return corrected;
}

export function equalizeMono8(bytes) {
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

export function enhanceMono8(bytes, { mode = "auto", gamma = 0.85 } = {}) {
  if (mode === "raw") {
    return Uint8ClampedArray.from(bytes);
  }
  let enhanced = stretchMono8Contrast(bytes);
  if (mode === "strong") {
    enhanced = equalizeMono8(enhanced);
  }
  return applyGammaToMono8(enhanced, gamma);
}

export function sensorImageToImageData(message, enhancementSettings) {
  const bytes = decodeImageBytes(message.data);
  const width = Number(message.width) || 0;
  const height = Number(message.height) || 0;
  const encoding = String(message.encoding || "").toLowerCase();
  if (!bytes || width <= 0 || height <= 0) {
    throw new Error("图像数据无效");
  }
  const rgba = new Uint8ClampedArray(width * height * 4);
  if (encoding.includes("mono8") || encoding.includes("8uc1")) {
    const displayBytes = enhanceMono8(bytes, enhancementSettings);
    for (let pixelIndex = 0; pixelIndex < width * height; pixelIndex += 1) {
      const value = displayBytes[pixelIndex] ?? 0;
      const rgbaIndex = pixelIndex * 4;
      rgba[rgbaIndex] = value;
      rgba[rgbaIndex + 1] = value;
      rgba[rgbaIndex + 2] = value;
      rgba[rgbaIndex + 3] = 255;
    }
    return new ImageData(rgba, width, height);
  }
  if (encoding.includes("rgb8") || encoding.includes("bgr8")) {
    for (let pixelIndex = 0; pixelIndex < width * height; pixelIndex += 1) {
      const sourceIndex = pixelIndex * 3;
      const rgbaIndex = pixelIndex * 4;
      const rgb =
        encoding.includes("bgr8")
          ? [bytes[sourceIndex + 2], bytes[sourceIndex + 1], bytes[sourceIndex]]
          : [bytes[sourceIndex], bytes[sourceIndex + 1], bytes[sourceIndex + 2]];
      rgba[rgbaIndex] = rgb[0] ?? 0;
      rgba[rgbaIndex + 1] = rgb[1] ?? 0;
      rgba[rgbaIndex + 2] = rgb[2] ?? 0;
      rgba[rgbaIndex + 3] = 255;
    }
    return new ImageData(rgba, width, height);
  }
  throw new Error(`暂不支持的图像编码: ${message.encoding}`);
}

export function decodeFloat32XYZImage(message, { sampleStep = 4, maxPoints = 24000 } = {}) {
  const bytes = decodeImageBytes(message?.data);
  const width = Number(message?.width) || 0;
  const height = Number(message?.height) || 0;
  if (!bytes || width <= 0 || height <= 0) {
    return { positions: new Float32Array(), count: 0 };
  }

  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  const step = Math.max(1, Number(sampleStep) || 1);
  const positions = [];
  let count = 0;

  for (let row = 0; row < height; row += step) {
    for (let col = 0; col < width; col += step) {
      const offset = (row * width + col) * 12;
      if (offset + 12 > view.byteLength) {
        continue;
      }
      const xMm = view.getFloat32(offset, true);
      const yMm = view.getFloat32(offset + 4, true);
      const zMm = view.getFloat32(offset + 8, true);
      if (!Number.isFinite(xMm) || !Number.isFinite(yMm) || !Number.isFinite(zMm)) {
        continue;
      }
      if (xMm === 0 && yMm === 0 && zMm === 0) {
        continue;
      }
      positions.push(xMm / 1000.0, yMm / 1000.0, zMm / 1000.0);
      count += 1;
      if (count >= maxPoints) {
        return { positions: Float32Array.from(positions), count };
      }
    }
  }

  return { positions: Float32Array.from(positions), count };
}
