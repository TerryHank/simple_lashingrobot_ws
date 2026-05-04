export const TCP_WORKSPACE_BOUNDARY_MM = Object.freeze({
  x: Object.freeze({ min: 0, max: 380 }),
  y: Object.freeze({ min: 0, max: 330 }),
  z: 0,
});

export function buildTcpWorkspaceBoundaryPointsMm(bounds = TCP_WORKSPACE_BOUNDARY_MM) {
  const xMin = Number(bounds?.x?.min);
  const xMax = Number(bounds?.x?.max);
  const yMin = Number(bounds?.y?.min);
  const yMax = Number(bounds?.y?.max);
  const z = Number(bounds?.z);
  if (![xMin, xMax, yMin, yMax, z].every(Number.isFinite)) {
    return [];
  }
  return [
    { x: xMin, y: yMin, z },
    { x: xMax, y: yMin, z },
    { x: xMax, y: yMax, z },
    { x: xMin, y: yMax, z },
  ];
}

export function normalizeCameraProjection(cameraInfo) {
  const matrix = Array.isArray(cameraInfo?.K) && cameraInfo.K.length >= 9
    ? cameraInfo.K
    : Array.isArray(cameraInfo?.P) && cameraInfo.P.length >= 12
      ? [cameraInfo.P[0], 0, cameraInfo.P[2], 0, cameraInfo.P[5], cameraInfo.P[6], 0, 0, 1]
      : null;
  const width = Number(cameraInfo?.width);
  const height = Number(cameraInfo?.height);
  if (!matrix || !Number.isFinite(width) || !Number.isFinite(height) || width <= 0 || height <= 0) {
    return null;
  }
  const fx = Number(matrix[0]);
  const fy = Number(matrix[4]);
  const cx = Number(matrix[2]);
  const cy = Number(matrix[5]);
  if (![fx, fy, cx, cy].every(Number.isFinite) || fx === 0 || fy === 0) {
    return null;
  }
  return { width, height, fx, fy, cx, cy };
}

export function projectCameraPointMetersToImagePixel(cameraPointMeters, cameraInfo) {
  const projection = normalizeCameraProjection(cameraInfo);
  const z = Number(cameraPointMeters?.z);
  if (!projection || !Number.isFinite(z) || z <= 0.001) {
    return null;
  }
  const normalizedX = Number(cameraPointMeters?.x) / z;
  const normalizedY = Number(cameraPointMeters?.y) / z;
  if (![normalizedX, normalizedY].every(Number.isFinite)) {
    return null;
  }
  const x = projection.fx * normalizedX + projection.cx;
  const y = projection.fy * normalizedY + projection.cy;
  if (![x, y].every(Number.isFinite)) {
    return null;
  }
  return {
    x,
    y,
    inside: x >= 0 && x <= projection.width - 1 && y >= 0 && y <= projection.height - 1,
  };
}
