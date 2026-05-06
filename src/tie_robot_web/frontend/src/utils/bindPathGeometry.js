function toFiniteNumber(value) {
  const number = Number(value);
  return Number.isFinite(number) ? number : null;
}

function millimetersToMeters(value) {
  const number = toFiniteNumber(value);
  return number === null ? null : number / 1000.0;
}

function compareNumber(left, right) {
  if (left !== right) {
    return left - right;
  }
  return 0;
}

function compareGridPointByColumnIndex(left, right) {
  return compareNumber(left.col, right.col)
    || compareNumber(left.x, right.x)
    || compareNumber(left.y, right.y)
    || compareNumber(left.z, right.z)
    || compareNumber(left.globalIdx, right.globalIdx);
}

function compareGridPointByRowIndex(left, right) {
  return compareNumber(left.row, right.row)
    || compareNumber(left.y, right.y)
    || compareNumber(left.x, right.x)
    || compareNumber(left.z, right.z)
    || compareNumber(left.globalIdx, right.globalIdx);
}

function makePointKey(point, fallbackIndex) {
  if (point.globalIdx > 0) {
    return `global:${point.globalIdx}`;
  }
  return [
    "coord",
    point.row,
    point.col,
    point.x.toFixed(6),
    point.y.toFixed(6),
    point.z.toFixed(6),
    fallbackIndex,
  ].join(":");
}

function normalizeBindGridPoint(rawPoint, fallbackIndex) {
  const x = millimetersToMeters(rawPoint?.world_x ?? rawPoint?.x);
  const y = millimetersToMeters(rawPoint?.world_y ?? rawPoint?.y);
  const z = millimetersToMeters(rawPoint?.world_z ?? rawPoint?.z);
  if (x === null || y === null || z === null) {
    return null;
  }

  const row = toFiniteNumber(rawPoint?.global_row ?? rawPoint?.planning_global_row);
  const col = toFiniteNumber(rawPoint?.global_col ?? rawPoint?.planning_global_col);
  const globalIdx = toFiniteNumber(rawPoint?.global_idx ?? rawPoint?.idx);
  return {
    globalIdx: globalIdx === null ? -1 : globalIdx,
    row: row === null ? -1 : row,
    col: col === null ? -1 : col,
    x,
    y,
    z,
    fallbackIndex,
  };
}

function collectRawAreaPoints(areas) {
  const bindPathAreas = Array.isArray(areas) ? areas : [];
  const rawPoints = [];
  bindPathAreas.forEach((area) => {
    const groups = Array.isArray(area?.groups) ? area.groups : [];
    groups.forEach((group) => {
      const groupPoints = Array.isArray(group?.points) ? group.points : [];
      rawPoints.push(...groupPoints);
    });
  });
  return rawPoints;
}

function collectBindGroupPoints(group, fallbackStartIndex = 0) {
  const rawPoints = Array.isArray(group?.points) ? group.points : [];
  const points = [];
  const seenKeys = new Set();
  let fallbackIndex = fallbackStartIndex;
  rawPoints.forEach((rawPoint) => {
    const point = normalizeBindGridPoint(rawPoint, fallbackIndex);
    fallbackIndex += 1;
    if (!point) {
      return;
    }
    const key = makePointKey(point, point.fallbackIndex);
    if (seenKeys.has(key)) {
      return;
    }
    seenKeys.add(key);
    points.push(point);
  });
  return points;
}

function pushSegment(positions, startPoint, endPoint, zOffsetMeters = 0) {
  positions.push(
    startPoint.x,
    startPoint.y,
    startPoint.z + zOffsetMeters,
    endPoint.x,
    endPoint.y,
    endPoint.z + zOffsetMeters,
  );
}

function pointCellKey(row, col) {
  return `${row}:${col}`;
}

function buildGroupPerimeterPoints(points) {
  const validGridPoints = points.filter((point) => point.row >= 0 && point.col >= 0);
  const rows = [...new Set(validGridPoints.map((point) => point.row))].sort(compareNumber);
  const cols = [...new Set(validGridPoints.map((point) => point.col))].sort(compareNumber);
  if (rows.length !== 2 || cols.length !== 2) {
    return null;
  }

  const pointsByCell = new Map();
  validGridPoints.forEach((point) => {
    pointsByCell.set(pointCellKey(point.row, point.col), point);
  });

  const corners = [
    pointsByCell.get(pointCellKey(rows[0], cols[0])),
    pointsByCell.get(pointCellKey(rows[0], cols[1])),
    pointsByCell.get(pointCellKey(rows[1], cols[1])),
    pointsByCell.get(pointCellKey(rows[1], cols[0])),
  ];
  return corners.every(Boolean) ? corners : null;
}

export function collectBindPathGridPoints(areas, gridPoints = []) {
  const authoritativeGridPoints = Array.isArray(gridPoints) && gridPoints.length > 0
    ? gridPoints
    : collectRawAreaPoints(areas);
  const points = [];
  const seenKeys = new Set();
  let fallbackIndex = 0;

  authoritativeGridPoints.forEach((rawPoint) => {
    const point = normalizeBindGridPoint(rawPoint, fallbackIndex);
    fallbackIndex += 1;
    if (!point) {
      return;
    }

    const key = makePointKey(point, point.fallbackIndex);
    if (seenKeys.has(key)) {
      return;
    }
    seenKeys.add(key);
    points.push(point);
  });

  return points;
}

export function buildBindPathPointPositions(areas, gridPoints = []) {
  return collectBindPathGridPoints(areas, gridPoints).flatMap((point) => [point.x, point.y, point.z]);
}

export function buildBindGridLineSegmentPositions(areas, { axis, gridPoints = [] } = {}) {
  const groupKey = axis === "column" ? "col" : "row";
  const sortPoints = axis === "column" ? compareGridPointByRowIndex : compareGridPointByColumnIndex;
  const groupsByAxis = new Map();

  collectBindPathGridPoints(areas, gridPoints).forEach((point) => {
    const key = point[groupKey];
    if (key < 0) {
      return;
    }
    if (!groupsByAxis.has(key)) {
      groupsByAxis.set(key, []);
    }
    groupsByAxis.get(key).push(point);
  });

  const positions = [];
  [...groupsByAxis.keys()].sort(compareNumber).forEach((key) => {
    const points = groupsByAxis.get(key).sort(sortPoints);
    if (points.length < 2) {
      return;
    }
    for (let index = 1; index < points.length; index += 1) {
      const startPoint = points[index - 1];
      const endPoint = points[index];
      positions.push(
        startPoint.x,
        startPoint.y,
        startPoint.z,
        endPoint.x,
        endPoint.y,
        endPoint.z,
      );
    }
  });

  return positions;
}

export function buildBindGroupLineSegmentPositions(areas, { zOffsetMeters = 0 } = {}) {
  const bindPathAreas = Array.isArray(areas) ? areas : [];
  const positions = [];
  let fallbackStartIndex = 0;

  bindPathAreas.forEach((area) => {
    const groups = Array.isArray(area?.groups) ? area.groups : [];
    groups.forEach((group) => {
      const points = collectBindGroupPoints(group, fallbackStartIndex);
      fallbackStartIndex += points.length;
      if (points.length < 2) {
        return;
      }
      if (points.length === 2) {
        pushSegment(positions, points[0], points[1], zOffsetMeters);
        return;
      }

      const perimeterPoints = buildGroupPerimeterPoints(points);
      if (perimeterPoints) {
        for (let index = 0; index < perimeterPoints.length; index += 1) {
          pushSegment(
            positions,
            perimeterPoints[index],
            perimeterPoints[(index + 1) % perimeterPoints.length],
            zOffsetMeters,
          );
        }
        return;
      }

      const sortedPoints = [...points].sort((left, right) => (
        compareNumber(left.row, right.row)
        || compareNumber(left.col, right.col)
        || compareNumber(left.globalIdx, right.globalIdx)
      ));
      for (let index = 1; index < sortedPoints.length; index += 1) {
        pushSegment(positions, sortedPoints[index - 1], sortedPoints[index], zOffsetMeters);
      }
    });
  });

  return positions;
}
