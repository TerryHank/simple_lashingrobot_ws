import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { decodeFloat32XYZImage } from "../utils/irImageUtils.js";
import {
  TCP_WORKSPACE_BOUNDARY_MM,
  buildTcpWorkspaceBoundaryPlanesMm,
  normalizeTcpWorkspaceBoundaryMm,
  normalizeCameraProjection,
  projectCameraPointMetersToImagePixel,
} from "../utils/tcpWorkspaceOverlay.js";
import {
  buildBindGridLineSegmentPositions,
  buildBindGroupLineSegmentPositions,
  buildBindPathPointPositions,
  buildJumpBindPointPositions,
} from "../utils/bindPathGeometry.js";

const MAP_FRAME = "map";
const BASE_LINK_FRAME = "base_link";
const SCEPTER_FRAME = "Scepter_depth_frame";
const GRIPPER_FRAME = "gripper_frame";
const ROBOT_BODY_SIZE_METERS = 0.7;
const TCP_TOOL_SIZE_METERS = {
  x: 0.1,
  y: 0.2,
  z: 0.2,
};
const FREE_VIEW_POSITION = new THREE.Vector3(2.6, -3.0, 2.0);
const FREE_VIEW_TARGET = new THREE.Vector3(0, 0, 0.15);
const WORLD_UP_AXIS = new THREE.Vector3(0, 0, 1);
const CAMERA_VIEW_FORWARD_AXIS = new THREE.Vector3(0, 0, 1);
const CAMERA_VIEW_UP_AXIS = new THREE.Vector3(0, -1, 0);
const CAMERA_VIEW_TARGET_DISTANCE_METERS = 1.0;
const TOP_VIEW_FORWARD_AXIS = new THREE.Vector3(0, 0, -1);
const TOP_VIEW_UP_AXIS = new THREE.Vector3(0, 1, 0);
const TOP_VIEW_HEIGHT_METERS = 6.5;
const MAP_VIEW_ORIGIN = new THREE.Vector3(0, 0, 0);
const MARKER_TYPE_SPHERE_LIST = 7;
const MARKER_TYPE_POINTS = 8;
const BIND_GROUP_LINE_Z_OFFSET_METERS = 0.012;
const LINEAR_MODULE_BIND_RANGE_MIN_SIZE_METERS = 0.001;

function buildPointsObject(colorHex) {
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.Float32BufferAttribute([], 3));
  const material = new THREE.PointsMaterial({
    color: colorHex,
    size: 0.035,
    transparent: true,
    opacity: 0.78,
    sizeAttenuation: true,
  });
  return new THREE.Points(geometry, material);
}

function buildLineSegmentsObject(colorHex) {
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.Float32BufferAttribute([], 3));
  const material = new THREE.LineBasicMaterial({
    color: colorHex,
    transparent: true,
    opacity: 0.6,
  });
  return new THREE.LineSegments(geometry, material);
}

function toVector3Meters(point) {
  return new THREE.Vector3(
    Number(point?.x || 0),
    Number(point?.y || 0),
    Number(point?.z || 0),
  );
}

function normalizeLinearModuleLocalPositionMm(localPosition) {
  if (!localPosition) {
    return null;
  }
  const x = Number(localPosition.x);
  const y = Number(localPosition.y);
  const z = Number(localPosition.z);
  if (![x, y, z].every(Number.isFinite)) {
    return null;
  }
  return { x, y, z };
}

function isPlanningPointMarker(marker) {
  return marker?.ns === "pseudo_slam_points"
    || marker?.type === MARKER_TYPE_SPHERE_LIST
    || marker?.type === MARKER_TYPE_POINTS;
}

function buildMarkerPointPositions(markers) {
  const positions = [];
  markers.forEach((marker) => {
    if (!isPlanningPointMarker(marker)) {
      return;
    }
    const markerPoints = Array.isArray(marker?.points) ? marker.points : [];
    markerPoints.forEach((point) => {
      positions.push(
        Number(point?.x || 0),
        Number(point?.y || 0),
        Number(point?.z || 0),
      );
    });
  });
  return positions;
}

function toMetersFromMillimeters(value) {
  return Number(value || 0) / 1000.0;
}

function buildSequentialSegmentPositionsFromVectors(vectors) {
  if (!Array.isArray(vectors) || vectors.length < 2) {
    return [];
  }

  const linePositions = [];
  for (let index = 1; index < vectors.length; index += 1) {
    const startPoint = vectors[index - 1];
    const endPoint = vectors[index];
    linePositions.push(
      startPoint.x,
      startPoint.y,
      startPoint.z,
      endPoint.x,
      endPoint.y,
      endPoint.z,
    );
  }
  return linePositions;
}

function buildClosedFrameSegmentPositionsFromVectors(vectors) {
  if (!Array.isArray(vectors) || vectors.length < 3) {
    return [];
  }

  const linePositions = [];
  for (let index = 0; index < vectors.length; index += 1) {
    const startPoint = vectors[index];
    const endPoint = vectors[(index + 1) % vectors.length];
    linePositions.push(
      startPoint.x,
      startPoint.y,
      startPoint.z,
      endPoint.x,
      endPoint.y,
      endPoint.z,
    );
  }
  return linePositions;
}

function buildAreaWorldPoints(area) {
  const points = [];
  const groups = Array.isArray(area?.groups) ? area.groups : [];
  groups.forEach((group) => {
    const groupPoints = Array.isArray(group?.points) ? group.points : [];
    groupPoints.forEach((point) => {
      points.push(new THREE.Vector3(
        toMetersFromMillimeters(point?.world_x),
        toMetersFromMillimeters(point?.world_y),
        toMetersFromMillimeters(point?.world_z),
      ));
    });
  });
  return points;
}

function dedupeAreaPoints(points) {
  const seenKeys = new Set();
  return points.filter((point) => {
    const key = `${point.x.toFixed(6)},${point.y.toFixed(6)}`;
    if (seenKeys.has(key)) {
      return false;
    }
    seenKeys.add(key);
    return true;
  });
}

function buildConvexHullXY(points) {
  const uniquePoints = dedupeAreaPoints(points).sort((left, right) => (
    left.x === right.x ? left.y - right.y : left.x - right.x
  ));
  if (uniquePoints.length <= 2) {
    return uniquePoints;
  }

  const cross = (origin, pointA, pointB) => (
    ((pointA.x - origin.x) * (pointB.y - origin.y))
    - ((pointA.y - origin.y) * (pointB.x - origin.x))
  );

  const lowerHull = [];
  uniquePoints.forEach((point) => {
    while (lowerHull.length >= 2 && cross(lowerHull[lowerHull.length - 2], lowerHull[lowerHull.length - 1], point) <= 0) {
      lowerHull.pop();
    }
    lowerHull.push(point);
  });

  const upperHull = [];
  [...uniquePoints].reverse().forEach((point) => {
    while (upperHull.length >= 2 && cross(upperHull[upperHull.length - 2], upperHull[upperHull.length - 1], point) <= 0) {
      upperHull.pop();
    }
    upperHull.push(point);
  });

  lowerHull.pop();
  upperHull.pop();
  return lowerHull.concat(upperHull);
}

function buildAreaCenterPositions(areas) {
  const positions = [];
  areas.forEach((area) => {
    const cabinPose = area?.cabin_pose || {};
    positions.push(
      toMetersFromMillimeters(cabinPose.x),
      toMetersFromMillimeters(cabinPose.y),
      toMetersFromMillimeters(cabinPose.z),
    );
  });
  return positions;
}

function buildAreaCenterPathPositions(areas, pathOrigin = null) {
  const routePoints = [];
  if (pathOrigin && typeof pathOrigin === "object") {
    routePoints.push(new THREE.Vector3(
      toMetersFromMillimeters(pathOrigin.x),
      toMetersFromMillimeters(pathOrigin.y),
      toMetersFromMillimeters(pathOrigin.z),
    ));
  }

  areas.forEach((area) => {
    const cabinPose = area?.cabin_pose || {};
    routePoints.push(new THREE.Vector3(
      toMetersFromMillimeters(cabinPose.x),
      toMetersFromMillimeters(cabinPose.y),
      toMetersFromMillimeters(cabinPose.z),
    ));
  });

  return buildSequentialSegmentPositionsFromVectors(routePoints);
}

function buildAreaOutlineSegmentPositions(areas) {
  const positions = [];
  areas.forEach((area) => {
    const hullPoints = buildConvexHullXY(buildAreaWorldPoints(area));
    if (hullPoints.length < 2) {
      return;
    }
    if (hullPoints.length === 2) {
      positions.push(
        hullPoints[0].x, hullPoints[0].y, hullPoints[0].z,
        hullPoints[1].x, hullPoints[1].y, hullPoints[1].z,
      );
      return;
    }
    for (let index = 0; index < hullPoints.length; index += 1) {
      const startPoint = hullPoints[index];
      const endPoint = hullPoints[(index + 1) % hullPoints.length];
      positions.push(
        startPoint.x, startPoint.y, startPoint.z,
        endPoint.x, endPoint.y, endPoint.z,
      );
    }
  });
  return positions;
}

function composeWorldTransform(frameId, transformMap, cache = new Map(), stack = new Set()) {
  if (cache.has(frameId)) {
    return cache.get(frameId);
  }

  if (frameId === MAP_FRAME) {
    const identity = {
      position: new THREE.Vector3(0, 0, 0),
      quaternion: new THREE.Quaternion(),
    };
    cache.set(frameId, identity);
    return identity;
  }

  if (stack.has(frameId)) {
    return null;
  }

  const record = transformMap.get(frameId);
  if (!record) {
    cache.set(frameId, null);
    return null;
  }

  stack.add(frameId);
  const parentWorld = composeWorldTransform(record.parentFrame, transformMap, cache, stack);
  if (!parentWorld) {
    cache.set(frameId, null);
    stack.delete(frameId);
    return null;
  }
  const localPosition = record.position.clone().applyQuaternion(parentWorld.quaternion);
  const world = {
    position: parentWorld.position.clone().add(localPosition),
    quaternion: parentWorld.quaternion.clone().multiply(record.quaternion.clone()),
  };
  cache.set(frameId, world);
  stack.delete(frameId);
  return world;
}

function getTfAxisFrameVisibility(state) {
  const configuredVisibility = state?.tfAxisFrameVisibility || {};
  return {
    [MAP_FRAME]: configuredVisibility[MAP_FRAME] !== false,
    [BASE_LINK_FRAME]: configuredVisibility[BASE_LINK_FRAME] !== false,
    [SCEPTER_FRAME]: configuredVisibility[SCEPTER_FRAME] !== false,
    [GRIPPER_FRAME]: configuredVisibility[GRIPPER_FRAME] !== false,
  };
}

export class Scene3DView {
  constructor({ container }) {
    this.container = container;
    this.layerState = {};
    this.transformMap = new Map();
    this.cachedWorldTransforms = new Map();
    this.linearModuleLocalPositionMm = null;
    this.linearModuleBindRangeMm = normalizeTcpWorkspaceBoundaryMm(TCP_WORKSPACE_BOUNDARY_MM);
    this.sourcePointCloudPositions = {
      filteredWorldCoord: new Float32Array(),
      rawWorldCoord: new Float32Array(),
    };
    this.sourceTiePointCameraPositions = new Float32Array();
    this.planningPointsFollowTiePoints = false;
    this.planningAreaPayload = null;
    this.jumpBindVisualizationState = {
      enabled: false,
      selectedParity: 0,
    };
    this.pointCounts = {
      filteredWorldCoord: 0,
      rawWorldCoord: 0,
      tiePoints: 0,
      planningPoints: 0,
      bindPathPoints: 0,
      jumpBindPoints: 0,
    };

    this.scene = new THREE.Scene();
    this.scene.up.set(0, 0, 1);

    this.camera = new THREE.PerspectiveCamera(55, 1, 0.01, 200);
    this.camera.up.set(0, 0, 1);
    this.camera.position.copy(FREE_VIEW_POSITION);

    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    this.renderer.setPixelRatio(window.devicePixelRatio || 1);
    this.container.appendChild(this.renderer.domElement);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.target.copy(FREE_VIEW_TARGET);
    this.controls.minDistance = 0.25;
    this.controls.maxDistance = 24;
    this.controls.minPolarAngle = 0.08;
    this.controls.maxPolarAngle = Math.PI * 0.48;

    this.ambientLight = new THREE.AmbientLight(0xffffff, 1.1);
    this.keyLight = new THREE.DirectionalLight(0xffffff, 0.9);
    this.keyLight.position.set(3, -2, 4);
    this.scene.add(this.ambientLight, this.keyLight);

    this.grid = new THREE.GridHelper(8, 16, 0x264c5f, 0x16303d);
    this.grid.rotation.x = Math.PI / 2;
    this.scene.add(this.grid);

    this.mapAxes = new THREE.AxesHelper(0.4);
    this.scene.add(this.mapAxes);

    this.baseLinkFrame = new THREE.Group();
    this.baseLinkFrame.add(new THREE.AxesHelper(0.32));
    this.scene.add(this.baseLinkFrame);

    this.scepterFrame = new THREE.Group();
    this.scepterFrame.add(new THREE.AxesHelper(0.28));
    this.scene.add(this.scepterFrame);

    this.gripperFrame = new THREE.Group();
    this.gripperFrame.add(new THREE.AxesHelper(0.22));
    this.scene.add(this.gripperFrame);

    this.robotGroup = new THREE.Group();
    const robotMaterial = new THREE.MeshStandardMaterial({
      color: 0x4c87ff,
      metalness: 0.1,
      roughness: 0.52,
      transparent: true,
      opacity: 0.42,
      side: THREE.DoubleSide,
    });
    const robotMesh = new THREE.Mesh(
      new THREE.BoxGeometry(ROBOT_BODY_SIZE_METERS, ROBOT_BODY_SIZE_METERS, ROBOT_BODY_SIZE_METERS),
      robotMaterial,
    );
    robotMesh.position.z = ROBOT_BODY_SIZE_METERS / 2.0;
    this.robotGroup.add(robotMesh);
    this.scene.add(this.robotGroup);

    this.tcpToolGroup = new THREE.Group();
    const tcpToolMaterial = new THREE.MeshStandardMaterial({
      color: 0xff8c4d,
      metalness: 0.08,
      roughness: 0.4,
      transparent: true,
      opacity: 0.78,
      side: THREE.DoubleSide,
    });
    const tcpToolMesh = new THREE.Mesh(
      new THREE.BoxGeometry(TCP_TOOL_SIZE_METERS.x, TCP_TOOL_SIZE_METERS.y, TCP_TOOL_SIZE_METERS.z),
      tcpToolMaterial,
    );
    tcpToolMesh.position.z = -TCP_TOOL_SIZE_METERS.z / 2.0;
    this.tcpToolGroup.add(tcpToolMesh);
    this.scene.add(this.tcpToolGroup);
    this.robotMaterial = robotMaterial;
    this.tcpToolMaterial = tcpToolMaterial;

    this.linearModuleBindRangeGroup = new THREE.Group();
    this.linearModuleBindRangeMaterial = new THREE.MeshStandardMaterial({
      color: 0x33d7c0,
      metalness: 0.02,
      roughness: 0.56,
      transparent: true,
      opacity: 0.14,
      depthWrite: false,
      side: THREE.DoubleSide,
    });
    this.linearModuleBindRangeEdgeMaterial = new THREE.LineBasicMaterial({
      color: 0x8df7e8,
      transparent: true,
      opacity: 0.72,
    });
    this.linearModuleBindRangeMesh = new THREE.Mesh(
      new THREE.BoxGeometry(1, 1, 1),
      this.linearModuleBindRangeMaterial,
    );
    this.linearModuleBindRangeEdges = new THREE.LineSegments(
      new THREE.EdgesGeometry(new THREE.BoxGeometry(1, 1, 1)),
      this.linearModuleBindRangeEdgeMaterial,
    );
    this.linearModuleBindRangeGroup.add(this.linearModuleBindRangeMesh, this.linearModuleBindRangeEdges);
    this.scene.add(this.linearModuleBindRangeGroup);
    this.updateLinearModuleBindRangeGeometry();

    this.filteredPointCloud = buildPointsObject(0x62d6ff);
    this.rawPointCloud = buildPointsObject(0x4f6b7d);
    this.tiePoints = buildPointsObject(0x57ff9d);
    this.planningPoints = buildPointsObject(0xf8d462);
    this.bindPathPoints = buildPointsObject(0xf8d462);
    this.bindPathPoints.material.size = 0.045;
    this.bindPathPoints.material.opacity = 0.92;
    this.jumpBindPoints = buildPointsObject(0xff5a1f);
    this.jumpBindPoints.material.size = 0.085;
    this.jumpBindPoints.material.opacity = 1;
    this.jumpBindPoints.material.depthTest = false;
    this.jumpBindPoints.material.depthWrite = false;
    this.planningAreaCenters = buildPointsObject(0xff8f3d);
    this.planningAreaCenters.material.size = 0.055;
    this.planningAreaCenters.material.opacity = 0.9;
    const planningAreaPathTemplate = buildLineSegmentsObject(0xffc14d);
    this.planningAreaPath = new THREE.LineSegments(
      planningAreaPathTemplate.geometry,
      planningAreaPathTemplate.material,
    );
    const planningAreaOutlinesTemplate = buildLineSegmentsObject(0xff8f3d);
    this.planningAreaOutlines = new THREE.LineSegments(
      planningAreaOutlinesTemplate.geometry,
      planningAreaOutlinesTemplate.material,
    );
    const bindRowLinesTemplate = buildLineSegmentsObject(0x35d7ff);
    this.bindRowLines = new THREE.LineSegments(
      bindRowLinesTemplate.geometry,
      bindRowLinesTemplate.material,
    );
    const bindColumnLinesTemplate = buildLineSegmentsObject(0xffd15c);
    this.bindColumnLines = new THREE.LineSegments(
      bindColumnLinesTemplate.geometry,
      bindColumnLinesTemplate.material,
    );
    const bindGroupLinesTemplate = buildLineSegmentsObject(0xff4f8a);
    this.bindGroupLines = new THREE.LineSegments(
      bindGroupLinesTemplate.geometry,
      bindGroupLinesTemplate.material,
    );
    this.bindGroupLines.material.opacity = 0.92;
    this.scene.add(
      this.filteredPointCloud,
      this.rawPointCloud,
      this.tiePoints,
      this.planningPoints,
      this.bindPathPoints,
      this.jumpBindPoints,
      this.planningAreaCenters,
      this.planningAreaPath,
      this.planningAreaOutlines,
      this.bindRowLines,
      this.bindColumnLines,
      this.bindGroupLines,
    );
    this.bindPathPoints.renderOrder = 3;
    this.jumpBindPoints.renderOrder = 6;
    this.planningAreaCenters.renderOrder = 2;
    this.planningAreaPath.renderOrder = 2;
    this.planningAreaOutlines.renderOrder = 2;
    this.bindRowLines.renderOrder = 3;
    this.bindColumnLines.renderOrder = 3;
    this.bindGroupLines.renderOrder = 4;

    this.viewMode = "free";
    this.followOrigin = false;
    this.lastFollowOrigin = null;
    this.needsViewReset = true;
    this.theme = "dark";

    this.resizeObserver = new ResizeObserver(() => this.resize());
    this.resizeObserver.observe(this.container);
    this.setTheme(this.theme);
    this.resize();
    this.startRenderLoop();
  }

  startRenderLoop() {
    const render = () => {
      if (this.needsViewReset) {
        this.resetView(this.viewMode);
        this.needsViewReset = false;
      } else if (this.viewMode === "camera" && this.followOrigin) {
        this.resetView(this.viewMode);
      } else if (this.followOrigin) {
        this.followCurrentViewOrigin();
      }
      this.controls.update();
      this.renderer.render(this.scene, this.camera);
      this.rafId = window.requestAnimationFrame(render);
    };
    render();
  }

  resize() {
    const width = Math.max(this.container.clientWidth, 280);
    const height = Math.max(this.container.clientHeight, 220);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height);
  }

  setLayerState(state) {
    this.layerState = { ...state };
    this.robotGroup.visible = this.isRobotBodyVisible();
    this.tcpToolGroup.visible = this.isTcpToolVisible();
    const baseLinkTransform = this.getWorldTransform(BASE_LINK_FRAME);
    const scepterTransform = this.getWorldTransform(SCEPTER_FRAME);
    const gripperTransform = this.getWorldTransform(GRIPPER_FRAME);
    const showLinearModuleBindRange = state.showLinearModuleBindRange !== false;
    if (this.linearModuleBindRangeGroup) {
      this.linearModuleBindRangeGroup.visible = showLinearModuleBindRange && Boolean(gripperTransform);
    }
    this.mapAxes.visible = this.isTfAxisFrameVisible(MAP_FRAME);
    this.baseLinkFrame.visible = Boolean(baseLinkTransform) && this.isTfAxisFrameVisible(BASE_LINK_FRAME);
    this.scepterFrame.visible = Boolean(scepterTransform) && this.isTfAxisFrameVisible(SCEPTER_FRAME);
    this.gripperFrame.visible = Boolean(gripperTransform) && this.isTfAxisFrameVisible(GRIPPER_FRAME);
    // 地面网格是固定参考面，不跟“坐标轴”开关绑定，避免主场景地面消失。
    this.grid.visible = true;

    this.filteredPointCloud.visible =
      Boolean(state.showPointCloud) && state.pointCloudSource === "filteredWorldCoord";
    this.rawPointCloud.visible =
      Boolean(state.showPointCloud) && state.pointCloudSource === "rawWorldCoord";
    const showBindPoints = Boolean(state.showBindPoints ?? state.showPlanningMarkers);
    const showBindGridLines = Boolean(state.showBindGridLines ?? state.showPlanningMarkers);
    const showBindGroups = Boolean(state.showBindGroups ?? state.showPlanningMarkers);
    const showCabinPath = Boolean(state.showCabinPath ?? state.showPlanningMarkers);
    const hasBindPathPointOverlay = this.pointCounts.bindPathPoints > 0;
    const hasJumpBindPointOverlay = this.pointCounts.jumpBindPoints > 0;
    this.tiePoints.visible = Boolean(state.showTiePoints);
    this.planningPoints.visible = showBindPoints && !hasBindPathPointOverlay;
    this.bindPathPoints.visible = showBindPoints && hasBindPathPointOverlay;
    this.jumpBindPoints.visible =
      showBindPoints && hasBindPathPointOverlay && hasJumpBindPointOverlay && this.jumpBindVisualizationState.enabled;
    this.planningAreaCenters.visible = showCabinPath;
    this.planningAreaPath.visible = showCabinPath;
    this.planningAreaOutlines.visible = showBindGroups;
    this.bindRowLines.visible = showBindGridLines;
    this.bindColumnLines.visible = showBindGridLines;
    this.bindGroupLines.visible = showBindGroups;

    [this.filteredPointCloud, this.rawPointCloud, this.tiePoints, this.planningPoints, this.bindPathPoints].forEach((object) => {
      object.material.size = Number(state.pointSize) || 0.035;
      object.material.opacity = Number(state.pointOpacity) || 0.78;
      object.material.needsUpdate = true;
    });
    this.jumpBindPoints.material.size = Math.max((Number(state.pointSize) || 0.035) * 2.2, 0.085);
    this.jumpBindPoints.material.opacity = 1;
    this.jumpBindPoints.material.needsUpdate = true;
  }

  setViewMode(viewMode) {
    this.viewMode = ["free", "camera", "top"].includes(viewMode) ? viewMode : "free";
    this.syncControlsForViewMode();
    this.applyFrameTransforms();
    this.lastFollowOrigin = this.resolveSceneViewOrigin();
    this.needsViewReset = true;
  }

  setFollowOrigin(enabled) {
    this.followOrigin = Boolean(enabled);
    this.lastFollowOrigin = this.resolveSceneViewOrigin();
    if (this.viewMode !== "free") {
      this.needsViewReset = true;
    }
  }

  syncControlsForViewMode() {
    const isCameraView = this.viewMode === "camera";
    this.controls.enabled = !isCameraView;
    this.controls.enablePan = !isCameraView;
    this.controls.enableRotate = this.viewMode === "free";
  }

  isRobotBodyVisible() {
    return this.layerState.showRobot !== false && this.viewMode !== "camera";
  }

  isTcpToolVisible() {
    return this.layerState.showRobot !== false;
  }

  resetView(viewMode = this.viewMode) {
    const normalizedViewMode = ["free", "camera", "top"].includes(viewMode) ? viewMode : "free";
    this.viewMode = normalizedViewMode;
    this.syncControlsForViewMode();
    if (normalizedViewMode === "camera") {
      const cameraPose = this.resolveCameraViewPose();
      if (cameraPose) {
        this.applyViewPose(cameraPose);
        this.lastFollowOrigin = cameraPose.position.clone();
        this.applyFrameTransforms();
        return;
      }
    }
    if (normalizedViewMode === "top") {
      const topPose = this.resolveTopViewPose();
      this.applyViewPose(topPose);
      this.lastFollowOrigin = topPose.target.clone();
      this.applyFrameTransforms();
      return;
    }

    this.applyViewPose({
      position: FREE_VIEW_POSITION,
      target: FREE_VIEW_TARGET,
      up: WORLD_UP_AXIS,
    });
    this.lastFollowOrigin = this.resolveSceneViewOrigin();
    this.applyFrameTransforms();
  }

  resolveCameraViewPose() {
    const scepterTransform = this.getWorldTransform(SCEPTER_FRAME);
    if (!scepterTransform) {
      return null;
    }
    const position = scepterTransform.position.clone();
    const forward = CAMERA_VIEW_FORWARD_AXIS.clone().applyQuaternion(scepterTransform.quaternion).normalize();
    const up = CAMERA_VIEW_UP_AXIS.clone().applyQuaternion(scepterTransform.quaternion).normalize();
    return {
      position,
      target: position.clone().add(forward.multiplyScalar(CAMERA_VIEW_TARGET_DISTANCE_METERS)),
      up,
    };
  }

  resolveTopViewPose() {
    const target = this.resolveSceneViewOrigin("top") || MAP_VIEW_ORIGIN.clone();
    const forward = TOP_VIEW_FORWARD_AXIS.clone().normalize();
    return {
      position: target.clone().sub(forward.multiplyScalar(TOP_VIEW_HEIGHT_METERS)),
      target,
      up: TOP_VIEW_UP_AXIS,
    };
  }

  resolveSceneViewOrigin(viewMode = this.viewMode) {
    if (viewMode === "camera") {
      return this.getWorldTransform(SCEPTER_FRAME)?.position.clone() || null;
    }
    if (viewMode === "top") {
      return MAP_VIEW_ORIGIN.clone();
    }
    const baseLinkTransform = this.getWorldTransform(BASE_LINK_FRAME);
    const scepterTransform = this.getWorldTransform(SCEPTER_FRAME);
    return (baseLinkTransform || scepterTransform)?.position.clone() || MAP_VIEW_ORIGIN.clone();
  }

  followCurrentViewOrigin() {
    const origin = this.resolveSceneViewOrigin();
    if (!origin) {
      this.lastFollowOrigin = null;
      return;
    }
    if (this.lastFollowOrigin) {
      const delta = origin.clone().sub(this.lastFollowOrigin);
      if (delta.lengthSq() > 0) {
        this.camera.position.add(delta);
        this.controls.target.add(delta);
      }
    }
    this.lastFollowOrigin = origin.clone();
  }

  applyViewPose({ position, target, up }) {
    this.camera.up.copy(up || WORLD_UP_AXIS).normalize();
    this.camera.position.copy(position);
    this.controls.target.copy(target);
  }

  handleTfMessage(message) {
    const transforms = Array.isArray(message?.transforms) ? message.transforms : [];
    transforms.forEach((transformMsg) => {
      const childFrame = transformMsg?.child_frame_id;
      const parentFrame = transformMsg?.header?.frame_id;
      if (!childFrame || !parentFrame) {
        return;
      }
      this.transformMap.set(childFrame, {
        parentFrame,
        position: new THREE.Vector3(
          Number(transformMsg.transform?.translation?.x || 0),
          Number(transformMsg.transform?.translation?.y || 0),
          Number(transformMsg.transform?.translation?.z || 0),
        ),
        quaternion: new THREE.Quaternion(
          Number(transformMsg.transform?.rotation?.x || 0),
          Number(transformMsg.transform?.rotation?.y || 0),
          Number(transformMsg.transform?.rotation?.z || 0),
          Number(transformMsg.transform?.rotation?.w || 1),
        ),
      });
    });
    this.cachedWorldTransforms.clear();
    this.applyFrameTransforms();
    this.refreshPointCloudWorldPositions();
    this.refreshTiePointWorldPositions();
  }

  applyFrameTransforms() {
    const baseLinkTransform = this.getWorldTransform(BASE_LINK_FRAME);
    this.applyGroupTfTransform(this.baseLinkFrame, baseLinkTransform, BASE_LINK_FRAME);
    const scepterTransform = this.getWorldTransform(SCEPTER_FRAME);
    this.applyGroupTfTransform(this.scepterFrame, scepterTransform, SCEPTER_FRAME);
    const gripperTransform = this.getWorldTransform(GRIPPER_FRAME);
    const tcpToolPosition = this.getLinearModuleTcpWorldPosition() || gripperTransform?.position || null;
    const gripperDisplayTransform = gripperTransform && tcpToolPosition
      ? {
        position: tcpToolPosition,
        quaternion: gripperTransform.quaternion,
      }
      : gripperTransform;
    this.applyGroupTfTransform(this.gripperFrame, gripperDisplayTransform, GRIPPER_FRAME);
    const robotTransform = baseLinkTransform || scepterTransform;
    if (robotTransform) {
      this.robotGroup.visible = this.isRobotBodyVisible();
      this.applyWorldAlignedDisplayPose(this.robotGroup, robotTransform.position);
    } else {
      this.robotGroup.visible = false;
    }

    if (tcpToolPosition) {
      this.tcpToolGroup.visible = this.isTcpToolVisible();
      this.applyCustomDisplayPose(this.tcpToolGroup, tcpToolPosition);
    } else {
      this.tcpToolGroup.visible = false;
    }
    this.applyLinearModuleBindRangeTransform(gripperTransform);
  }

  applyLinearModuleBindRangeTransform(gripperTransform) {
    if (!this.linearModuleBindRangeGroup) {
      return;
    }
    if (
      !gripperTransform
      || this.layerState.showLinearModuleBindRange === false
    ) {
      this.linearModuleBindRangeGroup.visible = false;
      return;
    }
    this.linearModuleBindRangeGroup.visible = true;
    this.applyTfFramePose(this.linearModuleBindRangeGroup, gripperTransform);
  }

  isTfAxisFrameVisible(frameId) {
    if (!this.layerState?.showAxes) {
      return false;
    }
    const tfAxisFrameVisibility = getTfAxisFrameVisibility(this.layerState);
    return tfAxisFrameVisibility[frameId] !== false;
  }

  applyGroupTfTransform(group, transform, frameId) {
    if (!transform) {
      group.visible = false;
      return;
    }
    group.visible = this.isTfAxisFrameVisible(frameId);
    this.applyTfFramePose(group, transform);
  }

  applyTfFramePose(group, transform) {
    group.position.copy(transform.position);
    group.quaternion.copy(transform.quaternion);
    group.scale.set(1, 1, 1);
  }

  applyWorldAlignedDisplayPose(group, position) {
    group.position.copy(position);
    group.quaternion.identity();
    group.scale.set(1, 1, 1);
  }

  applyCustomDisplayPose(group, position) {
    group.position.copy(position);
    group.quaternion.identity();
    group.scale.set(1, 1, -1);
  }

  applyCameraToTcpCalibration(calibration) {
    const translationMm = calibration?.translationMm || {};
    const xMm = Number(translationMm.x);
    const yMm = Number(translationMm.y);
    const zMm = Number(translationMm.z);
    if (![xMm, yMm, zMm].every(Number.isFinite)) {
      return null;
    }

    const currentRecord = this.transformMap.get(GRIPPER_FRAME);
    const quaternion = currentRecord?.quaternion?.clone()
      || new THREE.Quaternion().setFromEuler(new THREE.Euler(0, 0, Math.PI / 2));
    this.transformMap.set(GRIPPER_FRAME, {
      parentFrame: SCEPTER_FRAME,
      position: new THREE.Vector3(
        xMm / 1000.0,
        yMm / 1000.0,
        zMm / 1000.0,
      ),
      quaternion,
    });
    this.cachedWorldTransforms.clear();
    this.applyFrameTransforms();
    return this.getCameraToTcpCalibration();
  }

  getWorldTransform(frameId) {
    return composeWorldTransform(frameId, this.transformMap, this.cachedWorldTransforms);
  }

  resolveProjectionFrameTransform(frameId) {
    return this.getWorldTransform(frameId) || this.getWorldTransform(SCEPTER_FRAME);
  }

  projectFramePointToImagePixel(pointMm, sourceFrame, cameraInfo) {
    const sourceTransform = this.getWorldTransform(sourceFrame);
    const projection = normalizeCameraProjection(cameraInfo);
    const cameraFrame = cameraInfo?.header?.frame_id || SCEPTER_FRAME;
    const cameraTransform = this.resolveProjectionFrameTransform(cameraFrame);
    if (!sourceTransform || !cameraTransform || !projection) {
      return null;
    }

    const localPoint = new THREE.Vector3(
      Number(pointMm?.x || 0) / 1000.0,
      Number(pointMm?.y || 0) / 1000.0,
      Number(pointMm?.z || 0) / 1000.0,
    ).applyQuaternion(sourceTransform.quaternion);
    const mapPoint = sourceTransform.position.clone().add(localPoint);
    const cameraPoint = mapPoint
      .sub(cameraTransform.position)
      .applyQuaternion(cameraTransform.quaternion.clone().invert());
    return projectCameraPointMetersToImagePixel(cameraPoint, cameraInfo);
  }

  projectMapPointToImagePixel(mapPoint, cameraInfo) {
    const projection = normalizeCameraProjection(cameraInfo);
    const cameraFrame = cameraInfo?.header?.frame_id || SCEPTER_FRAME;
    const cameraTransform = this.resolveProjectionFrameTransform(cameraFrame);
    if (!cameraTransform || !projection) {
      return null;
    }
    const cameraPoint = mapPoint
      .clone()
      .sub(cameraTransform.position)
      .applyQuaternion(cameraTransform.quaternion.clone().invert());
    return projectCameraPointMetersToImagePixel(cameraPoint, cameraInfo);
  }

  projectTcpWorkspaceBoundaryToImage(cameraInfo) {
    const projection = normalizeCameraProjection(cameraInfo);
    if (!projection || !this.getWorldTransform(GRIPPER_FRAME)) {
      return null;
    }
    const planes = buildTcpWorkspaceBoundaryPlanesMm(this.linearModuleBindRangeMm)
      .map((plane) => ({
        z: plane.z,
        points: plane.points.map((point) => this.projectFramePointToImagePixel(point, GRIPPER_FRAME, cameraInfo)),
      }));
    if (!planes.length || planes.some((plane) => plane.points.length !== 4 || plane.points.some((point) => !point))) {
      return null;
    }
    return {
      points: planes[0].points,
      planes,
      sourceSize: {
        width: projection.width,
        height: projection.height,
      },
      frameId: GRIPPER_FRAME,
    };
  }

  setTheme(theme) {
    this.theme = theme === "light" ? "light" : "dark";
    if (this.theme === "light") {
      this.scene.background = new THREE.Color(0xf3f6fb);
      this.ambientLight.intensity = 1.2;
      this.keyLight.intensity = 0.82;
      this.robotMaterial.color.setHex(0x3d63d8);
      this.tcpToolMaterial.color.setHex(0xd67a2f);
      this.linearModuleBindRangeMaterial.color.setHex(0x1aa89d);
      this.linearModuleBindRangeEdgeMaterial.color.setHex(0x2faeac);
      const materials = Array.isArray(this.grid.material) ? this.grid.material : [this.grid.material];
      if (materials[0]?.color) {
        materials[0].color.setHex(0x7d8da4);
      }
      if (materials[1]?.color) {
        materials[1].color.setHex(0xa6b2c2);
      }
      this.planningAreaCenters.material.color.setHex(0xe1781d);
      this.planningAreaPath.material.color.setHex(0xd69314);
      this.planningAreaOutlines.material.color.setHex(0xe1781d);
      this.bindPathPoints.material.color.setHex(0xd69314);
      this.applyJumpBindPointMaterialColor();
      this.bindRowLines.material.color.setHex(0x1f8fb8);
      this.bindColumnLines.material.color.setHex(0xc9971f);
      this.bindGroupLines.material.color.setHex(0xd92f69);
      return;
    }

    this.scene.background = new THREE.Color(0x071018);
    this.ambientLight.intensity = 1.1;
    this.keyLight.intensity = 0.9;
    this.robotMaterial.color.setHex(0x4c87ff);
    this.tcpToolMaterial.color.setHex(0xff8c4d);
    this.linearModuleBindRangeMaterial.color.setHex(0x33d7c0);
    this.linearModuleBindRangeEdgeMaterial.color.setHex(0x8df7e8);
    const materials = Array.isArray(this.grid.material) ? this.grid.material : [this.grid.material];
    if (materials[0]?.color) {
      materials[0].color.setHex(0xffffff);
    }
    if (materials[1]?.color) {
      materials[1].color.setHex(0xe8eef7);
    }
    this.planningAreaCenters.material.color.setHex(0xff8f3d);
    this.planningAreaPath.material.color.setHex(0xffc14d);
    this.planningAreaOutlines.material.color.setHex(0xff8f3d);
    this.bindPathPoints.material.color.setHex(0xf8d462);
    this.applyJumpBindPointMaterialColor();
    this.bindRowLines.material.color.setHex(0x35d7ff);
    this.bindColumnLines.material.color.setHex(0xffd15c);
    this.bindGroupLines.material.color.setHex(0xff4f8a);
  }

  applyJumpBindPointMaterialColor() {
    const selectedParity = Number(this.jumpBindVisualizationState?.selectedParity) === 1 ? 1 : 0;
    if (selectedParity === 1) {
      this.jumpBindPoints.material.color.setHex(this.theme === "light" ? 0x0f62fe : 0x38bdf8);
      return;
    }
    this.jumpBindPoints.material.color.setHex(this.theme === "light" ? 0xc2410c : 0xff5a1f);
  }

  getKnownTransformCount() {
    return this.transformMap.size;
  }

  getKnownTransforms() {
    return Array.from(this.transformMap.entries())
      .map(([childFrame, record]) => ({
        childFrame,
        parentFrame: record.parentFrame,
      }))
      .sort((left, right) => left.childFrame.localeCompare(right.childFrame));
  }

  getCurrentCabinPositionMm() {
    const poseTransform = this.getWorldTransform(BASE_LINK_FRAME) || this.getWorldTransform(SCEPTER_FRAME);
    if (!poseTransform) {
      return null;
    }

    return {
      x: poseTransform.position.x * 1000.0,
      y: poseTransform.position.y * 1000.0,
      z: poseTransform.position.z * 1000.0,
    };
  }

  setLinearModuleLocalPosition(localPosition) {
    this.linearModuleLocalPositionMm = normalizeLinearModuleLocalPositionMm(localPosition);
    this.applyFrameTransforms();
    return this.getLinearModuleGlobalPositionMm(this.linearModuleLocalPositionMm);
  }

  setLinearModuleBindRange(range) {
    this.linearModuleBindRangeMm = normalizeTcpWorkspaceBoundaryMm(range);
    this.updateLinearModuleBindRangeGeometry();
    if (
      this.applyFrameTransforms !== Scene3DView.prototype.applyFrameTransforms
      || (this.baseLinkFrame && this.scepterFrame && this.gripperFrame && this.tcpToolGroup)
    ) {
      this.applyFrameTransforms();
    }
    return this.linearModuleBindRangeMm;
  }

  updateLinearModuleBindRangeGeometry() {
    if (!this.linearModuleBindRangeMesh || !this.linearModuleBindRangeEdges) {
      return;
    }
    const bounds = normalizeTcpWorkspaceBoundaryMm(this.linearModuleBindRangeMm);
    const sizeMeters = {
      x: Math.max((bounds.x.max - bounds.x.min) / 1000.0, LINEAR_MODULE_BIND_RANGE_MIN_SIZE_METERS),
      y: Math.max((bounds.y.max - bounds.y.min) / 1000.0, LINEAR_MODULE_BIND_RANGE_MIN_SIZE_METERS),
      z: Math.max((bounds.z.max - bounds.z.min) / 1000.0, LINEAR_MODULE_BIND_RANGE_MIN_SIZE_METERS),
    };
    const centerMeters = {
      x: (bounds.x.min + bounds.x.max) / 2000.0,
      y: (bounds.y.min + bounds.y.max) / 2000.0,
      z: (bounds.z.min + bounds.z.max) / 2000.0,
    };
    const boxGeometry = new THREE.BoxGeometry(sizeMeters.x, sizeMeters.y, sizeMeters.z);
    this.linearModuleBindRangeMesh.geometry?.dispose?.();
    this.linearModuleBindRangeMesh.geometry = boxGeometry;
    this.linearModuleBindRangeMesh.position.set(centerMeters.x, centerMeters.y, centerMeters.z);

    this.linearModuleBindRangeEdges.geometry?.dispose?.();
    this.linearModuleBindRangeEdges.geometry = new THREE.EdgesGeometry(boxGeometry);
    this.linearModuleBindRangeEdges.position.copy(this.linearModuleBindRangeMesh.position);
  }

  getLinearModuleTcpWorldPosition(localPosition = this.linearModuleLocalPositionMm) {
    const gripperTransform = this.getWorldTransform(GRIPPER_FRAME);
    if (!gripperTransform) {
      return null;
    }

    const normalizedPosition = normalizeLinearModuleLocalPositionMm(localPosition);
    if (!normalizedPosition) {
      return null;
    }

    const localPoint = new THREE.Vector3(
      normalizedPosition.x / 1000.0,
      normalizedPosition.y / 1000.0,
      normalizedPosition.z / 1000.0,
    ).applyQuaternion(gripperTransform.quaternion);
    return gripperTransform.position.clone().add(localPoint);
  }

  getLinearModuleGlobalPositionMm(localPosition) {
    const globalPoint = this.getLinearModuleTcpWorldPosition(localPosition);
    if (!globalPoint) {
      return null;
    }

    return {
      x: globalPoint.x * 1000.0,
      y: globalPoint.y * 1000.0,
      z: globalPoint.z * 1000.0,
    };
  }

  refreshPointCloudWorldPositions() {
    this.applyPointCloudWorldPositions("filteredWorldCoord");
    this.applyPointCloudWorldPositions("rawWorldCoord");
  }

  getGripperOffsetRelativeToScepter() {
    const directRecord = this.transformMap.get(GRIPPER_FRAME);
    if (directRecord?.parentFrame === SCEPTER_FRAME) {
      return directRecord.position.z;
    }

    const scepterTransform = this.getWorldTransform(SCEPTER_FRAME);
    const gripperTransform = this.getWorldTransform(GRIPPER_FRAME);
    if (!scepterTransform || !gripperTransform) {
      return 0;
    }

    return gripperTransform.position.z - scepterTransform.position.z;
  }

  getCameraToTcpCalibration() {
    const directRecord = this.transformMap.get(GRIPPER_FRAME);
    if (directRecord?.parentFrame === SCEPTER_FRAME) {
      return {
        parentFrame: SCEPTER_FRAME,
        childFrame: GRIPPER_FRAME,
        translationMm: {
          x: directRecord.position.x * 1000.0,
          y: directRecord.position.y * 1000.0,
          z: directRecord.position.z * 1000.0,
        },
        publishedTranslationMeters: {
          x: directRecord.position.x,
          y: directRecord.position.y,
          z: directRecord.position.z,
        },
      };
    }

    const scepterTransform = this.getWorldTransform(SCEPTER_FRAME);
    const gripperTransform = this.getWorldTransform(GRIPPER_FRAME);
    if (!scepterTransform || !gripperTransform) {
      return null;
    }

    const relative = gripperTransform.position.clone().sub(scepterTransform.position);
    return {
      parentFrame: SCEPTER_FRAME,
      childFrame: GRIPPER_FRAME,
      translationMm: {
        x: relative.x * 1000.0,
        y: relative.y * 1000.0,
        z: relative.z * 1000.0,
      },
      publishedTranslationMeters: {
        x: relative.x,
        y: relative.y,
        z: relative.z,
      },
    };
  }

  convertScepterPointCloudPointToMapPoint(localPoint) {
    const scepterTransform = this.getWorldTransform(SCEPTER_FRAME);
    if (!scepterTransform) {
      return null;
    }

    // pointAI 发布 Scepter_depth_frame 下的相机点，前端只负责走 TF 到 map。
    const pointInMap = localPoint.clone().applyQuaternion(scepterTransform.quaternion);
    return scepterTransform.position.clone().add(pointInMap);
  }

  convertScepterPointMmToFrameMm(cameraPointMm, targetFrame = MAP_FRAME) {
    const cameraPoint = new THREE.Vector3(
      Number(cameraPointMm?.x) / 1000.0,
      Number(cameraPointMm?.y) / 1000.0,
      Number(cameraPointMm?.z) / 1000.0,
    );
    if (![cameraPoint.x, cameraPoint.y, cameraPoint.z].every(Number.isFinite)) {
      return null;
    }

    if (targetFrame === SCEPTER_FRAME) {
      return {
        x: cameraPoint.x * 1000.0,
        y: cameraPoint.y * 1000.0,
        z: cameraPoint.z * 1000.0,
      };
    }

    const mapPoint = this.convertScepterPointCloudPointToMapPoint(cameraPoint);
    if (!mapPoint) {
      return null;
    }
    if (targetFrame === MAP_FRAME) {
      return {
        x: mapPoint.x * 1000.0,
        y: mapPoint.y * 1000.0,
        z: mapPoint.z * 1000.0,
      };
    }

    const targetTransform = this.getWorldTransform(targetFrame);
    if (!targetTransform) {
      return null;
    }
    const localPoint = mapPoint
      .clone()
      .sub(targetTransform.position)
      .applyQuaternion(targetTransform.quaternion.clone().invert());
    return {
      x: localPoint.x * 1000.0,
      y: localPoint.y * 1000.0,
      z: localPoint.z * 1000.0,
    };
  }

  applyPointCloudWorldPositions(source) {
    const localPositions = this.sourcePointCloudPositions[source];
    if (!localPositions?.length) {
      const target = source === "rawWorldCoord" ? this.rawPointCloud : this.filteredPointCloud;
      target.geometry.setAttribute("position", new THREE.Float32BufferAttribute([], 3));
      target.geometry.computeBoundingSphere();
      return;
    }
    const worldPositions = [];
    for (let index = 0; index < localPositions.length; index += 3) {
      const point = new THREE.Vector3(
        localPositions[index],
        localPositions[index + 1],
        localPositions[index + 2],
      );
      const mapPoint = this.convertScepterPointCloudPointToMapPoint(point);
      if (!mapPoint) {
        continue;
      }
      worldPositions.push(mapPoint.x, mapPoint.y, mapPoint.z);
    }
    const target = source === "rawWorldCoord" ? this.rawPointCloud : this.filteredPointCloud;
    target.geometry.setAttribute("position", new THREE.Float32BufferAttribute(worldPositions, 3));
    target.geometry.computeBoundingSphere();
  }

  refreshTiePointWorldPositions() {
    const cameraPositions = this.sourceTiePointCameraPositions;
    const worldPositions = [];
    for (let index = 0; index < cameraPositions.length; index += 3) {
      const vector = new THREE.Vector3(
        cameraPositions[index],
        cameraPositions[index + 1],
        cameraPositions[index + 2],
      );
      const mapPoint = this.convertScepterPointCloudPointToMapPoint(vector);
      if (!mapPoint) {
        continue;
      }
      worldPositions.push(mapPoint.x, mapPoint.y, mapPoint.z);
    }

    this.tiePoints.geometry.setAttribute("position", new THREE.Float32BufferAttribute(worldPositions, 3));
    this.tiePoints.geometry.computeBoundingSphere();
    this.pointCounts.tiePoints = worldPositions.length / 3;
    if (this.planningPointsFollowTiePoints) {
      this.planningPoints.geometry.setAttribute("position", new THREE.Float32BufferAttribute(worldPositions, 3));
      this.planningPoints.geometry.computeBoundingSphere();
      this.pointCounts.planningPoints = this.pointCounts.tiePoints;
    }
    return worldPositions;
  }

  setTiePointsMessage(message) {
    const points = Array.isArray(message?.PointCoordinatesArray) ? message.PointCoordinatesArray : [];
    const cameraPositions = [];
    points.forEach((point) => {
      const world = Array.isArray(point?.World_coord) ? point.World_coord : [];
      if (world.length < 3) {
        return;
      }
      const vector = toVector3Meters({
        x: Number(world[0]) / 1000.0,
        y: Number(world[1]) / 1000.0,
        z: Number(world[2]) / 1000.0,
      });
      cameraPositions.push(vector.x, vector.y, vector.z);
    });
    this.sourceTiePointCameraPositions = new Float32Array(cameraPositions);
    if (this.pointCounts.planningPoints <= 0 || this.planningPointsFollowTiePoints) {
      this.planningPointsFollowTiePoints = true;
    }
    const worldPositions = this.refreshTiePointWorldPositions();
    if (this.planningPointsFollowTiePoints) {
      this.planningPoints.geometry.setAttribute("position", new THREE.Float32BufferAttribute(worldPositions, 3));
      this.planningPoints.geometry.computeBoundingSphere();
      this.pointCounts.planningPoints = this.pointCounts.tiePoints;
    }
    return this.pointCounts.tiePoints;
  }

  setPlanningMarkersMessage(message) {
    const markers = Array.isArray(message?.markers) ? message.markers : [];
    const positions = buildMarkerPointPositions(markers);
    this.planningPointsFollowTiePoints = false;
    this.planningPoints.geometry.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
    this.planningPoints.geometry.computeBoundingSphere();
    this.pointCounts.planningPoints = positions.length / 3;
    return this.pointCounts.planningPoints;
  }

  setPlanningAreaPayload(payload) {
    this.planningAreaPayload = payload || null;
    const areas = Array.isArray(payload?.areas) ? payload.areas : [];
    const gridPoints = Array.isArray(payload?.grid_points) ? payload.grid_points : [];
    const areaCenterPositions = buildAreaCenterPositions(areas);
    const areaPathPositions = buildAreaCenterPathPositions(areas, payload?.path_origin || null);
    const areaOutlinePositions = buildAreaOutlineSegmentPositions(areas);
    const bindPathPointPositions = buildBindPathPointPositions(areas, gridPoints);
    const bindRowLinePositions = buildBindGridLineSegmentPositions(areas, { axis: "row", gridPoints });
    const bindColumnLinePositions = buildBindGridLineSegmentPositions(areas, { axis: "column", gridPoints });
    const bindGroupLinePositions = buildBindGroupLineSegmentPositions(areas, {
      zOffsetMeters: BIND_GROUP_LINE_Z_OFFSET_METERS,
    });

    this.bindPathPoints.geometry.setAttribute(
      "position",
      new THREE.Float32BufferAttribute(bindPathPointPositions, 3),
    );
    this.bindPathPoints.geometry.computeBoundingSphere();
    this.pointCounts.bindPathPoints = bindPathPointPositions.length / 3;
    this.updateJumpBindPointOverlay(areas, gridPoints);
    this.planningAreaCenters.geometry.setAttribute(
      "position",
      new THREE.Float32BufferAttribute(areaCenterPositions, 3),
    );
    this.planningAreaCenters.geometry.computeBoundingSphere();
    this.planningAreaPath.geometry.setAttribute(
      "position",
      new THREE.Float32BufferAttribute(areaPathPositions, 3),
    );
    this.planningAreaPath.geometry.computeBoundingSphere();
    this.planningAreaOutlines.geometry.setAttribute(
      "position",
      new THREE.Float32BufferAttribute(areaOutlinePositions, 3),
    );
    this.planningAreaOutlines.geometry.computeBoundingSphere();
    this.bindRowLines.geometry.setAttribute(
      "position",
      new THREE.Float32BufferAttribute(bindRowLinePositions, 3),
    );
    this.bindRowLines.geometry.computeBoundingSphere();
    this.bindColumnLines.geometry.setAttribute(
      "position",
      new THREE.Float32BufferAttribute(bindColumnLinePositions, 3),
    );
    this.bindColumnLines.geometry.computeBoundingSphere();
    this.bindGroupLines.geometry.setAttribute(
      "position",
      new THREE.Float32BufferAttribute(bindGroupLinePositions, 3),
    );
    this.bindGroupLines.geometry.computeBoundingSphere();
    this.setLayerState(this.layerState);
  }

  setJumpBindVisualizationState(state = {}) {
    this.jumpBindVisualizationState = {
      enabled: Boolean(state?.value ?? state?.enabled),
      selectedParity: Number(state?.selectedParity) === 1 ? 1 : 0,
    };
    const areas = Array.isArray(this.planningAreaPayload?.areas) ? this.planningAreaPayload.areas : [];
    const gridPoints = Array.isArray(this.planningAreaPayload?.grid_points)
      ? this.planningAreaPayload.grid_points
      : [];
    this.updateJumpBindPointOverlay(areas, gridPoints);
    this.applyJumpBindPointMaterialColor();
    this.setLayerState(this.layerState);
  }

  updateJumpBindPointOverlay(areas, gridPoints) {
    const jumpBindPointPositions = buildJumpBindPointPositions(areas, {
      gridPoints,
      enabled: this.jumpBindVisualizationState.enabled,
      selectedParity: this.jumpBindVisualizationState.selectedParity,
    });
    this.jumpBindPoints.geometry.setAttribute(
      "position",
      new THREE.Float32BufferAttribute(jumpBindPointPositions, 3),
    );
    this.jumpBindPoints.geometry.computeBoundingSphere();
    this.pointCounts.jumpBindPoints = jumpBindPointPositions.length / 3;
  }

  setPointCloudImageMessage(source, message) {
    const { positions, count } = decodeFloat32XYZImage(message, {
      sampleStep: source === "rawWorldCoord" ? 8 : 5,
      maxPoints: source === "rawWorldCoord" ? 12000 : 18000,
    });
    this.sourcePointCloudPositions[source] = positions;
    this.applyPointCloudWorldPositions(source);
    this.pointCounts[source] = count;
    return count;
  }

  clearPointCloudSource(source) {
    this.sourcePointCloudPositions[source] = new Float32Array();
    this.pointCounts[source] = 0;
    this.applyPointCloudWorldPositions(source);
    return 0;
  }

  clearAllPointCloudSources() {
    this.clearPointCloudSource("filteredWorldCoord");
    this.clearPointCloudSource("rawWorldCoord");
  }
}
