import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { decodeFloat32XYZImage } from "../utils/irImageUtils.js";

const CABIN_FRAME = "cabin_frame";
const SCEPTER_FRAME = "Scepter_depth_frame";
const GRIPPER_FRAME = "gripper_frame";
const ROBOT_BODY_SIZE_METERS = 0.7;
const TCP_TOOL_SIZE_METERS = {
  x: 0.2,
  y: 0.1,
  z: 0.2,
};
const INITIAL_CAMERA_POSITION = new THREE.Vector3(2.6, -3.0, 2.0);
const INITIAL_CAMERA_TARGET = new THREE.Vector3(0, 0, 0.15);
const CAMERA_VIEW_OFFSET = new THREE.Vector3(-0.85, -1.25, 0.55);
const CAMERA_VIEW_TARGET_OFFSET = new THREE.Vector3(0.18, 0, 0.04);
const GLOBAL_VIEW_POSITION = new THREE.Vector3(3.6, -4.1, 2.8);
const GLOBAL_VIEW_TARGET = new THREE.Vector3(0, 0, 0.1);

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

function toVector3Meters(point) {
  return new THREE.Vector3(
    Number(point?.x || 0),
    Number(point?.y || 0),
    Number(point?.z || 0),
  );
}

function composeWorldTransform(frameId, transformMap, cache = new Map(), stack = new Set()) {
  if (cache.has(frameId)) {
    return cache.get(frameId);
  }

  if (frameId === CABIN_FRAME) {
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

function getPointAiStyleLocalPoint(localPoint) {
  return new THREE.Vector3(
    localPoint.y,
    -localPoint.x,
    localPoint.z,
  );
}

export class Scene3DView {
  constructor({ container }) {
    this.container = container;
    this.layerState = {};
    this.transformMap = new Map();
    this.cachedWorldTransforms = new Map();
    this.sourcePointCloudPositions = {
      filteredWorldCoord: new Float32Array(),
      rawWorldCoord: new Float32Array(),
    };
    this.pointCounts = {
      filteredWorldCoord: 0,
      rawWorldCoord: 0,
      tiePoints: 0,
      planningPoints: 0,
    };

    this.scene = new THREE.Scene();
    this.scene.up.set(0, 0, 1);

    this.camera = new THREE.PerspectiveCamera(55, 1, 0.01, 200);
    this.camera.up.set(0, 0, 1);
    this.camera.position.copy(INITIAL_CAMERA_POSITION);

    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    this.renderer.setPixelRatio(window.devicePixelRatio || 1);
    this.container.appendChild(this.renderer.domElement);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.target.copy(INITIAL_CAMERA_TARGET);
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

    this.cabinAxes = new THREE.AxesHelper(0.4);
    this.scene.add(this.cabinAxes);

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
    this.tcpToolGroup.add(tcpToolMesh);
    this.scene.add(this.tcpToolGroup);
    this.robotMaterial = robotMaterial;
    this.tcpToolMaterial = tcpToolMaterial;

    this.filteredPointCloud = buildPointsObject(0x62d6ff);
    this.rawPointCloud = buildPointsObject(0x4f6b7d);
    this.tiePoints = buildPointsObject(0x57ff9d);
    this.planningPoints = buildPointsObject(0xf8d462);
    this.scene.add(
      this.filteredPointCloud,
      this.rawPointCloud,
      this.tiePoints,
      this.planningPoints,
    );

    this.viewMode = "camera";
    this.followCamera = false;
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
      this.controls.update();
      if (this.followCamera) {
        this.resetView(this.viewMode);
      }
      if (this.needsViewReset) {
        this.resetView(this.viewMode);
        this.needsViewReset = false;
      }
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
    this.robotGroup.visible = Boolean(state.showRobot);
    this.tcpToolGroup.visible = Boolean(state.showRobot);
    const axesVisible = Boolean(state.showAxes);
    this.cabinAxes.visible = axesVisible;
    this.scepterFrame.visible = axesVisible;
    this.gripperFrame.visible = axesVisible;
    // 地面网格是固定参考面，不跟“坐标轴”开关绑定，避免主场景地面消失。
    this.grid.visible = true;

    this.filteredPointCloud.visible =
      Boolean(state.showPointCloud) && state.pointCloudSource === "filteredWorldCoord";
    this.rawPointCloud.visible =
      Boolean(state.showPointCloud) && state.pointCloudSource === "rawWorldCoord";
    this.tiePoints.visible = Boolean(state.showTiePoints);
    this.planningPoints.visible = Boolean(state.showPlanningMarkers);

    [this.filteredPointCloud, this.rawPointCloud, this.tiePoints, this.planningPoints].forEach((object) => {
      object.material.size = Number(state.pointSize) || 0.035;
      object.material.opacity = Number(state.pointOpacity) || 0.78;
      object.material.needsUpdate = true;
    });
  }

  setViewMode(viewMode) {
    this.viewMode = viewMode;
    this.needsViewReset = true;
  }

  setFollowCamera(enabled) {
    this.followCamera = Boolean(enabled);
    this.needsViewReset = true;
  }

  resetView(viewMode = this.viewMode) {
    const scepterTransform = this.getWorldTransform(SCEPTER_FRAME);
    if (viewMode === "camera" && scepterTransform) {
      const nextPosition = scepterTransform.position.clone().add(CAMERA_VIEW_OFFSET);
      const nextTarget = scepterTransform.position.clone().add(CAMERA_VIEW_TARGET_OFFSET);
      this.camera.position.lerp(nextPosition, 0.28);
      this.controls.target.lerp(nextTarget, 0.28);
      return;
    }

    this.camera.position.lerp(GLOBAL_VIEW_POSITION, 0.28);
    this.controls.target.lerp(GLOBAL_VIEW_TARGET, 0.28);
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
  }

  applyFrameTransforms() {
    const scepterTransform = this.getWorldTransform(SCEPTER_FRAME);
    this.applyGroupTransform(this.scepterFrame, scepterTransform);
    const gripperTransform = this.getWorldTransform(GRIPPER_FRAME);
    this.applyGroupTransform(this.gripperFrame, gripperTransform);
    if (scepterTransform) {
      this.robotGroup.visible = this.layerState.showRobot !== false;
      this.applyCustomDisplayPose(this.robotGroup, scepterTransform.position);
    } else {
      this.robotGroup.visible = false;
    }

    if (gripperTransform) {
      this.tcpToolGroup.visible = this.layerState.showRobot !== false;
      this.applyCustomDisplayPose(this.tcpToolGroup, gripperTransform.position);
    } else {
      this.tcpToolGroup.visible = false;
    }
  }

  applyGroupTransform(group, transform) {
    if (!transform) {
      group.visible = false;
      return;
    }
    if (this.layerState.showAxes !== false) {
      group.visible = true;
    }
    this.applyCustomDisplayPose(group, transform.position);
  }

  applyCustomDisplayPose(group, position) {
    group.position.copy(position);
    group.quaternion.identity();
    group.scale.set(1, 1, -1);
  }

  getWorldTransform(frameId) {
    return composeWorldTransform(frameId, this.transformMap, this.cachedWorldTransforms);
  }

  setTheme(theme) {
    this.theme = theme === "light" ? "light" : "dark";
    if (this.theme === "light") {
      this.scene.background = new THREE.Color(0xf3f6fb);
      this.ambientLight.intensity = 1.2;
      this.keyLight.intensity = 0.82;
      this.robotMaterial.color.setHex(0x3d63d8);
      this.tcpToolMaterial.color.setHex(0xd67a2f);
      const materials = Array.isArray(this.grid.material) ? this.grid.material : [this.grid.material];
      if (materials[0]?.color) {
        materials[0].color.setHex(0x7d8da4);
      }
      if (materials[1]?.color) {
        materials[1].color.setHex(0xa6b2c2);
      }
      return;
    }

    this.scene.background = new THREE.Color(0x071018);
    this.ambientLight.intensity = 1.1;
    this.keyLight.intensity = 0.9;
    this.robotMaterial.color.setHex(0x4c87ff);
    this.tcpToolMaterial.color.setHex(0xff8c4d);
    const materials = Array.isArray(this.grid.material) ? this.grid.material : [this.grid.material];
    if (materials[0]?.color) {
      materials[0].color.setHex(0xffffff);
    }
    if (materials[1]?.color) {
      materials[1].color.setHex(0xe8eef7);
    }
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
    const scepterTransform = this.getWorldTransform(SCEPTER_FRAME);
    if (!scepterTransform) {
      return null;
    }

    return {
      x: scepterTransform.position.x * 1000.0,
      y: scepterTransform.position.y * 1000.0,
      z: scepterTransform.position.z * 1000.0,
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
          x: -directRecord.position.x * 1000.0,
          y: -directRecord.position.y * 1000.0,
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
        x: -relative.x * 1000.0,
        y: -relative.y * 1000.0,
        z: relative.z * 1000.0,
      },
      publishedTranslationMeters: {
        x: relative.x,
        y: relative.y,
        z: relative.z,
      },
    };
  }

  convertScepterPointCloudPointToCabinPoint(localPoint) {
    const scepterTransform = this.getWorldTransform(SCEPTER_FRAME);
    if (!scepterTransform) {
      return null;
    }

    // `world_coord` 在当前工程里不是 cabin_frame 下的全局点，
    // 而是 pointAI 使用的相机局部点云图像。pointAI 在接入时会先做 X/Y 对调、
    // Y 反号，再按与后端 tf_transform.py 一致的“索驱高度 - 深度 + gripper_tf.z”
    // 把相机深度映射到 cabin z。
    const pointAiStyleLocalPoint = getPointAiStyleLocalPoint(localPoint);
    const gripperOffsetZ = this.getGripperOffsetRelativeToScepter();

    return new THREE.Vector3(
      scepterTransform.position.x + pointAiStyleLocalPoint.x,
      scepterTransform.position.y + pointAiStyleLocalPoint.y,
      scepterTransform.position.z - pointAiStyleLocalPoint.z + gripperOffsetZ,
    );
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
      const cabinPoint = this.convertScepterPointCloudPointToCabinPoint(point);
      if (!cabinPoint) {
        continue;
      }
      worldPositions.push(cabinPoint.x, cabinPoint.y, cabinPoint.z);
    }
    const target = source === "rawWorldCoord" ? this.rawPointCloud : this.filteredPointCloud;
    target.geometry.setAttribute("position", new THREE.Float32BufferAttribute(worldPositions, 3));
    target.geometry.computeBoundingSphere();
  }

  setTiePointsMessage(message) {
    const points = Array.isArray(message?.PointCoordinatesArray) ? message.PointCoordinatesArray : [];
    const positions = [];
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
      positions.push(vector.x, vector.y, vector.z);
    });
    this.tiePoints.geometry.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
    this.tiePoints.geometry.computeBoundingSphere();
    this.pointCounts.tiePoints = positions.length / 3;
    return this.pointCounts.tiePoints;
  }

  setPlanningMarkersMessage(message) {
    const markers = Array.isArray(message?.markers) ? message.markers : [];
    const positions = [];
    markers.forEach((marker) => {
      const markerPoints = Array.isArray(marker?.points) ? marker.points : [];
      markerPoints.forEach((point) => {
        positions.push(
          Number(point?.x || 0),
          Number(point?.y || 0),
          Number(point?.z || 0),
        );
      });
    });
    this.planningPoints.geometry.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
    this.planningPoints.geometry.computeBoundingSphere();
    this.pointCounts.planningPoints = positions.length / 3;
    return this.pointCounts.planningPoints;
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
