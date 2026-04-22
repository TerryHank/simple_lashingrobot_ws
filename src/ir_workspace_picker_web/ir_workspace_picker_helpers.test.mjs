import test from 'node:test';
import assert from 'node:assert/strict';

import {
  applyGammaToMono8,
  buildWorkspaceQuadPayload,
  clampImagePixel,
  enhanceMono8,
  equalizeMono8,
  findDraggablePointIndex,
  findPercentileIntensity,
  mapCanvasClickToImagePixel,
  parseWorkspaceQuadPayload,
  replaceWorkspacePoint,
  resolveRosbridgeUrl,
  stretchMono8Contrast,
} from './ir_workspace_picker_helpers.mjs';

test('resolveRosbridgeUrl prefers explicit ws pathname', () => {
  const url = resolveRosbridgeUrl({
    pathname: '/ws%3A%2F%2F192.168.1.8%3A9090',
    savedPreferences: { ip: '10.0.0.2', port: '9090' },
    currentHost: 'localhost',
  });

  assert.equal(url, 'ws://192.168.1.8:9090');
});

test('resolveRosbridgeUrl prefers current page host over stale saved connection preferences', () => {
  const url = resolveRosbridgeUrl({
    pathname: '/',
    savedPreferences: { ip: '10.0.0.2', port: '9091' },
    currentHost: '127.0.0.1',
  });

  assert.equal(url, 'ws://127.0.0.1:9090');
});

test('resolveRosbridgeUrl falls back to saved connection preferences when current host is unavailable', () => {
  const url = resolveRosbridgeUrl({
    pathname: '/',
    savedPreferences: { ip: '10.0.0.2', port: '9091' },
    currentHost: '',
  });

  assert.equal(url, 'ws://10.0.0.2:9091');
});

test('mapCanvasClickToImagePixel scales click coordinates back to source pixels', () => {
  const pixel = mapCanvasClickToImagePixel({
    clientX: 210,
    clientY: 110,
    rect: { left: 10, top: 10, width: 320, height: 240 },
    imageWidth: 640,
    imageHeight: 480,
  });

  assert.deepEqual(pixel, { x: 400, y: 200 });
});

test('clampImagePixel keeps mapped pixels inside image bounds', () => {
  assert.deepEqual(clampImagePixel({ x: -3, y: 999 }, 640, 480), { x: 0, y: 479 });
  assert.deepEqual(clampImagePixel({ x: 641, y: -10 }, 640, 480), { x: 639, y: 0 });
});

test('buildWorkspaceQuadPayload rounds four clicked points into Float32 array order', () => {
  const payload = buildWorkspaceQuadPayload([
    { x: 10.4, y: 20.6 },
    { x: 30.5, y: 40.4 },
    { x: 50.4, y: 60.6 },
    { x: 70.5, y: 80.5 },
  ]);

  assert.deepEqual(payload, [10, 21, 31, 40, 50, 61, 71, 81]);
});

test('buildWorkspaceQuadPayload rejects non-four-point input', () => {
  assert.throws(() => buildWorkspaceQuadPayload([{ x: 1, y: 2 }]), /exactly four/i);
});

test('parseWorkspaceQuadPayload converts four point float array back to point list', () => {
  const points = parseWorkspaceQuadPayload([10, 20, 30, 40, 50, 60, 70, 80]);

  assert.deepEqual(points, [
    { x: 10, y: 20 },
    { x: 30, y: 40 },
    { x: 50, y: 60 },
    { x: 70, y: 80 },
  ]);
});

test('findDraggablePointIndex returns the nearest selected point inside hit radius', () => {
  const points = [
    { x: 100, y: 80 },
    { x: 180, y: 120 },
    { x: 240, y: 160 },
  ];

  assert.equal(findDraggablePointIndex(points, { x: 108, y: 86 }, 12), 0);
  assert.equal(findDraggablePointIndex(points, { x: 173, y: 128 }, 12), 1);
  assert.equal(findDraggablePointIndex(points, { x: 300, y: 300 }, 12), -1);
});

test('replaceWorkspacePoint returns a new point array with only the targeted corner replaced', () => {
  const points = [
    { x: 100, y: 80 },
    { x: 180, y: 120 },
    { x: 240, y: 160 },
  ];

  const updated = replaceWorkspacePoint(points, 1, { x: 200, y: 140 });

  assert.deepEqual(updated, [
    { x: 100, y: 80 },
    { x: 200, y: 140 },
    { x: 240, y: 160 },
  ]);
  assert.notEqual(updated, points);
});

test('findPercentileIntensity returns stable low and high clip intensities', () => {
  const bytes = new Uint8ClampedArray([10, 10, 10, 20, 20, 30, 200, 220, 240, 250]);

  assert.equal(findPercentileIntensity(bytes, 0), 10);
  assert.equal(findPercentileIntensity(bytes, 50), 20);
  assert.equal(findPercentileIntensity(bytes, 90), 240);
});

test('stretchMono8Contrast expands a narrow dark band into visible contrast', () => {
  const bytes = new Uint8ClampedArray([12, 14, 16, 18, 20, 22]);
  const stretched = stretchMono8Contrast(bytes, { lowClipPercent: 0, highClipPercent: 100 });

  assert.deepEqual(Array.from(stretched), [0, 51, 102, 153, 204, 255]);
});

test('applyGammaToMono8 with gamma below one brightens mid tones', () => {
  const bytes = new Uint8ClampedArray([0, 64, 128, 255]);
  const corrected = applyGammaToMono8(bytes, 0.5);

  assert.deepEqual(Array.from(corrected), [0, 128, 181, 255]);
});

test('equalizeMono8 increases separation for clustered values', () => {
  const bytes = new Uint8ClampedArray([10, 10, 10, 20, 20, 200]);
  const equalized = equalizeMono8(bytes);

  assert.equal(equalized[0], 0);
  assert.ok(equalized[3] > equalized[0]);
  assert.equal(equalized[5], 255);
});

test('enhanceMono8 auto mode brightens a dark mono frame without changing size', () => {
  const bytes = new Uint8ClampedArray([8, 9, 10, 11, 12, 13, 14, 15]);
  const enhanced = enhanceMono8(bytes, { mode: 'auto', gamma: 0.85 });

  assert.equal(enhanced.length, bytes.length);
  assert.ok(enhanced[4] > bytes[4]);
});

test('enhanceMono8 raw mode returns the original mono frame', () => {
  const bytes = new Uint8ClampedArray([1, 2, 3, 4]);
  const enhanced = enhanceMono8(bytes, { mode: 'raw', gamma: 0.85 });

  assert.deepEqual(Array.from(enhanced), [1, 2, 3, 4]);
});
