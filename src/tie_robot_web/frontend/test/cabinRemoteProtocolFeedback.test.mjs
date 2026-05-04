import assert from "node:assert/strict";

import { CabinRemoteController } from "../src/controllers/CabinRemoteController.js";

function createControllerWithServiceMessage(message) {
  return new CabinRemoteController({
    rosConnection: {
      async callCabinIncrementalMoveService() {
        return { success: false, message };
      },
    },
    sceneView: {
      getCurrentCabinPositionMm() {
        return { x: 0, y: 0, z: 0 };
      },
    },
  });
}

const peerClosedMessage = "索驱TCP相对位置运动驱动下发失败，索驱 TCP 接收失败，detail=read response: connection closed by peer，received=0/8，request_command=0x0012，request_frame=[EB 90 00 12 02 00 00 00 C8 42 00 00 00 00 00 00 00 00 00 00 C8 C2 00 00 00 00 00 00 00 00 00 00 00 00 23 04]";

const peerClosedResult = await createControllerWithServiceMessage(peerClosedMessage)
  .move("xPositive", { step: 50, speed: 300, moveMode: "relative" });

assert.equal(
  peerClosedResult.message,
  [
    "发送报文：[EB 90 00 12 02 00 00 00 C8 42 00 00 00 00 00 00 00 00 00 00 C8 C2 00 00 00 00 00 00 00 00 00 00 00 00 23 04]",
    "返回报文：未收到完整回包（received=0/8）",
    "返回含义：索驱 TCP 接收失败，对端关闭连接",
  ].join("\n"),
);

const rejectedMessage = "索驱TCP相对位置运动驱动下发失败，索驱上位机拒绝当前运动指令，detail=raw_status_le32=0xC3820000，status_word=0x000082C3，status_source=be32，原因=逆解未激活；电机未全部使能；设备运动中；X超负限位，response_frame=[EB 90 00 00 82 C3 A8 02]，request_command=0x0012，request_frame=[EB 90 00 12 02 00 00 00 C8 42 00 00 48 42 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 51 03]";

const rejectedResult = await createControllerWithServiceMessage(rejectedMessage)
  .move("yPositive", { step: 50, speed: 300, moveMode: "relative" });

assert.equal(
  rejectedResult.message,
  [
    "发送报文：[EB 90 00 12 02 00 00 00 C8 42 00 00 48 42 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 51 03]",
    "返回报文：[EB 90 00 00 82 C3 A8 02]",
    "返回含义：逆解未激活；电机未全部使能；设备运动中；X超负限位",
  ].join("\n"),
);
