import assert from "node:assert/strict";

import { inferLegacyLogLevel, sanitizeRosLogText } from "../src/utils/logText.js";

assert.equal(
  sanitizeRosLogText("\u001b[0m[INFO] [1777400623.639808751]: Cabin_log: 修改索驱速度指令，速度为: 300.00\u001b[0m"),
  "修改索驱速度指令，速度为: 300.00",
);

assert.equal(
  sanitizeRosLogText("[stdout] \u001b[32mCabin_log: 已连接\u001b[0m"),
  "已连接",
);

assert.equal(
  sanitizeRosLogText(" \u001b[1;31mCabin_Error: 故障\u001b[0K "),
  "故障",
);

assert.equal(
  sanitizeRosLogText("[ERROR] [1777401219.815650560]: Cabin_log: pseudo_slam棋盘格成员过滤：无法融入棋盘格行列邻接的点视为离群点，移除1个点，剩余158个规划点。"),
  "pseudo_slam棋盘格成员过滤：无法融入棋盘格行列邻接的点视为离群点，移除1个点，剩余158个规划点。",
);

assert.equal(
  inferLegacyLogLevel("[ERROR] [1777401219.815650560]: Cabin_log: pseudo_slam棋盘格成员过滤：无法融入棋盘格行列邻接的点视为离群点，移除1个点，剩余158个规划点。"),
  "info",
);

assert.equal(inferLegacyLogLevel("Cabin_Warn: 索驱暂停"), "warn");
assert.equal(inferLegacyLogLevel("Moduan_Error: 故障"), "error");
