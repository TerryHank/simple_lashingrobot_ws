import assert from "node:assert/strict";

import { formatAreaProgressDisplay } from "../src/utils/areaProgress.js";

assert.deepEqual(formatAreaProgressDisplay(null), {
  label: "区域 --/--",
  title: "等待区域进度上报",
  state: "waiting",
});

assert.deepEqual(formatAreaProgressDisplay({ current_area_index: 2, total_area_count: 8 }), {
  label: "区域 2/8",
  title: "当前执行到第 2 个区域，共 8 个区域",
  state: "live",
});

assert.deepEqual(
  formatAreaProgressDisplay({
    current_area_index: 8,
    total_area_count: 8,
    just_finished_area_index: 8,
    all_done: true,
  }),
  {
    label: "区域 8/8",
    title: "全部 8 个区域已完成",
    state: "done",
  },
);
