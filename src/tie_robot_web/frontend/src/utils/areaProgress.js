function normalizePositiveInteger(value) {
  const numberValue = Number(value);
  if (!Number.isFinite(numberValue) || numberValue <= 0) {
    return null;
  }
  return Math.round(numberValue);
}

export function formatAreaProgressDisplay(progress) {
  const total = normalizePositiveInteger(progress?.total_area_count);
  const current = normalizePositiveInteger(progress?.current_area_index);
  if (!total || (!current && !progress?.all_done)) {
    return {
      label: "区域 --/--",
      title: "等待区域进度上报",
      state: "waiting",
    };
  }

  const safeCurrent = Math.min(current || total, total);
  const label = `区域 ${safeCurrent}/${total}`;
  if (progress?.all_done) {
    return {
      label,
      title: `全部 ${total} 个区域已完成`,
      state: "done",
    };
  }

  const justFinished = normalizePositiveInteger(progress?.just_finished_area_index);
  if (progress?.ready_for_next_area && justFinished) {
    return {
      label,
      title: `第 ${justFinished} 个区域已完成，等待进入第 ${safeCurrent} 个区域`,
      state: "ready",
    };
  }

  return {
    label,
    title: `当前执行到第 ${safeCurrent} 个区域，共 ${total} 个区域`,
    state: "live",
  };
}
