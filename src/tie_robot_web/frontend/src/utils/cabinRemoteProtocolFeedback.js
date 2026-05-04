const POSITION_MOVE_STATUS_BITS = [
  [0, "逆解未激活"],
  [1, "电机未全部使能"],
  [2, "设备运动中"],
  [3, "设备报警"],
  [4, "速度错误"],
  [5, "X超正限位"],
  [6, "X超负限位"],
  [7, "Y超正限位"],
  [8, "Y超负限位"],
  [9, "Z超正限位"],
  [10, "Z超负限位"],
  [11, "A超正限位"],
  [12, "A超负限位"],
  [13, "B超正限位"],
  [14, "B超负限位"],
  [15, "C超正限位"],
  [16, "C超负限位"],
];

const STOP_STATUS_BITS = [
  [0, "逆解未激活"],
  [1, "电机未全部使能"],
  [2, "设备未运动"],
  [3, "设备报警"],
];

function extractFrame(text, fieldName) {
  const match = text.match(new RegExp(`${fieldName}=\\[([^\\]]*)\\]`));
  return match ? `[${match[1].trim()}]` : "";
}

function extractToken(text, fieldName) {
  const match = text.match(new RegExp(`${fieldName}=([^，,\\n]+)`));
  return match?.[1]?.trim() || "";
}

function parseHexToken(value) {
  if (!/^0x[0-9a-f]+$/i.test(value)) {
    return null;
  }
  const parsed = Number.parseInt(value.slice(2), 16);
  return Number.isFinite(parsed) ? parsed : null;
}

function getStatusBitsForCommand(commandWord) {
  if (commandWord === 0x0013) {
    return STOP_STATUS_BITS;
  }
  return POSITION_MOVE_STATUS_BITS;
}

function decodeStatusWord(commandWord, statusWord) {
  if (statusWord === 0) {
    return "状态正常";
  }

  const reasons = getStatusBitsForCommand(commandWord)
    .filter(([bit]) => (statusWord & (1 << bit)) !== 0)
    .map(([, label]) => label);
  return reasons.join("；");
}

function resolveProtocolMeaning(text) {
  const explicitReason = extractToken(text, "原因");
  if (explicitReason) {
    return explicitReason;
  }

  if (text.includes("connection closed by peer")) {
    return "索驱 TCP 接收失败，对端关闭连接";
  }

  if (text.includes("read timeout")) {
    return "索驱 TCP 接收超时，未收到完整回包";
  }

  if (text.includes("状态字包含协议未定义高位") || text.includes("疑似错帧")) {
    return "状态字包含协议未定义高位，疑似读到状态查询包或旧回包残留";
  }

  const statusWordText = extractToken(text, "status_word");
  const statusWord = parseHexToken(statusWordText);
  if (statusWord !== null) {
    const commandWord = parseHexToken(extractToken(text, "request_command"))
      ?? parseHexToken(extractToken(text, "command"))
      ?? 0x0012;
    return decodeStatusWord(commandWord, statusWord)
      || `状态字 ${statusWordText} 未匹配到协议错误位`;
  }

  if (text.includes("接收失败")) {
    return "索驱 TCP 接收失败，未收到完整回包";
  }

  if (text.includes("校验失败")) {
    return "索驱协议回包校验失败";
  }

  if (text.includes("回包头不匹配")) {
    return "索驱协议回包头不匹配";
  }

  return "未解析到协议状态字";
}

export function formatCabinRemoteProtocolFeedback(message) {
  const text = String(message || "").trim();
  if (!text) {
    return "";
  }

  const requestFrame = extractFrame(text, "request_frame");
  const responseFrame = extractFrame(text, "response_frame");
  if (!requestFrame && !responseFrame) {
    return text;
  }

  const received = extractToken(text, "received");
  const responseLine = responseFrame
    ? `返回报文：${responseFrame}`
    : received
      ? `返回报文：未收到完整回包（received=${received}）`
      : "返回报文：未收到回包";

  return [
    `发送报文：${requestFrame || "未记录"}`,
    responseLine,
    `返回含义：${resolveProtocolMeaning(text)}`,
  ].join("\n");
}
