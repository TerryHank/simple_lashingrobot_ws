const ANSI_ESCAPE_PATTERN =
  /[\u001b\u009b][[\]()#;?]*(?:(?:(?:[a-zA-Z\d]*(?:;[a-zA-Z\d]*)*)?\u0007)|(?:(?:\d{1,4}(?:;\d{0,4})*)?[\dA-PR-TZcf-nq-uy=><~]))/g;
const ROS_LOG_HEADER_PATTERN =
  /^\[(?:DEBUG|INFO|WARN|ERROR|FATAL)\]\s+\[[^\]]+\]:\s*/;
const LEGACY_LOG_SOURCE_PREFIX_PATTERN =
  /^(?:(?:Cabin|Moduan|PointAI|pointAI)(?:_(?:log|warn|error|info))?|web_action_bridge):\s*/i;
const LEGACY_LOG_LEVEL_PREFIX_PATTERN =
  /^(?:Cabin|Moduan|PointAI|pointAI)(?:_(log|warn|error|info))?:\s*/i;

function stripTransportLogNoise(value) {
  return String(value ?? "")
    .replace(ANSI_ESCAPE_PATTERN, "")
    .replace(/^\[stdout\]\s*/, "")
    .trim()
    .replace(ROS_LOG_HEADER_PATTERN, "")
    .trim();
}

export function inferLegacyLogLevel(value) {
  const text = stripTransportLogNoise(value);
  const match = text.match(LEGACY_LOG_LEVEL_PREFIX_PATTERN);
  if (!match) {
    return null;
  }
  const tag = String(match[1] || "log").toLowerCase();
  if (tag === "error") {
    return "error";
  }
  if (tag === "warn") {
    return "warn";
  }
  return "info";
}

export function sanitizeRosLogText(value) {
  let text = stripTransportLogNoise(value);
  text = text.replace(LEGACY_LOG_SOURCE_PREFIX_PATTERN, "").trim();
  return text;
}
