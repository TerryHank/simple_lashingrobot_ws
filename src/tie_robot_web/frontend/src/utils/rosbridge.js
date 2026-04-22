export function resolveRosbridgeUrl({
  pathname = "/",
  savedPreferences = null,
  currentHost = "localhost",
} = {}) {
  if (pathname && pathname.length > 1) {
    try {
      const decoded = decodeURIComponent(pathname.substring(1));
      if (decoded.startsWith("ws://") || decoded.startsWith("wss://")) {
        return decoded;
      }
    } catch {
      // ignore invalid encoded pathname
    }
  }

  if (currentHost) {
    return `ws://${currentHost}:9090`;
  }

  if (savedPreferences?.ip) {
    return `ws://${savedPreferences.ip}:${savedPreferences.port || "9090"}`;
  }

  return "ws://localhost:9090";
}
