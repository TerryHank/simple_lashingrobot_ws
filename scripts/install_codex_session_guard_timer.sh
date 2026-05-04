#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
USER_SYSTEMD_DIR="${XDG_CONFIG_HOME:-$HOME/.config}/systemd/user"
SERVICE_NAME="tie-codex-session-guard.service"
TIMER_NAME="tie-codex-session-guard.timer"
THRESHOLD_MB="${THRESHOLD_MB:-50}"
MIN_AGE_MINUTES="${MIN_AGE_MINUTES:-60}"

if [[ "${ALLOW_CODEX_SESSION_ARCHIVE_TIMER:-0}" != "1" ]]; then
  echo "archive-only Codex session moving is disabled by project preference."
  echo "Use scripts/install_codex_session_summary_timer.sh to copy original content into archive and leave compact summary JSONL in active sessions."
  echo "Only set ALLOW_CODEX_SESSION_ARCHIVE_TIMER=1 if the user explicitly asks to restore archive-only behavior."
  exit 0
fi

mkdir -p "$USER_SYSTEMD_DIR"

cat > "$USER_SYSTEMD_DIR/$SERVICE_NAME" <<EOF
[Unit]
Description=Archive oversized inactive Codex JSONL sessions

[Service]
Type=oneshot
WorkingDirectory=$PROJECT_ROOT
ExecStart=/usr/bin/env python3 $PROJECT_ROOT/scripts/codex_session_guard.py archive --threshold-mb $THRESHOLD_MB --min-age-minutes $MIN_AGE_MINUTES --skip-open --apply
EOF

cat > "$USER_SYSTEMD_DIR/$TIMER_NAME" <<EOF
[Unit]
Description=Run oversized Codex session guard periodically

[Timer]
OnBootSec=5min
OnUnitActiveSec=15min
Persistent=true

[Install]
WantedBy=timers.target
EOF

echo "installed $USER_SYSTEMD_DIR/$SERVICE_NAME"
echo "installed $USER_SYSTEMD_DIR/$TIMER_NAME"

if command -v systemctl >/dev/null 2>&1 && systemctl --user show-environment >/dev/null 2>&1; then
  systemctl --user daemon-reload
  systemctl --user enable --now "$TIMER_NAME"
  systemctl --user list-timers "$TIMER_NAME" --no-pager
else
  echo "user systemd is not available in this shell; enable later with:"
  echo "  systemctl --user daemon-reload"
  echo "  systemctl --user enable --now $TIMER_NAME"
fi
