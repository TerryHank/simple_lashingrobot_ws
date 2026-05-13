#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
HELP_DIR="$ROOT_DIR/src/tie_robot_web/help"
DEPLOY_DIR="$ROOT_DIR/.deploy/cloudflare-pages"
PROJECT_NAME="${CLOUDFLARE_PAGES_PROJECT_NAME:-tie-robot-help}"
BRANCH_NAME="${CLOUDFLARE_PAGES_BRANCH:-main}"

if [ -z "${CLOUDFLARE_API_TOKEN:-}" ]; then
  echo "CLOUDFLARE_API_TOKEN is required."
  echo "Set CLOUDFLARE_API_TOKEN and optionally CLOUDFLARE_PAGES_PROJECT_NAME / CLOUDFLARE_PAGES_BRANCH."
  exit 1
fi

echo "[cloudflare-pages] build help site"
cd "$HELP_DIR"
npm run build:cloudflare

cd "$ROOT_DIR"
npx --yes wrangler@latest pages deploy "$DEPLOY_DIR" \
  --project-name "$PROJECT_NAME" \
  --branch "$BRANCH_NAME" \
  --commit-dirty true
