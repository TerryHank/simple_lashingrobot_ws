#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
HELP_DIR="$ROOT_DIR/src/tie_robot_web/help"
DEPLOY_DIR="$ROOT_DIR/.deploy/github-pages"
PUBLISH_DIR="$ROOT_DIR/.worktrees/gh-pages"
REPO_SLUG="TerryHank/simple_lashingrobot_ws"
PAGES_BRANCH="gh-pages"

echo "[github-pages] build help site"
cd "$HELP_DIR"
npm run build:github

cd "$ROOT_DIR"
mkdir -p "$(dirname "$PUBLISH_DIR")"

if [ -d "$PUBLISH_DIR" ]; then
  git worktree remove "$PUBLISH_DIR" --force
fi

git worktree prune
rm -rf "$PUBLISH_DIR"

git worktree add --detach "$PUBLISH_DIR"
git -C "$PUBLISH_DIR" checkout --orphan "$PAGES_BRANCH"
git -C "$PUBLISH_DIR" reset --hard
find "$PUBLISH_DIR" -mindepth 1 -maxdepth 1 ! -name '.git' -exec rm -rf {} +
cp -a "$DEPLOY_DIR"/. "$PUBLISH_DIR"/

touch "$PUBLISH_DIR/.nojekyll"

cat > "$PUBLISH_DIR/README.md" <<'EOF'
# GitHub Pages Publishing Branch

This branch is generated from `src/tie_robot_web/help`.
Do not edit files here manually in normal development.
EOF

cd "$PUBLISH_DIR"
git add -A

if git diff --cached --quiet; then
  echo "[github-pages] no publishable changes"
else
  git commit -m "docs: deploy help site to GitHub Pages"
  git push -u origin "$PAGES_BRANCH" --force
fi

if gh api "repos/$REPO_SLUG/pages" >/dev/null 2>&1; then
  gh api --method PUT "repos/$REPO_SLUG/pages" -f source[branch]="$PAGES_BRANCH" -f source[path]="/"
else
  gh api --method POST "repos/$REPO_SLUG/pages" -f source[branch]="$PAGES_BRANCH" -f source[path]="/"
fi

echo "[github-pages] requested publish source: $PAGES_BRANCH /"
echo "[github-pages] expected URL: https://terryhank.github.io/simple_lashingrobot_ws/"
