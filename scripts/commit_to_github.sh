#!/usr/bin/env bash
set -euo pipefail

if [[ "${1:-}" != "--robot-verified" ]]; then
  echo "用法：$0 --robot-verified [提交说明]" >&2
  echo "提交前必须先按 README 在机器人端完成同步、Release、CTest、ldd、服务和零运动验收。" >&2
  exit 2
fi
shift

commit_message="${1:-feat: sync authoritative robot workspace updates}"
expected_branch="main"
expected_remote="origin"
expected_repo="ershui2500/UniRoboGui"

project_root="$(git rev-parse --show-toplevel 2>/dev/null || true)"
if [[ -z "$project_root" ]]; then
  echo "错误：当前目录不在 Git 仓库中。" >&2
  exit 1
fi
cd "$project_root"

if git ls-files --error-unmatch AGENTS.md >/dev/null 2>&1; then
  echo "错误：AGENTS.md 仅用于开发规则，禁止被 Git 跟踪或发布。" >&2
  exit 1
fi

if ! command -v gh >/dev/null 2>&1; then
  echo "错误：未找到 GitHub CLI（gh）。" >&2
  exit 1
fi
if ! gh auth status --hostname github.com >/dev/null 2>&1; then
  echo "错误：GitHub CLI 尚未登录 github.com。" >&2
  exit 1
fi

branch="$(git branch --show-current)"
if [[ "$branch" != "$expected_branch" ]]; then
  echo "错误：当前分支为 $branch，预期为 $expected_branch。" >&2
  exit 1
fi

remote_url="$(git remote get-url "$expected_remote")"
case "$remote_url" in
  "https://github.com/${expected_repo}.git"|"https://github.com/${expected_repo}"|"git@github.com:${expected_repo}.git") ;;
  *)
    echo "错误：origin 指向意外仓库：$remote_url" >&2
    exit 1
    ;;
esac

echo "[1/7] 更新远端分支信息"
git fetch --prune "$expected_remote" "$expected_branch"

if ! git merge-base --is-ancestor "$expected_remote/$expected_branch" HEAD; then
  echo "错误：远端 $expected_remote/$expected_branch 包含本地尚未合并的提交，请先处理分支差异。" >&2
  exit 1
fi

echo "[2/7] 审核待提交路径"
mapfile -t changed_paths < <(git diff --name-only HEAD)
if [[ ${#changed_paths[@]} -eq 0 ]]; then
  echo "没有已跟踪文件修改。"
fi

for path in "${changed_paths[@]}"; do
  case "$path" in
    README.md|README.en.md|LICENSE|VERSION|CMakeLists.txt|.gitignore|docs/*|include/*|src/*|tests/*|web/*|scripts/*|deploy/*)
      ;;
    *)
      echo "错误：发现发布范围外的已跟踪修改：$path" >&2
      exit 1
      ;;
  esac
done

while IFS= read -r path; do
  [[ -z "$path" ]] && continue
  case "$path" in
    README.md|README.en.md|LICENSE|VERSION|CMakeLists.txt|.gitignore|docs/*|include/*|src/*|tests/*|web/*|scripts/*|deploy/*)
      ;;
    .robot-backups/*|build/*|Testing/*|logs/*|config/customer_voice.json|config/customer_voice.json.tmp|config/joint_teach_actions.json|config/joint_teach_actions.json.tmp)
      ;;
    *)
      echo "错误：发现未审核的未跟踪文件：$path" >&2
      exit 1
      ;;
  esac
done < <(git ls-files --others --exclude-standard)

echo "[3/7] 检查前端脚本语法"
for file in \
  web/i18n.js \
  web/app.js \
  web/workspace.js \
  web/imu-gauges.js \
  web/robot-viewer.js \
  web/perception.js \
  web/joint-debug.js \
  web/nav-renderer.js \
  web/nav-store.js; do
  node --check "$file"
done

echo "[4/7] 确认机器人端验收"
echo "已通过 --robot-verified 确认机器人权威源码已完成 Release、CTest、ldd、服务和零运动验收。"

echo "[5/7] 暂存受管源码"
git add -- \
  .gitignore README.md README.en.md LICENSE VERSION CMakeLists.txt \
  docs include src tests web scripts deploy

for path in $(git diff --cached --name-only); do
  lowered="${path,,}"
  case "$lowered" in
    *password*|*passwd*|*credential*|*secret*|*token*|*cookie*|*.pem|*.key|*id_rsa*)
      echo "错误：暂存区含疑似凭据文件：$path" >&2
      git reset --quiet
      exit 1
      ;;
  esac
done

git diff --cached --check
if git diff --cached --quiet; then
  echo "没有可提交的变更。"
  exit 0
fi

echo "即将提交："
git diff --cached --stat

version="$(tr -d '[:space:]' < VERSION)"
if [[ ! "$version" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
  echo "错误：VERSION 不是合法语义版本：$version" >&2
  git reset --quiet
  exit 1
fi
tag="v${version}"
if git rev-parse -q --verify "refs/tags/$tag" >/dev/null || \
   git ls-remote --exit-code --tags "$expected_remote" "refs/tags/$tag" >/dev/null 2>&1; then
  echo "错误：版本标签已存在：$tag" >&2
  git reset --quiet
  exit 1
fi

echo "[6/7] 创建 Git 提交和版本标签"
git commit -m "$commit_message"
commit_sha="$(git rev-parse HEAD)"
git tag -a "$tag" -m "UniRoboGui $tag"

echo "[7/7] 原子推送 main 和 $tag 到 GitHub"
git push --atomic "$expected_remote" "$expected_branch" "$tag"

echo "提交完成：$commit_sha"
echo "版本标签：$tag"
gh repo view "$expected_repo" --json nameWithOwner,url,defaultBranchRef \
  --jq '"GitHub 仓库：\(.nameWithOwner)\n地址：\(.url)\n默认分支：\(.defaultBranchRef.name)"'
