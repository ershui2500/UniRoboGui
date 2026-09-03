#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "$PROJECT_DIR/scripts/deploy_i18n.sh"
ROBOT="unitree@192.168.123.164"
CACHE_ROOT="${XDG_CACHE_HOME:-$HOME/.cache}/unirobogui-deploy"
SDK2_CACHE="$CACHE_ROOT/unitree_sdk2"
REALSENSE_CACHE="$CACHE_ROOT/librealsense"
MODEL_NAME="kokoro-int8-multi-lang-v1_1"
MODEL_ARCHIVE="$CACHE_ROOT/${MODEL_NAME}.tar.bz2"
WHEELHOUSE="$CACHE_ROOT/wheelhouse-aarch64-py38"
SHERPA_VERSION="1.13.4"
REMOTE_PROJECT_DIR="/home/unitree/UniRoboGui"
REMOTE_PROJECT_MARKER="$REMOTE_PROJECT_DIR/.unirobogui-managed"
PROJECT_SYNC_DELETE=false
STEP_ZH=""
STEP_EN=""
LAST_HELP_ZH=""
LAST_HELP_EN=""

usage() {
  if [[ "$UNIROBOGUI_LANG" == "en" ]]; then
    cat <<EOF
Usage:
  bash scripts/deploy_g1_from_pc.en.sh [--robot USER@HOST]

Default robot:
  $ROBOT

Examples:
  bash scripts/deploy_g1_from_pc.en.sh
  bash scripts/deploy_g1_from_pc.en.sh --robot unitree@192.168.123.164

Run this script on an Ubuntu/Linux computer that can access GitHub.
The computer must also be able to reach the G1 PC2 over SSH.
EOF
  else
    cat <<EOF
用法：
  bash scripts/deploy_g1_from_pc.sh [--robot USER@HOST]

默认机器人：
  $ROBOT

示例：
  bash scripts/deploy_g1_from_pc.sh
  bash scripts/deploy_g1_from_pc.sh --robot unitree@192.168.123.164

该脚本必须运行在能访问 GitHub 的 Ubuntu/Linux 电脑上。
电脑还需要能通过 SSH 访问 G1 PC2。
EOF
  fi
}

while (($#)); do
  case "$1" in
    --robot)
      (($# >= 2)) || { ui_err "[错误] --robot 缺少 USER@HOST" "[ERROR] --robot requires USER@HOST"; exit 2; }
      ROBOT="$2"
      shift
      ;;
    -h|--help) usage; exit 0 ;;
    *) ui_err "[错误] 未知参数：$1" "[ERROR] Unknown option: $1"; usage >&2; exit 2 ;;
  esac
  shift
done

line() { printf '%*s\n' 72 '' | tr ' ' '='; }
info() { ui_line "[信息] $1" "[INFO] ${2:-$1}"; }
ok() { ui_line "[成功] $1" "[OK] ${2:-$1}"; }
warn() { ui_err "[警告] $1" "[WARN] ${2:-$1}"; }
help_customer() {
  LAST_HELP_ZH="$1"
  LAST_HELP_EN="${2:-$1}"
  ui_err "[需要客户协助] $1" "[CUSTOMER ACTION REQUIRED] ${2:-$1}"
}
step() {
  STEP_ZH="$1"
  STEP_EN="${2:-$1}"
  LAST_HELP_ZH=""
  LAST_HELP_EN=""
  echo
  line
  ui_line "[步骤] $1" "[STEP] ${2:-$1}"
  line
}
die() {
  ui_err "[错误] $1" "[ERROR] ${2:-$1}"
  if [[ -n "${LAST_HELP_ZH}" || -n "${LAST_HELP_EN}" ]]; then
    ui_err "[需要客户协助] ${LAST_HELP_ZH}" "[CUSTOMER ACTION REQUIRED] ${LAST_HELP_EN}"
  fi
  exit 1
}
on_error() {
  local rc=$?
  echo >&2
  line >&2
  ui_err "[部署失败] 当前步骤：${STEP_ZH:-未知}" "[DEPLOYMENT FAILED] Current step: ${STEP_EN:-unknown}"
  ui_err "[部署失败] 命令：${BASH_COMMAND}" "[DEPLOYMENT FAILED] Command: ${BASH_COMMAND}"
  ui_err "[部署失败] 行号：${BASH_LINENO[0]:-未知}，退出码：${rc}" "[DEPLOYMENT FAILED] Line: ${BASH_LINENO[0]:-unknown}, exit code: ${rc}"
  if [[ -n "${LAST_HELP_ZH}" || -n "${LAST_HELP_EN}" ]]; then
    ui_err "[需要客户协助] ${LAST_HELP_ZH}" "[CUSTOMER ACTION REQUIRED] ${LAST_HELP_EN}"
  fi
  ui_err "[提示] 已下载的依赖保存在 $CACHE_ROOT；修复问题后重新运行会复用缓存。" "[TIP] Downloaded dependencies are cached in $CACHE_ROOT; fix the issue and rerun to reuse them."
  line >&2
  exit "$rc"
}
trap on_error ERR

github_available() {
  if command -v curl >/dev/null 2>&1; then
    curl -IL --connect-timeout 5 --max-time 10 -fsS https://github.com/ >/dev/null 2>&1
  elif command -v git >/dev/null 2>&1; then
    git ls-remote https://github.com/unitreerobotics/unitree_sdk2.git HEAD >/dev/null 2>&1
  else
    return 1
  fi
}

require_local_tools() {
  local missing=()
  local cmd
  for cmd in git curl ssh rsync python3; do
    command -v "$cmd" >/dev/null 2>&1 || missing+=("$cmd")
  done
  (("${#missing[@]}" == 0)) ||
    die "联网电脑缺少命令：${missing[*]}。请先安装 git curl openssh-client rsync python3 python3-pip。" "The online computer is missing commands: ${missing[*]}. Install git curl openssh-client rsync python3 python3-pip first."
  python3 -m pip --version >/dev/null 2>&1 ||
    die "联网电脑没有可用的 python3 -m pip。请先安装 python3-pip。" "python3 -m pip is unavailable on the online computer. Install python3-pip first."
  if [[ ! -t 0 ]]; then
    warn "当前不是交互终端。若 SSH 或机器人 sudo 需要输入密码，最终安装步骤会失败；正常客户部署请在可交互的终端中运行本脚本。" "This is not an interactive terminal. If SSH or robot sudo requires a password, the final install step will fail. Run normal deployments from an interactive terminal."
  fi
}

require_clean_repo() {
  local name="$1" dir="$2" expected="$3"
  [[ -d "$dir/.git" ]] || die "$name 目录不是 Git 仓库：$dir" "$name is not a Git repository: $dir"
  local remote dirty
  remote="$(git -C "$dir" remote get-url origin 2>/dev/null || true)"
  [[ "$remote" == *"$expected"* ]] ||
    die "$name 的 origin 不是预期仓库：$remote" "$name origin is not the expected repository: $remote"
  dirty="$(git -C "$dir" status --short)"
  [[ -z "$dirty" ]] || {
    echo "$dirty" >&2
    die "$name 存在本地修改。为了避免自动覆盖，请先提交/备份修改后再运行。" "$name has local modifications. Commit or back them up before rerunning to avoid automatic overwrite."
  }
}

update_or_clone() {
  local name="$1" url="$2" dir="$3" expected="$4"
  if [[ -e "$dir" ]]; then
    info "复用缓存并更新 $name：$dir" "Reusing the cache and updating $name: $dir"
    require_clean_repo "$name" "$dir" "$expected"
    git -C "$dir" pull --ff-only
  else
    info "首次下载 $name..." "Downloading $name for the first time..."
    git clone --depth 1 "$url" "$dir"
  fi
  ok "$name 当前提交：$(git -C "$dir" rev-parse --short HEAD)" "$name current commit: $(git -C "$dir" rev-parse --short HEAD)"
}

remote_preflight() {
  help_customer "如果 SSH 连接失败，请把联网电脑接入机器人的 192.168.123.0/24 网络，确认机器人 IP，并确保能执行：ssh $ROBOT。首次连接需要确认主机指纹，密码请只在 SSH 提示中输入。" "If SSH fails, connect this computer to the robot's 192.168.123.0/24 network, verify the robot IP, and make sure this works: ssh $ROBOT. On first connection, confirm the host fingerprint and enter passwords only at the SSH prompt."
  ssh -o ConnectTimeout=8 "$ROBOT" '
    set -eu
    test "$(id -un)" = unitree
    test "$(uname -m)" = aarch64
    . /etc/os-release
    test "$ID" = ubuntu
    test "$VERSION_ID" = 20.04
    test -z "${ROS_DISTRO:-}"
    test -z "${RMW_IMPLEMENTATION:-}"
    test -z "${CYCLONEDDS_URI:-}"
    ip link show dev eth0 >/dev/null
  '
  if [[ ! -t 0 ]] && ! ssh "$ROBOT" 'sudo -n true' >/dev/null 2>&1; then
    help_customer "当前联网电脑没有交互 TTY，而机器人 sudo 需要密码。请在可交互终端中重新运行；或者仅在受控自动化环境中预先配置免密 sudo。" "This computer has no interactive TTY but robot sudo requires a password. Rerun in an interactive terminal, or configure passwordless sudo only in a controlled automation environment."
    die "非交互终端无法完成机器人 sudo 密码认证。" "A non-interactive terminal cannot complete robot sudo password authentication."
  fi
  ok "SSH 已连接机器人，G1 PC2 基础环境检查通过。" "SSH connected to the robot and the G1 PC2 base environment check passed."
}

prepare_remote_target() {
  local path="$1" name="$2" dependency="$3" expected="${4:-}"
  ssh "$ROBOT" "test -e '$path'" || return 0

  if [[ "$dependency" == "true" ]]; then
    local needs_backup=false dirty=""
    if ! ssh "$ROBOT" "test -d '$path/.git'"; then
      warn "机器人端 $name 不是 Git 仓库，将先完整备份再传入最新官方源码。" "Robot-side $name is not a Git repository; it will be fully backed up before the latest official source is transferred."
      needs_backup=true
    else
      local remote
      remote="$(ssh "$ROBOT" "git -C '$path' remote get-url origin 2>/dev/null" || true)"
      if [[ -n "$expected" && "$remote" != *"$expected"* ]]; then
        warn "机器人端 $name 的 origin 不是预期官方仓库：$remote" "Robot-side $name origin is not the expected official repository: $remote"
        needs_backup=true
      fi
      dirty="$(ssh "$ROBOT" "git -C '$path' status --short" || true)"
      if [[ -n "$dirty" ]]; then
        warn "机器人端 $name 有现场修改/删除，将先完整备份再传入最新官方源码。" "Robot-side $name has local modifications/deletions; it will be fully backed up before the latest official source is transferred."
        echo "$dirty" | head -40 >&2
        needs_backup=true
      fi
    fi
    if $needs_backup; then
      local stamp
      stamp="$(date +%Y%m%d-%H%M%S)"
      ssh "$ROBOT" "mkdir -p /home/unitree/.unirobogui-deploy-backups && mv '$path' '/home/unitree/.unirobogui-deploy-backups/$(basename "$path")-$stamp'"
      ok "$name 旧源码已备份到机器人 /home/unitree/.unirobogui-deploy-backups/" "Previous $name source was backed up on the robot under /home/unitree/.unirobogui-deploy-backups/"
    fi
    return 0
  fi

  if ssh "$ROBOT" "test -d '$path/.git'"; then
    local dirty
    dirty="$(ssh "$ROBOT" "git -C '$path' status --short" || true)"
    if [[ -n "$dirty" ]]; then
      echo "$dirty" >&2
      die "机器人端 $name 项目源码有未提交修改。为避免覆盖业务代码，请先备份/提交后重试。" "Robot-side $name project source has uncommitted changes. Back up or commit them before retrying to avoid overwriting project code."
    fi
    PROJECT_SYNC_DELETE=true
  elif ssh "$ROBOT" "test -f '$REMOTE_PROJECT_MARKER'"; then
    info "检测到方式 B 管理的项目目录；升级时会删除已从新版本移除的旧项目文件，同时保留 build/ 和客户配置。" "A Method B managed project directory was detected. Upgrade will remove project files deleted by newer versions while preserving build/ and customer configuration."
    PROJECT_SYNC_DELETE=true
  else
    info "机器人项目目录不是 Git 仓库，且没有方式 B 管理标记；本次按兼容模式同步，不删除额外文件。成功后会写入管理标记，后续升级可安全清理旧项目文件。" "The robot project directory is not a Git repository and has no Method B management marker. This run uses compatibility sync without deleting extra files. A management marker will be written after success so future upgrades can safely remove obsolete project files."
  fi
}

step "1/8 检测联网电脑能否访问 GitHub" "1/8 Check GitHub access from the online computer"
if github_available; then
  ok "联网电脑可以访问 GitHub。" "The online computer can access GitHub."
else
  help_customer "请先解决这台电脑访问 GitHub 的问题（网络、代理、DNS 或公司网络策略），确认浏览器/命令行可以打开 github.com 后再运行。" "Fix GitHub access on this computer first (network, proxy, DNS, or corporate network policy), and verify github.com is reachable from the browser/command line before rerunning."
  die "当前电脑无法访问 GitHub，无法保证下载到官方最新 SDK2/librealsense。" "This computer cannot access GitHub, so the latest official SDK2/librealsense sources cannot be guaranteed."
fi

step "2/8 检测联网电脑工具" "2/8 Check tools on the online computer"
require_local_tools
mkdir -p "$CACHE_ROOT" "$WHEELHOUSE"
ok "git/curl/ssh/rsync/python3/pip 均可用。缓存目录：$CACHE_ROOT" "git/curl/ssh/rsync/python3/pip are available. Cache directory: $CACHE_ROOT"

step "3/8 提前检测 SSH 到机器人" "3/8 Preflight SSH to the robot"
remote_preflight

step "4/8 更新 UniRoboGui、SDK2 和 librealsense 源码" "4/8 Update UniRoboGui, SDK2, and librealsense source"
[[ -d "$PROJECT_DIR/.git" ]] || die "请从 GitHub clone UniRoboGui 后，在仓库根目录运行本脚本。当前项目没有 .git。" "Clone UniRoboGui from GitHub and run this script from the repository root. The current project has no .git directory."
require_clean_repo "UniRoboGui" "$PROJECT_DIR" "ershui2500/UniRoboGui"
info "更新 UniRoboGui 到 origin 当前版本..." "Updating UniRoboGui to the current origin revision..."
git -C "$PROJECT_DIR" pull --ff-only
ok "UniRoboGui 当前提交：$(git -C "$PROJECT_DIR" rev-parse --short HEAD)" "UniRoboGui current commit: $(git -C "$PROJECT_DIR" rev-parse --short HEAD)"
update_or_clone "Unitree SDK2"   "https://github.com/unitreerobotics/unitree_sdk2.git"   "$SDK2_CACHE" "unitreerobotics/unitree_sdk2"
update_or_clone "librealsense2"   "https://github.com/realsenseai/librealsense.git"   "$REALSENSE_CACHE" "realsenseai/librealsense"

step "5/8 下载 Kokoro 模型和 G1 AArch64/Python 3.8 wheelhouse" "5/8 Download the Kokoro model and G1 AArch64/Python 3.8 wheelhouse"
if [[ -s "$MODEL_ARCHIVE" ]] && tar -tjf "$MODEL_ARCHIVE" >/dev/null 2>&1; then
  ok "复用已下载且校验通过的 Kokoro 模型压缩包。" "Reusing the downloaded and validated Kokoro model archive."
else
  info "下载 Kokoro 中英多语言 INT8 模型..." "Downloading the Kokoro Chinese/English multilingual INT8 model..."
  help_customer "如果模型下载失败，请确认 GitHub Release 也能访问，而不仅是 github.com 首页。删除损坏的缓存文件后重试不会影响机器人。" "If the model download fails, verify that GitHub Releases are reachable, not only the github.com homepage. Removing a damaged cache file and retrying will not affect the robot."
  curl -fL --retry 2 --retry-delay 2 --continue-at - -o "$MODEL_ARCHIVE"     "https://github.com/k2-fsa/sherpa-onnx/releases/download/tts-models/${MODEL_NAME}.tar.bz2"
  tar -tjf "$MODEL_ARCHIVE" >/dev/null
  ok "Kokoro 模型下载并校验完成。" "Kokoro model download and validation completed."
fi

info "准备适用于 G1 aarch64 + CPython 3.8 的离线 Python wheels..." "Preparing offline Python wheels for G1 aarch64 + CPython 3.8..."
help_customer "如果 pip 下载失败，先确认这台联网电脑能访问 PyPI；如果只有某个 aarch64/cp38 wheel 不存在，请保留完整错误输出并联系项目供应方，不要用 x86_64 wheel 代替。" "If pip download fails, verify PyPI access from this computer. If a required aarch64/cp38 wheel is unavailable, keep the full error output and contact the project provider; do not substitute x86_64 wheels."
python3 -m pip download --dest "$WHEELHOUSE"   --only-binary=:all:   --platform manylinux2014_aarch64   --python-version 38   --implementation cp   --abi cp38   'virtualenv==20.26.6' 'pip<25' 'setuptools<76' wheel   'numpy<2' "sherpa-onnx==${SHERPA_VERSION}"
ok "离线 wheelhouse 已准备：$WHEELHOUSE" "Offline wheelhouse is ready: $WHEELHOUSE"

step "6/8 检查机器人现场源码" "6/8 Check robot-side source trees"
prepare_remote_target "/home/unitree/UniRoboGui" "UniRoboGui" false
prepare_remote_target "/home/unitree/unitree_sdk2" "Unitree SDK2" true "unitreerobotics/unitree_sdk2"
prepare_remote_target "/home/unitree/librealsense" "librealsense2" true "realsenseai/librealsense"

step "7/8 通过 SSH/rsync 传输项目和全部 GitHub/PyPI 资源" "7/8 Transfer the project and GitHub/PyPI resources via SSH/rsync"
help_customer "传输过程中如果断线，请检查电脑与机器人网线/交换机连接；重新运行脚本会继续同步，不需要重新下载全部资源。" "If the transfer disconnects, check the Ethernet/switch connection between the computer and robot. Rerunning continues synchronization without redownloading all resources."
ssh "$ROBOT" "mkdir -p /home/unitree/UniRoboGui /home/unitree/unitree_sdk2 /home/unitree/librealsense /home/unitree/unitree_interface/tts_models /home/unitree/unirobogui-wheelhouse"

info "传输 UniRoboGui（不传 .git；保留机器人已有客户配置和 build 目录）..." "Transferring UniRoboGui (excluding .git while preserving robot customer configuration and build directory)..."
project_delete_args=()
$PROJECT_SYNC_DELETE && project_delete_args+=(--delete)
rsync -az "${project_delete_args[@]}" --info=progress2 \
  --exclude '/.git/' \
  --exclude '/.unirobogui-managed' \
  --exclude '/build/' \
  --exclude '/config/customer_voice.json' \
  --exclude '/config/joint_teach_actions.json' \
  "$PROJECT_DIR/" "$ROBOT:$REMOTE_PROJECT_DIR/"
project_revision="$(git -C "$PROJECT_DIR" rev-parse HEAD)"
ssh "$ROBOT" "printf '%s\\n' '$project_revision' > '$REMOTE_PROJECT_MARKER'"

info "传输最新 Unitree SDK2..." "Transferring the latest Unitree SDK2..."
rsync -az --delete --info=progress2 --exclude '/build-g1/'   "$SDK2_CACHE/" "$ROBOT:/home/unitree/unitree_sdk2/"

info "传输最新 librealsense2..." "Transferring the latest librealsense2..."
rsync -az --delete --info=progress2 --exclude '/build-g1/'   "$REALSENSE_CACHE/" "$ROBOT:/home/unitree/librealsense/"

info "传输 Kokoro 模型..." "Transferring the Kokoro model..."
rsync -az --info=progress2 "$MODEL_ARCHIVE"   "$ROBOT:/home/unitree/unitree_interface/tts_models/${MODEL_NAME}.tar.bz2"

info "传输 G1 AArch64/Python 3.8 wheelhouse..." "Transferring the G1 AArch64/Python 3.8 wheelhouse..."
rsync -az --info=progress2 "$WHEELHOUSE/"   "$ROBOT:/home/unitree/unirobogui-wheelhouse/"
ok "项目与离线依赖已全部传入机器人。" "Project and offline dependencies have been transferred to the robot."

step "8/8 在机器人端自动编译、安装并验收" "8/8 Build, install, and verify automatically on the robot"
ui_line "[信息] 后续日志来自机器人端。sudo 需要密码时，请按终端提示输入机器人 unitree 用户密码。" "[INFO] The following logs come from the robot. If sudo requests a password, enter the robot unitree user's password at the terminal prompt."
help_customer "如果远程部署失败，请直接查看下方机器人脚本输出；它会标出具体步骤、失败命令和需要处理的问题。" "If remote deployment fails, inspect the robot-side script output below; it identifies the step, failed command, and required corrective action."
ssh -t "$ROBOT"   "cd /home/unitree/UniRoboGui && UNIROBOGUI_LANG='$UNIROBOGUI_LANG' bash scripts/deploy_g1_online.sh --from-transferred-sources --wheelhouse /home/unitree/unirobogui-wheelhouse"

echo
line
ui_line "[部署完成] 联网电脑已完成下载、传输，并由机器人完成编译安装和健康检查。" "[DEPLOYMENT COMPLETE] The online computer completed downloads/transfers, and the robot completed build, installation, and health checks."
ui_line "[默认访问] http://192.168.123.164:8080" "[DEFAULT URL] http://192.168.123.164:8080"
ui_line "[提示] 如果机器人 wlan0 有其他 IP，也可以使用该 IP 的 8080 端口访问。" "[TIP] If the robot has another wlan0 IP, you can also access port 8080 at that address."
line
