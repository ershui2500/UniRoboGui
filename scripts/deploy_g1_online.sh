#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "$PROJECT_DIR/scripts/deploy_i18n.sh"
SDK2_DIR="/home/unitree/unitree_sdk2"
REALSENSE_DIR="/home/unitree/librealsense"
OFFLINE_MODE=false
WHEELHOUSE=""
SYSTEM_PACKAGES=(
  build-essential cmake pkg-config git curl bzip2 rsync
  python3 python3-pip
  libboost-system-dev libboost-thread-dev libjsoncpp-dev
  libcurl4-openssl-dev libopencv-dev libzmq3-dev
  libusb-1.0-0-dev libssl-dev libudev-dev
)

usage() {
  if [[ "$UNIROBOGUI_LANG" == "en" ]]; then
    cat <<'EOF'
Usage:
  bash scripts/deploy_g1_online.en.sh

Normally no options are required. Use this entry point when the robot itself can access GitHub.
If the robot cannot access GitHub, run this on an Ubuntu/Linux computer that can:
  bash scripts/deploy_g1_from_pc.en.sh

Internal options (used automatically by the offline deployment flow):
  --from-transferred-sources
  --wheelhouse DIR
EOF
  else
    cat <<'EOF'
用法：
  bash scripts/deploy_g1_online.sh

正常情况下不要加参数。该脚本用于“机器人本身可以访问 GitHub”的部署场景。
如果机器人不能访问 GitHub，请在一台能访问 GitHub 的 Ubuntu/Linux 电脑上运行：
  bash scripts/deploy_g1_from_pc.sh

内部参数（由离线部署脚本自动调用）：
  --from-transferred-sources
  --wheelhouse DIR
EOF
  fi
}

while (($#)); do
  case "$1" in
    --from-transferred-sources) OFFLINE_MODE=true ;;
    --wheelhouse)
      (($# >= 2)) || { ui_err "[错误] --wheelhouse 缺少目录参数" "[ERROR] --wheelhouse requires a directory argument"; exit 2; }
      WHEELHOUSE="$2"
      shift
      ;;
    -h|--help) usage; exit 0 ;;
    *) ui_err "[错误] 未知参数：$1" "[ERROR] Unknown option: $1"; usage >&2; exit 2 ;;
  esac
  shift
done

STEP_ZH=""
STEP_EN=""
LAST_HELP_ZH=""
LAST_HELP_EN=""

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
  ui_err "[提示] 修复上面的问题后，直接重新运行本脚本即可；脚本设计为可重复执行。" "[TIP] Fix the issue above and rerun this script; the deployment flow is designed to be repeatable."
  line >&2
  exit "$rc"
}
trap on_error ERR

github_available() {
  if command -v curl >/dev/null 2>&1; then
    curl --noproxy '*' -IL --connect-timeout 5 --max-time 10 -fsS https://github.com/ >/dev/null 2>&1
    return
  fi
  if command -v git >/dev/null 2>&1; then
    git ls-remote https://github.com/unitreerobotics/unitree_sdk2.git HEAD >/dev/null 2>&1
    return
  fi
  if command -v python3 >/dev/null 2>&1; then
    python3 - <<'PY' >/dev/null 2>&1
import urllib.request
urllib.request.urlopen("https://github.com/", timeout=8).close()
PY
    return
  fi
  return 1
}

archive_source() {
  local name="$1" dir="$2"
  local backup_root="/home/unitree/.unirobogui-deploy-backups"
  local stamp backup
  stamp="$(date +%Y%m%d-%H%M%S)"
  backup="$backup_root/$(basename "$dir")-$stamp"
  mkdir -p "$backup_root"
  warn "$name 现有源码无法安全直接更新，将完整保留后重新下载官方最新版。" "The existing $name source cannot be updated safely in place; it will be preserved before downloading the latest official source."
  info "旧源码备份位置：$backup" "Previous source backup: $backup"
  mv "$dir" "$backup"
}

update_or_clone() {
  local name="$1" url="$2" dir="$3" expected="$4"
  if [[ -e "$dir" ]]; then
    info "检测到已有 $name 源码：$dir" "Existing $name source detected: $dir"
    if [[ ! -d "$dir/.git" ]]; then
      warn "$name 目录不是 Git 仓库，无法确认版本或执行安全 pull。" "$name is not a Git repository, so its version cannot be verified or safely pulled."
      archive_source "$name" "$dir"
    else
      local remote dirty
      remote="$(git -C "$dir" remote get-url origin 2>/dev/null || true)"
      dirty="$(git -C "$dir" status --short)"
      if [[ "$remote" != *"$expected"* ]]; then
        warn "$name 的 origin 不是当前预期官方仓库：$remote" "$name origin is not the expected official repository: $remote"
        archive_source "$name" "$dir"
      elif [[ -n "$dirty" ]]; then
        warn "$name 源码存在现场修改/删除，不能直接 git pull：" "$name contains local modifications/deletions and cannot be updated with git pull safely:"
        echo "$dirty" | head -40 >&2
        [[ "$(echo "$dirty" | wc -l)" -le 40 ]] ||
          warn "仅显示前 40 条，共 $(echo "$dirty" | wc -l) 条变更。" "Showing only the first 40 of $(echo "$dirty" | wc -l) changes."
        archive_source "$name" "$dir"
      else
        info "从官方仓库更新 $name 到当前最新版本..." "Updating $name from the official repository..."
        help_customer "如果这里 Git 拉取失败，请检查机器人 wlan0、DNS 和 GitHub 访问；不要断开 eth0。若 GitHub 无法访问，请改用 scripts/deploy_g1_from_pc.sh。" "If Git pull fails here, check the robot's wlan0, DNS, and GitHub access. Do not disconnect eth0. If GitHub is unavailable, use scripts/deploy_g1_from_pc.en.sh from an online computer."
        git -C "$dir" pull --ff-only
      fi
    fi
  fi
  if [[ ! -e "$dir" ]]; then
    info "从官方仓库下载 $name 最新源码..." "Downloading the latest $name source from the official repository..."
    help_customer "如果 clone 失败，请检查机器人能否访问 GitHub；无法访问时请改用联网电脑脚本 scripts/deploy_g1_from_pc.sh。" "If clone fails, verify that the robot can access GitHub. Otherwise use scripts/deploy_g1_from_pc.en.sh from an online computer."
    git clone --depth 1 "$url" "$dir"
  fi
  ok "$name 源码已准备：$(git -C "$dir" rev-parse --short HEAD)" "$name source is ready: $(git -C "$dir" rev-parse --short HEAD)"
}

update_project_checkout() {
  [[ "$PROJECT_DIR" == "/home/unitree/UniRoboGui" ]] ||
    die "项目必须位于 /home/unitree/UniRoboGui；当前目录为 $PROJECT_DIR。" "The project must be located at /home/unitree/UniRoboGui; current directory: $PROJECT_DIR."
  [[ -d "$PROJECT_DIR/.git" ]] ||
    die "在线部署要求 /home/unitree/UniRoboGui 是从 GitHub clone 的 Git 仓库。若机器人不能访问 GitHub，请改用 scripts/deploy_g1_from_pc.sh。" "Online deployment requires /home/unitree/UniRoboGui to be a Git repository cloned from GitHub. If the robot cannot access GitHub, use scripts/deploy_g1_from_pc.en.sh."
  local remote dirty
  remote="$(git -C "$PROJECT_DIR" remote get-url origin 2>/dev/null || true)"
  [[ "$remote" == *"ershui2500/UniRoboGui"* ]] ||
    die "UniRoboGui 的 origin 不是预期仓库：$remote" "UniRoboGui origin is not the expected repository: $remote"
  dirty="$(git -C "$PROJECT_DIR" status --short)"
  if [[ -n "$dirty" ]]; then
    echo "$dirty" | head -40 >&2
    help_customer "请先提交或备份 /home/unitree/UniRoboGui 的现场修改，再重新运行；部署脚本不会自动覆盖业务代码。" "Commit or back up local changes in /home/unitree/UniRoboGui before rerunning. The deployment script will not overwrite project code automatically."
    die "UniRoboGui 存在未提交修改，无法安全自动升级。" "UniRoboGui has uncommitted changes and cannot be upgraded safely."
  fi
  local before after
  before="$(git -C "$PROJECT_DIR" rev-parse HEAD)"
  info "更新 UniRoboGui 到 origin 当前版本..." "Updating UniRoboGui to the current origin revision..."
  git -C "$PROJECT_DIR" pull --ff-only
  after="$(git -C "$PROJECT_DIR" rev-parse HEAD)"
  ok "UniRoboGui 当前提交：${after:0:7}" "UniRoboGui current commit: ${after:0:7}"
  if [[ "$before" != "$after" ]]; then
    info "项目源码已经更新，重新进入最新版部署脚本继续，避免旧脚本逻辑与新源码混用。" "The project was updated. Restarting with the latest deployment script to avoid mixing old script logic with new source."
    exec bash "$PROJECT_DIR/scripts/deploy_g1_online.sh"
  fi
}

missing_system_packages() {
  local pkg
  for pkg in "${SYSTEM_PACKAGES[@]}"; do
    if ! dpkg-query -W -f='${Status}\n' "$pkg" 2>/dev/null | grep -q 'ok installed'; then
      printf '%s\n' "$pkg"
    fi
  done
}

check_environment() {
  [[ "${EUID}" -ne 0 ]] || die "请使用 unitree 用户运行，不要直接使用 root。" "Run this script as the unitree user, not root."
  [[ "$(id -un)" == "unitree" ]] || die "当前用户是 $(id -un)，本脚本仅支持 G1 PC2 的 unitree 用户。" "Current user is $(id -un); this script supports only the unitree account on G1 PC2."
  [[ "$(uname -m)" == "aarch64" ]] || die "当前架构为 $(uname -m)，预期 G1 PC2 aarch64。" "Current architecture is $(uname -m); expected G1 PC2 aarch64."
  source /etc/os-release
  [[ "${ID:-}" == "ubuntu" && "${VERSION_ID:-}" == "20.04" ]] ||
    die "当前系统为 ${PRETTY_NAME:-unknown}，预期 Ubuntu 20.04。" "Current OS is ${PRETTY_NAME:-unknown}; expected Ubuntu 20.04."
  [[ -z "${ROS_DISTRO:-}" && -z "${RMW_IMPLEMENTATION:-}" && -z "${CYCLONEDDS_URI:-}" ]] || {
    help_customer "重新 SSH 登录机器人；出现 ROS 环境选择时直接按回车选择 none，然后重新运行脚本。" "Open a fresh SSH session to the robot, press Enter to select none at the ROS environment prompt, then rerun the script."
    die "检测到 ROS/RMW/CycloneDDS 环境变量。本项目生产部署只能使用 Unitree SDK2 DDS。" "ROS/RMW/CycloneDDS environment variables were detected. Production deployment must use Unitree SDK2 DDS only."
  }
  ip link show dev eth0 >/dev/null 2>&1 || die "未发现 eth0。eth0 是 Unitree SDK2 DDS 必需网卡。" "eth0 was not found. It is required for Unitree SDK2 DDS."
  ok "系统、用户、CPU 架构、无 ROS 环境和 eth0 检查通过。" "System, user, CPU architecture, clean ROS environment, and eth0 checks passed."
}

step "1/8 检测 GitHub 访问能力" "1/8 Check GitHub access"
if $OFFLINE_MODE; then
  info "当前由联网电脑离线部署脚本调用，使用已经传入机器人的源码，不要求机器人访问 GitHub。" "Invoked by the offline PC deployment flow; using sources already transferred to the robot, so robot-side GitHub access is not required."
else
  if github_available; then
    ok "机器人可以访问 GitHub，将使用机器人联网部署流程。" "The robot can access GitHub; using the robot-online deployment flow."
  else
    warn "机器人当前无法访问 GitHub，在线部署不能继续。" "The robot cannot currently access GitHub; online deployment cannot continue."
    if command -v nmcli >/dev/null 2>&1 &&
       nmcli -t -f DEVICE,TYPE,STATE device status 2>/dev/null | grep -q '^wlan0:wifi:unavailable$'; then
      warn "检测到 wlan0 为 wifi:unavailable。可先检查：rfkill list wifi；若为 Soft blocked: yes，执行 nmcli radio wifi on 后重试。不要操作 eth0。" "wlan0 is wifi:unavailable. Check: rfkill list wifi. If it reports Soft blocked: yes, run nmcli radio wifi on and retry. Do not modify eth0."
    fi
    echo
    ui_line "原因：在线流程需要 GitHub 来更新 UniRoboGui/SDK2，并在缺少 librealsense2 时取得官方源码。" "Reason: the online flow needs GitHub to update UniRoboGui/SDK2 and to obtain official librealsense2 source when needed."
    ui_line "请不要在机器人上反复重试，也不要断开 eth0。" "Do not keep retrying on the robot, and do not disconnect eth0."
    echo
    ui_line "请在一台能访问 GitHub、并能通过网线/局域网连接机器人的 Ubuntu/Linux 电脑上执行：" "On an Ubuntu/Linux computer that can access GitHub and reach the robot over Ethernet/LAN, run:"
    echo "  cd UniRoboGui"
    ui_line "  bash scripts/deploy_g1_from_pc.sh" "  bash scripts/deploy_g1_from_pc.en.sh"
    echo
    ui_line "该脚本会自动下载项目依赖、Kokoro 模型和 G1 AArch64/Python 3.8 wheelhouse，" "The script downloads project dependencies, the Kokoro model, and the G1 AArch64/Python 3.8 wheelhouse,"
    ui_line "然后通过 SSH/rsync 传到机器人，并在机器人端自动编译安装。" "then transfers them via SSH/rsync and builds/installs everything on the robot."
    exit 20
  fi
fi

step "2/8 检测 G1 PC2 环境" "2/8 Check the G1 PC2 environment"
check_environment

step "3/8 准备 UniRoboGui 项目源码" "3/8 Prepare UniRoboGui source"
if $OFFLINE_MODE; then
  [[ "$PROJECT_DIR" == "/home/unitree/UniRoboGui" ]] ||
    die "项目必须位于 /home/unitree/UniRoboGui；当前目录为 $PROJECT_DIR。" "The project must be located at /home/unitree/UniRoboGui; current directory: $PROJECT_DIR."
  info "使用联网电脑已经传入的 UniRoboGui 源码，不要求机器人访问 GitHub。" "Using UniRoboGui source already transferred by the online computer; robot-side GitHub access is not required."
else
  update_project_checkout
fi

step "4/8 检查系统编译依赖" "4/8 Check system build dependencies"
mapfile -t missing_packages < <(missing_system_packages)
if ((${#missing_packages[@]} == 0)); then
  ok "所需 Ubuntu 系统包已全部安装。" "All required Ubuntu system packages are installed."
else
  warn "当前缺少 Ubuntu 系统包：${missing_packages[*]}" "Missing Ubuntu system packages: ${missing_packages[*]}"
  info "后续统一由 install_g1.sh 处理：交互 sudo 模式会安装缺失包；无 sudo 的用户级模式会明确停止并提示先补包。" "install_g1.sh will handle this next: system install with interactive sudo can add missing packages; user-local mode without sudo will stop and tell you which packages must be installed first."
fi

step "5/8 准备 Unitree SDK2 源码" "5/8 Prepare Unitree SDK2 source"
if $OFFLINE_MODE; then
  [[ -f "$SDK2_DIR/CMakeLists.txt" ]] || die "离线部署缺少 SDK2 源码：$SDK2_DIR" "Offline deployment is missing SDK2 source: $SDK2_DIR"
  info "使用联网电脑已传入的 SDK2 源码：$SDK2_DIR" "Using SDK2 source transferred by the online computer: $SDK2_DIR"
else
  update_or_clone "Unitree SDK2"     "https://github.com/unitreerobotics/unitree_sdk2.git"     "$SDK2_DIR" "unitreerobotics/unitree_sdk2"
fi

step "6/8 检测/准备 librealsense2" "6/8 Check/prepare librealsense2"
if {
  pkg-config --exists realsense2 2>/dev/null ||
  [[ -f /usr/local/lib/cmake/realsense2/realsense2Config.cmake ]] ||
  [[ -f /usr/local/lib/aarch64-linux-gnu/cmake/realsense2/realsense2Config.cmake ]] ||
  [[ -f /opt/ros/noetic/lib/aarch64-linux-gnu/cmake/realsense2/realsense2Config.cmake ]];
}; then
  ok "机器人已经有可用 librealsense2，后续直接复用。" "A usable librealsense2 installation is already present and will be reused."
else
  if $OFFLINE_MODE; then
    info "使用联网电脑已传入的 librealsense 源码：$REALSENSE_DIR" "Using librealsense source transferred by the online computer: $REALSENSE_DIR"
  else
    update_or_clone "librealsense2"     "https://github.com/realsenseai/librealsense.git"     "$REALSENSE_DIR" "realsenseai/librealsense"
  fi
  [[ -f "$REALSENSE_DIR/CMakeLists.txt" ]] || die "librealsense 源码不完整：$REALSENSE_DIR" "librealsense source is incomplete: $REALSENSE_DIR"
fi

step "7/8 编译、测试并安装 UniRoboGui" "7/8 Build, test, and install UniRoboGui"
[[ "$PROJECT_DIR" == "/home/unitree/UniRoboGui" ]] ||
  die "项目必须位于 /home/unitree/UniRoboGui；当前目录为 $PROJECT_DIR。" "The project must be located at /home/unitree/UniRoboGui; current directory: $PROJECT_DIR."
[[ -f "$PROJECT_DIR/scripts/install_g1.sh" ]] || die "缺少 scripts/install_g1.sh。" "Missing scripts/install_g1.sh."
if $OFFLINE_MODE; then
  [[ -n "$WHEELHOUSE" && -d "$WHEELHOUSE" ]] || die "离线部署缺少 Kokoro wheelhouse：$WHEELHOUSE" "Offline deployment is missing the Kokoro wheelhouse: $WHEELHOUSE"
  [[ -s /home/unitree/unitree_interface/tts_models/kokoro-int8-multi-lang-v1_1.tar.bz2 ]] ||
    die "离线部署缺少 Kokoro 模型压缩包。" "Offline deployment is missing the Kokoro model archive."
  export KOKORO_WHEELHOUSE="$WHEELHOUSE"
  info "Kokoro 将只使用联网电脑传入的 wheelhouse：$WHEELHOUSE" "Kokoro will use only the wheelhouse transferred by the online computer: $WHEELHOUSE"
fi
help_customer "安装器会自动选择部署范围：交互 sudo 可用时安装系统级服务；无交互 sudo但系统包齐全且 Linger=yes 时使用持久 user-systemd。已有 Web 服务只有确认机器人运动状态为 stopped 且速度为 0 才会替换。" "The installer selects the service scope automatically: interactive sudo uses system services; without interactive sudo, a persistent user-systemd install is used only when required packages are present and Linger=yes. An existing Web service is replaced only after motion is confirmed stopped with all speeds at zero."
if $OFFLINE_MODE; then
  UNIROBOGUI_OFFLINE=1 bash "$PROJECT_DIR/scripts/install_g1.sh" \
    --sdk2-source "$SDK2_DIR" --realsense-source "$REALSENSE_DIR"
else
  bash "$PROJECT_DIR/scripts/install_g1.sh" \
    --sdk2-source "$SDK2_DIR" --realsense-source "$REALSENSE_DIR"
fi

step "8/8 最终只读验收" "8/8 Final read-only verification"
if systemctl is-active --quiet g1-web-control.service 2>/dev/null; then
  service_scope=system
elif systemctl --user is-active --quiet g1-web-control.service 2>/dev/null; then
  service_scope=user
else
  die "system 和 user 两种 g1-web-control.service 都未处于 active 状态。" "Neither the system nor user g1-web-control.service is active."
fi
ok "Web 服务正在运行（${service_scope} systemd）。" "Web service is running (${service_scope} systemd)."
curl --noproxy '*' -fsS http://127.0.0.1:8080/api/health
if [[ "$service_scope" == system ]] && systemctl is-enabled --quiet g1-local-tts.service 2>/dev/null; then
  systemctl is-active --quiet g1-local-tts.service || die "系统级 g1-local-tts.service 已启用但未正常运行。" "System g1-local-tts.service is enabled but not running."
  curl --noproxy '*' -fsS http://127.0.0.1:8765/health
elif [[ "$service_scope" == user ]] && systemctl --user is-enabled --quiet g1-local-tts.service 2>/dev/null; then
  systemctl --user is-active --quiet g1-local-tts.service || die "用户级 g1-local-tts.service 已启用但未正常运行。" "User g1-local-tts.service is enabled but not running."
  curl --noproxy '*' -fsS http://127.0.0.1:8765/health
fi
if ldd "$PROJECT_DIR/build/g1_web_server" | grep -q 'not found'; then
  ldd "$PROJECT_DIR/build/g1_web_server" | grep 'not found' >&2
  die "最终动态库检查发现 not found。" "Final shared-library check found unresolved libraries."
fi

robot_ip="$(ip -4 -o addr show dev wlan0 2>/dev/null | awk '{split($4,a,"/"); print a[1]; exit}')"
[[ -n "$robot_ip" ]] || robot_ip="$(ip -4 -o addr show dev eth0 2>/dev/null | awk '{split($4,a,"/"); print a[1]; exit}')"
echo
line
ui_line "[部署完成] UniRoboGui 已完成依赖安装、编译、CTest、服务安装和健康检查。" "[DEPLOYMENT COMPLETE] UniRoboGui dependencies, build, CTest, service installation, and health checks completed."
ui_line "[访问地址] http://${robot_ip:-192.168.123.164}:8080" "[OPEN] http://${robot_ip:-192.168.123.164}:8080"
ui_line "[安全说明] 本脚本没有主动执行机器人运动、导航、模式切换、上肢或示教动作。" "[SAFETY] This script did not actively execute robot motion, navigation, mode switching, upper-body motion, or taught actions."
line
