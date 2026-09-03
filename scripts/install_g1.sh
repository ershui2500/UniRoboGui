#!/usr/bin/env bash
set -euo pipefail

expected_repo="/home/unitree/UniRoboGui"
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "${repo_dir}/scripts/deploy_i18n.sh"
project_dir="${repo_dir}"
legacy_project="/home/unitree/unitree_interface/g1_web_control"
without_realsense=false
with_wifi_watchdog=false
without_kokoro=false
check_only=false
sdk2_source="/home/unitree/unitree_sdk2"
realsense_source=""
offline_mode="${UNIROBOGUI_OFFLINE:-0}"
install_scope_request="${UNIROBOGUI_INSTALL_SCOPE:-auto}"
install_scope=""
sdk_prefix=""
SYSTEM_PACKAGES=(
  build-essential cmake pkg-config git curl bzip2
  python3 python3-pip
  libboost-system-dev libboost-thread-dev libjsoncpp-dev
  libcurl4-openssl-dev libopencv-dev libzmq3-dev
  libusb-1.0-0-dev libssl-dev libudev-dev
)

usage() {
  if [[ "$UNIROBOGUI_LANG" == "en" ]]; then
    cat <<'EOF'
Usage: bash scripts/install_g1.en.sh [options]

Options:
  --sdk2-source DIR     Install a transferred SDK2 source tree when SDK2 is missing
  --realsense-source DIR
                        Build and install librealsense2 from a transferred source tree
  --without-realsense   Do not install native D435i support
  --without-kokoro      Skip local Kokoro TTS (Unitree TTS fallback remains available)
  --with-wifi-watchdog  Install the optional wlan0 recovery timer (system install only)
  --user-install        Force user-local SDK2 + systemctl --user services (requires Linger=yes)
  --system-install      Force /opt SDK2 + system systemd services (requires sudo)
  --check-only          Only check the PC2, network and transferred source trees
  -h, --help            Show this help
EOF
  else
    cat <<'EOF'
用法：bash scripts/install_g1.sh [选项]

选项：
  --sdk2-source DIR     SDK2 缺失时，从已传入的 SDK2 源码目录安装
  --realsense-source DIR
                        从已传入源码编译并安装 librealsense2
  --without-realsense   不安装原生 D435i 支持
  --without-kokoro      跳过本地 Kokoro TTS（仍可使用 Unitree TTS）
  --with-wifi-watchdog  安装可选的 wlan0 恢复定时器（仅系统级安装）
  --user-install        强制用户级 SDK2 + systemctl --user 服务（要求 Linger=yes）
  --system-install      强制 /opt SDK2 + 系统 systemd 服务（要求 sudo）
  --check-only          仅检查 PC2、网络和已传入源码，不执行安装
  -h, --help            显示帮助
EOF
  fi
}

while (($#)); do
  case "$1" in
    --sdk2-source)
      (($# >= 2)) || { ui_err "错误：--sdk2-source 需要目录参数" "ERROR: --sdk2-source requires a directory"; exit 2; }
      sdk2_source="$2"; shift ;;
    --realsense-source)
      (($# >= 2)) || { ui_err "错误：--realsense-source 需要目录参数" "ERROR: --realsense-source requires a directory"; exit 2; }
      realsense_source="$2"; shift ;;
    --without-realsense) without_realsense=true ;;
    --without-kokoro) without_kokoro=true ;;
    --with-wifi-watchdog) with_wifi_watchdog=true ;;
    --user-install) install_scope_request=user ;;
    --system-install) install_scope_request=system ;;
    --check-only) check_only=true ;;
    -h|--help) usage; exit 0 ;;
    *) ui_err "错误：未知参数：$1" "ERROR: Unknown option: $1"; usage >&2; exit 2 ;;
  esac
  shift
done

fail() {
  ui_err "[错误] $1" "[ERROR] ${2:-$1}"
  exit 1
}

warn() {
  ui_err "[警告] $1" "[WARN] ${2:-$1}"
}

wifi_help() {
  if [[ "$UNIROBOGUI_LANG" == "en" ]]; then
    cat >&2 <<'EOF'
[TIP] G1 PC2 is often not connected to Wi-Fi by default. Keep eth0 for robot DDS and configure wlan0 only:
  nmcli device status
  nmcli radio wifi
  rfkill list wifi
  # If nmcli reports wlan0:wifi:unavailable and rfkill says Soft blocked: yes:
  nmcli radio wifi on
  nmcli device wifi list ifname wlan0
  nmcli --ask device wifi connect "YOUR_WIFI_NAME" ifname wlan0
  ip -4 route show
Rerun the installer after connecting. Do not disconnect or modify eth0, and do not place the Wi-Fi password directly on the command line.
EOF
  else
    cat >&2 <<'EOF'
[提示] G1 PC2 默认通常没有连接 Wi-Fi。请保留 eth0（机器人 DDS 专用），只配置 wlan0：
  nmcli device status
  nmcli radio wifi
  rfkill list wifi
  # 如果 nmcli 显示 wlan0:wifi:unavailable 且 rfkill 为 Soft blocked: yes：
  nmcli radio wifi on
  nmcli device wifi list ifname wlan0
  nmcli --ask device wifi connect "你的Wi-Fi名称" ifname wlan0
  ip -4 route show
连接后重新运行安装脚本。不要断开或修改 eth0，也不要在命令中明文填写 Wi-Fi 密码。
EOF
  fi
}

github_help() {
  if [[ "$UNIROBOGUI_LANG" == "en" ]]; then
    cat >&2 <<'EOF'
[TIP] A working Wi-Fi link does not guarantee GitHub access. Do not retry indefinitely.
Download on a computer that can access GitHub, then transfer over the 192.168.123.164 Ethernet link:
  - project -> /home/unitree/UniRoboGui
  - latest unitree_sdk2 source -> /home/unitree/unitree_sdk2
  - librealsense source (only if the robot lacks usable librealsense2) -> /home/unitree/librealsense
  - kokoro-int8-multi-lang-v1_1.tar.bz2 -> /home/unitree/unitree_interface/tts_models/
See docs/deployment-dependencies.md for copy-ready offline transfer commands.
EOF
  else
    cat >&2 <<'EOF'
[提示] Wi-Fi 连上不等于能访问 GitHub，国内网络经常会超时。不要反复重试。
请在一台能访问 GitHub 的电脑下载，再通过 192.168.123.164 有线传到机器人：
  - 项目目录 -> /home/unitree/UniRoboGui
  - unitree_sdk2 最新源码 -> /home/unitree/unitree_sdk2
  - librealsense 源码（仅机器人缺少可用 librealsense2 时需要）-> /home/unitree/librealsense
  - kokoro-int8-multi-lang-v1_1.tar.bz2 -> /home/unitree/unitree_interface/tts_models/
docs/deployment-dependencies.md 给出了可直接复制的离线传包命令。
EOF
  fi
}

github_available=false
have_realsense() {
  pkg-config --exists realsense2 2>/dev/null ||
    [[ -f /usr/local/lib/cmake/realsense2/realsense2Config.cmake ]] ||
    [[ -f /usr/local/lib/aarch64-linux-gnu/cmake/realsense2/realsense2Config.cmake ]] ||
    [[ -f /opt/ros/noetic/lib/aarch64-linux-gnu/cmake/realsense2/realsense2Config.cmake ]]
}

probe_github() {
  if ! command -v curl >/dev/null 2>&1; then
    ui_err "[WARN] curl 尚未安装，暂时无法检测 GitHub；APT 安装基础工具后会自动重试。" "[WARN] curl is not installed yet, so GitHub cannot be checked; the check will retry after APT installs the base tools."
  elif curl --noproxy '*' -IL --connect-timeout 5 --max-time 10 -fsS \
      https://github.com/ >/dev/null 2>&1; then
    github_available=true
    ui_line "[OK] GitHub 当前可访问。" "[OK] GitHub is reachable."
  else
    ui_err "[WARN] GitHub 当前不可访问或超时。" "[WARN] GitHub is currently unreachable or timed out."
    github_help
  fi
}

configure_install_scope() {
  case "${install_scope_request}" in
    auto|system|user) ;;
    *) fail "无效安装范围：${install_scope_request}。请使用 auto、system 或 user。" "Invalid install scope: ${install_scope_request}. Use auto, system or user." ;;
  esac

  if [[ "${install_scope_request}" != "user" ]] && sudo -n true >/dev/null 2>&1; then
    install_scope=system
    sdk_prefix=/opt/unitree_robotics
    ui_line "[OK] sudo 可免交互使用；采用系统级安装。" "[OK] sudo access is available without an interactive password prompt; using system install."
    return
  fi

  if [[ "${install_scope_request}" == "system" ||
        ("${install_scope_request}" == "auto" && -t 0) ]]; then
    [[ -t 0 ]] || fail "系统级安装需要 sudo，但当前终端不可交互。请在交互终端重试；仅当 Linger=yes 且系统包已齐全时才使用 --user-install。" "System install requires sudo, but this terminal is not interactive. Rerun interactively or use --user-install when Linger=yes and system packages are already present."
    ui_line "[信息] 长时间编译前先验证 sudo 权限..." "[INFO] Verifying sudo access before long builds..."
    sudo -v
    install_scope=system
    sdk_prefix=/opt/unitree_robotics
    ui_line "[OK] sudo 权限验证通过；采用系统级安装。" "[OK] sudo access verified; using system install."
    return
  fi

  systemctl --user show-environment >/dev/null 2>&1 ||
    fail "用户级 systemd 不可用；在没有交互 sudo 的情况下无法进行用户级安装。" "User systemd is unavailable; cannot use user-local install without interactive sudo."
  local linger
  linger="$(loginctl show-user "$(id -un)" -p Linger --value 2>/dev/null || true)"
  [[ "${linger}" == "yes" ]] ||
    fail "用户级持久安装要求 Linger=yes，确保退出登录或重启后服务仍可运行。请改用带 sudo 的交互终端运行安装器。" "User-local install requires Linger=yes so services survive logout/reboot. Run the installer interactively with sudo instead."
  ${with_wifi_watchdog} &&
    fail "--with-wifi-watchdog 会管理系统级定时器，因此只能用于系统级安装。" "--with-wifi-watchdog requires a system install because it manages a system timer."
  install_scope=user
  sdk_prefix=/home/unitree/.local/unitree_robotics
  ui_line "[OK] 无交互 sudo；采用持久用户级安装（systemctl --user，Linger=yes）。" "[OK] Interactive sudo is unavailable; using persistent user-local install (systemctl --user, Linger=yes)."
}

missing_system_packages() {
  local pkg
  for pkg in "${SYSTEM_PACKAGES[@]}"; do
    if ! dpkg-query -W -f='${Status}\n' "$pkg" 2>/dev/null | grep -q 'ok installed'; then
      printf '%s\n' "$pkg"
    fi
  done
}

install_system_packages() {
  local missing=()
  mapfile -t missing < <(missing_system_packages)
  if ((${#missing[@]} == 0)); then
    ui_line "[OK] 所需 Ubuntu 系统包已全部安装；跳过 APT。" "[OK] Required Ubuntu system packages are already installed; skipping APT."
    return
  fi

  warn "缺少 Ubuntu 系统包：${missing[*]}" "Missing Ubuntu system packages: ${missing[*]}"
  if [[ "${install_scope}" == "user" ]]; then
    fail "用户级安装不能补装缺失的 Ubuntu 系统包。请在交互终端进行系统级安装，或先安装上面列出的 Ubuntu 20.04 arm64 软件包。" "User-local install cannot add missing Ubuntu packages. Rerun in an interactive terminal for system install, or install the listed arm64 Ubuntu 20.04 packages first."
  fi
  sudo -v
  if ! sudo apt-get update \
    -o Dir::Etc::sourcelist="sources.list" \
    -o Dir::Etc::sourceparts="-" \
    -o APT::Get::List-Cleanup="0"; then
    wifi_help
    fail "缺少必要软件包且 APT 更新失败。请保持 eth0 不变，让 wlan0 连接 Ubuntu 20.04 软件源，或先安装列出的软件包后重试。" "APT update failed while required packages are missing. Keep eth0 unchanged; connect wlan0 to an Ubuntu 20.04 mirror or install the listed packages first, then rerun."
  fi
  sudo apt-get install -y --no-install-recommends "${missing[@]}"
}

check_network() {
  if ! ip link show dev wlan0 >/dev/null 2>&1; then
    ui_err "[WARN] 没有发现 wlan0；如需在线安装，请先插好/配置 PC2 无线网卡。" "[WARN] wlan0 was not found. For online installation, connect/configure the PC2 wireless adapter first."
    wifi_help
  elif nmcli -t -f DEVICE,TYPE,STATE device status 2>/dev/null |
       grep -q '^wlan0:wifi:unavailable$'; then
    ui_err "[WARN] wlan0 当前不可用。若 rfkill 显示 Soft blocked: yes，请先执行 nmcli radio wifi on；不要操作 eth0。" "[WARN] wlan0 is unavailable. If rfkill reports Soft blocked: yes, run nmcli radio wifi on first. Do not modify eth0."
    wifi_help
  elif ! nmcli -t -f DEVICE,TYPE,STATE device status 2>/dev/null |
         grep -q '^wlan0:wifi:connected$'; then
    ui_err "[WARN] wlan0 尚未连接 Wi-Fi；在线安装 APT、SDK2 或 Kokoro 会失败。" "[WARN] wlan0 is not connected to Wi-Fi; online APT, SDK2, or Kokoro installation will fail."
    wifi_help
  elif ! ip -4 route show default | grep -q 'dev wlan0'; then
    ui_err "[WARN] wlan0 已连接，但没有经 wlan0 的 IPv4 默认路由，暂时不能确认外网可用。" "[WARN] wlan0 is connected but has no IPv4 default route, so Internet connectivity cannot be confirmed."
    wifi_help
  else
    ui_line "[OK] wlan0 已连接，eth0 保留给机器人 DDS。" "[OK] wlan0 is connected and eth0 remains dedicated to robot DDS."
  fi

  probe_github
}

install_sdk2() {
  local config="${sdk_prefix}/lib/cmake/unitree_sdk2/unitree_sdk2Config.cmake"
  [[ ! -f "${config}" ]] || {
    ui_line "[OK] SDK2 已安装在 ${sdk_prefix}。" "[OK] SDK2 is already installed in ${sdk_prefix}."
    return 0
  }

  if [[ ! -f "${sdk2_source}/CMakeLists.txt" ]]; then
    if ${github_available}; then
      [[ ! -e "${sdk2_source}" ]] ||
        fail "SDK2 源码路径存在但内容不完整：${sdk2_source}" "SDK2 source path exists but is incomplete: ${sdk2_source}"
      ui_line "[信息] 正在下载当前官方 SDK2 源码..." "[INFO] Downloading the current official SDK2 source..."
      git clone --depth 1 https://github.com/unitreerobotics/unitree_sdk2.git \
        "${sdk2_source}" || fail "SDK2 下载失败。请改用另一台联网电脑传入源码。" "SDK2 download failed. Transfer it from another computer instead."
    else
      github_help
      fail "缺少 SDK2。请把最新官方源码传到 ${sdk2_source} 后重新运行。" "SDK2 is missing. Transfer the latest official source to ${sdk2_source}, then rerun."
    fi
  fi

  ui_line "[信息] 正在从 ${sdk2_source} 安装 SDK2 到 ${sdk_prefix}..." "[INFO] Installing SDK2 from ${sdk2_source} into ${sdk_prefix}..."
  cmake -S "${sdk2_source}" -B "${sdk2_source}/build-g1" \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="${sdk_prefix}" \
    -DBUILD_EXAMPLES=OFF
  cmake --build "${sdk2_source}/build-g1" --parallel 2
  if [[ "${install_scope}" == "system" ]]; then
    sudo cmake --install "${sdk2_source}/build-g1"
  else
    mkdir -p "${sdk_prefix}"
    cmake --install "${sdk2_source}/build-g1"
  fi
  [[ -f "${config}" ]] || fail "SDK2 编译完成，但未安装 ${config}。" "SDK2 build finished, but ${config} was not installed."
}

install_realsense() {
  if have_realsense; then
    return 0
  fi
  if ${without_realsense}; then
    warn "已禁用原生 D435i 支持；普通 V4L2 摄像头仍可使用。" "Native D435i support is disabled; ordinary V4L2 cameras remain available."
    return 0
  fi
  if [[ "${install_scope}" == "user" ]]; then
    fail "缺少 librealsense2，用户级安装无法安全安装其系统 USB/udev 集成。请在交互终端进行系统级安装，或先安装 librealsense2。" "librealsense2 is missing, and user-local install cannot safely install its system USB/udev integration. Rerun interactively for system install, or install librealsense2 first."
  fi
  if [[ -z "${realsense_source}" ]]; then
    without_realsense=true
    warn "未安装 librealsense2；将继续部署，但不启用原生 D435i 支持。" "librealsense2 is not installed. Continuing without native D435i support."
    warn "G1 D435i 需要 librealsense2；请传入其源码，并使用 --realsense-source DIR 重新运行。" "G1 D435i requires librealsense2; transfer its source and rerun with --realsense-source DIR."
    return 0
  fi
  [[ -f "${realsense_source}/CMakeLists.txt" ]] ||
    fail "无效的 librealsense 源码目录：${realsense_source}" "Invalid librealsense source directory: ${realsense_source}"

  ui_line "[信息] 正在从 ${realsense_source} 编译 librealsense2..." "[INFO] Building librealsense2 from ${realsense_source}..."
  cmake -S "${realsense_source}" -B "${realsense_source}/build-g1" \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DBUILD_EXAMPLES=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF \
    -DBUILD_PYTHON_BINDINGS=OFF -DFORCE_RSUSB_BACKEND=ON
  cmake --build "${realsense_source}/build-g1" --parallel 2
  sudo cmake --install "${realsense_source}/build-g1"
  if [[ -f "${realsense_source}/config/99-realsense-libusb.rules" ]]; then
    sudo install -m 0644 "${realsense_source}/config/99-realsense-libusb.rules" \
      /etc/udev/rules.d/99-realsense-libusb.rules
    sudo udevadm control --reload-rules
  fi
  sudo ldconfig
  have_realsense || fail "librealsense2 安装结束，但 CMake 仍找不到 realsense2。" "librealsense2 installation finished, but CMake cannot find realsense2."
}

wait_for_url() {
  local url="$1"
  local attempts="$2"
  for ((attempt = 1; attempt <= attempts; ++attempt)); do
    curl --noproxy '*' -fsS "${url}" >/dev/null 2>&1 && return 0
    sleep 1
  done
  fail "服务健康检查超时：${url}" "Service health check timed out: ${url}"
}

assert_motion_stopped() {
  local control_status
  control_status="$(curl --noproxy '*' -fsS http://127.0.0.1:8080/api/control/status)" ||
    fail "正在运行的 Web 服务没有返回控制状态，因此不会停止/替换该服务。" "The running Web service did not return control status; it was not stopped."
  CONTROL_STATUS="${control_status}" python3 - <<'PY'
import json
import os

motion = json.loads(os.environ["CONTROL_STATUS"])["control"]["motion"]
safe = (
    motion.get("active") is False
    and motion.get("state") == "stopped"
    and all(float(motion.get(key, 1)) == 0 for key in ("vx_m_s", "vy_m_s", "vyaw_rad_s"))
)
if not safe:
    if os.environ.get("UNIROBOGUI_LANG") == "en":
        raise SystemExit("[ERROR] Robot motion is not confirmed stopped; refusing to replace the service.")
    raise SystemExit("[错误] 无法确认机器人已停止且速度为零；拒绝替换正在运行的服务。")
PY
}

stop_legacy_user_services() {
  if systemctl --user is-active --quiet g1-web-control.service 2>/dev/null; then
    ui_line "[信息] 发现正在运行的用户级 g1-web-control.service；替换前先确认机器人零运动..." "[INFO] Found active user g1-web-control.service; verifying zero motion before replacement..."
    assert_motion_stopped
    systemctl --user stop g1-web-control.service ||
      fail "停止现有用户级 g1-web-control.service 失败。" "Failed to stop existing user g1-web-control.service."
  fi
  if systemctl --user is-enabled --quiet g1-web-control.service 2>/dev/null; then
    systemctl --user disable g1-web-control.service ||
      fail "禁用现有用户级 g1-web-control.service 失败。" "Failed to disable existing user g1-web-control.service."
  fi

  if systemctl --user is-active --quiet g1-local-tts.service 2>/dev/null; then
    systemctl --user stop g1-local-tts.service ||
      fail "停止现有用户级 g1-local-tts.service 失败。" "Failed to stop existing user g1-local-tts.service."
  fi
  if systemctl --user is-enabled --quiet g1-local-tts.service 2>/dev/null; then
    systemctl --user disable g1-local-tts.service ||
      fail "禁用现有用户级 g1-local-tts.service 失败。" "Failed to disable existing user g1-local-tts.service."
  fi

  sleep 1
  if ss -ltn 'sport = :8080' | grep -q LISTEN; then
    fail "停止已知 Web 服务后 8080 端口仍被占用。请检查：ss -ltnp 'sport = :8080'" "Port 8080 is still occupied after stopping known Web services. Check: ss -ltnp 'sport = :8080'"
  fi
  if ! ${without_kokoro} && ss -ltn 'sport = :8765' | grep -q LISTEN; then
    fail "停止已知 TTS 服务后 8765 端口仍被占用。请检查：ss -ltnp 'sport = :8765'" "Port 8765 is still occupied after stopping known TTS services. Check: ss -ltnp 'sport = :8765'"
  fi
}

[[ "${EUID}" -ne 0 ]] || fail "请以 unitree 用户运行，不要直接使用 root；脚本只在需要时调用 sudo。" "Run this script as unitree, not root; it uses sudo only when needed."
[[ "$(id -un)" == "unitree" ]] || fail "此安装器仅支持 unitree 账号。" "This installer supports the unitree account only."
[[ "${repo_dir}" == "${expected_repo}" ]] || fail "请把仓库 clone 到 ${expected_repo}；当前路径为 ${repo_dir}。" "Clone the repository to ${expected_repo}; current path is ${repo_dir}."
[[ "$(uname -m)" == "aarch64" ]] || fail "预期 G1 PC2 为 aarch64，当前为 $(uname -m)。" "Expected G1 PC2 aarch64, found $(uname -m)."

source /etc/os-release
[[ "${ID:-}" == "ubuntu" && "${VERSION_ID:-}" == "20.04" ]] ||
  fail "预期 G1 PC2 使用 Ubuntu 20.04，当前为 ${PRETTY_NAME:-unknown}。" "Expected Ubuntu 20.04 on G1 PC2, found ${PRETTY_NAME:-unknown}."
[[ -z "${ROS_DISTRO:-}" && -z "${RMW_IMPLEMENTATION:-}" &&
   -z "${CYCLONEDDS_URI:-}" ]] ||
  fail "请重新建立 SSH 会话，并在 ROS 环境提示时直接按回车；不要加载 ROS/RMW/CycloneDDS。" "Open a fresh SSH session and press Enter at the ROS prompt; do not load ROS/RMW/CycloneDDS."
ip link show dev eth0 >/dev/null 2>&1 || fail "缺少 DDS 网卡 eth0。" "DDS interface eth0 is missing."

if ${check_only}; then
  mapfile -t missing_packages < <(missing_system_packages)
  if ((${#missing_packages[@]} == 0)); then
    ui_line "[OK] 所需 Ubuntu 系统包已全部安装。" "[OK] Required Ubuntu system packages are already installed."
  else
    warn "缺少 Ubuntu 系统包：${missing_packages[*]}" "Missing Ubuntu system packages: ${missing_packages[*]}"
    warn "正式安装需要访问 Ubuntu 20.04 软件源，或先安装这些软件包。" "A real installation will need access to the Ubuntu 20.04 package mirror, or these packages must be installed first."
  fi
  if [[ -f /opt/unitree_robotics/lib/cmake/unitree_sdk2/unitree_sdk2Config.cmake ]]; then
    ui_line "[OK] SDK2 已安装在 /opt/unitree_robotics。" "[OK] SDK2 is installed in /opt/unitree_robotics."
  elif [[ -f /home/unitree/.local/unitree_robotics/lib/cmake/unitree_sdk2/unitree_sdk2Config.cmake ]]; then
    ui_line "[OK] SDK2 已安装在持久用户目录：/home/unitree/.local/unitree_robotics。" "[OK] SDK2 is installed in the persistent user prefix: /home/unitree/.local/unitree_robotics."
  elif [[ -f "${sdk2_source}/CMakeLists.txt" ]]; then
    ui_line "[OK] SDK2 尚未安装，但已传入源码可用：${sdk2_source}" "[OK] SDK2 is not installed yet, but transferred source is ready: ${sdk2_source}"
  else
    warn "SDK2 尚未安装且没有已传入源码；请提供 ${sdk2_source}，或让 GitHub 可访问。" "SDK2 is not installed and transferred source is missing; provide ${sdk2_source} or make GitHub reachable."
  fi
  if have_realsense; then
    ui_line "[OK] librealsense2 已安装。" "[OK] librealsense2 is installed."
  else
    warn "缺少 librealsense2；若不提供源码，将禁用原生 D435i 支持。" "librealsense2 is missing; native D435i support will be disabled unless source is provided."
  fi
  if nmcli -t -f DEVICE,TYPE,STATE device status 2>/dev/null | grep -q '^wlan0:wifi:connected$'; then
    ui_line "[OK] wlan0 已连接。" "[OK] wlan0 is connected."
  else
    ui_line "[信息] wlan0 未连接；若所需 Ubuntu 包和离线资源已齐全，离线部署可以接受此状态。" "[INFO] wlan0 is not connected. This is acceptable for offline deployment when required Ubuntu packages and transferred resources are already present."
  fi
  if systemctl --user is-active --quiet g1-web-control.service 2>/dev/null ||
     systemctl --user is-enabled --quiet g1-web-control.service 2>/dev/null; then
    ui_line "[OK] 已存在持久用户级 g1-web-control.service；无交互 sudo 时自动模式可复用/更新它。" "[OK] Persistent user g1-web-control.service is present. Auto mode may reuse/update it when interactive sudo is unavailable."
  fi
  if systemctl --user is-active --quiet g1-local-tts.service 2>/dev/null ||
     systemctl --user is-enabled --quiet g1-local-tts.service 2>/dev/null; then
    ui_line "[OK] 已存在持久用户级 g1-local-tts.service。" "[OK] Persistent user g1-local-tts.service is present."
  fi
  if [[ -x /usr/local/sbin/g1-web-first-person-service &&
        -f /etc/sudoers.d/g1-web-camera ]]; then
    ui_line "[OK] 相机占用服务 helper 和 sudoers 已安装。" "[OK] Camera owner-service helper and sudoers are installed."
  else
    ui_line "[信息] 相机 helper/sudoers 尚未安装。系统级安装会补齐；用户级安装优先并发使用 librealsense2 RGB+depth，只有固件拒绝并发访问时才需要 helper。" "[INFO] Camera helper/sudoers are not installed. System install will add them; user install first uses concurrent librealsense2 RGB+depth and only needs the helper if that firmware rejects concurrent camera access."
  fi
  ui_line "[OK] PC2 预检查完成；没有修改软件包、文件或服务。" "[OK] PC2 preflight completed; no packages, files or services were changed."
  exit 0
fi

configure_install_scope

if [[ "$offline_mode" == "1" ]]; then
  ui_line "[OK] 已提供离线部署资源；跳过 Wi-Fi/GitHub 必需性检查。" "[OK] Offline deployment resources were supplied; skipping Wi-Fi/GitHub requirement checks."
else
  check_network
fi

install_system_packages

if [[ ! -f "${sdk_prefix}/lib/cmake/unitree_sdk2/unitree_sdk2Config.cmake" &&
      ! -f "${sdk2_source}/CMakeLists.txt" ]]; then
  ${github_available} || probe_github
fi
install_sdk2
install_realsense

export CMAKE_PREFIX_PATH="${sdk_prefix}"
export LD_LIBRARY_PATH="${sdk_prefix}/lib:/usr/local/lib:/opt/ros/noetic/lib/aarch64-linux-gnu"

node_major=0
if command -v node >/dev/null 2>&1; then
  node_major="$(node -p 'Number(process.versions.node.split(".")[0])' 2>/dev/null || echo 0)"
fi
if ((node_major >= 14)); then
  for file in web/i18n.js web/app.js web/workspace.js web/imu-gauges.js \
    web/robot-viewer.js web/perception.js web/joint-debug.js \
    web/nav-renderer.js web/nav-store.js; do
    node --check "${project_dir}/${file}"
  done
else
  ui_line "[警告] Node.js >= 14 不可用；跳过可选前端语法检查。" "[WARN] Node.js >= 14 is unavailable; skipping the optional frontend syntax check."
fi

cmake -S "${project_dir}" -B "${project_dir}/build" \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build "${project_dir}/build" --parallel 2
(cd "${project_dir}/build" && ctest --output-on-failure)

ldd_output="$(ldd "${project_dir}/build/g1_web_server")"
if grep -q 'not found' <<<"${ldd_output}"; then
  grep 'not found' <<<"${ldd_output}" >&2
  fail "Web 二进制存在未解析的动态库。" "The Web binary has unresolved shared libraries."
fi
grep -q 'libddsc\.so' <<<"${ldd_output}" || fail "Web 二进制没有链接 SDK2 CycloneDDS。" "The Web binary is not linked to SDK2 CycloneDDS."
if ! ${without_realsense}; then
  grep -q 'librealsense2\.so' <<<"${ldd_output}" || fail "Web 二进制未包含原生 D435i 支持。" "The Web binary was built without native D435i support."
fi

if ! ${without_kokoro}; then
  UNITREE_INTERFACE_ROOT=/home/unitree/unitree_interface \
    bash "${project_dir}/scripts/install_kokoro_tts.sh"
else
  warn "已跳过 Kokoro 安装；Web 语音仍可回退使用 Unitree 原生 TTS。" "Kokoro installation skipped; Web voice can still fall back to Unitree native TTS."
fi

mkdir -p "${project_dir}/config"
chmod 700 "${project_dir}/config"
for name in customer_voice.json joint_teach_actions.json; do
  old_file="${legacy_project}/config/${name}"
  new_file="${project_dir}/config/${name}"
  if [[ -f "${old_file}" && ! -e "${new_file}" ]]; then
    install -m 0600 "${old_file}" "${new_file}"
    ui_line "已从旧部署迁移 ${name}。" "Migrated ${name} from the legacy deployment."
  fi
  [[ ! -f "${new_file}" ]] || chmod 600 "${new_file}"
done

if [[ "${install_scope}" == "system" ]]; then
  if systemctl is-active --quiet g1-web-control.service; then
    assert_motion_stopped
    sudo systemctl stop g1-web-control.service
  fi

  stop_legacy_user_services

  sudo install -m 0755 "${project_dir}/scripts/g1-web-first-person-service" \
    /usr/local/sbin/g1-web-first-person-service
  sudo visudo -cf "${project_dir}/scripts/g1-web-camera.sudoers"
  sudo install -m 0440 "${project_dir}/scripts/g1-web-camera.sudoers" \
    /etc/sudoers.d/g1-web-camera
  helper_rc=0
  sudo -n /usr/local/sbin/g1-web-first-person-service is-active >/dev/null 2>&1 || helper_rc=$?
  if ((helper_rc != 0 && helper_rc != 3)); then
    fail "相机 helper 的 sudoers 验证失败（退出码 ${helper_rc}）。从 Web 使用 D435i 前请检查 /etc/sudoers.d/g1-web-camera。" "Camera helper sudoers verification failed (exit ${helper_rc}). Check /etc/sudoers.d/g1-web-camera before using D435i from Web."
  fi
  ui_line "[OK] 相机 helper 和免密 sudo 规则验证通过。" "[OK] Camera helper and passwordless sudo rule verified."

  sudo install -m 0644 "${project_dir}/deploy/g1-local-tts.service" \
    /etc/systemd/system/g1-local-tts.service
  sudo install -m 0644 "${project_dir}/deploy/g1-web-control.service" \
    /etc/systemd/system/g1-web-control.service

  if ${with_wifi_watchdog}; then
    sudo install -m 0755 "${project_dir}/deploy/g1-wifi-ensure" /usr/local/sbin/g1-wifi-ensure
    sudo install -m 0644 "${project_dir}/deploy/g1-wifi-connect.service" \
      /etc/systemd/system/g1-wifi-connect.service
    sudo install -m 0644 "${project_dir}/deploy/g1-wifi-connect.timer" \
      /etc/systemd/system/g1-wifi-connect.timer
  fi

  sudo systemctl daemon-reload
  if ! ${without_kokoro}; then
    sudo systemctl enable g1-local-tts.service
    sudo systemctl restart g1-local-tts.service
  fi
  sudo systemctl enable --now g1-web-control.service
  if ${with_wifi_watchdog}; then
    sudo systemctl enable --now g1-wifi-connect.timer
  fi

  ${without_kokoro} || systemctl is-active --quiet g1-local-tts.service
  systemctl is-active --quiet g1-web-control.service
else
  if systemctl is-active --quiet g1-web-control.service 2>/dev/null ||
     systemctl is-enabled --quiet g1-web-control.service 2>/dev/null ||
     systemctl is-active --quiet g1-local-tts.service 2>/dev/null ||
     systemctl is-enabled --quiet g1-local-tts.service 2>/dev/null; then
    fail "系统级 UniRoboGui 服务已存在。用户级安装不会与其争抢端口；请在交互终端重新执行系统级安装。" "System-level UniRoboGui services already exist. User-local install will not compete for ports; rerun interactively for system install."
  fi

  stop_legacy_user_services
  mkdir -p /home/unitree/.config/systemd/user
  install -m 0644 "${project_dir}/deploy/g1-web-control.user.service" \
    /home/unitree/.config/systemd/user/g1-web-control.service
  install -m 0644 "${project_dir}/deploy/g1-local-tts.user.service" \
    /home/unitree/.config/systemd/user/g1-local-tts.service
  systemctl --user daemon-reload
  if ! ${without_kokoro}; then
    systemctl --user enable g1-local-tts.service
    systemctl --user restart g1-local-tts.service
  fi
  systemctl --user enable --now g1-web-control.service
  ${without_kokoro} || systemctl --user is-active --quiet g1-local-tts.service
  systemctl --user is-active --quiet g1-web-control.service
  if [[ ! -x /usr/local/sbin/g1-web-first-person-service ||
        ! -f /etc/sudoers.d/g1-web-camera ]]; then
    ui_line "[信息] 用户级安装没有特权相机 helper。D435i 会先尝试 librealsense2 RGB+depth 并发访问；只有固件拒绝并发访问时才需要后续系统级安装提供 helper 兜底。" "[INFO] User-local install has no privileged camera helper. D435i first tries concurrent librealsense2 RGB+depth; only firmware that rejects concurrent access needs a later system install for the helper fallback."
  fi
fi

${without_kokoro} || wait_for_url http://127.0.0.1:8765/health 120
wait_for_url http://127.0.0.1:8080/api/health 60

robot_ip="$(ip -4 -o addr show dev wlan0 2>/dev/null | awk '{split($4, a, "/"); print a[1]; exit}')"
[[ -n "${robot_ip}" ]] || robot_ip="192.168.123.164"
echo
ui_line "UniRoboGui 部署完成。" "UniRoboGui deployment completed."
ui_line "安装范围：${install_scope}（SDK2：${sdk_prefix}）" "Install scope: ${install_scope} (SDK2: ${sdk_prefix})"
ui_line "访问：http://${robot_ip}:8080" "Open: http://${robot_ip}:8080"
ui_line "g1-web-control.service 默认启用真实导航；Web 安全确认与 10 m 目标距离限制仍然生效。" "Real navigation is enabled by default in g1-web-control.service; Web safety confirmation and the 10 m goal limit still apply."
