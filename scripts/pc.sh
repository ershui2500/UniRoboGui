#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "$project_dir/scripts/deploy_i18n.sh"
# This revision includes every product header required by the shared binary.
sdk_revision=7740f8b67e386ab09c3b333187fd5f8582a75ddc
sdk_source=""
sdk_prefix_explicit=false
sdk_prefix="${HOME}/.local/share/unirobogui/sdk2/${sdk_revision}"
build_dir="$project_dir/build-pc"
command_name="${1:---help}"
(($# == 0)) || shift
port=8080
mock=false
a2_joint_gains=""
usage() {
  cat <<'EOF'
UniRoboGui Ubuntu 22.04 x86_64 电脑 SDK2 控制台 / PC SDK2 console
  bash scripts/pc.sh install [--sdk2-source DIR] [--sdk-prefix DIR] [--build-dir DIR]
  bash scripts/pc.sh check   [--sdk-prefix DIR] [--build-dir DIR]
  bash scripts/pc.sh start   [--build-dir DIR] [--port 8080] [--mock] [--a2-joint-gains FILE]
  bash scripts/pc.sh shortcut [same options as start]

推荐默认安装到机器人，功能覆盖最完整；PC 为可选，部分功能受限。
Robot installation is recommended/default with the fullest feature set; PC has feature limitations.

安装后使用 start，在本机浏览器中选择 G1/R1/GO2/A2 和有线网卡；启动时不连接机器人。
After install, start and select G1/R1/GO2/A2 and a wired interface in the local browser.
不会修改网络、安装机器人 helper、创建机器人 service 或自动连接机器人。
Does not change networking, install robot helpers/services, or connect automatically.
EOF
}
case "$command_name" in install|check|start|shortcut) ;; -h|--help) usage; exit 0 ;; *) usage >&2; exit 2 ;; esac
while (($#)); do
  case "$1" in
    --sdk2-source|--sdk-prefix|--build-dir|--port|--a2-joint-gains)
      (($# >= 2)) || { usage >&2; exit 2; }
      option="$1"; value="$2"; shift
      case "$command_name:$option" in
        install:--sdk2-source|install:--sdk-prefix|install:--build-dir|check:--sdk-prefix|check:--build-dir|start:--build-dir|start:--port|start:--a2-joint-gains|shortcut:--build-dir|shortcut:--port|shortcut:--a2-joint-gains) ;;
        *) printf 'Invalid option / 参数不适用: %s %s\n' "$command_name" "$option" >&2; exit 2 ;;
      esac
      case "$option" in
        --sdk2-source) sdk_source="$(realpath -m -- "$value")" ;;
        --sdk-prefix) sdk_prefix="$(realpath -m -- "$value")"; sdk_prefix_explicit=true ;;
        --build-dir) build_dir="$(realpath -m -- "$value")" ;;
        --port) port="$value" ;;
        --a2-joint-gains) a2_joint_gains="$(realpath -e -- "$value")" ;;
      esac ;;
    --mock) [[ "$command_name" == start || "$command_name" == shortcut ]] || { usage >&2; exit 2; }; mock=true ;;
    -h|--help) usage; exit 0 ;;
    *) usage >&2; exit 2 ;;
  esac
  shift
done
fail() { ui_err "$1" "$2"; exit 1; }
[[ "$build_dir" != / && "$sdk_prefix" != / ]] || exit 2
[[ "$port" =~ ^[0-9]{1,5}$ ]] && ((10#$port >= 1 && 10#$port <= 65535)) || exit 2
[[ "$(uname -m)" == x86_64 ]] || fail '仅支持 x86_64' 'Only x86_64 is supported'
source /etc/os-release
[[ "$ID" == ubuntu && "$VERSION_ID" == 22.04 ]] || fail '仅支持 Ubuntu 22.04' 'Only Ubuntu 22.04 is supported'
for variable in ROS_DISTRO RMW_IMPLEMENTATION CYCLONEDDS_URI AMENT_PREFIX_PATH COLCON_PREFIX_PATH; do
  [[ -z "${!variable:-}" ]] || fail '请使用未加载 ROS/RMW/CycloneDDS 的终端' 'Use a clean terminal without ROS/RMW/CycloneDDS'
done
[[ "${LD_LIBRARY_PATH:-}:${CMAKE_PREFIX_PATH:-}:${PYTHONPATH:-}" != *'/ros/'* ]] ||
  fail '检测到 ROS 路径污染' 'ROS path contamination detected'

if [[ "$command_name" == start || "$command_name" == shortcut ]]; then
  [[ -x "$build_dir/g1_web_server" && -f "$build_dir/pc-sdk-prefix" ]] ||
    fail '请先执行 pc.sh install' 'Run pc.sh install first'
  if [[ "$command_name" == shortcut ]]; then
    args=(create --build-dir "$build_dir" --port "$port")
    if $mock; then args+=(--mock); fi
    if [[ -n "$a2_joint_gains" ]]; then args+=(--a2-joint-gains "$a2_joint_gains"); fi
    exec python3 "$project_dir/scripts/pc_shortcut.py" "${args[@]}"
  fi
  IFS= read -r sdk_prefix < "$build_dir/pc-sdk-prefix"
  [[ -d "$sdk_prefix/lib" ]] || fail 'SDK 路径失效，请重新安装' 'SDK path is unavailable; reinstall'
  export LD_LIBRARY_PATH="$sdk_prefix/lib"
  args=(--binary "$build_dir/g1_web_server" --port "$port")
  if $mock; then args+=(--mock); fi
  if [[ -n "$a2_joint_gains" ]]; then args+=(--a2-joint-gains "$a2_joint_gains"); fi
  exec python3 "$project_dir/scripts/pc_session.py" "${args[@]}"
fi

if [[ "$command_name" == check && "$sdk_prefix_explicit" == false && -f "$build_dir/pc-sdk-prefix" ]]; then
  IFS= read -r sdk_prefix < "$build_dir/pc-sdk-prefix"
fi

packages=(build-essential cmake pkg-config git python3 libboost-system-dev libboost-thread-dev libjsoncpp-dev libcurl4-openssl-dev libopencv-dev
          gstreamer1.0-tools gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-libav)
missing=()
for package in "${packages[@]}"; do
  if [[ "$(dpkg-query -W -f='${Status}' "$package" 2>/dev/null || true)" != 'install ok installed' ]]; then
    missing+=("$package")
  fi
done
if ((${#missing[@]})); then
  ui_err "缺少系统包：${missing[*]}" "Missing Ubuntu packages: ${missing[*]}"
  [[ "$command_name" != check ]] || exit 1
  sudo apt-get update
  sudo apt-get install -y -- "${missing[@]}"
fi
sdk_ready=false
if [[ -f "$sdk_prefix/.unirobo-sdk-revision" ]] &&
   [[ "$(cat "$sdk_prefix/.unirobo-sdk-revision")" == "$sdk_revision" ]] &&
   [[ -f "$sdk_prefix/lib/cmake/unitree_sdk2/unitree_sdk2Config.cmake" &&
      -f "$sdk_prefix/include/unitree/robot/r1/audio/audio_client.hpp" &&
      -f "$sdk_prefix/include/unitree/robot/a2/sport/sport_client.hpp" &&
      -f "$sdk_prefix/lib/libunitree_sdk2.a" && -f "$sdk_prefix/lib/libddsc.so" &&
      -f "$sdk_prefix/lib/libddscxx.so" ]]; then
  sdk_ready=true
fi
if [[ "$command_name" == check ]]; then
  $sdk_ready || fail '固定版本 SDK 尚未安装，请运行 pc.sh install' 'Pinned SDK is missing; run pc.sh install'
  ui_line '电脑安装前置检查通过；未修改文件、网络或服务。' 'PC preflight passed; no files, networking or services changed.'
  exit 0
fi
if ! $sdk_ready; then
  [[ ! -e "$sdk_prefix" ]] || fail 'SDK 目录已存在但不完整，请指定新目录' 'SDK prefix exists but is incomplete; choose a new directory'
  if [[ -z "$sdk_source" ]]; then
    sdk_source="${HOME}/.cache/unirobogui/unitree_sdk2-${sdk_revision}"
    if [[ ! -d "$sdk_source/.git" ]]; then
      git clone https://github.com/unitreerobotics/unitree_sdk2.git "$sdk_source"
      git -C "$sdk_source" checkout --detach "$sdk_revision"
    fi
  fi
  [[ "$(git -C "$sdk_source" rev-parse HEAD)" == "$sdk_revision" &&
     -z "$(git -C "$sdk_source" status --porcelain --untracked-files=no)" ]] ||
    fail 'SDK 源码必须是指定提交且没有修改' 'SDK source must be clean at the pinned revision'
  cmake -S "$sdk_source" -B "$build_dir/sdk2" -DBUILD_EXAMPLES=OFF \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$sdk_prefix" -DCMAKE_INSTALL_LIBDIR=lib
  cmake --build "$build_dir/sdk2" --parallel 2
  cmake --install "$build_dir/sdk2"
  printf '%s\n' "$sdk_revision" > "$sdk_prefix/.unirobo-sdk-revision"
fi
export LD_LIBRARY_PATH="$sdk_prefix/lib"
cmake -S "$project_dir" -B "$build_dir" -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=ON -DUNIROBO_PC_BUILD=ON \
  -Dunitree_sdk2_DIR="$sdk_prefix/lib/cmake/unitree_sdk2"
cmake --build "$build_dir" --parallel 2
ctest --test-dir "$build_dir" --output-on-failure
libraries="$(ldd "$build_dir/g1_web_server")"
printf '%s\n' "$libraries"
[[ "$libraries" != *'not found'* && "$libraries" != *'/opt/ros/'* ]] ||
  fail '动态库检查失败' 'Dynamic library validation failed'
for library in libddsc.so libddscxx.so; do
  [[ "$libraries" == *"$sdk_prefix/lib/$library"* ]] ||
    fail 'DDS 动态库未来自指定 SDK' 'DDS library does not come from the selected SDK'
done
printf '%s\n' "$sdk_prefix" > "$build_dir/pc-sdk-prefix"
printf -v next_command 'bash %q start --target pc --build-dir %q' "$project_dir/scripts/deploy.sh" "$build_dir"
ui_line "PC 安装完成。启动：$next_command；浏览器访问 http://127.0.0.1:8080" \
  "PC installed. Start: $next_command; open http://127.0.0.1:8080"
ui_line '可选快捷方式：将上述 start 改为 shortcut。' 'Optional shortcuts: replace start above with shortcut.'
