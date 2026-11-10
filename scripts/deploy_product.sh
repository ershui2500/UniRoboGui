#!/usr/bin/env bash

load_product_profile() {
  local product="${1:-}"
  TARGET_ARCH=aarch64
  TARGET_UBUNTU_VERSION=20.04
  KOKORO_PYTHON_MAJOR=3
  KOKORO_PYTHON_MINOR=8
  KOKORO_PYTHON_VERSION=38
  KOKORO_PYTHON_ABI=cp38
  KOKORO_PYTHON_DISPLAY=3.8
  KOKORO_PIP_PLATFORM=manylinux2014_aarch64
  KOKORO_WHEELHOUSE_SUFFIX=aarch64-py38

  case "$product" in
    g1)
      PRODUCT_ID=g1
      PRODUCT_NAME="Unitree G1"
      DEFAULT_HOST="unitree@192.168.123.164"
      DDS_INTERFACE=eth0
      WEB_SERVICE="g1-web-control.service"
      OTHER_WEB_SERVICE="r1-web-control.service go2-web-control.service a2-web-control.service"
      WEB_SYSTEM_UNIT="deploy/g1-web-control.service"
      WEB_USER_UNIT="deploy/g1-web-control.user.service"
      SYSTEM_SDK_PREFIX="/opt/unitree_robotics"
      USER_SDK_PREFIX="${HOME}/.local/unitree_robotics"
      SDK_POLICY=install_if_missing
      NEEDS_REALSENSE=true
      NEEDS_KOKORO=true
      CAMERA_HELPER="g1-web-first-person-service"
      CAMERA_SUDOERS="g1-web-camera.sudoers"
      ;;
    r1)
      PRODUCT_ID=r1
      PRODUCT_NAME="Unitree R1"
      DEFAULT_HOST="unitree@192.168.123.164"
      DDS_INTERFACE=eth10
      WEB_SERVICE="r1-web-control.service"
      OTHER_WEB_SERVICE="g1-web-control.service go2-web-control.service a2-web-control.service"
      WEB_SYSTEM_UNIT="deploy/r1-web-control.service"
      WEB_USER_UNIT="deploy/r1-web-control.user.service"
      SYSTEM_SDK_PREFIX="/usr/local"
      USER_SDK_PREFIX="/usr/local"
      SDK_POLICY=verify_existing
      NEEDS_REALSENSE=false
      NEEDS_KOKORO=false
      CAMERA_HELPER="r1-web-camera-service"
      CAMERA_SUDOERS="r1-web-camera.sudoers"
      ;;
    go2)
      PRODUCT_ID=go2
      PRODUCT_NAME="Unitree GO2"
      DEFAULT_HOST="unitree@192.168.123.18"
      DDS_INTERFACE=eth0
      WEB_SERVICE="go2-web-control.service"
      OTHER_WEB_SERVICE="g1-web-control.service r1-web-control.service a2-web-control.service"
      WEB_SYSTEM_UNIT="deploy/go2-web-control.service"
      WEB_USER_UNIT="deploy/go2-web-control.user.service"
      SYSTEM_SDK_PREFIX="/usr/local"
      USER_SDK_PREFIX="/usr/local"
      SDK_POLICY=verify_existing
      NEEDS_REALSENSE=false
      NEEDS_KOKORO=true
      CAMERA_HELPER=""
      CAMERA_SUDOERS=""
      ;;
    a2)
      PRODUCT_ID=a2
      PRODUCT_NAME="Unitree A2"
      DEFAULT_HOST="unitree@192.168.123.162"
      DDS_INTERFACE=eth0
      WEB_SERVICE="a2-web-control.service"
      OTHER_WEB_SERVICE="g1-web-control.service r1-web-control.service go2-web-control.service"
      WEB_SYSTEM_UNIT="deploy/a2-web-control.service"
      WEB_USER_UNIT="deploy/a2-web-control.user.service"
      SYSTEM_SDK_PREFIX="/usr/local"
      USER_SDK_PREFIX="/usr/local"
      SDK_POLICY=verify_existing
      NEEDS_REALSENSE=false
      NEEDS_KOKORO=true
      CAMERA_HELPER=""
      CAMERA_SUDOERS=""
      TARGET_ARCH=x86_64
      TARGET_UBUNTU_VERSION=22.04
      KOKORO_PYTHON_MINOR=10
      KOKORO_PYTHON_VERSION=310
      KOKORO_PYTHON_ABI=cp310
      KOKORO_PYTHON_DISPLAY=3.10
      KOKORO_PIP_PLATFORM=manylinux2014_x86_64
      KOKORO_WHEELHOUSE_SUFFIX=x86_64-py310
      ;;
    *)
      printf 'Unsupported product: %s (use g1, r1, go2 or a2)\n' "${product:-<missing>}" >&2
      return 2
      ;;
  esac

  TTS_MODEL_NAME=kokoro-int8-multi-lang-v1_1
  TTS_MODEL_FILE=model.int8.onnx
  TTS_SERVICE=g1-local-tts.service
  TTS_SYSTEM_UNIT=deploy/g1-local-tts.service
  TTS_USER_UNIT=deploy/g1-local-tts.user.service
  if [[ "$PRODUCT_ID" == go2 ]]; then
    TTS_MODEL_NAME=kokoro-multi-lang-v1_1
    TTS_MODEL_FILE=model.onnx
    TTS_SERVICE=go2-local-tts.service
    TTS_SYSTEM_UNIT=deploy/go2-local-tts.service
    TTS_USER_UNIT=deploy/go2-local-tts.user.service
  fi
  if [[ "$PRODUCT_ID" == a2 ]]; then
    TTS_MODEL_NAME=kokoro-multi-lang-v1_1
    TTS_MODEL_FILE=model.onnx
    TTS_SERVICE=a2-local-tts.service
    TTS_SYSTEM_UNIT=deploy/a2-local-tts.service
    TTS_USER_UNIT=deploy/a2-local-tts.user.service
  fi
  export TTS_MODEL_NAME TTS_MODEL_FILE TTS_SERVICE TTS_SYSTEM_UNIT TTS_USER_UNIT

  export PRODUCT_ID PRODUCT_NAME DEFAULT_HOST DDS_INTERFACE WEB_SERVICE OTHER_WEB_SERVICE
  export WEB_SYSTEM_UNIT WEB_USER_UNIT SYSTEM_SDK_PREFIX USER_SDK_PREFIX
  export SDK_POLICY NEEDS_REALSENSE NEEDS_KOKORO CAMERA_HELPER CAMERA_SUDOERS
  export TARGET_ARCH TARGET_UBUNTU_VERSION KOKORO_PYTHON_MAJOR KOKORO_PYTHON_MINOR
  export KOKORO_PYTHON_VERSION KOKORO_PYTHON_ABI KOKORO_PYTHON_DISPLAY
  export KOKORO_PIP_PLATFORM KOKORO_WHEELHOUSE_SUFFIX
}

resolve_product_host() {
  local requested="${1:-}"
  RESOLVED_HOST="${requested:-$DEFAULT_HOST}"
  if [[ "$RESOLVED_HOST" == -* || ! "$RESOLVED_HOST" =~ ^([a-zA-Z0-9._-]+@)?[a-zA-Z0-9][a-zA-Z0-9._-]*$ ]]; then
    ui_err '--host 必须是 SSH Host 别名、主机名或 USER@IPv4/主机名；不能包含选项或命令。' \
      '--host must be an SSH Host alias, hostname or USER@IPv4/hostname; options and commands are not allowed.'
    return 2
  fi
}

# Paths are data, never shell fragments. Remote paths must already be absolute.
normalize_deploy_path() {
  local value="$1"
  [[ "$value" == /* && ! "$value" =~ [[:cntrl:]] && "$value" != *:* && "$value" != *\;* ]] || {
    ui_err '路径必须是绝对路径，不能包含控制字符、冒号或分号。' \
      'Use an absolute path without control characters, colon or semicolon.'
    return 2
  }
  value="$(realpath -ms -- "$value")" || return 2
  [[ "$value" != / && "$value" != //* ]] || {
    ui_err '不能使用根目录。' 'The filesystem root is not a deployment directory.'
    return 2
  }
  printf '%s\n' "$value"
}

# POSIX quoting also works when the SSH server uses /bin/sh.
remote_quote() {
  local value="${1//\'/\'\\\'\'}"
  printf "'%s'" "$value"
}

# Shared dependency options keep all public and legacy entries consistent.
parse_dependency_option() {
  case "$1" in
    --sdk-prefix|--sdk2-source|--realsense-source|--realsense-prefix)
      (($# >= 2)) && [[ -n "$2" ]] || { ui_err "$1 需要目录" "$1 requires a directory"; return 2; }
      local value
      value="$(normalize_deploy_path "$2")" || return 2
      case "$1" in
        --sdk-prefix) export UNIROBOGUI_SDK_PREFIX="$value" ;;
        --sdk2-source) export UNIROBOGUI_SDK2_SOURCE="$value" ;;
        --realsense-source) export UNIROBOGUI_REALSENSE_SOURCE="$value" ;;
        --realsense-prefix) export UNIROBOGUI_REALSENSE_PREFIX="$value" ;;
      esac
      ;;
    --install-sdk2) export UNIROBOGUI_INSTALL_SDK2=1 ;;
    *) return 2 ;;
  esac
}

print_dependency_help() {
  ui_line '路径选项（均使用绝对路径；含空格时加引号）：' \
    'Path options (absolute paths; quote paths containing spaces):'
  ui_line '  --sdk-prefix DIR        已有 SDK2 或新安装前缀' '  --sdk-prefix DIR        Existing SDK2 or new installation prefix'
  ui_line '  --sdk2-source DIR       机器人上的 SDK2 源码目录' '  --sdk2-source DIR       SDK2 source directory on the robot'
  ui_line '  --install-sdk2          缺失时允许安装 SDK2（所有产品；不覆盖已有 SDK）' \
    '  --install-sdk2          Allow SDK2 installation if missing (all products; no overwrite)'
  ui_line '  --realsense-source DIR  机器人上的 librealsense 源码目录' '  --realsense-source DIR  librealsense source directory on the robot'
  ui_line '  --realsense-prefix DIR  librealsense 前缀（默认 /usr/local）' '  --realsense-prefix DIR  librealsense prefix (default /usr/local)'
  ui_line '  --install-dir DIR       仅 from-pc；机器人项目目录（默认 /home/unitree/UniRoboGui）' \
    '  --install-dir DIR       from-pc only; robot project directory (default /home/unitree/UniRoboGui)'
  ui_line '本机项目目录自动识别；重新运行时请带上自定义依赖参数。' \
    'The local project root is detected automatically; pass custom dependency options again when rerunning.'
}
