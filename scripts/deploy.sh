#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"

source "$project_dir/scripts/deploy_i18n.sh"
source "$project_dir/scripts/deploy_product.sh"

usage() {
  cat <<'EOF'
用法 / Usage:
  bash scripts/deploy.sh <COMMAND> [--target robot] --product <g1|r1|go2|a2> [OPTIONS]
  bash scripts/deploy.sh <install|check|start|shortcut> --target pc [OPTIONS]

推荐且默认安装到机器人：对应产品的功能覆盖最完整，实际能力取决于附件、服务和验证状态。
Recommended/default: install on the robot for the fullest implemented feature set for that product.
Availability depends on model, attachments, services and validation.
PC 是可选方式，部分相机、深度、本地语音和感知/SLAM 功能受限；详见 docs/pc-guide.md。
PC is optional with camera, depth, local speech and perception/SLAM limitations; see docs/pc-guide.en.md.

命令 / Commands:
  install    在所选目标安装/升级（默认机器人）；在线安装需访问依赖源
             Install/upgrade on the target (default: robot); dependency sources must be reachable
  from-pc   在联网 Linux 电脑准备资源，再通过 SSH/rsync 部署到机器人
             Prepare resources on an online Linux PC, then deploy over SSH/rsync
  start     仅 PC：启动本机工作台 / PC only: start the local workbench
  shortcut  仅 PC：创建当前用户快捷方式 / PC only: create user shortcuts
  check     只读检查所选安装环境，不安装、不重启服务
             Read-only target preflight; does not install or restart services

from-pc 最终安装到机器人，不是 PC 安装。
from-pc installs on the robot, not on the PC.

参数 / Options:
  --target robot|pc     默认 robot（推荐）；PC 须显式指定 / Default: robot (recommended)
  --product g1|r1|go2|a2    机器人端必填；PC 在网页选产品 / Required for robot; select in the PC web UI
  --host USER@HOST      仅 from-pc；可选；默认使用产品配置地址
                        from-pc only; optional; defaults to the product profile address
  --lang zh|en          可选；输出语言，默认 zh / Optional output language; default: zh
  --build-dir DIR      PC：自定义构建目录 / PC: custom build directory
  --port PORT          PC start/shortcut：端口 / Port (default: 8080)
  --mock               PC start/shortcut：离线模拟 / Offline simulation
  --a2-joint-gains FILE PC start/shortcut：现场确认的增益 / Operator-provided gains
  --sdk-prefix DIR     PC install/check：独立 SDK 前缀 / Independent SDK prefix
  --sdk2-source DIR    PC install：固定提交的源码 / Source at the pinned revision
  -h, --help            显示帮助 / Show this help

常用示例 / Examples:
  bash scripts/deploy.sh install --product g1
  bash scripts/deploy.sh install --product r1
  bash scripts/deploy.sh install --product go2
  bash scripts/deploy.sh install --product a2
  bash scripts/deploy.sh from-pc --product g1
  bash scripts/deploy.sh from-pc --product r1
  bash scripts/deploy.sh from-pc --product go2
  bash scripts/deploy.sh from-pc --product a2
  bash scripts/deploy.sh from-pc --product g1 --host unitree@192.168.123.164
  bash scripts/deploy.sh from-pc --product r1 --host unitree@192.168.123.164
  bash scripts/deploy.sh check --product g1
  bash scripts/deploy.sh check --product r1
  bash scripts/deploy.sh check --product go2
  bash scripts/deploy.sh check --product a2
  bash scripts/deploy.sh install --product g1 --lang en
  bash scripts/deploy.sh install --product r1 --lang en
  bash scripts/deploy.sh install --target pc
  bash scripts/deploy.sh check --target pc
  bash scripts/deploy.sh start --target pc
  bash scripts/deploy.sh shortcut --target pc

不要原样输入 <g1|r1|go2|a2>；请根据实际机器人填写产品 ID。产品没有隐式默认值。
Do not type <g1|r1|go2|a2> literally; use the actual product ID. There is no implicit product default.
EOF
  printf '\n机器人端路径选项 / Robot deployment path options:\n'
  print_dependency_help
}

(($# > 0)) || { usage >&2; exit 2; }
command_name="$1"
shift

target=robot
pc_args=()
robot_args=()
product=""
host=""
lang="${UNIROBOGUI_LANG:-zh}"
while (($#)); do
  case "$1" in
    --target)
      (($# >= 2)) || { usage >&2; exit 2; }
      target="$2"; shift ;;
    --install-dir|--realsense-source|--realsense-prefix)
      (($# >= 2)) || { usage >&2; exit 2; }
      robot_args+=("$1" "$2"); shift ;;
    --install-sdk2) robot_args+=("$1") ;;
    --sdk-prefix|--sdk2-source|--build-dir|--port|--a2-joint-gains)
      (($# >= 2)) || { usage >&2; exit 2; }
      pc_args+=("$1" "$2"); shift ;;
    --mock) pc_args+=("$1") ;;
    --product)
      (($# >= 2)) || { printf '%s\n' '--product requires a value' >&2; exit 2; }
      product="$2"
      shift
      ;;
    --host)
      (($# >= 2)) || { printf '%s\n' '--host requires a value' >&2; exit 2; }
      host="$2"
      shift
      ;;
    --lang)
      (($# >= 2)) || { printf '%s\n' '--lang requires a value' >&2; exit 2; }
      lang="$2"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      printf 'Unknown option: %s\n' "$1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

case "$command_name" in
  install|from-pc|check|start|shortcut) ;;
  -h|--help) usage; exit 0 ;;
  *) printf 'Unknown command: %s\n' "$command_name" >&2; usage >&2; exit 2 ;;
esac

export UNIROBOGUI_LANG="$lang"
source "$project_dir/scripts/deploy_i18n.sh"
case "$target" in
  pc)
    [[ "$command_name" != from-pc && -z "$product" && -z "$host" && ${#robot_args[@]} == 0 ]] || {
      ui_err 'PC 不接受 from-pc、--product、--host 或机器人专用参数' 'PC does not accept from-pc, --product, --host or robot-only options'; exit 2;
    }
    ui_line '目标：本机 PC（可选，部分功能受限）' 'Target: local PC (optional; some features are limited)'
    exec bash "$project_dir/scripts/pc.sh" "$command_name" "${pc_args[@]}" ;;
  robot) ;;
  *) ui_err '无效 --target；请使用 robot 或 pc' 'Invalid --target; use robot or pc'; exit 2 ;;
esac
[[ "$command_name" != start && "$command_name" != shortcut ]] || {
  ui_err 'start/shortcut 仅用于 --target pc' 'start/shortcut require --target pc'; exit 2;
}
# Preserve the robot dependency parser and its path validation.
set -- "${pc_args[@]}" "${robot_args[@]}"
while (($#)); do
  case "$1" in
    --install-dir)
      UNIROBOGUI_INSTALL_DIR="$(normalize_deploy_path "$2")" || exit 2
      export UNIROBOGUI_INSTALL_DIR; shift ;;
    --sdk-prefix|--sdk2-source|--realsense-source|--realsense-prefix)
      parse_dependency_option "$@" || exit 2; shift ;;
    --install-sdk2) parse_dependency_option "$1" ;;
    *) ui_err "机器人端不支持参数：$1" "Unsupported robot option: $1"; exit 2 ;;
  esac
  shift
done
[[ -n "$product" ]] || { printf '%s\n' '--product g1|r1|go2|a2 is required' >&2; exit 2; }
ui_line '目标：机器人（推荐、默认；对应产品功能覆盖最完整）' 'Target: robot (recommended/default; fullest feature set for the product)'
load_product_profile "$product"

if [[ "$command_name" != from-pc && -n "${UNIROBOGUI_INSTALL_DIR:-}" ]]; then
  ui_err "--install-dir 仅可用于 from-pc；本机自动检测项目目录" "--install-dir is only valid for from-pc; local deployment detects the checkout"
  exit 2
fi

if [[ "$command_name" != from-pc && -n "$host" ]]; then
  ui_err "--host 仅可用于 from-pc" "--host is only valid for from-pc"
  exit 2
fi

case "$command_name" in
  install)
    exec bash "$project_dir/scripts/deploy_online.sh" --product "$PRODUCT_ID"
    ;;
  from-pc)
    resolve_product_host "$host"
    host="$RESOLVED_HOST"
    exec bash "$project_dir/scripts/deploy_from_pc.sh" \
      --product "$PRODUCT_ID" --host "$host"
    ;;
  check)
    exec bash "$project_dir/scripts/install.sh" --product "$PRODUCT_ID" --check-only
    ;;
esac
