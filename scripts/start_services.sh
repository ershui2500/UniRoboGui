#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
source "${script_dir}/deploy_i18n.sh"
source "${script_dir}/deploy_product.sh"

usage() {
  ui_line '用法：bash scripts/start_services.sh --product g1|r1|go2' \
    'Usage: bash scripts/start_services.sh --product g1|r1|go2'
  ui_line '启动已安装的对应 systemd 服务；不重新安装、不重启正在运行的服务。' \
    'Start the installed product systemd services; does not reinstall or restart running services.'
}

product=""
while (($#)); do
  case "$1" in
    --product)
      (($# >= 2)) || { usage >&2; exit 2; }
      product="$2"; shift ;;
    -h|--help) usage; exit 0 ;;
    *) usage >&2; exit 2 ;;
  esac
  shift
done
[[ -n "$product" ]] || { usage >&2; exit 2; }
load_product_profile "$product"

for other in ${OTHER_WEB_SERVICE}; do
  if systemctl is-active --quiet "$other" 2>/dev/null ||
     systemctl --user is-active --quiet "$other" 2>/dev/null; then
    ui_err "另一产品服务 $other 正在运行；请先确认现场产品。" \
      "Another product service ($other) is active; verify the on-site product first."
    exit 1
  fi
done

scope=system
ctl=(systemctl)
if systemctl --user is-active --quiet "$WEB_SERVICE" 2>/dev/null ||
   { ! systemctl is-active --quiet "$WEB_SERVICE" 2>/dev/null &&
     ! systemctl is-enabled --quiet "$WEB_SERVICE" 2>/dev/null &&
     systemctl --user is-enabled --quiet "$WEB_SERVICE" 2>/dev/null; } ||
   [[ "$(systemctl show "$WEB_SERVICE" -p LoadState --value 2>/dev/null || true)" != loaded ]]; then
  scope=user
  ctl=(systemctl --user)
  if [[ "$("${ctl[@]}" show "$WEB_SERVICE" -p LoadState --value 2>/dev/null || true)" != loaded ]]; then
    ui_err "未安装 $WEB_SERVICE；请先运行 bash scripts/deploy.sh install --product $PRODUCT_ID。" \
      "$WEB_SERVICE is not installed; run bash scripts/deploy.sh install --product $PRODUCT_ID first."
    exit 1
  fi
fi

exec_start="$("${ctl[@]}" show "$WEB_SERVICE" -p ExecStart --value)"
if [[ "$exec_start" != *g1_web_server* || "$exec_start" != *"--robot $PRODUCT_ID "* ||
      "$exec_start" != *"--interface $DDS_INTERFACE "* ]]; then
  ui_err '已安装服务的程序、产品或网卡参数不匹配；请先检查该 unit。' \
    'The installed unit executable, product or interface does not match; inspect the unit first.'
  exit 1
fi
if ss -ltn 'sport = :8080' | grep -q LISTEN &&
   ! "${ctl[@]}" is-active --quiet "$WEB_SERVICE"; then
  ui_err '8080 已被其他进程占用；请检查精确 unit 与 MainPID。' \
    'Port 8080 is occupied by another process; inspect the exact unit and MainPID.'
  exit 1
fi

start=("${ctl[@]}")
if [[ "$scope" == system && "$EUID" -ne 0 ]]; then
  start=(sudo -n "${ctl[@]}")
fi
if ${NEEDS_KOKORO} &&
   [[ "$("${ctl[@]}" show "$TTS_SERVICE" -p LoadState --value 2>/dev/null || true)" == loaded ]]; then
  "${ctl[@]}" is-active --quiet "$TTS_SERVICE" || "${start[@]}" start "$TTS_SERVICE"
fi
"${ctl[@]}" is-active --quiet "$WEB_SERVICE" || "${start[@]}" start "$WEB_SERVICE"
ui_line "已请求启动 $WEB_SERVICE（$scope）；检查页面或服务日志确认就绪。" \
  "Start requested for $WEB_SERVICE ($scope); check the page or service logs for readiness."
