#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
source "${script_dir}/deploy_i18n.sh"

project_dir="/home/unitree/UniRoboGui"
if [[ -n "${ROS_DISTRO:-}" || -n "${RMW_IMPLEMENTATION:-}" || -n "${CYCLONEDDS_URI:-}" ]]; then
  ui_err "请在未加载 ROS/CycloneDDS 的终端中启动" "Start this script from a terminal where ROS/CycloneDDS is not loaded."
  exit 1
fi

cd "$project_dir"
mkdir -p logs

if ss -ltn 'sport = :8080' | grep -q LISTEN; then
  ui_line "8080 已有服务监听，未启动第二个 Web 进程" "Port 8080 is already listening; a second Web process was not started."
else
  nohup ./build/g1_web_server \
    --interface eth0 \
    --bind 0.0.0.0 \
    --port 8080 \
    --publish-hz 10 \
    --web-root "$project_dir/web" \
    > logs/g1_web_server.log 2>&1 &
  echo "$!" > logs/g1_web_server.pid
fi

sleep 2
ss -ltnp 'sport = :8080'
curl --noproxy '*' -fsS http://127.0.0.1:8080/api/health
echo
curl --noproxy '*' -fsS http://127.0.0.1:8080/api/perception/topics
echo
