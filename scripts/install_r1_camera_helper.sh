#!/bin/bash
set -euo pipefail

project_dir="${1:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
helper_source="${project_dir}/scripts/r1-web-camera-service"
sudoers_source="${project_dir}/scripts/r1-web-camera.sudoers"

if [[ ! -f "${helper_source}" || ! -f "${sudoers_source}" ]]; then
  echo "R1 camera helper sources are missing under ${project_dir}/scripts" >&2
  exit 1
fi

sudo install -m 0755 "${helper_source}" /usr/local/sbin/r1-web-camera-service
sudo visudo -cf "${sudoers_source}"
sudo install -m 0440 "${sudoers_source}" /etc/sudoers.d/r1-web-camera

helper_rc=0
sudo -n /usr/local/sbin/r1-web-camera-service depth-is-active >/dev/null 2>&1 || helper_rc=$?
if [[ ${helper_rc} -ne 0 && ${helper_rc} -ne 3 ]]; then
  echo "R1 camera helper sudoers verification failed with exit ${helper_rc}." >&2
  exit 1
fi

echo "R1 camera helper and sudoers installed successfully."
