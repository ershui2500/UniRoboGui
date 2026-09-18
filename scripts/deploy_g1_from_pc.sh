#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
translated=()
while (($#)); do
  case "$1" in
    --robot)
      (($# >= 2)) || { printf '%s\n' '[ERROR] --robot requires USER@HOST' >&2; exit 2; }
      translated+=(--host "$2")
      shift
      ;;
    *) translated+=("$1") ;;
  esac
  shift
done
printf '%s\n' '[DEPRECATED] Use: bash scripts/deploy.sh from-pc --product g1 --host USER@HOST' >&2
exec bash "${script_dir}/deploy_from_pc.sh" --product g1 "${translated[@]}"
