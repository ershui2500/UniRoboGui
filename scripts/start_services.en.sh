#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
export UNIROBOGUI_LANG=en
exec bash "${script_dir}/start_services.sh" "$@"
