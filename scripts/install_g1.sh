#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
printf '%s\n' '[DEPRECATED] Use: bash scripts/deploy.sh install --product g1' >&2
exec bash "${script_dir}/install.sh" --product g1 "$@"
