#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"

usage() {
  cat <<'EOF'
用法 / Usage:
  bash scripts/deploy.sh <COMMAND> --product <g1|r1|go2|a2> [OPTIONS]

命令 / Commands:
  install    在机器人本机安装/升级；机器人需要能访问 GitHub
             Install/upgrade on the robot; the robot must be able to reach GitHub
  from-pc   在联网 Linux 电脑准备资源，再通过 SSH/rsync 部署到机器人
             Prepare resources on an online Linux PC, then deploy over SSH/rsync
  check     只读检查机器人环境，不安装、不重启服务
             Read-only robot preflight; does not install or restart services

参数 / Options:
  --product g1|r1|go2|a2    必填；选择 Unitree 产品 / Required; select a Unitree product
  --host USER@HOST      仅 from-pc；可选；默认使用产品配置地址
                        from-pc only; optional; defaults to the product profile address
  --lang zh|en          可选；输出语言，默认 zh / Optional output language; default: zh
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

不要原样输入 <g1|r1|go2|a2>；请根据实际机器人填写产品 ID。产品没有隐式默认值。
Do not type <g1|r1|go2|a2> literally; use the actual product ID. There is no implicit product default.
EOF
}

(($# > 0)) || { usage >&2; exit 2; }
command_name="$1"
shift

product=""
host=""
lang="${UNIROBOGUI_LANG:-zh}"
while (($#)); do
  case "$1" in
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
  install|from-pc|check) ;;
  -h|--help) usage; exit 0 ;;
  *) printf 'Unknown command: %s\n' "$command_name" >&2; usage >&2; exit 2 ;;
esac

[[ -n "$product" ]] || { printf '%s\n' '--product g1|r1|go2|a2 is required' >&2; exit 2; }
export UNIROBOGUI_LANG="$lang"
source "$project_dir/scripts/deploy_i18n.sh"
source "$project_dir/scripts/deploy_product.sh"
load_product_profile "$product"

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
