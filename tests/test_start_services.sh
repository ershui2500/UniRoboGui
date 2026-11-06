#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
TMP="$(mktemp -d)"
trap 'rm -rf -- "$TMP"' EXIT
mkdir -p "$TMP/bin"
export START_TEST_LOG="$TMP/log"
cat >"$TMP/bin/systemctl" <<'STUB'
#!/usr/bin/env bash
printf '%s\n' "$*" >>"$START_TEST_LOG"
scope=system
if [[ "${1:-}" == --user ]]; then scope=user; shift; fi
case "$1" in
  is-active)
    [[ "$*" == *"${ACTIVE_OTHER:-none}"* ]] ||
      [[ "$scope" == "${ACTIVE_WEB_SCOPE:-none}" && "$*" == *go2-web-control.service* ]] ;;
  is-enabled) [[ "$scope" == "${ENABLED_SCOPE:-none}" ]] ;;
  show)
    if [[ "$*" == *LoadState* ]]; then
      if [[ "${TEST_SCOPE:-system}" == both || "$scope" == "${TEST_SCOPE:-system}" ]]; then echo loaded; else echo not-found; fi
    else
      printf '/home/unitree/UniRoboGui/build/g1_web_server --interface %s --robot %s ;\n' \
        "${TEST_INTERFACE:-eth0}" "${TEST_PRODUCT:-go2}"
    fi ;;
  start) exit 0 ;;
  *) exit 1 ;;
esac
STUB
cat >"$TMP/bin/ss" <<'STUB'
#!/usr/bin/env bash
if [[ "${PORT_BUSY:-0}" == 1 ]]; then echo LISTEN; fi
exit 0
STUB
cat >"$TMP/bin/sudo" <<'STUB'
#!/usr/bin/env bash
[[ "$1" == -n ]] || exit 1
shift
exec "$@"
STUB
chmod +x "$TMP/bin/"*
export PATH="$TMP/bin:$PATH"
expect_failure() {
  : >"$START_TEST_LOG"
  if "$@" >"$TMP/output" 2>&1; then
    printf 'Expected failure: %s\n' "$*" >&2; exit 1
  fi
  if grep -q '^\(--user \)\?start ' "$START_TEST_LOG"; then
    echo 'Unsafe start after failed preflight' >&2; exit 1
  fi
}
: >"$START_TEST_LOG"
bash "$ROOT/scripts/start_services.sh" --help >"$TMP/output"
test ! -s "$START_TEST_LOG"
expect_failure bash "$ROOT/scripts/start_services.sh"
expect_failure bash "$ROOT/scripts/start_services.sh" --product unknown
expect_failure env TEST_SCOPE=missing bash "$ROOT/scripts/start_services.sh" --product go2
expect_failure env ACTIVE_OTHER=r1-web-control.service bash "$ROOT/scripts/start_services.sh" --product go2
expect_failure env TEST_PRODUCT=g1 bash "$ROOT/scripts/start_services.sh" --product go2
expect_failure env TEST_INTERFACE=eth10 bash "$ROOT/scripts/start_services.sh" --product go2
expect_failure env PORT_BUSY=1 bash "$ROOT/scripts/start_services.sh" --product go2
for product in g1 r1 go2 a2; do
  interface=eth0
  [[ "$product" != r1 ]] || interface=eth10
  for scope in system user; do
    : >"$START_TEST_LOG"
    TEST_SCOPE="$scope" TEST_PRODUCT="$product" TEST_INTERFACE="$interface" \
      bash "$ROOT/scripts/start_services.sh" --product "$product" >"$TMP/output"
    prefix=""
    [[ "$scope" != user ]] || prefix='--user '
    grep -Fxq -- "${prefix}start $product-web-control.service" "$START_TEST_LOG"
    if [[ "$product" == r1 ]]; then
      ! grep -q 'start .*local-tts' "$START_TEST_LOG"
    else
      grep -Fxq -- "${prefix}start $product-local-tts.service" "$START_TEST_LOG"
    fi
    ! grep -Eq '(^| )(stop|restart|enable|disable)( |$)' "$START_TEST_LOG"
  done
done
: >"$START_TEST_LOG"
TEST_PRODUCT=r1 TEST_INTERFACE=eth10 bash "$ROOT/scripts/start_services.en.sh" --product r1 >"$TMP/output"
grep -q 'Start requested for r1-web-control.service' "$TMP/output"

# A disabled system template must not shadow the user's installed/running service.
for active_scope in user none; do
  : >"$START_TEST_LOG"
  TEST_SCOPE=both ACTIVE_WEB_SCOPE="$active_scope" ENABLED_SCOPE=user \
    bash "$ROOT/scripts/start_services.sh" --product go2 >"$TMP/output"
  ! grep -Fxq 'start go2-web-control.service' "$START_TEST_LOG"
  if [[ "$active_scope" == none ]]; then
    grep -Fxq -- '--user start go2-web-control.service' "$START_TEST_LOG"
  else
    ! grep -q 'start .*web-control' "$START_TEST_LOG"
  fi
done
printf '%s\n' 'start-services regression passed'
