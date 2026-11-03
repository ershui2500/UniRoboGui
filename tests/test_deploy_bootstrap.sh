#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT
fail() { printf 'deployment bootstrap regression: %s\n' "$*" >&2; exit 1; }
export BOOTSTRAP_LOG="$TMP/bootstrap.log"
mkdir -p "$TMP/bin" "$TMP/repo/scripts" "$TMP/repo/.git"
cp "$ROOT/scripts/"{deploy_from_pc.sh,deploy_i18n.sh,deploy_product.sh} "$TMP/repo/scripts/"

# Invoke the complete real from-pc script. These stubs never connect or download.
cat >"$TMP/bin/git" <<'STUB'
#!/usr/bin/env bash
printf 'git %s\n' "$*" >>"$BOOTSTRAP_LOG"
case "$*" in
  *'remote get-url origin') echo https://github.com/ershui2500/UniRoboGui.git ;;
  *'status --short') [[ "${DIRTY_PC:-false}" != true ]] || echo ' M customer.cpp' ;;
  *'rev-parse'*) [[ ! -f "$TEST_REPO/updated" ]] && echo oldrevision || echo newrevision ;;
  *'pull --ff-only')
    if [[ "${UPDATE_PC:-false}" == true ]]; then
      touch "$TEST_REPO/updated"
      cat >"$TEST_REPO/scripts/deploy_from_pc.sh" <<'NEW'
#!/usr/bin/env bash
printf 'new-script %s\n' "$*" >>"$BOOTSTRAP_LOG"
NEW
    fi ;;
esac
exit 0
STUB
cat >"$TMP/bin/ssh" <<'STUB'
#!/usr/bin/env bash
printf 'ssh %s\n' "$*" >>"$BOOTSTRAP_LOG"
case "$*" in
  *'stat -c %Y'*) date +%s; date +%s ;;
  *'sudo -n true'*) exit 1 ;;
  *'test -e '*|*'test -d '*|*'test -f '*) exit 1 ;;
esac
STUB
cat >"$TMP/bin/curl" <<'STUB'
#!/usr/bin/env bash
printf 'curl %s\n' "$*" >>"$BOOTSTRAP_LOG"
[[ "$*" == *'-IL '* ]] || exit 99
STUB
cat >"$TMP/bin/rsync" <<'STUB'
#!/usr/bin/env bash
printf 'rsync %s\n' "$*" >>"$BOOTSTRAP_LOG"
STUB
cat >"$TMP/bin/python3" <<'STUB'
#!/usr/bin/env bash
printf 'python3 %s\n' "$*" >>"$BOOTSTRAP_LOG"
[[ "$*" == '-m pip '* ]] || exit 99
STUB
chmod +x "$TMP/bin/"*
export TEST_REPO="$TMP/repo"
export XDG_CACHE_HOME="$TMP/cache"
cache="$XDG_CACHE_HOME/unirobogui-deploy"
mkdir -p "$cache" "$TMP/model/kokoro-multi-lang-v1_1"
touch "$TMP/model/kokoro-multi-lang-v1_1/model.onnx"
tar -cjf "$cache/kokoro-multi-lang-v1_1.tar.bz2" -C "$TMP/model" kokoro-multi-lang-v1_1
PATH="$TMP/bin:$PATH" bash "$TEST_REPO/scripts/deploy_from_pc.sh" --product go2 >"$TMP/from-pc.out" 2>&1 || {
  cat "$TMP/from-pc.out" >&2; fail 'fresh PC flow failed';
}
grep -q 'rsync .*unitree@192.168.123.18:/home/unitree/UniRoboGui/' "$BOOTSTRAP_LOG" || fail 'GO2 project not transferred'
grep -q 'rsync .*kokoro-multi-lang-v1_1.tar.bz2' "$BOOTSTRAP_LOG" || fail 'FP32 archive not transferred'
grep -q "deploy_online.sh --product 'go2' --from-transferred-sources --wheelhouse" "$BOOTSTRAP_LOG" || fail 'robot install not reached without sudo'
! grep '^rsync ' "$BOOTSTRAP_LOG" | grep -q -- '--delete' || fail 'fresh destination used delete'

: >"$BOOTSTRAP_LOG"
if DIRTY_PC=true PATH="$TMP/bin:$PATH" bash "$TEST_REPO/scripts/deploy_from_pc.sh" --product go2 >"$TMP/dirty.out" 2>&1; then
  fail 'dirty PC source accepted'
fi
! grep -q '^rsync ' "$BOOTSTRAP_LOG" || fail 'dirty PC source transferred'

: >"$BOOTSTRAP_LOG"
UPDATE_PC=true PATH="$TMP/bin:$PATH" bash "$TEST_REPO/scripts/deploy_from_pc.sh" --product go2 >"$TMP/update.out" 2>&1
grep -q 'new-script --product go2 --host unitree@192.168.123.18' "$BOOTSTRAP_LOG" || fail 'updated script was not reentered'
! grep -q '^rsync ' "$BOOTSTRAP_LOG" || fail 'old script continued after pull'

# Robot prerequisites do not require system pip; only the PC downloader uses it.
for installer in install.sh deploy_online.sh; do
  packages="$(source <(sed -n '/^SYSTEM_PACKAGES=(/,/^)/p' "$ROOT/scripts/$installer"); printf '%s\n' "${SYSTEM_PACKAGES[@]}")"
  [[ "$packages" != *python3-pip* ]] || fail "$installer still requires system pip"
done

# Online system setup adds venv only when no existing/offline bootstrap exists.
source <(sed -n '/^prepare_kokoro_bootstrap() {$/,/^}$/p' "$ROOT/scripts/install.sh")
project_dir="$TMP/conditional-bootstrap"
without_kokoro=false
KOKORO_WHEELHOUSE=""
ensurepip_present=false
python3() { $ensurepip_present; }
ui_line() { :; }
fail_bootstrap() { printf '%s\n' "$2" >&2; exit 1; }
bootstrap_in_subshell() ( fail() { fail_bootstrap "$@"; }; prepare_kokoro_bootstrap )
install_scope=user
if bootstrap_in_subshell >"$TMP/bootstrap-preflight.out" 2>&1; then
  fail 'online user install accepted missing ensurepip and wheelhouse'
fi
grep -q 'compilation has not started' "$TMP/bootstrap-preflight.out" || fail 'bootstrap precondition unclear'
install_scope=system
SYSTEM_PACKAGES=()
prepare_kokoro_bootstrap
[[ "${SYSTEM_PACKAGES[*]}" == python3-venv ]] || fail 'online system install did not prepare venv'
install_scope=user
SYSTEM_PACKAGES=()
KOKORO_WHEELHOUSE="$TMP/conditional-wheels"
mkdir -p "$KOKORO_WHEELHOUSE"
touch "$KOKORO_WHEELHOUSE/pip-24.3.1-py3-none-any.whl"
prepare_kokoro_bootstrap
[[ "${#SYSTEM_PACKAGES[@]}" == 0 ]] || fail 'offline wheel bootstrap forced an APT dependency'
KOKORO_WHEELHOUSE=""
ensurepip_present=true
prepare_kokoro_bootstrap
[[ "${#SYSTEM_PACKAGES[@]}" == 0 ]] || fail 'working ensurepip forced an APT dependency'
ensurepip_present=false
mkdir -p "$project_dir/tts_kokoro/bin"
printf '#!/usr/bin/env bash\nexit 0\n' >"$project_dir/tts_kokoro/bin/python"
cp "$project_dir/tts_kokoro/bin/python" "$project_dir/tts_kokoro/bin/pip"
chmod +x "$project_dir/tts_kokoro/bin/"*
prepare_kokoro_bootstrap
[[ "${#SYSTEM_PACKAGES[@]}" == 0 ]] || fail 'existing project venv forced an APT dependency'
unset -f python3 ui_line
unset KOKORO_WHEELHOUSE

# Auto install in a real TTY falls back to user mode when sudo is unavailable.
python3 - "$ROOT" <<'PYTEST'
import os
import pty
import subprocess
import sys

root = sys.argv[1]
code = r'''
set -euo pipefail
source "$1/scripts/deploy_i18n.sh"
source <(sed -n '/^configure_install_scope() {$/,/^}$/p' "$1/scripts/install.sh")
sudo() { return 1; }
systemctl() { return 0; }
loginctl() { echo yes; }
id() { echo unitree; }
fail() { echo "$2" >&2; exit 1; }
warn() { :; }
install_scope_request="$2"
SYSTEM_SDK_PREFIX=/usr/local
USER_SDK_PREFIX=/usr/local
with_wifi_watchdog=false
configure_install_scope
[[ "$install_scope" == user ]]
'''
for scope, expected in [('auto', 0), ('system', 1), ('user', 0)]:
    master, slave = pty.openpty()
    try:
        result = subprocess.run(['bash', '-c', code, 'bash', root, scope],
                                stdin=slave, capture_output=True, text=True, timeout=5)
        assert result.returncode == expected, (scope, result.stdout, result.stderr)
    finally:
        os.close(slave)
        os.close(master)
PYTEST

# Invoke the real TTS installer with no system pip or ensurepip available.
fixture="$TMP/tts"
mkdir -p "$fixture/wheels" "$fixture/tts_models/kokoro-multi-lang-v1_1"
touch "$fixture/wheels/pip-24.3.1-py3-none-any.whl"
mkdir -p "$fixture/tts_models/kokoro-multi-lang-v1_1/espeak-ng-data"
for name in model.onnx voices.bin tokens.txt lexicon-us-en.txt lexicon-zh.txt phone-zh.fst date-zh.fst number-zh.fst; do
  touch "$fixture/tts_models/kokoro-multi-lang-v1_1/$name"
done
cat >"$TMP/python" <<'STUB'
#!/usr/bin/env bash
printf 'python[%s] %s\n' "$0" "$*" >>"$BOOTSTRAP_LOG"
case "$*" in
  '-m venv --without-pip '*)
    target="${@: -1}"
    mkdir -p "$target/bin"
    cp "$0" "$target/bin/python"
    chmod +x "$target/bin/python" ;;
  '-m venv '*) echo 'EXPECTED ensurepip probe traceback' >&2; exit 1 ;;
  '-m pip --version')
    [[ -x "$(dirname "$0")/pip" && ! -f "$(dirname "$0")/broken-pip" ]] ;;
  '-m pip install '*)
    [[ "$0" == *'/tts_kokoro/bin/python' ]] || exit 98
    [[ "$*" != *'--user'* ]] || exit 97
    touch "$(dirname "$0")/pip"
    chmod +x "$(dirname "$0")/pip"
    rm -f "$(dirname "$0")/broken-pip"
    [[ "$*" != *numpy* ]] || touch "$(dirname "$0")/dependencies-ready" ;;
  '- 1.13.4') echo 'EXPECTED dependency probe traceback' >&2; cat >/dev/null; test -f "$(dirname "$0")/dependencies-ready" ;;
  '-') cat >/dev/null ;;
  *) exit 96 ;;
esac
STUB
chmod +x "$TMP/python"
: >"$BOOTSTRAP_LOG"
UNIROBOGUI_ROOT="$fixture" KOKORO_WHEELHOUSE="$fixture/wheels" PYTHON="$TMP/python" \
  bash "$ROOT/scripts/install_kokoro_tts.sh" --product go2 >"$TMP/tts.out" 2>&1 || {
  cat "$TMP/tts.out" >&2; fail 'offline project venv bootstrap failed';
}
! grep -q 'EXPECTED .*traceback' "$TMP/tts.out" || fail 'expected probe tracebacks leaked to the customer'
grep -q 'ensurepip' "$TMP/tts.out" || fail 'offline fallback was not explained'
grep -q -- '-m venv --without-pip' "$BOOTSTRAP_LOG" || fail 'venv fallback not exercised'
grep -q -- '-m pip install --no-index --find-links' "$BOOTSTRAP_LOG" || fail 'offline pip bootstrap not exercised'
! grep -Eq -- '--user|--clear|virtualenv' "$BOOTSTRAP_LOG" || fail 'bootstrap changed user packages or cleared environment'
test -f "$fixture/tts_kokoro/bin/dependencies-ready" || fail 'TTS dependencies not installed in project venv'

# Executable pip wrappers with a broken pip module must reenter bootstrap.
touch "$fixture/tts_kokoro/bin/broken-pip"
: >"$BOOTSTRAP_LOG"
UNIROBOGUI_ROOT="$fixture" KOKORO_WHEELHOUSE="$fixture/wheels" PYTHON="$TMP/python" \
  bash "$ROOT/scripts/install_kokoro_tts.sh" --product go2 >"$TMP/broken-pip.out" 2>&1 || {
  cat "$TMP/broken-pip.out" >&2; fail 'broken project pip did not recover';
}
grep -q -- '-m venv --without-pip' "$BOOTSTRAP_LOG" || fail 'broken pip skipped bootstrap'
[[ ! -f "$fixture/tts_kokoro/bin/broken-pip" ]] || fail 'broken pip module not repaired'

# An interrupted extraction that left model.onnx must recover from the archive.
tar -cjf "$fixture/tts_models/kokoro-multi-lang-v1_1.tar.bz2" \
  -C "$fixture/tts_models" kokoro-multi-lang-v1_1
rm "$fixture/tts_models/kokoro-multi-lang-v1_1/voices.bin"
UNIROBOGUI_ROOT="$fixture" KOKORO_WHEELHOUSE="$fixture/wheels" PYTHON="$TMP/python" \
  bash "$ROOT/scripts/install_kokoro_tts.sh" --product go2 >"$TMP/recovery.out" 2>&1 || {
  cat "$TMP/recovery.out" >&2; fail 'partial model did not recover';
}
test -f "$fixture/tts_models/kokoro-multi-lang-v1_1/voices.bin" || fail 'missing voice asset was not restored'

printf '%s\n' 'deployment bootstrap regression passed'
