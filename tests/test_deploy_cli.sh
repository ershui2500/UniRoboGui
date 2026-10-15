#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

fail() {
  printf 'deploy CLI test failed: %s\n' "$*" >&2
  exit 1
}

expect_rc() {
  local expected="$1"
  shift
  set +e
  "$@" >"$TMP/stdout" 2>"$TMP/stderr"
  local rc=$?
  set -e
  [[ "$rc" -eq "$expected" ]] ||
    fail "expected rc=$expected, got rc=$rc: $*"
}

source "$ROOT/scripts/deploy_product.sh"
load_product_profile g1
[[ "$DDS_INTERFACE" == eth0 && "$WEB_SERVICE" == g1-web-control.service &&
   "$SYSTEM_SDK_PREFIX" == /opt/unitree_robotics && "$NEEDS_KOKORO" == true &&
   "$DEFAULT_HOST" == unitree@192.168.123.164 ]]
load_product_profile r1
[[ "$DDS_INTERFACE" == eth10 && "$WEB_SERVICE" == r1-web-control.service &&
   "$SYSTEM_SDK_PREFIX" == /usr/local && "$NEEDS_KOKORO" == false &&
   "$DEFAULT_HOST" == unitree@192.168.123.164 ]]
load_product_profile go2
[[ "$DDS_INTERFACE" == eth0 && "$WEB_SERVICE" == go2-web-control.service &&
   "$SYSTEM_SDK_PREFIX" == /usr/local && "$SDK_POLICY" == verify_existing &&
   "$NEEDS_KOKORO" == true && "$NEEDS_REALSENSE" == false &&
   "$DEFAULT_HOST" == unirobo-go2 && -z "$CAMERA_HELPER" &&
   "$TTS_MODEL_NAME" == kokoro-multi-lang-v1_1 && "$TTS_MODEL_FILE" == model.onnx &&
   "$TTS_SERVICE" == go2-local-tts.service ]]
for profile in g1 r1 go2; do
  load_product_profile "$profile"
  for other in g1 r1 go2; do
    [[ "$other" == "$profile" ]] || [[ " $OTHER_WEB_SERVICE " == *" $other-web-control.service "* ]]
  done
done
load_product_profile g1
[[ "$TTS_MODEL_FILE" == model.int8.onnx && "$TTS_SERVICE" == g1-local-tts.service ]]
load_product_profile r1
[[ "$NEEDS_KOKORO" == false ]]
if load_product_profile unknown >/dev/null 2>&1; then
  fail "unknown product was accepted"
fi

mkdir -p "$TMP/repo/scripts"
cp "$ROOT/scripts/deploy.sh" "$ROOT/scripts/deploy_i18n.sh" \
  "$ROOT/scripts/deploy_product.sh" "$TMP/repo/scripts/"
for name in deploy_online.sh deploy_from_pc.sh install.sh; do
  cat >"$TMP/repo/scripts/$name" <<'EOF'
#!/usr/bin/env bash
printf '%s lang=%s\n' "$0 $*" "${UNIROBOGUI_LANG:-}" >>"${DEPLOY_TEST_LOG:?}"
EOF
done
chmod +x "$TMP/repo/scripts/"*.sh
export DEPLOY_TEST_LOG="$TMP/dispatch.log"

bash "$TMP/repo/scripts/deploy.sh" --help >"$TMP/help"
grep -q -- '--product g1|r1|go2' "$TMP/help" || fail "help does not document supported products"
grep -q -- '--host USER@HOST' "$TMP/help" || fail "help does not document --host"
grep -q -- '--lang zh|en' "$TMP/help" || fail "help does not document --lang"
grep -q 'install' "$TMP/help" && grep -q 'from-pc' "$TMP/help" && grep -q 'check' "$TMP/help" ||
  fail "help does not document all deployment commands"

expect_rc 2 bash "$TMP/repo/scripts/deploy.sh" install
grep -q -- '--product g1|r1|go2 is required' "$TMP/stderr" ||
  fail "missing product was not rejected"
expect_rc 2 bash "$TMP/repo/scripts/deploy.sh" install --product unknown
grep -q 'Unsupported product' "$TMP/stderr" ||
  fail "unknown product was not rejected"
expect_rc 2 bash "$TMP/repo/scripts/deploy.sh" install --product g1 --host robot
grep -q -- '--host' "$TMP/stderr" || fail "--host leaked into install"
expect_rc 2 bash "$TMP/repo/scripts/deploy.sh" check --product g1 --lang de
grep -q 'Unsupported UNIROBOGUI_LANG' "$TMP/stderr" ||
  fail "unsupported language was not rejected"

: >"$DEPLOY_TEST_LOG"
bash "$TMP/repo/scripts/deploy.sh" install --product g1 --lang en
bash "$TMP/repo/scripts/deploy.sh" check --product r1
bash "$TMP/repo/scripts/deploy.sh" from-pc --product r1
grep -q 'deploy_online.sh --product g1' "$DEPLOY_TEST_LOG"
grep -q 'deploy_online.sh --product g1 lang=en' "$DEPLOY_TEST_LOG" ||
  fail "--lang en was not forwarded through the environment"
grep -q 'install.sh --product r1 --check-only' "$DEPLOY_TEST_LOG"
grep -q 'deploy_from_pc.sh --product r1 --host unitree@192.168.123.164' "$DEPLOY_TEST_LOG"
bash "$TMP/repo/scripts/deploy.sh" install --product go2
bash "$TMP/repo/scripts/deploy.sh" check --product go2
bash "$TMP/repo/scripts/deploy.sh" from-pc --product go2
grep -q 'deploy_online.sh --product go2' "$DEPLOY_TEST_LOG"
grep -q 'install.sh --product go2 --check-only' "$DEPLOY_TEST_LOG"
grep -q 'deploy_from_pc.sh --product go2 --host unirobo-go2' "$DEPLOY_TEST_LOG"

# Execute the real resource installer with a fake venv; never download or run models.
export INSTALL_TEST_LOG="$TMP/tts-install.log"
for profile in g1 go2; do
  load_product_profile "$profile"
  fixture="$TMP/tts-$profile"
  mkdir -p "$fixture/tts_kokoro/bin" "$fixture/archive/$TTS_MODEL_NAME" "$fixture/tts_models"
  for name in "$TTS_MODEL_FILE" voices.bin tokens.txt lexicon-us-en.txt lexicon-zh.txt phone-zh.fst date-zh.fst number-zh.fst; do
    touch "$fixture/archive/$TTS_MODEL_NAME/$name"
  done
  tar -cjf "$fixture/tts_models/$TTS_MODEL_NAME.tar.bz2" -C "$fixture/archive" "$TTS_MODEL_NAME"
  cat >"$fixture/tts_kokoro/bin/python" <<'EOF'
#!/usr/bin/env bash
printf '%s|%s\n' "${KOKORO_MODEL_DIR:-}" "${KOKORO_MODEL_FILE:-}" >>"${INSTALL_TEST_LOG:?}"
cat >/dev/null
EOF
  chmod +x "$fixture/tts_kokoro/bin/python"
  touch "$fixture/tts_kokoro/bin/pip"
  chmod +x "$fixture/tts_kokoro/bin/pip"
  UNIROBOGUI_ROOT="$fixture" bash "$ROOT/scripts/install_kokoro_tts.sh" --product "$profile" >"$TMP/tts-out" 2>&1
  test -f "$fixture/tts_models/$TTS_MODEL_NAME/$TTS_MODEL_FILE"
  grep -Fq "$TTS_MODEL_NAME|$TTS_MODEL_FILE" "$INSTALL_TEST_LOG"
  if [[ "$profile" == go2 ]]; then
    touch "$fixture/tts_models/$TTS_MODEL_NAME/model.int8.onnx"
    expect_rc 1 env UNIROBOGUI_ROOT="$fixture" bash "$ROOT/scripts/install_kokoro_tts.sh" --product go2
    grep -q 'INT8' "$TMP/stderr" || fail "GO2 mixed precision directory was accepted"
  fi
done
expect_rc 2 bash "$ROOT/scripts/install_kokoro_tts.sh" --product r1
for unit in go2-web-control.service go2-web-control.user.service; do
  grep -q -- '--robot go2' "$ROOT/deploy/$unit"
  grep -q -- '--interface eth0' "$ROOT/deploy/$unit"
  ! grep -Eq -- '--enable-navigation|--device[[:space:]]+mid360' "$ROOT/deploy/$unit"
done
for unit in go2-local-tts.service go2-local-tts.user.service; do
  grep -q -- 'kokoro-multi-lang-v1_1 --sid 3 --speed 1.2 --threads 8' "$ROOT/deploy/$unit"
  ! grep -q 'kokoro-int8' "$ROOT/deploy/$unit"
done

if grep -q 'BatchMode=yes' "$ROOT/scripts/deploy_from_pc.sh"; then
  fail "from-pc must allow first-time interactive SSH authentication"
fi
if grep -q 'test -r /home/unitree/UniRoboGui/CMakeLists.txt || test ! -e /home/unitree/UniRoboGui' \
    "$ROOT/scripts/deploy_from_pc.sh"; then
  fail "remote preflight rejects resumable partial project directories"
fi
grep -q -- "--exclude '/GO2参考资料/'" "$ROOT/scripts/deploy_from_pc.sh" || fail "GO2 reference mirror must not ship"
grep -q -- "--exclude '/config/'" "$ROOT/scripts/deploy_from_pc.sh" ||
  fail "from-pc must preserve the robot runtime config directory"

for preserved in tts_models tts_kokoro; do
  grep -Fq -- "--exclude '/$preserved/'" "$ROOT/scripts/deploy_from_pc.sh" ||
    fail "from-pc must preserve robot $preserved"
done

legacy="$TMP/repo/scripts/deploy_g1_from_pc.sh"
cp "$ROOT/scripts/deploy_g1_from_pc.sh" "$legacy"
: >"$DEPLOY_TEST_LOG"
bash "$legacy" --robot unitree@example
grep -q 'deploy_from_pc.sh --product g1 --host unitree@example' "$DEPLOY_TEST_LOG" ||
  fail "legacy --robot was not translated to --host"

cp "$ROOT/scripts/deploy_g1_online.sh" "$ROOT/scripts/install_g1.sh" "$TMP/repo/scripts/"
: >"$DEPLOY_TEST_LOG"
bash "$TMP/repo/scripts/deploy_g1_online.sh"
bash "$TMP/repo/scripts/install_g1.sh" --check-only
grep -q 'deploy_online.sh --product g1' "$DEPLOY_TEST_LOG" ||
  fail "legacy online entry did not forward to g1"
grep -q 'install.sh --product g1 --check-only' "$DEPLOY_TEST_LOG" ||
  fail "legacy install entry did not forward to g1"

cp "$ROOT/scripts/deploy_g1_from_pc.en.sh" "$ROOT/scripts/deploy_g1_online.en.sh" \
  "$ROOT/scripts/install_g1.en.sh" "$TMP/repo/scripts/"
: >"$DEPLOY_TEST_LOG"
bash "$TMP/repo/scripts/deploy_g1_from_pc.en.sh" --robot unitree@example
bash "$TMP/repo/scripts/deploy_g1_online.en.sh"
bash "$TMP/repo/scripts/install_g1.en.sh" --check-only
grep -q 'deploy_from_pc.sh --product g1 --host unitree@example' "$DEPLOY_TEST_LOG" ||
  fail "legacy English from-pc entry did not forward to g1"
grep -q 'deploy_online.sh --product g1' "$DEPLOY_TEST_LOG" ||
  fail "legacy English online entry did not forward to g1"
grep -q 'install.sh --product g1 --check-only' "$DEPLOY_TEST_LOG" ||
  fail "legacy English install entry did not forward to g1"

grep -q -- '--robot g1' "$ROOT/deploy/g1-web-control.service"
grep -q -- '--interface eth0' "$ROOT/deploy/g1-web-control.service"
grep -q -- '--enable-navigation' "$ROOT/deploy/g1-web-control.service"
grep -q -- '--robot g1' "$ROOT/deploy/g1-web-control.user.service"
grep -q -- '--interface eth0' "$ROOT/deploy/g1-web-control.user.service"
grep -q -- '--robot r1' "$ROOT/deploy/r1-web-control.service"
grep -q -- '--interface eth10' "$ROOT/deploy/r1-web-control.service"
grep -q -- '--robot r1' "$ROOT/deploy/r1-web-control.user.service"
grep -q -- '--interface eth10' "$ROOT/deploy/r1-web-control.user.service"
if grep -Eq -- '--enable-navigation|--device[[:space:]]+mid360' "$ROOT/deploy/r1-web-control.service"; then
  fail "R1 service enables navigation or Mid-360 by default"
fi
if grep -Eq -- '--enable-navigation|--device[[:space:]]+mid360' "$ROOT/deploy/r1-web-control.user.service"; then
  fail "R1 user service enables navigation or Mid-360 by default"
fi

for script in \
  scripts/deploy.sh scripts/deploy_product.sh scripts/deploy_online.sh \
  scripts/deploy_from_pc.sh scripts/install.sh scripts/deploy_g1_online.sh \
  scripts/deploy_g1_from_pc.sh scripts/install_g1.sh scripts/install_kokoro_tts.sh; do
  bash -n "$ROOT/$script"
done

printf '%s\n' 'deploy CLI regression passed'
