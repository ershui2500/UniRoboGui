#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
source "${script_dir}/deploy_i18n.sh"

ROOT="${UNITREE_INTERFACE_ROOT:-/home/unitree/unitree_interface}"
PYTHON="${PYTHON:-python3}"
VENV="${KOKORO_VENV:-${ROOT}/tts_kokoro}"
MODEL_ROOT="${KOKORO_MODEL_ROOT:-${ROOT}/tts_models}"
MODEL_NAME="kokoro-int8-multi-lang-v1_1"
MODEL_DIR="${MODEL_ROOT}/${MODEL_NAME}"
SHERPA_VERSION="${SHERPA_ONNX_VERSION:-1.13.4}"
MODEL_URL="https://github.com/k2-fsa/sherpa-onnx/releases/download/tts-models/${MODEL_NAME}.tar.bz2"
ARCHIVE="${MODEL_ROOT}/${MODEL_NAME}.tar.bz2"
WHEELHOUSE="${KOKORO_WHEELHOUSE:-}"

fail() {
  ui_err "[错误] $1" "[ERROR] ${2:-$1}"
  exit 1
}

pip_install() {
  local args=()
  if [[ -n "${WHEELHOUSE}" ]]; then
    [[ -d "${WHEELHOUSE}" ]] || fail "KOKORO_WHEELHOUSE 不是目录：${WHEELHOUSE}" "KOKORO_WHEELHOUSE is not a directory: ${WHEELHOUSE}"
    args+=(--no-index --find-links "${WHEELHOUSE}")
  fi
  "${VENV}/bin/python" -m pip install "${args[@]}" "$@" || {
    if [[ "$UNIROBOGUI_LANG" == "en" ]]; then
      cat >&2 <<EOF
[ERROR] Kokoro Python dependency installation failed. Verify that wlan0 is connected and PyPI is reachable.
If the robot cannot access PyPI, provide a wheelhouse for G1 aarch64/Python 3.8 and run:
  KOKORO_WHEELHOUSE=/uploaded/wheelhouse bash scripts/install_kokoro_tts.sh
EOF
    else
      cat >&2 <<EOF
[提示] Kokoro Python 依赖安装失败。请先确认 wlan0 已连接且 PyPI 可访问。
如果机器人不能访问 PyPI，请让供应方提供 G1 aarch64/Python 3.8 的 wheelhouse，上传后运行：
  KOKORO_WHEELHOUSE=/上传目录 bash scripts/install_kokoro_tts.sh
EOF
    fi
    exit 1
  }
}

if [[ "$(uname -m)" != "aarch64" ]]; then
  ui_err "[警告] 此安装器针对 G1 Jetson/aarch64 调优；当前架构为 $(uname -m)，继续执行。" "[WARN] This installer is tuned for the G1 Jetson/aarch64 target; continuing on $(uname -m)."
fi

"${PYTHON}" - <<'PY'
import os
import sys
if sys.version_info < (3, 8):
    if os.environ.get("UNIROBOGUI_LANG") == "en":
        raise SystemExit("Python >= 3.8 is required")
    raise SystemExit("需要 Python >= 3.8")
print("Python", sys.version.split()[0])
PY

if [[ ! -x "${VENV}/bin/pip" ]]; then
  # G1 images may not contain Debian's python3-venv/ensurepip package. Keep
  # everything under the unitree user's home instead of changing APT state.
  virtualenv_args=()
  if [[ -n "${WHEELHOUSE}" ]]; then
    [[ -d "${WHEELHOUSE}" ]] || fail "KOKORO_WHEELHOUSE 不是目录：${WHEELHOUSE}" "KOKORO_WHEELHOUSE is not a directory: ${WHEELHOUSE}"
    virtualenv_args+=(--no-index --find-links "${WHEELHOUSE}")
    ui_line "正在从已传入的 wheelhouse 安装 virtualenv：${WHEELHOUSE}" "Installing virtualenv from transferred wheelhouse: ${WHEELHOUSE}"
  fi
  "${PYTHON}" -m pip install --user "${virtualenv_args[@]}" 'virtualenv==20.26.6' || {
    if [[ -n "${WHEELHOUSE}" ]]; then
      fail "无法从已传入的 wheelhouse 安装 virtualenv：${WHEELHOUSE}" "Cannot install virtualenv from transferred wheelhouse: ${WHEELHOUSE}"
    fi
    fail "无法安装 virtualenv。请让 wlan0 联网，或提供 KOKORO_WHEELHOUSE。" "Cannot install virtualenv. Connect wlan0 to the Internet or provide KOKORO_WHEELHOUSE."
  }
  "${PYTHON}" -m virtualenv --clear "${VENV}"
fi

if "${VENV}/bin/python" - "${SHERPA_VERSION}" <<'PY'
import importlib.metadata
import sys
import numpy
import sherpa_onnx
raise SystemExit(importlib.metadata.version("sherpa-onnx") != sys.argv[1])
PY
then
  ui_line "Kokoro Python 依赖已安装；跳过 PyPI。" "Kokoro Python dependencies are already installed; skipping PyPI."
else
  pip_install --upgrade 'pip<25' 'setuptools<76' wheel
  pip_install 'numpy<2' "sherpa-onnx==${SHERPA_VERSION}"
fi

mkdir -p "${MODEL_ROOT}"
if [[ ! -f "${MODEL_DIR}/model.int8.onnx" ]]; then
  if [[ ! -s "${ARCHIVE}" ]]; then
    curl --noproxy '*' -fL --retry 2 --retry-delay 2 --connect-timeout 8 \
      --continue-at - -o "${ARCHIVE}" "${MODEL_URL}" || {
      if [[ "$UNIROBOGUI_LANG" == "en" ]]; then
        cat >&2 <<EOF
[ERROR] Kokoro model download failed. Wi-Fi may be connected while GitHub is still unreachable.
Download ${MODEL_NAME}.tar.bz2 on another computer and upload it to:
  ${ARCHIVE}
Then rerun this installer. Use --without-kokoro in install_g1.sh only when local TTS is not required.
EOF
      else
        cat >&2 <<EOF
[错误] Kokoro 模型下载失败。Wi-Fi 可能已经连接，但 GitHub 仍不可访问。
请在另一台电脑下载 ${MODEL_NAME}.tar.bz2，并上传到：
  ${ARCHIVE}
然后重新运行安装器。只有不需要本地 TTS 时才在 install_g1.sh 中使用 --without-kokoro。
EOF
      fi
      exit 1
    }
  else
    ui_line "使用已传入的 Kokoro 模型压缩包：${ARCHIVE}" "Using transferred Kokoro model archive: ${ARCHIVE}"
  fi
  tar -tjf "${ARCHIVE}" >/dev/null 2>&1 || fail "无效的 Kokoro 模型压缩包：${ARCHIVE}" "Invalid Kokoro model archive: ${ARCHIVE}"
  tar -xjf "${ARCHIVE}" -C "${MODEL_ROOT}"
fi

for required in \
  model.int8.onnx voices.bin tokens.txt lexicon-us-en.txt lexicon-zh.txt \
  phone-zh.fst date-zh.fst number-zh.fst; do
  test -f "${MODEL_DIR}/${required}" || {
    ui_err "缺少 Kokoro 资源：${MODEL_DIR}/${required}" "Missing Kokoro asset: ${MODEL_DIR}/${required}"
    exit 1
  }
done

KOKORO_MODEL_DIR="${MODEL_DIR}" "${VENV}/bin/python" - <<'PY'
import os
import sherpa_onnx
root = os.environ['KOKORO_MODEL_DIR']
config = sherpa_onnx.OfflineTtsConfig(
    model=sherpa_onnx.OfflineTtsModelConfig(
        kokoro=sherpa_onnx.OfflineTtsKokoroModelConfig(
            model=os.path.join(root, 'model.int8.onnx'),
            voices=os.path.join(root, 'voices.bin'),
            tokens=os.path.join(root, 'tokens.txt'),
            data_dir=os.path.join(root, 'espeak-ng-data'),
            lexicon=','.join([
                os.path.join(root, 'lexicon-us-en.txt'),
                os.path.join(root, 'lexicon-zh.txt'),
            ]),
        ),
        provider='cpu',
        num_threads=2,
        debug=False,
    ),
    rule_fsts=','.join([
        os.path.join(root, 'phone-zh.fst'),
        os.path.join(root, 'date-zh.fst'),
        os.path.join(root, 'number-zh.fst'),
    ]),
    max_num_sentences=1,
)
if not config.validate():
    if os.environ.get('UNIROBOGUI_LANG') == 'en':
        raise SystemExit('Kokoro sherpa-onnx config validation failed')
    raise SystemExit('Kokoro sherpa-onnx 配置验证失败')
sherpa_onnx.OfflineTts(config)
if os.environ.get('UNIROBOGUI_LANG') == 'en':
    print('Kokoro zh-en INT8 model ready with sherpa-onnx')
else:
    print('Kokoro 中英双语 INT8 模型已通过 sherpa-onnx 验证')
PY

if [[ "$UNIROBOGUI_LANG" == "en" ]]; then
  cat <<EOF
Kokoro local TTS installed:
  venv:  ${VENV}
  model: ${MODEL_DIR}
  runtime: sherpa-onnx ${SHERPA_VERSION}
EOF
else
  cat <<EOF
Kokoro 本地 TTS 已安装：
  虚拟环境：${VENV}
  模型：    ${MODEL_DIR}
  运行时：  sherpa-onnx ${SHERPA_VERSION}
EOF
fi
