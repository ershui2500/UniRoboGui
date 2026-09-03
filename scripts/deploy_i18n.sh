#!/usr/bin/env bash

case "${UNIROBOGUI_LANG:-zh}" in
  zh|zh-CN|zh_CN|cn) UNIROBOGUI_LANG=zh ;;
  en|en-US|en_US|en-GB|en_GB) UNIROBOGUI_LANG=en ;;
  *)
    printf '[ERROR] Unsupported UNIROBOGUI_LANG: %s (use zh or en)\n' "${UNIROBOGUI_LANG}" >&2
    exit 2
    ;;
esac
export UNIROBOGUI_LANG

ui_text() {
  if [[ "${UNIROBOGUI_LANG}" == "en" ]]; then
    printf '%s' "$2"
  else
    printf '%s' "$1"
  fi
}

ui_line() {
  if [[ "${UNIROBOGUI_LANG}" == "en" ]]; then
    printf '%s\n' "$2"
  else
    printf '%s\n' "$1"
  fi
}

ui_err() {
  if [[ "${UNIROBOGUI_LANG}" == "en" ]]; then
    printf '%s\n' "$2" >&2
  else
    printf '%s\n' "$1" >&2
  fi
}
