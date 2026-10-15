#!/usr/bin/env bash

load_product_profile() {
  local product="${1:-}"

  case "$product" in
    g1)
      PRODUCT_ID=g1
      PRODUCT_NAME="Unitree G1"
      DEFAULT_HOST="unitree@192.168.123.164"
      DDS_INTERFACE=eth0
      WEB_SERVICE="g1-web-control.service"
      OTHER_WEB_SERVICE="r1-web-control.service go2-web-control.service"
      WEB_SYSTEM_UNIT="deploy/g1-web-control.service"
      WEB_USER_UNIT="deploy/g1-web-control.user.service"
      SYSTEM_SDK_PREFIX="/opt/unitree_robotics"
      USER_SDK_PREFIX="/home/unitree/.local/unitree_robotics"
      SDK_POLICY=install_if_missing
      NEEDS_REALSENSE=true
      NEEDS_KOKORO=true
      CAMERA_HELPER="g1-web-first-person-service"
      CAMERA_SUDOERS="g1-web-camera.sudoers"
      ;;
    r1)
      PRODUCT_ID=r1
      PRODUCT_NAME="Unitree R1"
      DEFAULT_HOST="unitree@192.168.123.164"
      DDS_INTERFACE=eth10
      WEB_SERVICE="r1-web-control.service"
      OTHER_WEB_SERVICE="g1-web-control.service go2-web-control.service"
      WEB_SYSTEM_UNIT="deploy/r1-web-control.service"
      WEB_USER_UNIT="deploy/r1-web-control.user.service"
      SYSTEM_SDK_PREFIX="/usr/local"
      USER_SDK_PREFIX="/usr/local"
      SDK_POLICY=verify_existing
      NEEDS_REALSENSE=false
      NEEDS_KOKORO=false
      CAMERA_HELPER="r1-web-camera-service"
      CAMERA_SUDOERS="r1-web-camera.sudoers"
      ;;
    go2)
      PRODUCT_ID=go2
      PRODUCT_NAME="Unitree GO2"
      DEFAULT_HOST="unirobo-go2"
      DDS_INTERFACE=eth0
      WEB_SERVICE="go2-web-control.service"
      OTHER_WEB_SERVICE="g1-web-control.service r1-web-control.service"
      WEB_SYSTEM_UNIT="deploy/go2-web-control.service"
      WEB_USER_UNIT="deploy/go2-web-control.user.service"
      SYSTEM_SDK_PREFIX="/usr/local"
      USER_SDK_PREFIX="/usr/local"
      SDK_POLICY=verify_existing
      NEEDS_REALSENSE=false
      NEEDS_KOKORO=true
      CAMERA_HELPER=""
      CAMERA_SUDOERS=""
      ;;
    *)
      printf 'Unsupported product: %s (use g1, r1 or go2)\n' "${product:-<missing>}" >&2
      return 2
      ;;
  esac

  TTS_MODEL_NAME=kokoro-int8-multi-lang-v1_1
  TTS_MODEL_FILE=model.int8.onnx
  TTS_SERVICE=g1-local-tts.service
  TTS_SYSTEM_UNIT=deploy/g1-local-tts.service
  TTS_USER_UNIT=deploy/g1-local-tts.user.service
  if [[ "$PRODUCT_ID" == go2 ]]; then
    TTS_MODEL_NAME=kokoro-multi-lang-v1_1
    TTS_MODEL_FILE=model.onnx
    TTS_SERVICE=go2-local-tts.service
    TTS_SYSTEM_UNIT=deploy/go2-local-tts.service
    TTS_USER_UNIT=deploy/go2-local-tts.user.service
  fi
  export TTS_MODEL_NAME TTS_MODEL_FILE TTS_SERVICE TTS_SYSTEM_UNIT TTS_USER_UNIT

  export PRODUCT_ID PRODUCT_NAME DEFAULT_HOST DDS_INTERFACE WEB_SERVICE OTHER_WEB_SERVICE
  export WEB_SYSTEM_UNIT WEB_USER_UNIT SYSTEM_SDK_PREFIX USER_SDK_PREFIX
  export SDK_POLICY NEEDS_REALSENSE NEEDS_KOKORO CAMERA_HELPER CAMERA_SUDOERS
}
