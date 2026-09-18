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
      OTHER_WEB_SERVICE="r1-web-control.service"
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
      OTHER_WEB_SERVICE="g1-web-control.service"
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
    *)
      printf 'Unsupported product: %s (use g1 or r1)\n' "${product:-<missing>}" >&2
      return 2
      ;;
  esac

  export PRODUCT_ID PRODUCT_NAME DEFAULT_HOST DDS_INTERFACE WEB_SERVICE OTHER_WEB_SERVICE
  export WEB_SYSTEM_UNIT WEB_USER_UNIT SYSTEM_SDK_PREFIX USER_SDK_PREFIX
  export SDK_POLICY NEEDS_REALSENSE NEEDS_KOKORO CAMERA_HELPER CAMERA_SUDOERS
}
