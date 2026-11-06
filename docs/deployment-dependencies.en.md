# UniRoboGui G1 / R1 / GO2 / A2 Deployment and Troubleshooting Guide

[简体中文](deployment-dependencies.md) | **English**

This guide covers Unitree G1, R1, GO2 and A2. They share one deployment entry point, build flow, and safety checks. Product-specific differences such as network interfaces, SDK paths, services, cameras, TTS, and optional attachments are kept in controlled product profiles instead of duplicated deployment guides.

For A2 installation, default SSH address, offline architecture and JT128/navigation defaults, start with [Quick deployment in README](../README.en.md#quick-deployment). A2 uses Ubuntu 22.04 amd64 / Python 3.10 wheels; do not use arm64 / Python 3.8 packages from other products.

## 1. Recommended entry point

First choose the actual product and one deployment method; the commands below are alternatives, not a sequence to paste in full. Run `install`/`check` in the robot terminal and `from-pc` in the online PC terminal, always from the cloned project root.

Before cloning on Ubuntu, make sure Git is installed. The Method B PC needs `git curl openssh-client rsync python3 python3-pip`; check with `command -v git curl ssh rsync python3` and `python3 -m pip --version`. Install missing PC packages first. Robot-side missing packages must be Ubuntu 20.04 arm64 (G1/R1/GO2) or Ubuntu 22.04 amd64 (A2) packages, matching the target OS and architecture. `from-pc` does not package the complete Ubuntu repository.


~~~bash
# Robot terminal: install/upgrade; choose only your product
bash scripts/deploy.sh install --product g1 --lang en
bash scripts/deploy.sh install --product r1 --lang en
bash scripts/deploy.sh install --product go2 --lang en
bash scripts/deploy.sh install --product a2 --lang en

# Online PC terminal: deploy; choose only your product
bash scripts/deploy.sh from-pc --product g1 --lang en
bash scripts/deploy.sh from-pc --product r1 --lang en
bash scripts/deploy.sh from-pc --product go2 --lang en
bash scripts/deploy.sh from-pc --product a2 --lang en

# Robot terminal: read-only preflight; choose only your product
bash scripts/deploy.sh check --product g1 --lang en
bash scripts/deploy.sh check --product r1 --lang en
bash scripts/deploy.sh check --product go2 --lang en
bash scripts/deploy.sh check --product a2 --lang en
~~~

`--product` is required and currently accepts only `g1`, `r1`, `go2`, or `a2`, as registered by the project. `--host` is optional for all four products and valid only with `from-pc`; defaults are listed below. Override it only when the actual address differs, or use an existing SSH Host alias. Legacy `deploy_g1_*` and `install_g1*` entry scripts remain for one compatibility cycle; they print a deprecation notice and forward to `--product g1`. The legacy `--robot USER@HOST` option is translated to `--host`.

## 2. Product matrix

These are repository deployment defaults; verify interfaces, attachments and availability on the actual robot.

| Item | G1 | R1 | GO2 | A2 |
| --- | --- | --- | --- | --- |
| Default SSH target | `unitree@192.168.123.164` | `unitree@192.168.123.164` | `unitree@192.168.123.18` | `unitree@192.168.123.162` (user development unit) |
| DDS interface | `eth0` | `eth10` | `eth0` (verify on site) | `eth0` (verify on site) |
| SDK / CMake prefix | `/opt/unitree_robotics`; user-local installs may use `/home/unitree/.local/unitree_robotics` | `/usr/local`; validate the existing installation and never overwrite it automatically | `/usr/local`; validate existing compatible SDK, never overwrite | Existing `/usr/local`; validate only |
| Runtime library path | SDK `prefix/lib` + `/usr/local/lib`; reuse existing librealsense when needed | `/usr/local/lib` | `/usr/local/lib` | `/usr/local/lib` |
| Web service | `g1-web-control.service` | `r1-web-control.service` | `go2-web-control.service` | `a2-web-control.service` |
| systemd user unit | `deploy/g1-web-control.user.service` | `deploy/r1-web-control.user.service` | `deploy/go2-web-control.user.service` | `deploy/a2-web-control.user.service` |
| Camera | D435i + librealsense + `g1-web-first-person-service` | Fixed R1 EDU RGB/Depth sources + `r1-web-camera-service` | Fixed native RGB; depth unsupported; no privileged helper | Fixed native RGB; no depth or privileged helper |
| Default TTS | Kokoro + Unitree fallback | Unitree native TTS | Kokoro FP32; no native fallback | Unitree native + optional Kokoro FP32 |
| SDK installation policy | Install from the official SDK2 source when missing | Validate the existing R1 headers, CMake package, and CycloneDDS libraries; stop if they are missing | `/usr/local`; validate existing compatible SDK, never overwrite | Validate compatible A2 SDK; stop if missing, never overwrite |
| Mid-360 / navigation | Keep the existing G1 configuration and service defaults | Do not declare Mid-360 or enable real navigation by default | No optional LiDAR declaration or real navigation by default | JT128/navigation enabled; requires matching attachment and external service |
| Binary | `build/g1_web_server --robot g1` | `build/g1_web_server --robot r1` | `build/g1_web_server --robot go2` | `build/g1_web_server --robot a2` |

All four products use the same source tree and the same historical `g1_web_server` binary name. Runtime product identity comes from `--robot`, `RobotRegistry`, the Manifest, and Capability state—not from the binary filename.

The shared binary compiles all registered products. Even for GO2, the existing SDK must include the R1 Audio/Loco headers required by this project. If missing, have a maintainer provide a complete firmware/ABI-compatible SDK rather than removing product code.

## 3. Shared deployment flow

Products using local Kokoro also check Python initialization before compilation. They reuse a working project venv, a pip wheel in the offline wheelhouse, or system ensurepip. If all three are missing, a system installation conditionally prepares `python3-venv`; a user installation stops early and asks for `KOKORO_WHEELHOUSE` containing a pip wheel or administrator installation of `python3-venv`. Thus `python3-venv` is not a mandatory system dependency when an offline pip wheel is available.

The deployment scripts follow the same sequence for all four products:

1. Validate G1/R1/GO2: Ubuntu 20.04 AArch64; A2: Ubuntu 22.04 x86_64, the `unitree` user, the selected DDS interface, and the absence of ROS/RMW/CycloneDDS environment contamination.
2. Check the source tree for local modifications. Stop instead of overwriting uncommitted project code.
3. Validate or prepare SDK, camera, and TTS dependencies according to the selected product profile.
4. Perform a Release build and run the complete CTest suite.
5. Use `ldd` to verify there are no `not found` entries and that CycloneDDS resolves from the selected product's SDK prefix.
6. Before replacing a running Web service, read the Manifest and `/api/control/status`.
7. Stop or replace the service only when the product identity matches, `motion.active=false`, `state=stopped`, and `vx/vy/vyaw` are all zero.
8. If the other product's service is active, stop the deployment instead of shutting it down or switching products automatically.
9. Install the exact system/user units and only the camera helper/sudoers files declared by the product; GO2 has no privileged helper.
10. After startup, perform read-only acceptance checks: a single listener on port 8080, health, Manifest, required DDS sources, zero-motion state, MainPID arguments, process environment, and helper status.

Automated acceptance never calls real `SetVelocity`, `StopMove`, `Start`, `StandUp`, `SetFsmId`, joint, head, mode-switching, or navigation commands. Real motion acceptance is performed manually by an on-site operator through the Web interface.

## 4. Product deployment notes

### G1 deployment notes

G1 keeps the existing baseline:

- `eth0` carries SDK2 DDS traffic, while `wlan0` is used for Web access and Internet connectivity.
- The system SDK prefix is `/opt/unitree_robotics`; a persistent user-local prefix may be used when required.
- D435i support requires librealsense. System-level installation provides the controlled `g1-web-first-person-service` helper.
- Kokoro is the default local TTS backend, with Unitree TTS retained as a fallback.
- `g1-web-control.service` explicitly includes `--robot g1` and preserves the existing `--enable-navigation` default.
- `from-pc` prepares SDK2, librealsense, the Kokoro model, and an AArch64/Python 3.8 wheelhouse.

### R1 deployment notes

R1 uses these boundaries:

- The DDS interface is fixed to `eth10`.
- The SDK/CMake prefix is fixed to `/usr/local`.
- The deployer validates only the R1 Audio/Loco headers required by this project, the `unitree_sdk2` CMake package, and the `libddsc` / `libddscxx` runtime libraries. If any are missing, deployment stops instead of overwriting `/usr/local`.
- Unitree native TTS is the default; Kokoro is not installed.
- System-level installation uses `r1-web-camera-service`. If a user-level deployment has no helper, the related Camera Capability is downgraded according to runtime evidence.
- `r1-web-control.service` explicitly includes `--robot r1` and does not include `--device mid360` or `--enable-navigation`.
- `from-pc` defaults to `unitree@192.168.123.164`. OpenSSH first tries existing keys/agents, allows host-key confirmation on first connection, and prompts for a password only when needed. The scripts do not store or echo passwords. Use `--host` to override the address or use an existing SSH Host alias.

### GO2 deployment notes

- Confirm the development-capable model, reliable clock and actual wired DDS interface using the [GO2 guide](go2-guide.en.md#3-deployment). The repository defaults to `eth0`; there is no `--interface` deploy option.
- Reuse the existing compatible `/usr/local` SDK; missing Go2 headers/CMake/DDS libraries stop deployment.
- Install `go2-web-control.service` and `go2-local-tts.service` in the selected system/user scope. The default model is FP32 `kokoro-multi-lang-v1_1/model.onnx`, not the G1 INT8 model.
- Do not install RealSense/camera helpers or enable optional LiDAR/navigation by default. Depth and ASR/native TTS/built-in LLM are unavailable.
- The PC flow transfers the model and AArch64/Python 3.8 wheelhouse, but does not bundle every Ubuntu system package. Missing arm64 system packages still require a compatible package source.

## 5. `from-pc` transfer boundaries

When `from-pc` transfers the project with rsync:

- `.git` is not transferred.
- `AGENTS.md`, `*_AGENTS.md`, `.robot-workspace`, `.robot-backups`, `.robot-sync`, local G1/R1/GO2/A2 reference directories, and internal field records under `docs/GO2-*` and `docs/voice/` are not transferred.
- The robot's complete `config/`, `build/`, `tts_models/` and `tts_kokoro/` directories are preserved; product model archives are transferred separately.
- Deployment stops if the robot-side Git checkout contains uncommitted project changes.
- Project sync does not use `--delete`; extra on-site files are preserved. A maintainer must review obsolete source files before separately removing them. Upstream SDK/RealSense source directories use a separate backup-then-mirror flow.
- SSH, sudo, Wi-Fi, API credentials, and other secrets are never written into project files, command arguments, logs, or caches.

## 6. Read-only preflight

In the robot terminal, run only the command for your product:

~~~bash
bash scripts/deploy.sh check --product g1 --lang en
bash scripts/deploy.sh check --product r1 --lang en
bash scripts/deploy.sh check --product go2 --lang en
bash scripts/deploy.sh check --product a2 --lang en
~~~

`check` does not install packages, modify files, or stop/start services. It validates the operating system, architecture, selected network interface, required packages, SDK state, camera-helper state, and other deployment prerequisites. On R1/GO2/A2, the SDK is validated directly under `/usr/local`. A successful preflight is not runtime or motion acceptance.

## 7. Post-deployment acceptance

The scripts perform the core read-only acceptance checks automatically. For manual verification, run these in the robot terminal; `127.0.0.1` means the robot:

~~~bash
curl --noproxy '*' -fsS http://127.0.0.1:8080/api/health
curl --noproxy '*' -fsS http://127.0.0.1:8080/api/robot/manifest
curl --noproxy '*' -fsS http://127.0.0.1:8080/api/control/status
ss -ltnp 'sport = :8080'
ldd /home/unitree/UniRoboGui/build/g1_web_server | grep -E 'ddsc|ddscxx|realsense|not found'
~~~

Verify that:

- Manifest `identity.product_id` matches the deployed product.
- `motion.active=false`, `state=stopped`, and `vx/vy/vyaw` are all zero.
- DDS is initialized in health and all Manifest-declared `required_sources` are online.
- Only the target Web process is listening on port 8080.
- The MainPID command line contains the correct `--robot` and `--interface` arguments.
- The Web process does not contain `ROS_DISTRO`, `RMW_IMPLEMENTATION`, `CYCLONEDDS_URI`, `AMENT_PREFIX_PATH`, or `COLCON_PREFIX_PATH`.
- G1/R1/GO2/A2 CycloneDDS libraries resolve from the SDK prefix expected for the selected product.
- Read-only camera-helper status checks return valid active/inactive semantics.

## 8. Troubleshooting

### GitHub is unreachable

Do not keep retrying GitHub operations on the robot and do not modify the DDS interface to obtain Internet access. Use an Internet-connected computer instead:

~~~bash
bash scripts/deploy.sh from-pc --product g1 --lang en
bash scripts/deploy.sh from-pc --product r1 --lang en
bash scripts/deploy.sh from-pc --product go2 --lang en
bash scripts/deploy.sh from-pc --product a2 --lang en
~~~

### R1 SDK validation fails

If the required R1 headers, `unitree_sdk2Config.cmake`, `libddsc`, or `libddscxx` are missing under `/usr/local`, deployment stops. The generic deployer intentionally does not reinstall or overwrite `/usr/local`; restore an SDK compatible with the robot firmware/ABI first.

### `undefined symbol: ddsi_sertype_v0`

This usually indicates that another CycloneDDS/ROS environment has been mixed into the process environment. Start a clean SSH session, select `none` at the ROS environment prompt, do not source Foxy, Noetic, ROS 2, or a custom RMW setup, and rerun `check`.

### Port 8080 is occupied

Do not use `pkill` or `killall`. First confirm the exact unit, MainPID, Manifest product identity, and zero-motion state, then inspect the corresponding service:

~~~bash
ss -ltnp 'sport = :8080'
systemctl --no-pager --full status g1-web-control.service
systemctl --no-pager --full status r1-web-control.service
systemctl --no-pager --full status go2-web-control.service
~~~

### Camera

On G1, check librealsense, D435i USB enumeration, and `g1-web-first-person-service`. On R1, check the fixed RTP/Depth sources and `r1-web-camera-service`. On GO2, inspect the fixed RGB stream and DDS interface; depth is unsupported and no helper is expected. The browser must not override controlled source addresses, ports, pipelines, service names, or helper commands.

## 9. Adding another product

The initial generic deployment flow intentionally registers `g1`, `r1` and `go2`. Add future products by extending the controlled product-profile branch, system/user service definitions, helper/dependency declarations, and acceptance checks. Do not clone the whole deployment stack or introduce YAML, plugin frameworks, or new deployment dependencies unless the current Bash `case` model can no longer express the required behavior.
