# UniRoboGui G1 / R1 Deployment and Troubleshooting Guide

[简体中文](deployment-dependencies.md) | **English**

This guide covers both Unitree G1 and R1. They share one deployment entry point, build flow, and safety checks. Product-specific differences such as network interfaces, SDK paths, services, cameras, TTS, and optional attachments are kept in controlled product profiles instead of duplicated deployment guides.

## 1. Recommended entry point

~~~bash
# Robot has Internet access: install or upgrade
bash scripts/deploy.sh install --product g1 --lang en
bash scripts/deploy.sh install --product r1 --lang en

# Deploy from an Internet-connected computer
bash scripts/deploy.sh from-pc --product r1 --lang en

# Read-only environment preflight
bash scripts/deploy.sh check --product g1 --lang en
bash scripts/deploy.sh check --product r1 --lang en
~~~

`--product` is required and currently accepts only `g1` or `r1`, as registered by the project. `--host` is valid only with `from-pc`. Legacy `deploy_g1_*` and `install_g1*` entry scripts remain for one compatibility cycle; they print a deprecation notice and forward to `--product g1`. The legacy `--robot USER@HOST` option is translated to `--host`.

## 2. Product matrix

| Item | G1 | R1 |
| --- | --- | --- |
| DDS interface | `eth0` | `eth10` |
| SDK / CMake prefix | `/opt/unitree_robotics`; user-local installs may use `/home/unitree/.local/unitree_robotics` | `/usr/local`; validate the existing installation and never overwrite it automatically |
| Runtime library path | SDK `prefix/lib` + `/usr/local/lib`; reuse existing librealsense when needed | `/usr/local/lib` |
| Web service | `g1-web-control.service` | `r1-web-control.service` |
| systemd user unit | `deploy/g1-web-control.user.service` | `deploy/r1-web-control.user.service` |
| Camera | D435i + librealsense + `g1-web-first-person-service` | Fixed R1 EDU RGB/Depth sources + `r1-web-camera-service` |
| Default TTS | Kokoro + Unitree fallback | Unitree native TTS |
| SDK installation policy | Install from the official SDK2 source when missing | Validate the existing R1 headers, CMake package, and CycloneDDS libraries; stop if they are missing |
| Mid-360 / navigation | Keep the existing G1 configuration and service defaults | Do not declare Mid-360 or enable real navigation by default |
| Binary | `build/g1_web_server --robot g1` | `build/g1_web_server --robot r1` |

Both products use the same source tree and the same historical `g1_web_server` binary name. Runtime product identity comes from `--robot`, `RobotRegistry`, the Manifest, and Capability state—not from the binary filename.

## 3. Shared deployment flow

The deployment scripts follow the same sequence for both products:

1. Validate Ubuntu 20.04 AArch64, the `unitree` user, the selected DDS interface, and the absence of ROS/RMW/CycloneDDS environment contamination.
2. Check the source tree for local modifications. Stop instead of overwriting uncommitted project code.
3. Validate or prepare SDK, camera, and TTS dependencies according to the selected product profile.
4. Perform a Release build and run the complete CTest suite.
5. Use `ldd` to verify there are no `not found` entries and that CycloneDDS resolves from the selected product's SDK prefix.
6. Before replacing a running Web service, read the Manifest and `/api/control/status`.
7. Stop or replace the service only when the product identity matches, `motion.active=false`, `state=stopped`, and `vx/vy/vyaw` are all zero.
8. If the other product's service is active, stop the deployment instead of shutting it down or switching products automatically.
9. Install the exact system-level or user-level unit plus the selected product's controlled camera helper/sudoers files.
10. After startup, perform read-only acceptance checks: a single listener on port 8080, health, Manifest, required DDS sources, zero-motion state, MainPID arguments, process environment, and helper status.

Automated acceptance never calls real `SetVelocity`, `StopMove`, `Start`, `StandUp`, `SetFsmId`, joint, head, mode-switching, or navigation commands. Real motion acceptance is performed manually by an on-site operator through the Web interface.

## 4. G1 deployment notes

G1 keeps the existing baseline:

- `eth0` carries SDK2 DDS traffic, while `wlan0` is used for Web access and Internet connectivity.
- The system SDK prefix is `/opt/unitree_robotics`; a persistent user-local prefix may be used when required.
- D435i support requires librealsense. System-level installation provides the controlled `g1-web-first-person-service` helper.
- Kokoro is the default local TTS backend, with Unitree TTS retained as a fallback.
- `g1-web-control.service` explicitly includes `--robot g1` and preserves the existing `--enable-navigation` default.
- `from-pc` prepares SDK2, librealsense, the Kokoro model, and an AArch64/Python 3.8 wheelhouse.

## 5. R1 deployment notes

R1 uses these boundaries:

- The DDS interface is fixed to `eth10`.
- The SDK/CMake prefix is fixed to `/usr/local`.
- The deployer validates only the R1 Audio/Loco headers required by this project, the `unitree_sdk2` CMake package, and the `libddsc` / `libddscxx` runtime libraries. If any are missing, deployment stops instead of overwriting `/usr/local`.
- Unitree native TTS is the default; Kokoro is not installed.
- System-level installation uses `r1-web-camera-service`. If a user-level deployment has no helper, the related Camera Capability is downgraded according to runtime evidence.
- `r1-web-control.service` explicitly includes `--robot r1` and does not include `--device mid360` or `--enable-navigation`.
- `from-pc` defaults to `unitree@192.168.123.164`. OpenSSH first tries existing keys/agents, allows host-key confirmation on first connection, and prompts for a password only when needed. The scripts do not store or echo passwords. Use `--host` to override the address or use an existing SSH Host alias.

## 6. `from-pc` transfer boundaries

When `from-pc` transfers the project with rsync:

- `.git` is not transferred.
- `AGENTS.md`, `*_AGENTS.md`, `.robot-workspace`, `.robot-backups`, `.robot-sync`, and local G1/R1 reference-material directories are not transferred.
- The robot's complete `config/` runtime configuration directory and `build/` directory are preserved.
- Deployment stops if the robot-side Git checkout contains uncommitted project changes.
- Controlled `--delete` is used only for non-Git directories explicitly marked as managed by the deployment scripts.
- SSH, sudo, Wi-Fi, API credentials, and other secrets are never written into project files, command arguments, logs, or caches.

## 7. Read-only preflight

~~~bash
bash scripts/deploy.sh check --product g1 --lang en
bash scripts/deploy.sh check --product r1 --lang en
~~~

`check` does not install packages, modify files, or stop/start services. It validates the operating system, architecture, selected network interface, required packages, SDK state, camera-helper state, and other deployment prerequisites. On R1, the SDK is validated directly under `/usr/local`.

## 8. Post-deployment acceptance

The scripts perform the core read-only acceptance checks automatically. For manual verification:

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
- G1/R1 CycloneDDS libraries resolve from the SDK prefix expected for the selected product.
- Read-only camera-helper status checks return valid active/inactive semantics.

## 9. Troubleshooting

### GitHub is unreachable

Do not keep retrying GitHub operations on the robot and do not modify the DDS interface to obtain Internet access. Use an Internet-connected computer instead:

~~~bash
bash scripts/deploy.sh from-pc --product g1 --host unitree@<G1_IP> --lang en
bash scripts/deploy.sh from-pc --product r1 --host unitree@<R1_IP> --lang en
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
~~~

### Camera

On G1, check librealsense, D435i USB enumeration, and `g1-web-first-person-service`. On R1, check the fixed RTP/Depth sources and `r1-web-camera-service`. The browser must not override controlled source addresses, ports, pipelines, service names, or helper commands.

## 10. Adding another product

The initial generic deployment flow intentionally registers only `g1` and `r1`. Add future products by extending the controlled product-profile branch, system/user service definitions, helper/dependency declarations, and acceptance checks. Do not clone the whole deployment stack or introduce YAML, plugin frameworks, or new deployment dependencies unless the current Bash `case` model can no longer express the required behavior.
