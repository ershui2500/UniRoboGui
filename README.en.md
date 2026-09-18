# UniRoboGui — Unitree G1 / R1 Multi-Robot Development & Debugging Web GUI

[简体中文](README.md) | **English**

> **A unified development and debugging platform for Unitree G1 and R1** — one C++17 Web service selects the active product through \`RobotRegistry\`, Manifest/Capability data, and \`--robot\`, while shared HTTP/WebSocket, frontend, and deployment flows remain product-independent.

Unitree SDK2 provides a wide range of low-level robot capabilities, but in a real project those capabilities are spread across different data channels, service APIs, example programs, and sensor pipelines. Before application development can begin, customers often have to combine these interfaces themselves, write test programs, verify robot state, bring up perception and control paths, and prepare separate visualization or debugging tools for each subsystem.

**UniRoboGui is designed to remove that repeated integration and debugging work.** The project keeps a **single multi-product binary**, \`g1_web_server\`; choosing G1 or R1 changes dependencies, SDK prefix, network interface, helpers, systemd unit, and runtime arguments rather than compiling separate product binaries.

UniRoboGui does not replace Unitree SDK2. It provides an **observable, debuggable, and reusable engineering layer on top of SDK2**, reducing repetitive interface integration and shortening the path from “getting the SDK working” to “building with the robot”.

### Core capabilities

- **Unified SDK2 capability access**: G1 and R1 share the HTTP/WebSocket core, frontend, and binary while product Profile/RuntimeBundle configuration selects each robot's DDS, joint, control, and device capabilities.
- **Integrated debugging and visualization**: view robot state, product-specific RGB/depth streams, URDF pose, SLAM/point cloud, control state, and voice interaction in one browser workspace; visible modules follow Manifest/Capability state.
- **Perception, mapping, and navigation pipeline**: shares map, track, and navigation UI. G1 keeps its existing Mid-360/navigation pipeline; R1 exposes those capabilities only when attachments and deployment flags are explicitly enabled.
- **Joint debugging and action development**: product JointSchema drives live pose, upper/full-body debugging, hand-guided teaching, and action entry points without a second product joint-index table in the frontend.
- **Voice and LLM integration**: shares ASR, Unitree TTS, and built-in/customer LLM support. G1 can use local Kokoro; R1 defaults to Unitree native TTS only.
- **A foundation for further development**: provides Chinese/English UI, online/offline automated deployment, Mock regression mode, and control safety interlocks, making it suitable as a base environment for robot feature validation, field debugging, and higher-level application development.

Runtime truth comes from \`/api/robot/manifest\` and Capability state. Features that are not implemented, detected, explicitly enabled, or verified stay \`unsupported\`, \`disabled\`, or \`unverified\`; a visible UI entry is not proof that a capability is available.

> **Ready to deploy?** Jump to [Quick deployment](#quick-deployment).
>
> Both deployment cases are supported: the robot can access GitHub, or the robot is offline while another Ubuntu/Linux computer has Internet access.

---

## 1. Interface preview

All screenshots below are **G1 deployment examples** showing the shared Web workspace. They do not claim equivalent R1 attachment or hardware acceptance; R1 content is narrowed at runtime by Manifest/Capability and detected devices.

### 1.1 Integrated workstation

![UniRoboGui integrated workstation](docs/screenshots/web-workstation.jpg)

The integrated workstation is the main page for demonstrations, debugging, and field operation.

It can:

- show D435i RGB and depth streams, with independent enlarged views;
- render the G1 29-DoF model and follow the real joint state;
- display Mid-360 live point cloud, accumulated map, robot path, and navigation goals;
- start mapping, save/load maps, set initial pose, and run single-point or multi-point navigation;
- save frequently used navigation tasks for later reuse;
- display the current control state and expose supported motion modes, keyboard motion, and upper-body actions;
- use ASR, TTS, the robot's built-in LLM, or a customer OpenAI-compatible API;
- configure role prompts, fixed Q&A, wake phrases, and local TTS in customer mode.

Map controls include pan, orbit, top-down view, 3D view, zoom, fit-to-map, and locate-robot actions. Motion, navigation, and taught actions remain subject to robot state and safety interlocks.

### 1.2 Robot status

![UniRoboGui robot status](docs/screenshots/robot-status.jpg)

The Robot Status page is intended for deployment acceptance, runtime observation, and fault isolation.

It displays:

- SDK2 DDS connection and source availability;
- motion mode, state version, and telemetry age;
- BMS state including charge, voltage, current, and temperature;
- odometry, velocity, and robot pose;
- waist IMU and main-board IMU;
- position, velocity, torque, temperature, voltage, and raw state for the 29 body joints;
- the local G1 URDF selected for the detected model, synchronized with real joint angles;
- joint search/filtering and raw motor slots.

This page is primarily read-only and does not require entering a robot control mode.

### 1.3 Robot debug console

![UniRoboGui robot debug console](docs/screenshots/joint-debug.jpg)

The debug console targets secondary development, joint-pose debugging, and taught-action workflows.

It supports:

- **upper body** and **full body** debug modes;
- radians or degrees for joint targets;
- loading the current real robot pose as the target to reduce sudden jumps;
- visibility into backend interlocks, FSM, LowState, and DDS-control safety conditions;
- manual joint target editing and application when safety requirements are satisfied;
- 20 Hz hand-guided teaching and local action storage;
- playback and deletion of saved actions, with either control release or hold-last-pose at completion;
- binding local actions to reserved G1 remote-control key combinations;
- an explicit stop/release action to return control.

This page can command real joints. Support the robot securely, clear people and obstacles from the workspace, and confirm that the current robot state is appropriate before using it.

### 1.4 Diagnostics and raw data

![UniRoboGui diagnostics](docs/screenshots/diagnostics.png)

The Diagnostics page helps determine whether an issue is more likely related to the data path, motion state, motors, battery, main board, or service initialization.

It currently summarizes:

- **DDS / odometry**: DDS initialization, online source count, LowState latency, and odometry error code;
- **motor diagnostics**: number of reported motor faults, maximum case/winding temperature, motor-voltage range, and known fault items;
- **battery diagnostics**: SOC/SOH, cell range, cell delta, current, maximum temperature, and raw BMS state;
- **main-board diagnostics**: temperature, fan state, and raw board fields;
- **service state**: control-service or voice-service initialization failures;
- **raw data**: original BMS and main-board values for comparison with official documentation and field logs.

Fields whose bit definitions are not publicly documented are shown as raw values rather than guessed fault meanings.

---

## 2. Feature overview

| Module | User-facing capability |
| --- | --- |
| Integrated workstation | Camera, 3D robot, SLAM map, control, and voice/LLM interaction in one responsive page |
| Robot status | Product-profile DDS, BMS, FSM, odometry, IMU, semantic joints, and current-model 3D pose |
| Diagnostics | DDS, odometry, motor, battery, and main-board summaries plus raw values |
| 3D model | Local Three.js + URDF Loader selects G1/R1 assets from the Manifest and follows the product JointSchema |
| SLAM / point cloud | Product-Capability-gated live point cloud, accumulated map, robot track, and occupancy voxels |
| Map management | Start mapping, save/load/download/exit maps, and set initial pose |
| Navigation | Single/multi-point tasks when Capability allows them; real R1 navigation is disabled by default |
| RGB / depth camera | G1 uses D435i/librealsense; R1 uses controlled fixed RGB/depth sources, both narrowed by runtime evidence |
| Robot control | Safety lock, common motion modes, keyboard motion, upper-body presets, and firmware taught actions |
| Debug and teaching | Upper/full-body joint targets, hand-guided recording, playback, hold/release, and remote binding |
| ASR | Receive robot ASR results, display recent recognition history, and forward text to the LLM |
| TTS | Unitree native TTS; G1 may use local Kokoro while R1 does not install it by default |
| LLM | Robot built-in dialog or customer OpenAI-compatible API with role, fixed Q&A, and wake phrases |
| Internationalization | Chinese / English switching for static and dynamic Web UI |
| Mock mode | Development/regression without real DDS initialization for UI, HTTP, WebSocket, and safety-state testing |

---

<a id="quick-deployment"></a>

## 3. Quick deployment

G1 and R1 share the same deployment entry point: `scripts/deploy.sh`. The script never guesses the robot model: `--product` is **required**. Use `g1` for a G1 and `r1` for an R1.

General syntax:

```bash
bash scripts/deploy.sh <COMMAND> --product <g1|r1> [OPTIONS]
```

> `g1|r1` means “choose one”; do not type `g1|r1` literally. For example, use `--product g1` for G1 or `--product r1` for R1.

Supported commands:

| Command | Run it on | Purpose | Modifies the robot? |
| --- | --- | --- | --- |
| `install` | Robot | When the robot can reach GitHub: update source, prepare dependencies, perform a Release build and CTest, install/update the service, and run read-only acceptance checks | Yes |
| `from-pc` | Ubuntu/Linux PC that can reach GitHub and SSH to the robot | When the robot cannot reach GitHub: prepare resources on the PC, transfer them with SSH/rsync, then automatically install and verify on the robot | Yes |
| `check` | Robot | Check OS/architecture, DDS interface, packages, SDK, helpers, and other deployment prerequisites | No; it does not install or restart services |

Supported options:

| Option | Required | Accepted values | Default / scope | Meaning |
| --- | --- | --- | --- | --- |
| `--product` | **Yes** | `g1`, `r1` | No default | Select the actual robot. Use `--product g1` for G1 and `--product r1` for R1 |
| `--host` | No | `USER@HOST` or an SSH Host alias | `from-pc` only; G1/R1 currently default to `unitree@192.168.123.164` | Override the robot SSH target, for example `--host unitree@192.168.123.200` |
| `--lang` | No | `zh`, `en` | Default: `zh` | Changes deployment-script output language only; it does not change the Web UI language |
| `-h` / `--help` | No | None | Any scenario | Show the complete CLI help |

There is **no default robot model**; this prevents accidentally deploying G1 settings to an R1 or vice versa. For a first deployment, use one of these recommended templates based on **whether the robot itself can reach GitHub**, then replace `<PRODUCT>` with `g1` or `r1`:

```bash
# Robot can reach GitHub: run on the robot
bash scripts/deploy.sh install --product <PRODUCT>

# Robot cannot reach GitHub: run on the online PC
bash scripts/deploy.sh from-pc --product <PRODUCT>
```

Copy-ready G1/R1 examples:

```bash
# G1
bash scripts/deploy.sh install --product g1
bash scripts/deploy.sh from-pc --product g1
bash scripts/deploy.sh check --product g1

# R1
bash scripts/deploy.sh install --product r1
bash scripts/deploy.sh from-pc --product r1
bash scripts/deploy.sh check --product r1

# Help / English deployment logs
bash scripts/deploy.sh --help
bash scripts/deploy.sh install --product g1 --lang en
bash scripts/deploy.sh install --product r1 --lang en
```

Legacy `deploy_g1_*` / `install_g1*` Chinese and English scripts remain for one compatibility cycle only. They print a deprecation message and forward to `--product g1`.

### 3.1 Before deployment

Target environment:

| Item | Requirement |
| --- | --- |
| Robot | Unitree G1 / R1 currently registered in RobotRegistry |
| PC2 OS | Ubuntu 20.04 AArch64 |
| User | `unitree` |
| DDS interface | G1: `eth0`; R1: `eth10` |
| SDK prefix | G1: `/opt/unitree_robotics` or user prefix; R1: existing `/usr/local` |
| Web service | G1: `g1-web-control.service`; R1: `r1-web-control.service` |
| Project directory | `/home/unitree/UniRoboGui` |
| Camera | G1 D435i/librealsense; R1 EDU fixed camera chain/helper |
| Browser client | Computer/tablet with network reachability to the robot |

When SSH login displays a ROS environment selection prompt, press Enter and select **none**. Do not repurpose the selected product's DDS interface or source ROS/RMW/CycloneDDS into the Web process.

The installer chooses one of two service scopes:

- **Interactive sudo available:** install the selected product's system service and controlled camera helper. G1 can install/reuse SDK2; R1 only validates and reuses `/usr/local` and never overwrites it automatically.
- **Interactive sudo unavailable:** user services are allowed only when system packages are complete, `systemctl --user` works, and `Linger=yes`. G1 may use a user SDK prefix; R1 still reuses `/usr/local`.

External resources differ by method:

- **G1:** keeps the existing D435i/librealsense, Kokoro, and navigation defaults.
- **R1:** defaults to Unitree native TTS, does not install Kokoro, does not declare Mid-360, and does not enable real navigation.
- **This is a public repository**, so a normal clone does not require a repository access token. Regardless of deployment method, do not put GitHub tokens, SSH/Wi-Fi passwords, API keys, or other credentials in README files, script arguments, logs, or Git commits.

### 3.2 Method A: the robot can access GitHub

Clone the project on the robot, then choose the command for the actual model:

```bash
git clone https://github.com/ershui2500/UniRoboGui.git /home/unitree/UniRoboGui
cd /home/unitree/UniRoboGui

# Unitree G1
bash scripts/deploy.sh install --product g1 --lang en

# Unitree R1
bash scripts/deploy.sh install --product r1 --lang en
```

If the repository already exists, enter it and run the command for the actual product:

```bash
cd /home/unitree/UniRoboGui

# Choose this for G1
bash scripts/deploy.sh install --product g1 --lang en

# Choose this for R1
bash scripts/deploy.sh install --product r1 --lang en
```

The script validates the selected product's Ubuntu/AArch64/`unitree` environment, DDS interface, and clean ROS/RMW state; protects dirty source trees; handles SDK/camera/TTS dependencies according to the product profile; then performs a Release build, full CTest, `ldd`, exact service installation, and read-only runtime acceptance. G1 keeps the existing SDK2/librealsense/Kokoro flow; R1 validates the existing `/usr/local` SDK and skips automatic librealsense/Kokoro installation.

If the first step shows that GitHub is unreachable, the script stops and tells you to use Method B.

### 3.3 Method B: the robot cannot access GitHub, but another computer can

On an Ubuntu/Linux computer that can access GitHub **and** connect to the robot over SSH, clone the project and run the command for the actual product:

```bash
git clone https://github.com/ershui2500/UniRoboGui.git
cd UniRoboGui

# Unitree G1
bash scripts/deploy.sh from-pc --product g1 --lang en

# Unitree R1
bash scripts/deploy.sh from-pc --product r1 --lang en
```

The current default SSH target for both G1 and R1 is:

```text
unitree@192.168.123.164
```

If the robot uses a different address, override it with `--host` while keeping the correct product value:

```bash
# G1 example
bash scripts/deploy.sh from-pc --product g1 --host unitree@<G1_IP> --lang en

# R1 example
bash scripts/deploy.sh from-pc --product r1 --host unitree@<R1_IP> --lang en
```

The online-computer flow checks GitHub/SSH and robot-side dirty state, prepares only the selected product's resources, and transfers them with rsync. G1 prepares SDK2, librealsense, Kokoro, and the AArch64/Python 3.8 wheelhouse. R1 neither downloads nor overwrites the existing `/usr/local` SDK and does not prepare Kokoro. Project transfer excludes `.git/`, AGENTS/reference material, and local robot workspaces while preserving customer runtime configuration and build output.

If Ubuntu packages are missing, provide the required Ubuntu 20.04 arm64 packages. Do not disconnect or repurpose the selected product's DDS interface.

Run Method B from an interactive terminal. The first SSH connection may ask you to verify the host fingerprint, and SSH/sudo may ask for a password. Passwords are entered only in the terminal prompts; deployment scripts do not store SSH, sudo, Wi-Fi, or API passwords.

If interactive sudo is unavailable on the robot but system packages are already complete and persistent user systemd is available with `Linger=yes`, the installer can switch to user-local deployment automatically.

### 3.4 After deployment

Check network addresses as appropriate for the selected product:

```bash
ip -4 addr show wlan0
ip -4 addr show eth0
ip -4 addr show eth10
```

Open:

```text
http://<ROBOT_IP>:8080
```

Read-only acceptance checks. System installations use `systemctl`; user-local installations use `systemctl --user`:

```bash
# Replace <service> with g1-web-control.service or r1-web-control.service
systemctl is-active <service> 2>/dev/null || systemctl --user is-active <service>
curl --noproxy '*' -fsS http://127.0.0.1:8080/api/health
curl --noproxy '*' -fsS http://127.0.0.1:8080/api/robot/manifest
curl --noproxy '*' -fsS http://127.0.0.1:8080/api/control/status
```

If local Kokoro TTS is enabled:

```bash
systemctl is-active g1-local-tts.service 2>/dev/null || \
  systemctl --user is-active g1-local-tts.service
curl --noproxy '*' -fsS http://127.0.0.1:8765/health
```

See [docs/deployment-dependencies.en.md](docs/deployment-dependencies.en.md) for the detailed dependency and offline-transfer procedure.

### 3.5 Upgrade

Use the same deployment flow for upgrades:

- Robot can access GitHub: `bash scripts/deploy.sh install --product <PRODUCT> --lang en`.
- Robot cannot access GitHub: run `bash scripts/deploy.sh from-pc --product <PRODUCT> --lang en` on the online computer; add `--host <SSH_HOST>` only when the robot does not use the default SSH target.

Replace `<PRODUCT>` with `g1` or `r1`, for example:

```bash
# G1 upgrade
bash scripts/deploy.sh install --product g1 --lang en

# R1 upgrade
bash scripts/deploy.sh install --product r1 --lang en
```

Method A uses a safe `git pull --ff-only`. Method B updates the online checkout and synchronizes it to the robot. Both refresh upstream dependencies as needed, rebuild the project, and preserve customer runtime configuration.

If uncommitted project source changes are detected, the scripts stop instead of overwriting them.

When replacing an already running Web service, the installer requires a matching Manifest product identity, robot motion confirmed as `stopped`, and all three commanded velocities at zero. If the other product's service is active, deployment stops rather than switching products automatically. Rerunning the deployment scripts is supported; existing product dependencies and build output are reused according to policy.

---

## 4. Access and operation

### 4.1 Web entry point

Open:

```text
http://<ROBOT_IP>:8080
```

The left navigation contains:

- **Workstation**
- **Robot Status**
- **Robot Debug**
- **Diagnostics**

The top-right area provides Chinese / English switching plus connection, battery, FSM, and control-lock summaries.

### 4.2 Network layout

Production communication uses Unitree SDK2 DDS:

```text
G1: eth0  -> Unitree SDK2 DDS
R1: eth10 -> Unitree SDK2 DDS
wlan0     -> Web access / Internet when configured
```

Do not disconnect or repurpose the selected product's DDS interface to obtain Internet access. The Web, SLAM, and camera processes do not require sourcing ROS Foxy, ROS Noetic, ROS 2, or another RMW/CycloneDDS environment.

### 4.3 Motion and debugging safety

Robot control, navigation, upper-body actions, and joint debugging can create real physical motion.

Before using them:

- securely support the robot or place it in a stable state appropriate for the action;
- clear nearby people and obstacles;
- verify the Web connection and continuously updating robot telemetry;
- verify that the current FSM/control mode satisfies the UI requirements;
- prefer Mock or read-only validation first when the expected physical result is uncertain.

The Web interface does not provide built-in user authentication and should not be exposed to an untrusted network or the public Internet.

---

## 5. Compatibility

| Item | G1 | R1 |
| --- | --- | --- |
| DDS interface | `eth0` | `eth10` |
| SDK prefix | `/opt/unitree_robotics` or user prefix | existing `/usr/local`; validate only, never auto-overwrite |
| Web service | `g1-web-control.service` | `r1-web-control.service` |
| Camera | D435i/librealsense + G1 helper | fixed R1 EDU RGB/Depth + R1 helper |
| Default TTS | Kokoro + Unitree fallback | Unitree native TTS |
| LiDAR/navigation default | existing G1 Mid-360/navigation configuration | no Mid-360 declaration and no real navigation |
| Binary | `build/g1_web_server --robot g1` | `build/g1_web_server --robot r1` |

Both products use Ubuntu 20.04 AArch64, the `unitree` account, `/home/unitree/UniRoboGui`, and a modern Chromium-class browser. Runtime Manifest/Capability state and field hardware remain the source of truth for actual availability.

Robot firmware, SDK2, LiDAR services, device nodes, and network addresses can differ by robot batch and field configuration. Verify against the actual robot.

### 5.1 Project structure

```text
UniRoboGui/
├── README.md                   # Chinese documentation
├── README.en.md                # English documentation
├── LICENSE
├── VERSION
├── CMakeLists.txt
├── include/                    # C++ headers
├── src/                        # C++ backend
├── web/                        # Web frontend and local static assets
├── tests/                      # CTest / HTTP regressions
├── scripts/                    # deployment, TTS, camera helper, release scripts
├── deploy/                     # systemd services
├── config/                     # robot-local runtime configuration created during install/use
└── docs/                       # deployment notes and screenshots
```

The recommended deployment entry point is `scripts/deploy.sh`. Legacy `deploy_g1_*` / `install_g1*` wrappers remain only for one compatibility cycle.

---

## 6. Troubleshooting

### 6.1 The robot cannot access GitHub

Do not keep retrying GitHub operations on the robot. On an Ubuntu/Linux computer that can reach GitHub:

```bash
bash scripts/deploy.sh from-pc --product g1 --host unitree@<G1_IP> --lang en
bash scripts/deploy.sh from-pc --product r1 --host unitree@<R1_IP> --lang en
```

The script prepares dependencies and transfers them with SSH/rsync.

### 6.2 Method B still reports APT / Ubuntu mirror failures

Method B transfers GitHub, GitHub Release, and PyPI resources, but it does not package the complete Ubuntu repository.

Run the read-only preflight:

```bash
cd /home/unitree/UniRoboGui
bash scripts/deploy.sh check --product g1 --lang en
# or:
bash scripts/deploy.sh check --product r1 --lang en
```

If it reports missing Ubuntu packages, temporarily give `wlan0` access to an Ubuntu 20.04 mirror or install the exact arm64 `.deb` dependencies. Do not disconnect or modify the selected product's DDS interface, and do not substitute x86_64 or a different Ubuntu release.

### 6.3 G1: GitHub works but Kokoro / PyPI fails

GitHub connectivity does not guarantee PyPI connectivity. First-time Kokoro setup needs Python wheels. If robot-side PyPI access is unreliable, use Method B so the online computer prepares the AArch64 / Python 3.8 wheelhouse.

### 6.4 `wlan0` is `wifi:unavailable` or sees no networks

Check:

```bash
nmcli device status
nmcli radio wifi
rfkill list wifi
```

If `rfkill` reports `Soft blocked: yes`:

```bash
nmcli radio wifi on
```

Then scan/connect again. Do not disable, reset, or disconnect the selected product's DDS interface.

If Internet access on the robot is not desired, leave Wi-Fi alone and deploy through Method B over the robot's reachable SSH network.

### 6.5 `undefined symbol: ddsi_sertype_v0`

This usually indicates that a ROS/CycloneDDS environment was mixed with Unitree SDK2 libraries.

Open a new SSH session and select **none** at the ROS environment prompt, then restart or redeploy the service.

### 6.6 Port 8080 / Web UI is unavailable

On the robot:

```bash
systemctl --no-pager --full status g1-web-control.service
systemctl --no-pager --full status r1-web-control.service
curl --noproxy '*' -v http://127.0.0.1:8080/api/health
ss -ltnp 'sport = :8080'
```

If the local health check succeeds, investigate the network path between the browser computer and robot.

### 6.7 G1: D435i has no image

Check:

- USB enumeration for the D435i;
- whether librealsense2 is installed correctly;
- whether another robot service owns the camera;
- whether a fixed `/dev/videoN` value was entered accidentally.

"Start RGB + Depth" first tries concurrent librealsense2 access alongside the robot's existing camera service. On G1 firmware/images that allow concurrent access, `master_service` does not need to be stopped and the root helper is not required.

Only when concurrent access is rejected does the Web backend fall back to "pause camera owner service + automatic V4L2 detection".

If that fallback reports `first_person_status_failed`, rerun the official deployment flow so these files are installed and verified:

```text
/usr/local/sbin/g1-web-first-person-service
/etc/sudoers.d/g1-web-camera
```

Because video device numbers can change across USB enumeration, leave the manual device field blank unless you have a specific reason to pin it.

### 6.8 Robot data is offline in the page

Check the selected product's DDS interface (G1 `eth0` / R1 `eth10`), the matching Web service, ROS/RMW/CycloneDDS environment contamination, and whether the Manifest-required data sources are actually publishing.

### 6.9 Customer LLM does not work

Verify the API Base URL, model identifier, and authentication against the customer's actual service. UniRoboGui supports an OpenAI-compatible Chat Completions interface; model names differ between providers.

---

## 7. Documentation and official references

### 7.1 Project documentation

- [Chinese README](README.md)
- [Dependency installation and offline deployment](docs/deployment-dependencies.en.md)
- [Third-party static asset notices](web/assets/THIRD_PARTY_NOTICES.md)

### 7.2 Unitree references

- [G1: Get SDK](https://support.unitree.com/home/en/G1_developer/get_sdk)
- [G1 SLAM navigation service interface](https://support.unitree.com/home/en/G1_developer/slam_navigation_services_interface)
- [G1 LiDAR service interface](https://support.unitree.com/home/en/G1_developer/lidar_services_interface)
- [G1 depth camera instruction](https://support.unitree.com/home/en/G1_developer/depth_camera_instruction)
- [Unitree SDK2](https://github.com/unitreerobotics/unitree_sdk2)

---

## 8. Issues and contributions

For a new G1, different firmware, or a different field network, review [Troubleshooting](#6-troubleshooting) and the deployment dependency document first.

If the issue is project-specific, open a GitHub Issue and include, where possible:

- UniRoboGui version;
- Unitree G1 EDU variant and `mode_machine` if known;
- the page and operation where the issue occurs;
- browser/HTTP errors;
- relevant `systemctl status` / `journalctl` output;
- whether it reproduces reliably and whether the robot was moving or under debug control beforehand.

Remove API keys, Wi-Fi passwords, SSH credentials, cookies, tokens, and other secrets before attaching logs or screenshots.

Contributions and secondary development are welcome. User-visible Web features must keep Chinese and English in sync. Robot-motion, navigation, mode switching, upper-body control, and teaching changes should be validated in Mock/read-only paths first, followed by explicitly safety-approved robot tests.

---

## 9. Acknowledgements and attribution

UniRoboGui builds on infrastructure from the robotics and open-source communities.

### 9.1 Robot and device stack

- [Unitree SDK2](https://github.com/unitreerobotics/unitree_sdk2) — DDS communication, robot state, and service APIs.
- [Unitree unitree_ros](https://github.com/unitreerobotics/unitree_ros) — source of the G1 URDF/STL assets distributed at pinned upstream revisions.
- [librealsense](https://github.com/realsenseai/librealsense) — Intel RealSense D435i device support.

### 9.2 3D and robot visualization

- [Three.js](https://github.com/mrdoob/three.js) — browser-side G1 model, point-cloud, and map rendering.
- [urdf-loaders](https://github.com/gkjohnson/urdf-loaders) — browser-side URDF parsing/model loading.
- [RViz](https://github.com/ros2/rviz) and [Foxglove Studio](https://github.com/foxglove/studio) — design references for robot-state visualization, point clouds, map interaction, and diagnostic organization. UniRoboGui does not require RViz or Foxglove at runtime.

### 9.3 Voice

- [sherpa-onnx](https://github.com/k2-fsa/sherpa-onnx) — ONNX runtime and model distribution used for local Kokoro TTS.

Thanks also to the maintainers of Boost, JsonCpp, OpenCV, libcurl, ZeroMQ, and other foundational open-source components.

---

## 10. License

### 10.1 UniRoboGui-owned code

UniRoboGui-owned code is released under the [MIT License](LICENSE).

The MIT License permits use, copying, modification, merging, publication, distribution, sublicensing, and sale, subject to preservation of the copyright and license notice. The software is provided "as is" without warranty. The repository [LICENSE](LICENSE) file is authoritative.

### 10.2 Third-party static assets distributed in this repository

Some frontend/model assets are redistributed under their upstream licenses:

| Asset | Source | License |
| --- | --- | --- |
| G1 URDF / STL | Unitree `unitree_ros` | BSD-3-Clause |
| Three.js 0.164.1 | Three.js | MIT |
| urdf-loader 0.13.1 | urdf-loaders | Apache-2.0 |

See [web/assets/THIRD_PARTY_NOTICES.md](web/assets/THIRD_PARTY_NOTICES.md) for pinned versions, upstream revisions, and local license files.

### 10.3 External dependencies obtained during deployment

Unitree SDK2, librealsense2, sherpa-onnx, Kokoro model assets, and system libraries are obtained from their respective upstream sources during deployment. Their licenses, model terms, and redistribution requirements are controlled by those upstream projects.

If you redistribute binaries, models, third-party assets, or a commercial distribution, review every applicable upstream LICENSE / NOTICE / model license. UniRoboGui's MIT License does not relicense third-party components.

### 10.4 Trademarks and project relationship

Unitree, Intel RealSense, Three.js, Foxglove, ROS / RViz, and other third-party names and trademarks belong to their respective owners.

UniRoboGui is an independent third-party project. It is not an official Unitree product and does not imply endorsement by Unitree or any other upstream project.
