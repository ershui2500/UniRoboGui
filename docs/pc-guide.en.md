# Ubuntu PC SDK2 console: G1 / R1 / GO2 / A2

Run UniRoboGui on an Ubuntu 22.04 x86_64 PC with Python 3.10. The robot does not need this project installed; its existing telemetry services must remain available. This is separate from the robot deployment commands in `deploy.sh`.

## Install and start

From the project checkout on the PC:

```bash
bash scripts/pc.sh install
bash scripts/pc.sh check
bash scripts/pc.sh start
```

Open `http://127.0.0.1:8080`. The server binds only to loopback and starts disconnected. Ctrl+C shuts down the manager and its worker. No robot service, camera helper, TTS model, automatic connection or network configuration is installed.

Missing Ubuntu build packages are installed through sudo. The installer pins SDK2 revision `7740f8b67e386ab09c3b333187fd5f8582a75ddc` under `~/.local/share/unirobogui/sdk2/<revision>/`, preserving existing SDK installations. Reinstallation reuses that SDK and repeats the Release build, full CTest and ldd checks. A prepared clean official source checkout at that revision can be supplied with `--sdk2-source /absolute/path`. Use `--sdk-prefix` for a separate SDK destination; incomplete or unverified existing prefixes are rejected. `--build-dir` selects a build directory and must also be supplied to subsequent start/check commands. Reinstall after moving installed directories.

Use a terminal without ROS/RMW/CycloneDDS environment contamination. Runtime DDS libraries come from the selected SDK. Normal execution does not require sudo.

PC install and preflight check require GStreamer tools, good/bad plug-ins and libav. The fixed RTP/H.264 pipeline uses `watchdog` from `gstreamer1.0-plugins-bad`. For existing installations missing this package, rerun `bash scripts/pc.sh install`. OpenCV availability and online DDS alone do not establish that the video pipeline dependencies are complete.

R1 PC RGB and depth are not connected. Both panels show “Robot camera transport is not connected on PC”. Camera reception, start commands and frame endpoints remain disabled; PC does not use the R1 GST video paths.

## Connect

1. Connect one G1, R1, GO2 or A2 through Ethernet or an isolated switch. Only one robot may be present on the selected network.
2. Configure the PC's wired IPv4 manually for the actual robot network, with a unique address. The installer does not change addresses, routes or firewall settings.
3. Select G1, R1, GO2 or A2, choose an UP physical Ethernet/USB interface with IPv4 in the header, and click Connect. While disconnected, the list updates automatically every 5 seconds and preserves your selection. Detection pauses while connected.
4. Check the connection status at the top right. The info icon beside the interface shows the current interface, IPv4 and last data time. Fresh required telemetry determines whether the connection is online; DDS initialization alone is insufficient.
5. The button shows Disconnect for the current connection, including after data/network loss; without fresh stop evidence normal disconnection is refused. Both products require fresh LowState, control state and odometry. R1 odometry is optional telemetry, but a missing or stale odometry source still blocks disconnect confirmation. Use Recover read-only connection in connection info as described below. Choosing another interface changes it to Connect; clicking ends the old connection before switching. Disconnect before replacing the robot.

The selected model is a Profile choice, not independently verified physical identity. DDS uses the existing domain 0 and fixed product topics. A robot IP does not isolate DDS devices. Wi-Fi, bridges, tunnels and simultaneous robots are outside this release.

The PC reuses existing telemetry and SDK2 capabilities for G1, R1, GO2 and A2. R1 retains its 26 semantic joints and sparse motor slots; GO2 and A2 retain their 12-joint quadruped schemas. Real motion and joint control still require field operator validation. GO2/A2 RGB uses the existing fixed RTP/H264 receiver. G1/R1 RGB, all PC depth streams, and the R1 GST camera path remain disabled; the UI reports that robot camera transport is not connected on PC. GO2 and A2 audio volume and stream controls use their native SDK2 adapters; robot-local Kokoro TTS is not installed on the PC. A2 real joint control defaults to the built-in experimental gains described below; an operator-provided gains file can override them. Local PC devices/processes are not evidence of robot hardware/services.

For G1/R1/GO2/A2, PC connection status requires initialized DDS and fresh LowState, independently of the Manifest `required_sources` used for overall health. Missing motion state, odometry, battery or mainboard sources remain visibly stale/offline without disconnecting the entire workbench. With GO2 MCF stopped, fresh LowState keeps status and joint-debug views available. High-level motion/actions still require fresh motion state; low-level control retains its ownership, LowState, gains and other safety checks. Cached FSM values are not shown as current when motion state is stale. Normal disconnect/switch stop-evidence requirements are unchanged.

Stale LowState marks the PC connection stale using existing freshness limits. The robot session does not reconnect automatically or change interfaces. Browser WebSocket reconnection only reconnects to the existing PC session. For missing data, check cable, IPv4, multicast, firewall and existing robot publishers; do not probe through motion commands.

G1/R1/GO2/A2 wired hardware acceptance, cable loss/recovery and clean-machine first installation require separate verification. Mock success does not establish direct hardware or motion validation.

## Mock and interfaces

```bash
bash scripts/pc.sh start --mock --port 18080
```

Mock explicitly identifies simulated data and allows `lo`; real mode rejects loopback.

| Endpoint | Contract |
| --- | --- |
| `GET /api/pc/interfaces` | Wired interfaces, UP state and IPv4 addresses |
| `GET /api/pc/session` | Session ID, selected product, state, Mock flag, DDS, last telemetry reception and validation status |
| `POST /api/pc/connect` | JSON `product_id` and `interface`; G1/R1/GO2/A2 and ready local wired interfaces only |
| `POST /api/pc/disconnect` | JSON `{}`; replace the worker with an offline view without DDS |
| `POST /api/robot/switch` | JSON `product_id`; stop old session and select offline product view |
| `POST /api/pc/recover` | JSON `session_id`; retire the old worker and rebuild telemetry only for the same product/interface |
| `POST /api/pc/confirm-recovery` | JSON `session_id`, `confirmed: true`; explicit field confirmation ends read-only isolation and opens an offline view; missing telemetry does not block exit, but fresh movement evidence still does |

Existing health, snapshot, manifest, products, control-status and telemetry WebSocket endpoints are preserved. Manifest adds `runtime_mode=pc_sdk2` (`pc_readonly` for offline workers), `pc_session_id`, `physical_identity_verified=false` and `pc_session`. Telemetry adds `pc_session_id`; product entries add `pc_connect_supported` and `pc_direct_validation`. Connected workers expose fixed control, voice and perception endpoint allowlists. Only fixed RGB start/stop, camera status and RGB JPEG routes are exposed. Depth, custom sources and arbitrary topic configuration remain blocked. The manager validates Host/Origin and accepts JSON actions (empty joint-heartbeat bodies are supported).

States: `disconnected`, `waiting_telemetry`, `online`, `stale`, `network_unavailable`, `error`. Additional fields `worker_alive`, `worker_readonly`, `recovery_required`, `recovery_reason` and `arm_action_state` distinguish process liveness, recovery isolation and arm-task uncertainty. Each replacement gets a fresh session ID; the browser reloads all views and rejects old session data. All writes, including session management, require the current `X-Unirobo-PC-Session` header. Recovery and confirmation also require the current `session_id` in the body.

## G1, R1, GO2 and A2 SDK2 console update

Connected G1 workers reuse the existing Profile, telemetry, locomotion/audio adapters,
joint debugging/teaching policies and SLAM implementation. The manager proxies a fixed
list of existing control, voice and perception APIs. The manifest uses `runtime_mode=pc_sdk2`
for a connected worker; offline workers remain `pc_readonly`. PC verification starts
independently from robot-side verification. Business writes require the current
`X-Unirobo-PC-Session` header; stale pages are rejected.

R1 reuses its existing locomotion, head/joint debugging and teaching, firmware
arm-action and native audio SDK2 implementations. PC verification is independent
from robot-side verification, and real control remains operator-validation only.
R1 depth and undeclared SLAM stay disabled; the PC does not inspect `/dev/video-dep`, robot
services or an undeclared Mid-360 attachment. Fixed RGB reception does not switch robot video services. Kokoro is unavailable on PC; select native Unitree TTS.
Required robot SLAM services must already be running. Their state is queried via SDK2;
PC never automatically starts/stops them or uses its own processes as robot evidence.
Map RPC paths still refer to robot files. Downloads use SDK2 global map data or the
current user's Web map cache, never a local `/home/unitree/testN.pcd` fallback.
Teaching actions use the PC project's `config/joint_teach_actions.json`; customer configuration uses the working directory's `config/customer_voice.json`.

GO2 uses the existing `unitree_go` telemetry transport, V2 SportClient locomotion,
full-body joint policy and SDK2 audio adapter. PC verification remains independent
from robot-side verification. RGB uses the existing fixed receiver; attachment-dependent SLAM remains disabled;
recovery workers subscribe only to GO2 telemetry and construct no control clients.

A2 uses the existing HG LowState transport, A2 SportClient locomotion, full-body
joint policy and native A2 audio adapter. PC workers do not probe JT128 or start
`unitree_slam`; RGB uses the existing fixed receiver; JT128 and SLAM remain disabled without an explicit attachment declaration. Real A2 joint control
uses built-in experimental gains unless an operator-provided file overrides them, and still requires field validation.

Normal disconnection, product/interface changes and Ctrl+C require fresh stop evidence
and released control. Disconnect never issues stop commands. Recovery isolation provides
a bounded path when a worker exits, data remains stale or firmware-action completion
cannot be established. Mock, read-only hardware checks and operator control validation
remain separate acceptance levels.

Only G1/R1 Profile-declared crouched/passive FSM states 0/1/2/3/706 are exempt from odometry checks; A2/GO2 do not inherit these numbers. Other states require planar speed at most 0.03 m/s and yaw speed at most 0.05 rad/s; vertical velocity is excluded. These are disconnect tolerances, not motion limits. Rejection identifies the specific active request, motion, joint/teaching, SLAM task or resource ownership.

## Recovery after crashes, prolonged data loss or unknown firmware actions

Normal switching with complete stop evidence needs no additional confirmation.
Process exit and SDK initialization never establish that the robot stopped.

1. Open connection info beside the interface and choose **Recover read-only connection**. In-flight requests, active local motion/joint control, teaching holds, mapping/navigation and unresolved navigation confirmation still block retirement. End those tasks on site first. A dead worker needs no unavailable old-process evidence. A live control worker must respond, freeze its request/physical-remote entry points and pass a second local-ownership check. An unresponsive live control worker cannot establish that its publishers are inactive. A shutdown timeout keeps the old process reference and blocks every new worker; no force kill or second worker is used.
2. After the old process exits, a new `--read-only` worker subscribes on the same product/interface. It exposes no writable control, audio or SLAM capability and starts no teaching remote control. R1 recovery uses the existing LocoClient only to query fresh FSM state; its read-only Profile and HTTP backend still reject control writes. Old queues, velocity leases, holds and navigation are never restored. Manager and worker reject business writes; model/interface changes are blocked during isolation. Restore the cable and wait for telemetry. Manual read-only rebuilding remains available after initialization failure or another data loss; there is no automatic robot reconnection.
3. On-site personnel must confirm that the same selected G1/R1/GO2/A2 is safely stopped and all previous motion, actions, joint control, teaching holds, mapping and navigation have ended. Choose **Confirm field stop and disconnect** and accept the explanation. Missing/stale telemetry or an exited read-only worker does not block field confirmation; fresh telemetry showing movement still blocks exit. Telemetry cannot prove old actions or SLAM tasks ended, so this portion explicitly relies on field confirmation. No stop, release or mode command is sent. Confirmation ends isolation into an offline view.
4. Connect manually to create a new control session. No old command is replayed. Ctrl+C can close an isolated read-only worker without stop evidence because business writes are blocked; R1 keeps only its FSM query for stop checks. This does not confirm physical stop.

## Firmware arm-action observation limits

The [official arm-action interface](https://support.unitree.com/home/en/G1_developer/arm_action_interface), checked on November 12, 2026, describes integer preset calls as blocking and named firmware teaching calls as nonblocking. Error `7400` reports an executing action; `7401` reports a held pose. The pinned SDK2 `include/unitree/robot/g1/arm/g1_arm_action_client.hpp` also documents final-pose holds and an `rt/arm/action/state` JSON example with `id/name/holding`. The checked sources do not establish the topic's wire type, an execution sequence or a completion/release receipt correlated to this request. This implementation does not guess the wire type or enable that topic.

Connection info distinguishes `dispatching` (RPC in progress, physical progress unknown), `running_reported` / `holding_reported` (the last RPC explicitly reported execution/holding), `unknown` (completion/release unconfirmed), and `lost` (telemetry unavailable, ownership retained). A successful RPC, including manually requested action 99 / stop_custom, does not automatically establish physical completion/release. Duration metadata, elapsed time, FSM/task_id and stationary base odometry are not completion evidence. The resource manager retains real arm ownership to prevent competing joint/locomotion control.

Without a provable completion receipt, read-only isolation followed by field confirmation retires the local ownership; no extra action 99 is needed just to leave the session. `idle` means no arm request in this session, not proof that external tasks are absent. Mock has explicit simulated completion for ordinary actions and a persistent hold for ID 15; `completed_mock` never represents hardware completion. Automatic recognition of hardware natural completion/hold and the topic's type/timing still require read-only observation and field acceptance.

## A2 gains file

Real A2 control sessions on PC default to experimental gains compiled into the binary: `kp=60, kd=5`, with `kp=0, kd=1` for recording, for all 12 joints. A new PC needs no separate gains file or launch argument after compilation. These are previously selected GO2 reference values, not official or validated A2 gains. Experimental provenance remains in configuration metadata and this guide; the debugger shows no gains banner. Field evaluation and operator validation remain required. Enter Low-level Mode checks fresh state, issues StandDown when needed, and confirms sustained STAND_DOWN with settled joints before disabling `ai_sport`. Failure, stale feedback or timeout leaves motion control enabled. Exit Low-level Mode releases LowCmd and restores `ai_sport` without standing up.

Override them with `bash scripts/pc.sh start --a2-joint-gains /absolute/path/to/a2-gains.json`. An explicit file takes priority; invalid files disable joint debugging/teaching without falling back. The existing A2 parser and safety interlocks apply. Mock retains zero default gains. Idle and read-only recovery workers never load gains or create joint control. Robot-side startup still requires an explicit gains file.

A2 low-level entry allows up to 8 seconds for its single StandDown RPC, then restores the ordinary 1-second motion RPC timeout. An unresolved 3104 timeout is reported as `stand_down_rpc_timeout`; it does not retry or release ai_sport. Fresh crouch and joint-stability confirmation still follows a successful RPC.
