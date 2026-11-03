# GO2 deployment and Web guide

[简体中文](go2-guide.md) · [Shared deployment guide](deployment-dependencies.en.md)

Applies to the registered development-capable GO2 variant: `product_id=go2`, internal `variant=go2_edu`. The display name Unitree GO2 does not change protocol identifiers. Confirm the actual model, firmware, SDK and attachments before use.

## 1. Implemented features and limits

| Feature | Implementation and boundary |
|---|---|
| Telemetry and pose | SDK2 LowState/SportModeState, 12 joints, embedded BMS, odometry, official URDF; stale/default data must not appear live |
| Diagnostics | Preserve unknown/raw fields; no error-list sample is not proof of fault-free operation. Motion-state IDs are not fault codes |
| RGB | Controlled native RTP/H264 source `udp://230.1.1.1:1720`; no browser-supplied pipelines |
| Depth | Unsupported in the base configuration; its panel is hidden |
| Motion | Product-specific speed/mode/action allowlists, limits, state checks, leases and confirmation; automated regression uses Mock, physical acceptance belongs to the operator |
| Joint debug/teach | Full body only, 12 active joints in a 20-slot message; slots 12–19 reserved. Guarded 500 Hz LowCmd, no automatic takeover. Teaching removes support and requires secure suspension/support plus explicit authorization |
| Teach remote bindings | Shared storage and playback; GO2 shows F2 without changing shared persistence identifiers |
| TTS/volume | Local offline Kokoro FP32 + sherpa-onnx, sid=3, speed=1.2, total 8 inference threads; complete-utterance AudioHub playback and VUI volume |
| ASR/native assistant | Not integrated; no native TTS or built-in LLM fallback |
| Customer LLM | Shared text/API configuration; requires a configured compatible API. ASR wake is unavailable. Offline TTS does not imply an offline LLM |
| LiDAR/SLAM | Read-only L2 sources are separate from SLAM. Mapping/navigation require the dock, Mid-360 or XT16, compatible firmware/services and explicit attachment declarations. Real navigation is off by default |

Runtime Manifest/Capability is authoritative. One audible test does not automatically raise Profile verification levels. The page lock prevents accidental actions; it provides no authentication.

Common modes show Stand up, Free walk, Stop move and Lie down together. Free walk is available directly after Stand up; the Balance stand entry is removed.

StaticWalk, TrotRun and EconomicGait are not verified as independently selectable modes on the current deployment and have been removed from the Web and backend command allowlist. A zero API return code does not confirm the target state.

The shared binary compiles all registered products. Even for GO2, the existing SDK must include the R1 Audio/Loco headers required by this project. If missing, have a maintainer provide a complete firmware/ABI-compatible SDK rather than removing product code.

## 2. Defaults for a new deployment

With source containing this update, `--product go2`:

- Verifies and reuses compatible SDK2 under `/usr/local`; does not overwrite the SDK. Default DDS interface: `eth0`, subject to on-site verification.
- Prepares `kokoro-multi-lang-v1_1.tar.bz2`, verifies `tts_models/kokoro-multi-lang-v1_1/model.onnx` and supporting assets, and performs silent synthesis checks.
- Installs the exact `go2-local-tts.service` and GO2 Web unit, with system/user templates. Persistent user installation requires Linger=yes.
- Does not install RealSense or a privileged camera helper, declare optional LiDAR attachments, or enable real navigation by default.
- Preserves runtime `config/`, `build/`, `tts_models/` and `tts_kokoro/`; the product resource step transfers model archives separately. Online/PC deployment refuses dirty source instead of overwriting it.

The model and service must both be installed. Old GitHub revisions, older installers, or copying Web files alone do not upgrade TTS. Unpublished local changes are not fetched automatically by online deployment. Explicit `--without-kokoro` skips local TTS setup; a new GO2 deployment then has no native fallback.

## 3. Deployment

### 3.1 Prerequisites and clock

Use a development-capable GO2 expansion computer running Ubuntu 20.04 AArch64 as `unitree`, with a compatible SDK2 already installed under `/usr/local`. The default SSH target is `unitree@192.168.123.18` (expansion computer). Verify the actual address and wired DDS interface; override only if different, and do not use a motion-controller address. Do not source ROS environments.

Run `ip -br -4 addr` on the robot. The repository defaults to `eth0`. If the actual DDS interface differs, have a maintainer align `scripts/deploy_product.sh` and both `deploy/go2-web-control{,.user}.service` templates before installation. The deployment CLI has no `--interface` option.

First complete PC tool setup and clone the project as described in [the shared prerequisites](deployment-dependencies.en.md#1-recommended-entry-point). A robot terminal is an SSH session such as `ssh unitree@192.168.123.18`; a PC terminal is your local Linux shell. Keep their commands separate.

An offline boot or invalid RTC may reset system time. Before each deployment, confirm the PC clock is reliable, then run these commands on the PC to correct the robot clock and save the systemd-timesyncd baseline. Configure existing SSH keys/agent and verify the host fingerprint first; this step requires passwordless remote sudo:

```bash
(
set -e
GO2_HOST='unitree@192.168.123.18' # Change only if the actual address differs
ssh -o BatchMode=yes "$GO2_HOST" 'id -un; uname -m'
pc_epoch=$(date +%s)
ssh -o BatchMode=yes "$GO2_HOST" \
  "sudo -n date -u -s '@${pc_epoch}' && sudo -n touch /var/lib/systemd/timesync/clock"
pc_epoch=$(date +%s)
go2_epoch=$(ssh -o BatchMode=yes "$GO2_HOST" 'date +%s')
delta=$((pc_epoch - go2_epoch))
if ((delta < 0)); then delta=$((-delta)); fi
[[ "$delta" -le 5 ]]
echo 'GO2 clock OK'
)
```

Proceed only after identity is `unitree`, architecture is `aarch64`, both clock operations succeed, and the difference is at most 5 seconds. Stop if any step fails. Have the on-site maintainer resolve authentication through a controlled flow; never put passwords in chat/arguments. Do not run `hwclock -s` or replace time services. The installer does not perform this clock correction automatically. GO2 `from-pc` checks clock difference ≤5 seconds and a persistent baseline updated within the last 300 seconds. Correct the clock within five minutes before running `from-pc`; this window is deployment policy, not a firmware requirement.

### 3.2 Method A: robot has Internet access

Use a published revision containing GO2 support. On a new robot installation:

```bash
git clone https://github.com/ershui2500/UniRoboGui.git /home/unitree/UniRoboGui
cd /home/unitree/UniRoboGui
bash scripts/deploy.sh check --product go2 --lang en
bash scripts/deploy.sh install --product go2 --lang en
```

For an existing directory, skip clone, inspect `git status --short` and preserve local changes before check/install. The robot needs GitHub, model-host and PyPI access, plus an Ubuntu package source if system packages are missing. Installation performs Release/complete CTest/ldd checks, installs the selected Web/TTS service scope, and runs read-only acceptance. Unpublished local changes cannot be fetched from an older remote revision.

### 3.3 Method B: transfer from an online PC

On an Ubuntu/Linux PC with GitHub/model/PyPI access and SSH reachability to GO2, clone first. Skip clone if the directory already exists and inspect `git status --short`; do not overwrite it. After entering the project directory, run the clock procedure in §3.1 on this PC, then run `from-pc` within five minutes:

```bash
git clone https://github.com/ershui2500/UniRoboGui.git
cd UniRoboGui
# Complete the PC clock procedure in section 3.1 now, then run:
bash scripts/deploy.sh from-pc --product go2 --lang en
```

GO2 defaults to `unitree@192.168.123.18`; no `--host` or development alias is needed. If the actual address differs, override it with `bash scripts/deploy.sh from-pc --product go2 --host unitree@<GO2_IP> --lang en`, or use an existing SSH Host alias. The script updates the PC's clean checkout, prepares the FP32 model and AArch64/Python 3.8 wheelhouse, and transfers them for robot-side build/install. This is not an offline OS image: missing Ubuntu 20.04 arm64 system packages still need a compatible source or prior installation. Do not substitute x86_64 wheels/packages.

If the PC needs a proxy, configure its real proxy address only on the PC. Include localhost, 127.0.0.1 and the actual robot address in `no_proxy`. Robot localhost is not the PC's proxy; do not repurpose the DDS interface for Internet access.

### 3.4 TTS resources only

In the robot terminal, from `/home/unitree/UniRoboGui`, prepare only TTS resources:

```bash
bash scripts/install_kokoro_tts.sh --product go2
bash scripts/install_kokoro_tts.sh --product go2 --help
```

The resource installer does not install/restart units or the Web server. Help performs no installation. The GO2 FP32 directory must not contain `model.int8.onnx`; renaming weights does not change precision. Use the shared deployment entry for full installation.

## 4. Web and acceptance

Open `http://192.168.123.18:8080` in the PC browser (use the actual robot address if different), then select Voice → TTS. Keep the current volume; announce the test text/volume, then test short, long, unpunctuated, numeric/mixed-language and consecutive utterances. Confirm the final words were actually heard without gaps. Expansion-board audio or `paplay` does not prove body-speaker playback.

Run the following in the **robot terminal**; `127.0.0.1` refers to the robot, not the PC.

```bash
curl --noproxy '*' -fsS http://127.0.0.1:8080/api/health
curl --noproxy '*' -fsS http://127.0.0.1:8765/health
curl --noproxy '*' -fsS http://127.0.0.1:8080/api/voice/status
curl --noproxy '*' -fsS http://127.0.0.1:8080/api/robot/manifest
curl --noproxy '*' -fsS http://127.0.0.1:8080/api/control/status
systemctl --user show go2-local-tts.service -p ExecStart -p ActiveState
```

Use systemctl without `--user` for a system installation. Confirm `kokoro-multi-lang-v1_1 --sid 3 --speed 1.2 --threads 8`, `model.onnx`, GO2 identity and no voice initialization error. Unconfigured LLM/optional offline sources can make overall health degraded without TTS being broken.

GO2 has no reliable end-of-play event. “Speech request processed; please confirm the actual sound” separates protocol success from listening acceptance. Request duration includes synthesis, upload, playback and protective waiting.

## 5. Upgrades, maintenance and troubleshooting

For daily startup of installed services, run `bash scripts/start_services.sh --product go2` on the robot. This requires an explicit product and starts only its installed exact units; it does not install or restart running services.

Reuse Method A/B for upgrades. Preserve `config/`, `build/`, `tts_models/` and `tts_kokoro/`. Back up customer configuration to a controlled location and keep keys out of Git/public attachments. Have a maintainer merge local source changes instead of bypassing checks with `reset --hard`. On failure, retain logs and inspect the exact service state. For rollback, restore a previously validated revision and rebuild/verify through the same flow; do not overwrite a running binary.

Read-only service/log inspection follows. Add `--user` to each systemctl/journalctl command for a user installation:

```bash
systemctl show go2-web-control.service -p MainPID -p ExecStart -p ActiveState -p SubState
systemctl show go2-local-tts.service -p ExecStart -p ActiveState
journalctl -u go2-web-control.service -n 80 --no-pager
journalctl -u go2-local-tts.service -n 80 --no-pager
ss -ltnp 'sport = :8080'
```

Before an on-site operator restarts anything, verify the GO2 unit/MainPID/interface, `motion.active=false`, `state=stopped`, `vx/vy/vyaw=0`, and no active debug/teaching ownership. Use `sudo systemctl restart go2-web-control.service` for system scope or `systemctl --user restart go2-web-control.service` for user scope. Then verify a new MainPID, single port-8080 listener, health, GO2 Manifest, DDS sources and stopped/zero motion. Never call motion APIs to help stop it or use pkill/killall.

- Silence/`local_tts_silent_pcm`: check for the old INT8 model or script. The previous INT8 configuration generated NaNs for some input; new code rejects NaN/Inf instead of turning it into silence.
- `local_tts_synthesis_transfer_failed_*`: inspect TTS logs/assets. HTTP 200 headers alone do not prove a complete stream.
- AudioHub enter/upload/exit failures retain the stage, block and API code. Do not substitute G1/R1 topics or LowCmd.
- Volume 3104 is a timeout; check current read-only state before treating an old startup warning as a current failure.
- Before restarting, confirm exact unit/PID/product and stopped/zero motion. Do not issue movement APIs to help stop it.

Melo/Matcha remain evaluation candidates. G1/R1 default models were not changed by this GO2 work.
