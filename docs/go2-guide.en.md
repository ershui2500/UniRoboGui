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

## 2. Defaults for a new deployment

With source containing this update, `--product go2`:

- Verifies and reuses compatible SDK2 under `/usr/local`; does not overwrite the SDK. Default DDS interface: `eth0`, subject to on-site verification.
- Prepares `kokoro-multi-lang-v1_1.tar.bz2`, verifies `tts_models/kokoro-multi-lang-v1_1/model.onnx` and supporting assets, and performs silent synthesis checks.
- Installs the exact `go2-local-tts.service` and GO2 Web unit, with system/user templates. Persistent user installation requires Linger=yes.
- Does not install RealSense or a privileged camera helper, declare optional LiDAR attachments, or enable real navigation by default.
- Preserves runtime `config/`, `build/`, `tts_models/` and `tts_kokoro/`; the product resource step transfers model archives separately. Online/PC deployment refuses dirty source instead of overwriting it.

The model and service must both be installed. Old GitHub revisions, older installers, or copying Web files alone do not upgrade TTS. Unpublished local changes are not fetched automatically by online deployment. Explicit `--without-kokoro` skips local TTS setup; a new GO2 deployment then has no native fallback.

## 3. Deployment

Confirm development-capable hardware, firmware, interface, SDK and reliable system time first. An invalid RTC can reset time after an offline reboot. Correct it from a trusted PC and persist the time baseline before development/deployment; do not use `hwclock -s` to replace correct time or source ROS environments.

On the robot:

```bash
cd /home/unitree/UniRoboGui
bash scripts/deploy.sh check --product go2
# Requires published updated source, clean checkout and access to GitHub/PyPI
bash scripts/deploy.sh install --product go2
```

On an online development PC when the robot cannot access the Internet:

```bash
export http_proxy=http://127.0.0.1:7890
export https_proxy=http://127.0.0.1:7890
export no_proxy=localhost,127.0.0.1,192.168.123.0/24
bash scripts/deploy.sh from-pc --product go2 --host unirobo-go2
```

The proxy belongs to the PC; the robot's localhost is not the PC. `unirobo-go2` is this development environment's SSH alias. Another PC can specify `--host unitree@<actual-expansion-computer-address>`. Do not use the motion-controller address or put passwords in chat/arguments. The PC prepares the model and aarch64/Python 3.8 wheelhouse. Robot HTTP and DDS remain direct.

To prepare only TTS resources:

```bash
bash scripts/install_kokoro_tts.sh --product go2
bash scripts/install_kokoro_tts.sh --product go2 --help
```

The resource installer does not install/restart units or the Web server. Help performs no installation. The GO2 FP32 directory must not contain `model.int8.onnx`; renaming weights does not change precision. Use the shared deployment entry for full installation.

## 4. Web and acceptance

Open port 8080, select Voice → TTS. Keep the current volume; announce the test text/volume, then test short, long, unpunctuated, numeric/mixed-language and consecutive utterances. Confirm the final words were actually heard without gaps. Expansion-board audio or `paplay` does not prove body-speaker playback.

```bash
curl --noproxy '*' -fsS http://127.0.0.1:8765/health
curl --noproxy '*' -fsS http://127.0.0.1:8080/api/voice/status
curl --noproxy '*' -fsS http://127.0.0.1:8080/api/robot/manifest
curl --noproxy '*' -fsS http://127.0.0.1:8080/api/control/status
systemctl --user show go2-local-tts.service -p ExecStart -p ActiveState
```

Use systemctl without `--user` for a system installation. Confirm `kokoro-multi-lang-v1_1 --sid 3 --speed 1.2 --threads 8`, `model.onnx`, GO2 identity and no voice initialization error. Unconfigured LLM/optional offline sources can make overall health degraded without TTS being broken.

GO2 has no reliable end-of-play event. “Speech request processed; please confirm the actual sound” separates protocol success from listening acceptance. Request duration includes synthesis, upload, playback and protective waiting.

## 5. Measured result and troubleshooting

One on-site run reduced the same request from 104.33 s to 23.14 s; FP32 synthesis of short Chinese replies took about 0.64–0.76 s. The operator confirmed short/long/unpunctuated/numeric/mixed/consecutive requests and the actual Web button output were complete and clear, without obvious gaps. These are recorded samples, not guarantees for every robot/text. See the [detailed diagnostic/evaluation record](voice/GO2-TTS-诊断与离线方案评估.md).

- Silence/`local_tts_silent_pcm`: check for the old INT8 model or script. The previous INT8 configuration generated NaNs for some input; new code rejects NaN/Inf instead of turning it into silence.
- `local_tts_synthesis_transfer_failed_*`: inspect TTS logs/assets. HTTP 200 headers alone do not prove a complete stream.
- AudioHub enter/upload/exit failures retain the stage, block and API code. Do not substitute G1/R1 topics or LowCmd.
- Volume 3104 is a timeout; check current read-only state before treating an old startup warning as a current failure.
- Before restarting, confirm exact unit/PID/product and stopped/zero motion. Do not issue movement APIs to help stop it.

Melo/Matcha remain evaluation candidates. G1/R1 default models were not changed by this GO2 work.
