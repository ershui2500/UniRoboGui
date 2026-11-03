# UniRoboGui G1 / R1 / GO2 通用部署与排障指南

**简体中文** | [English](deployment-dependencies.en.md)

本文件同时覆盖 Unitree G1、R1 与 GO2。部署入口、公共构建流程和安全检查保持一致；网卡、SDK、service、相机、TTS 和附件差异集中在产品配置中，不维护两套重复教程。

## 1. 唯一推荐入口

先选实际机型和一种部署方式；以下命令是不同选择，不是需要整段粘贴的执行顺序。`install` / `check` 在机器人终端运行，`from-pc` 在联网电脑终端运行，均先进入已 clone 的项目根目录。

Ubuntu 机器执行 clone 前先准备 Git。方式 B 的电脑需要 `git curl openssh-client rsync python3 python3-pip`，可运行 `command -v git curl ssh rsync python3` 与 `python3 -m pip --version` 检查，缺失时先安装电脑端软件包。机器人缺失的系统包必须是 Ubuntu 20.04 arm64 版本，不能复制电脑的 x86_64 包；`from-pc` 不打包整个 Ubuntu 软件源。


~~~bash
# 机器人终端：安装或升级，只选实际产品的一条
bash scripts/deploy.sh install --product g1
bash scripts/deploy.sh install --product r1
bash scripts/deploy.sh install --product go2

# 联网电脑终端：传输并部署，只选实际产品的一条
bash scripts/deploy.sh from-pc --product g1
bash scripts/deploy.sh from-pc --product r1
bash scripts/deploy.sh from-pc --product go2

# 机器人终端：只读检查，只选实际产品的一条
bash scripts/deploy.sh check --product g1
bash scripts/deploy.sh check --product r1
bash scripts/deploy.sh check --product go2

# 使用英文部署日志
bash scripts/deploy.sh install --product g1 --lang en
~~~

`--product` 必填，接受当前部署配置支持的 `g1`、`r1` 或 `go2`。`--host` 只用于 `from-pc`，三产品均为可选参数；默认地址见下表。仅在现场地址不同时覆盖，也可填写已有 SSH Host 别名。旧 `deploy_g1_*` / `install_g1*` 中英文脚本仅保留一个兼容周期，打印弃用提示后转发到 `--product g1`；旧 `--robot USER@HOST` 会转换为 `--host`。

## 2. 产品矩阵

以下是仓库默认部署配置；网卡、附件和可用能力还需现场确认。

| 项目 | G1 | R1 | GO2 |
| --- | --- | --- | --- |
| 默认 SSH 目标 | `unitree@192.168.123.164` | `unitree@192.168.123.164` | `unitree@192.168.123.18` |
| DDS 网卡 | `eth0` | `eth10` | `eth0`（先核对现场） |
| SDK / CMake 前缀 | `/opt/unitree_robotics`；用户安装可用 `/home/unitree/.local/unitree_robotics` | `/usr/local`，已有安装只校验，不自动覆盖 | `/usr/local`，仅校验已有兼容 SDK，不覆盖 |
| 动态库路径 | SDK `prefix/lib` + `/usr/local/lib`；需要时兼容已有 librealsense | `/usr/local/lib` | `/usr/local/lib` |
| Web service | `g1-web-control.service` | `r1-web-control.service` | `go2-web-control.service` |
| systemd user unit | `deploy/g1-web-control.user.service` | `deploy/r1-web-control.user.service` | `deploy/go2-web-control.user.service` |
| 相机 | D435i + librealsense + `g1-web-first-person-service` | R1 EDU 固定 RGB/Depth + `r1-web-camera-service` | 固定原生 RGB；深度不支持；无特权 helper |
| TTS 默认 | Kokoro + Unitree fallback | Unitree native TTS | Kokoro FP32；无原生回退 |
| SDK 安装策略 | 缺失时可从官方 SDK2 源码安装 | 校验现有 R1 头文件、CMake 配置和 CycloneDDS 动态库；缺失即停止 | `/usr/local`，仅校验已有兼容 SDK，不覆盖 |
| Mid-360 / navigation | 保持现有 G1 配置与 service 默认值 | 默认不声明 Mid-360，不启用真实导航 | 默认不声明可选雷达，不启用真实导航 |
| 二进制 | `build/g1_web_server --robot g1` | `build/g1_web_server --robot r1` | `build/g1_web_server --robot go2` |

三种产品都使用同一代码库和同一个 `g1_web_server` 历史二进制名。产品身份由 `--robot`、`RobotRegistry`、Manifest 和 Capability 决定，不由二进制文件名决定。

同一二进制会编译所有已注册产品，因此 GO2 的现有 SDK 也必须包含项目需要的 R1 Audio/Loco 头文件；缺失时应由维护人员提供匹配固件/ABI 的完整 SDK，不通过删掉产品代码绕过。

## 3. 公共部署流程

启用本地 Kokoro 的产品还会在编译前检查 Python 初始化条件：已有可用项目虚拟环境、离线 wheelhouse 中的 pip wheel、系统 ensurepip 任一可用即可复用。三者都缺少时，系统级安装按需准备 `python3-venv`；用户级安装提前停止，提示提供含 pip wheel 的 `KOKORO_WHEELHOUSE` 或由管理员先安装 `python3-venv`。因此 `python3-venv` 不是已具备离线 pip wheel 时的强制系统依赖。

部署脚本按同一流程执行：

1. 检查 Ubuntu 20.04 AArch64、`unitree` 用户、目标 DDS 网卡和 ROS/RMW/CycloneDDS 环境污染；
2. 检查项目源码脏状态，发现未提交业务代码时停止而不是覆盖；
3. 根据产品配置检查或准备 SDK、相机依赖和 TTS 资源；
4. Release 构建并运行完整 CTest；
5. 使用 `ldd` 检查无 `not found`，并确认 CycloneDDS 来自所选产品 SDK 前缀；
6. 在替换正在运行的 Web service 前读取 Manifest 与 `/api/control/status`；
7. 只有确认产品身份匹配、`motion.active=false`、`state=stopped` 且 `vx/vy/vyaw` 全零才允许停止或替换该 service；
8. 如果发现另一产品 service 正在运行，直接停止部署，不自动关闭或切换；
9. 安装精确的 system 或 user unit；仅对产品声明的相机安装 helper/sudoers，GO2不需要；
10. 启动后只做只读验收：8080 唯一监听、health、Manifest、DDS 必需 source、零运动状态、MainPID 参数、进程环境和 helper 状态。

自动验收不会调用真实 `SetVelocity`、`StopMove`、`Start`、`StandUp`、`SetFsmId`、关节、头部、模式切换或导航命令。真实运动验收由现场操作员通过 Web 执行。

## 4. 产品部署说明

### G1 部署说明

G1 保持现有基线：

- `eth0` 用于 SDK2 DDS，`wlan0` 用于 Web/外网；
- SDK 系统前缀为 `/opt/unitree_robotics`，必要时可使用持久用户前缀；
- D435i 需要 librealsense；系统级安装会提供受控 `g1-web-first-person-service` helper；
- Kokoro 是默认本地 TTS，Unitree TTS 仍作为 fallback；
- `g1-web-control.service` 显式包含 `--robot g1`，并保持现有 `--enable-navigation` 默认值；
- `from-pc` 会准备 SDK2、librealsense、Kokoro 模型与 AArch64/Python 3.8 wheelhouse。

### R1 部署说明

R1 使用以下边界：

- DDS 网卡固定为 `eth10`；
- SDK/CMake 前缀固定为 `/usr/local`；
- 部署脚本只校验 R1 Audio/Loco 必需头文件、`unitree_sdk2` CMake 配置以及 `libddsc` / `libddscxx` 动态库，缺失时停止并提示，不自动覆盖 `/usr/local`；
- 默认使用 Unitree 原生 TTS，不安装 Kokoro；
- 系统级安装使用 `r1-web-camera-service`；用户级安装无 helper 时相关 Camera Capability 按运行时证据降级；
- `r1-web-control.service` 显式包含 `--robot r1`，不带 `--device mid360`，也不带 `--enable-navigation`；
- `from-pc` 默认连接 `unitree@192.168.123.164`；OpenSSH 会先尝试已有 key/agent，首次连接可在交互终端确认主机指纹，确有需要时再由 SSH 自己提示输入密码；脚本不保存或回显密码。现场地址不同或已有 SSH Host 别名时可通过 `--host` 显式覆盖。

### GO2 部署说明

- 按 [GO2 教程](go2-guide.md#3-部署步骤) 先确认二次开发型号、可信系统时间与实际有线 DDS 网卡。仓库默认 `eth0`，部署 CLI 没有 `--interface` 参数。
- 复用兼容的 `/usr/local` SDK；缺少 Go2 头文件、CMake 配置或 DDS 动态库时停止，不自动覆盖。
- 在选定 system/user 范围安装 `go2-web-control.service` 和 `go2-local-tts.service`。默认 FP32 `kokoro-multi-lang-v1_1/model.onnx`，不使用 G1 INT8 模型。
- 默认不装 RealSense/相机 helper，不声明可选雷达/真实导航；深度及 ASR/原生TTS/内置LLM 不可用。
- 电脑准备模型与 AArch64/Python3.8 wheelhouse，但不会打包整个 Ubuntu 软件源。缺失的 arm64 系统包仍需兼容来源。

## 5. from-pc 传输边界

`from-pc` 使用 rsync 传输项目时：

- 不传 `.git`；
- 不传 `AGENTS.md`、`*_AGENTS.md`、`.robot-workspace`、`.robot-backups`、`.robot-sync`、本地 G1/R1/GO2 参考资料及 `docs/GO2-*`、`docs/voice/` 内部现场记录；
- 保留机器人整个 `config/`、`build/`、`tts_models/` 和 `tts_kokoro/` 运行目录；
- 如果机器人项目 Git checkout 有未提交修改，停止部署；
- 项目同步不使用 `--delete`，保留现场额外文件；版本已删除的旧源码由维护人员审阅后单独清理。SDK/RealSense 上游源码目录采用先备份后镜像的独立流程；
- 不把 SSH、sudo、Wi-Fi、API 密钥或其他凭据写入项目、参数、日志或缓存。

## 6. 只读 check

在机器人终端执行下面与实际产品对应的一条命令：

~~~bash
bash scripts/deploy.sh check --product g1
bash scripts/deploy.sh check --product r1
bash scripts/deploy.sh check --product go2
~~~

`check` 不安装软件包、不修改文件、不停止或启动 service。它检查系统、架构、目标网卡、所需包、SDK 状态、相机 helper 状态等。R1 / GO2 `check` 直接使用 `/usr/local` 进行 SDK 校验。检查通过不等于运行或运动验收通过。

## 7. 部署后验收

脚本会自动完成核心只读验收。人工复核在机器人终端执行下列命令；这里的 `127.0.0.1` 是机器人本机：

~~~bash
curl --noproxy '*' -fsS http://127.0.0.1:8080/api/health
curl --noproxy '*' -fsS http://127.0.0.1:8080/api/robot/manifest
curl --noproxy '*' -fsS http://127.0.0.1:8080/api/control/status
ss -ltnp 'sport = :8080'
ldd /home/unitree/UniRoboGui/build/g1_web_server | grep -E 'ddsc|ddscxx|realsense|not found'
~~~

验收应确认：

- Manifest `identity.product_id` 与部署产品一致；
- `motion.active=false`、`state=stopped`、`vx/vy/vyaw` 为零；
- health 中 DDS 已初始化，Manifest 声明的 `required_sources` 为 online；
- 8080 只有目标 Web 进程监听；
- MainPID 命令行包含正确 `--robot` 与 `--interface`；
- Web 进程没有 `ROS_DISTRO`、`RMW_IMPLEMENTATION`、`CYCLONEDDS_URI`、`AMENT_PREFIX_PATH`、`COLCON_PREFIX_PATH`；
- G1/R1/GO2 CycloneDDS 动态库解析到所选产品 SDK 前缀；
- 相机 helper 的只读状态检查返回正常 active/inactive 语义。

## 8. 常见故障

### GitHub 不可达

机器人不能访问 GitHub 时不要反复重试，也不要修改 DDS 网卡。改用联网电脑：

~~~bash
bash scripts/deploy.sh from-pc --product g1
bash scripts/deploy.sh from-pc --product r1
bash scripts/deploy.sh from-pc --product go2
~~~

### R1 SDK 校验失败

如果缺少 `/usr/local` 下的 R1 头文件、`unitree_sdk2Config.cmake`、`libddsc` 或 `libddscxx`，部署会停止。不要让通用部署器自动重装 `/usr/local`；应先按机器人固件/ABI 要求恢复兼容 SDK。

### `undefined symbol: ddsi_sertype_v0`

通常表示混入了另一套 CycloneDDS/ROS 环境。重新建立干净 SSH 会话，选择 `none`，不要 source Foxy、Noetic、ROS 2 或自建 RMW 环境，然后重新执行 `check`。

### 8080 被占用

不要 `pkill` / `killall`。先确认精确 unit、MainPID、Manifest 产品身份和零运动状态，再处理对应 service：

~~~bash
ss -ltnp 'sport = :8080'
systemctl --no-pager --full status g1-web-control.service
systemctl --no-pager --full status r1-web-control.service
systemctl --no-pager --full status go2-web-control.service
~~~

### 相机

G1 先检查 librealsense、D435i USB 枚举和 `g1-web-first-person-service`。R1 检查固定 RTP/Depth source 与 `r1-web-camera-service`。GO2 检查固定 RGB 与 DDS 网卡；深度不支持，也不要求 helper。浏览器不能覆盖受控 source、端口、pipeline、service 名或 helper 命令。

## 9. 扩展新产品

通用部署已登记 `g1`、`r1` 与 `go2`。后续产品通过新增一个受控产品配置分支、对应 system/user service、helper/依赖声明和验收项接入；不要复制整套部署脚本，也不要引入 YAML、插件框架或新的部署依赖，除非现有 Bash `case` 模式已无法表达需求。
