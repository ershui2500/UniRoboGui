# UniRoboGui G1 / R1 通用部署与排障指南

**简体中文** | [English](deployment-dependencies.en.md)

本文件同时覆盖 Unitree G1 与 R1。部署入口、公共构建流程和安全检查保持一致；网卡、SDK、service、相机、TTS 和附件差异集中在产品配置中，不维护两套重复教程。

## 1. 唯一推荐入口

~~~bash
# 机器人可联网：安装或升级
bash scripts/deploy.sh install --product g1
bash scripts/deploy.sh install --product r1

# 从联网电脑传输并部署
bash scripts/deploy.sh from-pc --product r1

# 只读环境检查
bash scripts/deploy.sh check --product g1
bash scripts/deploy.sh check --product r1

# 使用英文部署日志
bash scripts/deploy.sh install --product g1 --lang en
~~~

`--product` 必填，只接受 Registry 当前支持的 `g1` 或 `r1`。`--host` 只用于 `from-pc`。旧 `deploy_g1_*` / `install_g1*` 中英文脚本仅保留一个兼容周期，打印弃用提示后转发到 `--product g1`；旧 `--robot USER@HOST` 会转换为 `--host`。

## 2. 产品矩阵

| 项目 | G1 | R1 |
| --- | --- | --- |
| DDS 网卡 | `eth0` | `eth10` |
| SDK / CMake 前缀 | `/opt/unitree_robotics`；用户安装可用 `/home/unitree/.local/unitree_robotics` | `/usr/local`，已有安装只校验，不自动覆盖 |
| 动态库路径 | SDK `prefix/lib` + `/usr/local/lib`；需要时兼容已有 librealsense | `/usr/local/lib` |
| Web service | `g1-web-control.service` | `r1-web-control.service` |
| systemd user unit | `deploy/g1-web-control.user.service` | `deploy/r1-web-control.user.service` |
| 相机 | D435i + librealsense + `g1-web-first-person-service` | R1 EDU 固定 RGB/Depth + `r1-web-camera-service` |
| TTS 默认 | Kokoro + Unitree fallback | Unitree native TTS |
| SDK 安装策略 | 缺失时可从官方 SDK2 源码安装 | 校验现有 R1 头文件、CMake 配置和 CycloneDDS 动态库；缺失即停止 |
| Mid-360 / navigation | 保持现有 G1 配置与 service 默认值 | 默认不声明 Mid-360，不启用真实导航 |
| 二进制 | `build/g1_web_server --robot g1` | `build/g1_web_server --robot r1` |

两种产品都使用同一代码库和同一个 `g1_web_server` 历史二进制名。产品身份由 `--robot`、`RobotRegistry`、Manifest 和 Capability 决定，不由二进制文件名决定。

## 3. 公共部署流程

部署脚本按同一流程执行：

1. 检查 Ubuntu 20.04 AArch64、`unitree` 用户、目标 DDS 网卡和 ROS/RMW/CycloneDDS 环境污染；
2. 检查项目源码脏状态，发现未提交业务代码时停止而不是覆盖；
3. 根据产品配置检查或准备 SDK、相机依赖和 TTS 资源；
4. Release 构建并运行完整 CTest；
5. 使用 `ldd` 检查无 `not found`，并确认 CycloneDDS 来自所选产品 SDK 前缀；
6. 在替换正在运行的 Web service 前读取 Manifest 与 `/api/control/status`；
7. 只有确认产品身份匹配、`motion.active=false`、`state=stopped` 且 `vx/vy/vyaw` 全零才允许停止或替换该 service；
8. 如果发现另一产品 service 正在运行，直接停止部署，不自动关闭或切换；
9. 安装精确的 system 或 user unit、产品相机 helper/sudoers；
10. 启动后只做只读验收：8080 唯一监听、health、Manifest、DDS 必需 source、零运动状态、MainPID 参数、进程环境和 helper 状态。

自动验收不会调用真实 `SetVelocity`、`StopMove`、`Start`、`StandUp`、`SetFsmId`、关节、头部、模式切换或导航命令。真实运动验收由现场操作员通过 Web 执行。

## 4. G1 部署说明

G1 保持现有基线：

- `eth0` 用于 SDK2 DDS，`wlan0` 用于 Web/外网；
- SDK 系统前缀为 `/opt/unitree_robotics`，必要时可使用持久用户前缀；
- D435i 需要 librealsense；系统级安装会提供受控 `g1-web-first-person-service` helper；
- Kokoro 是默认本地 TTS，Unitree TTS 仍作为 fallback；
- `g1-web-control.service` 显式包含 `--robot g1`，并保持现有 `--enable-navigation` 默认值；
- `from-pc` 会准备 SDK2、librealsense、Kokoro 模型与 AArch64/Python 3.8 wheelhouse。

## 5. R1 部署说明

R1 使用以下边界：

- DDS 网卡固定为 `eth10`；
- SDK/CMake 前缀固定为 `/usr/local`；
- 部署脚本只校验 R1 Audio/Loco 必需头文件、`unitree_sdk2` CMake 配置以及 `libddsc` / `libddscxx` 动态库，缺失时停止并提示，不自动覆盖 `/usr/local`；
- 默认使用 Unitree 原生 TTS，不安装 Kokoro；
- 系统级安装使用 `r1-web-camera-service`；用户级安装无 helper 时相关 Camera Capability 按运行时证据降级；
- `r1-web-control.service` 显式包含 `--robot r1`，不带 `--device mid360`，也不带 `--enable-navigation`；
- `from-pc` 默认连接 `unitree@192.168.123.164`；OpenSSH 会先尝试已有 key/agent，首次连接可在交互终端确认主机指纹，确有需要时再由 SSH 自己提示输入密码；脚本不保存或回显密码。现场地址不同或已有 SSH Host 别名时可通过 `--host` 显式覆盖。

## 6. from-pc 传输边界

`from-pc` 使用 rsync 传输项目时：

- 不传 `.git`；
- 不传 `AGENTS.md`、`*_AGENTS.md`、`.robot-workspace`、`.robot-backups`、`.robot-sync` 和本地 G1/R1 参考资料；
- 保留机器人整个 `config/` 运行时配置目录和 `build/`；
- 如果机器人项目 Git checkout 有未提交修改，停止部署；
- 对脚本自己管理的非 Git 目录只在有管理标记时使用受控 `--delete`；
- 不把 SSH、sudo、Wi-Fi、API 密钥或其他凭据写入项目、参数、日志或缓存。

## 7. 只读 check

~~~bash
bash scripts/deploy.sh check --product g1
bash scripts/deploy.sh check --product r1
~~~

`check` 不安装软件包、不修改文件、不停止或启动 service。它检查系统、架构、目标网卡、所需包、SDK 状态、相机 helper 状态等。R1 `check` 直接使用 `/usr/local` 进行 SDK 校验。

## 8. 部署后验收

脚本会自动完成核心只读验收。人工复核可使用：

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
- G1/R1 CycloneDDS 动态库解析到所选产品 SDK 前缀；
- 相机 helper 的只读状态检查返回正常 active/inactive 语义。

## 9. 常见故障

### GitHub 不可达

机器人不能访问 GitHub 时不要反复重试，也不要修改 DDS 网卡。改用联网电脑：

~~~bash
bash scripts/deploy.sh from-pc --product g1 --host unitree@<G1_IP>
bash scripts/deploy.sh from-pc --product r1 --host unitree@<R1_IP>
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
~~~

### 相机

G1 先检查 librealsense、D435i USB 枚举和 `g1-web-first-person-service`。R1 检查固定 RTP/Depth source 与 `r1-web-camera-service`。浏览器不能覆盖受控 source、端口、pipeline、service 名或 helper 命令。

## 10. 扩展新产品

首版通用部署只登记 `g1` 与 `r1`。后续产品通过新增一个受控产品配置分支、对应 system/user service、helper/依赖声明和验收项接入；不要复制整套部署脚本，也不要引入 YAML、插件框架或新的部署依赖，除非现有 Bash `case` 模式已无法表达需求。
