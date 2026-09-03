# UniRoboGui

**简体中文** | [English](README.en.md)

> **面向 Unitree G1 EDU 的一站式二次开发与调试平台** —— 将分散在 Unitree SDK2 中的机器人状态、感知、控制、SLAM、关节、语音等能力统一接入、可视化和操作，让开发者不用先重复搭建调试工具，就能更快进入机器人功能验证与业务开发。

Unitree SDK2 提供了丰富的底层能力，但在实际项目中，这些能力分布在不同的数据通道、服务接口、示例程序和传感器链路中。客户在开始真正的二次开发之前，往往需要先自行组合接口、编写测试程序、确认机器人状态、调通感知与控制链路，再为各个模块分别准备可视化和调试工具。

**UniRoboGui 就是为解决这部分重复集成和调试工作而设计的。** 项目以 **C++17 Web 服务直接运行在机器人 PC2**，以 **Unitree SDK2 DDS** 作为核心通信链路，把原本分散的 SDK 能力整理成一套统一的 Web 开发工作台。开发者通过浏览器即可观察机器人、验证接口、调试功能和执行常用操作，并在这套已经打通的数据与控制基础上继续开发自己的机器人应用。

它不是对 Unitree SDK2 的替代，而是建立在 SDK2 之上的 **可观察、可调试、可复用的工程化开发底座**：减少重复的接口拼接和调试工作，让“先把 SDK 调通”更快过渡到“基于机器人能力继续开发”。

### 核心能力

- **统一 SDK2 能力入口**：直接接入 Unitree SDK2 DDS，将 LowState、BMS、双 IMU、里程计、FSM、29 关节以及官方控制 / 服务能力集中到同一套工程和界面中。
- **一体化调试与可视化**：在一个浏览器工作台中同时查看机器人状态、D435i RGB / 深度画面、G1 三维姿态、Mid-360 点云、SLAM 地图、导航、控制状态和语音交互，减少在多个工具之间反复切换。
- **感知、建图与导航链路**：支持实时点云、累积地图、轨迹与占据体素显示，以及建图、地图保存 / 加载 / 下载、初始位姿、单点 / 多点导航和本地导航任务。
- **关节调试与动作开发**：提供 29DoF URDF 实时姿态、上半身 / 全身关节调试、20 Hz 手掰示教、本地动作播放与保持 / 释放，以及 G1 遥控器按键绑定。
- **语音与大模型能力整合**：整合 ASR、Unitree 原生 TTS、本地 Kokoro 中英双语 TTS、机器人内置大模型，以及客户 OpenAI-compatible API、角色提示词、固定问答和唤醒短语。
- **面向后续二次开发**：提供中英文界面、在线 / 离线自动部署、Mock 回归模式和控制安全互锁，可直接作为机器人功能验证、现场调试以及上层应用开发的基础环境。

> **准备部署？** 直接跳转到 [快速部署](#quick-deploy)。
>
> 机器人可以访问 GitHub 和机器人无法访问 GitHub 两种场景都提供自动部署脚本。

---

## 1. 界面预览

以下图片来自 UniRoboGui 实际运行界面。不同机器人固件、机型、屏幕尺寸和现场设备状态可能使数据显示略有差异。

### 1.1 综合工作台

![UniRoboGui 综合工作台](docs/screenshots/web-workstation.jpg)

综合工作台是日常使用的主要入口，适合演示、调试和现场操作时集中观察机器人。

你可以在同一页面中：

- 查看 D435i RGB 与深度画面，并在需要时单独放大；
- 查看 G1 29DoF 三维姿态，实时跟随机器人关节状态；
- 查看 Mid-360 实时点云、累积地图、机器人轨迹与导航目标；
- 开始建图、保存 / 加载地图、设置初始位姿，以及执行单点或多点导航；
- 保存常用导航任务，后续直接恢复目标并发起导航；
- 查看当前控制状态，并使用已开放的运动模式、键盘运动和上肢动作；
- 使用 ASR、TTS、机器人内置大模型，或切换到客户自己的 OpenAI-compatible API；
- 在客户模式中配置角色提示词、固定问答、唤醒短语和本地 TTS。

地图操作提供平移、旋转、俯视、三维视角、缩放、全图和定位机器人等常用工具。
导航、运动和动作类操作仍需满足机器人当前状态与安全互锁条件。

### 1.2 机器人状态

![UniRoboGui 机器人实时状态](docs/screenshots/robot-status.jpg)

机器人状态页用于确认“机器人现在是什么状态”，适合部署完成后的验收、运行观察和问题定位。

页面会集中显示：

- SDK2 DDS 连接状态和各数据源在线情况；
- 当前运动模式、状态版本和遥测更新时间；
- BMS 电量、电压、电流、温度等电池信息；
- 里程计、速度与机器人位姿数据；
- 腰部 IMU 与主板 IMU；
- 29 个机身关节的角度、速度、力矩、温度、电压和原始状态；
- 根据当前机型自动选择的本地 G1 URDF，并同步显示真实关节姿态；
- 关节搜索、状态筛选和原始电机槽数据。

这个页面以观察为主，不需要为了查看状态而进入控制模式。

### 1.3 机器人调试台

![UniRoboGui 机器人调试台](docs/screenshots/joint-debug.jpg)

机器人调试台面向需要进行二次开发、关节姿态调试和动作示教的用户。

主要能力包括：

- 在 **上半身** 与 **全身** 两种调试模式之间切换；
- 使用 rad / ° 两种单位查看和编辑关节目标；
- 从机器人当前真实姿态加载目标值，减少突然跳变；
- 查看后端下发互锁、FSM、LowState、DDS 控制等安全条件；
- 手动调整关节目标并在满足安全条件时应用；
- 进行 20 Hz 手掰示教录制并保存为本地动作；
- 播放、删除已保存动作，并选择播放结束后释放控制或保持最后姿态；
- 将本地动作绑定到 G1 遥控器预留组合键，便于现场重复触发；
- 随时使用“停止 / 释放控制”恢复控制权。

该页面涉及真实关节控制。使用前应保证机器人可靠支撑、周围无人和障碍物，并确认当前机器人状态允许调试。

### 1.4 诊断与原始数据

![UniRoboGui 诊断与原始数据](docs/screenshots/diagnostics.png)

诊断页用于快速判断问题更可能来自 **数据链路、运动状态、电机、电池还是主板**，适合部署验收和现场故障排查。

页面当前会汇总：

- **DDS / 里程计**：DDS 初始化状态、数据源在线数量、LowState 延迟、里程计错误码；
- **电机诊断**：机身故障电机数量、最高壳体温度、最高绕组温度、电机电压范围及已公开故障项；
- **电池诊断**：SOC / SOH、电芯范围、电芯压差、电流、最高温度和 BMS 原始状态；
- **主板诊断**：温度、风扇状态以及主板原始状态字段；
- **服务状态**：控制服务或语音服务初始化异常会进入诊断汇总，方便先判断是否属于服务启动问题；
- **原始数据**：BMS 与主板原始内容，便于进一步与官方资料或现场日志对照。

对于官方没有公开位定义的字段，页面只显示原始值，不会自行猜测故障含义。

---

## 2. 功能概览

| 模块 | 面向用户的能力 |
| --- | --- |
| 综合工作台 | 将摄像头、三维机器人、SLAM 地图、控制和语音交互集中在一个响应式页面 |
| 机器人状态 | 查看 DDS、BMS、FSM、里程计、双 IMU、29 关节和当前机型三维姿态 |
| 诊断 | 汇总 DDS、里程计、电机、电池和主板状态，并保留未公开字段的 raw 数据 |
| 三维模型 | 使用本地 Three.js + URDF Loader 显示 G1 29DoF 模型并同步真实关节 |
| SLAM / 点云 | 显示 Mid-360 实时点云、累积地图、机器人轨迹和占据体素 |
| 地图管理 | 建图、保存、加载、下载、退出地图，以及地图初始位姿设置 |
| 导航 | 单点、多点、暂停、继续、取消，以及可保存的本地导航任务 |
| RGB / 深度相机 | 自动识别 D435i RGB / Z16 输入，也支持手动指定设备 |
| 机器人控制 | 安全锁、常用运动模式、键盘运动、上肢预设动作和固件示教动作 |
| 调试与示教 | 上半身 / 全身关节目标、手掰录制、动作播放、保持 / 释放和遥控器绑定 |
| ASR | 接收机器人语音识别结果、显示最近识别历史并转交给大模型 |
| TTS | Unitree 原生 TTS 与本地 Kokoro 中英双语播报 |
| 大模型 | 机器人内置对话或客户 OpenAI-compatible API，支持角色、固定问答和唤醒短语 |
| 国际化 | 中文 / English 切换，静态和动态界面使用同一套翻译机制 |
| Mock | 无真实 DDS 初始化的开发与回归模式，用于验证 UI、HTTP、WebSocket 和安全状态机 |

---

<a id="quick-deploy"></a>

## 3. 快速部署

部署已经封装为两套脚本。正常情况下不需要手工安装 SDK2、librealsense2、Kokoro，
也不需要逐条执行 CMake 命令。英文终端提示可使用对应的 `.en.sh` 薄入口；中英文入口复用同一套部署逻辑，
参数、安全检查和退出码保持一致。

### 3.1 部署前确认

目标机器人环境：

| 项目 | 要求 |
| --- | --- |
| 机器人 | Unitree G1 EDU |
| PC2 系统 | Ubuntu 20.04 AArch64 |
| 用户 | `unitree` |
| DDS 网卡 | `eth0` |
| 项目目录 | `/home/unitree/UniRoboGui` |
| 深度相机 | Intel RealSense D435i |
| 浏览器访问 | 与机器人网络互通的电脑 / 平板 |

SSH 登录机器人时，如果出现 ROS 环境选择，请直接按回车选择 **none**。
部署和运行过程中请保留 `eth0` 给 Unitree SDK2 DDS。

部署脚本涉及三类外部资源，第一次部署前请区分清楚：

安装器还会自动选择服务安装范围：

- **有交互 sudo**：使用系统级安装，SDK2 放到 `/opt/unitree_robotics`，服务由系统 `systemd` 管理；同时安装 D435i 兼容 helper/sudoers。
- **没有可交互 sudo**：仅当 Ubuntu 系统包已经齐全、`systemctl --user` 可用且 `Linger=yes` 时，自动使用持久用户级安装；SDK2 放到 `/home/unitree/.local/unitree_robotics`，服务由 `systemctl --user` 管理，退出 SSH 后仍继续运行并可随用户 systemd 在开机后恢复。
- 用户级安装下 D435i 会优先通过 librealsense2 与机器人原相机服务并发共享 RGB+Depth；只有某些固件拒绝并发访问、需要暂停原相机服务时，才必须改用有 sudo 的系统级安装来获得 helper 兜底。

- **方式 A**：机器人需要能访问 GitHub；首次安装 Kokoro 还需要访问 PyPI；如果缺少 Ubuntu 系统包，还需要能访问 Ubuntu 20.04 软件源。
- **方式 B**：GitHub、GitHub Release 和 PyPI 都由联网电脑访问，机器人本身可以完全访问不了 GitHub/PyPI；但如果机器人缺少 Ubuntu 系统包，仍需要临时通过 `wlan0` 访问 Ubuntu 20.04 软件源，或先人工安装脚本列出的缺失 `.deb` 包。脚本会在安装前列出缺失包，已全部安装时会直接跳过 APT。
- 如果你拿到的是**私有仓库**版本，请先确认当前 GitHub 凭据具有仓库读取权限。不要把 GitHub Token、SSH 密码或其他凭据写进 README、脚本参数或提交到仓库。

### 3.2 方式 A：机器人可以访问 GitHub

在机器人中执行：

```bash
git clone https://github.com/ershui2500/UniRoboGui.git /home/unitree/UniRoboGui
cd /home/unitree/UniRoboGui
bash scripts/deploy_g1_online.sh
```

如果项目已经存在：

```bash
cd /home/unitree/UniRoboGui
bash scripts/deploy_g1_online.sh
```

脚本会自动：

1. 检测机器人是否能访问 GitHub；
2. 检查 G1 PC2、Ubuntu 20.04、AArch64、`unitree` 用户、无 ROS 环境和 `eth0`；
3. 在已有安装场景中安全更新 UniRoboGui 自身；发现未提交现场修改时停止而不是覆盖；
4. 检查系统编译依赖，已齐全时跳过 APT；缺包时由有 sudo 的系统级安装补齐；
5. 获取/准备官方 Unitree SDK2 源码，并按最终安装范围放到 `/opt/unitree_robotics` 或用户本地前缀；
6. 检测机器人已有 librealsense2；可用时直接复用，缺失时在系统级安装中获取官方源码并编译安装；
7. 编译 UniRoboGui，并运行 CTest 和动态库检查；
8. 安装 Kokoro 本地 TTS；
9. 自动安装 / 更新系统级或持久用户级 systemd 服务；
10. 完成 Web 与 TTS 健康检查。

如果第一步发现机器人无法访问 GitHub，脚本会停止并提示改用方式 B。

### 3.3 方式 B：机器人不能访问 GitHub，但另一台电脑可以访问

在一台能访问 GitHub、并且能通过 SSH 连接机器人的 Ubuntu / Linux 电脑上执行：

```bash
git clone https://github.com/ershui2500/UniRoboGui.git
cd UniRoboGui
bash scripts/deploy_g1_from_pc.sh
```

默认机器人地址：

```text
unitree@192.168.123.164
```

如果现场机器人地址不同：

```bash
bash scripts/deploy_g1_from_pc.sh --robot unitree@<机器人IP>
```

联网电脑脚本会自动：

1. 检查本机 GitHub 连接和部署工具；
2. 先检查 SSH 到机器人，尽早发现 IP、账号或网络错误；
3. 下载 / 更新 UniRoboGui 和 Unitree SDK2，并准备缺失时可用的 librealsense2 源码；
4. 从 GitHub Release 下载 Kokoro 模型；
5. 从 PyPI 准备 G1 AArch64 + Python 3.8 所需的离线 Python wheelhouse；
6. 检查机器人已有源码并通过 rsync 传入项目和 GitHub/PyPI 依赖；项目传输明确排除 `.git/`，避免把联网电脑的 Git 配置或潜在凭据带到机器人；
7. 在机器人端检查 Ubuntu 系统包、编译、CTest、部署服务并完成健康检查。

因此联网电脑不仅要能打开 GitHub，也需要能访问 PyPI。机器人如果缺少 Ubuntu 系统包，
脚本会明确列出包名；此时需要让机器人临时通过 `wlan0` 访问 Ubuntu 20.04 软件源，或先人工安装这些包，
不要为上网而断开或修改 `eth0`。

推荐从可交互终端运行方式 B。首次 SSH 连接可能要求确认主机指纹；SSH 或 sudo 需要密码时按终端提示输入即可。
如果最终机器人安装阶段没有可交互 sudo，但系统包已经齐全且 `systemctl --user` + `Linger=yes`，安装器会自动切换到持久用户级部署；只有缺系统包、缺 librealsense2 系统集成或需要特权相机 helper 时才必须重新使用交互 sudo。
部署脚本不会保存 SSH、sudo、Wi-Fi 或 API 密码。

### 3.4 部署完成后

在机器人查看 IP：

```bash
ip -4 addr show wlan0
ip -4 addr show eth0
```

浏览器打开：

```text
http://<机器人IP>:8080
```

只读验收。系统级安装使用 `systemctl`，用户级安装使用 `systemctl --user`：

```bash
systemctl is-active g1-web-control.service 2>/dev/null || \
  systemctl --user is-active g1-web-control.service
curl --noproxy '*' -fsS http://127.0.0.1:8080/api/health
curl --noproxy '*' -fsS http://127.0.0.1:8080/api/control/status
```

如启用了本地 Kokoro：

```bash
systemctl is-active g1-local-tts.service 2>/dev/null || \
  systemctl --user is-active g1-local-tts.service
curl --noproxy '*' -fsS http://127.0.0.1:8765/health
```

更完整的人工依赖安装与故障定位步骤见
[`docs/deployment-dependencies.md`](docs/deployment-dependencies.md)。

### 3.5 升级

升级和首次安装使用相同脚本：

- 机器人可访问 GitHub：`bash scripts/deploy_g1_online.sh`
- 机器人无法访问 GitHub：在联网电脑运行 `bash scripts/deploy_g1_from_pc.sh`

在线方式会先安全执行 UniRoboGui 的 `git pull --ff-only`，离线方式会由联网电脑更新项目后再同步；
两种方式都会重新准备上游依赖并构建项目，同时保留客户侧运行配置。检测到项目源码存在未提交修改时会停止并提示先备份/提交，避免升级覆盖现场代码。
升级时安装器会识别当前 system/user 服务范围；替换正在运行的 Web 服务前必须先确认机器人运动状态为 `stopped` 且三个速度均为 0。重复运行同一部署脚本是受支持的，已有 SDK2/Kokoro/build 会被复用或增量更新。

---

## 4. 访问与使用

### 4.1 页面入口

部署完成后，浏览器访问：

```text
http://<机器人IP>:8080
```

左侧导航包含：

- **综合工作台**
- **机器人状态**
- **机器人调试台**
- **诊断**

右上角可以切换中文 / English，并查看连接、电量、FSM、控制锁等摘要状态。

### 4.2 网络说明

UniRoboGui 的生产运行使用 Unitree SDK2 DDS：

```text
eth0  -> Unitree SDK2 DDS
wlan0 -> 网页访问 / 外网
```

不要为了机器人上网而断开 `eth0`。
Web、SLAM 和摄像头进程也不需要 source Foxy、Noetic 或 ROS 2 环境。

### 4.3 运动与调试安全

机器人控制、导航、上肢动作和关节调试具有真实物理副作用。

使用这些功能前应确认：

- 机器人已可靠支撑或处于适合当前动作的稳定状态；
- 周围人员与障碍物已经清空；
- 页面连接正常，机器人状态数据持续更新；
- 当前 FSM / 控制模式满足页面提示；
- 对动作结果不确定时优先使用 Mock 或只读页面验证。

Web 页面本身没有身份认证，因此不应暴露到不可信网络或公网。

---

## 5. 兼容性

| 项目 | 当前目标 |
| --- | --- |
| 机器人 | Unitree G1 EDU，29DoF 机型优先 |
| PC2 | Ubuntu 20.04 AArch64 |
| SDK | Unitree SDK2 |
| SDK 安装前缀 | 系统级 `/opt/unitree_robotics`；无交互 sudo 的持久用户级 `/home/unitree/.local/unitree_robotics` |
| 默认项目目录 | `/home/unitree/UniRoboGui` |
| 深度相机 | Intel RealSense D435i |
| 激光雷达 | Livox Mid-360 / Mid360s，以机器人实际配置为准 |
| 浏览器 | 现代 Chromium / Edge / Chrome 类浏览器 |

机器人固件、SDK2、LiDAR 服务版本、设备节点和网络地址可能随机器人批次或现场环境变化，
应以实际机器人状态为准。

---

### 5.1 项目结构

```text
UniRoboGui/
├── README.md                   # 中文 README
├── README.en.md                # English README
├── AGENTS.md
├── LICENSE
├── VERSION
├── CMakeLists.txt
├── include/                    # C++ 头文件
├── src/                        # C++ 后端
├── web/                        # Web 前端与本地静态资产
├── tests/                      # CTest / HTTP 回归
├── scripts/                    # 自动部署、TTS、相机 helper、发布脚本
├── deploy/                     # systemd 服务
├── config/                     # 安装/运行时创建的机器人本机配置目录
└── docs/                       # 部署说明与界面截图
```

---

<a id="常见问题"></a>

## 6. 常见问题

### 6.1 机器人不能访问 GitHub

不要在机器人上持续重试。

在可以访问 GitHub 的 Ubuntu / Linux 电脑中运行：

```bash
bash scripts/deploy_g1_from_pc.sh
```

脚本会准备依赖并通过 SSH / rsync 传入机器人。

### 6.2 方式 B 仍提示 APT / Ubuntu 软件源失败

方式 B 会把 GitHub、GitHub Release 和 PyPI 资源从联网电脑传入机器人，但不会把整套 Ubuntu 软件源一起打包。
如果机器人缺少某个系统编译包，仍然需要 Ubuntu 20.04 arm64 软件包。

先运行只读检查：

```bash
cd /home/unitree/UniRoboGui
bash scripts/install_g1.sh --check-only
```

如果输出 `Missing Ubuntu system packages`，按提示临时让 `wlan0` 能访问 Ubuntu 20.04 软件源，
或离线安装脚本列出的正确 arm64 `.deb` 及其依赖；不要断开或修改 `eth0`，也不要使用 x86_64 或其他 Ubuntu 版本的软件包。
如果系统包已经齐全，新版脚本会直接跳过 APT。

### 6.3 GitHub 可以访问，但 Kokoro / PyPI 安装失败

能打开 GitHub 不代表 PyPI 一定可访问。首次安装 Kokoro 需要 Python wheel；如果机器人访问 PyPI 不稳定，
可以直接改用方式 B，让联网电脑准备 G1 AArch64 / Python 3.8 wheelhouse，再通过 SSH / rsync 传入机器人。

### 6.4 `wlan0` 显示 `wifi:unavailable` / 扫不到热点

先检查：

```bash
nmcli device status
nmcli radio wifi
rfkill list wifi
```

如果 `wlan0` 为 `wifi:unavailable`，同时 `rfkill` 显示 `Soft blocked: yes`，执行：

```bash
nmcli radio wifi on
```

再重新扫描和连接 Wi-Fi。不要禁用、重置或断开 `eth0`，它是 Unitree SDK2 DDS 链路。
如果现场不希望让机器人访问外网，可以不处理 Wi-Fi，直接使用方式 B 从联网电脑通过 SSH / rsync 部署。

### 6.5 出现 `undefined symbol: ddsi_sertype_v0`

通常表示 ROS / CycloneDDS 环境与 Unitree SDK2 动态库发生混用。

重新建立 SSH 会话，在 ROS 环境选择中直接按回车选择 **none**，再重新启动或部署服务。

### 6.6 8080 页面无法访问

机器人上检查：

```bash
systemctl --no-pager --full status g1-web-control.service
journalctl -u g1-web-control.service -n 80 --no-pager
curl --noproxy '*' -v http://127.0.0.1:8080/api/health
ss -ltnp 'sport = :8080'
```

如果机器人本机健康检查正常，再检查访问电脑与机器人之间的网络。

### 6.7 D435i 没有画面

优先检查：

- USB 是否识别 D435i；
- librealsense2 是否已正确安装；
- 摄像头是否被其他机器人服务占用；
- 页面输入框是否误填了固定 `/dev/videoN`。

D435i 的“启动 RGB + 深度”会优先使用 librealsense2 与机器人现有相机服务并发共享；在支持并发访问的 G1 固件/镜像上，不需要停止 `master_service`，因此也不依赖 root helper。只有设备确实被独占、并发探测失败时，Web 才回退到“暂停相机占用服务 + V4L2 自动识别”的兼容路径。

如果回退路径出现 `first_person_status_failed`，通常表示用于暂停/恢复机器人原相机占用服务的 helper 或 sudoers 没有安装完整。重新运行正式部署脚本；安装器会安装并验证：

```text
/usr/local/sbin/g1-web-first-person-service
/etc/sudoers.d/g1-web-camera
```

安装器会使用 `sudo -n ... is-active` 做只读权限验证，失败时直接停止并提示检查 sudoers。设备号可能随 USB 枚举变化，V4L2 模式正常情况下建议保持设备输入为空，让页面自动识别。

### 6.8 页面显示数据离线

检查：

- `eth0` 是否存在且保持连接；
- 是否错误加载了 ROS / RMW / CycloneDDS 环境；
- `g1-web-control.service` 是否正常运行；
- 机器人对应数据源本身是否正在发布。

### 6.9 客户大模型无法使用

检查 API Base URL、模型名和鉴权配置是否与客户实际服务一致。
页面支持 OpenAI-compatible Chat Completions 接口；不同云厂商的模型标识应按其官方 API 文档填写。

---

## 7. 文档与官方资料

### 7.1 项目文档

- [依赖安装与离线部署](docs/deployment-dependencies.md)
- [第三方静态资产声明](web/assets/THIRD_PARTY_NOTICES.md)
- [项目开发与安全规则](AGENTS.md)

### 7.2 Unitree 官方资料

- [G1 获取 SDK](https://support.unitree.com/home/zh/G1_developer/get_sdk)
- [G1 SLAM 导航服务接口](https://support.unitree.com/home/zh/G1_developer/slam_navigation_services_interface)
- [G1 LiDAR 服务接口](https://support.unitree.com/home/zh/G1_developer/lidar_services_interface)
- [G1 深度相机例程](https://support.unitree.com/home/zh/G1_developer/depth_camera_instruction)
- [Unitree SDK2](https://github.com/unitreerobotics/unitree_sdk2)

---

## 8. 问题反馈与贡献

如果你在新的 G1、不同固件或不同现场网络中遇到问题，建议先查看上面的[常见问题](#常见问题)和
[依赖安装与离线部署文档](docs/deployment-dependencies.md)。

确认属于项目问题后，可以通过 [GitHub Issues](https://github.com/ershui2500/UniRoboGui/issues) 提交。
为了更快定位问题，建议同时提供：

- UniRoboGui 版本；
- Unitree G1 EDU 机型与 `mode_machine`（如已知）；
- 问题发生在哪个页面和操作步骤；
- 浏览器提示或 HTTP 错误；
- `systemctl status` / `journalctl` 中与问题相关的日志；
- 是否能够稳定复现，以及复现前机器人是否处于运动或调试状态。

提交日志或截图前，请先移除 API Key、Wi-Fi 密码、SSH 凭据、Cookie 和其他敏感信息。

欢迎基于本项目进行二次开发。涉及 Web 用户可见功能时，请同时维护中文和英文；涉及机器人运动、导航、
模式切换、上肢或示教时，优先完成 Mock 回归，再进行经过明确安全确认的真机验证。

---

## 9. 致谢与引用

UniRoboGui 建立在机器人和开源社区提供的优秀基础设施之上。感谢以下项目和维护者：

### 9.1 机器人与设备

- [Unitree SDK2](https://github.com/unitreerobotics/unitree_sdk2)  
  提供 G1 的 DDS 通信、状态数据和机器人服务接口，是 UniRoboGui 与机器人通信的核心基础。
- [Unitree unitree_ros](https://github.com/unitreerobotics/unitree_ros)  
  本项目随仓库分发的 G1 URDF / STL 模型资源来源于该项目的固定提交。
- [librealsense](https://github.com/realsenseai/librealsense)  
  提供 Intel RealSense D435i 的底层设备支持。

### 9.2 三维与机器人可视化

- [Three.js](https://github.com/mrdoob/three.js)  
  用于浏览器端 G1 三维模型、点云和地图可视化。
- [urdf-loaders](https://github.com/gkjohnson/urdf-loaders)  
  提供浏览器端 URDF 解析与模型加载能力。
- [RViz](https://github.com/ros2/rviz) 与 [Foxglove Studio](https://github.com/foxglove/studio)  
  在机器人状态可视化、点云观察、地图交互和诊断信息组织方式上提供了重要设计参考。
  UniRoboGui 不依赖 RViz 或 Foxglove 运行。

### 9.3 语音

- [sherpa-onnx](https://github.com/k2-fsa/sherpa-onnx)  
  为本地 Kokoro TTS 提供 ONNX 推理运行时和相关模型发布资源。

同时感谢 Boost、JsonCpp、OpenCV、libcurl、ZeroMQ 以及其他基础开源组件的维护者。

---

## 10. 许可证

### 10.1 UniRoboGui 自有代码

UniRoboGui 自有代码使用 [MIT License](LICENSE)。

MIT License 允许在保留版权与许可证声明的前提下，对项目代码进行使用、复制、修改、合并、
发布、分发、再许可和销售。软件按“原样”提供，不附带明示或默示担保。
完整法律条款以仓库中的 [`LICENSE`](LICENSE) 为准。

### 10.2 随仓库分发的第三方静态资产

部分前端和模型资源随仓库一起分发，因此继续遵循各自的上游许可证：

| 资源 | 来源 | 许可证 |
| --- | --- | --- |
| G1 URDF / STL | Unitree `unitree_ros` | BSD-3-Clause |
| Three.js 0.164.1 | Three.js | MIT |
| urdf-loader 0.13.1 | urdf-loaders | Apache-2.0 |

完整版本、来源提交和本地许可证文件见
[`web/assets/THIRD_PARTY_NOTICES.md`](web/assets/THIRD_PARTY_NOTICES.md)。

### 10.3 部署时取得的外部依赖

Unitree SDK2、librealsense2、sherpa-onnx、Kokoro 模型以及系统库在部署过程中从各自上游获取，
其许可证、模型权重许可和再分发条件由对应上游项目决定。

如果你计划重新分发二进制、模型、第三方资源或制作商业发行版，请同时检查所有上游项目的
LICENSE / NOTICE / 模型许可，不要把 UniRoboGui 的 MIT License 视为对第三方组件的重新授权。

### 10.4 商标与项目关系

Unitree、Intel RealSense、Three.js、Foxglove、ROS / RViz 以及其他第三方名称和商标归各自权利人所有。
UniRoboGui 是独立的第三方项目，不是 Unitree 官方产品，也不代表任何上游项目对本项目的背书。
