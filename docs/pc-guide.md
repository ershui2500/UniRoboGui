# Ubuntu 电脑 SDK2 控制台（G1 / R1 / GO2 / A2）

UniRoboGui 在电脑运行，通过 Unitree SDK2 DDS 连接 G1、R1、GO2 或 A2。机器人不需要安装本项目，仍须运行其原有遥测和其他必要服务。该入口与将项目部署到机器人的 `deploy.sh install/from-pc` 相互独立。

## 支持与验证状态

- 系统：Ubuntu 22.04 x86_64 / Python 3.10。
- 真实连接入口：G1、R1、GO2、A2。
- 能力：按所选产品复用现有 Profile、遥测、控制服务、SDK2 Adapter、关节调试/示教和音频。R1 保持 26 个语义关节与稀疏电机槽映射；GO2/A2 保持 12 个关节槽与四足布局。真实运动和关节控制仍须现场操作员验证。
- GO2、A2 的 PC RGB 复用固定 RTP/H264 接收；G1/R1 RGB、所有 PC 深度以及 R1 的 GST 相机链路保持关闭，页面显示“电脑端尚未接入机器人相机传输”。PC 不启停机器人相机服务、不运行深度 helper。未声明附件型 SLAM 保持关闭。G1/R1/A2 使用原生 TTS，GO2 保留音量控制但没有原生 TTS；PC 不安装 Kokoro。
- 当前实现有 Mock 回归；G1/R1/GO2/A2 有线直连、拔线恢复和干净系统首次安装仍须分别验收。Mock 不代表真实连接或控制已验证。

## 安装

在电脑上的项目目录执行（普通账号，系统包缺失时由 sudo 安装）：

```bash
bash scripts/pc.sh install
bash scripts/pc.sh check
bash scripts/pc.sh start
```

浏览器打开 `http://127.0.0.1:8080`。程序只监听电脑回环地址，启动时未连接机器人；不创建开机自动连接或机器人 systemd 服务。关闭终端程序用 Ctrl+C，等待工作进程退出。普通控制会话有占用或缺少停止证据时拒绝退出；可先按下文进入只读恢复隔离。隔离工作进程拒绝控制写入；R1 只保留 FSM 查询，以验证新鲜停止状态。退出不证明机器人已停止。

PC 安装与检查会校验 GStreamer tools、good/bad 插件和 libav 解码器；固定 RTP/H.264 相机管线的 `watchdog` 由 `gstreamer1.0-plugins-bad` 提供。已有安装若报告缺少该包，重新运行 `bash scripts/pc.sh install` 补齐依赖；仅有 OpenCV 和 DDS 在线不代表视频管线依赖完整。

R1 的 PC RGB 和深度画面均未接入，两个面板统一显示“电脑端尚未接入机器人相机传输”。相机接收、启动命令和图像接口保持关闭；PC 不接入 R1 的 GST 视频链路。

安装固定 SDK2 提交 `7740f8b67e386ab09c3b333187fd5f8582a75ddc`，在当前账号的 `~/.local/share/unirobogui/sdk2/<提交>/` 独立安装，不覆盖已有 SDK。重复安装复用该版本并重新构建和测试。源码下载失败可提前准备该提交的干净官方源码：

```bash
bash scripts/pc.sh install --sdk2-source /absolute/path/to/unitree_sdk2
```

`--sdk-prefix` 可以指定独立 SDK 安装目录；已存在但无法确认版本或不完整的目录会被拒绝。`--build-dir` 可以选择独立构建目录，后续 start/check 必须传同一目录。编译后的 SDK 路径记录在构建目录，移动目录后重新安装。电脑安装不修改网络、路由或防火墙，也不安装相机 helper、TTS 模型或机器人服务。

使用未加载 ROS/RMW/CycloneDDS 的终端。运行环境中的 DDS 库只使用选定 SDK，不能混用其他 ROS DDS 库。安装包含 Release 构建、全部 CTest 和 ldd 检查；正常运行无需 sudo。

## 有线连接

1. 用网线或隔离交换机连接电脑和一台 G1、R1、GO2 或 A2，确认所选网络没有第二台机器人。
2. 根据实际机器人网络手动配置电脑有线 IPv4，保持地址唯一；安装器不代替现场配置，也不改变机器人网络。
3. 在页面选择 G1、R1、GO2 或 A2，在顶部网卡组件选择有线网卡并点击“连接”。未连接时网卡列表每 5 秒自动检测，保留当前选择；连接后停止自动检测。物理 Ethernet/USB 网卡需要处于 UP 并有 IPv4；Wi-Fi、桥接和隧道不在首期范围。
4. 查看顶部右侧连接状态；网卡旁的信息入口显示当前网卡、IPv4 和最近数据时间。只有必需遥测持续更新才显示在线；DDS 初始化成功不代表收到机器人数据，也不代表确认物理设备身份。
5. 断开、产品切换或换网卡前，必须结束运动/内置动作、关节接管、示教保持、建图和导航，确认停止状态及遥测新鲜。需要新鲜 LowState、控制状态和里程计；GO2 的运动状态来自 `rt/sportmodestate`，A2 使用已登记的 HG 高层状态源。活跃控制或缺少停止证据会拒绝切换，不自动发送停止或释放命令。
6. 连接后按钮显示“断开”；选择其他网卡时显示“连接”，点击后才结束旧连接并切换，选回当前网卡则恢复“断开”。缺少新鲜停止证据时普通断开仍会拒绝；使用连接信息中的“只读恢复连接”按下文恢复。更换机器人前完成断开，再更换连接和产品。

DDS 首期仍使用现有 domain 0 和固定产品 topic；填写机器人 IP 不能提供设备隔离。产品身份来自所选 Profile，不自动识别某个序列号。必须由操作员确认现场型号和单设备网络。

G1/R1/GO2/A2 的 PC 连接状态以 DDS 初始化和新鲜 LowState 为准，与 Manifest 的 `required_sources` 整机健康检查分开。运控、里程计、电池或主板等辅助源缺失时，各数据源仍如实显示过期/离线，但不会把仍有 LowState 的整台机器人判为断线。停用 GO2 MCF 后，只要 LowState 仍新鲜，状态和关节调试页面可继续使用；高层运动/动作仍检查新鲜运控状态，低层调试仍检查控制权、LowState、增益和其他既有互锁。运控状态过期时不显示旧 FSM 为当前状态；普通断开/切换的停止证据要求不变。

LowState 超过既有新鲜度窗口会显示过期/离线，不自动重连机器人、不自动换网卡。页面 WebSocket 可以重新连接电脑工作台，这不会创建新的机器人会话。数据源持续缺失时，检查网线、IPv4、组播、防火墙和机器人原有发布服务，不通过运动命令探测。

## Mock 与接口

不连接机器人测试页面：

```bash
bash scripts/pc.sh start --mock --port 18080
```

Mock 模式可选择 `lo`，页面明确显示“模拟”。真实模式不允许该接口。

| 接口 | 行为 |
| --- | --- |
| `GET /api/pc/interfaces` | 有线接口、UP、IPv4 列表；不修改网络 |
| `GET /api/pc/session` | 会话 ID、所选产品、运行状态、Mock 标志、DDS、数据接收时间及验收状态 |
| `POST /api/pc/connect` | JSON `product_id`、`interface`，只接受 G1/R1/GO2/A2 和本机就绪有线接口 |
| `POST /api/pc/disconnect` | JSON `{}`，结束旧工作进程并创建无 DDS 的离线视图 |
| `POST /api/robot/switch` | JSON `product_id`，结束旧会话，切换到离线产品视图 |
| `POST /api/pc/recover` | JSON `session_id`，冻结并结束旧进程，仅对同一产品/网卡创建只读遥测会话 |
| `POST /api/pc/confirm-recovery` | JSON `session_id`、`confirmed: true`；现场明确确认后，结束只读隔离并进入离线视图；遥测缺失不阻止退出，新鲜数据仍显示运动时拒绝 |

保留 `/api/health`、`/api/snapshot`、`/api/robot/manifest`、`/api/robot/products`、`/api/control/status` 与 `/ws/telemetry`。Manifest 增加 `runtime_mode=pc_sdk2`（离线 worker 为 `pc_readonly`）、`pc_session_id`、`physical_identity_verified=false` 和 `pc_session`；遥测包含 `pc_session_id`。产品列表增加 `pc_connect_supported` 和 `pc_direct_validation`。已连接时还代理现有控制、关节调试/示教、音频及感知的固定接口白名单；仅开放固定 RGB 接收/停止、状态和 RGB JPEG 接口；深度、任意视频源、任意 topic 配置和未知接口关闭。管理入口检查 Host/Origin，业务写操作校验 `X-Unirobo-PC-Session`，旧会话返回 `pc_session_changed`；写操作只接受 JSON（关节心跳允许空请求体）。离线 worker 后端仍拒绝设备写操作。

状态包括 `disconnected`、`waiting_telemetry`、`online`、`stale`、`network_unavailable`、`error`。附加字段 `worker_alive`、`worker_readonly`、`recovery_required`、`recovery_reason` 和 `arm_action_state` 区分进程状态、恢复隔离及上肢任务状态。每次实际切换生成新会话 ID；前端重载全部视图并丢弃旧会话数据。包括管理操作在内的所有写入均校验 `X-Unirobo-PC-Session`；恢复和确认还要求请求体中的 `session_id` 与当前会话一致。

PC 验收状态独立于机器人端：Manifest 不继承机器人端控制已验证标志。G1 相机、PC 深度以及 R1/GO2/A2 的未声明附件型 SLAM 保持关闭；不探测本机设备或自动启停机器人服务。A2 真机全身关节控制默认使用下文说明的内置实验增益，也可用现场确认的每关节增益文件覆盖。G1 地图下载优先 SDK2 全局地图数据，其次本账号 Web 缓存；机器人地图地址仍作为机器人端 RPC 参数使用。示教动作存于 PC 项目的 `config/joint_teach_actions.json`；客户配置沿用工作目录下的 `config/customer_voice.json`，不读取机器人文件。

断开检查区分实际控制占用与里程计估计：新鲜遥测和已释放控制仍为前提，仅 G1/R1 Profile 声明的蹲姿/被动状态（FSM 0/1/2/3/706）不使用漂移的里程计速度阻止断开；A2/GO2 不继承这些编号；其他状态只检查平面速度（0.03 m/s）和转速（0.05 rad/s），不将垂向估计当作行走。拒绝时提示具体的未完成请求、运动、关节/示教、建图/导航或控制占用原因。阈值用于断开判断，不改变运动限幅。

## 进程崩溃、长期断线与动作未知的恢复

正常有停止证据且没有占用时，仍可直接断开或切换，无需新增确认。缺少证据时不从进程退出、SDK 初始化成功或底盘静止推断机器人已停止：

1. 打开顶部网卡旁的连接信息，点击“只读恢复连接”。本机仍有正在处理的请求、运动、关节/示教接管、保持、建图/导航或未确认导航任务时拒绝恢复，应先由现场结束这些任务。旧进程已退出时可直接进入下一步；旧进程尚存时先冻结控制入口并复查占用，等待它正常退出。HTTP 不响应的控制进程不能证明无本机占用，恢复会拒绝；进程退出超时也禁止创建新进程，不强杀、不启动第二个工作进程。
2. 旧进程退出后，只在原产品和原网卡上建立新的只读会话。该进程不创建可用的控制、音频或 SLAM 能力，不启动示教遥控，不恢复旧队列、租约、保持或导航。R1 只读恢复会通过现有 LocoClient 查询 FSM，以提供新鲜控制状态；只读 Profile 和 HTTP 后端仍拒绝控制写入。所有产品/网卡切换被管理端阻止。网线未恢复时等待数据；只读初始化失败或再次失联，可再次手动点击只读恢复。没有自动重连。
3. 由现场人员确认：同一台所选 G1/R1/GO2/A2 已可靠停止，所有旧运动、动作、关节接管、示教保持、建图及导航任务均已结束。点击“确认现场停止并断开”并确认说明。该按钮不会向机器人发送停止、release 或模式命令；它只结束隔离会话，回到离线视图。遥测缺失、过期或只读进程异常退出时仍可通过现场确认退出；如新鲜遥测显示机器人仍在运动，仍会拒绝。只读遥测不证明旧动作/SLAM 任务结束，这一部分明确依赖现场确认。
4. 按正常流程手动连接，才能建立新的控制会话。新会话不重放任何旧命令。隔离期间可用 Ctrl+C 正常退出本机程序；这不代替现场停止确认。

## 内置上肢与固件示教动作的状态边界

2026-11-12 核实的[官方手臂动作服务接口](https://support.unitree.com/home/zh/G1_developer/arm_action_interface)将整数预设调用描述为阻塞，字符串固件示教调用描述为非阻塞；`7400` 表示有动作执行，`7401` 表示保持姿态。固定 SDK2 提交的 `include/unitree/robot/g1/arm/g1_arm_action_client.hpp` 说明部分预设会在末帧保持，并给出 `rt/arm/action/state` 的 `id/name/holding` JSON 示例；未给出可核实的该话题 wire type、执行序号或与本次请求关联的完成/释放回执。本实现不猜测 wire type，不额外启用该话题。

连接信息显示 `dispatching`（RPC 处理中，物理进度未知）、`running_reported` / `holding_reported`（最近 RPC 明确报告执行占用/保持）、`unknown`（完成/释放未证实）或 `lost`（遥测失联，保留占用）。RPC 成功，包括手动 99 / stop_custom 成功，均不自动证明物理完成/释放；动作列表里的时长、等待时间、FSM、task_id 和底盘零速度也不作为证明。真实上肢占用保留在资源管理器内，防止关节/底盘接管。

在没有可证明的完成反馈时，通过上述只读隔离与现场确认释放本机会话资源，不必为了退出额外派发动作 99。状态 `idle` 只表示本会话没有上肢请求，不证明外部任务不存在。Mock 使用明确的模拟完成结果释放普通动作，ID 15 模拟持续保持；`completed_mock` 仅代表模拟结果，不是实机完成判断。实机自然结束、持续保持及状态话题的类型/时序仍需后续只读抓取和现场验收，不能宣称已经实现自动完成识别。

## A2 关节增益

PC 真实 A2 控制会话默认使用编译进程序的实验增益：12 个关节均为 `kp=60、kd=5`，录制为 `kp=0、kd=1`。新电脑编译后无需另拷贝配置文件或增加启动参数。这是此前选用的 GO2 参考实验值，并非 A2 官方或已验收增益；实验来源保留在配置元数据与本说明中，调试台不显示增益提示条。仍须现场人员评估并完成验证；“进入底层模式”经确认后先检查状态、按需趴下并持续确认趴稳，再停用 `ai_sport`；失败、过期或超时保留运控。“退出底层模式”先释放 LowCmd 再恢复 `ai_sport`，不附带起身动作。

需要覆盖内置值时，使用经过现场确认的本机增益文件启动：

```bash
bash scripts/pc.sh start --a2-joint-gains /absolute/path/to/a2-gains.json
```

显式文件优先，文件无效时关闭关节调试/示教，不回退内置值。复用已有 A2 JSON 校验和安全互锁；Mock 未指定文件时仍用零增益。PC 离线和只读恢复 worker 不加载增益、不创建关节控制。机器人端启动仍须显式提供增益文件。

A2 底层入口的单次 `StandDown` RPC 最多等待 8 秒，结束后恢复普通运动 RPC 的 1 秒超时。仍超时（3104）时单独提示 `stand_down_rpc_timeout`，不重试、不释放 ai_sport；RPC 成功后仍须通过新鲜趴下状态和关节稳定确认。
