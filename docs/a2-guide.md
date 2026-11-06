# A2 控制台使用说明

本项目支持普通足式 A2 的控制台：产品选择、官方三维模型、12 关节状态、双电池、运动模式、速度控制、Mock 全身关节调试、语音交互和 JT128 建图导航。综合工作台按 GO2 单摄像头布局显示 RGB 画面与关节姿态。真机关节调试需提供增益配置并由现场操作员验证；机器人端运动与导航入口供现场操作员逐项验证，`mock_verified` 不代表真实控制已验收。

部署安装以 [README 快速部署](../README.md#quick-deploy) 为主：统一使用 `scripts/deploy.sh`，选择 `--product a2`。本文补充功能和安全互锁。

## 1. 启动和查看

先按 [README 快速部署](../README.md#quick-deploy) 完成安装。在能访问 GitHub 且能 SSH 到机器人的电脑上，进入项目根目录运行：

```bash
bash scripts/deploy.sh from-pc --product a2
```

默认连接 A2 用户开发单元；地址不同或已配置 SSH 别名时，按 README 使用 `--host`。部署完成后，浏览器打开 `http://<机器人IP>:8080`，确认产品显示 **Unitree A2**。

- 综合工作台提供模式与运动控制；机器人状态页显示模型、12 关节、数据来源和两块电池。
- 页面右上角可切换中文 / English。
- 默认部署不提供关节增益配置；真机关节调试入口保持关闭。
- 二进制名称 `g1_web_server` 是历史名称，实际产品由 `--robot a2` 选择。
- 只读部署需在服务参数中配置 `--read-only`，它会关闭运动与 SLAM 命令入口。
- 语音区提供 ASR、Unitree 原生 TTS、可选 Kokoro 和客户模型入口；不提供内置 chat_go、深度相机或头部控制。

## 2. 模式和运动

先解锁本页控制，再使用需要的模式按钮；执行指令需二次确认。页面锁用于防误触，当前 Web 没有访问认证。

- **站高/站低**：对应关节锁定状态；**平衡站立**与锁定站立语义不同。
- **恢复站立**：只允许从阻尼或趴下状态提交。
- **行走/跑步/攀爬**：只在可移动状态切换；不会隐式站立或解锁。
- **侧步、倒立、直立**：从 FSM 3 进入，只在对应 FSM 6/7/8/9 显示退出入口。
- **前空翻、后空翻**：在“高层动作”页显示，只允许从 FSM 3 提交；属于高动态动作，需现场操作员确认安全空间。
- **阻尼**可能失去姿态支撑；真机入口仅供现场操作员在可靠支撑、清场后逐项验证，尚未完成实体运动验收。
- 默认选择行走和低速。启用键盘运动后，W/S 前后、A/D 转向、Q/E 横移，松键停止；失焦、锁定、切页或续租中断走停止路径。

下表来自[官方 A2 高层运动服务接口](https://support.unitree.com/home/zh/A2_SDK_Development_Guide/sport_client_interface)，数值为三个方向的绝对值上限。网页按当前步态使用原生两档，`0=低速、1=高速`：

| 步态 | 档位 | 前后 m/s | 横移 m/s | 转向 rad/s |
|---|---|---:|---:|---:|
| 行走 | 低速 | 0.8 | 0.5 | 2.0 |
| 行走 | 高速 | 1.5 | 0.5 | 2.5 |
| 跑步 | 低速 | 2.5 | 0.3 | 1.5 |
| 跑步 | 高速 | 4.0 | 0.3 | 1.0 |
| 攀爬 | 两档相同 | 0.6 | 0.3 | 1.0 |

以上为接口边界。网页低速预设采用更缓和的速度：行走为前后 0.6 m/s、横移 0.4 m/s、转向 1.5 rad/s；跑步为 2.0 / 0.25 / 1.2；攀爬为 0.5 / 0.25 / 0.8（单位依次同上）。高速预设和后端接口边界保持不变。真机操作由现场操作员逐项验证。后端在提交和执行时按 fresh FSM 重新校验；步态变化后不继续使用超限旧速度。`SpeedLevel` 失败时不发送 `Move`，保留 SDK 返回码。A2 的 `Move` 无时长参数，共享后端按250 ms租约停止未续租运动。

## 3. 状态和双电池

### 关节调试台

`--mock` 运行时，“机器人调试台”按 A2 JointSchema 显示四条腿的 12 个关节，仅提供“全身”模式。滑块先更新数字孪生，只有经过二次确认才进入共享 Mock LowCmd 循环；页面心跳中断或点击“停止 / 释放控制”都会释放调试资源。

A2 官方底层接口使用 `unitree_hg::msg::dds_::LowCmd_` 和 `rt/lowcmd`。真机关节调试和示教需要通过启动参数 `--a2-joint-gains <文件>` 提供增益配置，并由现场操作员逐项验证；未配置或配置无效时入口关闭。配置是 JSON 对象，`joints` 是按 motor slot 0–11 排列的 12 项数组，每项必须包含数值 `kp`、`kd`、`record_kp`、`record_kd`，分别用于控制和录制。程序只校验有限、非负且可表示为 float，这不是安全范围或真机认证；增益须由熟悉 A2 的现场人员评估。

遥控按官方 `rt/wirelesscontroller` 的 `WirelessController_.keys()` 接收，组合键按遥控器外壳显示 F1/F3；F3 对应 SDK 的 F2（bit 7）。按住组合键只触发一次，松开后可再次触发；START 可停止当前关节调试并释放 LowCmd（录制中请先保存）；页面显示累计收帧、键值和最近触发结果。Web 不自动释放或恢复 `ai_sport`；操作员须在可靠支撑和清场后手动停用运控，Web 检测到任何活动运控或无法确认状态时拒绝底层控制。停止/释放只结束 LowCmd，不会自动恢复站立或运控，恢复由现场操作员执行。

真机调试台可显示四腿 12 关节的目标滑块和数字孪生预览；未完成配置时滑块不会发送机器人指令。实际起立、步态与速度操作位于综合工作台，使用独立的 A2 高层运动接口。

12 个语义关节对应 raw slot 0–11，35 槽消息的其余位置不作为关节展示。模型使用官方 A2 URDF，不使用 GO2 模型替代，也不添加零位偏置。

两块电池分别显示编号、来源和状态，不合成为“整机电量”：

| 编号 | DDS 来源 | 遥测来源键 |
|---|---|---|
| ②号 | `rt/lf/bmsstate` | `bms` |
| ①号 | `rt/lf/secondary_bmsstate` | `secondary_bms` |

HTTP/WebSocket 新增 `batteries` 数组，分别包含 `id`、`source`、`topic`、`state` 和电池数据；兼容字段 `battery` 对应②号电池。每块电池独立维护 freshness；缺失或过期时显示状态及 `--`，不使用另一块电池或默认零值补齐。

## 4. 官方依据与待验收项

- [官方 A2 底层接口](https://support.unitree.com/home/zh/A2_SDK_Development_Guide/basic_service_interface)：HG LowState、关节顺序、主板与双 BMS 话题。
- [官方 A2 运控切换服务](https://support.unitree.com/home/zh/A2_SDK_Development_Guide/motion_witcher_service_interface)：通过 `CheckMode` 检测 `ai_sport`；Web 不调用 `ReleaseMode` / `SelectMode`。
- [官方 A2 SportClient](https://github.com/unitreerobotics/unitree_sdk2/blob/7740f8b67e386ab09c3b333187fd5f8582a75ddc/include/unitree/robot/a2/sport/sport_client.hpp)：核对目标设备的 SDK 版本与接口。
- [官方 A2 状态示例](https://github.com/unitreerobotics/unitree_sdk2/blob/7740f8b67e386ab09c3b333187fd5f8582a75ddc/example/a2/sport/a2_sport_state.cpp)：使用 `rt/lf/sportmodestate`，而高层接口正文写 `rt/sportmodestate`。代码固定采用示例话题，不自动回退或混读；其他机器人或固件须分别确认。
- [官方 A2 模型](https://github.com/unitreerobotics/unitree_ros/tree/25746e8c73f097c3b32bed7b6c16d4e195dfacd8/robots/a2_description)：保留目录结构、URDF、引用 mesh 和 BSD 3-Clause 许可证，详见资产目录 `SOURCE.md`。

迁移到其他机器人须先确认实际型号、固件、SDK、网卡、unit、DDS话题和动态库，再完成机器人端 Release/CTest及只读遥测验证。运动能力仍须逐项现场授权和验收，不能因为构建、Mock 或一次 RPC 成功而开启。

## 5. 部署后检查

在机器人终端执行以下只读检查：

```bash
curl --noproxy '*' -fsS http://127.0.0.1:8080/api/health
curl --noproxy '*' -fsS http://127.0.0.1:8080/api/robot/manifest
curl --noproxy '*' -fsS http://127.0.0.1:8080/api/control/status
```

确认 `identity.product_id` 为 `a2`、必需遥测来源在线，且控制状态为 `motion.active=false`、`state=stopped`、三个速度均为零。安装器自动执行 Release 构建与 CTest；Mock 回归通过不代表真实运动已经验收。

## 6. 机器人部署与操作员验证

部署前确认目标设备的型号、SDK、DDS 网卡和服务参数。用户级 unit 模板为 `deploy/a2-web-control.user.service`；只读部署可使用 `--read-only`，需要运动入口时由现场操作员按安全条件逐项验证。

在机器人上查看精确 unit：

```bash
systemctl --user status a2-web-control.service
journalctl _SYSTEMD_USER_UNIT=a2-web-control.service -n 50 --no-pager
```

重启前先通过 `/api/control/status` 确认运动 stopped、inactive、三轴为零，再执行 `systemctl --user restart a2-web-control.service`。

A2 Locomotion 使用 `operator_validation` 策略。打开控制台后，先核对产品为 A2、实时 FSM 与周围环境，再由现场操作员自行解锁和逐项操作。键盘速度控制只在可移动状态开放。阻尼、恢复站立和跑步均有额外确认，但确认框不能替代可靠支撑、清场和急停准备。Web 当前没有身份认证，请将访问范围限制在受控现场网络。

机器人控制入口的开放不改变 `mock_verified` 等级。侧步、倒立、直立及空翻虽有官方 A2 SportClient 接口，但当前仅通过 Mock 验证，具体固件是否接受及实际效果待现场操作员逐项记录。关节调试仅用于 Mock UI/安全链路回归，真机关节调试和示教须由操作员配置并逐项验证；头部及非 RGB 设备功能继续保持关闭。

## 7. RGB 图传与占位布局

综合工作台按 GO2 方式并排显示 RGB 与关节姿态，建图导航区按 JT128 与 SLAM 服务运行证据开放。当前 A2 只有一路 RGB 图传，不显示深度面板。RGB 复用固定 RTP/H.264 接收器，来源为官方 [`230.1.1.1:1720`](https://support.unitree.com/home/zh/A2_SDK_Development_Guide/Multimedia_Services)，通过部署配置的 DDS 网卡接收（默认 `eth0`）；页面不接受任意视频源。

持续收到 fresh RGB 帧后，Manifest 才能将 RGB 能力标为 `readonly_verified`。深度相机不支持；JT128 点云需显式声明附件，见第 9 节。A2 音频接口依据[官方音量灯光服务文档](https://support.unitree.com/home/zh/A2_SDK_Development_Guide/vui_service)，语音发声与收音须在目标设备分别验收。

## 8. 语音交互接入

A2 复用通用语音页面和 HTTP/WebSocket 服务，底层使用 A2 SDK `AudioClient`。页面提供本地 ASR、Unitree 原生 TTS、Kokoro 本地 TTS 和 0–100% 音量控制。Kokoro 使用离线 FP32 中英模型生成 16 kHz 单声道 PCM，再通过 A2 `AudioClient::PlayStream` 播放；Unitree 原生 TTS 仍是默认引擎。首次准备模型和运行时：

```bash
bash scripts/install_kokoro_tts.sh --product a2
```

然后用 A2 产品安装流程安装并启动 `a2-local-tts.service`（user scope 或 system scope）；检查 `http://127.0.0.1:8765/health` 正常后，Web 的 Kokoro 引擎才可完整使用。电脑部署会准备目标架构对应的 wheelhouse 与 FP32 模型，详见 `scripts/deploy.sh from-pc --product a2`。大模型互动需另行配置客户 API，不显示 A2 未证实的内置 chat_go。官方要求 Vui Service 高于 1.253.0.3；实际版本和扬声器、麦克风效果须在目标设备验收。接口返回成功不代表声音可闻或完整，Audio Manifest 仍为 `mock_verified`。

## 9. JT128 建图与导航接入

[A2 官方 SLAM 与导航接口](https://support.unitree.com/home/zh/A2_SDK_Development_Guide/slam_and_navigation_service_interface)提供 `slam_operate` API 1801/1802/1804/1102/1201/1202/1901 及建图、定位、全局地图、任务信息 DDS 话题。Web 复用共享感知服务，地图名限定 `test1.pcd` 至 `test10.pcd`；单次导航目标或多点路线的每段距离不得超过 30 m。官方适用范围为特征丰富的静态室内平地，X/Y 小于 45 m；机器人可能沿直线行走，低于 30 cm 的障碍物未必可靠识别。

仅 JT128 机型且 SLAM NAV 服务高于 1.0.0 时声明 `--device jt128`。本机 `unitree_slam` 是外部服务，Web 只检查服务/RPC，不启停；真实导航另需 `--enable-navigation`，`--read-only` 会禁用 SLAM 命令入口。附件声明、外部服务或 RPC 任何一项缺失时建图导航入口关闭。声明 JT128 后，未加载地图时也会订阅官方融合雷达点云用于实时预览。第二台 JT128 的单独地址可达性不作硬门禁，不能因此断言雷达与点云完整健康。

页面确认框只防误触，Web 没有认证。真机建图、保存和定位须按授权及运动停止条件验证；真实导航由现场操作员按环境、人员清场和急停准备逐项验收，自动回归只使用 Mock。收到点云不代表实体导航已经验证。

若地图显示“暂无 SLAM 点云”，先检查 `/api/perception/frame` 是否持续返回空帧，再检查 `unitree_slam` 的雷达或建图 DDS 输出。预览使用[官方雷达接口](https://support.unitree.com/home/zh/A2_SDK_Development_Guide/lidar_service_interface)的 `rt/unitree/slam_lidar/points`，按官方结构设计外参转换到机身坐标系；它不是标定外参。实时最多 3600 点，累积地图最多 30000 点，与 G1 共用过滤和压缩逻辑；建图和定位数据在线时优先显示任务点云。原始预览不会启动建图，也不启用未经验证的低障碍自动控制。地图显示在 URDF 加载后隐藏高于机器人实时顶部 30 cm 的点云，障碍柱使用相同高度过滤；高度随机器人位姿更新。此过滤仅影响 Web 可视化，不裁剪保存的地图，也不改变官方导航避障策略。若 SDK 返回 `3203`，其含义是 API 未实现，但外部 SLAM 服务可能已经收到并执行请求，不要反复点击。应对照 `unitree_slam` 日志核查对应 API 的实际执行结果、SLAM NAV 安装是否完整以及官方 OTA 版本；已确认的 3203 空回执列入 A2 显示白名单，不显示常驻异常提醒；原始码仍保留在诊断信息中。其他错误码或任务失败仍正常提示。

若 API 返回 `3203` 且响应为空，使用 A2 Profile 的 `empty_rpc_warning_whitelist` 过滤已知回执提醒，保留原始诊断数据，不关闭已通过 `--enable-navigation` 开启的操作员入口。建图/定位由请求后的新鲜 DDS、保存由新地图文件、关闭由数据流停止证据确认；导航/暂停/恢复/取消遇到同类空回执时显示“请求已发送”，允许继续手动操作；不自动重试，不把请求已发送冒充执行成功。其他错误及位姿新鲜度、距离和部署限制仍生效。此兼容处理不代表真实导航已验收。A2 Web 的 system/user unit 在网卡就绪后最多等待 30 秒让外部 `unitree_slam` 进程出现，再额外等待 3 秒完成服务初始化；外部服务缺失时仍启动其他 Web 功能并显示 SLAM 降级。此等待不代替运行时服务健康检查。
