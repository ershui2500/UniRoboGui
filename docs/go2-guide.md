# GO2 部署与 Web 使用教程

[English](go2-guide.en.md) · [通用部署指南](deployment-dependencies.md) · [TTS 诊断与实测](voice/GO2-TTS-诊断与离线方案评估.md)

适用于本项目已注册的 GO2 二次开发变体，运行身份为 `product_id=go2`、内部 `variant=go2_edu`。页面显示 Unitree GO2 不会改变协议标识。不要将其他 GO2 型号或其他机器人的能力直接类推到当前设备。

## 1. 当前功能与边界

| 功能 | 当前实现与使用边界 |
|---|---|
| 状态与姿态 | SDK2 LowState / SportModeState、12关节、内嵌BMS、里程计、官方URDF；过期数据标记离线，不用默认零位冒充 |
| 诊断 | 原始故障/未知字段保留；`rt/errorlist` 未收到数据不代表无故障，运动状态编号不是故障码 |
| RGB | 固定原生 RTP/H264 来源 `udp://230.1.1.1:1720`；不允许浏览器配置任意 pipeline |
| 深度相机 | 当前基础配置不支持，面板隐藏；不能因有RGB就声称深度可用 |
| 机器人控制 | GO2独立速度/模式/动作白名单，受状态、限速、租约和确认约束；开发回归用Mock，真实动作由现场操作员验收 |
| 关节调试、示教 | 仅全身12个有效关节，20槽消息的12–19保留；500Hz受控LowCmd，检测高层释放但不自动抢占。录制会去掉支撑，必须可靠固定并逐项授权 |
| 遥控示教绑定 | 共享录制/播放与绑定逻辑；GO2显示F2，持久化标识保持共享约定，不能复制G1标签改变协议 |
| TTS与音量 | 机载离线 Kokoro FP32 + sherpa-onnx，原音色sid=3、speed=1.2、总线程预算8；AudioHub整句播放，VUI音量 |
| ASR/内置语音助手 | 当前未接入ASR，无原生TTS或内置LLM回退；不能把页面入口当作支持 |
| 客户大模型 | 使用共享配置与文本消息流程；需用户配置兼容API，GO2语音唤醒不可用。离线TTS不等于LLM可离线 |
| LiDAR / SLAM | L2只读数据独立于建图导航。SLAM还需扩展坞、Mid-360或XT16、固件/服务与显式部署声明；默认不启用真实导航 |

Manifest/Capability 是运行时真值。现有 Profile 的 Mock/operator-validation 等级不会因一次播报成功自动升级；页面解锁仅防误触，没有访问认证。

## 2. 新部署默认会安装什么

使用**包含本次更新的源码**执行 `--product go2`：

- 复用 `/usr/local` 已安装且兼容的SDK2，不自动覆盖SDK；DDS默认 `eth0`，部署前检查实际网卡。
- 自动准备 `kokoro-multi-lang-v1_1.tar.bz2`，校验 `tts_models/kokoro-multi-lang-v1_1/model.onnx` 及配套文件，做无声短句/长句合成检查。
- 安装 `go2-local-tts.service`，启动参数指向上述FP32目录。GO2不会使用G1的INT8模型或G1的TTS service。
- 同时提供system/user unit；用户级持久部署需要Linger=yes。默认不安装RealSense、相机特权helper，不声明可选雷达或真实导航。
- 保留当前 `config/`、`build/`、`tts_models/` 与 `tts_kokoro/`；模型压缩包由产品资源步骤单独传入。若源码有未提交修改，正式在线/传包流程会停止，不覆盖成果；不要用 `reset --hard` 绕过。

只复制新unit不复制模型不够；旧GitHub版本、旧安装器、仅同步Web静态文件，也不会自动升级FP32。更新尚未提交/发布时，`install` / `from-pc` 从远端仓库取得的版本仍可能是旧版。`--without-kokoro` 是显式跳过本地TTS安装；新装GO2此时没有原生TTS回退，不会自动得到完整播报能力。

## 3. 部署步骤

先确认GO2型号、固件、直连网卡、兼容SDK与可靠系统时间。无效RTC/离线重启可能使时间回退；由可信开发机校正，并确认系统时间与持久时间基准已更新，再部署。不要用 `hwclock -s` 覆盖正确时间，不加载ROS环境。

在机器人上，只读检查与在线部署：

```bash
cd /home/unitree/UniRoboGui
bash scripts/deploy.sh check --product go2
# 确保新源码已发布、机器人可访问GitHub/PyPI，且现有源码没有未提交修改
bash scripts/deploy.sh install --product go2
```

若机器人无法联网，在开发电脑准备资源并传输：

```bash
# 代理仅设置在这台开发电脑；不要把机器人自己的127.0.0.1当作电脑代理
export http_proxy=http://127.0.0.1:7890
export https_proxy=http://127.0.0.1:7890
export no_proxy=localhost,127.0.0.1,192.168.123.0/24
bash scripts/deploy.sh from-pc --product go2 --host unirobo-go2
```

`unirobo-go2` 是本开发环境的SSH别名；另一台电脑可用 `--host unitree@<实际拓展计算单元地址>`。不使用运控单元地址，不在聊天或命令参数中填写密码。模型与aarch64/Python3.8 wheelhouse由电脑准备，机器人端不需要访问模型站点。公网代理不能套到本地HTTP或DDS链路上。

仅准备/修复TTS资源时，仍使用同一个安装器：

```bash
bash scripts/install_kokoro_tts.sh --product go2
# --help 仅显示对应产品默认模型，不下载、不安装
bash scripts/install_kokoro_tts.sh --product go2 --help
```

该命令准备虚拟环境与模型，不安装/重启Web或systemd unit；完整安装使用统一部署入口。模型目录混入 `model.int8.onnx` 会拒绝继续，不能用重命名伪装模型精度。

## 4. Web与只读验收

打开机器人的8080页面，进入“语音交互 → TTS文本播报”。先保持当前音量，输入短句，再测试长句、数字、中英混读和连续播报。测试前告知旁人文本与音量，确认末句完整读完、无明显断续。不要用 `paplay` 或拓展板耳机输出代替机身扬声器验证。

```bash
curl --noproxy '*' -fsS http://127.0.0.1:8765/health
curl --noproxy '*' -fsS http://127.0.0.1:8080/api/voice/status
curl --noproxy '*' -fsS http://127.0.0.1:8080/api/robot/manifest
curl --noproxy '*' -fsS http://127.0.0.1:8080/api/control/status
systemctl --user show go2-local-tts.service -p ExecStart -p ActiveState
# system安装则使用systemctl，不加--user
```

确认TTS服务参数含 `kokoro-multi-lang-v1_1 --sid 3 --speed 1.2 --threads 8`、模型为 `model.onnx`、语音无初始化错误、Manifest为GO2。health成功只证明服务可响应，不证明声音内容完整；若LLM未配置或可选传感器离线，整体health可能degraded，不能因此判断TTS失败。

Web显示“请求处理完成，请确认实际声音”，不声称硬件已经自动确认播放完毕。GO2没有可靠播放结束事件，因此保留保护等待；请求总耗时包含合成、上传、播放和等待，不等于合成时长。

## 5. 已测效果与排障

一次现场验收中，同一句请求总耗时由104.33秒降至23.14秒；短中文FP32合成约0.64–0.76秒。短句、长句、无标点、数字中英混读、连续播报及真实Web按钮播放获现场确认“都完整清楚，没有明显断续”。这是该机器与样例的记录，不是所有设备、任意文本的保证；详见[TTS评估报告](voice/GO2-TTS-诊断与离线方案评估.md)。

- **无声/`local_tts_silent_pcm`**：检查是否仍指向INT8或旧脚本；当前旧INT8配置曾输出NaN并被转成静音。新代码拒绝NaN/Inf，不用更多静音填充掩盖。
- **`local_tts_synthesis_transfer_failed_*`**：检查本地TTS日志、模型资源与合成异常，不能只看HTTP响应头200。
- **AudioHub enter/upload/exit失败**：保留阶段、块号和原始返回码；不要改成G1/R1的音频topic或任意LowCmd。
- **音量3104**：表示请求超时，先只读复查当前状态；不能只凭遗留启动错误判断扬声器失效。
- **重启**：先核对精确unit、MainPID、产品和停止/全零状态；只重启目标服务，不调用运动API“帮助停止”。

Melo/Matcha仅完成候选评估，未设为默认；G1/R1也没有因本次GO2修改自动换模型。
