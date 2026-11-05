# GO2 部署与 Web 使用教程

[English](go2-guide.en.md) · [通用部署指南](deployment-dependencies.md)

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

常用模式集中显示起立、灵动模式、停止移动、趴下；起立后可直接选择灵动模式，不再显示平衡站立入口。

常规行走（StaticWalk）、常规跑步（TrotRun）和续航（EconomicGait）当前未验证为可独立切换的模式，已从Web和后端命令白名单撤下；接口返回0不代表实际进入目标模式。

同一二进制会编译所有已注册产品，因此 GO2 的现有 SDK 也必须包含项目需要的 R1 Audio/Loco 头文件；缺失时应由维护人员提供匹配固件/ABI 的完整 SDK，不通过删掉产品代码绕过。

## 2. 新部署默认会安装什么

使用**包含本次更新的源码**执行 `--product go2`：

- 复用 `/usr/local` 已安装且兼容的SDK2，不自动覆盖SDK；DDS默认 `eth0`，部署前检查实际网卡。
- 自动准备 `kokoro-multi-lang-v1_1.tar.bz2`，校验 `tts_models/kokoro-multi-lang-v1_1/model.onnx` 及配套文件，做无声短句/长句合成检查。
- 安装 `go2-local-tts.service`，启动参数指向上述FP32目录。GO2不会使用G1的INT8模型或G1的TTS service。
- 同时提供system/user unit；用户级持久部署需要Linger=yes。默认不安装RealSense、相机特权helper，不声明可选雷达或真实导航。
- 保留当前 `config/`、`build/`、`tts_models/` 与 `tts_kokoro/`；模型压缩包由产品资源步骤单独传入。若源码有未提交修改，正式在线/传包流程会停止，不覆盖成果；不要用 `reset --hard` 绕过。

只复制新unit不复制模型不够；旧GitHub版本、旧安装器、仅同步Web静态文件，也不会自动升级FP32。更新尚未提交/发布时，`install` / `from-pc` 从远端仓库取得的版本仍可能是旧版。`--without-kokoro` 是显式跳过本地TTS安装；新装GO2此时没有原生TTS回退，不会自动得到完整播报能力。

## 3. 部署步骤

### 3.1 准备与校时

准备一台可二次开发的 GO2 拓展计算单元：Ubuntu 20.04 AArch64、`unitree` 用户、兼容 SDK2 已安装到 `/usr/local`。电脑和拓展单元应网络互通。默认 SSH 地址为 `unitree@192.168.123.18`（拓展计算单元），部署前确认现场地址与有线 DDS 网卡；地址不同时再覆盖，不使用运控单元地址。不要加载 ROS 环境。

机器人执行 `ip -br -4 addr` 核对网卡。仓库默认 `eth0`；若现场不同，先由维护人员统一修改 `scripts/deploy_product.sh` 与 `deploy/go2-web-control{,.user}.service` 的接口配置并验证，不能通过传入不存在的 `--interface` 部署参数解决。

先按[通用前置检查](deployment-dependencies.md#1-唯一推荐入口)备好电脑工具并 clone 项目。机器人终端可通过 `ssh unitree@192.168.123.18` 登录；电脑终端指本机 Linux shell，两处命令不要混用。

离线机器或无效 RTC 可能导致开机时间回退。每次部署前先确认电脑时间可靠，再在电脑执行以下命令，校正机器人时间并保存 systemd-timesyncd 基准。先配置已有 SSH key/agent，首次连接核对主机指纹；该步骤需要远端免密 sudo：

```bash
(
set -e
GO2_HOST='unitree@192.168.123.18' # 现场地址不同时修改此值
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

确认身份为 `unitree`、架构为 `aarch64`，校时和持久基准更新成功，时差不超过5秒后再继续。任一步失败均停止；认证不足由现场维护人员通过受控认证处理，不在聊天或命令中填写密码。不执行 `hwclock -s`，不替换时间服务。安装脚本不会代替这一步自动校时。GO2 的 `from-pc` 会只读检查时差≤5秒、持久基准更新时间在最近300秒内；请在执行 `from-pc` 前5分钟内完成校时。该窗口是部署脚本策略，不是固件要求。

### 3.2 方式 A：机器人可联网

使用包含 GO2 支持的已发布源码；首次在机器人执行：

```bash
git clone https://github.com/ershui2500/UniRoboGui.git /home/unitree/UniRoboGui
cd /home/unitree/UniRoboGui
bash scripts/deploy.sh check --product go2
bash scripts/deploy.sh install --product go2
```

已有目录时跳过 clone，先检查 `git status --short` 并保留现有改动，再运行 check/install。机器人需能访问 GitHub、模型下载站点、PyPI；缺系统包时还需 Ubuntu 软件源。安装会 Release 构建、完整 CTest、ldd 检查，安装选定范围的 Web/TTS unit 并做只读验收。不能从旧远端版本获得尚未发布的本地修改。

### 3.3 方式 B：联网电脑传输

在能访问 GitHub/模型/PyPI 且能 SSH 到 GO2 的 Ubuntu/Linux 电脑上先 clone。已有目录时跳过 clone，先检查 `git status --short`，不要覆盖目录。进入项目后，先在电脑执行3.1的校时步骤，再于5分钟内执行 `from-pc`：

```bash
git clone https://github.com/ershui2500/UniRoboGui.git
cd UniRoboGui
# 现在先执行3.1的电脑端校时步骤，再运行：
bash scripts/deploy.sh from-pc --product go2
```

GO2 默认连接 `unitree@192.168.123.18`，无需填写 `--host` 或创建开发机别名。现场地址不同时，可用 `bash scripts/deploy.sh from-pc --product go2 --host unitree@<GO2_IP>` 覆盖；也可填写已有 SSH Host 别名。脚本更新电脑上的干净 checkout、准备 FP32 模型与 aarch64/Python3.8 wheelhouse，再传输到机器人进行构建和安装。它不是完整离线系统镜像；机器人缺失的 Ubuntu 20.04 arm64 系统包仍需预装或提供兼容来源。不要用 x86_64 wheel 或软件包替代。

如电脑需要代理，只在电脑设置真实可用的代理地址；`no_proxy` 应包含 localhost、127.0.0.1 与实际机器人地址。不要把机器人自己的127.0.0.1当作电脑代理，不修改 DDS 网卡来获取外网。

### 3.4 仅准备 TTS

在机器人终端进入 `/home/unitree/UniRoboGui` 后，仅准备/修复TTS资源时使用同一个安装器：

```bash
bash scripts/install_kokoro_tts.sh --product go2
# --help 仅显示对应产品默认模型，不下载、不安装
bash scripts/install_kokoro_tts.sh --product go2 --help
```

该命令准备虚拟环境与模型，不安装/重启Web或systemd unit；完整安装使用统一部署入口。模型目录混入 `model.int8.onnx` 会拒绝继续，不能用重命名伪装模型精度。

## 4. Web与只读验收

在电脑浏览器打开 `http://192.168.123.18:8080`（现场地址不同则替换），进入“语音交互 → TTS文本播报”。先保持当前音量，输入短句，再测试长句、数字、中英混读和连续播报。测试前告知旁人文本与音量，确认末句完整读完、无明显断续。不要用 `paplay` 或拓展板耳机输出代替机身扬声器验证。

以下命令在**机器人终端**执行；`127.0.0.1` 指机器人，不是电脑。

```bash
curl --noproxy '*' -fsS http://127.0.0.1:8080/api/health
curl --noproxy '*' -fsS http://127.0.0.1:8765/health
curl --noproxy '*' -fsS http://127.0.0.1:8080/api/voice/status
curl --noproxy '*' -fsS http://127.0.0.1:8080/api/robot/manifest
curl --noproxy '*' -fsS http://127.0.0.1:8080/api/control/status
systemctl --user show go2-local-tts.service -p ExecStart -p ActiveState
# system安装则使用systemctl，不加--user
```

确认TTS服务参数含 `kokoro-multi-lang-v1_1 --sid 3 --speed 1.2 --threads 8`、模型为 `model.onnx`、语音无初始化错误、Manifest为GO2。health成功只证明服务可响应，不证明声音内容完整；若LLM未配置或可选传感器离线，整体health可能degraded，不能因此判断TTS失败。

Web显示“请求处理完成，请确认实际声音”，不声称硬件已经自动确认播放完毕。GO2没有可靠播放结束事件，因此保留保护等待；请求总耗时包含合成、上传、播放和等待，不等于合成时长。

## 5. 升级、维护与排障

日常启动已安装服务可在机器人运行 `bash scripts/start_services.sh --product go2`。该入口要求显式产品，只启动已安装的精确 unit，不安装、不重启已运行服务。

升级复用方式 A/B，保留 `config/`、`build/`、`tts_models/`、`tts_kokoro/`。先备份客户配置到受控位置，不将含密钥的配置放进 Git 或公开附件。源码有未提交修改时先由维护人员合并保留，不能用 `reset --hard` 绕过检查。升级失败时先保留日志，确认精确服务状态；需要回退时由维护人员恢复已验证版本并按同一流程重新构建验收，不直接覆盖正在运行的二进制。

查看日志与模板使用下列只读命令；用户级安装给每个 systemctl/journalctl 命令加 `--user`：

```bash
systemctl show go2-web-control.service -p MainPID -p ExecStart -p ActiveState -p SubState
systemctl show go2-local-tts.service -p ExecStart -p ActiveState
journalctl -u go2-web-control.service -n 80 --no-pager
journalctl -u go2-local-tts.service -n 80 --no-pager
ss -ltnp 'sport = :8080'
```

仅在实际 unit/MainPID 属于 GO2、接口正确，`/api/control/status` 为 `motion.active=false`、`state=stopped`、`vx/vy/vyaw=0` 且没有调试/示教占用时，现场人员才可重启精确服务。系统级使用 `sudo systemctl restart go2-web-control.service`，用户级使用 `systemctl --user restart go2-web-control.service`。重启后复查新 MainPID、8080唯一监听、health、GO2 Manifest、DDS source 和停止/全零状态。不要调用运动 API 来帮助停止，也不要使用 pkill/killall。

- **无声/`local_tts_silent_pcm`**：检查是否仍指向INT8或旧脚本；当前旧INT8配置曾输出NaN并被转成静音。新代码拒绝NaN/Inf，不用更多静音填充掩盖。
- **`local_tts_synthesis_transfer_failed_*`**：检查本地TTS日志、模型资源与合成异常，不能只看HTTP响应头200。
- **AudioHub enter/upload/exit失败**：保留阶段、块号和原始返回码；不要改成G1/R1的音频topic或任意LowCmd。
- **音量3104**：表示请求超时，先只读复查当前状态；不能只凭遗留启动错误判断扬声器失效。
- **重启**：先核对精确unit、MainPID、产品和停止/全零状态；只重启目标服务，不调用运动API“帮助停止”。

Melo/Matcha仅完成候选评估，未设为默认；G1/R1也没有因本次GO2修改自动换模型。
