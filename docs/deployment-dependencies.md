# G1 PC2 依赖安装与离线部署

本文档用于安装项目必需的宇树 SDK2 和 librealsense2，以及处理 G1 PC2 无法直接
访问 GitHub/PyPI 的情况。README 只保留最短项目部署流程。

正常客户部署优先使用自动脚本，不需要逐条执行本文命令：

- 机器人可以访问 GitHub：在机器人运行 `bash scripts/deploy_g1_online.sh`；
- 机器人不能访问 GitHub：在联网 Ubuntu/Linux 电脑运行 `bash scripts/deploy_g1_from_pc.sh`。

本文后续步骤保留为脚本实现依据、人工排障和特殊现场的手工兜底。

G1 头部标配 RealSense D435i，因此本项目的生产部署必须安装 librealsense2。
SDK2 不固定历史提交，每次部署或升级前都更新到宇树官方仓库
当前最新版。

官方资料：

- [G1 获取 SDK](https://support.unitree.com/home/zh/G1_developer/get_sdk)
- [G1 深度相机例程](https://support.unitree.com/home/zh/G1_developer/depth_camera_instruction)
- [unitree_sdk2 官方仓库](https://github.com/unitreerobotics/unitree_sdk2)
- [librealsense 官方仓库](https://github.com/realsenseai/librealsense)

## 部署边界

- 目标系统：G1 PC2，Ubuntu 20.04 AArch64，`unitree` 用户；
- SSH 登录后的 ROS 选择直接按回车；
- 不 source Foxy/Noetic/ROS 2，不设置 RMW/CycloneDDS；
- `eth0` 保留给 SDK2 DDS，`wlan0` 用于外网；
- 不运行 SDK2 运动例程，不触发任何实机动作。

## 1. 安装编译工具

在 G1 PC2 的新 SSH 会话中执行：

```bash
test -z "${ROS_DISTRO:-}"
test -z "${RMW_IMPLEMENTATION:-}"
test -z "${CYCLONEDDS_URI:-}"

sudo apt-get update \
  -o Dir::Etc::sourcelist="sources.list" \
  -o Dir::Etc::sourceparts="-" \
  -o APT::Get::List-Cleanup="0"
sudo apt-get install -y --no-install-recommends \
  build-essential cmake pkg-config git curl bzip2 rsync \
  python3 python3-pip \
  libboost-system-dev libboost-thread-dev libjsoncpp-dev \
  libcurl4-openssl-dev libopencv-dev libzmq3-dev \
  libusb-1.0-0-dev libssl-dev libudev-dev
```

这些包与当前自动部署脚本保持一致。脚本会先检查安装状态：如果全部已存在，就不会执行
`apt-get update/install`，因此机器人即使没有公网也可以继续使用已经传入的 GitHub/PyPI 资源。
只有确实缺包时才需要 Ubuntu 软件源。

这些命令只使用 Ubuntu 主软件源，不加载机器人中已配置的 ROS 软件源。如果 APT
失败，先检查 `wlan0`、DNS 和 Ubuntu 镜像，不要盲目执行 `apt --fix-broken install`。

## 2. 安装官方最新 SDK2

自动安装器有两种 SDK2 前缀：交互 sudo 可用时使用系统级 `/opt/unitree_robotics`；没有可交互 sudo、但系统包齐全且 `systemctl --user` 可用并且 `Linger=yes` 时，可使用持久用户级 `/home/unitree/.local/unitree_robotics`。正常客户优先让脚本自动选择，不需要手工指定。

系统级首次安装：

```bash
git clone --depth 1 https://github.com/unitreerobotics/unitree_sdk2.git \
  /home/unitree/unitree_sdk2
cd /home/unitree/unitree_sdk2
git rev-parse HEAD

cmake -S . -B build-g1 \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics
cmake --build build-g1 --parallel 2
sudo cmake --install build-g1
sudo ldconfig
```

如果 `/home/unitree/unitree_sdk2` 已存在，不要重新 clone，而是先检查再更新：

```bash
cd /home/unitree/unitree_sdk2
git remote get-url origin
git status --short
git pull --ff-only
git rev-parse HEAD

cmake -S . -B build-g1 \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics
cmake --build build-g1 --parallel 2
sudo cmake --install build-g1
sudo ldconfig
```

`git remote get-url origin` 应指向宇树官方 `unitreerobotics/unitree_sdk2`。
`git status --short` 有输出时立即停止，先确认并保留现场修改，不得直接覆盖。
`git rev-parse HEAD` 记录本次实际安装的最新提交，不在文档中写死易过期的版本号。

## 3. 安装 librealsense2

部分 G1 PC2 镜像已经带有 librealsense2，例如本项目兼容检查会识别
`/opt/ros/noetic/lib/aarch64-linux-gnu/cmake/realsense2/realsense2Config.cmake`。这只表示复用其中的
librealsense2 库，**不需要也不得 source Noetic**。自动部署检测到可用版本时会直接复用；只有缺失时才从源码安装：

```bash
git clone --depth 1 https://github.com/realsenseai/librealsense.git \
  /home/unitree/librealsense
cd /home/unitree/librealsense

cmake -S . -B build-g1 \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_GRAPHICAL_EXAMPLES=OFF \
  -DBUILD_PYTHON_BINDINGS=OFF \
  -DFORCE_RSUSB_BACKEND=ON
cmake --build build-g1 --parallel 2
sudo cmake --install build-g1
sudo install -m 0644 config/99-realsense-libusb.rules \
  /etc/udev/rules.d/99-realsense-libusb.rules
sudo udevadm control --reload-rules
sudo ldconfig
```

已有源码时先确认 `git status --short` 为空，再执行 `git pull --ff-only`，然后重复上面的
CMake 构建和安装命令。`FORCE_RSUSB_BACKEND=ON` 不需要 ROS，也不依赖固定的
`/dev/videoN` 编号。

检查两项依赖：

```bash
test -f /opt/unitree_robotics/lib/cmake/unitree_sdk2/unitree_sdk2Config.cmake || \
  test -f /home/unitree/.local/unitree_robotics/lib/cmake/unitree_sdk2/unitree_sdk2Config.cmake
pkg-config --modversion realsense2 2>/dev/null || \
  test -f /opt/ros/noetic/lib/aarch64-linux-gnu/cmake/realsense2/realsense2Config.cmake
```

## 4. 安装 UniRoboGui

```bash
cd /home/unitree/UniRoboGui
bash scripts/install_g1.sh --check-only
bash scripts/install_g1.sh
```

安装器会编译 UniRoboGui、运行 CTest 和动态库检查，然后安装并启动服务。默认 `auto` 模式会优先使用可交互 sudo 的系统级安装；如果没有可交互 sudo，但系统包已齐全、用户 systemd 可用且 `Linger=yes`，会自动改用 `/home/unitree/.local/unitree_robotics` + `systemctl --user` 的持久用户级安装。也可以用 `--system-install` 或 `--user-install` 显式指定。

它可以重复执行，不会覆盖 `config/customer_voice.json` 或 `config/joint_teach_actions.json`。替换正在运行的 Web 服务前必须确认运动状态为 stopped 且速度为 0。

## 5. G1 PC2 无法访问 GitHub

在能访问 GitHub 的 Ubuntu/Linux 电脑上准备项目、最新 SDK2、librealsense 和 Kokoro 模型：

```bash
git clone https://github.com/ershui2500/UniRoboGui.git
git clone --depth 1 https://github.com/unitreerobotics/unitree_sdk2.git
git clone --depth 1 https://github.com/realsenseai/librealsense.git
curl -fL -o kokoro-int8-multi-lang-v1_1.tar.bz2 \
  https://github.com/k2-fsa/sherpa-onnx/releases/download/tts-models/kokoro-int8-multi-lang-v1_1.tar.bz2
```

让该电脑接入机器人 `192.168.123.0/24` 有线网络，再传入 PC2：

```bash
rsync -az --exclude='/.git/' --exclude='/build/' \
  UniRoboGui/ unitree@192.168.123.164:/home/unitree/UniRoboGui/
rsync -az unitree_sdk2/ unitree@192.168.123.164:/home/unitree/unitree_sdk2/
rsync -az librealsense/ unitree@192.168.123.164:/home/unitree/librealsense/
ssh unitree@192.168.123.164 'mkdir -p /home/unitree/unitree_interface/tts_models'
scp kokoro-int8-multi-lang-v1_1.tar.bz2 \
  unitree@192.168.123.164:/home/unitree/unitree_interface/tts_models/
```

回到 G1 PC2 后，按本文第 1～4 节执行；第 2、3 节跳过 clone 和 pull，从
`git rev-parse HEAD` 与 CMake 构建开始执行。
离线传入的 SDK2 必须是联网电脑刚从宇树官方仓库取得的最新版。

注意：这里的“无法访问 GitHub”不等于“完全不需要网络”。如果第 1 节列出的 Ubuntu 系统包有缺失，
机器人仍需访问 Ubuntu 20.04 软件源。先运行下面的只读检查可以提前看到缺失项：

```bash
cd /home/unitree/UniRoboGui
bash scripts/install_g1.sh --check-only
```

如果系统包已经齐全，自动安装会跳过 APT。若系统包有缺失但机器人完全不能访问 Ubuntu 软件源，
应先按现场的软件分发方式安装对应的 arm64 Ubuntu 20.04 `.deb` 及其依赖；不要拿 x86_64 或其他 Ubuntu
版本的包代替。

如果 PC2 也不能访问 PyPI，在联网电脑准备适用于 G1 AArch64/Python 3.8 的 Kokoro wheelhouse：

```bash
mkdir -p wheelhouse-aarch64-py38
python3 -m pip download --dest wheelhouse-aarch64-py38 \
  --only-binary=:all: \
  --platform manylinux2014_aarch64 \
  --python-version 38 \
  --implementation cp \
  --abi cp38 \
  'virtualenv==20.26.6' 'pip<25' 'setuptools<76' wheel \
  'numpy<2' 'sherpa-onnx==1.13.4'
rsync -az wheelhouse-aarch64-py38/ \
  unitree@192.168.123.164:/home/unitree/unirobogui-wheelhouse/
```

机器人上使用：

```bash
KOKORO_WHEELHOUSE=/home/unitree/unirobogui-wheelhouse \
  bash scripts/install_g1.sh
```

正常客户部署优先使用 `scripts/deploy_g1_from_pc.sh`，它已经自动完成上述模型和 wheelhouse 准备，
手工命令只用于排障兜底。

## 6. 验收与常见错误

```bash
systemctl is-active g1-web-control.service 2>/dev/null || \
  systemctl --user is-active g1-web-control.service
systemctl is-active g1-local-tts.service 2>/dev/null || \
  systemctl --user is-active g1-local-tts.service
curl --noproxy '*' -fsS http://127.0.0.1:8080/api/health
curl --noproxy '*' -fsS http://127.0.0.1:8765/health
ss -ltnp 'sport = :8080 or sport = :8765'
ldd /home/unitree/UniRoboGui/build/g1_web_server | \
  grep -E 'ddsc|ddscxx|jsoncpp|curl|opencv|realsense|not found'
```

- `undefined symbol: ddsi_sertype_v0`：重新 SSH 登录并在 ROS 选择中直接按回车，不要用
  ROS 桥接绕过 SDK2 动态库冲突。
- GitHub 超时：使用本文的离线传包流程，不要在 PC2 上无限重试。
- APT 失败：检查 `wlan0`、DNS 和 Ubuntu 镜像；不要执行 `apt --fix-broken install`、
  网络重置或安装 ROS 环境。
- D435i 未出图：先检查 librealsense2 动态库与 USB 枚举。D435i RGB+Depth 会优先尝试与机器人原相机服务并发共享；共享成功时不会停止 `master_service`。只有并发访问被固件拒绝时才回退到特权 helper + V4L2 路径。`/dev/videoN` 会变化，V4L2 模式不要固定设备号。

旧服务正在运行时，安装器只会在控制状态返回零速度后才切换服务。
