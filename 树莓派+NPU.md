# Raspberry Pi 5 与 Hailo-8L 环境配置与部署

本文说明本项目的硬件、软件依赖及首次部署步骤。项目功能与状态见 [README](readme.md)，各程序的完整命令见 [编译运行命令总手册](编译运行命令总手册.md)。

> 发布范围：本文以电脑端当前适配源码为依据。本次仅更新两份说明文档，未同步 GitHub 默认分支上的其他源码；部署前需确认所用源码版本包含本文所述入口和参数。

这里的“树莓派 + NPU”具体指 **Raspberry Pi 5 主机通过 PCIe 使用 Hailo-8L AI 加速器**。CPU 运行图像处理、解算、跟踪和通信；Hailo-8L 执行模型推理；HailoRT 是主机侧调用加速器的软件运行时；HEF 是部署到 Hailo 的模型文件。

## 1. 环境基线

### 1.1 硬件

| 部件 | 当前配置 | 对部署的影响 |
| --- | --- | --- |
| 主机 | Raspberry Pi 5，四核 Cortex-A76，2 GiB 内存 | 使用 ARM64 程序与 SDK；编译先用 `-j2` |
| AI 加速器 | Hailo-8L，PCIe 接入 | 需要匹配的驱动、固件和 HailoRT |
| 相机 | Hikrobot MV-CS016-10UC，USB 连接 | 使用海康 ARM64 SDK；按序列号选设备 |
| 镜头 | 8 mm | 更换镜头或对焦后应重新评估内参 |
| 下位机 | RoboMaster C 板，QDU 固件 | 视觉侧使用 USB CDC / SharedTopic；需与电控确认接线和协议 |
| 调试电脑 | Windows，局域网 SSH 与浏览器连接 Pi | 不要求 Pi 安装图形桌面 |

现有相机序列号为 `DA6589759`。这是测试机设备信息，新相机需要修改 YAML 中的 `serial_number`。

当前通信路径在主机上表现为 USB CDC ACM 串口；板间 UART、USB 转接板和调试器的具体接法以实物和电控工程为准，不能只凭 USB 外形判断。

### 1.2 软件

| 软件 | 当前要求或已记录版本 | 用途 |
| --- | --- | --- |
| 操作系统 | Debian GNU/Linux 13，ARM64 / AArch64 | Pi 运行环境，无图形桌面 |
| C++ / CMake | C++17，CMake ≥ 3.16.3 | 主工程构建 |
| OpenCV | Pi 已记录 4.10.0；WSL 离线环境为 4.5.4 | 图像处理、PnP、标定与编码 |
| Eigen3、yaml-cpp | CMake 查找系统开发包 | 线性代数与 YAML 配置 |
| fmt、spdlog、nlohmann-json | CMake 查找系统开发包 | 格式化、日志及数据处理 |
| HailoRT | **4.24.0 EXACT** | 当前生产构建精确要求的推理运行时 |
| Hailo PCIe 驱动 / 固件 | 现有环境记录为 4.24.0，部署时重新核对 | 加速器连接与运行 |
| Hikrobot SDK | 仓库 `io/hikrobot/include/` 与 `lib/arm64/` 配套文件 | 相机取流和图像转换 |

当前默认步兵工程不要求安装 OpenVINO、CUDA、TensorRT 或 ROS。保留的 ROS 导航接口与示例需另配依赖，不能据此视为本平台已经验收。

上游 SP25 的公开环境是 Intel NUC / Ubuntu 22.04 / OpenVINO，详见[官方项目环境](https://github.com/TongjiSuperPower/sp_vision_25#31-项目环境)。本地曾使用的 Jetson Orin NX 与上游基线应分别记录；不能把“Orin NX 有 NVIDIA GPU”写成“OpenVINO 已在该 GPU 上运行”。

## 2. 安装与检查依赖

以下命令在 **Pi 的 SSH 终端**执行。已能正常编译运行的现有 Pi 不需要为了本文重复安装或升级 SDK。

### 2.1 基础开发包

新环境先安装 C++ 构建工具和主工程依赖。这一步安装系统软件包，不修改项目源码。

```bash
sudo apt update
sudo apt install -y build-essential cmake pkg-config \
  libopencv-dev libeigen3-dev libyaml-cpp-dev libfmt-dev \
  libspdlog-dev nlohmann-json3-dev libusb-1.0-0-dev
```

如果需要运行 Python 编写的离线测试，再安装 `python3`。本项目使用电脑 Git 管理版本；Pi 接收源码包并编译运行，不强制在 Pi 建立 Git 仓库。

查看基础环境，预期架构为 `aarch64`，CMake 满足最低版本，OpenCV 能输出版本号：

```bash
cat /etc/os-release
uname -m
cmake --version
g++ --version
pkg-config --modversion opencv4
```

### 2.2 HailoRT 与设备

准备 **Hailo-8L 对应的 HailoRT 4.24.0 运行库、开发文件、CLI 和匹配驱动/固件**。开发文件需包含头文件和供 `find_package(HailoRT)` 使用的 CMake 配置；只有运行库或 Python 包不足以编译本项目。

安装方法以 [HailoRT 4.24.0](https://github.com/hailo-ai/hailort/tree/v4.24.0) 和[官方驱动项目](https://github.com/hailo-ai/hailort-drivers)说明为准，选择 ARM64 与当前内核对应的版本。仓库没有自动安装驱动的脚本；首次安装后是否重启，以所用安装包说明为准。

安装后执行以下只读检查：

```bash
hailortcli --version
hailortcli scan
hailortcli fw-control identify
ls -l /dev/hailo0
modinfo hailo_pci
```

预期 CLI 为 4.24.0，能枚举并识别 Hailo-8L，存在设备节点。`modinfo` 显示已安装模块的信息，**不能单独证明该模块已加载或设备可用**；应结合 `scan` 和 `identify` 结果判断。

版本不一致时先核对安装来源，不通过删除 CMake 的 `EXACT` 限制或把不同版本库文件互相软链接来绕过。

### 2.3 海康 SDK

主工程按 CPU 架构选择仓库 SDK，Pi 使用 `io/hikrobot/lib/arm64/`。从项目根目录查看配套文件：

```bash
cd ~/sp_vision_25-main
ls -l io/hikrobot/lib/arm64/
```

当前 CMake 检查以下文件：

```text
libMvCameraControl.so
libMvUsb3vTL.so
libFormatConversion.so
libMediaProcess.so
libMVRender.so
libMvSDKVersion.so
CommonParameters.ini
```

这些文件需要配套，不能只补一个 `libMvCameraControl.so`。文件存在检查不代替架构、运行库兼容性与真实相机取流验证。独立相机诊断支持 `HIK_SDK_LIBRARY`，默认主工程仍使用仓库指定路径。

### 2.4 模型

当前适配模型是 `szu_int16_head_l.hef`，来自 [QDU ArmorDetector 模型目录](https://github.com/QDU-Robomaster/ArmorDetector/tree/master/model)。HEF 不包含在本项目主源码交付包中，需要单独准备。

现有 Pi 复用 `~/workspace/current/` 下的模型，完整路径已写入 `configs/standard3.yaml`。新设备只需将 `hef_path` 改为本机实际文件位置，不必复制整套第三方工程，也不要为整理本项目而删除原工作区。

模型名称相似不代表输出兼容。当前解码实现按 `szu-int16-head-l` 的输入尺寸、输出结构、类别和角点约定工作；更换 HEF 需要重新检查这些约定。

## 3. 配置设备与参数

`standard` 和 `standard_readonly` 共用 [configs/standard3.yaml](configs/standard3.yaml)。在电脑上编辑并纳入 Git 后同步到 Pi；若在 Pi 临时修改，下一次交付前必须带回电脑。

| 配置项 | 首次部署时核对什么 |
| --- | --- |
| `hef_path` / `hailo_model` | 模型文件存在，标识为当前适配的 `szu-int16-head-l` |
| `serial_number` | 与正在连接的海康相机一致 |
| `enemy_color` | 与测试装甲板颜色一致 |
| `acquisition_frame_rate` | 当前默认 100；实际采集速率看 `camera_fps` |
| `expected_width` / `expected_height` | 当前为 720×540，与 2×2 下采样匹配 |
| `rotate_180` | 当前为 `true`，是旋转 180°，不是左右镜像 |
| `camera_matrix` / `distort_coeffs` | 与实际镜头、图像尺寸和图像方向一致 |
| `R_camera2gimbal` / `t_camera2gimbal` | 相机到云台的安装外参；当前仍有临时值 |
| `R_gimbal2imubody` | 云台与 IMU 本体系关系；当前单位阵待实车核对 |
| `qdu_communication.device` | 当前板子的 `/dev/serial/by-id/...` 路径 |
| `tx_enabled` / `auto_fire` | 正式入口是否发指令、是否允许开火判断；两者含义不同 |
| `default_mode` / `default_bullet_speed` | 当前配置默认值，不是实时电控反馈 |

当前内参记录为 **2026-09-25、8 mm、720×540、2×2 下采样、旋转 180°**，重投影误差为 0.0417 px。它不是其他镜头和任意分辨率的通用参数，也不能代替安装外参标定。

查看 Pi 枚举到的串口，只读取设备信息：

```bash
ls -l /dev/serial/by-id/
```

找到对应设备后将路径填入 YAML。串口存在或打开成功，只证明主机能访问端口；还需看到有效姿态数据持续更新。

若当前用户确实没有串口权限，可在 Pi 执行以下命令加入 `dialout`，随后退出 SSH 并重新登录：

```bash
sudo usermod -aG dialout "$USER"
```

此命令改变用户组权限，不修改 C 板固件。Hailo 和相机的权限按各自 SDK 安装规则配置。

## 4. 编译与首次运行

先完成以上依赖、模型和 YAML 检查，再在项目根目录构建。成功表现为生成两个程序，且没有缺库或配置错误。

```bash
cd ~/sp_vision_25-main
cmake -S . -B build/pi-standard
cmake --build build/pi-standard --target standard standard_readonly -j2
```

首次使用只读入口，连接持续发送姿态的真实 C 板后运行：

```bash
./build/pi-standard/standard_readonly configs/standard3.yaml --seconds=30 --preview-port=8080
```

电脑浏览器打开 `http://<树莓派IP>:8080/`，其中 IP 可在 Pi 上用 `hostname -I` 查询。预览通过 HTTP 传输，不是 SSH 自动转发 OpenCV 窗口，也不需要 Pi 桌面。

预期画面更新、有效姿态计数增长、`pipeline_fps` 大于 0，结束时 `tx=0`。`standard_readonly` 强制不写串口，不受 YAML 中 `tx_enabled: true` 影响。

需要模拟姿态、正式发送、单独相机/NPU 检查或标定时，使用[命令总手册](编译运行命令总手册.md)对应小节。同一时刻不要让两个程序争用同一相机、Hailo 或串口。

## 5. 常见现象

| 现象 | 含义与先检查的地方 |
| --- | --- |
| `No such file or directory` | 先确认在项目根目录执行、目标已经编译；串口报此错则核对设备路径 |
| CMake 找不到 HailoRT | 检查 4.24.0 开发包和 CMake 配置；只有 CLI 能运行还不够 |
| 缺少海康运行库 / `MV_E_LOAD_LIBRARY` | 核对 ARM64 配套库是否齐全，避免混用不同 SDK |
| `XOpenDisplay Fail` | 在已有测试中未阻止后续取图；以相机成功出帧为准，网页预览不依赖本地显示窗口 |
| `waiting for fresh IMU` / `rx_q=0` | 检查真实姿态发送、串口路径或模拟器；不能仅凭这个现象认定 MCU 被 halt |
| `det=0` | 当前没有合格装甲板候选；先看颜色配置、画面、曝光和装甲板是否在视野内 |
| 浏览器没有画面 | 确认运行时启用了预览端口，程序仍在运行，IP/端口和网络可达 |
| 预览约 10 FPS、计算约 100 FPS | 两者统计范围不同，预览主动限频 |
| 四板模型大小波动 | 已记录的几何/状态估计问题，不能只通过升帧率或换显示方式解决 |

提交排错记录时保留程序入口、配置、源码提交号、启动错误和 `[perf/end]`，比只发一张 FPS 截图更容易定位。

## 6. 开发、同步与恢复

1. 电脑 Git 管理源码和文档，保留能继续使用的 SP25 工具与实现。
2. 从同一个 Git 提交导出完整主源码、配置、CMake 和必要头文件，生成内容清单。
3. Pi 解包并校验内容及额外源码；未知文件先核对归属，不直接删除。拿到 Pi 的 PASS 后才记录同步完成。
4. 在 Pi 原生重新编译；WSL 的 x86_64 可执行文件不能直接在 ARM64 Pi 上运行。
5. 成功交付后清理传输包。Pi 不堆叠源码备份；恢复版本通过电脑 Git 选择旧提交后重新同步。

主源码部署包排除 Git 历史、文档、构建目录、日志、用户图片和标定采集数据。临时工具与结果放忽略的 `build/`，不修改青大原工作区。

最新 Pi 同步状态记录于本地 `docs/9.27-main-source-sync.md`，本次未上传。电脑端当前的 `autostart.sh` 只提供持续相机诊断；完整主程序的开机启动、异常恢复和长期运行需另行验收。
