# Raspberry Pi 5 + Hailo-8L：SP25 适配当前事实版

> 整理日期：2026-09-12。环境表沿用 09-08 整理的采样；连接信息、HEF 解析和开发依赖查询按 09-12 用户提供的输出更新。未重新采样的项目不视为本次复测。  
> 工作方式：先在开发电脑本地整理、修改 SP25 源码，再部署到目标树莓派。  
> 当前阶段：[09-12 上板验收] Pi 独立检测工程编译、CTest **5/5**、HAILO8L 设备查询及真实 HEF 推理均已通过。完整静态图片检测达到 **110.453 FPS，平均 9.053 ms，p95 9.140 ms，最大 9.291 ms**（同图预热 10 次、计时 100 次），最后返回 **1 个 blue sentry small，目标分数 0.970**。原始推理/融合基准为 169.112 FPS。Pi 画框图已回收并查看，框位于蓝色装甲区域、未见角点交叉；真实标识、角点精度与大小板仍需核验。图像预处理及检测已计时，相机、跟踪、解算、控制和整套长稳仍未验收。见 [Pi 接入说明](docs/9.9-pi-integration.md)。  
> 最终要求：面向实际硬件保留必要功能；不再使用的 OpenVINO 源码、构建依赖、配置与模型应彻底清理，不保留无用兼容分支。

**信息来源说明**：标注“本轮实测”的内容来自用户粘贴的 `xiao@PI-HAILO-13T` 终端输出；“此前实测”来自 `pineapple@thinkbook25`；“此前记录”来自上一版文档。命令建议和修改原则不是已经执行的操作，未验证的项目统一标为待确认。

## 1. 当前结论与工程状态

| 项目 | 当前状态 |
| --- | --- |
| 开发电脑 | WSL2、Ubuntu 22.04.5、`x86_64`；用于编辑源码和硬件无关测试 |
| 目标树莓派 | Debian 13、`aarch64`、glibc 2.41；与开发电脑不是同一环境 |
| NPU 型号 | **已实测确认为 `HAILO8L`，不再是根据主机名推测** |
| Hailo 软件与固件 | [09-12 复核] 两个软件包为 4.24.0-0local1，已加载驱动、CLI 链接运行库、设备固件均为 4.24.0；identify 查询成功 |
| 相机连接 | 海康 `MV-CS016-10UC` 已枚举，连接速率为 `5000M`；实际取图尚未验证 |
| HEF | [09-12] 校验/解析、真实推理与完整检测流程成功；本图输出蓝色哨兵小板，真实标签及画框几何待核实 |
| C 板通信 | 本轮未显示 CAN 接口；指定的 USB 串口设备路径无输出，实际链路仍未定位 |
| 目标机开发依赖 | [上板核对 09-12] 五个通用开发包均已安装；独立检测工程已用 OpenCV 4.10.0/HailoRT 4.24.0 编译链接成功，完整自瞄及相机 SDK 链接仍待验证 |
| 用户修改情况 | SP25 本机适配已推进至第 9、10 部分，源码位于 `D:\workspace\sp_vision_25-main`；当前未使用 Git |
| SP25 基线 | Pi 独立检测工程编译、基础测试和真实完整检测流程通过；原始下载来源版本仍未固定，识别正确性、实际视频/相机与整套自瞄尚未验收 |
| 树莓派原有文件 | 本轮查到青大 `source-live/Modules/ArmorDetector/model` 与历史模型文件，不再写成“Pi 完全没有代码或模型” |

**现在已完成主要环境清点、HEF 实际运行和完整静态检测计时；识别正确性、相机取图、C 板通信及时间同步仍待验收。** 检测流程成功返回结果不等于整套自瞄已完成验收。

[上板核对 09-12] 用户在目标终端确认：用户名 `xiao`，主机名 `PI-HAILO-13T`，当次 IP 为 `192.168.1.105`。IP 是连接快照，可能随网络分配变化。

## 2. 开发机与目标机分开记录

### 2.1 系统及工具链

| 项目 | 开发电脑：此前实测 | 目标树莓派：本轮实测 |
| --- | --- | --- |
| 终端 | `pineapple@thinkbook25` | `xiao@PI-HAILO-13T` |
| 系统 | Ubuntu 22.04.5 LTS / Jammy | Debian GNU/Linux 13 / trixie |
| 系统补充字段 | — | `DEBIAN_VERSION_FULL=13.5` |
| 内核 | `6.6.87.2-microsoft-standard-WSL2` | `6.18.34+rpt-rpi-2712` |
| CPU 架构 | `x86_64` | `aarch64` |
| glibc | 2.35 | 2.41 |
| g++ | 11.4.0 | 14.2.0，包版本 `14.2.0-19` |
| CMake | 3.22.1 | 3.31.6 |
| OpenCV | 4.5.4 | 4.10.0 |
| Eigen | 3.4.0 | [09-12] libeigen3-dev 3.4.0-5，已安装 |
| yaml-cpp | 0.7.0 | [09-12] libyaml-cpp-dev:arm64 0.8.0+dfsg-7，已安装 |
| fmt | 8.1.1 | [09-12] libfmt-dev:arm64 10.1.1+ds1-4，已安装 |
| spdlog | 1.9.2 | [09-12] libspdlog-dev:arm64 1:1.15.2+ds-2，已安装 |
| nlohmann-json | 开发机验证时曾临时解包使用 | [09-12] nlohmann-json3-dev 3.11.3-2.1，已安装 |
| HailoRT | 未安装运行库；本次临时用官方 4.24.0 头文件验证 | [09-12 复核] CMake 开发版本 4.24.0、运行库 4.24.0、软件包 4.24.0-0local1 |

**解释**：此前 pkg-config、dpkg 和指定目录的 CMake 配置搜索均未找到这五项开发依赖；09-12 用户随后提供的包查询已确认它们全部安装。无需继续重复缺包调查，但安装登记不代替实际 CMake 配置、编译及链接验证。第 7.2 节查询命令保留用于检查。

本地编辑源码不要求先安装 Hailo 驱动，也不要求先建立 Git 仓库。后续必须固定所使用的 SP25 源码基线；没有 Git 时，可用原始压缩包、下载来源和 SHA-256 记录版本。

**部署原则**：默认传输源码，在 Pi 上使用目标机工具链原生编译。WSL 默认编译产生的 x86_64 程序不能直接当作 Pi 的 ARM64 程序使用。交叉编译是另一项需要单独配置的工作，当前没有配置完成的证据。WSL 测试通过也不替代 Pi 上的编译、动态库与硬件验证。

### 2.2 树莓派资源

主板型号 `Raspberry Pi 5 Model B Rev 1.1`、四核 Cortex-A76 来自此前记录，本轮没有重新读取主板型号。

| 项目 | 本轮快照 |
| --- | --- |
| RAM | 总计 2.0 GiB；used 230 MiB；available 约 1.7 GiB |
| Swap | 总计 2.0 GiB；当前使用 0 B |
| 根分区 | `/dev/mmcblk0p2`；29 G，总用量 19 G，可用 9.5 G，使用率 66% |
| `/dev/shm` | tmpfs 容量 1006 M，当前使用 0 |

`/dev/shm` 的容量已查到，不再列为待采集项。但这只能确认共享内存文件系统约 1 GB，不能证明此前别人说的“1 GB”一定指它，也不代表额外增加了一块独立内存。

这些是采集时的资源状态，不是视觉程序运行后的性能结果。后续保留有界队列、缓冲区复用、默认关闭高负载预览/录像、限制日志体积等要求；Pi 首次编译采用 `-j2`，内存不足时降为 `-j1`。不要为了性能先升级系统、超频或更改驱动。

## 3. Hailo 与 HEF 的已知事实

### 3.1 设备与运行环境

[首次 Pi 配置更新] CMake 曾因要求 4.23.0 EXACT、找到 4.24.0 而停止。工程现已采用 4.24.0 EXACT，并由 CMake 将构建版本传给运行时检查，Pi 独立检测工程编译成功。用户后续输出确认两项包、当前驱动、CLI 所用运行库及固件均为 4.24.0。下方单独标明的旧表格保留历史采样。

| 09-12 最新查询 | 实际结果 |
| --- | --- |
| `dpkg -l '*hailo*'` | hailort 与 hailort-pcie-driver 均为 ii、4.24.0-0local1、arm64 |
| `/sys/module/hailo_pci/version` | 4.24.0，表示内核当前已经加载的驱动版本 |
| `ldd` 当前 hailortcli | libhailort.so.4.24.0 → /lib/libhailort.so.4.24.0 |
| 当前 `fw-control identify` | 查询成功，PCIe 0001:01:00.0；HAILO8L；固件 4.24.0 (release,app,extended context switch buffer)，控制协议 2，Logger 0 |

`-0local1` 是 Debian 软件包修订后缀；实际开发配置、驱动和运行库的核心版本为 4.24.0。`un h10-*` 不表示当前 Hailo-8L 缺依赖，不据此安装 Hailo-10 专用包。

| 项目 | 历史 4.23.0 采样（非当前版本） |
| --- | --- |
| PCIe 设备地址 | `0001:01:00.0` |
| `Device Architecture` | **`HAILO8L`** |
| `Board Name` | `Hailo-8` |
| `Firmware Version` | `4.23.0 (release,app,extended context switch buffer)` |
| `Control Protocol Version` | 2 |
| CLI | `HailoRT-CLI version 4.23.0` |
| `hailort` 包 | 4.23.0，arm64 |
| `hailort-pcie-driver` 包 | 4.23.0，all |
| 运行库 | `/lib/libhailort.so.4.23.0`、`/lib/libhailort.so`；链接器缓存标为 AArch64 |
| CMake 配置 | `/usr/lib/cmake/HailoRT/HailoRTConfig.cmake` |
| 基础验证 | `scan` 与 `fw-control identify` 均成功 |

型号以明确的 `Device Architecture: HAILO8L` 为准；不能因为同一输出中 `Board Name` 写着 `Hailo-8`，就改判为 HAILO8。

推理实现原按 **HAILO8L + HailoRT 4.23.0** 编写，本机现依据最新查询改为 **HAILO8L + 4.24.0**。只修改构建版本、启动时版本一致性检查和相关说明，保持现有同步单帧 API 及模型处理流程。[官方 4.24.0 驱动接口源码](https://github.com/hailo-ai/hailort/blob/v4.24.0/hailort/libhailort/src/vdma/driver/hailort_driver.cpp)仍会比较运行库与驱动的主、次及修订版本。

目前已证明固件查询、原生编译、基础测试、具体 HEF 推理及完整检测流程成功；当前仅一张真实图片的短测结果，标签、角点贴合、长时间稳定性及整套自瞄性能仍待验证。

### 3.2 已发现的模型位置

当前工作区中查到：

```text
/home/xiao/workspace/current/pi-camera-249fps-20260822-product-arm64/source-live/Modules/ArmorDetector/model/
├── szu_int16_head.hef
└── szu_int16_head_l.hef
```

历史归档中查到：

```text
/home/xiao/archive-20260906/historical/codex_runs/calibration_preview_hw_20260816T194327/payload/source/Modules/ArmorDetector/model/
├── skd_int8_grid_l.hef
├── int16_fast_l.hef
├── skd_int8_head_l.hef
├── int16_fast.hef
├── skd_int8_head.hef
└── skd_int8_grid.hef
```

这是用户本轮提供的搜索结果，不据此保证已覆盖所有目录和可访问文件。`current`、`source-live`、`249fps` 等只是路径文字，**不能证明该程序正在运行、代码完整、采用哪份配置或实测帧率达到 249 FPS**。历史归档也不能直接当作当前运行基线。

优先检查当前工作区的 `szu_int16_head_l.hef`，但它目前只是候选模型，不是已验证的最终选择。校验值和元数据已于 09-12 获得，见第 3.3 节；实际推理与解码仍待验证。不要混用不同目录的 HEF 和未经核对的解码器。

### 3.3 HEF 校验与解析结果（2026-09-12）

[上板核对 09-12] 用户修正路径中的真实换行及空格后，成功读取当前工作区的 `szu_int16_head_l.hef`。`ls -lh` 显示约 8.5M，SHA-256 为：

```text
ceaef30dda3de1dadcfb7f35dbbef86ce91ce8d4969754de3fc987d961e1599c
```

目标架构为 **HAILO8L**。网络组及网络名为 `szu_fp32_conv45_46_47_a16`；解析显示 `Multi Context`，上下文数量 2。下表省略各流名称共有的网络名前缀：

| 流 | 元素类型 | HEF 报告的格式与 shape |
| --- | --- | --- |
| input_layer1 | UINT8 | NHWC(512x640x3) |
| conv47 | UINT16 | FCR(64x80x66) |
| conv54 | UINT8 | FCR(32x40x66) |
| conv60 | UINT8 | FCR(16x20x66) |

输入高 512、宽 640、3 通道；三个输出网格对应 8/16/32 倍下采样，特征数均为 66。这些尺寸和混合整数类型与当前 SP25 Hailo 运行时的结构预期相符。`hailo_runtime.cpp` 已显式请求主机缓冲为 NHWC，并按每个流的实际类型分配和访问；HailoRT 4.23 的 [FCR 输出转换实现](https://github.com/hailo-ai/hailort/blob/v4.23.0/hailort/libhailort/src/transform/transform.cpp)包含 UINT8/UINT16 到主机 NHWC 的路径。此处为源码与元数据核对，转换、量化参数及推理仍须真机测试。

66 个特征本身不足以证明语义；09-12 已进一步核对同一模型对应的青大 CMake、Network、Inference、ModelAdapter 和 PnP 源码，确认每格三个候选、每候选 22 项及相应前后处理规则，见第 3.4 节。真实 HEF 输出与量化参数仍需上板验证，多上下文数量不能直接换算为推理 FPS。

此前两次解析分别因空路径变量、引号内粘入换行而失败；本次路径问题已解决，无需重复排查。第 7.1 节命令保留供后续模型指纹复核。

### 3.4 青大 INT16 参考适配器（09-12 用户提供源码）

[上板核对 09-12] 用户已将参考模块复制到 `D:\workspace\sp25-qdu-reference`。在先前注册表和 INT16 适配器之外，现已核对 CMake 的实际绑定：`INT16_HEAD_L` / `int16-quality-l` 的路径宏对应 `model/szu_int16_head_l.hef`。这确认了本次候选 HEF 的源码处理路径，仍不代表已经核实 Pi 当前程序的启动配置。

INT16 适配器声明：20160 个候选、每候选 22 项；颜色区间 `[9,13)`，编号区间 `[13,22)`；`reject_aux_colors=true`；`confidence_is_logit=false`；角点索引顺序 `{0,3,2,1}`。颜色函数将原始 0/1 映射为蓝/红，其他 ID 映射 UNKNOWN；编号函数将 0 映射为哨兵、1–5 映射同号、6 映射为前哨站、7 和 8 均映射为基地。

已追踪调用链：`BuildNetworkInput` 直接 resize 到 640×512 并 BGR→RGB，不填边、不除以 255；`FuseOneHailoOutput` 反量化后在 ARM 上先舍入至 FP16，再用三组 anchor 与 grid 偏移恢复角点；`decode_confidence` 的 **false 分支实际执行 sigmoid**。不能按字段名误判为直接使用概率，也不能把类别分数一概当作概率。PnP 注释确认 canonical 角点为左上、右上、右下、左下。

SP25 本轮已补齐 `SzuInt16Model` 和 `YOLO::detect`，放宽编号映射的唯一性约束以允许两个基地 ID，同时仍检查合法枚举；新增原始颜色 2/3 的丢弃规则。生产配置明确使用 8→基地；旧通用 Decoder 测试的 8→not_armor 仅是测试夹具。具体公式、源文件指纹、修改原因和验收方式见 [Pi 接入说明](docs/9.9-pi-integration.md)。

## 4. 相机与 C 板：已看到什么，没看到什么

### 4.1 相机

| 项目 | 状态 |
| --- | --- |
| 型号与 USB ID | `Hikrobot MV-CS016-10UC`、`2bdf:0001`，本轮实测 |
| 枚举位置 | Bus 004，Device 002；这是本次枚举编号 |
| USB 连接 | `lsusb -t` 中相机对应接口均为 `5000M` |
| ARM64 相机 SDK | [上板核对 09-12] 青大工程自带主库已确认 ELF 64-bit ARM aarch64，`ldd` 列出的依赖全部解析；匹配头文件、按需加载组件及实际取图仍待验收 |
| 实际图像输出 | 分辨率、像素格式、颜色通道、曝光、增益、帧率：待确认 |
| 采集方式 | 自由运行/软件触发/硬件触发，触发源与触发沿：待确认 |
| 图像几何 | ROI、decimation、是否旋转：待确认 |
| 同步与标定 | 时间戳语义、IMU 配对方法、当前内外参：待确认 |

`5000M` 只确认 USB 链路速率，不等于实际视频吞吐或帧率。`Driver=[none]` 是本次 USB 树输出，不能仅凭它判定相机 SDK 不可用；当前并没有取图测试结果。

[上板核对 09-12] 用户查到 `/usr/include/hailo/hailort.h`，以及青大工程 `source-live/Modules/HikCamera/hikSDK/lib/arm64/libMvCameraControl.so` 和带 `.4.8.1.2` 后缀的文件。对 arm64 主库执行 `file -L` 得到 `ELF 64-bit LSB shared object, ARM aarch64`；ELF Build ID 为 `5b58d6f730d771e5f88b7188d3369507d51d75cf`（不是文件 SHA-256）。`ldd` 中 pthread、dl、stdc++、m、gcc_s、libc 及加载器均找到，没有 `not found`。结果证明基础架构及列出的依赖匹配，不证明相机已经能枚举、取图或完整 SDK 组件均已加载。`stripped` 表示调试符号已剥离，不代表库文件损坏。

同一工程还有 amd64 库，不能用于 Pi ARM64。模型注册表及完整 INT16 检测处理链的源码核对见第 3.4 节。五个通用开发包现已确认安装，版本见第 2.1 节。

最终输入层仍应统一输出明确约定的 BGR 图像，并让图像、几何参数、时间戳和对应 IMU 保持一致。不得把旧车的固定 `t - 1ms` 或示例标定值直接当成本车实测值。

### 4.2 C 板链路

| 检查 | 本轮结果 | 能得出的结论 |
| --- | --- | --- |
| `ip -br link` | 仅 `lo`、`eth0`、`wlan0`；eth0 为 DOWN，wlan0 为 UP | 本次未显示 CAN 接口，不只是缺少 `can0` 这一名字 |
| `/dev/ttyACM*` | 无输出 | 本次列表未显示这一类 USB 串口路径 |
| `/dev/ttyUSB*` | 无输出 | 本次列表未显示这一类 USB 串口路径 |
| `/dev/serial/by-id/*` | 无输出 | 本次列表未显示这一类稳定串口链接 |
| `lsusb` | 除 root hub 外，看到 ST-LINK/V2 和海康相机 | 没有显示额外的 USB 通信设备 |
| ST-LINK/V2 | `0483:3748` | 该设备被枚举不等于已经确认了 C 板运行时数据通道 |

**当前结论是“C 板通信尚未定位”，不是“确定使用 SharedTopic”，也不是“确定没有 C 板”。** 本次串口命令隐藏了错误输出，而且没有检查所有可能的设备路径；上述结果不足以排除 GPIO UART、其他设备名、网络或自定义接口方案。

`DevC-USB` 在参考代码里可能是硬件容器中的逻辑名称，不能预设它必然对应 `/dev/DevC-USB`。最终要核对初始化代码、实际设备路径和电控固件。

还需确认：数据物理通道；四元数顺序、坐标轴、单位和时间域；模式与弹速来源；目标角命令的绝对/相对含义；开火许可、掉线处理；相机触发命令与反馈。仅查看设备列表不能确定这些协议语义。

## 5. 源码状态与后续清理范围

### 5.1 不再要求先建立 Git

用户当前采用“电脑本地修改，再放到 Pi”的工作方式。本机模块适配及离线回归已完成，用户已确认完整文件夹传至 `/home/xiao/sp_vision_25-main`，首次配置因 HailoRT 版本不符停止。文档不要求先 clone、创建分支或提交 patch。

当前适配代码和各部分验证记录已在本机；原始包的下载来源、版本与校验值仍待补充。Git 可作为以后选择的管理工具，不是当前前置条件。

树莓派上已经查到青大模型所在目录。下一步可只读检查附近源码、配置和版本说明；暂不下载另一套青大最新代码覆盖它，也不把历史目录复制成新的生产环境。

### 5.2 清理原则

| 范围 | 后续处理原则 |
| --- | --- |
| SP25 上层 | 保留 Armor、Solver、Tracker、Target、Aimer 等所需算法；必要时修正数据初始化、图像尺寸、接口和时间语义，不保证源码一字不改 |
| 推理 | 面向 Hailo 的单一生产推理路线；复用与最终 HEF 匹配的预处理、量化/输出融合及解码能力 |
| OpenVINO | 删除不再使用的 include、类型、源文件、CMake 查找与链接、配置项及专属模型，不只关闭编译开关 |
| 相机和通信 | 按最终实际方案保留需要的实现；未确认前不选定 SocketCAN 或 SharedTopic，不预先删除可能仍有引用的模块 |
| 其他功能 | auto_buff、全向感知、ROS2、UAV、其他相机、规划等是否保留，仍取决于用户功能范围和代码依赖；尚未确认永久删除 |
| 测试、标定、故障保护 | 必要内容保留；硬件无关离线测试不等于保留 OpenVINO 备用后端 |

**不得按 `.xml`、`.bin` 扩展名或 YOLO 名称整包删除文件。** 删除依据是文件用途和引用关系；XML 可能是其他配置，模型算法名称也不等于推理运行库。第三方许可证与必要运行库不能当作冗余清掉。

09-12 在用户复制完整参考模块后，助手已在本机补齐 YOLO、INT16 模型 tail、类别映射和图片验收入口，相关代码标记 `[Pi接入]`；参考目录未改动。用户已在 Pi 安装上述五项开发包。此前工程内的 OpenVINO 清理见第一部分记录；工程内移除依赖不自动意味着卸载目标机上供其他软件使用的系统包。

## 6. 验证边界与待办

| 验证层次 | 当前状态 |
| --- | --- |
| WSL 开发环境清点 | 已完成此前列出的工具链与五项库版本记录 |
| Pi 系统、工具链、资源清点 | 已完成本轮记录 |
| NPU 架构、软件版本、设备识别 | [09-12] identify 查询成功并确认 HAILO8L，固件、已加载驱动、CLI 运行库均为 4.24.0 |
| 相机 USB 枚举与连接速率 | 已完成；海康设备、5000M |
| HEF 路径定位 | 已找到两处模型目录及候选文件 |
| HEF 校验、解析、真实推理 | [09-12] 校验/解析/推理通过，原始推理融合 169.112 FPS、完整静态检测 110.453 FPS；实际标签与角点待核实 |
| Pi 开发依赖、Hailo 头文件、相机 SDK | 五个开发包已安装，独立检测工程已通过真实 SDK 编译链接；海康 arm64 主库架构/ldd 已核对，相机集成和调用仍待验证 |
| 当前青大源码、配置、启动方式与版本 | 待核实；不能由目录名判定 |
| C 板链路与同步协议 | 待核实，是完成硬件适配的重要阻塞项 |
| 实际 SP25 源码包与最终功能范围 | 完整 YOLO 检测入口及模型语义已接入，整目录已上传，独立检测编译/基础测试通过；真实检测与最终运行功能仍待验收 |
| Pi 原生编译、连续取图、同步、标定与长稳验证 | 独立检测工程原生编译与 5 项基础测试通过；连续取图、同步、标定及长稳测试仍未完成 |

不重复索要已提供的 Hailo 型号、系统版本、USB 速率和内存信息。环境或接线发生变化后，再按需要更新相关项。

后续验收需要分别记录：源码审查、硬件无关测试、Pi 编译链接、HEF 推理、相机取图、通信与同步、标定和故障处理。**完成代码修改不等于完成真机验证**。联调期间保持禁弹和输出保护，不能仅依赖一个 YAML 开关证明整条控制链已安全。

## 7. 仅保留尚缺信息的只读采集命令

> 以下均应在目标 Pi 的终端执行，不在 WSL 用结果判断 Pi。  
> 本文只是提供命令，没有代为执行。除临时 shell 变量赋值外不写文件；不安装软件、不发送控制命令、不升级固件。没有输出或出现权限/路径错误时保留原结果，不自行补装或删除。

### 7.1 先修复 HEF 路径，再解析候选模型

把下面整块粘贴到**同一个 Pi 终端**：

```bash
HEF='/home/xiao/workspace/current/pi-camera-249fps-20260822-product-arm64/source-live/Modules/ArmorDetector/model/szu_int16_head_l.hef'
printf 'HEF=<%s>\n' "$HEF"

if [ -f "$HEF" ] && [ -r "$HEF" ]; then
    sha256sum "$HEF"
    hailortcli parse-hef "$HEF"
else
    printf '文件不存在或不可读：%s\n' "$HEF"
fi
```

`HEF=...` 只在当前 shell 保存路径，不下载、不生成、不修改模型；新开终端后应重新赋值。`printf` 显示实际传入的路径，避免再把空字符串传给程序。`-f` 和 `-r` 检查普通文件是否存在且可读。`sha256sum` 记录文件指纹，不证明精度或兼容性。`parse-hef` 查看模型元数据，不等于完成设备推理。

本轮暂不把 `hailortcli run` 列为必执行项：先核对解析结果，并确认是否有现有程序占用设备，再安排推理测试。

### 7.2 开发依赖与 SDK 的只读查询（五个开发包现已安装）

先查看系统开发包登记：

```bash
dpkg-query -W -f='${binary:Package}\t${Version}\t${Status}\n' \
  libeigen3-dev libyaml-cpp-dev libfmt-dev libspdlog-dev
```

作用：区分“pkg-config 没找到”和“Debian 包管理器是否登记了开发包”。包查询没有命中，也不能排除手工安装；因此继续找 CMake 配置：

```bash
find /usr /opt \( \
  -name 'Eigen3Config.cmake' -o -name 'eigen3-config.cmake' -o \
  -name 'yaml-cpp-config.cmake' -o -name 'yaml-cppConfig.cmake' -o \
  -name 'fmt-config.cmake' -o -name 'fmtConfig.cmake' -o \
  -name 'spdlogConfig.cmake' -o -name 'spdlog-config.cmake' \
\) -print 2>/dev/null
```

该搜索只覆盖 `/usr` 与 `/opt` 下可访问位置，不据此断言其他目录没有依赖。后续实际 CMake 配置测试才确定工程能否正确找到它们。

再查看 Hailo 包提供的头文件/CMake 文件，以及相机库缓存：

```bash
dpkg -L hailort | grep -E '(/include/hailo/|/cmake/HailoRT/)'
ldconfig -p | grep -i MvCameraControl
```

前者补充 C++ 开发文件信息；后者只检查链接器缓存，未命中不代表相机 SDK 一定不存在。

### 7.3 检查已发现的青大目录与剩余接口

```bash
QDU='/home/xiao/workspace/current/pi-camera-249fps-20260822-product-arm64/source-live'

if [ -d "$QDU" ]; then
    ls -la "$QDU"
    find "$QDU" -maxdepth 4 -type f \( \
      -name 'xrobot.yaml' -o -name 'modules.yaml' -o \
      -name 'hik.yaml' -o -name 'sentry.yaml' -o \
      -name 'vision_capture.yaml' -o -name 'README.md' \
    \) -print
else
    printf '目录不存在：%s\n' "$QDU"
fi
```

作用：从已经发现模型的同一工作区定位配置与说明，不再要求重新搜索整块磁盘或先下载仓库。找到的文件仍需核对内容和启动关系，不能默认第一份 YAML 就是实际配置。`-maxdepth 4` 只限定本次查找范围。

相机库可能随原工程或系统 SDK 保存，可补充：

```bash
find '/home/xiao/workspace/current/pi-camera-249fps-20260822-product-arm64/source-live' \
  /opt /usr/local /usr/lib /lib \
  -name 'libMvCameraControl.so*' -print 2>/dev/null
```

只确定文件位置，不等于已经通过加载或取图测试。

最后查看此前串口列表未覆盖的几类路径：

```bash
for dev in /dev/DevC-USB /dev/ttyAMA* /dev/ttyS* /dev/serial/by-path/*; do
    if [ -e "$dev" ] || [ -L "$dev" ]; then
        ls -l "$dev"
    fi
done
```

作用：列出可能的自定义链接和板载 UART 路径；设备存在也不说明它已连接 C 板。这里不打开串口、不读取业务数据，也不修改波特率。若仍无可用线索，应结合实物接线和电控固件核对，不继续假定必须是 USB 串口。

---

### 本版已清理的过时内容

删除了“Hailo 型号仍未知”的反复确认、要求再次采集已提供环境信息的清单、强制先建 Git/clone 的流程，以及“Pi 没有任何代码或模型”的笼统表述。将“缺库”改为“pkg-config 未找到、待核实”，将 HEF 解析失败归位为空路径问题，并保留 C 板协议、实际相机参数、模型验证和最终功能范围这些真正尚缺的信息。
