# 操作手册

[仓库首页](../README.md) · [学习路线](ROADMAP.md) · [技术说明](TECHNICAL.md)

Bash命令默认在WSL项目根目录执行；pxh命令在运行中的PX4 Shell输入。原理与源码解释见技术说明。

[环境](#environment) · [编译](#build) · [SIH](#sim) · [模块](#module) · [测试](#tests) · [日志](#logs) · [Gazebo](#gazebo) · [通信](#communications) · [硬件](#hardware) · [Git](#git) · [排错](#troubleshooting)

<a id="environment"></a>
## 环境与版本

源码目录为/home/yiyi/Workspace/px4/PX4-Autopilot，开发任务为px4-main，容器工作目录统一为/workspaces/PX4-Autopilot。任务名不决定Git分支。

```bash
cd /home/yiyi/Workspace/px4/PX4-Autopilot
git status --short --branch
git describe --tags --always --dirty
~/embedded-lab/bin/lab info px4-main
```

基线为v1.17.0，分支为learn/px4-v1.17.0。提交自己的修改后，describe出现额外提交数是正常现象。用git submodule status --recursive查看依赖，常规更新按主仓库gitlink锁定；--remote属于依赖升级。

需要容器Shell时执行 `~/embedded-lab/bin/lab shell px4-main`，退出用exit。下面都使用lab完整路径，避免裸lab不在PATH的问题。

<a id="build"></a>
## 编译

WSL执行：

```bash
~/embedded-lab/bin/lab exec px4-main -- make px4_sitl_default -j2 PX4_MAKE_ARGS=-j2
```

这是默认SITL增量编译。链接完成并正常退出说明构建通过，运行另用下一节命令。日常修改C++无需先clean。

镜像尚未准备时，先看任务信息，再用 `~/embedded-lab/bin/lab image px4` 准备。已锁镜像的任务，构建新镜像后还需按本机 `~/embedded-lab/bin/lab upgrade --help` 切换；镜像准备成功不等于当前任务已使用它。

<a id="sim"></a>
## SIH四旋翼起降

WSL启动：

```bash
~/embedded-lab/bin/lab exec px4-main -- make px4_sitl sihsim_quadx -j2 PX4_MAKE_ARGS=-j2
```

出现pxh后逐条输入：

```text
commander check
commander status
listener vehicle_local_position
```

在这个SIH四旋翼环境中，预检通过后起飞：

```text
commander takeoff
```

看到Takeoff detected并等待稳定，再查看：

```text
commander status
listener vehicle_local_position
listener vehicle_land_detected
```

结束时输入commander land，等Landing detected和Disarmed，再核对status及落地主题，最后shutdown退出PX4。SIH没有三维窗口也能正常运行，位置、模式和落地状态是这里的观察依据。

<a id="module"></a>
## 加入观察模块

完整代码和四文件分工见[第5章](TECHNICAL.md#ch05)，现成源码在[示例目录](examples/learn_state_monitor)。

仅当src/examples/learn_state_monitor尚不存在时，在WSL复制：

```bash
cp -a learning/examples/learn_state_monitor src/examples/learn_state_monitor
```

若已存在，先比较并保留自己的修改。编辑boards/px4/sitl/default.px4board，确保仅有一条 `CONFIG_EXAMPLES_LEARN_STATE_MONITOR=y`，再按前两节编译和启动。

pxh中：

```text
learn_state_monitor start
learn_state_monitor status
param show LEARN_MON_HZ
param set LEARN_MON_HZ 2
```

先保存参数原值，观察频率变化后恢复，再stop/start/stop检查生命周期。示例是待实际编译运行的教学代码，源码核对不能代替运行验证。

<a id="tests"></a>
## 测试

WSL以位置控制测试为例：

```bash
~/embedded-lab/bin/lab exec px4-main -- make tests -j2 PX4_MAKE_ARGS=-j2 TESTFILTER='^unit-PositionControl$'
```

测试使用独立的build/px4_sitl_test配置。目录生成后列出测试：

```bash
~/embedded-lab/bin/lab exec px4-main -- ctest --test-dir build/px4_sitl_test -N
```

按改动替换TESTFILTER：

| 内容 | 名称 |
|---|---|
| 高度融合 | unit-test_EKF_height_fusion |
| 角速度控制 | unit-rate_control_test |
| 姿态控制 | unit-AttitudeControl |
| 位置控制 | unit-PositionControl |
| 分配伪逆 | unit-ControlAllocationPseudoInverse |
| 分配去饱和 | functional-ControlAllocationSequentialDesaturation |
| 失效保护 | functional-failsafe_test |

确认实际名称、数量和结果。零项匹配不是通过；依赖构建失败与行为断言失败分别处理。

<a id="logs"></a>
## 日志与参数

WSL查找默认SITL日志：

```bash
rg --files --hidden --no-ignore build/px4_sitl_default/rootfs/log -g '*.ulg'
```

路径以logger输出和实际rootfs为准。用现有ULog分析工具先看时间范围、参数、主题和dropout。比较目标、状态、模式和限制时按时间对齐，不按不同CSV的行号拼接。

参数实验先用param show保存原值，再set，结束明确恢复并show。参数可能自动保存，未手动save不保证重启恢复。起飞高度对照原理见[第10章](TECHNICAL.md#ch10)。

大日志保存在本地构建或制品目录；Git保存小型分析结果、关键参数和结论。learning/.local/已被忽略，可放临时导出文件。

<a id="gazebo"></a>
## Gazebo

先检查实际任务镜像的依赖：

```bash
~/embedded-lab/bin/lab exec px4-main -- gz sim --versions
~/embedded-lab/bin/lab exec px4-main -- protoc --version
```

依赖就绪且CMake发现组件后，先运行无图形X500：

```bash
~/embedded-lab/bin/lab exec px4-main -- env HEADLESS=1 make px4_sitl gz_x500 -j2 PX4_MAKE_ARGS=-j2
```

模型与PX4连接正常后再检查起降。GUI另涉及WSLg、显示变量、socket和图形设备。此前APT 502是下载失败，需要解决下载链路，不能把Dockerfile已写包名当作安装成功。

<a id="communications"></a>
## 通信观察

pxh先运行mavlink status核对端口。历史远端14550常用于地面站，14540用于伴随接口，不能与本地18570/14580混称。host共享当前Linux网络，bridge容器的127.0.0.1只指自身。

已配置Agent/ROS2及匹配的px4_msgs时，Agent终端运行：

```bash
MicroXRCEAgent udp4 -p 8888
```

已经source ROS及消息工作区的终端运行：

```bash
ros2 topic list
ros2 topic info -v /fmu/out/vehicle_local_position_v1
ros2 topic echo /fmu/out/vehicle_local_position_v1 --qos-reliability best_effort
```

按实际命名空间、QoS和端口调整。本标签MESSAGE_VERSION参与DDS主题命名，实现见[第12章](TECHNICAL.md#ch12)。外部控制要分别检查持续信号、实际模式、正常退出和断流保护。

<a id="hardware"></a>
## 其他机型与硬件

SIH固定翼目标是sihsim_airplane，标准VTOL是sihsim_standard_vtol。启动形式与四旋翼相同，但起降和Hold的含义不同，先读[机型说明](TECHNICAL.md#ch11)。

固件目标必须对应实际板型。确认设备匹配fmu-v6x后，才使用此示例：

```bash
~/embedded-lab/bin/lab exec px4-main -- make px4_fmu-v6x_default -j2 PX4_MAKE_ARGS=-j2
```

刷写入口取决于板卡和bootloader。台架按启动、传感器、输出映射、门控、通信和资源顺序检查；执行器测试先拆桨或隔离动力。原理见[硬件系统](TECHNICAL.md#ch14)。

<a id="git"></a>
## Git提交、推送与恢复

个人仓库为[orbits-yiyi/px4-system](https://github.com/orbits-yiyi/px4-system)，开发分支沿用learn/px4-v1.17.0。远程约定：

| 名称 | 地址 | 用途 |
|---|---|---|
| origin | https://github.com/PX4/PX4-Autopilot.git | 官方来源 |
| personal | https://github.com/orbits-yiyi/px4-system.git | 个人提交与推送 |

先检查实际状态：

```bash
git remote -v
git branch --show-current
git status --short --branch
```

仅当personal不存在时执行 `git remote add personal https://github.com/orbits-yiyi/px4-system.git`。仓库局部配置 `git config remote.pushDefault personal` 可指定默认推送目标。

保存文档改动：

```bash
git add -- README.md learning
git diff --cached --stat
git diff --cached
git commit -m "docs: organize PX4 v1.17 learning documentation"
git push -u personal learn/px4-v1.17.0
```

核对暂存内容再提交。源码实验按具体路径另行提交，避免git add .混入五个旧版残留目录。推送前检查远端已有历史，不用force覆盖不明内容。

若WSL直连GitHub超时，而已有的127.0.0.1:7897代理可用，仅对本次命令使用代理：

```bash
HTTPS_PROXY=http://127.0.0.1:7897 HTTP_PROXY=http://127.0.0.1:7897 \
  git push -u personal learn/px4-v1.17.0
```

登录状态用gh auth status核对，网络超时与凭据失效分开判断。代理地址来自你现有环境，换机器后需核对。

首次推送完成后，GitHub应能直接打开首页和三份文档；若默认页没有显示学习分支，在仓库设置中将默认分支设为learn/px4-v1.17.0。

另一目录或机器恢复源码：

```bash
git clone --branch learn/px4-v1.17.0 --recurse-submodules https://github.com/orbits-yiyi/px4-system.git
```

再用embedded-lab绑定新源码路径和实际工具环境。Git不保存正在运行的容器、全部参数或大日志；子模块内部修改需分别提交并能从远端获取，主仓库只记录其提交指针。

<a id="troubleshooting"></a>
## 常见问题

| 现象 | 首先检查 |
|---|---|
| lab找不到 | 使用~/embedded-lab/bin/lab完整路径 |
| 镜像尚未准备 | 任务配方、实际镜像和锁状态 |
| CMakeCache绝对路径不同 | 是否混用/workspace和/workspaces |
| ccache不能写 | 容器UID/GID、挂载和目录权限 |
| gz或Protobuf NOTFOUND | 当前镜像是否真的装好依赖 |
| APT 502 | 代理和软件源下载链路 |
| 新模块命令不存在 | 板级开关、构建结果、实际运行二进制 |
| GitHub TLS超时 | 当前终端网络及代理 |

make Error 1通常不是根因，向上找第一项实际错误。确认ccache问题时可单次加env CCACHE_DISABLE=1诊断。

仅在确认构建来自错误绝对路径或不兼容工具环境，且已保存其中需要的日志和参数时，才删除build/px4_sitl_default后重建。这个目录含rootfs，清理会一并删除运行状态；日常增量构建无需清理。

<a id="files"></a>
## 文档和笔记的位置

仓库根README是唯一总入口。ROADMAP安排顺序，TECHNICAL解释实现，OPERATIONS保存命令。后续直接修改对应文件，随Git提交。

examples保存示例源码；records保存个人笔记；records/previous保存历史资料，其中旧状态仅代表当时情况。当前操作以本手册和实际Git、任务状态为准。
