# PX4 操作与调试手册

[文档首页](README.md) · [系统架构与技术原理](TECHNICAL.md) · [代码架构与开发指南](DEVELOPMENT.md) · [学习路线](ROADMAP.md)

本手册把学习路线转为具体工作：建立系统模型、读懂工程、编译调试、配置载具、分析测量、验证算法、规划任务、集成应用和完成开发。每个阶段给出实施步骤、结果解释和完成标准。首次接触 PX4 时，从阶段 1 开始；处理具体问题时，可直接查对应专题。

涉及动力、模式切换和故障注入的入门实验均在仿真中进行。真实硬件操作集中在阶段 9，采用单独的设备识别与台架流程。文中的示例数据是推演条件，不代表读者已有实验结果。

## 目录

1. [整体架构：建立系统模型](#s1)
2. [飞行基础：计算与模型推演](#s2)
3. [软件工程：代码、构建、运行与调试](#s3)
4. [测量系统：传感器与数据处理](#s4)
5. [状态估计：融合、失效与重置](#s5)
6. [运动控制：算法、执行能力与性能](#s6)
7. [导航与安全：任务规划和场景验证](#s7)
8. [通信与应用：接口设计和系统集成](#s8)
9. [硬件与机型：部署、台架和迁移](#s9)
10. [综合工程：从需求到验证](#s10)
11. [开发与调试方法：按问题选择工具](#methods)

附录：[环境与版本](#environment) · [工具速查与排错](#tools) · [Git 工作方式](#git) · [Offboard 接口实验程序](#offboard-example)

## 执行位置约定

| 标记 | 所在环境 | 使用方式 |
|---|---|---|
| 宿主 | WSL/Linux 的项目终端 | Git、编辑文件、通过 `~/embedded-lab/bin/lab` 调用开发环境 |
| 容器 | `~/embedded-lab/bin/lab shell px4-main` 打开的 Bash | 编译器、GDB、Python 分析工具，源码路径为 `/workspaces/PX4-Autopilot` |
| PX4 | 仿真启动后的 `pxh>` | PX4 模块、参数和状态命令，不是 Linux Shell |
| GDB | 调试器的 `(gdb)` | 断点、单步、栈和变量操作 |
| QGC | 地面站界面 | 设备配置、任务规划、模式请求与状态查看 |
| ROS | 已加载 ROS 和消息工作区的终端 | ROS 节点、topic、QoS 与应用集成 |

代码块不包含终端提示符。不要把 `pxh>` 或 `(gdb)` 一并复制。开发环境尚未准备时，先完成不依赖工具的阶段 1、2，再按[环境附录](#environment)准备阶段 3。

<a id="s1"></a>
## 1. 整体架构：建立系统模型

阅读：[技术说明第 1 章](TECHNICAL.md#c1)。这一阶段的成果是一张能解释完整任务的系统图。

### 1.1 将任务需求分配到系统组成

采用一个贯穿后续练习的虚构需求：四旋翼在已知区域起飞，访问两个观察点，停留采集图像，返回并降落；操作者能够中途取消任务。暂不加入自动识别和避障。

先填下面的责任表，再画连接关系：

| 需求 | 主要承担者 | 必需信息 | 确认完成的证据 |
|---|---|---|---|
| 获得机体运动 | 传感器、驱动、测量处理 | 单位、方向、时间和设备健康 | 测量持续更新，质量满足要求 |
| 知道当前位置与姿态 | 估计器 | IMU 与相应外部观测 | 状态有效，参考明确 |
| 按顺序访问观察点 | 导航、轨迹生成、控制器 | 任务、状态、运动限制 | 任务推进及实际轨迹 |
| 采集图像 | 相机及任务/应用接口 | 触发、曝光反馈和时间对应 | 设备反馈与图像文件 |
| 中途取消 | 操作者、通信、系统管理 | 请求、当前模式、可执行能力 | 实际转入接管行为 |
| 返回并落地 | 导航、控制、落地检测 | 返回点、高度、位置、能量 | 返回过程、落地和上锁状态 |

例如“任务项已经发出拍照指令”只能证明飞控执行到该动作，尚不能证明相机成功保存图片。责任表应暴露这种跨组件边界。

### 1.2 画出四类联系

第一遍画物理连接：供电、传感器、飞控、执行器、通信、载荷。第二遍在飞控内部画估计、管理、导航、控制和平台服务。第三遍补上四类箭头：

1. **数据**：谁产生测量、状态、目标，谁消费它们。
2. **控制权**：谁可以请求模式，谁判定请求是否成立，谁决定输出门控。
3. **配置**：机型、几何、校准、参数和任务如何进入系统。
4. **证据**：哪些事件、状态和日志可以证明行为发生。

把“修改巡航速度”“GNSS 失效”“新增视觉定位”“飞到点后拍照”放到图上解释。若所有需求都指向控制器，说明尚未分清系统职责。

### 1.3 用一次行为贯穿架构

在纸上推演自动起飞：请求进入 → 条件检查 → 解锁和行为选择 → 起飞目标 → 连续轨迹 → 控制 → 分配 → 物理运动 → 测量和估计反馈。旁边单独列出模式、解锁、落地三个状态维度。

然后推演“请求被拒绝”和“请求接受但尚未离地”。这两个情形不需要故障代码就能解释：前者停在管理条件，后者可能仍处于起飞建立推力的过程。不能只用一个“起飞成功/失败”标签描述全部过程。

**完成标准**：能够不看源码讲清整体组成，并把一个新需求分配到正确层级。源码入口表可以查阅，不要求记住文件名。

<a id="s2"></a>
## 2. 飞行基础：计算与模型推演

阅读：[技术说明第 2 章](TECHNICAL.md#c2)。本阶段使用明确数值建立物理判断能力。

### 2.1 坐标、旋转与参考

完成三个计算，并把坐标、单位、假设写在结果旁边：

| 问题 | 计算结果 | 检查重点 |
|---|---|---|
| ENU 位移 `[2,3,4] m` 转 NED | `[3,2,-4] m` | 两者同原点，只转换向量表示 |
| 水平机体朝东，机体前向速度为 `2 m/s` | NED 速度 `[0,2,0] m/s` | 机体系与地理系不能直接混用 |
| 地面局部 z 为 −0.5 m，空中 z 为 −3.5 m | 相对上升 3 m | 两样本之间参考连续，无未处理重置 |

再令机体从朝北转到朝东：绕 NED 正 z 轴的偏航为 +90°，对应四元数约为 `[0.7071,0,0,0.7071]`。用旋转矩阵验证机体前向映射到东向；仅比较欧拉角数字不能替代这一步。

最后假设原地发生 z 重置，z 从 −0.5 变成 −1.0。分别解释坐标差、真实运动、reset counter 与 delta 的关系。这是以后处理定位跳变和外部视觉的基础。

### 2.2 力、倾斜与执行能力

设四旋翼质量为 1.5 kg、重力加速度取 9.81 m/s²、忽略阻力：

```text
水平悬停总推力 = mg = 14.715 N
倾斜 30° 且保持高度：T cos(30°) = mg
T = 16.99 N
水平加速度 = T sin(30°) / m = g tan(30°) ≈ 5.66 m/s²
```

总推力增加约 15.5%。如果动力系统已经接近最大推力，就不能同时满足任意倾斜和高度保持。把这个结论与控制器的倾角、推力和加速度限制联系起来。

再考虑一个电机作用：旋翼位置向量 r 与推力 F 产生 `r×F` 力矩，旋转阻力还产生反扭矩。改变旋翼位置、推力方向、旋转方向，分别会改变分配矩阵的哪些项？先画方向，再读分配器。

### 2.3 积分误差与时间延迟

假设陀螺仪在 4 ms 内恒定为 0.5 rad/s，则角增量是 0.002 rad。若存在 0.01 rad/s 的未补偿偏置，10 s 后仅由该项产生的角度误差可达 0.1 rad，约 5.73°。这解释了为什么高频积分仍需要其他观测约束。

若高度方向的加速度误差恒定为 0.1 m/s²，初始速度误差为零，10 s 后速度误差是 1 m/s，位置误差是 5 m。该推导忽略其他校正，目的是认识误差积累的量级。

延迟的频域影响可以用纯延迟相位 `−360°×f×τ` 估算：10 Hz 信号延迟 20 ms，对应 −72°。降低噪声的滤波若增加了大量延迟，可能损害控制稳定余量。

### 2.4 运动约束与任务尺度

速度为 5 m/s、可用恒定减速度为 2 m/s² 时，理想刹车时间 2.5 s、距离 6.25 m。若反应延迟为 0.2 s，仅延迟段又前进约 1 m。实际还有 jerk、风、姿态和执行器限制。

据此判断：两个距离仅 5 m 的点，能否要求飞机以 5 m/s 到点后立即静止？应修改速度和轨迹要求，不能把无法实现的任务全归咎于 PID。

**完成标准**：能自行检查一个公式或接口的单位、符号、参考和时间，并用量级计算发现明显不合理的设计。

<a id="s3"></a>
## 3. 软件工程：代码、构建、运行与调试

阅读：[开发指南第 1～6 章](DEVELOPMENT.md#d1)。本阶段建立之后所有源码实验都会使用的开发方法。

### 3.1 从功能找到代码，再找到构建与启动

先按职责认识目录，再选一个模块纵向研究。不要从根目录开始逐文件阅读。

| 研究对象 | 阅读入口 | 要回答的问题 |
|---|---|---|
| 平台与通信机制 | `platforms/`、`platforms/common/uORB/` | 运行环境如何提供调度、时间和消息 |
| 系统模块 | `src/modules/` | 输入、状态、参数和输出怎样组合成功能 |
| 算法 | `src/lib/` 及模块内算法子目录 | 数学计算与平台封装如何分离 |
| 驱动 | `src/drivers/`、`src/lib/drivers/` | 设备与规范物理消息如何转换 |
| 配置与启动 | `boards/`、`ROMFS/`、`Kconfig` | 什么代码被编入、什么模块被启动 |
| 接口与生成代码 | `msg/`、模块元数据、构建目录 | 原始定义在哪里，产物怎样生成 |

用 [work_item](../src/examples/work_item) 建立第一张模块卡片。依次打开 `CMakeLists.txt`、`Kconfig`、`WorkItemExample.hpp`、`WorkItemExample.cpp`，填入：

```text
目录：src/examples/work_item
构建目标：examples__work_item
运行命令：work_item_example
运行上下文：ScheduledWorkItem
触发：sensor_accel 的订阅回调
其他输入：vehicle_status、parameter_update
输出：orb_test
退出：停止调度、释放模块资源
```

在宿主核对配置与接口：

```bash
rg -n 'WORK_ITEM|work_item' boards/px4/sitl src/examples/work_item
rg -n 'registerCallback|Run\(|publish|updateParams|should_exit' src/examples/work_item
rg --files msg | rg 'SensorAccel|VehicleStatus|OrbTest|ParameterUpdate'
```

接着研究真实模块 `mc_pos_control`：模块封装在 `MulticopterPositionControl.cpp`，算法在 `PositionControl/`。把订阅、模式门控、参数更新、算法调用、发布分别标出来。这样能看清“算法正确”与“模块正确运行”是两层问题。

### 3.2 理解并完成一次构建

确认[环境附录](#environment)中的源码挂载关系后，在宿主执行：

```bash
cd /home/yiyi/Workspace/px4/PX4-Autopilot
~/embedded-lab/bin/lab exec px4-main -- make px4_sitl_default -j2 PX4_MAKE_ARGS=-j2
```

构建过程分为配置、生成和编译链接。`boards/px4/sitl/default.px4board` 选择功能，CMake 整理依赖和代码生成，Ninja 执行编译。这里同时限制外层 Make 和内层构建器的并行数。

构建完成后检查产物，而不是只看最后一行输出：

```bash
ls -l build/px4_sitl_default/bin/px4
rg -n 'CMAKE_HOME_DIRECTORY|CMAKE_BUILD_TYPE' build/px4_sitl_default/CMakeCache.txt
ls -l build/px4_sitl_default/compile_commands.json
```

`bin/px4` 是 SITL 程序，`CMakeCache.txt` 记录配置和绝对路径，`compile_commands.json` 记录每个源文件的真实编译参数。`.px4board`、CMake 和源码是输入，`build/` 是生成物。修改生成的头文件并不能形成可重现的源码改动。

再次运行相同构建命令应主要复用已有产物。更改一个 `.cpp` 后观察重编范围；更改消息定义或公共头文件可能触发更多依赖重编。遇到错误，从第一条编译器/CMake 错误开始读，末尾 `Error 1` 只是失败汇总。

### 3.3 建立准确的编辑器索引

在容器开发入口打开项目，确认编辑器所见路径为 `/workspaces/PX4-Autopilot`。已有 Dev Container 设置时，将 C/C++ 扩展的 `C_Cpp.default.compileCommands` 指向：

```text
/workspaces/PX4-Autopilot/build/px4_sitl_default/compile_commands.json
```

使用该数据库跳转 `ORB_ID(vehicle_local_position)`、`PositionControl::update()` 和参数包装类。若编译成功但编辑器报缺头文件，先核对索引是否运行在相同环境、是否采用当前构建数据库；不要凭编辑器红线给工程添加无关 include 路径。

构建配置改变后更新索引。研究 `#if` 分支时，区分当前目标实际启用的代码和源码中存在但未编译的代码。

### 3.4 仿真与模块运行

在宿主启动 SIH：

```bash
~/embedded-lab/bin/lab exec px4-main -- make px4_sitl sihsim_quadx -j2 PX4_MAKE_ARGS=-j2
```

出现 `pxh>` 后，先研究运行系统，不要求立即起飞。依次在 PX4 输入：

```text
ver all
param show SYS_AUTOSTART
commander status
work_queue status
uorb top
```

`uorb top` 按回车结束观察并返回提示符。将实际机型与 [10040_sihsim_quadx](../ROMFS/px4fmu_common/init.d-posix/airframes/10040_sihsim_quadx)、[rcS](../ROMFS/px4fmu_common/init.d-posix/rcS)、[rc.mc_apps](../ROMFS/px4fmu_common/init.d/rc.mc_apps) 对照，解释启动顺序与默认值。

再运行示例模块：

```text
work_item_example start
work_item_example status
listener orb_test
work_item_example stop
```

示例发布的 `val=314159` 只是教学数据。停止后主题的最后一个样本可能仍可读取；要通过时间戳与更新率判断是否继续发布。不要把“主题仍然存在”当成“模块仍在运行”。

`px4_simple_app` 的示例会发布合成姿态，不能未经检查就与正常估计器并行用于飞行实验。选择教学模块也需要检查其输出影响。

### 3.5 完成一个有明确边界的源码实验

按[开发指南第 9 章的 9.1～9.6 节](DEVELOPMENT.md#d9)完成参数驱动的 WorkItem 实验：参数定义和构建注册 → 定时调度 → 消息发布 → 参数更新 → 停止/重启验证，再整理为独立模块并定义自己的消息。该处提供完整修改位置与代码，本章不维护另一份略有不同的实现。9.7 节的算法测试放到阶段 6。

先预测周期从 100 ms 改为 200 ms 会产生什么变化，再编译并记录实际消息频率、参数生效延迟和停止行为。若结果不符，分别检查是否运行新二进制、参数是否进入构建、是否收到参数更新、是否仍注册旧回调；不要同时修改所有层。

这一练习的通过条件是能解释行为和定位失败。完成后对照真实 `mc_pos_control` 的输入、调度和模式条件，明确教学模块尚未涉及的闭环与异常处理。

### 3.6 追踪参数的一次完整生命周期

以 `MIS_TAKEOFF_ALT` 为例，宿主搜索：

```bash
rg -n 'MIS_TAKEOFF_ALT|_param_mis_takeoff_alt' src/modules/navigator ROMFS
```

找到定义、默认值、包装成员和使用点。再在 QGC 参数页或 PX4 `param show MIS_TAKEOFF_ALT` 读取实际值。说明保存值、airframe 默认值和源码默认值的区别。

在仅用于仿真的运行中记录原值，再修改一个目标高度，观察参数读回。行为实验放到阶段 6；此处先确定消费者在何时读取它。实验结束恢复原值并再次读回，不能假设重启必然恢复默认配置。

### 3.7 用 GDB 理解算法调用

先调试输入确定的单元测试。宿主：

```bash
~/embedded-lab/bin/lab exec px4-main -- make tests -j2 PX4_MAKE_ARGS=-j2 PX4_CMAKE_BUILD_TYPE=Debug TESTFILTER='^unit-rate_control_test$'
~/embedded-lab/bin/lab shell px4-main
```

容器：

```bash
gdb -nx --args build/px4_sitl_test/unit-rate_control_test
```

GDB：

```text
break RateControl::update
run
bt
info args
print dt
next
continue
quit
```

将调用栈与 [rate_control_test.cpp](../src/lib/rate_control/rate_control_test.cpp) 对照，确认测试怎样构造输入、算法怎样产生输出。已有零输入用例适合熟悉调用过程；非零 P/I/D 行为在阶段 6 单独研究。

断点不命中时检查二进制、测试筛选和实际执行路径；显示 `optimized out` 时检查 Debug 构建与该源文件编译选项。单步进入函数用 `step`，只执行下一源代码行用 `next`，查看所有线程栈用 `thread apply all bt`。

接着调试完整 SITL 的模块调度。结束其他 SITL 实例，在宿主将默认目标编为 Debug，再进入容器：

```bash
~/embedded-lab/bin/lab exec px4-main -- make px4_sitl_default -j2 PX4_MAKE_ARGS=-j2 PX4_CMAKE_BUILD_TYPE=Debug
~/embedded-lab/bin/lab shell px4-main
```

在容器以 SIH 目标相同的环境变量和运行目录启动调试器：

```bash
cd /workspaces/PX4-Autopilot/build/px4_sitl_default/rootfs
env PX4_SIM_MODEL=sihsim_quadx PX4_SIMULATOR=sihsim gdb -nx --args ../bin/px4
```

`-nx` 跳过初始化脚本，避免生成的 `.gdbinit` 提前执行 `run`。在 GDB 中设置一个示例模块断点并运行：

```text
handle SIGCONT nostop noprint nopass
break WorkItemExample::Run
run
```

程序启动显示 `pxh>` 后输入 `work_item_example start`，命中断点时会回到 GDB。用 `bt`、`info threads`、`print _armed`、`next` 查看工作队列的调用上下文；用 `disable` 禁用断点，再 `continue` 返回运行。结束时在 PX4 停止模块、执行 `shutdown`，最后退出 GDB。

这个流程不需要 attach 到既有进程。若调试器报告权限问题，检查任务容器的调试权限配置；若找不到启动资源，核对 binary、rootfs 和模型变量。不要用其他仿真器的目标名称替代这些条件。

调试结束后，在宿主显式恢复常规 SITL 构建类型，仍复用原目标目录：

```bash
~/embedded-lab/bin/lab exec px4-main -- make px4_sitl_default -j2 PX4_MAKE_ARGS=-j2 PX4_CMAKE_BUILD_TYPE=RelWithDebInfo
```

暂停会影响整个进程的时序与链路，所以用断点检查局部状态，用不中断运行的日志分析飞行性能。编辑器调试器同样需要填写程序路径、rootfs 工作目录和模型环境变量；参考 [PX4 的 launch 模板](../platforms/posix/Debug/launch_sitl.json.in)，不要只配置一个可执行文件路径。

### 3.8 按问题选择调试方式

| 问题 | 首选方法 | 产出 |
|---|---|---|
| 没编进程序、符号不存在 | Kconfig/CMake、编译数据库、构建日志 | 哪个配置或依赖阻断了构建 |
| 模块未运行或没数据 | 启动脚本、模块 status、uORB 频率/时间戳 | 启动、触发和发布的具体断点 |
| 数值计算错误 | 小输入测试、GDB、断言 | 输入、内部量、预期与实际差异 |
| 跨模块行为异常 | 模式、目标、状态的同时间日志 | 异常从哪一层开始传播 |
| 延迟、丢样、CPU 高 | perf、任务/队列统计、日志间隔 | 执行与等待的分布、负载条件 |
| 内存越界或未定义行为 | 合适平台上的 sanitizer 构建与可复现输入 | 错误位置、调用栈和触发条件 |

**完成标准**：能从需求找到模块，解释如何编入与启动，完成一次增量修改，并用测试、调试器或运行证据确认改动。编译、调试和源码研究从此成为后续各阶段的共同工具。

<a id="s4"></a>
## 4. 测量系统：传感器与数据处理

阅读：[技术说明第 3 章](TECHNICAL.md#c3)、[第 5 章](TECHNICAL.md#c5)。先解释测量代表什么，再研究消息和驱动。

### 4.1 建立传感器误差表

选择 IMU、GNSS、气压计三个对象，分别记录物理量、单位、坐标、采样时间、量程、主要误差和下游消费者。对 IMU 加入安装旋转、FIFO、积分周期和饱和；对 GNSS 加入质量、天线杆臂和延迟；对气压加入天气与气流影响。

对于静止水平 IMU，预测陀螺仪接近零、加速度计 FRD z 接近 −g。对于匀速直线运动，理想加速度计并不能直接告诉你速度是多少。先给出预测，再用数据核对，能避免把静止比力误判成运动加速度。

### 4.2 对照设备消息、积分消息与控制输入

启动仿真后，在 PX4 分别读取：

```text
listener sensor_accel
listener sensor_gyro
listener vehicle_imu
listener vehicle_angular_velocity
```

比较单位、时间和设备标识。用 `delta_angle / (delta_angle_dt × 1e-6)` 估算积分区间的平均角速度，再与相同时段角速度对照；二者处理链和带宽不同，不应要求逐点完全相同。

随后阅读 [VehicleIMU.cpp](../src/modules/sensors/vehicle_imu/VehicleIMU.cpp)、[VehicleAngularVelocity.cpp](../src/modules/sensors/vehicle_angular_velocity/VehicleAngularVelocity.cpp)。画出各自校准、选择、积分/滤波、发布的顺序，并写清 EKF 与角速度控制器为什么需要不同的数据形式。

### 4.3 静态与动态数据分析

采用一段静止数据和一段可解释的运动数据；先确认所需主题已记录，可用[日志方法](#log-analysis)查看。对每个轴计算均值、标准差、峰值和样本间隔分布。静止均值可揭示偏置或重力方向，但动态均值不能直接当零偏。

若有频谱工具，使用近似均匀采样、足够长的稳定片段，并说明采样率、窗函数和频率分辨率。短促运动包含真实低频分量；不能看到峰就全部归为电机振动。仿真噪声特征也不能代替实际安装的振动评价。

### 4.4 阅读设备驱动与校准过程

阅读 [BMI088_Gyroscope.cpp](../src/drivers/imu/bosch/bmi088/BMI088_Gyroscope.cpp) 的 `RunImpl()`、`FIFORead()` 与 `ConfigureGyro()`。画 RESET、配置、读 FIFO、异常恢复之间的转移，标出每次等待由调度还是阻塞实现。

沿数据追到 [PX4Gyroscope.cpp](../src/lib/drivers/gyroscope/PX4Gyroscope.cpp)，说明设备数值怎样换算为 rad/s、时间戳怎样对应样本、设备错误怎样报告。用“读成功但时间错了”和“读失败”两个情形检验你的解释。

有实物后按 [QGC 传感器配置说明](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/sensors_px4.html) 完成安装方向与标定；保持电机停止，在已知姿态下检查方向。保存设备 ID 和校准前后配置，不能把模拟器的校准结果复制到实物。

**完成标准**：能够从异常数据回到时间、安装、设备、校准、滤波中的具体层，并说明需要什么实验才能区分候选原因。

<a id="s5"></a>
## 5. 状态估计：融合、失效与重置

阅读：[技术说明第 6 章](TECHNICAL.md#c6)。按数学更新、信息源管理、系统输出三个层次学习。

### 5.1 手算更新与门限

复算技术说明中的高度例子：预测 2.0 m、方差 0.25 m²，观测 2.4 m、方差 0.75 m²，更新结果 2.1 m、方差 0.1875 m²。保持其他条件不变，将 R 改为 0.25 m²，结果应为 2.2 m、方差 0.125 m²。

这表示更信任观测，不表示观测实际更准确。再设创新为 4 m、创新方差为 1 m²、门限为 3，则检验比为 `16/9>1`。解释接受观测、拒绝观测、融合超时与参考切换四个不同动作。

### 5.2 用现有测试研究高度参考

宿主运行：

```bash
~/embedded-lab/bin/lab exec px4-main -- make tests -j2 PX4_MAKE_ARGS=-j2 TESTFILTER='^unit-test_EKF_height_fusion$'
```

阅读 [test_EKF_height_fusion.cpp](../src/modules/ekf2/test/test_EKF_height_fusion.cpp) 中的 `baroRef`，按代码列出输入变化与断言：

| 输入步骤 | 测试检查什么 | 技术含义 |
|---|---|---|
| 使用气压高度参考，并启用其他高度源 | 参考与融合标志 | 参考选择与融合启用不同 |
| 气压高度增加 4 m，运行给定时长 | 局部 z 约为 −4 m，相关偏置变化 | 符号、参考和偏置估计协同 |
| 气压再跳变 200 m | 气压融合停止，GNSS 成为参考 | 持续异常触发信息源管理 |
| GNSS 高度增加 1 m | 估计跟随新的参考 | 切换后状态仍需保持合理关系 |

这里的时长和误差容限属于测试条件，不是所有飞机都会在同样秒数内切换。可复制该用例增加一个自拟场景，但先写预期与理由，再改输入；不能为使测试通过随意放宽断言。

### 5.3 连接到实际融合路径

沿 `EKF2::UpdateGpsSample()` → `estimator_interface.cpp` → `gps_control.cpp` → `gnss_height_control.cpp` → `PublishLocalPosition()` 阅读。每一处只记录四项：输入、处理、退出/拒绝条件、下一层。

再打开对应 GNSS aid-source 消息，查看 observation、innovation、variance、rejected 和最后融合时间。使用当前运行实例的日志；多 EKF 情况下必须确认被选择的估计器，不能混用不同实例曲线。

### 5.4 区分真实运动与估计跳变

对一个 z 突变片段，在同一时间轴放置：位置、速度、reset counter、垂直观测、目标、推力及模式。先判断是否有重置，再判断是否发生物理运动。

若只有估计状态跳变而物理真值连续，应检查参考、观测和重置处理。若目标、速度、推力、真值均支持该运动，再分析控制原因。仿真真值可用于误差评价，但要先对齐坐标、时间和参考；真实飞行没有独立真值时，不能用估计器自身输出证明绝对精度。

**完成标准**：能解释一条融合路径的正常、拒绝、超时和恢复过程，并能说明一次位置跳变需要哪些证据才能定性。

<a id="s6"></a>
## 6. 运动控制：算法、执行能力与性能

阅读：[技术说明第 7 章](TECHNICAL.md#c7)、[第 13 章](TECHNICAL.md#c13)。把目标生成、闭环控制和实际输出分开分析，再连接起来。

### 6.1 计算一轮控制并对照实现

先用简化单轴角速度控制做数值练习，假设本轮已有积分项为 0.02，`Kp=0.15`、`Kd=0.003`、`Kff=0.01`，目标角速度 1 rad/s、实际 0.4 rad/s、实际角加速度 2 rad/s²：

```text
error = 1 - 0.4 = 0.6 rad/s
u = Kp × error + I - Kd × angular_acceleration + Kff × rate_setpoint
  = 0.09 + 0.02 - 0.006 + 0.01 = 0.114
```

这是给定归一化控制系数的教学算例，不是推荐调参值。若 Ki 为 0.1、dt 为 0.004 s，未考虑积分抑制与饱和时，本次积分增量为 0.00024。随后阅读 [rate_control.cpp](../src/lib/rate_control/rate_control.cpp)，解释实际实现为什么还要限制积分、考虑饱和方向和非有限数值。

### 6.2 从已有测试认识算法契约

宿主执行位置控制测试：

```bash
~/embedded-lab/bin/lab exec px4-main -- make tests -j2 PX4_MAKE_ARGS=-j2 TESTFILTER='^unit-PositionControl$'
```

阅读 [PositionControlTest.cpp](../src/modules/mc_pos_control/PositionControl/PositionControlTest.cpp)，从 `PositionDirection`、`VelocityLimit`、`TiltLimit`、`IntegratorWindupWithInvalidSetpoint` 中分别选方向、约束和失效问题。画出输入、预期方向/边界、实际断言。

接着按研究问题选择测试，名称见[工具附录](#test-tools)。姿态测试研究旋转误差，分配测试研究力矩/推力可实现性，轨迹平滑测试研究速度、加速度与 jerk。每次选一个机制，避免一次运行全部测试后不知道结果证明了什么。

再按[开发指南 9.7 节](DEVELOPMENT.md#algorithm-exercise)建立独立的一维参考限速库及测试，完成“行为要求 → C++ 实现 → CMake 注册 → 定向运行 → 故意破坏后确实失败 → 恢复”。这一步把会运行已有测试推进到能为自己的改动设计证据；算法先通过测试，再接入教学模块观察，不直接发布飞行控制目标。

### 6.3 用仿真区分轨迹变化和控制响应

先在 SIH 完成一次默认起飞、保持、降落，流程见[仿真工具](#simulation-tools)。记录当前 `MIS_TAKEOFF_ALT`，得到基线日志。再在地面把它设为另一个明确高度，例如 4 m，重复相同步骤，结束后恢复原值。

对照 `trajectory_setpoint`、`vehicle_local_position_setpoint` 与 `vehicle_local_position`：最终目标变化、爬升过程中的目标生成、实际跟踪是三个不同观察对象。这项实验改变导航要求，不能称为 PID 调参实验。

要研究轨迹限制，在阶段 7 使用同一路线，保持控制增益不变，仅改变 `MPC_XY_CRUISE` 的一个值；先记录原值与其他加速度/jerk 限制。若路线太短，可能从未达到巡航速度，应先分析目标轨迹而不是判定参数无效。

要研究控制增益，先在算法测试中引入固定输入或在仿真中采用重复场景，每次只改变一个目标环的参数，保留原值和恢复步骤。真实飞机上的调参还需满足测量、机械和动力条件，参考 [PX4 多旋翼调参说明](https://docs.px4.io/v1.17/en/config_mc/pid_tuning_guide_multicopter)。

<a id="log-analysis"></a>
### 6.4 从 ULog 建立性能证据

宿主查找实际日志：

```bash
rg --files --no-ignore build/px4_sitl_default -g '*.ulg'
~/embedded-lab/bin/lab exec px4-main -- python3 -c 'import pyulog, numpy; print("analysis dependencies available")'
~/embedded-lab/bin/lab shell px4-main
```

在容器使用 `ulog_info` 查看选定日志的主题和实例，用 `ulog2csv --help` 查看导出选项。日志路径以本次 logger 输出和上述搜索结果为准，命令中的 `YOUR_LOG.ulg` 需替换为实际文件：

```bash
ulog_info YOUR_LOG.ulg
ulog2csv -m vehicle_local_position,trajectory_setpoint,vehicle_attitude,vehicle_attitude_setpoint YOUR_LOG.ulg
```

使用 CSV 绘图、PlotJuggler 或 [pyulog](https://github.com/PX4/pyulog) 分析时，按下列顺序处理：

1. 选择一次清晰动作，标记开始、模式切换、达到目标和结束。
2. 确认目标、反馈采用相同坐标和单位；姿态比较需要旋转误差，不能直接相减四元数。
3. 按时间戳对齐。目标可按其保持语义取前一值；状态插值仅用于足够短且连续的间隔。
4. 标出数据空洞、reset、无效状态和饱和，不跨这些边界盲目插值。
5. 分开评价瞬态与稳定阶段，并保存原配置、场景与评价窗口。

对连续等间隔误差序列，可计算 `RMSE=sqrt(mean(error²))`；不等间隔数据先重采样或使用时间加权。报告峰值、稳定偏差、响应时间与饱和时长，不能只留一个平均数。没有真实位置基准时，“目标与估计之差”是跟踪指标，不是绝对定位误差。


**生成一张可复查的垂直跟踪图。** 完成概念核对后，可把下面代码保存为 `build/learning-experiments/plot_vertical.py`。它使用 pyulog、numpy、matplotlib；在具备这些分析依赖的容器内运行。脚本只接受一个无高度重置、目标有效且记录连续的窗口，比较控制器局部高度目标与估计反馈；它不把此误差称为绝对定位误差。

先在项目目录执行 `mkdir -p build/learning-experiments` 创建存放位置。若所需消息未记录，需先调整日志配置并重新录制，无法仅靠脚本从现有文件恢复缺失主题。

```python
import argparse
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from pyulog import ULog

parser = argparse.ArgumentParser()
parser.add_argument("log")
parser.add_argument("--start", type=float, required=True)
parser.add_argument("--end", type=float, required=True)
parser.add_argument("--max-age", type=float, default=0.25)
parser.add_argument("--out", default="/tmp/px4-vertical.png")
args = parser.parse_args()
if args.end <= args.start or args.max_age <= 0:
    raise SystemExit("Require end > start and max-age > 0")
ulog = ULog(args.log)
pos = ulog.get_dataset("vehicle_local_position").data
sp = ulog.get_dataset("vehicle_local_position_setpoint").data
origin = float(ulog.start_timestamp)
t = (pos["timestamp"].astype(float) - origin) / 1e6
ts = (sp["timestamp"].astype(float) - origin) / 1e6
window = (t >= args.start) & (t <= args.end)
tw, z = t[window], pos["z"][window]
reset = pos["z_reset_counter"][window]
if len(tw) < 2 or len(ts) == 0:
    raise SystemExit("Missing samples in selected window")
if np.any(np.diff(ts) < 0) or np.any(np.diff(tw) <= 0):
    raise SystemExit("Timestamps are not ordered")
if np.any(reset[1:] != reset[:-1]):
    raise SystemExit("Height reset: split the analysis window")
index = np.searchsorted(ts, tw, side="right") - 1
if np.any(index < 0):
    raise SystemExit("No earlier setpoint for some samples")
zs = sp["z"][index]
valid = (pos["z_valid"][window].astype(bool)
         & np.isfinite(z) & np.isfinite(zs)
         & ((tw - ts[index]) <= args.max_age))
dt = np.diff(tw)
if not np.all(valid) or np.any(dt > args.max_age):
    raise SystemExit("Invalid/stale samples or long gaps: inspect and split window")
height, target = -z, -zs
error = height - target
rmse = np.sqrt(np.sum(0.5 * (error[:-1]**2 + error[1:]**2) * dt) / np.sum(dt))
bias = np.sum(0.5 * (error[:-1] + error[1:]) * dt) / np.sum(dt)
print(f"window={tw[0]:.3f}..{tw[-1]:.3f}s samples={len(tw)}")
print(f"time-weighted RMSE={rmse:.3f}m bias={bias:.3f}m peak={np.max(np.abs(error)):.3f}m")
fig, axes = plt.subplots(2, 1, sharex=True, figsize=(10, 6))
axes[0].plot(tw, target, label="local position setpoint (ZOH)")
axes[0].plot(tw, height, label="estimated height")
axes[0].set_ylabel("Height from local origin [m]")
axes[0].legend()
axes[1].plot(tw, error)
axes[1].set_ylabel("Estimate - target [m]")
axes[1].set_xlabel("Time from log start [s]")
for axis in axes:
    axis.grid(True)
fig.tight_layout()
fig.savefig(Path(args.out), dpi=160)
print(args.out)
```

先从日志与模式时间线选定窗口，再在容器运行；下面 10～20 秒只是参数格式示例，不代表你的日志已经在此时起飞：

```bash
python3 build/learning-experiments/plot_vertical.py YOUR_LOG.ulg --start 10 --end 20 --out /tmp/px4-vertical.png
```

程序拒绝窗口时，查看对应的无效、重置或空洞，缩小到连续阶段，不要直接删掉检查。`max-age` 是本次分析对记录密度的要求，不是 PX4 模块内部的超时参数。图中以局部原点为高度参考；要评价相对地面高度，另行确定连续的地面基准。

保存图、输出数值、窗口、参数和原日志。比较两次实验时使用相同事件阶段及评价规则；若要研究轨迹规划，再加入 `trajectory_setpoint`，不能把内部位置控制目标与任务最终航点混为一条曲线。

### 6.5 从逻辑作用追到物理输出

查看 airframe 中旋翼几何，再读控制分配器和输出映射。为每个电机记录位置、轴向、旋转方向、逻辑 Motor 编号和实际通道。

用一个滚转力矩请求推演左右旋翼如何变化，再核对分配结果。若有饱和，检查 `control_allocator_status` 中未分配作用与饱和信息。控制器请求、分配器输出、电调命令、实际转速属于不同层，后者不会因前者存在就自动成立。

**完成标准**：能把跟踪问题分为目标、估计、控制、分配和执行五类，并用一次有控制变量的实验支持判断。

<a id="s7"></a>
## 7. 导航与安全：任务规划和场景验证

阅读：[技术说明第 8 章](TECHNICAL.md#c8)、[第 9 章](TECHNICAL.md#c9)。这一阶段主要使用任务规划、状态机和完整场景，Shell 用于辅助观察。

### 7.1 设计任务的几何与行为

先在纸上定义一项仿真任务：起飞至相对 Home 10 m，在约 20 m 范围内访问两个观察点，各停留 3 s，随后返回并降落。巡航速度先取 2 m/s，检查现有加速度与 jerk 限制。数值仅用于小范围仿真，不是实际场地的飞行建议。

记录任务高度参考、点间距离、到达与停留条件、结束动作、取消动作、返航目的地和返航高度。把计划中的 Home 与运行时实际 Home 分开；若参考不一致，应先修正计划。

先忽略相机动作，完成纯飞行任务；随后再加入载荷接口，才能区分导航失败与载荷失败。

### 7.2 在 QGC 完成规划、上传与核对

前提是已有可运行的 QGC，并连接到本次 SITL。SIH 可以连接地面站，但不提供 Gazebo 式三维场景。网络连接方法见[工具附录](#qgc-tools)。

按 [QGC Plan View](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/plan_view/plan_view.html) 的任务编辑流程进行，界面名称可能随 QGC 版本变化：

1. 在 Plan 视图新建普通航点任务，以模拟器当前位置附近作为计划区域。
2. 添加起飞项与两个航点，逐项检查高度模式、位置、停留和自动继续；不要只看地图连线。
3. 设置明确的结束方式，核对返回高度与落地点。保存 `.plan` 文件。
4. 上传任务，等完整传输成功。再从载具下载任务，对照任务项数量、命令、坐标和高度参考。
5. 回到 Fly 视图，查看健康、Home、实际位置和当前模式。请求开始任务后，观察模式和任务进度。
6. 完成后检查落地、上锁及日志；不要仅凭地图图标停住判断任务完成。

保存的计划文件与下载读回结果分别证明“打算执行什么”和“飞控实际收到了什么”。若当前 QGC 只提供保存/同步提示而没有明显的下载按钮，使用其任务文件/载具同步菜单完成读回核对，不能把本地显示当成载具内容。

### 7.3 解释航点推进与运动轨迹

在日志中按任务项变化标出前一、当前、下一目标，与轨迹、位置、速度对照。阅读 [mission_block.cpp](../src/modules/navigator/mission_block.cpp) 的到达判断，再阅读 [FlightTaskAuto.cpp](../src/modules/flight_mode_manager/tasks/Auto/FlightTaskAuto.cpp) 与运动平滑库。

研究两个变化：先只改变某航点的停留时间，再只改变巡航速度。停留改变任务推进条件，速度改变轨迹时间尺度。若航点被平滑经过，不能把地图点到航迹的距离直接当成位置控制稳态误差。

### 7.4 正常完成、中途取消和返航

用同一计划执行三个独立场景，每次从明确的地面初态开始：

| 场景 | 操作 | 需要确认 |
|---|---|---|
| 正常完成 | 完整执行任务 | 项目推进、结束动作、落地/上锁 |
| 人工取消 | 飞向观察点时请求 Hold，再请求降落 | 实际退出 Mission、目标变化、降落过程 |
| 中途返航 | 飞向第二点时请求 Return | 返回目的地、高度阶段、结束动作 |

请求使用 QGC 模式/动作控件。Shell 中也可观察 `commander status`、`listener mission_result`、`listener vehicle_status`，但日志应保存完整时序。任务暂停后是否恢复到原项、是否重做某项动作，要根据实际任务状态判断。

### 7.5 先在地面研究定位质量退化

在已确认落地且 Disarmed 的 SIH 中，先记录 `SIM_GPS_USED` 原值，再执行：

```text
param show SIM_GPS_USED
param set SIM_GPS_USED 0
listener sensor_gps
commander check
listener failsafe_flags
```

等待质量检查和估计状态发生相应变化，再重复观察。这个参数改变模拟 GNSS 的质量/可用卫星条件；消息仍可能在发布，因此它不是“拔掉 GPS 数据线”的等价实验。系统有备用观测时，某些状态可能继续有效。

随后恢复记录的原值并读回，等待质量与估计恢复后再开始其他实验。不要固定假设原值是某个数字。地面预检变化不能证明空中一定采取某种保护动作。

### 7.6 用测试与场景矩阵研究保护策略

宿主运行：

```bash
~/embedded-lab/bin/lab exec px4-main -- make tests -j2 PX4_MAKE_ARGS=-j2 TESTFILTER='^functional-failsafe_test$'
rg -n 'TEST_F|offboard|position|battery|rtl' src/modules/commander/failsafe/failsafe_test.cpp
```

选一个测试，读清初始模式、异常标志、延时和预期动作。然后为自己的任务制作场景矩阵：当前模式 × 位置是否可用 × 链路状态 × 电量状态。对每格写“可执行的动作”和“负责决定的组件”。

至少比较遥控丢失、地面站丢失和 Offboard 断流，不能用一种断网实验替代三者。空中故障实验应先由现有测试和参数策略明确预期，再在独立仿真中实施。

**完成标准**：能规划并验证一项任务，同时解释完成、取消、返航和一种异常行为的条件及系统实现。

<a id="s8"></a>
## 8. 通信与应用：接口设计和系统集成

阅读：[技术说明第 10 章](TECHNICAL.md#c10)。应用开发从协议契约开始，随后实现数据、请求和控制流程。

### 8.1 为外部程序定义接口

以“伴随计算机在两个点采集图像”为例，先决定使用 Mission 还是 Offboard。固定航点及简单动作可使用任务机制；需要程序实时决定目标时才引入持续外部控制。

写出以下接口表，作为后续代码的输入：

| 内容 | 必须明确的字段 |
|---|---|
| 身份与传输 | system/component、端口方向、网络命名空间 |
| 状态输入 | 位置、姿态、有效性、模式、解锁、时间与重置 |
| 请求 | 命令、确认、拒绝、超时、重试是否可重复执行 |
| 控制输出 | 位置/速度等层级、坐标、单位、频率、有效字段 |
| 退出与恢复 | 人工接管、正常结束、断流、进程重启的行为 |
| 载荷交互 | 请求编号、触发/曝光/完成时间、失败与重试 |

应用的状态机可以从 `等待连接 → 检查状态 → 准备目标 → 请求控制权 → 执行 → 交还控制权 → 完成` 开始，每个状态另设超时和取消出口。收到 ACK 后仍需观察实际模式；任务完成还需等待实际运动或设备反馈。

### 8.2 先验证只读遥测

在 PX4 用 `mavlink status` 确认实际实例和端口。默认单机 SITL 的 Onboard 路径通常将消息发到 UDP 14540。确认没有其他应用占用此端口，在宿主检查依赖：

```bash
~/embedded-lab/bin/lab exec px4-main -- python3 -c 'from pymavlink import mavutil; print("pymavlink available")'
```

把以下程序保存到本地实验目录，例如 `build/learning-experiments/read_position.py`；先创建目录。该目录用于临时运行，计划长期维护的应用应另存为可提交源码。

```python
from pymavlink import mavutil

link = mavutil.mavlink_connection("udpin:127.0.0.1:14540")
try:
    hb = link.wait_heartbeat(timeout=10)
    if hb is None or hb.autopilot != mavutil.mavlink.MAV_AUTOPILOT_PX4:
        raise SystemExit("No PX4 heartbeat")
    print("PX4 identity:", hb.get_srcSystem(), hb.get_srcComponent())
    for _ in range(5):
        msg = link.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=5)
        if msg is None:
            raise SystemExit("No local position: inspect stream and port")
        if msg.get_srcSystem() == hb.get_srcSystem():
            print(msg.time_boot_ms, msg.x, msg.y, msg.z)
finally:
    link.close()
```

在第二个宿主终端运行，保持第一个终端的 SIH 正常运行：

```bash
~/embedded-lab/bin/lab exec px4-main -- python3 build/learning-experiments/read_position.py
```

将接收到的位置与 PX4 的 `vehicle_local_position` 对照。MAVLink 的 `LOCAL_POSITION_NED` 不包含内部消息的全部有效性与重置字段；能读到位置不能替代完整的控制输入健康检查。

### 8.3 在仿真中验证外部控制权交接

[附录程序](#offboard-example)用于单机 SIH 的短时间位置保持接口练习。它不承担完整飞行应用，模式请求由操作者发出。

实验前记录 `COM_OF_LOSS_T`、`COM_OBL_RC_ACT` 原值。只在 SIH 中设置断流后的 Land 策略：

```text
param show COM_OF_LOSS_T
param show COM_OBL_RC_ACT
param set COM_OF_LOSS_T 1
param set COM_OBL_RC_ACT 4
```

本仓库参数定义中 4 表示 Land。先按[仿真流程](#simulation-tools)起飞到稳定 Hold，确认位置有效，然后运行附录程序。它捕获当前 NED 位置并持续发送，不会立即命令飞到原点。

目标流持续至少约两秒后，在 PX4 请求模式并检查实际状态：

```text
listener offboard_control_mode
commander mode offboard
commander status
listener trajectory_setpoint
```

[Offboard 官方说明](https://docs.px4.io/v1.17/en/flight_modes/offboard.html)规定了持续信号及进入条件；本练习使用约 20 Hz 流留出余量。若请求拒绝，检查新鲜度、估计、目标字段和健康条件，不重复强制切换。

第一次验证正常退出：保持程序发送，在 PX4 请求 `commander mode auto:loiter`，确认实际 Hold 后停止程序，再降落。第二次重新起飞、启动目标流并进入 Offboard，然后停止程序，观察断流检测与 Land 过程。对比两次日志的模式变化原因。

最后恢复两个参数原值并读回。切换完成、流停止、实际降落分别是不同事件；程序退出并不等于载具已经安全结束动作。

### 8.4 验证 DDS/ROS 路径

ROS 集成需要四项匹配：PX4 Client、Agent、消息定义和 ROS 环境。安装与版本放在[环境附录](#ros-environment)，不改变主学习顺序。

先只读一条状态消息。确认 PX4 Client 指向可达的 Agent 地址，启动 Agent 后，在已经加载消息工作区的 ROS 终端：

```bash
ros2 topic list
ros2 topic info -v /fmu/out/vehicle_local_position_v1
ros2 topic echo /fmu/out/vehicle_local_position_v1 --qos-reliability best_effort
```

这里的主题名对应附录中的参考实现；实际名称受消息版本和命名空间影响。看到主题但没有样本时，检查会话、domain、消息类型、QoS 和网络，不能只反复编译 PX4。

读通之后再迁移应用状态机。ROS Offboard 的 `OffboardControlMode` 与目标流是不同主题；程序要同时管理控制信号、目标及其有效状态。直接把遥测订阅代码改成 publisher，还没有形成可靠的控制应用。

### 8.5 外部定位与载荷的进一步集成

外部视觉先做一份数据契约：世界系、机体系、原点、单位、时间基准、延迟、协方差、质量和重置。按静止、单轴平移、单轴旋转、短时中断、重定位五种输入验证。

先确认 PX4 收到的观测方向和时间正确，再启用对应融合并观察创新与状态。ROS TF 已存在、图像看起来正常，都不能证明进入 EKF 的坐标与时间正确。

相机集成则记录请求、设备接受、实际曝光、文件保存四个时刻。测试延迟响应和失败重试，防止同一个任务项因重发造成重复拍摄。把图像位姿关联到实际曝光时间，不能直接采用几百毫秒后的文件保存时间。

**完成标准**：能写出明确接口，验证正常、拒绝、断流和恢复过程，并区分消息送达、请求接受、实际行为与业务完成。

<a id="s9"></a>
## 9. 硬件与机型：部署、台架和迁移

阅读：[技术说明第 3 章](TECHNICAL.md#c3)、[第 11 章](TECHNICAL.md#c11)。没有硬件时先完成配置研究和迁移分析，实测部分保留为未验证。

### 9.1 从板卡身份建立资源表

先根据实际板卡型号、硬件修订和厂家资料确定 PX4 目标。以下以 `px4_fmu-v6x_default` 作为源码研究例子，不表示任意 Pixhawk 都应刷这个固件。

依次阅读 [default.px4board](../boards/px4/fmu-v6x/default.px4board)、[board_config.h](../boards/px4/fmu-v6x/src/board_config.h)、[init.cpp](../boards/px4/fmu-v6x/src/init.cpp)、[rc.board_sensors](../boards/px4/fmu-v6x/init/rc.board_sensors)。建立资源表：

| 资源 | 需要记录的关系 |
|---|---|
| 处理器、RAM、Flash | 目标架构、空间预算、启动方式 |
| SPI/I²C 传感器 | 总线、片选/地址、设备型号、方向、中断 |
| UART/CAN | 物理端口、协议、波特率/总线配置、供电 |
| 输出 | 定时器/总线、PWM/DShot/CAN、逻辑功能、物理插座 |
| 电源与存储 | 主备供电、测量、SD 卡、掉电行为 |

对照 [spi.cpp](../boards/px4/fmu-v6x/src/spi.cpp)、[timer_config.cpp](../boards/px4/fmu-v6x/src/timer_config.cpp) 理解资源怎样映射到软件。板级启动、通用应用启动和机型默认值分别在哪一层生效？

### 9.2 编译并识别固件产物

仅当研究目标确为上述板型时，在宿主编译：

```bash
~/embedded-lab/bin/lab exec px4-main -- make px4_fmu-v6x_default -j2 PX4_MAKE_ARGS=-j2
```

检查 `build/px4_fmu-v6x_default/` 中的 ELF、固件包及大小报告。ELF 用于符号与调试，固件包供支持的刷写流程使用；不能把 SITL 可执行文件刷进 MCU。

真实设备刷写按该板卡官方文档与 QGC 的固件流程选择明确文件。刷写前保存当前参数与固件身份；设备校准、启动加载器、硬件修订和机型配置都需要对应，不能只匹配文件名的一部分。

### 9.3 无桨台架的配置顺序

1. 拆除螺旋桨，确认供电与连接符合板卡和外设要求。USB 能连上不证明动力和外设供电正确。
2. 在 QGC 确认固件身份，选择实际机型，核对控制分配几何与飞控安装方向。
3. 标定传感器、遥控和电源测量；手动转动机体，核对姿态和角速度方向。
4. 在执行器配置中逐个测试输出功能，核对物理位置与转向。测试驱动不等于正常解锁流程，仍须按无桨条件操作。
5. 检查通信实例、接管开关、返回与保护配置；先在地面观察输入丢失的状态识别。
6. 查看 CPU、工作队列、内存、栈、日志写入和传感器错误，记录负载条件。

这些步骤验证身份、测量与连接。它们尚不能证明真实带载闭环稳定；进入动力台架和飞行前，要具备相应装配、场地和操作条件。

### 9.4 板上调试与仿真调试的区别

SITL GDB 调试的是宿主进程；MCU 调试通常通过该板支持的 SWD/JTAG 调试器、调试服务器和匹配的 ELF。CPU 架构、接口、复位方式和启动加载器决定具体命令，不应对未知板卡给出统一烧写地址。

有合适硬件后，先建立连接并读取寄存器/调用栈，再对不运行动力的程序设置断点。检查故障现场时保留 PC、LR、SP、线程栈和对应固件提交。断点导致的任务暂停不能用来评价正常控制性能；运行时延迟仍用板上 perf/日志与外部测量研究。

具体的控制台选择、GDB 接入、探针目标与 HardFault 保存步骤见[板上调试方法](#e8)。

### 9.5 用另一机型检验架构理解

至少选一种非四旋翼机型深入研究。先比较启动脚本与控制模块，再建立不同的物理约束：

| 方向 | 具体工作 | 解释目标 |
|---|---|---|
| 固定翼 | 比较 `rc.fw_apps`，计算空速 15 m/s、转弯半径 50 m 的协调转弯倾角约 24.6° | 地速/空速、侧向导引、TECS 与最小飞行速度 |
| VTOL | 画多旋翼、前过渡、固定翼、后过渡状态，标出执行器和回退条件 | 过渡中的控制职责和执行能力如何变化 |
| 车辆 | 计算轴距 0.3 m、速度 1 m/s、转角 20° 时 yaw rate 约 1.21 rad/s | 曲率限制、转向与差速/全向运动差异 |

SIH 支持的模型可在 [simulator_sih/CMakeLists.txt](../src/modules/simulation/simulator_sih/CMakeLists.txt) 查到。选择 `sihsim_airplane` 等已有目标进行独立实验，先确认启动的机型与参数，不能沿用四旋翼的所有操纵、起飞和保护假设。

**完成标准**：能说明真实部署需要补充哪些证据，并能解释同一导航或控制需求移到另一机型时，哪些接口可复用、哪些物理条件必须重建。

<a id="s10"></a>
## 10. 综合工程：从需求到验证

阅读：[技术说明第 12 章](TECHNICAL.md#c12)、[第 13 章](TECHNICAL.md#c13)。这一阶段将知识组合成独立完成项目的能力。

### 10.1 完成一个可恢复的系统应用

把前面的“两观察点巡检”实现为可重复运行的应用：连接载具、核对状态、执行任务、协调图像采集、允许取消、返回或降落、输出运行结果。

先定义约束：支持的机型与环境、控制方式、坐标与高度参考、有效输入、超时、接管、重复启动规则。再选择 Mission 或 Offboard，避免同时存在多个持续发目标的程序。

至少验证五个场景：正常完成、开始前条件不满足、执行中人工取消、通信中断、应用进程重启。重启后先读取实际状态并要求明确恢复策略，不能从第一行代码直接重发起飞请求。

交付一张状态图、接口表、启动方法、配置、场景结果和日志分析。模拟相机响应可以验证接口流程；真实拍摄效果与曝光时间仍需设备实测。

### 10.2 完成一个飞控内部改动

选择与已学机制相连的改动，例如：为一个独立实验模块增加带速度/加速度限制的目标生成算法，或为明确的输入处理路径补充缺失的有效性/重置处理。先提出可验证的行为，不以“改了一个文件”为目标。

采用以下开发流程：

1. **定位职责**：纯数学放算法类；订阅、时钟、参数与发布放模块封装；设备读写放驱动；模式权限保留在系统管理层。
2. **定义接口**：写明字段、坐标、单位、时间、有效性、初始化和停止行为。需要新消息时定义独立主题，避免试验模块与正式控制器争用同一个输出。
3. **建立算法测试**：覆盖零输入、方向、边界、无效输入、复位和连续调用；用可解释数值给出断言。
4. **注册构建**：新模块需要 CMake 的 `MODULE`/`MAIN`/`SRCS`/`DEPENDS`，以及 Kconfig 和目标配置选择。已有模块改动只调整必要部分。
5. **接入生命周期**：实现初始化、触发、参数更新、输入获取、输出与退出清理；共享工作队列内不做无界阻塞。
6. **集成验证**：先用独立教学主题观察，再在受控仿真中设计唯一目标生产者的接入方式，验证正常与退出行为。
7. **回归与资源检查**：运行相关已有测试，核对场景结果、运行耗时和发布频率；涉及硬件路径时补相应台架验证。

以现有 [work_item/CMakeLists.txt](../src/examples/work_item/CMakeLists.txt) 作为模块注册参考，以 [PositionControl/CMakeLists.txt](../src/modules/mc_pos_control/PositionControl/CMakeLists.txt) 作为算法库与测试分离的参考。参数通过工程生成机制加入，不手动修改生成头文件；消息定义改变时核对所有消费者及 DDS/外部类型。

如果选择新增轨迹算法，最低测试包括：当前位置等于目标时不产生无意义运动；反向目标具有正确方向；速度和加速度不超限；异常 dt 或无效输入有规定行为；停止/重启不沿用旧目标。测试中的限制应来自需求，不能直接复制实现表达式作为“预期值”。

### 10.3 完成部署或跨机型验证

有硬件时完成阶段 9 的实测，再验证项目在实际资源与传感器条件下运行。无硬件时完成另一机型的模型、启动、控制、任务和保护分析，并明确哪些判断仅由仿真支持。

如果迁移失败，按接口、物理模型、执行能力和状态机分别定位。例如固定翼不能原地保持，不应通过不断提高位置增益来“修复”。

### 10.4 形成可审查的结果

每个综合项目用一份紧凑记录说明：

```text
需求与适用条件
系统职责和接口
模型或实现的关键选择
源码、依赖、配置和工具身份
正常、边界、异常场景及结果
尚未验证的条件与恢复方法
```

记录应让另一位开发者按步骤复现，并知道结果证明了什么。能够独立完成这种工作，才表明整体架构、技术机制、工程开发与系统应用已经连成体系。

<a id="methods"></a>
## 11. 开发与调试方法：按问题选择工具

前十章按学习阶段安排实践，本章是跨阶段使用的方法库。先确认“想证明什么、问题发生在哪个平台、能否暂停”，再选工具；无需每次把所有方法都跑一遍。首次入门先做阶段 3，随后随问题查阅本章。

| 方法 | 主要用途 | 适用条件 | 入口 |
|---|---|---|---|
| Shell、状态、uORB | 模块、输入、参数和模式是否成立 | 模块已构建且系统可交互 | [11.1](#e1) |
| GDB、编辑器、core | 调用、内部变量、崩溃与并发上下文 | 匹配的符号及可调试进程 | [11.2](#e2) |
| ULog、遥测、埋点 | 连续行为与跨模块时间关系 | 记录或传输所需主题 | [11.3](#e3) |
| Replay | 相同输入下重算算法 | 合适的主题、频率、参数和单实例配置 | [11.4](#e4) |
| perf、任务/队列、栈与内存 | 延迟、阻塞、资源不足 | 平台支持相应统计 | [11.5](#e5) |
| 测试、Sanitizer、覆盖率 | 算法边界、内存错误和回归 | 测试输入及对应工具链 | [11.6](#e6) |
| 仿真与故障场景 | 闭环、模式和退化恢复 | 模型与故障机制实际支持 | [11.7](#e7) |
| 系统控制台、SWD、HardFault | 启动、驱动和板上异常 | 匹配的设备、探针及固件 | [11.8](#e8) |

<a id="e1"></a>
### 11.1 控制台、模块状态与消息诊断

**先选正确入口。** SITL 的 `pxh>` 来自 PX4 进程；NuttX 系统控制台可以显示早期启动；QGC 的 MAVLink Console 或 `Tools/mavlink_shell.py` 依赖 MAVLink 已正常运行。系统在早期启动就失败时，MAVLink Shell 可能根本不存在。

在 PX4 Shell 先核对身份和系统状态：

```text
ver all
commander status
param show SYS_AUTOSTART
work_queue status
logger status
mavlink status
```

然后只观察当前问题涉及的模块与主题。例如分析定位输入：

```text
ekf2 status
listener vehicle_local_position -n 5
listener vehicle_imu -i 0 -n 3
uorb top
```

`-i` 选择实例，`-n` 限制样本数；`uorb top` 按回车结束。先看时间是否推进，再看字段和有效性，最后核对实际选中的设备或估计实例。最新值可读不代表发布者仍工作，多实例的 0 也不必然是系统最终采用的实例。

用 `param show 参数名` 读回修改结果，再在源码中找到参数消费者及刷新条件。参数定义、airframe 默认、保存值与当前生效值应分别核对；行为在激活时捕获的目标，不一定会随运行中参数修改立即重建。实验前保存原值，结束后恢复并验证。

连接板卡 MAVLink Shell 时，在**能访问串口且已安装 pymavlink/pyserial 的主机或容器**运行；下面设备名和波特率仅作示例，必须替换成真实 MAVLink 端口配置：

```bash
python3 Tools/mavlink_shell.py /dev/ttyACM0 -b 57600
```

串口被 QGC 占用、设备未映射到容器、用户无权限、端口未运行 MAVLink，属于不同故障。`mavlink_shell.py` 不是原始 UART 终端；它无法代替启动控制台。完整使用说明以[工具实现](../Tools/mavlink_shell.py)及[官方主题诊断说明](https://docs.px4.io/main/en/debug/sensor_uorb_topic_debugging.html)为依据。

**判断结果**：应能指出问题停在构建、启动、发布、实例选择、消费或模式门控中的哪一层。需要连续时序时转入日志，不靠反复手敲 listener 拼接不同时间的样本。

<a id="e2"></a>
### 11.2 GDB、编辑器调试与崩溃分析

基础启动流程见[阶段 3](#s3)。先调试确定输入的测试，再调试完整 SITL。断点会暂停进程，位置控制等性能问题应使用日志评价。

**条件断点与调用上下文。** 在带 Debug 符号的 RateControl 测试中，可先用普通断点确认参数存在，再设条件：

```text
break RateControl::update if dt > 0.02
run
bt
info args
info locals
info threads
thread apply all bt
```

只有测试实际产生满足条件的 dt 才会命中。没命中首先意味着条件或路径未出现，不证明调试器失效。想观察对象字段被谁修改，可在已命中成员函数时使用 `watch _period_us` 等观察点；对象必须仍存活，硬件观察点数量和表达式大小受平台限制。停止并重建对象后需要重新设置相应观察点。

优化构建可能内联函数或消除局部变量。先核对构建类型和实际编译参数，再使用 `info line`、`disassemble` 观察映射；不要把 `optimized out` 理解成变量值为零。条件表达式尽量不调用有副作用的函数。

**在容器内使用 VS Code 调试。** 保持编辑器、GDB、二进制和源码在同一个开发容器；按阶段 3 先生成 Debug 二进制与 rootfs。可将下面对象合并到已有 `.vscode/launch.json` 的 `configurations`，不要覆盖其他配置：

```json
{
  "name": "PX4 SIH Debug",
  "type": "cppdbg",
  "request": "launch",
  "program": "${workspaceFolder}/build/px4_sitl_default/bin/px4",
  "args": ["${workspaceFolder}/ROMFS/px4fmu_common"],
  "cwd": "${workspaceFolder}/build/px4_sitl_default/rootfs",
  "MIMode": "gdb",
  "miDebuggerPath": "/usr/bin/gdb",
  "miDebuggerArgs": "-nx",
  "externalConsole": false,
  "environment": [
    {"name": "PX4_SIM_MODEL", "value": "sihsim_quadx"},
    {"name": "PX4_SIMULATOR", "value": "sihsim"}
  ],
  "setupCommands": [
    {"text": "-enable-pretty-printing", "ignoreFailures": true},
    {"text": "handle SIGCONT nostop noprint nopass", "ignoreFailures": false}
  ]
}
```

这是使用 C/C++ 扩展的启动配置。首次在 `WorkItemExample::Run` 或可控启动路径设断点；程序启动后在承载进程的终端输入 PX4 命令，变量和线程在调试面板观察。调试控制台用于调试器表达式，不等同于 PX4 Shell。若终端交互受扩展环境限制，先使用阶段 3 的终端 GDB 完成相同检查。PX4 自带模板在 [launch_sitl.json.in](../platforms/posix/Debug/launch_sitl.json.in)。

**attach 与容器权限。** 启动一个自己的调试子进程和 attach 已有进程的权限条件不同。attach 前用 `ps` 确认 PID、用户及所在 PID namespace；遇到 ptrace 拒绝，再核对容器 `SYS_PTRACE` 和 seccomp 设置。不要为了一个符号问题直接把容器改成 privileged。

**SITL 崩溃与 core。** 在发生问题的同一容器 Bash 中先检查：

```bash
ulimit -c
cat /proc/sys/kernel/core_pattern
```

需要本次进程允许生成 core 时，在该终端设置 `ulimit -c unlimited` 后启动程序；能否产生文件还受主机 core handler、资源限制和路径权限影响。系统使用集中式 handler 时按其规则获取 core，不能假设当前目录必有 `core`。

取得匹配文件后，用发生崩溃的那一份二进制分析：

```bash
gdb -nx /absolute/path/to/px4 /absolute/path/to/core
```

在 GDB 查看 `bt full`、`thread apply all bt`、`info registers`。保存源码提交、编译类型、二进制和触发输入，再尝试缩成测试。SITL core 与 MCU HardFault 是不同机制，板上异常见 [11.8](#e8)。

<a id="e3"></a>
### 11.3 日志、实时观测与开发埋点

**先定义要观察的关系。** 分析位置跟踪需要模式、轨迹、位置估计和控制输出；分析采样问题需要样本时间、处理时间、设备实例和错误计数。只记录一个最终输出，很难区分上游来源。

在 PX4 先检查 logger，再按需要开启一段地面运行记录：

```text
logger status
logger on
```

完成目标实验后执行 `logger off`，等待文件关闭，再复制分析。on/off 要求 logger 模块本身已启动，它们不等同于启动/销毁模块。常规解锁记录方式由 `SDLOG_MODE` 决定，改日志模式或 profile 后按参数元数据要求重启。详见 [logger 参数定义](../src/modules/logger/module.yaml)。

**配置主题。** `SDLOG_PROFILE` 是位掩码，bit 0 为常规集合，bit 1 为 EKF 回放集合，bit 5 为调试主题。增加某位应保留原先需要的位，不把十进制值误当 profile 序号。保存原配置并记录预期带宽。

自定义主题文件在 `PX4_STORAGEDIR/etc/logging/logger_topics.txt`；本仓库默认 SITL 对应 rootfs 下的 `etc/logging/`，NuttX 常为 SD 卡上的 `etc/logging/`，以目标存储路径配置为准。格式是：

```text
vehicle_local_position 20 0
trajectory_setpoint 20 0
vehicle_status 100 0
```

每行依次为主题、记录最小间隔毫秒、实例。**有效的自定义文件会替代按 profile 初始化的主题集合**，不是只在默认集合上添加三条。上面是格式示例，不是完整飞行日志配置。若选择自定义文件，应把所需主题全部列入；实验后恢复原文件或移除本次新建的文件，并重启核对。逻辑见 [initialize_logged_topics](../src/modules/logger/logged_topics.cpp)。

**选择输出方式。** 偶发状态变化用 `PX4_INFO/WARN` 或结构化事件；高频内部量用有限频率的调试消息或专用主题；长期接口使用具有明确语义的正式消息。不要逐控制周期打印长字符串。

例如在模块中加入以下成员，并按选定低频发布一个内部标量；这只是埋点结构示例，`measured_value` 必须换成实际要研究的变量：

```cpp
// Header includes and class member:
#include <uORB/Publication.hpp>
#include <uORB/topics/debug_key_value.h>
uORB::Publication<debug_key_value_s> _debug_pub{ORB_ID(debug_key_value)};

// Inside a rate-limited module method:
debug_key_value_s sample{};
sample.timestamp = hrt_absolute_time();
sample.key[0] = 'e';
sample.key[1] = 'r';
sample.key[2] = 'r';
sample.value = measured_value;
_debug_pub.publish(sample);
```

零初始化保留字符串终止符。重编后先用 `listener debug_key_value` 核对，再检查日志 profile 或 MAVLink 流是否包含它。uORB 发布、ULog 记录和遥测发送是三条不同条件。埋点应有退出计划：问题解决后移除临时输出，或把有长期诊断价值的部分整理为正式接口。

实时地面站曲线适合查看趋势，最终评估使用本机完整 ULog 并检查 dropout、主题频率和有效窗口。数据导出与时间对齐步骤见[日志分析](#log-analysis)。

<a id="e4"></a>
### 11.4 使用 Replay 重算估计器

回放适合比较同一段测量下参数或算法怎样影响估计。它重放已记录输入，不会因为新控制输出而重新生成真实运动，因此不能单独证明新的闭环控制性能。普通飞行日志也未必记录了足够输入。

**准备可回放日志。** 第一次在仿真研究：记录原参数，选择单 EKF 实例，启动前设置从开机记录并启用回放主题。参考实现 `SENS_IMU_MODE=1` 选择单实例输入路径；核对实际 `ekf2 status`，不要仅凭改了参数判定。`SDLOG_MODE=1` 表示从开机到上锁，`2` 表示到关闭；`SDLOG_PROFILE` 需包含 bit 1，常规集合加回放集合为 3。若已有自定义日志主题文件，先按 [11.3](#e3)核对覆盖关系。

重启后录制一段短、条件明确的输入，结束并保存原日志、配置与源码身份。检查有 `sensor_combined`、`ekf2_timestamps` 以及实际使用的辅助观测主题，检查 dropout。多个 EKF 实例的普通日志不能直接套用这一专用流程。[官方回放说明](https://docs.px4.io/main/en/debug/system_wide_replay.html)也明确了单实例要求。

**构建和准备参数。** 以下在容器内项目根目录运行，把路径替换为已保存且容器可见的日志。路径应在普通构建 rootfs 外妥善保存，避免与生成日志混淆：

```bash
cd /workspaces/PX4-Autopilot
px4_replay_log=/workspaces/PX4-Autopilot/flight-evidence/estimator-input.ulg
test -r "$px4_replay_log"
env replay="$px4_replay_log" replay_mode=ekf2 make px4_sitl_default -j2 PX4_MAKE_ARGS=-j2
```

若 `test` 或构建失败，先处理该错误，不继续。设置 replay 后，Make 使用 `build/px4_sitl_default_replay`，它是特殊回放配置的产物，源码仍只有一套。

当前仓库的 [rc.replay](../ROMFS/px4fmu_common/init.d-posix/rc.replay) 在参数文件缺失时调用旧的 `ulog_params -l` 选项；当前 pyulog 使用 `-d` 指定分隔符。先 `ulog_params --help` 核对已安装版本，再显式准备文件。下面仅适用于支持 `-d` 的版本：

```bash
ulog_params -i "$px4_replay_log" -d ' ' -o /tmp/px4-replay-initial-params.txt
rg '^EKF2' /tmp/px4-replay-initial-params.txt > build/px4_sitl_default_replay/rootfs/replay_params.txt
```

这会替换当前回放的参数覆盖文件；有自己的试验覆盖值时先另存，再生成新的基线。逐步检查上面两条命令都成功且文件有正确的“名称 空格 数值”内容。文件中的参数成为固定覆盖值，原日志相同参数的动态变化不会再照原样生效；基线实验应选参数不变的记录片段。CLI 定义见 [pyulog params](https://github.com/PX4/pyulog/blob/main/pyulog/params.py)。

**运行和比较。** 仍在同一容器终端：

```bash
env replay="$px4_replay_log" replay_mode=ekf2 make px4_sitl none -j2 PX4_MAKE_ARGS=-j2
```

确认进入 replay 启动路径、没有缺主题或解析错误，保存新日志路径。先保持算法和参数不变验证基线，再只改变一个参数或一处实现重跑。按时间、创新、状态、重置和有效性比较，不能只比较最终位置。

每次切换原日志，都应重新核对 `replay_params.txt` 与可能存在的 `replay_params_dynamic.txt`，防止沿用上个实验的覆盖。这里用 `env` 限制环境变量到单个命令，后续普通构建不受它影响。恢复录制时改过的参数。

更一般的系统回放还需要设计发布者规则，明确哪些主题由 replay 提供、哪些由新模块计算，以及是否需要 dataman 等运行数据。先完成专用 EKF 回放，再研究 [replay 模块](../src/modules/replay)，不能把任意 `.ulg` 当成可还原整机的一键快照。

<a id="e5"></a>
### 11.5 性能、调度、内存与产物分析

**把执行耗时与运行间隔分开。** 一个函数算得很快，仍可能因为排队而迟迟没执行。模块的 elapsed counter 测计算片段，interval counter 测执行间隔；样本年龄还包含上游采样与传输。先记录相同场景和运行时长的基线。

PX4 中按目标支持情况使用：

```text
work_queue status
perf
perf latency
uorb top
```

`perf` 无参数输出已有计数器，`perf latency` 查看 HRT 延迟分布；先保存结果后，才在需要重新计量窗口时用 `perf reset`。检查调用次数、平均/最大耗时、间隔分布与错误数。平均值正常但最大值异常，仍可能影响实时闭环。

在代码里，用 `perf_alloc(PC_ELAPSED, ...)` 配合 `perf_begin/perf_end` 包围要研究的片段，用 `PC_INTERVAL` 的 `perf_count` 观察调度，用 `PC_COUNT` 记录丢样/错误。所有分支应正确结束计量，析构释放计数器；直接参考 [WorkItem 示例](../src/examples/work_item/WorkItemExample.cpp)。

**NuttX 资源。** 在提供相应命令和统计的板型上，用 `top once` 查看任务与栈，`free` 查看内存，`work_queue status` 检查队列。栈高水位依赖构建启用的栈统计，未走到最坏路径时不能据此保证永不溢出。检查中断栈、任务栈、堆及 DMA/静态缓冲，它们不是同一块余量。

POSIX 上使用宿主进程/线程工具与 Sanitizer 分析资源；不要把 Linux 的 `free` 当 MCU 剩余 RAM，也不要拿 SIH lockstep 中接近零的 elapsed 统计证明 MCU 算法免费。需要真机实时性能时，在匹配板卡和真实负载上重新测量。

**固件体积。** 以已完成构建的 fmu-v6x 为路径示例，在容器执行：

```bash
arm-none-eabi-size build/px4_fmu-v6x_default/px4_fmu-v6x_default.elf
arm-none-eabi-nm -S --size-sort -C build/px4_fmu-v6x_default/px4_fmu-v6x_default.elf | tail -n 20
```

结合 map 文件判断静态段、主要符号和链接依赖。text/data/bss 是静态产物视角，动态堆和运行栈仍需板上观察。多内存区或内核/用户分离目标还需看链接脚本，不能用一个 size 总数代替所有内存区域预算。

**一次优化实验**：先定位耗时片段，提出一种改动，保持输入与场景一致，对比耗时分布、功能结果与资源；把性能改善和行为正确同时保留。减少打印或复制可能有用，但没有测量依据时，不应随意改实时队列或算法频率。

<a id="e6"></a>
### 11.6 测试、Sanitizer、覆盖率与静态检查

**先选验证层。** 算法单元测试控制输入与状态；功能测试验证多个部件组合；SITL 验证闭环和模式；硬件验证时序与设备。具体测试名见[测试速查](#test-tools)，必须确认筛选命中。每个新增断言应对应一个行为要求，例如边界方向、饱和恢复或超时退出。

一个有效开发顺序是：先用输入暴露问题 → 确认旧实现失败 → 修改 → 当前用例通过 → 相邻边界通过 → 系统场景没有退化。只断言“函数返回 true”而不检查其代表的行为，常不足以覆盖问题。

**AddressSanitizer 示例。** 先结束同一测试构建目录的其他任务。在宿主执行：

```bash
~/embedded-lab/bin/lab exec px4-main -- make tests -j2 PX4_MAKE_ARGS=-j2 PX4_CMAKE_BUILD_TYPE=AddressSanitizer TESTFILTER='^unit-rate_control_test$'
```

ASan 用于发现越界、释放后使用等问题；UBSan 研究未定义行为；TSan 研究数据竞争；MSan 依赖合适的编译器和受检测库。配置见 [sanitizers.cmake](../cmake/sanitizers.cmake)。它们改变程序开销，也不替代正常环境的性能检查；某工具不支持目标环境时应先处理工具兼容，不能直接把启动失败认定为业务缺陷。

Sanitizer 的链接标志可能保存在 CMake cache 中。退出专用实验时，在容器项目根目录只刷新测试目标配置，再重建需要的测试：

```bash
cmake --fresh -S . -B build/px4_sitl_test -G Ninja -DCONFIG=px4_sitl_test -DCMAKE_BUILD_TYPE=RelWithDebInfo -DTESTFILTER='^unit-rate_control_test$'
cmake --build build/px4_sitl_test --target unit-rate_control_test --parallel 2
ctest --test-dir build/px4_sitl_test --output-on-failure --no-tests=error -R '^unit-rate_control_test$'
```

`--fresh` 会清除该目录的 CMake cache 与配置状态；若该测试目标使用了其他自定义 CMake 选项，先记录并在重配时带回。它不要求清空整个项目，也不涉及普通 SITL 的运行日志。

**覆盖率。** 需要评估一项测试覆盖到哪些分支时，使用支持 Coverage 的工具链，将上面配置中的构建类型改为 `Coverage`，重建并运行相关用例，再用匹配编译器版本的 gcov/lcov 汇总。首次覆盖率实验先确认没有旧的 `.gcda` 混入；已有数据只清理该测试构建目录内的覆盖率计数，不清理源码或飞行记录。

已安装 lcov/genhtml 时，容器内可用：

```bash
lcov --capture --directory build/px4_sitl_test --base-directory . --gcov-tool gcov --output-file /tmp/px4-coverage.info
genhtml /tmp/px4-coverage.info --output-directory /tmp/px4-coverage-html
```

覆盖率是发现没走到的代码的线索，不证明断言有效或工况充分。这里不用 `make tests_coverage` 作为快捷命令，因为本仓库该目标会先调用全局 `make clean`。

**集成测试与检查。** 本仓库 `make tests_integration` 依赖 Gazebo Classic 和 MAVSDK 测试链；安装 Harmonic 不表示已具备这套环境。先读 [Makefile](../Makefile) 对应目标与 [test/mavsdk_tests](../test/mavsdk_tests)，选择匹配的测试场景。可把已验证的仿真应用整理为自动场景，并设置超时、判定和日志保存，避免自动化只检查进程启动。

提交代码前先检查改动范围与格式。在具备项目格式工具的容器中，针对具体文件执行：

```bash
Tools/astyle/check_code_style.sh src/examples/work_item/WorkItemExample.cpp
```

`make format` 可能修改更大范围，不能当成只读检查。Clang 静态分析需要匹配编译数据库、生成头和目标宏，先限制到改动相关源文件；静态检查报告要结合执行路径解释，不通过加 suppressions 一次性隐藏全部问题。

<a id="e7"></a>
### 11.7 仿真、故障、自动场景和多实例

**选择环境。** SIH 适合第一条闭环、模式和接口实验；Gazebo 适合场景、模型和模拟传感器；算法测试适合精确控制输入与时间。HITL 将真实飞控接入模拟输入，SIH 在板运行又是另一种组织方式，二者都不是硬件实飞验证的完整替代。

先确认板型、固件、仿真器和数据路径的实际支持。官方 [HITL 页面](https://docs.px4.io/main/en/simulation/hitl.html)将相关方案列为社区维护，并提示当前版本不保证可用；现有 Gazebo Harmonic 环境不自动等于 Gazebo Classic HITL 环境。第一次学习先完成已有支持明确的 SITL 场景，再选择专项方案。

**故障命令要追到接收者。** `failure` 将请求转换为注入命令；是否生效取决于谁处理它。用源码确认：

```bash
rg -n 'VEHICLE_CMD_INJECT_FAILURE|FAILURE_UNIT_SENSOR_GPS' src/modules/simulation src/modules/commander
```

本参考实现的 `simulator_mavlink` 处理 GNSS 故障请求，SIH 的 `sensor_gps_sim` 和 gz_bridge 不能据此假定支持同样命令。在确认由支持该请求的仿真链接收、且只连接模拟载具后，才使用下列 PX4 命令，并分别观察消息与质量是否改变：

```text
param show SYS_FAILURE_EN
param set SYS_FAILURE_EN 1
failure gps off
```

恢复请求为 `failure gps ok`，随后恢复原 `SYS_FAILURE_EN`。没有状态或测量变化时，先确认命令处理与 ACK，不能直接得出“系统容错成功”。SIH 初学实验使用[阶段 7 的 GNSS 质量退化](#s7)，其中修改卫星数与停止发布不是同一故障。

**每个故障场景都写六项**：初始状态、故障作用层、触发时间、预计状态变化、恢复方式、通过判据。按检测信息、估计、模式、目标、输出的顺序保存证据。至少先做地面实验，确认机制确实改变了目标输入，再进入空中仿真。

**自动场景**应从单机稳定用例开始：启动成功且状态就绪 → 执行行为 → 判断完成 → 保存日志 → 有界退出。加入超时和失败状态，不能以等待固定秒数代替完成条件。一次进程退出不等于任务成功。

多实例需要分别设置实例 ID、system ID、端口、模型名、DDS 命名空间与运行目录；特别要分开参数、dataman 和日志。先通过 `Tools/simulation` 中当前仿真器的启动脚本及其 `--help` 检查支持参数，再启动第二架。只改变 UDP 端口而共用其他身份，可能把指令发送给错误实例；在进行控制前逐一核对遥测身份。

模型修改也应有证据：质量、惯量、旋翼几何、传感器噪声、更新率和接口要相互一致。先跑静止与悬停基线，再加风、延迟或异常输入，说明哪些效果来自模型而非飞控改变。

<a id="e8"></a>
### 11.8 板上控制台、SWD/GDB 与 HardFault

本节在[硬件识别与无桨台架](#s9)之后使用。先确定具体板型、MCU、探针、接口电压、接线、供电和已刷固件；容器还需实际看得见串口/探针设备。不同板卡不能共用未经核对的烧录地址或调试脚本。

**启动控制台。** 根据板卡原理图和启动配置找到系统 UART、信号电平、引脚与波特率。连接匹配的 USB-UART，在上电前打开串口终端并保存启动输出。按 bootloader、RTOS、板级初始化、驱动、PX4 启动脚本逐段定位停止位置。USB 枚举成功也不表示上层模块已经启动。

能进入 NSH 后，再查看版本、任务、设备和模块。早期没有 MAVLink 时使用系统控制台；运行中需要远程模块交互时再用 MAVLink Shell。输出协议或电机驱动问题同时需要逻辑分析仪/示波器核对时序、幅值和接线，软件消息不能证明引脚电气正确。

**匹配 ELF 与调试服务器。** 已完成对应 Debug 构建、探针服务器已按 MCU 配置并监听本地 3333 端口时，下面是 GDB 接入示意：

```bash
arm-none-eabi-gdb -nx build/px4_fmu-v6x_default/px4_fmu-v6x_default.elf
```

GDB 中：

```text
target remote localhost:3333
info registers
bt
```

路径和端口都必须与实际目标一致。这一步连接和观察，未提供 `load` 烧录命令；服务器是否上电复位/暂停取决于其配置。先读 PC、栈和任务状态，保存证据后再决定复位或重刷。

本仓库 `make 对应板型 debug` 启动带辅助脚本的 GDB，但不自动替你建立探针连接。NuttX 的 [调试辅助目录](../platforms/nuttx/Debug)包含 PX4、NuttX 和 ARMv7M 工具，按架构加载后用 `help px4`、`help nuttx`、`help armv7m` 查看支持命令。不可把 ARMv7M 工具无条件用于所有架构。

J-Link 相关目标由 [jlink.cmake](../platforms/nuttx/cmake/jlink.cmake)按工具可用性生成：`jlink_gdb_backtrace` 用于连接取栈，`jlink_upload` 写入固件，**`jlink_debug_gdb` 包含刷写再调试**。使用前查看目标内容，不把名字含 debug 的目标都当成只读附加。

**HardFault。** 目标启用相应支持、重启后保留了未提交记录时，PX4 Shell 先检查：

```text
hardfault_log check
```

确认存在后，用 `hardfault_log commit` 写入故障文件并重新准备记录机制；保存生成的 SD 卡文件、启动输出和完全匹配的 ELF。`rearm` 会丢弃未提交记录，`fault` 会主动制造崩溃，不是初始排错步骤。

对记录中的 PC/LR 等地址，用对应 ELF 解析；以下地址只是格式占位，替换成实际故障地址：

```bash
arm-none-eabi-addr2line -e build/px4_fmu-v6x_default/px4_fmu-v6x_default.elf -f -C 0x08000000
```

地址映射可能受优化、内联及内存损坏影响，应结合调用栈、任务、寄存器和触发条件。最终把问题缩到可复现输入；若能在 POSIX 测试复现内存问题，再用 ASan 辅助定位。不能用另一版固件的 ELF 得出可信的源码行。

### 11.9 把方法组合成一次有效定位

以“自动任务高度变化异常”为例，先核对任务高程参考和实际模式，再在日志中比较导航/轨迹目标、估计高度及 reset counter；目标本身变化则研究任务与轨迹，估计跳变则研究融合和参考，目标与估计连续而输出受限则研究控制与执行器。

确定责任层后，再选择测试或 GDB 追某次计算，选择 Replay 比较估计输入，选择 perf 检查调度。不要一开始同时打开全部工具。修复后重跑原场景和相邻边界，恢复实验参数，并保留说明根因的最短证据链。

这种方法同样用于学习：先预测，再观察；先明确接口，再深入实现；先说明证据能支持什么，再提出改动。学习记录与代码研究方法见[开发指南第 12 章](DEVELOPMENT.md#d12)。

<a id="environment"></a>
## 附录 A：开发环境与版本约定

### A.1 本仓库的参考实现

正文介绍 PX4 的系统与工程方法。本仓库用一份固定实现来核对源码、测试、消息和参数，便于复现；它不限制知识体系只适用于某个版本。

| 项目 | 约定 |
|---|---|
| PX4 源码基线 | `v1.17.0`，标签提交 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` |
| 学习分支 | `learn/px4-v1.17.0`，后续包含文档和实验提交 |
| 个人远程 | `https://github.com/orbits-yiyi/px4-system.git` |
| 上游远程 | `https://github.com/PX4/PX4-Autopilot.git` |
| 宿主源码 | `/home/yiyi/Workspace/px4/PX4-Autopilot` |
| 容器源码 | `/workspaces/PX4-Autopilot`，注意 `workspaces` 有 s |
| 环境入口 | `~/embedded-lab/bin/lab` |
| 环境任务 | `px4-main`，它是任务名，不代表 Git 的 main 分支 |

学习分支有新提交后，`git describe` 带提交数或 dirty 是正常的身份信息，不能要求它永久精确等于标签。镜像标签描述工具环境，PX4 标签描述源码，两者不是同一个版本。

在宿主核对：

```bash
cd /home/yiyi/Workspace/px4/PX4-Autopilot
git status --short --branch
git describe --tags --always --dirty
git rev-parse HEAD
git submodule status
~/embedded-lab/bin/lab info px4-main
```

### A.2 首次使用开发环境

这里复用 `embedded-lab` 的 PX4 配方；它是本机环境管理项目，不是 PX4 上游工具。若换机器，应先恢复或安装该项目，不能以为克隆 PX4 就会生成 `lab`。

先在宿主检查 Docker 与环境登记：

```bash
docker version
docker buildx version
~/embedded-lab/bin/lab tasks
~/embedded-lab/bin/lab doctor px4-main
```

`docker version` 应能看到 Server；只有 Client 不足以说明能运行容器。不要为 `lab: command not found` 安装 apt 建议的同名软件，使用这里的完整路径。

任务尚不存在时，先确认源码目录，再登记一次；已有任务直接检查 `info`，不重复添加：

```bash
~/embedded-lab/bin/lab add px4-main --env px4 --source /home/yiyi/Workspace/px4/PX4-Autopilot --jobs 2 --network host
```

镜像尚未准备时：

```bash
~/embedded-lab/bin/lab image px4
```

任务已经固定到旧环境，而你有意采用新配方时，先结束该任务容器中的运行，再执行显式升级：

```bash
~/embedded-lab/bin/lab upgrade px4-main
~/embedded-lab/bin/lab doctor px4-main --runtime
~/embedded-lab/bin/lab snapshot px4-main
```

正常日常编译不需要反复升级。环境更新与 PX4 源码更新分别管理，实验期间尽量保持工具环境固定。

进入容器并核对实际工具：

```bash
~/embedded-lab/bin/lab shell px4-main
```

容器内：

```bash
pwd
id
cmake --version
ninja --version
g++ --version
ccache --version
gdb --version
```

确认 UID/GID 与挂载目录写入条件匹配。宿主编辑的源码通过 bind mount 出现在容器中，构建结果也留在同一项目的 `build/`；不需要手动复制两套源码。

### A.3 依赖与子模块

主仓库锁定依赖提交。首次恢复缺失子模块前先检查是否有自己的改动；无需要保留的子模块改动时，在宿主执行：

```bash
git submodule update --init --recursive
```

它会按当前主仓库的 gitlink 初始化依赖，可能下载较多内容。只研究一个目标时，也可按构建提示初始化所需路径。不要用 `git submodule update --remote` 把所有依赖提升到各自最新分支。

切换源码版本后，原版本的子模块目录可能变成未跟踪目录。先用 `.gitmodules`、`git ls-files --stage` 和目录内状态辨认来源，不能把它们当新功能直接 `git add .`，也不能不检查就用全仓 `git clean` 删除。

<a id="ros-environment"></a>
### A.4 ROS 与 Gazebo 的独立依赖

PX4 的构建环境、Gazebo、QGC 和 ROS 各有安装与运行要求。基础 SITL 编译成功不代表全部组件都已安装。

Gazebo 使用与当前 PX4 实现匹配的依赖组合。本地 PX4 Dockerfile 配方包含 Harmonic 及 Protobuf 等依赖；需要更新镜像时按 A.2 操作。安装依据见 [Gazebo Harmonic 官方文档](https://gazebosim.org/docs/harmonic/install_ubuntu/)，实际可用性要进入任务容器检查。

ROS 接入参考 [PX4 ROS 2 指南](https://docs.px4.io/v1.17/en/ros2/user_guide.html)。该指南主要安装示例使用 Ubuntu 22.04/Humble；不能把该安装命令直接套进 Ubuntu 24.04 的 PX4 镜像。应准备匹配的独立 ROS 环境，并明确与 PX4 的网络关系。

本参考实现使用 2.4 系列 XRCE Client，官方指南给出 Agent v2.4.3。`px4_msgs` 的 `release/1.17` 可作为消息匹配起点，但仍应核对字段、顺序、类型和版本常量，记录实际提交。首次建立消息工作区，在已加载 ROS 基础环境且安装 colcon 的终端执行：

```bash
mkdir -p ~/Workspace/px4/ros2_ws/src
cd ~/Workspace/px4/ros2_ws/src
git clone --branch release/1.17 https://github.com/PX4/px4_msgs.git
git -C px4_msgs rev-parse HEAD
```

已有目录时先检查而不是重复 clone。在能同时访问两份源码的环境，比较本次要使用的消息：

```bash
diff -u /home/yiyi/Workspace/px4/PX4-Autopilot/msg/versioned/VehicleLocalPosition.msg ~/Workspace/px4/ros2_ws/src/px4_msgs/msg/VehicleLocalPosition.msg
```

确认兼容后构建并加载：

```bash
cd ~/Workspace/px4/ros2_ws
colcon build --packages-select px4_msgs
source install/setup.bash
```

Agent 按官方指南安装在能访问 PX4 的环境，启动命令为：

```bash
MicroXRCEAgent udp4 -p 8888
```

在 PX4 执行 `uxrce_dds_client status`，核对实际地址与连接。若 Client 指向 `127.0.0.1`，Agent 必须处于相同网络命名空间可达的环境；另一个桥接容器里的 localhost 不是同一个地址。

<a id="tools"></a>
## 附录 B：工具速查与排错

<a id="simulation-tools"></a>
### B.1 SIH、Gazebo 与一次完整仿真

| 工具 | 主要用途 | 前提 |
|---|---|---|
| SIH | 研究 PX4 模块、模式和闭环、快速集成 | SITL 构建与对应机型 |
| Gazebo | 三维场景、外部物理、传感器与视觉应用 | Gazebo 库、模型、桥接；图形界面还需显示/GPU |
| QGC | 配置、规划、状态和人工操作 | 可运行的地面站与 MAVLink 连接 |
| ULog / 测试 / GDB | 历史分析、可控输入、内部程序状态 | 相应日志、测试产物和符号 |

启动单机 SIH 的宿主命令：

```bash
~/embedded-lab/bin/lab exec px4-main -- make px4_sitl sihsim_quadx -j2 PX4_MAKE_ARGS=-j2
```

仅在该仿真 PX4 提示符执行，并在每步观察结果后继续：

```text
commander check
commander status
listener vehicle_local_position
```

确认预检与位置状态满足条件后：

```text
commander takeoff
```

等待离地并进入保持，观察：

```text
commander status
listener vehicle_local_position
listener vehicle_land_detected
```

最后：

```text
commander land
```

等待落地和自动上锁，再检查 `commander status` 与 `vehicle_land_detected`。`landed=true`、Disarmed 和无失效状态分别说明不同内容。用 `shutdown` 结束 SITL，返回宿主或容器终端。

SIH 没有自动弹出的三维窗口。需要 Gazebo 时先检查任务容器中的依赖：

```bash
~/embedded-lab/bin/lab exec px4-main -- gz sim --versions
~/embedded-lab/bin/lab exec px4-main -- protoc --version
```

依赖已安装但构建仍保存旧探测结果时，在同一挂载路径重新配置，再启动无图形场景：

```bash
~/embedded-lab/bin/lab exec px4-main -- cmake -S . -B build/px4_sitl_default -DCONFIG=px4_sitl_default
~/embedded-lab/bin/lab exec px4-main -- env HEADLESS=1 make px4_sitl gz_x500 -j2 PX4_MAKE_ARGS=-j2
```

只有图形环境也已配置时才移除 `HEADLESS=1`。WSLg/X11/Wayland 的显示变量、socket 和 GPU 设备是否传入容器，要按实际环境验证；host 网络并不会自动配置图形转发。无图形仿真可以验证闭环，不能证明图形渲染链已就绪。

<a id="qgc-tools"></a>
### B.2 地面站与网络

安装 QGC 使用其[官方下载说明](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html)。先只保留一个 SITL 实例和一个地面站连接，确认收到的载具身份。

当 QGC 和当前 host 网络任务位于同一 Linux 网络命名空间时，SITL 发往本地 UDP 14550 的遥测通常可供 QGC 自动连接。实际端口在 `mavlink status` 查看，QGC 中核对 UDP 连接及 MAVLink Inspector 的更新。

QGC 若运行在 Windows、另一台机器或桥接容器中，localhost 的含义会改变。此时明确发送端地址、接收端监听端口、返回端口与防火墙，配置可达的单播目标；不能只打开广播后猜测已连通。连通后再进入任务上传和命令实验。

<a id="test-tools"></a>
### B.3 测试筛选与结果

| 研究内容 | 本参考实现的 CTest 名称 |
|---|---|
| 角速度控制 | `unit-rate_control_test` |
| 姿态控制 | `unit-AttitudeControl` |
| 位置控制 | `unit-PositionControl` |
| 分配伪逆 | `unit-ControlAllocationPseudoInverse` |
| 分配去饱和 | `functional-ControlAllocationSequentialDesaturation` |
| EKF 高度融合 | `unit-test_EKF_height_fusion` |
| 保护选择 | `functional-failsafe_test` |
| 轨迹平滑 | `unit-PositionSmoothing`、`unit-VelocitySmoothing`、`unit-HeadingSmoothing` |

以位置控制为例，首次构建和执行：

```bash
~/embedded-lab/bin/lab exec px4-main -- make tests -j2 PX4_MAKE_ARGS=-j2 TESTFILTER='^unit-PositionControl$'
```

`TESTFILTER` 筛选运行范围，不保证只编译这一项所需的全部目标。构建完成后，可列出匹配项并单独重跑：

```bash
~/embedded-lab/bin/lab exec px4-main -- ctest --test-dir build/px4_sitl_test -N -R '^unit-PositionControl$'
~/embedded-lab/bin/lab exec px4-main -- ctest --test-dir build/px4_sitl_test --output-on-failure --no-tests=error -R '^unit-PositionControl$'
```

`No tests were found` 不算通过。修改源码后，直接 ctest 不会自动替你编译，需要先重新构建对应测试。失败时保留用例名、输入和断言，区分构建失败、测试执行失败与断言失败。

### B.4 调试证据速查

| 目标 | PX4 中的典型入口 | 解读限制 |
|---|---|---|
| 模式与解锁 | `commander status`、`commander check` | 接受请求和实际动作完成不同 |
| 消息 | `listener vehicle_local_position`、`uorb top` | 样本存在、更新、新鲜和有效不同 |
| 参数 | `param show 参数名` | 当前值可能来自保存配置 |
| 调度与模块 | `work_queue status`、具体模块的 `status` | 统计含义受时钟和平台影响 |
| MAVLink/DDS | `mavlink status`、`uxrce_dds_client status` | 建立会话不等于应用逻辑正确 |
| 历史行为 | `logger status` 与 ULog | 未记录的主题不能事后还原 |

### B.5 按失败层定位环境问题

| 症状 | 首先核对 | 下一步 |
|---|---|---|
| `lab` 找不到 | 完整路径是否存在 | 使用 `~/embedded-lab/bin/lab` |
| Docker socket 权限失败 | 当前会话用户、组、Docker 服务 | 修正访问条件后重新验证 Server |
| 缺 Buildx | Docker 安装来源与 `docker buildx version` | 按同一发行源补齐插件 |
| APT 502 | 失败软件源、代理、具体下载项 | 修复/重试网络，保留构建缓存 |
| ccache 写入失败 | 容器 UID/GID、`/ccache` 挂载权限 | 先隔离缓存故障，再修权限 |
| CMake 路径不同 | 缓存的源码/构建绝对路径 | 统一挂载路径后重新生成该目标 |
| Gazebo/Protobuf NOTFOUND | 实际任务镜像是否含依赖 | 更新环境并重新配置 CMake |
| 模块 command not found | Kconfig、目标配置、CMake MAIN | 核对是否编入当前二进制 |
| ROS 有主题无样本 | 会话、消息、QoS、domain、网络 | 按接口层逐项验证 |

隔离 ccache 故障时，在宿主执行：

```bash
~/embedded-lab/bin/lab exec px4-main -- id
~/embedded-lab/bin/lab exec px4-main -- ls -ld /ccache
~/embedded-lab/bin/lab exec px4-main -- env CCACHE_DISABLE=1 make px4_sitl_default -j2 PX4_MAKE_ARGS=-j2
```

禁用缓存后成功，只说明缓存路径值得检查，不代表应永久关闭缓存或修改源码。

遇到旧 `/workspace/PX4-Autopilot` 与当前 `/workspaces/PX4-Autopilot` 冲突时，结束该目标的仿真，先取出需要的日志/参数。下面仅删除一个目标的生成目录；其中运行日志与保存参数也会消失：

```bash
cd /home/yiyi/Workspace/px4/PX4-Autopilot
rm -rf -- build/px4_sitl_default
~/embedded-lab/bin/lab exec px4-main -- make px4_sitl_default -j2 PX4_MAKE_ARGS=-j2
```

不需要因此创建第二套源码。环境错误记录应保留执行终端、命令、源码身份、环境身份、第一条错误和关键状态。

<a id="git"></a>
## 附录 C：Git 工作方式

### C.1 上游与个人仓库

本地仓库约定 `origin` 指向 PX4 上游，`personal` 指向个人学习仓库。先读实际配置：

```bash
git remote -v
git branch -vv
git status --short --branch
```

已有 `personal` 时不重复添加。只有缺失时才执行：

```bash
git remote add personal https://github.com/orbits-yiyi/px4-system.git
```

个人开发在学习分支或专题分支进行。不要为了让版本号显示成精确标签而 reset 掉文档与学习提交。

### C.2 保存经过审查的改动

先按功能范围检查差异。以下是文档修改审查完成后的示例流程，并非要求现在提交：

```bash
git diff -- README.md learn/
git add -- README.md learn/
git diff --cached --stat
git diff --cached --check
git diff --cached
```

确认暂存内容正确后，再由操作者执行 `git commit` 与 `git push personal learn/px4-v1.17.0`。源码实验只暂存对应路径；不把不相关子模块残留、构建产物、日志或私人配置一起加入。

需要撤销一个已提交实验时，先确认工作区，使用针对该实验提交的 revert 保留历史。未提交修改应先逐段检查，不能用全仓 reset/clean 代替理解差异。

### C.3 在新位置恢复学习仓库

仅在目标目录尚不存在时执行，之后按 A.2 将环境任务指向新路径：

```bash
git clone --branch learn/px4-v1.17.0 https://github.com/orbits-yiyi/px4-system.git px4-system
cd px4-system
git remote rename origin personal
git remote add origin https://github.com/PX4/PX4-Autopilot.git
git submodule update --init --recursive
```

构建目录在新环境重新生成。Git 保存源码、依赖指针和文档；Docker 镜像、私有实验数据和工具环境需要各自恢复。

<a id="offboard-example"></a>
## 附录 D：Offboard 接口实验程序

此程序配合阶段 8 使用，条件为单机 SIH、正常位置估计、已起飞进入 Hold、已设置并理解断流策略。它最多发送 120 秒的当前位置目标，供研究 MAVLink 掩码、控制信号和模式交接。

先在宿主创建临时实验目录：

```bash
mkdir -p build/learning-experiments
```

将下面代码保存为 `build/learning-experiments/hold_stream.py`：

```python
import math
import time
from pymavlink import mavutil

mav = mavutil.mavlink
link = mavutil.mavlink_connection(
    "udpin:127.0.0.1:14540",
    source_system=245,
    source_component=mav.MAV_COMP_ID_ONBOARD_COMPUTER,
)
try:
    hb = link.wait_heartbeat(timeout=10)
    if hb is None or hb.autopilot != mav.MAV_AUTOPILOT_PX4:
        raise SystemExit("Expected a PX4 heartbeat")
    if not (hb.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED):
        raise SystemExit("First establish airborne Hold in SIH")
    system_id = hb.get_srcSystem()
    component_id = hb.get_srcComponent()
    pos = link.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=5)
    if (pos is None or pos.get_srcSystem() != system_id
            or pos.get_srcComponent() != component_id
            or not all(math.isfinite(v) for v in (pos.x, pos.y, pos.z))):
        raise SystemExit("No finite position from the selected PX4")

    target = (pos.x, pos.y, pos.z)
    mask = (
        mav.POSITION_TARGET_TYPEMASK_VX_IGNORE
        | mav.POSITION_TARGET_TYPEMASK_VY_IGNORE
        | mav.POSITION_TARGET_TYPEMASK_VZ_IGNORE
        | mav.POSITION_TARGET_TYPEMASK_AX_IGNORE
        | mav.POSITION_TARGET_TYPEMASK_AY_IGNORE
        | mav.POSITION_TARGET_TYPEMASK_AZ_IGNORE
        | mav.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        | mav.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )
    start = time.monotonic()
    position_seen = start
    heartbeat_seen = start
    heartbeat_sent = 0.0
    boot_ms = pos.time_boot_ms
    print("Streaming NED target:", target, flush=True)
    print("120 s limit; request and check modes in PX4.", flush=True)

    while time.monotonic() - start < 120:
        now = time.monotonic()
        for _ in range(100):
            msg = link.recv_match(blocking=False)
            if msg is None:
                break
            if (msg.get_srcSystem() != system_id
                    or msg.get_srcComponent() != component_id):
                continue
            if msg.get_type() == "LOCAL_POSITION_NED":
                if not all(math.isfinite(v) for v in (msg.x, msg.y, msg.z)):
                    raise RuntimeError("Non-finite local position")
                position_seen = now
            elif msg.get_type() == "HEARTBEAT":
                heartbeat_seen = now

        if now - position_seen > 1.0 or now - heartbeat_seen > 3.0:
            raise RuntimeError("Telemetry stale; ending target stream")
        if now - heartbeat_sent >= 1.0:
            link.mav.heartbeat_send(
                mav.MAV_TYPE_ONBOARD_CONTROLLER, mav.MAV_AUTOPILOT_INVALID,
                0, 0, mav.MAV_STATE_ACTIVE,
            )
            heartbeat_sent = now

        link.mav.set_position_target_local_ned_send(
            (boot_ms + int((now - start) * 1000)) & 0xFFFFFFFF,
            system_id, component_id, mav.MAV_FRAME_LOCAL_NED, mask,
            *target, 0, 0, 0, 0, 0, 0, 0, 0,
        )
        time.sleep(0.05)
except KeyboardInterrupt:
    print("Target stream stopped by user")
finally:
    link.close()
```

在第二个宿主终端运行：

```bash
~/embedded-lab/bin/lab exec px4-main -- python3 build/learning-experiments/hold_stream.py
```

按阶段 8 请求 Offboard、确认实际模式，再执行正常退出或断流实验。脚本到期、异常或 Ctrl+C 都会终止目标流；正常实验结束应先确认模式交还，再停流。

位置字段参与控制，速度、加速度、航向和航向速率通过 mask 忽略。教学程序不处理坐标参考重置、完整估计有效性、真实载具识别、自动接管确认或复杂网络时钟同步，不能作为实际飞行应用直接使用。阶段 10 的应用必须把这些条件纳入自己的接口与状态机。
