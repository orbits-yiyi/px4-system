# 三、分阶段操作手册

[学习入口](README.md) · [技术实现与架构](01-understand-px4.md) · [学习路线](02-learning-path.md)

阶段编号与学习路线一致。每一节先说明实验问题，再给步骤、观察和解释。命令供你逐步执行；本文没有自动运行任何构建或飞行实验。

## 先认清命令应该输入在哪里

| 标记 | 终端 | 判断方法 |
|---|---|---|
| **WSL** | 宿主 Bash，当前仓库根目录 | 提示符通常为 `yiyi@yiyi:...$` |
| **容器** | `lab shell px4-main` 打开的 Bash | 当前源码为 `/workspaces/PX4-Autopilot` |
| **PX4** | 仿真运行后的 PX4 Shell | 提示符是 `pxh>` |
| **GDB / ROS** | 对应工具或已准备环境 | 相应章节会单独说明 |

代码块中不包含提示符。**不要在 `pxh>` 输入 Git、Docker 或 Python 命令。** WSL 中统一使用 `~/embedded-lab/bin/lab`，因为你当前没有把裸命令 `lab` 加到 PATH。需要第二个终端时，在新的 WSL 窗口先进入同一个源码目录。

[1 工程](#s1) · [2 起降](#s2) · [3 模块开发](#s3) · [4 传感器](#s4) · [5 EKF](#s5) · [6 控制与日志](#s6) · [7 模式与故障](#s7) · [8 外部接口](#s8) · [9 硬件与机型](#s9) · [10 独立开发](#s10) · [Git](#git) · [排错](#troubleshooting)

<a id="s1"></a>
## 阶段 1：确认工程，编译并理解启动

### 1.1 确认正在研究哪份代码

**WSL：**

```bash
cd /home/yiyi/Workspace/px4/PX4-Autopilot
git status --short --branch
git branch --show-current
git rev-parse 'v1.17.0^{commit}'
git describe --tags --always --dirty
git submodule status
~/embedded-lab/bin/lab info px4-main
```

核对这些含义：

- 当前分支应是你的 `learn/px4-v1.17.0`，基线标签提交应为 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`。
- 文档或代码提交之后，describe 不再恰好是 `v1.17.0` 是正常的；带 `dirty` 表示存在已跟踪文件的未提交改动。它不是编译错误。
- lab 的 `source` 应是当前宿主目录。任务叫 `px4-main`，不表示 Git 在 main 分支。
- 子模块前缀空格表示与当前 gitlink 对应；`-` 表示未初始化；`+` 表示当前 checkout 与锁定提交不一致；`U` 表示冲突。

首次拉取所需依赖时，应按当前主仓库锁定版本初始化。若已有子模块改动，先进入对应目录看 `git status`，不要直接覆盖。

```bash
git submodule update --init --recursive
```

这可能下载较多内容，也可能因网络或某个专用平台依赖失败。记录具体失败路径；不需要为一个无关板卡的依赖去升级整个主仓库。日常不执行 `git submodule update --remote`。

### 1.2 确认容器入口与编译工具

**WSL：**

```bash
~/embedded-lab/bin/lab exec px4-main -- pwd
~/embedded-lab/bin/lab exec px4-main -- id
~/embedded-lab/bin/lab exec px4-main -- cmake --version
~/embedded-lab/bin/lab exec px4-main -- ninja --version
~/embedded-lab/bin/lab exec px4-main -- g++ --version
```

工作目录应为 `/workspaces/PX4-Autopilot`。身份应与现有 `lab` 用户及宿主文件权限相容。以前 GID=123 的提示源于运行身份与容器用户/组定义不一致，不能靠重复下载源码解决。

如果出现“镜像尚未准备”，按当前配方准备镜像，再核对任务采用的镜像：

```bash
~/embedded-lab/bin/lab image px4
~/embedded-lab/bin/lab info px4-main
```

当前 lab 支持固定环境。需要明确切换到刚准备的环境时使用 `~/embedded-lab/bin/lab upgrade px4-main`，切换后再次查看 info。平常编译不需要每次构建镜像。

### 1.3 编译

**WSL：**

```bash
~/embedded-lab/bin/lab exec px4-main -- make px4_sitl_default -j2 PX4_MAKE_ARGS=-j2
```

`-j2` 限制外层 Make 并行度，`PX4_MAKE_ARGS=-j2` 明确传给内部构建工具；避免任务默认 6 核导致首次构建资源过大。增量构建显示 `no work to do` 也是正常结果。

正常完成后可确认产物：

```bash
ls -lh build/px4_sitl_default/bin/px4
```

若又出现 CMake 路径不一致，使用 [路径修复](#cmake-path)。不要每次构建前都删除 build；这会丢失增量编译收益和该目录中的运行状态。

### 1.4 启动 SIH，确认它使用哪套配置

**WSL：**

```bash
~/embedded-lab/bin/lab exec px4-main -- make px4_sitl sihsim_quadx -j2 PX4_MAKE_ARGS=-j2
```

观察启动输出中的 `SYS_AUTOSTART=10040`、`SIH simulator`、参数文件、MAVLink 端口和日志路径。出现 `pxh>` 后，输入：

**PX4：**

```text
ver all
param show SYS_AUTOSTART
commander status
work_queue status
```

在另一个 **WSL** 终端阅读启动配置：

```bash
rg -n 'param|sensors start|commander start|ekf2 start|rc.vehicle_setup|navigator start' ROMFS/px4fmu_common/init.d-posix/rcS
cat ROMFS/px4fmu_common/init.d-posix/airframes/10040_sihsim_quadx
cat ROMFS/px4fmu_common/init.d/rc.mc_apps
```

把日志中的启动顺序与脚本连接起来。完成本节时，应该能指出“选了哪个模型”“启动了哪些控制器”“哪些代码只是编入而未启动”。结束仿真可在 PX4 输入 `shutdown`；后面的阶段 2 则可以沿用当前运行。

<a id="s2"></a>
## 阶段 2：用一次起降认识完整闭环

本节只在 `sihsim_quadx` 中执行。先准备一个能保存四个时段输出的普通文本窗口；不需要额外记录工具。

### 2.1 在地面取得基线

**PX4：**

```text
commander check
commander status
listener vehicle_status
listener vehicle_land_detected
listener vehicle_local_position
listener vehicle_attitude
param show MIS_TAKEOFF_ALT
logger status
```

需要记录的最少字段是：

| 主题 | 字段 | 目的 |
|---|---|---|
| vehicle_status / commander | arming、nav_state、failsafe | 区分权限、模式和保护 |
| vehicle_land_detected | landed、ground_contact、at_rest | 识别地面状态 |
| vehicle_local_position | timestamp、z、vz、z_valid、z_reset_counter、ref_timestamp、dist_bottom | 计算相对上升，判断参考连续性 |
| 参数 | MIS_TAKEOFF_ALT | 解释本次默认起飞目标 |

如果预检失败，先读具体失败信息并检查对应消息。此实验不使用强制解锁，也不把关闭检查作为默认修复。

### 2.2 起飞并观察目标与反馈

**PX4：**

```text
commander takeoff
```

观察解锁、离地及后续模式。起飞过程中和稳定后分别查看：

```text
commander status
listener trajectory_setpoint
listener vehicle_local_position_setpoint
listener vehicle_local_position
listener vehicle_attitude_setpoint
listener vehicle_rates_setpoint
listener actuator_motors
```

这些快照来自不同时间，适合理解字段，不适合精确计算同一时刻的跟踪误差。连续数值可用：

```text
listener vehicle_local_position -n 10 -r 2
```

它最多按订阅速率观察 10 条，可能占用当前 Shell 数秒；频率还受发布者限制。需要分析精确时间关系时使用阶段 6 的 ULog。

### 2.3 降落并验证状态

**PX4：**

```text
commander land
```

等待落地和自动上锁，然后：

```text
commander status
listener vehicle_land_detected
listener vehicle_local_position
logger status
```

完整结束条件是 Disarmed、落地状态成立且运动已停止，不能只以 `commander land` 命令返回为准。把启动/关闭时报告的 ULog 路径记下，再 `shutdown`。

### 2.4 用三个计算/判断解释结果

1. 同一参考下，相对上升量为 `z_ground - z_air`。先比 reset 计数和参考时间，再做减法。
2. 比较该上升量与 dist_bottom：它们参考和估计来源不同，解释接近或不同的原因。
3. 把三个状态放在一行：`Armed/Disarmed + nav_state + landed`。说明为什么落地后仍可能显示 Hold。

这三步能把“运行了一次仿真”变成“理解了一次飞行”。

### 2.5 找到起飞高度在代码中的旅程

**WSL：**

```bash
rg -n 'MIS_TAKEOFF_ALT|Using default takeoff altitude' src/modules/navigator
rg -n 'position_setpoint_triplet|trajectory_setpoint' src/modules/navigator src/modules/flight_mode_manager
rg -n 'PublishLocalPosition|lpos.z =|z_reset_counter' src/modules/ekf2/EKF2.cpp
rg -n 'trajectory_setpoint|setInputSetpoint' src/modules/mc_pos_control
```

打开匹配位置前后完整函数，回答四件事：参数何时读取、目标怎样从全局/导航表示转为局部轨迹、实际高度由谁发布、哪个控制器比较目标与状态。搜索结果只是入口，不能把同名字符串匹配当成调用关系。

<a id="s3"></a>
## 阶段 3：把现有示例改成状态观察模块

### 3.1 先理解不改代码时的行为

阅读 [WorkItemExample.hpp](../src/examples/work_item/WorkItemExample.hpp)、[WorkItemExample.cpp](../src/examples/work_item/WorkItemExample.cpp)、[CMakeLists.txt](../src/examples/work_item/CMakeLists.txt) 和 [Kconfig](../src/examples/work_item/Kconfig)。

默认 SITL 配置已有 `CONFIG_EXAMPLES_WORK_ITEM=y`。编译并启动 SIH 后：

**PX4：**

```text
work_item_example start
work_item_example status
listener orb_test
work_item_example stop
```

原示例由 sensor_accel 更新调度，发布 `orb_test.val=314159`，并观察解锁变化。它运行在工作队列中。`stop` 后该主题保留的旧样本可能仍可读，检查 timestamp 年龄，不能因此断言模块还在发布。

不要把 `px4_simple_app` 当作这次只读观察实验的替代命令：该旧教学例程会发布示意用的 vehicle_attitude，不适合与正常飞控估计链混跑。

### 3.2 修改头文件：增加输入和跨轮状态

结束 SIH 后编辑 `src/examples/work_item/WorkItemExample.hpp`，在头文件引用处加入：

```cpp
#include <uORB/topics/vehicle_local_position.h>
```

在 `private:` 内增加以下成员：

```cpp
uORB::Subscription _lpos_sub{ORB_ID(vehicle_local_position)};
vehicle_local_position_s _lpos{};
hrt_abstime _last_report{0};
```

用下面的块替换原有 `DEFINE_PARAMETERS(...)`，本实验不再使用原来的 SYS_AUTOSTART/SYS_AUTOCONFIG 参数成员：

```cpp
DEFINE_PARAMETERS(
    (ParamInt<px4::params::LEARN_MON_HZ>) _param_learn_mon_hz
)
```

原来 `_sensor_accel_sub`、`_vehicle_status_sub`、`_orb_test_pub` 和 `_armed` 会在新的 Run 中不再使用，可连同专用 include 删除；保留参数更新订阅与两个 perf 计数器。下面的方法不再发布任何控制主题。

### 3.3 增加参数定义和构建注册

新建 `src/examples/work_item/learn_mon_params.c`，内容为：

```c
/**
 * Learning observer report rate
 *
 * @unit Hz
 * @min 1
 * @max 10
 * @group Learning
 */
PARAM_DEFINE_INT32(LEARN_MON_HZ, 1);
```

编辑同目录 CMakeLists.txt，在现有 `SRCS` 列表中加入 `learn_mon_params.c`。参数定义需要进入构建，才能生成 `px4::params::LEARN_MON_HZ`；仅在头文件里写参数名是不够的。

### 3.4 修改调度和 Run

在 `WorkItemExample.cpp` 中，把整个 `init()` 替换为：

```cpp
bool WorkItemExample::init()
{
    ScheduleOnInterval(100_ms);
    return true;
}
```

这使观察器每 100 ms 检查一次数据。报告频率独立受参数控制。采用定时调度的理由是：即使传感器停止发布，观察器仍会运行，从而有机会识别“没有新数据”。

把整个 `Run()` 替换为以下代码；其余构造、析构、task_spawn 和命令入口沿用上游实现：

```cpp
void WorkItemExample::Run()
{
    if (should_exit()) {
        ScheduleClear();
        exit_and_cleanup();
        return;
    }

    perf_begin(_loop_perf);
    perf_count(_loop_interval_perf);

    if (_parameter_update_sub.updated()) {
        parameter_update_s update{};
        _parameter_update_sub.copy(&update);
        updateParams();
    }

    _lpos_sub.update(&_lpos);
    const hrt_abstime now = hrt_absolute_time();
    const int configured_hz = _param_learn_mon_hz.get();
    const int hz = configured_hz < 1 ? 1 : (configured_hz > 10 ? 10 : configured_hz);
    const hrt_abstime report_interval = 1000000ULL / hz;

    if (_last_report == 0 || now - _last_report >= report_interval) {
        _last_report = now;

        if (_lpos.timestamp == 0) {
            PX4_INFO("no position sample");

        } else if (_lpos.timestamp > now) {
            PX4_WARN("position timestamp is in the future");

        } else if (now - _lpos.timestamp > 500_ms) {
            PX4_WARN("position stale: age=%.3f s",
                     (double)(now - _lpos.timestamp) / 1e6);

        } else if (!_lpos.z_valid || !PX4_ISFINITE(_lpos.z)) {
            PX4_WARN("vertical position invalid");

        } else {
            PX4_INFO("z=%.3f m, vz=%.3f m/s, vz_valid=%d, z_reset=%u",
                     (double)_lpos.z, (double)_lpos.vz,
                     (int)_lpos.v_z_valid, (unsigned)_lpos.z_reset_counter);
        }
    }

    perf_end(_loop_perf);
}
```

这里有意使用已保存的最新消息：update 没有新数据时，`_lpos` 保留旧值，再通过时间判断是否过期。500 ms 是这个低频**教学观察器**的阈值，不能直接套用于角速度控制等实时算法。输出中的 vz 仅供观察，必须结合 `v_z_valid` 和有限性判断后才能用于计算。

### 3.5 编译、运行、改变参数、恢复

**WSL：**

```bash
git diff -- src/examples/work_item
~/embedded-lab/bin/lab exec px4-main -- make px4_sitl_default -j2 PX4_MAKE_ARGS=-j2
~/embedded-lab/bin/lab exec px4-main -- make px4_sitl sihsim_quadx -j2 PX4_MAKE_ARGS=-j2
```

**PX4：**

```text
param show LEARN_MON_HZ
work_item_example start
work_item_example status
```

先记录原值；首次实验默认是 1。然后：

```text
param set LEARN_MON_HZ 2
```

报告应从约每秒一条变成约每秒两条，参数更新订阅有自己的节流，变化可能稍后才出现。停止、重启，确认不会重复启动两个对象：

```text
work_item_example stop
work_item_example start
work_item_example status
work_item_example stop
```

如果实验前原值为 1，恢复：

```text
param set LEARN_MON_HZ 1
param show LEARN_MON_HZ
```

若原值不同则恢复实际原值。本阶段只证明正常发布、参数更新和生命周期；stale/无效/重置边界留到阶段 10 用可控输入验证。不要为触发观察器的告警而在空中随意停掉 EKF。

### 3.6 把这次修改解释清楚

应能说明：Kconfig 与板级配置控制是否编入，CMake 选择源文件，入口支持 start/stop，定时器调度 Run，订阅取得消息，时间/有效性检查决定如何解释，参数更新改变报告频率。缺任何一环，现象都会不同。

暂存源码时仅加入本次修改的具体目录。提交方法见 [Git](#git)。这些代码片段是本教材设计的练习，需在你的实际环境编译运行后记录结果。

<a id="s4"></a>
## 阶段 4：追踪传感器数据

### 4.1 在 SIH 中观察三个处理层

启动 SIH，保持未解锁，在 **PX4** 输入：

```text
sensors status
listener sensor_gyro
listener sensor_accel
listener vehicle_imu
listener sensor_combined
listener vehicle_angular_velocity
listener sensor_selection
```

先看时间、device_id、实例，再看数值。水平静止时陀螺仪应接近零，加速度计模长应接近 g；真实数值受噪声、姿态和处理阶段影响。不要期待所有字段全为零。

多实例主题指定实例：

```text
listener vehicle_imu -i 0
```

如果该实例未发布，先用 `listener vehicle_imu` 和 `uorb top` 确认当前有哪些实例。不要把“没有实例 1”直接解释为设备坏了。

### 4.2 手算一个积分量

从 `vehicle_imu` 取同一条消息的 `delta_angle[i]` 与 `delta_angle_dt`：

```text
平均角速度 = delta_angle[i] / (delta_angle_dt × 10⁻⁶)
```

若示例值为 0.004 rad 和 4000 µs，平均角速度就是 1 rad/s。用自己的实际消息计算，再与 vehicle_angular_velocity 对比数量级。两者的窗口、校准、滤波和采样时刻不同，不能用快照不相等认定出错。

### 4.3 依次阅读五个源码点

**WSL：**

```bash
rg -n 'RunImpl|STATE::|FIFORead|timestamp_sample|set_scale' src/drivers/imu/bosch/bmi088/BMI088_Gyroscope.cpp
rg -n 'updateFIFO|sensor_gyro|timestamp_sample' src/lib/drivers/gyroscope/PX4Gyroscope.cpp
rg -n 'Correct|delta_angle|delta_velocity|_vehicle_imu_pub' src/modules/sensors/vehicle_imu/VehicleIMU.cpp
rg -n 'bias|filter|_vehicle_angular_velocity_pub' src/modules/sensors/vehicle_angular_velocity/VehicleAngularVelocity.cpp
rg -n '_vehicle_imu_sub|_sensor_combined_sub|setIMUData' src/modules/ekf2/EKF2.cpp
```

完成一张自己的数据流图：设备寄存器 → 原始物理量 → 校准/积分或滤波 → 算法消费者。标出每一层单位和时间含义。SITL 读不到真实 BMI088 的 SPI 事务，本节驱动部分是源码研究，硬件行为留到阶段 9。

<a id="s5"></a>
## 阶段 5：用可控输入学习 EKF2

### 5.1 先复算一个更新，再定位代码条件

复算架构篇 7.2 的标量例子，结果应为 `h⁺=2.1 m`、`P⁺=0.1875 m²`。然后把 R 从 0.75 增大到 2.25，计算增益如何变小。说明这代表降低观测信任程度，而非把观测改得更正确。

**WSL：**

```bash
rg -n 'UpdateGpsSample|setGpsData|PublishLocalPosition' src/modules/ekf2/EKF2.cpp
rg -n 'setGpsData|gps_delay|time_us' src/modules/ekf2/EKF/estimator_interface.cpp
rg -n 'starting_conditions|continuing_conditions|innovation|reset|stopGpsHgtFusion' src/modules/ekf2/EKF/aid_sources/gnss/gnss_height_control.cpp
```

把一条 GNSS 高度观测要通过的条件列出来：配置启用、数值/方差有效、原点条件、质量检查、时间新鲜度以及当前融合状态。区分没有样本与有样本但被拒绝。

### 5.2 运行现有高度融合测试

停止仿真，在 **WSL** 执行：

```bash
~/embedded-lab/bin/lab exec px4-main -- make tests -j2 PX4_MAKE_ARGS=-j2 TESTFILTER='^unit-test_EKF_height_fusion$'
```

它使用 `build/px4_sitl_test`。首次构建可能准备测试依赖并编译许多测试二进制；TESTFILTER 主要筛选运行项，不保证只编译这一项。

生成后查看并明确运行匹配项：

```bash
~/embedded-lab/bin/lab exec px4-main -- ctest --test-dir build/px4_sitl_test -N -R '^unit-test_EKF_height_fusion$'
~/embedded-lab/bin/lab exec px4-main -- ctest --test-dir build/px4_sitl_test --output-on-failure --no-tests=error -R '^unit-test_EKF_height_fusion$'
```

需要看到实际测试数量和退出结果。构建阶段失败、测试程序崩溃、断言失败是不同问题，先保留最早的有效错误信息，不只看最后一行 Make Error。

### 5.3 读懂 baroRef 用例在证明什么

打开 [test_EKF_height_fusion.cpp](../src/modules/ekf2/test/test_EKF_height_fusion.cpp)，按 `SetUp` → `baroRef` 顺序阅读：

- 初始怎样建立 EKF、GNSS、测距和视觉输入。
- 怎样选择气压为高度参考，再启用其他高度辅助。
- 气压观测增加 4 m 后，为什么局部 z 的目标结果约为 -4 m。
- 其他高度源为什么需要估计偏置，断言的容差说明什么。
- 后续巨大跳变怎样与正常缓慢变化区分处理。

这里的 `runSeconds(60)` 是测试推进传感器模拟时间，不等于 Shell 要盲等一分钟。把数学更新与状态机分支分别找出来，才能解释测试通过的含义。

### 5.4 回到运行系统观察

重新启动 SIH，**PX4：**

```text
ekf2 status
listener estimator_status_flags
listener estimator_innovation_test_ratios
listener vehicle_local_position
```

某个主题未发布时，先查当前 EKF 配置和可用主题，不要把“不存在的主题名称”当成 EKF 故障。当前运行样本只用于认识字段，阶段 7 再引入可控的输入质量变化。

<a id="s6"></a>
## 阶段 6：控制算法、起飞对照与日志

### 6.1 先运行库测试

**WSL：**

```bash
~/embedded-lab/bin/lab exec px4-main -- make tests -j2 PX4_MAKE_ARGS=-j2 TESTFILTER='^(unit-rate_control_test|unit-AttitudeControl|unit-PositionControl|unit-ControlAllocationPseudoInverse|functional-ControlAllocationSequentialDesaturation)$'
```

查看具体测试名称，确认至少匹配了上面的五项：

```bash
~/embedded-lab/bin/lab exec px4-main -- ctest --test-dir build/px4_sitl_test -N -R 'rate_control_test|AttitudeControl|PositionControl|ControlAllocation'
```

阅读 [rate_control_test.cpp](../src/lib/rate_control/rate_control_test.cpp)、[PositionControlTest.cpp](../src/modules/mc_pos_control/PositionControl/PositionControlTest.cpp) 中一个有清晰输入/预期的测试。把一次更新的 P、I、D、前馈手算出来；特别留意积分更新发生在本轮输出之前还是之后。

然后阅读 `_inputValid()`：为什么位置、速度、加速度目标的 NaN 组合影响哪个控制环参与；为什么 x/y 需要成对。这个问题会直接影响阶段 8 的外部控制。

### 6.2 做两次只有起飞高度不同的实验

每次使用新的 SIH 进程，并先确认在地面、预检通过。日志可能在第一次上锁后关闭，因此两次实验分别启动更便于分析。

**实验 A，PX4：**

```text
param show MIS_TAKEOFF_ALT
param set MIS_TAKEOFF_ALT 2.5
commander check
listener vehicle_local_position
logger status
commander takeoff
```

第一次 show 的值是最终应恢复的原值。空中稳定后保持约 10 秒，期间读取目标和位置，然后 `commander land`，等 Disarmed 后 `shutdown`。保存本次日志路径。

**实验 B** 重新启动同一 SIH 目标，执行同样步骤，但起飞前设置：

```text
param set MIS_TAKEOFF_ALT 4.0
```

仍需预检、地面基线、稳定保持、降落和上锁。结束前把 MIS_TAKEOFF_ALT 恢复为实验前记录值，再 show 确认。

你应预测：改变最先影响导航起飞目标和下游轨迹，实际位置随控制响应变化；控制器增益并未因此改变。两次上升高度应有明显差别，但不能要求某张快照的 `-z` 恰好等于参数值。

### 6.3 找到日志并查看内容

**WSL：**

```bash
rg --files --hidden --no-ignore build/px4_sitl_default/rootfs/log -g '*.ulg'
~/embedded-lab/bin/lab exec px4-main -- python3 -c 'import pyulog, numpy; print("ULog analysis dependencies OK")'
```

如果实际日志目录不同，以 logger 输出为准。可以进入容器使用已有的 pyulog 工具：

```bash
~/embedded-lab/bin/lab shell px4-main
```

**容器：**

```bash
ulog_info --help
ulog2csv --help
```

工具不存在时先核对实际镜像与 [requirements.txt](../Tools/setup/requirements.txt)，其中列有 pyulog。不要为了分析日志，直接修改宿主系统 Python 包。

### 6.4 一个最小但可审查的分析程序

在 **WSL** 创建本地实验目录：

```bash
mkdir -p build/learning-experiments
```

把以下程序保存为 `build/learning-experiments/analyze_height.py`。它按时间戳做“最近的过去目标”对齐，超过 0.5 秒没有目标就不计算误差；保留 z/reset/有效性信息，输出 CSV。时间零点是本日志首个局部位置样本。

```python
import argparse
import csv
from pathlib import Path
import numpy as np
from pyulog import ULog

parser = argparse.ArgumentParser()
parser.add_argument("log")
parser.add_argument("--from-s", type=float)
parser.add_argument("--to-s", type=float)
args = parser.parse_args()
if (args.from_s is None) != (args.to_s is None):
    parser.error("--from-s and --to-s must be used together")
if args.from_s is not None and args.from_s >= args.to_s:
    parser.error("the time interval must have positive length")

ulog = ULog(args.log)
def dataset(name):
    matches = [d for d in ulog.data_list if d.name == name and d.multi_id == 0]
    if not matches:
        raise SystemExit("Missing logged topic: " + name)
    return matches[0].data

pos = dataset("vehicle_local_position")
sp = dataset("trajectory_setpoint")
ts = np.asarray(pos["timestamp"], dtype=np.int64)
sp_ts = np.asarray(sp["timestamp"], dtype=np.int64)
if not len(ts) or not len(sp_ts):
    raise SystemExit("No samples")
if np.any(np.diff(ts) <= 0) or np.any(np.diff(sp_ts) < 0):
    raise SystemExit("Non-monotonic timestamps: inspect the log first")

t = (ts - ts[0]) * 1e-6
index = np.searchsorted(sp_ts, ts, side="right") - 1
safe_index = np.maximum(index, 0)
age = (ts - sp_ts[safe_index]) * 1e-6
target = np.asarray(sp["position[2]"], dtype=float)[safe_index].copy()
target[(index < 0) | (age < 0) | (age > 0.5)] = np.nan
z = np.asarray(pos["z"], dtype=float)
valid = np.asarray(pos["z_valid"], dtype=bool)
reset = np.asarray(pos["z_reset_counter"])
mask = valid & np.isfinite(z) & np.isfinite(target)
err = np.where(mask, z - target, np.nan)

out = Path(args.log).with_suffix(".height.csv")
with out.open("w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["time_s", "z_m", "target_z_m", "error_z_m",
                     "z_valid", "z_reset_counter"])
    writer.writerows(zip(t, z, target, err, valid.astype(int), reset))
print("CSV:", out)
print("Duration:", round(float(t[-1]), 2), "s")
print("Logged dropouts:", len(ulog.dropouts))
print("Reset changes:", int(np.count_nonzero(np.diff(reset.astype(int)))))

if args.from_s is not None:
    window = (t >= args.from_s) & (t <= args.to_s)
    good = window & mask
    if np.count_nonzero(good) < 2:
        raise SystemExit("Too few valid aligned samples in the chosen interval")
    if len(np.unique(reset[window])) != 1:
        raise SystemExit("The chosen interval crosses a position reset")
    if np.any(np.diff(ts[good]) > 500000):
        raise SystemExit("The chosen interval contains a large sample gap")
    e = err[good]
    print("Matched samples:", len(e), "/", int(np.count_nonzero(window)))
    print("z tracking RMSE:", float(np.sqrt(np.mean(e * e))), "m")
    print("z tracking max abs error:", float(np.max(np.abs(e))), "m")
```

第一次运行不指定时间窗口，先生成 CSV。**WSL** 中用本次实际日志路径替换下例的 `你的日志文件.ulg`：

```bash
~/embedded-lab/bin/lab exec px4-main -- python3 build/learning-experiments/analyze_height.py build/px4_sitl_default/rootfs/log/你的日志文件.ulg
```

根据 CSV、事件和模式确定稳定悬停时间段，再加 `--from-s` 与 `--to-s` 重算。窗口值必须来自本次日志，不能照抄另一次运行的秒数。

这个误差指标是“局部位置估计跟踪轨迹目标”的数值，不是独立地面真值的绝对精度。z 为 NED 坐标，误差为正意味着估计位置在目标下方。只有窗口内模式、参考、数据与目标均可比，A/B 结果才有解释价值。

CSV 可以画成目标 z、实际 z 和误差三条曲线；注意向下为正。先看整个动作，再看稳定段，不能只挑一个误差最小的片段证明控制很好。

### 6.5 如果响应与预测不同

| 现象 | 下一步检查 |
|---|---|
| 改参数后起飞目标没变 | 参数实际值、读取时机、本次是否真是新的起飞 |
| 目标改变，轨迹缓慢变化 | FlightTask 的速度/加速度/jerk 限制 |
| 轨迹合理，姿态目标达到限制 | 倾角、推力、速度环约束 |
| 姿态目标合理，实际角速度跟不上 | 速率环误差、分配饱和和执行器 |
| 位置突然跳变但无相应运动 | reset、融合源和参考变更 |

沿最先出现偏差的位置深入，避免一次同时修改高度、PID、滤波和机型参数。

<a id="s7"></a>
## 阶段 7：观察健康、模式与故障传播

### 7.1 正常状态下先认识模式关系

完成一次普通起降，分别查看：

**PX4：**

```text
commander status
listener vehicle_control_mode
listener failsafe_flags
listener vehicle_land_detected
```

将位置、速度、姿态和角速度控制启用标志与当前模式对应。切换请求与实际模式分开记录；飞行状态不能只读一个 nav_state 数字，常量定义在 VehicleStatus 消息中。

### 7.2 在地面降低模拟 GNSS 质量

重新启动 SIH，保持 **Disarmed、landed=true**。先记录原值和基线：

```text
param show SIM_GPS_USED
listener vehicle_gps_position
listener estimator_status_flags
listener vehicle_local_position
commander check
```

确认本次使用 `sensor_gps_sim` 后，设置：

```text
param set SIM_GPS_USED 0
```

分别在变化刚发生、数秒后、十几秒后查看：

```text
listener vehicle_gps_position
listener estimator_status_flags
listener vehicle_local_position
listener failsafe_flags
commander check
```

这项参数在本标签 [SensorGpsSim.cpp](../src/modules/simulation/sensor_gps_sim/SensorGpsSim.cpp) 中决定卫星数及 fix/质量字段。这里模拟的是测量质量变差，消息本身仍可能继续发布，因此它与“停掉发布者”不同。

**不要预设马上出现某个保护模式。** 地面未解锁、其他观测可用、超时未到或局部估计仍暂时有效，都可能使结果不同。记录最先改变的字段，再顺着 GNSS 检查与融合条件解释。

恢复实验前的卫星数，例如原值为 10 时：

```text
param set SIM_GPS_USED 10
param show SIM_GPS_USED
```

等待质量与融合恢复，再 `commander check`。若原值不是 10，应使用原值。恢复后没有立即通过也可能有质量持续时间要求，去相应检查实现确认。

### 7.3 用功能测试研究实际保护策略

退出仿真后，**WSL：**

```bash
~/embedded-lab/bin/lab exec px4-main -- make tests -j2 PX4_MAKE_ARGS=-j2 TESTFILTER='^functional-failsafe_test$'
rg -n 'TEST_F|offboard|position|battery|rtl' src/modules/commander/failsafe/failsafe_test.cpp
```

选择一项测试阅读输入标志、当前意图、时间推进和预期 Action。再打开 [failsafe.cpp](../src/modules/commander/failsafe/failsafe.cpp) 找到相应判断。地面质量实验说明故障如何传入，测试说明特定系统状态下策略如何决定；二者合起来才是完整理解。

<a id="s8"></a>
## 阶段 8：MAVLink、Offboard、DDS 与 Gazebo

### 8.1 先把内部位置读到外部程序

只启动一个 SIH 实例。**PX4：**

```text
mavlink status
```

确认 Onboard 实例的远端 UDP 端口。以下按你此前的 14540 编写；若实际不同，需要同时修改程序监听端口。该端口只让本实验客户端监听，避免与另一个 MAVSDK/ROS 桥占用冲突。

在第二个 **WSL** 终端：

```bash
~/embedded-lab/bin/lab exec px4-main -- python3 -c 'from pymavlink import mavutil; print("pymavlink OK")'
mkdir -p build/learning-experiments
```

把下面代码保存为 `build/learning-experiments/read_position.py`：

```python
from pymavlink import mavutil

link = mavutil.mavlink_connection("udpin:127.0.0.1:14540", source_system=245)
try:
    heartbeat = link.wait_heartbeat(timeout=10)
    if heartbeat is None:
        raise SystemExit("No heartbeat: check endpoint and mavlink status")
    print("system/component:",
          heartbeat.get_srcSystem(), heartbeat.get_srcComponent())
    for _ in range(10):
        msg = link.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=3)
        if msg is None:
            raise SystemExit("No LOCAL_POSITION_NED stream")
        print("t_ms=%d x=%.3f y=%.3f z=%.3f vz=%.3f" %
              (msg.time_boot_ms, msg.x, msg.y, msg.z, msg.vz))
finally:
    link.close()
```

**WSL：**

```bash
~/embedded-lab/bin/lab exec px4-main -- python3 build/learning-experiments/read_position.py
```

与 `listener vehicle_local_position` 对照坐标和数量级。外部遥测可能降频，不要求输出时刻逐条相同。没有 heartbeat 先查传输；有 heartbeat 没位置先查消息流与状态；数据正常但符号认知相反先查 NED。

当前 lab 任务使用 host 网络，容器与 WSL 共享 Linux 网络。另一个 bridge 容器的 127.0.0.1 指它自己；Windows 原生 QGC 与 WSL 之间也可能需要额外网络配置。不要把几种 localhost 混为一谈。

### 8.2 SIH Offboard 实验的顺序

本实验让外部程序持续发送**当前悬停位置**，切模式和起降仍由 PX4 Shell 操作。这样先把“目标流”“模式切换”“动作完成”分开观察。仅用于隔离的 SIH，不能把脚本直接用于真实载具。

第一终端启动 SIH，预检通过后按阶段 2 起飞并进入稳定 Hold。检查 `xy_valid`、`z_valid`、实际模式与位置新鲜度。

记录保护参数原值，再设置本次断流实验明确使用 Land：

**PX4：**

```text
param show COM_OF_LOSS_T
param show COM_OBL_RC_ACT
param set COM_OF_LOSS_T 1.0
param set COM_OBL_RC_ACT 4
```

COM_OBL_RC_ACT 的 4 在本版本表示 Land。其他失效条件可能有更高优先级，所以仍需看实际模式和 failsafe_flags。

### 8.3 持续发送当前位置目标

把下面代码保存为 `build/learning-experiments/hold_stream.py`。运行前应已处于 SIH 的稳定 Hold。程序在开始时捕获当前位置，以约 20 Hz 发位置目标，最多运行 120 秒；遥测停止则终止目标流，交由已设置的断流策略处理。

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
        raise SystemExit("First take off and establish Hold in SIH")
    system_id = hb.get_srcSystem()
    component_id = hb.get_srcComponent()
    pos = link.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=5)
    if pos is None or not all(math.isfinite(v) for v in (pos.x, pos.y, pos.z)):
        raise SystemExit("No finite local position")

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
    print("120 s limit. Confirm actual PX4 mode in the PX4 shell.", flush=True)

    while time.monotonic() - start < 120:
        now = time.monotonic()
        for _ in range(100):
            msg = link.recv_match(blocking=False)
            if msg is None:
                break
            if msg.get_srcSystem() != system_id:
                continue
            if msg.get_type() == "LOCAL_POSITION_NED":
                if not all(math.isfinite(v) for v in (msg.x, msg.y, msg.z)):
                    raise RuntimeError("Non-finite local position")
                position_seen = now
            elif msg.get_type() == "HEARTBEAT" and msg.get_srcComponent() == component_id:
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

**第二个 WSL 终端：**

```bash
~/embedded-lab/bin/lab exec px4-main -- python3 build/learning-experiments/hold_stream.py
```

它不自动解锁、不自动请求模式。收到当前坐标后持续发送至少约两秒，再回到第一个终端：

**PX4：**

```text
listener offboard_control_mode
commander mode offboard
commander status
listener trajectory_setpoint
listener vehicle_local_position
```

应确认实际模式为 Offboard，轨迹位置与捕获目标接近，飞机继续保持。若被拒绝，查看位置有效性、offboard_control_mode 的时间和本次系统信息，而不是重复强切。

位置目标中的速度/加速度已通过 mask 忽略；如果把 mask 改错，零速度/加速度会参与控制，实验含义会改变。脚本也不处理坐标参考重置、复杂估计失效、自动降落确认等任务，因此属于短时间接口练习，不能当正式飞行程序。

### 8.4 分别验证正常退出和断流

**第一次，正常退出：**保持脚本仍在发送，在 PX4 输入：

```text
commander mode auto:loiter
commander status
```

确认实际模式已是 Hold，再在脚本终端 Ctrl+C，最后在 PX4 正常降落。这样退出顺序是“确认控制权交回 → 停目标流”，不会把正常结束混同于断流故障。

**第二次，断流实验：**重新起飞到 Hold，启动脚本、切入 Offboard 并确认后，在脚本终端 Ctrl+C，留在 PX4 观察：

```text
commander status
listener failsafe_flags
listener vehicle_land_detected
```

预期目标信号过期后触发配置的保护动作，并最终降落、上锁；动作时刻还受检查周期及其他策略条件影响。如果实际行为不同，保存日志，按 offboard signal lost → failsafe 选择 → 实际模式追踪。需要主动结束该 SIH 实验时在 PX4 请求 `commander land` 并确认落地。

实验结束恢复 COM_OF_LOSS_T 和 COM_OBL_RC_ACT 的原值，逐项 show 确认。比较两次日志里模式转移的原因，才能证明你理解正常退出和保护退出。

### 8.5 DDS / ROS：先准备独立环境，再读一条主题

这一小节的前提是有匹配 Ubuntu 的 ROS 2 环境、colcon、px4_msgs 和兼容的 Micro XRCE-DDS Agent。PX4 开发镜像有 Client，不等于有 Agent，更不等于 ROS 已经安装。

先按 [PX4 v1.17 ROS 2 指南](https://docs.px4.io/v1.17/en/ros2/user_guide.html) 准备兼容的 ROS 基础环境，该页的主要安装示例是 Ubuntu 22.04 + ROS 2 Humble。你的 PX4 开发镜像是 Ubuntu 24.04，不能直接在其中照抄 Humble 的安装步骤；应在独立兼容环境准备这部分。此项尚未准备时，先完成本阶段的 MAVLink 实验。

消息仓库选择 [px4_msgs 的 release/1.17 分支](https://github.com/PX4/px4_msgs/tree/release/1.17) 作为候选，然后与本机 PX4 消息核对并记录实际提交；发布分支也会演进，不能仅凭分支名认定逐字匹配。

在**已经 source ROS 基础环境且具备 colcon 的终端**，首次建立消息工作区：

```bash
mkdir -p ~/Workspace/px4/ros2_ws/src
cd ~/Workspace/px4/ros2_ws/src
git clone --branch release/1.17 https://github.com/PX4/px4_msgs.git
git -C px4_msgs rev-parse HEAD
```

已有目录时跳过 clone，先检查里面的改动和提交。至少核对本次要用的消息：

```bash
diff -u /home/yiyi/Workspace/px4/PX4-Autopilot/msg/versioned/VehicleLocalPosition.msg ~/Workspace/px4/ros2_ws/src/px4_msgs/msg/VehicleLocalPosition.msg
diff -u /home/yiyi/Workspace/px4/PX4-Autopilot/msg/versioned/VehicleStatus.msg ~/Workspace/px4/ros2_ws/src/px4_msgs/msg/VehicleStatus.msg
```

注释差别不必然导致通信失败，但字段、类型、顺序、版本常量的差异必须解释清楚；必要时选择与该 PX4 提交匹配的消息提交。不要直接追 px4_msgs main。

确认后在 **ROS 终端**：

```bash
cd ~/Workspace/px4/ros2_ws
colcon build --packages-select px4_msgs
source install/setup.bash
```

Agent 与 ROS 消息包分别安装。该版官方指南使用 **Micro XRCE-DDS Agent v2.4.3**；本仓库锁定的 Client 属于 2.4 系列。若所选 ROS 开发环境还没有 Agent，在具备 Git、CMake、C/C++ 工具链的环境中首次执行：

```bash
mkdir -p ~/Workspace/px4
cd ~/Workspace/px4
git clone --branch v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cmake -S Micro-XRCE-DDS-Agent -B Micro-XRCE-DDS-Agent/build
cmake --build Micro-XRCE-DDS-Agent/build --parallel 2
sudo cmake --install Micro-XRCE-DDS-Agent/build
sudo ldconfig
```

构建可能下载 Fast DDS 等依赖，网络失败要在这一层处理。已有 Agent 时先核对来源和版本，不重复覆盖安装。另开 Agent 终端：

```bash
MicroXRCEAgent udp4 -p 8888
```

PX4 启动日志曾显示 Client 指向 127.0.0.1:8888；在 PX4 用 `uxrce_dds_client status` 检查连接。Agent 所在网络必须能被该地址访问。如果 ROS 环境放在另一个容器，应明确使用相同 Linux host 网络或配置可达地址，不与其他机器的 v3 Client 教程混用。

在已经 source 消息工作区的 **ROS 终端**：

```bash
ros2 topic list
ros2 topic info -v /fmu/out/vehicle_local_position_v1
ros2 topic echo /fmu/out/vehicle_local_position_v1 --qos-reliability best_effort
```

与 PX4 listener 对照单位、坐标和有效性。没有消息时依次检查 Client/Agent 会话、domain/namespace、主题版本、类型和 QoS。完成只读观察后，再把前面的外部控制状态流程迁移到 ROS；MAVLink 接口跑通不代表 DDS 接口自动兼容。

### 8.6 Gazebo：先验证依赖和无图形闭环

**WSL：**

```bash
~/embedded-lab/bin/lab exec px4-main -- gz sim --versions
~/embedded-lab/bin/lab exec px4-main -- protoc --version
```

如果失败，先处理实际任务镜像。你当前 `~/embedded-lab/environments/px4/Dockerfile` 已声明 Harmonic、Protobuf 等依赖，并配置了 APT 缓存与重试；配置文件存在不表示构建成功或任务已切换。按阶段 1 的 image/info/upgrade 流程核对。

你此前的 APT 502 是软件包下载经代理失败，发生在镜像构建阶段。不要用删除 PX4 build 缓存修它，也不要在宿主装一套 Gazebo 来掩盖容器缺依赖。[Harmonic 官方安装页](https://gazebosim.org/docs/harmonic/install_ubuntu/) 用于核对软件源和系统支持；[Docker 缓存说明](https://docs.docker.com/build/cache/optimize/) 解释为什么缓存挂载有助于保留下载。

依赖具备后，在 **WSL** 初始化对应模型并重新生成构建配置：

```bash
git submodule update --init --recursive Tools/simulation/gz
~/embedded-lab/bin/lab exec px4-main -- cmake -S . -B build/px4_sitl_default
~/embedded-lab/bin/lab exec px4-main -- env HEADLESS=1 make px4_sitl gz_x500 -j2 PX4_MAKE_ARGS=-j2
```

先确认模型连接、传感器更新、估计有效和预检通过，再做一次正常起降。SITL 基础构建成功不保证 Gazebo 插件、桥接和模型都已准备完成。

GUI 是另一个问题：需要 WSLg/显示服务器、DISPLAY 或 Wayland 变量、相应 socket 及图形设备挂载。已有支持 GUI 的容器环境中可去掉 HEADLESS；当前 lab 若未透传这些资源，单改命令不会让窗口出现。先在现有任务中运行 `printenv DISPLAY WAYLAND_DISPLAY` 并检查 compose 挂载，针对缺少的一项配置。

本阶段要能解释：SIH 和 Gazebo 提供不同物理/传感器环境，但 Commander、估计和控制主线仍是同一套 PX4。两边都飞起来不是终点，应比较同类日志，并说明模型差异为什么会改变响应。

<a id="s9"></a>
## 阶段 9：比较机型并进入硬件

### 9.1 从启动差异找到控制差异

**WSL：**

```bash
diff -u ROMFS/px4fmu_common/init.d/rc.mc_apps ROMFS/px4fmu_common/init.d/rc.fw_apps
rg -n 'tecs|lateral|longitudinal' src/modules/fw_lateral_longitudinal_control/FwLateralLongitudinalControl.cpp
rg -n 'TRANSITION|transition' src/modules/vtol_att_control/standard.cpp
rg -n 'wheel|steering|yaw|curvature' src/modules/rover_ackermann/AckermannPosControl/AckermannPosControl.cpp
```

先列出哪些平台机制共用：uORB、参数、日志、调度、通信、构建。再列出四旋翼特有假设在其他机型为什么不成立：可以悬停、推力主要沿机体负 z、位置目标可以在原地保持等。

本版本提供的 SIH 模型包括 airplane、standard_vtol 和 rover_ackermann，可从 [simulator_sih 的 CMake](../src/modules/simulation/simulator_sih/CMakeLists.txt) 核对。切机型前结束当前仿真并导出要保留的参数；不同 airframe 可能触发参数重新配置。

例如只启动固定翼模型观察系统：

```bash
~/embedded-lab/bin/lab exec px4-main -- make px4_sitl sihsim_airplane -j2 PX4_MAKE_ARGS=-j2
```

此时先看 SYS_AUTOSTART、模块状态和机型参数，不直接复制四旋翼起降流程。学习固定翼自动飞行应先理解起飞、空速、盘旋和着陆条件；VTOL 应先理解转换状态与失败回退。

### 9.2 用一个真实板型读懂配置

以 fmu-v6x 作为源码阅读样本：

```bash
cat boards/px4/fmu-v6x/default.px4board
cat boards/px4/fmu-v6x/init/rc.board_sensors
rg -n 'GPIO|SPI|I2C|UART|PWM' boards/px4/fmu-v6x/src/board_config.h
cat boards/px4/fmu-v6x/src/spi.cpp
cat boards/px4/fmu-v6x/src/timer_config.cpp
```

把一个传感器从“构建中启用的驱动”连到“板级总线/片选/数据就绪引脚”，再连到启动脚本中的探测。把一个输出从逻辑功能连到定时器通道和物理接口。

如果只是学习固件构建，可以编译这个样本目标：

```bash
~/embedded-lab/bin/lab exec px4-main -- make px4_fmu-v6x_default -j2 PX4_MAKE_ARGS=-j2
```

它会使用 NuttX 与交叉工具链，产物通常位于 `build/px4_fmu-v6x_default/`。是否能刷到实际设备，必须看真实板型号、硬件变体和 bootloader 识别，不能从构建成功推断。

### 9.3 真机台架按这个顺序推进

设备确定后，先拆桨或隔离动力，用对应板卡文档和 QGC 选择正确固件。WSL/Docker 不会自动获得 Windows 的 USB 设备；如果控制台看不到设备，先处理连接与透传，再研究 PX4 驱动。

| 顺序 | 做什么 | 看什么才说明本步成立 |
|---|---|---|
| 1 启动身份 | 读取固件与板卡信息 | 目标板型、版本、启动无关键错误 |
| 2 设备枚举 | 查看 sensors、设备实例和 ID | 与实际板卡设备吻合，消息持续更新 |
| 3 方向与校准 | 静置后分别缓慢转动各轴 | 角速度轴/符号正确，姿态随动作变化 |
| 4 外设通信 | 逐一连接 GNSS、遥控、数传等 | 端口/速率/协议对应，数据质量满足要求 |
| 5 输出映射 | 无桨、低输出，逐个识别执行器 | 逻辑电机、物理通道、旋向一致 |
| 6 门控 | 检查解锁、停止、保护输出行为 | 没有意外持续驱动，异常行为可解释 |
| 7 资源 | 检查 CPU、工作队列、栈、日志 | 无持续超时、栈不足、明显日志丢失 |
| 8 受控飞行 | 完成装配与飞行前检查后小范围验证 | 日志支持估计、控制、输出均正常 |

可用控制台观察命令包括 `ver all`、`sensors status`、`work_queue status`、`perf`、`logger status`；具体可用命令取决于板级配置。输出测试按实际设备和协议执行，不能给未知硬件统一套一条电机命令。

记录每次问题属于板级定义、总线、驱动、校准、估计还是输出映射。只有进入真实台架后，才有依据评价你对硬件链的掌握程度。

<a id="s10"></a>
## 阶段 10：把观察器做成可验证的完整修改

本阶段用一个具体案例练习开发闭环：在阶段 3 的观察器上增加**相对高度**，正确处理无数据、过期、无效值和参考重置。先定义策略，再写代码，最后用可控测试和 SIH 验证。

### 10.1 明确行为，避免“高度怎么算都行”

本例采用以下策略：

| 输入情况 | 输出/状态 |
|---|---|
| 从未取得样本 | NoData，不输出有效高度 |
| 时间戳在未来 | InvalidTime，不继续使用旧参考 |
| 消息年龄超过 500 ms | Stale，清除旧会话参考 |
| z 无效或非有限 | Invalid，清除旧参考 |
| 第一条有效样本 | ReferenceSet，高度为 0 |
| 同一参考下连续有效样本 | Valid，高度为 `reference_z - z` |
| z_reset_counter 或 ref_timestamp 改变 | ReferenceReset，以当前样本重建零点 |

这里主动重建参考，所以重置时高度会重新为零并明确报告状态。它不承诺跨重置保持起飞点高度。如果业务需要连续高度，应另行设计重置补偿并测试，不应悄悄改变本例含义。

### 10.2 把逻辑抽成可测类

在 `src/examples/work_item/` 新建 `HeightTracker.hpp`：

```cpp
#pragma once
#include <cmath>
#include <cstdint>
#include <limits>

class HeightTracker
{
public:
    enum class State {
        NoData, InvalidTime, Stale, Invalid,
        ReferenceSet, ReferenceReset, Valid
    };
    struct Result {
        State state;
        float height;
    };

    Result update(uint64_t now, uint64_t sample_time, float z, bool z_valid,
                  uint8_t reset_counter, uint64_t reference_time)
    {
        if (sample_time == 0) {
            return reject(State::NoData);
        }
        if (sample_time > now) {
            return reject(State::InvalidTime);
        }
        if (now - sample_time > 500000) {
            return reject(State::Stale);
        }
        if (!z_valid || !std::isfinite(z)) {
            return reject(State::Invalid);
        }
        if (!_has_reference) {
            setReference(z, reset_counter, reference_time);
            return {State::ReferenceSet, 0.f};
        }
        if (reset_counter != _reset_counter || reference_time != _reference_time) {
            setReference(z, reset_counter, reference_time);
            return {State::ReferenceReset, 0.f};
        }
        return {State::Valid, _reference_z - z};
    }

private:
    Result reject(State state)
    {
        _has_reference = false;
        return {state, std::numeric_limits<float>::quiet_NaN()};
    }
    void setReference(float z, uint8_t reset_counter, uint64_t reference_time)
    {
        _has_reference = true;
        _reference_z = z;
        _reset_counter = reset_counter;
        _reference_time = reference_time;
    }

    bool _has_reference{false};
    float _reference_z{0.f};
    uint8_t _reset_counter{0};
    uint64_t _reference_time{0};
};
```

类不访问真实时钟、不订阅 uORB；调用者显式传入时间与样本。这让边界测试可以精确推进时间，不需要暂停整个飞控。

### 10.3 先写行为测试

新建 `src/examples/work_item/HeightTrackerTest.cpp`：

```cpp
#include <gtest/gtest.h>
#include "HeightTracker.hpp"

using State = HeightTracker::State;

TEST(HeightTrackerTest, RelativeHeightUsesNedSign)
{
    HeightTracker tracker;
    EXPECT_EQ(tracker.update(1000000, 1000000, -0.5f, true, 2, 100).state,
              State::ReferenceSet);
    const auto result = tracker.update(1100000, 1100000, -2.5f, true, 2, 100);
    EXPECT_EQ(result.state, State::Valid);
    EXPECT_FLOAT_EQ(result.height, 2.f);
}

TEST(HeightTrackerTest, StaleDataCannotRemainValid)
{
    HeightTracker tracker;
    tracker.update(1000000, 1000000, -0.5f, true, 2, 100);
    EXPECT_EQ(tracker.update(1500000, 1000000, -0.5f, true, 2, 100).state,
              State::Valid);
    const auto stale = tracker.update(1500001, 1000000, -0.5f, true, 2, 100);
    EXPECT_EQ(stale.state, State::Stale);
    EXPECT_TRUE(std::isnan(stale.height));
    EXPECT_EQ(tracker.update(1600000, 1600000, -3.f, true, 2, 100).state,
              State::ReferenceSet);
}

TEST(HeightTrackerTest, ReferenceChangesAreExplicit)
{
    HeightTracker tracker;
    tracker.update(1000000, 1000000, -0.5f, true, 2, 100);
    const auto reset = tracker.update(1100000, 1100000, -10.f, true, 3, 100);
    EXPECT_EQ(reset.state, State::ReferenceReset);
    EXPECT_FLOAT_EQ(reset.height, 0.f);
    EXPECT_EQ(tracker.update(1200000, 1200000, -10.f, true, 3, 200).state,
              State::ReferenceReset);
}

TEST(HeightTrackerTest, InvalidInputDoesNotEstablishReference)
{
    HeightTracker tracker;
    EXPECT_EQ(tracker.update(1000000, 0, 0.f, true, 0, 0).state, State::NoData);
    EXPECT_EQ(tracker.update(1000000, 1000001, 0.f, true, 0, 0).state,
              State::InvalidTime);
    EXPECT_EQ(tracker.update(1000000, 1000000, 0.f, false, 0, 0).state,
              State::Invalid);
    EXPECT_EQ(tracker.update(1000000, 1000000,
              std::numeric_limits<float>::quiet_NaN(), true, 0, 0).state,
              State::Invalid);
    EXPECT_EQ(tracker.update(1100000, 1100000, -2.f, true, 0, 0).state,
              State::ReferenceSet);
}
```

这些测试关注外部可观察的行为：NED 符号、超时边界、恢复策略、重置与非法输入。它们不依赖飞行中的偶然噪声。

在同目录 CMakeLists.txt 的 `px4_add_module(...)` **之后**增加：

```cmake
px4_add_unit_gtest(SRC HeightTrackerTest.cpp)
```

不要把测试文件加入模块的 SRCS；测试程序与飞控程序是两个构建目标。

**WSL：**

```bash
~/embedded-lab/bin/lab exec px4-main -- make tests -j2 PX4_MAKE_ARGS=-j2 TESTFILTER='^unit-HeightTracker$'
~/embedded-lab/bin/lab exec px4-main -- ctest --test-dir build/px4_sitl_test --output-on-failure --no-tests=error -R '^unit-HeightTracker$'
```

若没有匹配项，先查 CMake 注册、实际使用的源码目录，以及测试配置是否包含 work_item。默认 SITL test 配置以 default 为基础，再应用测试差异；自己的定制配置可能改变模块选择。

### 10.4 接回模块，再做集成验证

在 WorkItemExample.hpp 中 include `"HeightTracker.hpp"`，并增加：

```cpp
HeightTracker _height_tracker;
```

在阶段 3 的 Run 中，读取 `_lpos` 并取得 `now` 后，每轮调用：

```cpp
const auto height = _height_tracker.update(
    now, _lpos.timestamp, _lpos.z, _lpos.z_valid,
    _lpos.z_reset_counter, _lpos.ref_timestamp);
```

保留原有报告节流，在 `if (_last_report == 0 || ...)` 块内增加：

```cpp
PX4_INFO("height_state=%d, relative_height=%.3f m",
         static_cast<int>(height.state), (double)height.height);
```

高度只在 ReferenceSet、ReferenceReset、Valid 状态具有本例定义的意义；其他状态输出 NaN。枚举数与类中的定义顺序对应，熟悉后可以改成文本状态。

ReferenceReset 是短暂事件，可能出现在两次节流报告之间。在调用 update 之后、报告节流判断之前，单独记录一次：

```cpp
if (height.state == HeightTracker::State::ReferenceReset) {
    PX4_WARN("height reference reset: relative height starts from zero");
}
```

下一轮在参考未继续变化时会进入 Valid，不会把同一次重置每轮重复报告。这样能区分“定期状态快照”与“不能漏掉的状态变化”。

重新编译并做一次 SIH 起降。观察器首次有效消息建立零点，所以其高度参考取决于你何时 start：地面启动会得到相对地面的升高，空中启动则以空中位置为零。让这两个实验结果都符合定义，再证明 stop/start 重新建立参考。

测试负责受控边界，SIH 负责真实消息、参数和调度的接入。二者都完成，才能说这项修改形成了完整结果。

### 10.5 学一次 GDB 调试

先对已有的算法测试使用 GDB，避免把高频飞控暂停带来的副作用混入学习。**WSL：**

```bash
~/embedded-lab/bin/lab exec px4-main -- make tests -j2 PX4_MAKE_ARGS=-j2 PX4_CMAKE_BUILD_TYPE=Debug TESTFILTER='^unit-rate_control_test$'
~/embedded-lab/bin/lab shell px4-main
```

**容器：**

```bash
gdb --args build/px4_sitl_test/unit-rate_control_test
```

**GDB：**

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

本版本已有 `AllZeroCase`，适合观察零输入和调用栈；要研究非零 P/I/D 行为，可另外添加有明确预期的测试。变量若被优化，应检查实际构建类型及模块的编译选项，不要把 optimized out 误解为程序没有该变量。

### 10.6 提交一项可以让别人复现的结果

在提交说明里写清：原来怎样、什么输入会暴露问题、现在怎样、为什么放在此层、运行了哪些测试、SIH 日志显示什么，以及还有哪些范围未验证。一个可复现案例、几项有意义的断言和一段可解释日志，比大量无对应证据的“学习总结”更有用。

<a id="git"></a>
## 日常 Git：保存工作、推送与恢复环境

### 当前仓库关系

| 远程 | 地址 | 用途 |
|---|---|---|
| origin | https://github.com/PX4/PX4-Autopilot.git | 当前本地仓库保留的官方来源 |
| personal | https://github.com/orbits-yiyi/px4-system.git | 你的学习与开发仓库 |

当前开发分支为 `learn/px4-v1.17.0`。查看：

```bash
git remote -v
git branch -vv
git status --short --branch
```

修改文档时：

```bash
git add -- README.md learn
git diff --cached --stat
git diff --cached
git commit -m "docs: explain a PX4 learning experiment"
git push personal learn/px4-v1.17.0
```

修改本教材的 work_item 实验时，只将本次源码路径加入暂存：

```bash
git add -- src/examples/work_item
git diff --cached
git commit -m "feat: report relative height and stale position data"
git push personal learn/px4-v1.17.0
```

实际提交信息按本次改动填写。不要用 `git add .` 顺手加入旧版本残留或无关文件；ULog 和本地生成脚本在 build 下，保留于本机。

### 网络代理与安全更新

若直连 GitHub 失败，且你现有的 127.0.0.1:7897 代理正在工作，可仅为本次推送启用：

```bash
HTTPS_PROXY=http://127.0.0.1:7897 HTTP_PROXY=http://127.0.0.1:7897 \
  git -c http.version=HTTP/1.1 push personal learn/px4-v1.17.0
```

看到远端领先时先 fetch 和比较历史，不能用 force 抹掉远端工作。更新上游也应作为一次独立升级实验；不要为了学习新功能把当前 v1.17.0 分支悄悄切到 main。

### 在新位置恢复

只在目标目录尚不存在时 clone，例如另一台机器：

```bash
git clone --branch learn/px4-v1.17.0 --recurse-submodules https://github.com/orbits-yiyi/px4-system.git PX4-Autopilot
cd PX4-Autopilot
git remote rename origin personal
git remote add origin https://github.com/PX4/PX4-Autopilot.git
git config remote.pushDefault personal
git config branch.learn/px4-v1.17.0.pushRemote personal
```

新 clone 默认 origin 指向你克隆的个人仓库，所以上面显式调整为当前教材的远程命名约定。源码恢复不等于 lab 已指向新位置；之后必须登记或调整任务，核对 `lab info` 中的 source。容器内继续统一使用 `/workspaces/PX4-Autopilot`，构建目录在新环境重新生成。

<a id="troubleshooting"></a>
## 排错：先判断哪一层失败

| 症状 | 所属层 | 第一件该做的事 |
|---|---|---|
| `lab: command not found` | 宿主命令入口 | 使用 `~/embedded-lab/bin/lab`，不用 apt 安装同名程序 |
| Docker socket permission denied | Docker 访问 | 检查当前用户/组与会话生效状态 |
| 需要 Buildx/BuildKit | 镜像构建工具 | 检查 `docker buildx version` 与安装来源 |
| APT 502、下载中断 | 镜像依赖/网络 | 找失败包、软件源和代理；利用已配置缓存重试 |
| CMake 路径不同 | 构建缓存 | 统一路径，按下节清理该目标生成物 |
| ccache Permission denied | 缓存挂载权限 | 检查实际 UID/GID 和 /ccache 的权限 |
| Gazebo/Protobuf NOTFOUND | 当前镜像依赖/配置缓存 | 确认容器内依赖，再重跑 CMake |
| pxh 中没有三维画面 | 仿真类型 | 确认当前是 SIH；用数据观察或进入 Gazebo 阶段 |
| command not found（PX4 模块） | 构建注册 | 核对 Kconfig、px4board、CMake MAIN 与重新构建 |
| 有消息但数据不动 | 数据与运行状态 | 比时间戳、发布者状态和模式，不只看旧样本数值 |
| takeoff 被拒绝 | 健康与模式 | `commander check`、实际失败项和估计有效性 |
| 测试零项 | 测试筛选/注册 | `ctest -N` 核对名字，使用 `--no-tests=error` |
| ROS topic 存在却没样本 | 外部接口 | Agent 会话、消息版本、类型、QoS、网络与 domain |

<a id="cmake-path"></a>
### CMake 路径冲突：只重建受影响的生成目录

先结束该目标的仿真。当前确定使用 `/workspaces/PX4-Autopilot`，若缓存来自旧 `/workspace/PX4-Autopilot`，需要重新生成。

下面命令会删除默认 SITL 的产物、rootfs 内日志和保存参数。只在你已经取出需要的实验数据，或明确不要这些生成数据时，在 **WSL** 执行：

```bash
cd /home/yiyi/Workspace/px4/PX4-Autopilot
rm -rf -- build/px4_sitl_default
~/embedded-lab/bin/lab exec px4-main -- make px4_sitl_default -j2 PX4_MAKE_ARGS=-j2
```

它不会删除源码，不新建第二套编译目录。不要扩大为删除整个仓库，也不要用 `git clean -ffdx` 清理你尚未理解的未跟踪子目录。

### ccache 失败：先区分权限与编译器错误

**WSL：**

```bash
~/embedded-lab/bin/lab exec px4-main -- id
~/embedded-lab/bin/lab exec px4-main -- ls -ld /ccache
~/embedded-lab/bin/lab exec px4-main -- env CCACHE_DISABLE=1 make px4_sitl_default -j2 PX4_MAKE_ARGS=-j2
```

若禁用缓存后能编译，说明应修缓存挂载权限或配置；这不能证明源码此前有问题。禁用是隔离诊断，日常应恢复可写缓存。

### 错误记录应保留什么

保留本次命令、执行终端、源码提交、镜像信息、第一条真正的错误和相关状态。Make 最后输出的 Error 1/2 往往只是上游步骤失败的总结。用这组信息回到架构图定位，下一次排查才不会重新从安装环境开始。
