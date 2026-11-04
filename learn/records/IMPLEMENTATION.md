# 课程实施、源码基线与实际验证

[学习首页](../README.md) · [课程目录](../WORKBOOK.md#course-map) · [覆盖表](#coverage) · [个人进度](../WORKBOOK.md#mastery-record)

日期：2026-11-04，Asia/Shanghai。这里记录作者本次实际完成的材料和工程验证，个人能力仍由独立任务评价。

## 最终材料结构

学习主线集中在[一本实战文档](../WORKBOOK.md)，按 20 章覆盖完整工程领域。前六章连续完成运行身份、启动/调度、内部接口、真实位置控制分支和完整模块开发；构建、GDB、日志、验收和记录模板均在同一本书内。

[SYSTEM_GUIDE](../SYSTEM_GUIDE.md)统一维护全部模块职责和接口契约；其余既有专题用于按需深入。`labs/` 仅保留可编译模块、算法和检查程序；本目录保存本轮证据。没有独立的小课目录、重复模块卡片或空白进度文件。

首页提供唯一主线，ROADMAP 保留原有锚点并指向书内相关章节。`px4-sih-debug.code-workspace` 没有恢复；既有编辑器设置改动保留，主飞行算法、板级配置与主消息目录未因教学示例而修改。

## 源码与工具身份

| 项目 | 本次实际条件 |
|---|---|
| 源码根目录 | `/home/yiyi/Workspace/px4/PX4-Autopilot` |
| 分支/提交 | `learn/px4-v1.17.0` / `310e5c30ee64784b13c94de430c463ebe4656024` |
| 本地差异 | 原有文档和编辑器改动，加本轮 learn 文档/实验；未提交、未推送 |
| 子模块 | [完整提交记录](evidence/submodules.txt)；本次没有切换依赖版本 |
| 原生工具 | Linux、G++ 13.3.0、CMake 3.28.3、Python 3.12 |
| Python 构建环境 | `/tmp/px4-course-venv`，Ninja 1.13.2、empy 3.3.4 等；[完整包版本](evidence/python-packages.txt) |
| 教学构建 | `build/px4_course`，px4_sitl_default、Debug、外部模块目录 |
| PX4 测试构建 | `build/px4_course_test`，px4_sitl_test、Debug |
| GDB | Ubuntu 包 15.1，解包到 `/tmp/px4-course-gdb/root`，未安装到系统路径 |
| 模拟器与机型 | 原生 SIH，sihsim_quadx，SYS_AUTOSTART=10040 |

已有 lab 的 `doctor px4-main --runtime` 报 Docker socket 不存在，因此本次没有把 Docker/lab 验证记为通过。原生环境通过临时 Python 环境补齐构建包；系统缺少 ensurepip，使用官方 get-pip 为这个临时解释器引导 pip。工具准备没有要求改用另一份 PX4 源码。

GDB 从配置的 Ubuntu 软件源下载并在临时目录解包，依赖通过该目录的 LD_LIBRARY_PATH 加载；课程的通用步骤只要求所在开发环境有正常可用的 GDB，不依赖本次临时目录。临时环境和运行文件可能被清理，长期证据已摘录到本目录。

## 实际构建与测试

下面是本轮执行过的构建选择，环境路径记录用于复核，不是读者必须照用的安装方案：

```bash
env PATH=/tmp/px4-course-venv/bin:$PATH cmake -S . -B build/px4_course -G Ninja \
  -DCONFIG=px4_sitl_default -DCMAKE_BUILD_TYPE=Debug \
  -DPYTHON_EXECUTABLE=/tmp/px4-course-venv/bin/python \
  -DEXTERNAL_MODULES_LOCATION="$PWD/learn/labs/external_module"
env PATH=/tmp/px4-course-venv/bin:$PATH cmake --build build/px4_course --target px4 -j4
```

外部模块完整构建成功。首次配置曾因提前把 `px4_work_queue` 放入外部模块 DEPENDS 而失败；外部模块声明早于平台库，改用后续 `target_link_libraries` 后配置和链接完成。这个差异保存在实验 CMake 的注释中。

测试构建与执行：

```bash
env PATH=/tmp/px4-course-venv/bin:$PATH cmake -S . -B build/px4_course_test -G Ninja \
  -DCONFIG=px4_sitl_test -DCMAKE_BUILD_TYPE=Debug \
  -DPYTHON_EXECUTABLE=/tmp/px4-course-venv/bin/python
env PATH=/tmp/px4-course-venv/bin:$PATH cmake --build build/px4_course_test \
  --target unit-PositionControl unit-rate_control_test unit-test_EKF_height_fusion -j4
ctest --test-dir build/px4_course_test \
  -R '^unit-(PositionControl|rate_control_test|test_EKF_height_fusion)$' --output-on-failure
```

| 验证 | 结果 | 证据与实际覆盖 |
|---|---|---|
| 教学独立算法 CTest | 1/1 通过，60 项检查 | [结果](evidence/algorithm-test.txt)；周期限幅、非法值、限速与连续轨迹 |
| 原有 PositionControl | 17 个 GTest 用例通过 | [用例计数](evidence/px4-cases.txt) |
| 原有 rate_control_test | 1 个 GTest 用例通过 | 同上；不代表所有速率控制场景 |
| 原有 EKF height fusion | 9 个 GTest 用例通过 | 同上；包含 baroRef，非零测试执行 |
| PX4 三组 CTest | 3/3 通过 | [CTest 输出](evidence/px4-tests.txt)，合计 27 个底层 GTest 用例 |

没有执行全仓测试、全平台构建或硬件测试。配置提示缺 Gazebo/Protobuf 等依赖并跳过 GZ bridge；测试与 SIH 的通过不支持“Gazebo 可运行”的结论。

## SIH 与模块运行

实例 77 使用独立 `/tmp/px4-course-sih`；启动参数为 SIH 四旋翼，运行至 commander 报就绪。没有发送解锁、起飞或飞行指令。

`learn_status` 状态、listener 和工作队列均能观察到运行。该次观察中 test1 上约 10 Hz，mc_pos_control 约 125 Hz，mc_att_control/rate/allocator/EKF 约 250 Hz；这些只是这次配置和窗口的读数，不能当作所有 PX4 的固定频率。

运行[检查程序](../labs/check_module.py)实际验证：

1. 模块从停止状态开始，默认周期发布，时间和计数推进。
2. LEARN_PERIOD 从 100 改为 200，有效周期成为 200000 us。
3. 参数 -1 被命令行接受，模块限制为 20000 us，并报告 period_limited=True。
4. 恢复 100，停止后不再更新；重启计数回到新生命周期的低值。
5. 最终停止模块，恢复实验前参数。

完整输出见[运行记录](evidence/module-runtime.txt)。初版检查曾假定停止后一定能读旧样本，本次实际显示 `never published`；检查程序已按“没有继续更新”和重启前最后样本判断，兼容主题撤销后的行为。

另一次最小自定义启动脚本没有完整 SIH 时间推进，周期工作项未按预期运行；随后改用完整 SIH 启动复现。课程因此要求区分调度/模拟时钟与模块逻辑问题。

## 自定义消息日志

实例 79 使用 `/tmp/px4-course-log-run`，`etc` 是 ROMFS 的真实副本，日志配置包含 learn_status 和 vehicle_status。这样修改配置不会通过符号链接写回源码。

教学模块发布后，重启 Logger `-f`，记录 100→200 ms 变化再关闭。`logger status` 报本段无 dropout。解析 `/tmp/px4-course-log-run/log/sess100/log100.ulg` 得到：

| 项目 | 实际值 |
|---|---|
| 文件大小 | 53997 字节 |
| learn_status 样本 | 45 条 |
| 字段 | timestamp、sample_count、period_us、requested_period_ms、period_limited |
| 时间范围 | 8336000～14132000 us，PX4 模拟时钟 |
| 有效周期 | 100000 和 200000 us |
| 时间与计数 | 严格递增；本段无重启 |

[原始操作输出](evidence/logging-runtime.txt)、[提取 CSV](evidence/learn-status.csv)、[日志与代码 SHA-256](evidence/sha256.json)已保留。`inspect_log.py` 对实际 ULog 检查通过。完整 ULog 留在本机临时运行目录；若需长期回放，应另行归档该二进制文件。

## GDB 实际断点与消息对应

实例 80 使用相同 Debug 产物和独立运行目录。设置 `LearnStatus::Run` 断点后由客户端启动模块，实际停止在线程 `wq:test1`。先跨过函数序言，读取业务栈帧：

```text
LearnStatus::Run
px4::WorkQueue::Run
px4::WorkQueueRunner (wq_configurations::test1)
_sample_count = 0
_period_us = 100000
```

发布行临时断点读到 `sample_count=1`、`period_us=100000`、`requested_period_ms=100`。另设参数更新行临时断点；写入 200 后，旧值 100000、next_period 200000；单步后 `_period_us=200000`，恢复运行后 listener 读回 200000。

证据：[GDB 输出片段](evidence/gdb-excerpt.txt)、[恢复运行后的消息与清理](evidence/gdb-messages.txt)。函数序言可能暂时显示错误的 this 位置，因此课程要求到业务语句再读成员。批量粘贴跨越 next 的命令曾被其他线程的 PX4 Shell 读走，之后逐条等待提示符完成验证；课程已写明此操作要求。

这是本机终端 GDB 实测。VS Code 图形配置解释已核对，但本轮没有执行图形界面的逐项操作；SWD、core 与 HardFault 仍需相应平台验证。

## 真实位置控制启动与算法路径

实例 81 使用相同 Debug 产物，GDB 中直接启动 PX4 `-d`，由另一个终端使用实例客户端观察。没有解锁或飞行指令。

- `mc_pos_control_main` 命中，参数为 mc_pos_control/start，栈中包含 Pxh::process_line 和 Server::_handle_client。
- `init` 命中，栈中包含 ModuleBase::main、start_command_base 与 task_spawn。
- Run 内控制条件命中，栈来自 WorkQueueRunner，context 为 nav_and_controllers。
- 本次早期样本中 flag_armed=false、位置控制使能=true、xy_valid=false，目标时间 248000 us ≥ 启用时间 244000 us。
- 实际进入 `_control.update(dt)`；not_taken_off=true，目标经起飞前分支改写为 acceleration={0,0,100}，积分被重置。单步进入保存有效目标分支。
- 启动完成后位置模块 running，work_item_example not running。正常 shutdown 后 GDB 报 inferior exited normally；客户端因服务退出返回非零。

这份结果已完整展开为[主文档第 5 章](../WORKBOOK.md#c05)，用于说明“未解锁”和“算法未执行”不能等同。原始[GDB 记录](evidence/control-trace.txt)与[状态记录](evidence/control-status.txt)可审阅。断点改变时序，早期 dt 和有效性不能作为连续运行性能指标。

## 恢复与验证边界

实验模块停止、周期恢复，Logger 正常关闭。实例 79 与 GDB 实例 80、81 正常 shutdown；早期实例 77 的客户端 shutdown 曾未退出，随后仅在该实例终端 Ctrl+C，进程正常结束。最终进程检查未发现本轮 PX4/GDB 残留。原有仿真运行目录与参数未被这些隔离实验覆盖。

已完成：源码定位、教学完整构建、SIH 启动、模块生命周期/参数/消息检查、ULog、终端 GDB、独立算法和三组原有测试。文档检查命令为 `python3 learn/tools/check_docs.py`；检查本地链接/锚点、围栏、Shell/Python 语法、20 章结构和模块数。其最终输出保存为[结构检查记录](evidence/docs-check.txt)。

待对应环境验证：Gazebo、ROS 2/Agent/外部 Offboard 端到端、动态闭环任务、其他机型、真实板卡/设备/输出、SWD/HardFault、全量测试。相关课程已经提供解释、操作/设计步骤、判据和恢复，但不填写虚构实测结果。个人独立任务和 L3/L4 项目也仍由[学习进度](../WORKBOOK.md#mastery-record)记录。

<a id="coverage"></a>
## 工程覆盖与尚未验证的范围

“课程已写”表示有解释、练习和判断标准；“源码核对”表示本基线存在对应路径；“实测”只指记录中执行过的范围。以下目录覆盖 56/56，不表示所有平台、模块和功能已在运行中验证。学习者等级只在个人进度中记录。

| 源码对象 | 课程 | 接口/入口与解释 | 作者验证范围 | 学习等级 |
|---|---|---|---|---|
| `airship_att_control` | [19](../WORKBOOK.md#c19) | [职责、入口、接口](../SYSTEM_GUIDE.md#module-airship-att-control) | 源码定位；运行待验证 | 待评 |
| `airspeed_selector` | [07](../WORKBOOK.md#c07) | [职责、入口、接口](../SYSTEM_GUIDE.md#module-airspeed-selector) | 源码定位；运行待验证 | 待评 |
| `attitude_estimator_q` | [08](../WORKBOOK.md#c08) | [职责、入口、接口](../SYSTEM_GUIDE.md#module-attitude-estimator-q) | 源码定位；运行待验证 | 待评 |
| `battery_status` | [18](../WORKBOOK.md#c18) | [职责、入口、接口](../SYSTEM_GUIDE.md#module-battery-status) | 源码定位；运行待验证 | 待评 |
| `camera_feedback` | [18](../WORKBOOK.md#c18) | [职责、入口、接口](../SYSTEM_GUIDE.md#module-camera-feedback) | 源码定位；运行待验证 | 待评 |
| `commander` | [11](../WORKBOOK.md#c11) | [职责、入口、接口](../SYSTEM_GUIDE.md#module-commander) | SIH 启动就绪；未验证飞行模式流程 | 待评 |
| `control_allocator` | [10](../WORKBOOK.md#c10) | [职责、入口、接口](../SYSTEM_GUIDE.md#module-control-allocator) | SIH 调度观察；未飞行 | 待评 |
| `dataman` | [11](../WORKBOOK.md#c11) | [职责、入口、接口](../SYSTEM_GUIDE.md#module-dataman) | 源码定位；运行待验证 | 待评 |
| `ekf2` | [08](../WORKBOOK.md#c08) | [职责、入口、接口](../SYSTEM_GUIDE.md#module-ekf2) | 高度融合测试；SIH 状态/调度观察 | 待评 |
| `esc_battery` | [18](../WORKBOOK.md#c18) | [职责、入口、接口](../SYSTEM_GUIDE.md#module-esc-battery) | 源码定位；运行待验证 | 待评 |
| `events` | [11](../WORKBOOK.md#c11) | [职责、入口、接口](../SYSTEM_GUIDE.md#module-events) | 源码定位；运行待验证 | 待评 |
| `flight_mode_manager` | [11](../WORKBOOK.md#c11) | [职责、入口、接口](../SYSTEM_GUIDE.md#module-flight-mode-manager) | SIH 调度观察；未飞行 | 待评 |
| `fw_att_control` | [17](../WORKBOOK.md#c17) | [职责、入口、接口](../SYSTEM_GUIDE.md#module-fw-att-control) | 源码定位；运行待验证 | 待评 |
| `fw_autotune_attitude_control` | [17](../WORKBOOK.md#c17) | [职责、入口、接口](../SYSTEM_GUIDE.md#module-fw-autotune-attitude-control) | 源码定位；运行待验证 | 待评 |
| `fw_lateral_longitudinal_control` | [17](../WORKBOOK.md#c17) | [职责、入口、接口](../SYSTEM_GUIDE.md#module-fw-lateral-longitudinal-control) | 源码定位；运行待验证 | 待评 |
| `fw_mode_manager` | [17](../WORKBOOK.md#c17) | [职责、入口、接口](../SYSTEM_GUIDE.md#module-fw-mode-manager) | 源码定位；运行待验证 | 待评 |
| `fw_rate_control` | [17](../WORKBOOK.md#c17) | [职责、入口、接口](../SYSTEM_GUIDE.md#module-fw-rate-control) | 源码定位；运行待验证 | 待评 |
| `gimbal` | [18](../WORKBOOK.md#c18) | [职责、入口、接口](../SYSTEM_GUIDE.md#module-gimbal) | 源码定位；运行待验证 | 待评 |
| `gyro_calibration` | [07](../WORKBOOK.md#c07) | [职责、入口、接口](../SYSTEM_GUIDE.md#module-gyro-calibration) | 源码定位；运行待验证 | 待评 |
| `gyro_fft` | [07](../WORKBOOK.md#c07) | [职责、入口、接口](../SYSTEM_GUIDE.md#module-gyro-fft) | 源码定位；运行待验证 | 待评 |
| `hardfault_stream` | [16](../WORKBOOK.md#c16) | [职责、入口、接口](../SYSTEM_GUIDE.md#module-hardfault-stream) | 源码定位；运行待验证 | 待评 |
| `internal_combustion_engine_control` | [18](../WORKBOOK.md#c18) | [职责、入口、接口](../SYSTEM_GUIDE.md#module-internal-combustion-engine-control) | 源码定位；运行待验证 | 待评 |
| `land_detector` | [11](../WORKBOOK.md#c11) | [职责、入口、接口](../SYSTEM_GUIDE.md#module-land-detector) | 源码定位；运行待验证 | 待评 |
| `landing_target_estimator` | [08](../WORKBOOK.md#c08) | [职责、入口、接口](../SYSTEM_GUIDE.md#module-landing-target-estimator) | 源码定位；运行待验证 | 待评 |
| `load_mon` | [15](../WORKBOOK.md#c15) | [职责、入口、接口](../SYSTEM_GUIDE.md#module-load-mon) | 源码定位；运行待验证 | 待评 |
| `local_position_estimator` | [08](../WORKBOOK.md#c08) | [职责、入口、接口](../SYSTEM_GUIDE.md#module-local-position-estimator) | 源码定位；运行待验证 | 待评 |
| `logger` | [15](../WORKBOOK.md#c15) | [职责、入口、接口](../SYSTEM_GUIDE.md#module-logger) | 自定义消息 ULog 实测 | 待评 |
| `mag_bias_estimator` | [07](../WORKBOOK.md#c07) | [职责、入口、接口](../SYSTEM_GUIDE.md#module-mag-bias-estimator) | 源码定位；运行待验证 | 待评 |
| `manual_control` | [11](../WORKBOOK.md#c11) | [职责、入口、接口](../SYSTEM_GUIDE.md#module-manual-control) | 源码定位；运行待验证 | 待评 |
| `mavlink` | [13](../WORKBOOK.md#c13) | [职责、入口、接口](../SYSTEM_GUIDE.md#module-mavlink) | 源码定位；运行待验证 | 待评 |
| `mc_att_control` | [09](../WORKBOOK.md#c09) | [职责、入口、接口](../SYSTEM_GUIDE.md#module-mc-att-control) | SIH 调度观察；未飞行 | 待评 |
| `mc_autotune_attitude_control` | [09](../WORKBOOK.md#c09) | [职责、入口、接口](../SYSTEM_GUIDE.md#module-mc-autotune-attitude-control) | 源码定位；运行待验证 | 待评 |
| `mc_hover_thrust_estimator` | [09](../WORKBOOK.md#c09) | [职责、入口、接口](../SYSTEM_GUIDE.md#module-mc-hover-thrust-estimator) | 源码定位；运行待验证 | 待评 |
| `mc_nn_control` | [19](../WORKBOOK.md#c19) | [职责、入口、接口](../SYSTEM_GUIDE.md#module-mc-nn-control) | 源码定位；运行待验证 | 待评 |
| `mc_pos_control` | [05](../WORKBOOK.md#c05) | [职责、入口、接口](../SYSTEM_GUIDE.md#module-mc-pos-control) | 位置算法测试；SIH 状态/调度观察 | 待评 |
| `mc_raptor` | [19](../WORKBOOK.md#c19) | [职责、入口、接口](../SYSTEM_GUIDE.md#module-mc-raptor) | 源码定位；运行待验证 | 待评 |
| `mc_rate_control` | [09](../WORKBOOK.md#c09) | [职责、入口、接口](../SYSTEM_GUIDE.md#module-mc-rate-control) | 角速度算法测试；SIH 调度观察 | 待评 |
| `muorb` | [14](../WORKBOOK.md#c14) | [职责、入口、接口](../SYSTEM_GUIDE.md#module-muorb) | 源码定位；运行待验证 | 待评 |
| `navigator` | [11](../WORKBOOK.md#c11) | [职责、入口、接口](../SYSTEM_GUIDE.md#module-navigator) | 源码定位；运行待验证 | 待评 |
| `payload_deliverer` | [18](../WORKBOOK.md#c18) | [职责、入口、接口](../SYSTEM_GUIDE.md#module-payload-deliverer) | 源码定位；运行待验证 | 待评 |
| `px4iofirmware` | [16](../WORKBOOK.md#c16) | [职责、入口、接口](../SYSTEM_GUIDE.md#module-px4iofirmware) | 源码定位；运行待验证 | 待评 |
| `rc_update` | [11](../WORKBOOK.md#c11) | [职责、入口、接口](../SYSTEM_GUIDE.md#module-rc-update) | 源码定位；运行待验证 | 待评 |
| `replay` | [15](../WORKBOOK.md#c15) | [职责、入口、接口](../SYSTEM_GUIDE.md#module-replay) | 源码定位；运行待验证 | 待评 |
| `rover_ackermann` | [19](../WORKBOOK.md#c19) | [职责、入口、接口](../SYSTEM_GUIDE.md#module-rover-ackermann) | 源码定位；运行待验证 | 待评 |
| `rover_differential` | [19](../WORKBOOK.md#c19) | [职责、入口、接口](../SYSTEM_GUIDE.md#module-rover-differential) | 源码定位；运行待验证 | 待评 |
| `rover_mecanum` | [19](../WORKBOOK.md#c19) | [职责、入口、接口](../SYSTEM_GUIDE.md#module-rover-mecanum) | 源码定位；运行待验证 | 待评 |
| `sensors` | [07](../WORKBOOK.md#c07) | [职责、入口、接口](../SYSTEM_GUIDE.md#module-sensors) | 源码定位；运行待验证 | 待评 |
| `simulation` | [12](../WORKBOOK.md#c12) | [职责、入口、接口](../SYSTEM_GUIDE.md#module-simulation) | SIH 启动；未验证 Gazebo/HITL | 待评 |
| `spacecraft` | [19](../WORKBOOK.md#c19) | [职责、入口、接口](../SYSTEM_GUIDE.md#module-spacecraft) | 源码定位；运行待验证 | 待评 |
| `temperature_compensation` | [07](../WORKBOOK.md#c07) | [职责、入口、接口](../SYSTEM_GUIDE.md#module-temperature-compensation) | 源码定位；运行待验证 | 待评 |
| `time_persistor` | [16](../WORKBOOK.md#c16) | [职责、入口、接口](../SYSTEM_GUIDE.md#module-time-persistor) | 源码定位；运行待验证 | 待评 |
| `uuv_att_control` | [19](../WORKBOOK.md#c19) | [职责、入口、接口](../SYSTEM_GUIDE.md#module-uuv-att-control) | 源码定位；运行待验证 | 待评 |
| `uuv_pos_control` | [19](../WORKBOOK.md#c19) | [职责、入口、接口](../SYSTEM_GUIDE.md#module-uuv-pos-control) | 源码定位；运行待验证 | 待评 |
| `uxrce_dds_client` | [14](../WORKBOOK.md#c14) | [职责、入口、接口](../SYSTEM_GUIDE.md#module-uxrce-dds-client) | 源码定位；运行待验证 | 待评 |
| `vtol_att_control` | [17](../WORKBOOK.md#c17) | [职责、入口、接口](../SYSTEM_GUIDE.md#module-vtol-att-control) | 源码定位；运行待验证 | 待评 |
| `zenoh` | [14](../WORKBOOK.md#c14) | [职责、入口、接口](../SYSTEM_GUIDE.md#module-zenoh) | 源码定位；运行待验证 | 待评 |

### 模块目录之外的工程范围

| 范围 | 解释/课程 | 实践与未覆盖条件 |
|---|---|---|
| [src/drivers](../../src/drivers) | 07、16、18：设备状态机、总线、物理输出 | 代表路径与台架步骤；实际器件未验证 |
| [src/lib](../../src/lib) | 08～10：算法、矩阵、滤波、控制与设备封装 | rate_control 原有测试、独立限速算法已运行 |
| [platforms](../../platforms) | 02～03、16：启动、任务、工作队列、uORB、NuttX | POSIX/SIH 已运行；NuttX/SWD 未验证 |
| [boards](../../boards) | 01、16：目标、Kconfig、板级资源 | SITL default/test 已构建；实体板未构建部署 |
| [ROMFS](../../ROMFS) | 02、11～12、17～19：机型和启动顺序 | SIH 10040 已运行；其他机型仅源码对照 |
| [msg](../../msg) | 04、06、13～14：生成、字段、日志与桥接 | 外部 LearnStatus 生成/发布/记录已验证 |
| [Tools](../../Tools) | 01、12、15：依赖、仿真、生成和分析工具 | 本机原生构建；Docker daemon 不可用 |
| [test](../../test) | 08～09、15：测试框架与回归 | 三组 PX4 原有 CTest 通过；未执行全仓测试 |
| [cmake](../../cmake) | 01、06：构建注册、生成与依赖 | 外部模块与消息接入已构建 |
| [src/examples](../../src/examples) | 03、06：模块生命周期范式 | 独立 learn_status 完整运行检查 |

### 后续扩大覆盖的方法

增加对象时同时补职责、真实入口、接口语义、正常/异常过程、可执行实验和恢复，不只增加目录链接。换版本重新检查目录数、命令注册、消息及参数；运行记录绑定版本，不能跨版本直接继承结论。每一个实际设备型号、模拟器插件和闭源依赖都需要自己的验证条目。
