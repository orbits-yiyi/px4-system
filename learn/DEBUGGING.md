# PX4 开发调试实战：从源码修改到板端故障定位

本文作为**问题诊断与调试工具参考**。连续学习入口是[实战主文档](WORKBOOK.md)，当前环境与实际验证以[实施记录](records/IMPLEMENTATION.md)为准。

[学习首页](README.md) · [从零系统导读](SYSTEM_GUIDE.md) · [硬件与技术架构](ARCHITECTURE.md) · [当前工程全景导读](PROJECT_GUIDE.md) · [代码架构与开发指南](DEVELOPMENT.md) · [操作手册](OPERATIONS.md)

本文提供一条可操作的开发调试路线：建立版本和环境基线，完成一次源码修改，选择正确的调试工具，用数据定位问题，再验证和保存结果。每段命令都说明执行环境；不仅说明“输入什么”，也说明“应该看到什么，以及没有看到时查哪里”。

**第一次调试从[第 0 章](#first-debug)与[可执行 GDB 实验](WORKBOOK.md#debug-lab)开始。** 先核对实际程序，再命中断点、读取变量并与消息对应；不依赖已删除的工作区文件。

**如果困惑的是“每次启动到底有哪些代码参与”，先看[当前启动的源码地图](SYSTEM_GUIDE.md#startup-code)。** 它把 F5 配置、`main.cpp`、`rcS`、机型脚本、模块命令和 C++ 执行函数逐项对应，并给出从启动入口开始的断点位置。

如果还不能区分仿真器、PX4 模块、消息接口和调试器，先看[系统导读中的仿真概念](SYSTEM_GUIDE.md#simulation-basics)、[内部接口](SYSTEM_GUIDE.md#uorb)与[调试方法](SYSTEM_GUIDE.md#debugging)，再按本篇深入实践。

**依据：2026-11-02，本仓库 `learn/px4-v1.17.0`，提交 `3e4c67df8a1518a13ca661f06d9ff8cb8393009b`，以及本机 `embedded-lab` 的 `px4-main` 任务。** 下文实验供读者实施，本次文档编写未执行源码练习、重编译、仿真飞行或硬件烧录。历史验证与当前分支的区别见 [PROJECT_GUIDE](PROJECT_GUIDE.md#environment)。

## 阅读导航

| 目的 | 章节 |
|---|---|
| 不知道本次启动经过哪些文件、断点应放在哪里 | [启动源码地图与函数对应表](SYSTEM_GUIDE.md#startup-code) |
| 第一次调试，不清楚软件和窗口怎样配合 | [0. 逐步操作](#first-debug)：工具与窗口、GDB 启动、断点、单步、读回消息与退出 |
| `px4-sih-debug.code-workspace` 是什么、配置怎样生效、能否删除 | [3.5～3.9：工作区文件详解与替代操作](#workspace-explained) |
| 建立开发闭环 | [1. 工作方法](#workflow)、[2. lab 与编译](#environment)、[3. VS Code 与索引](#editor)、[4. 第一次源码修改](#first-change) |
| 看懂运行与断点 | [5. 控制台和模块](#runtime)、[6. 终端 GDB](#gdb)、[7. VS Code 调试](#vscode-debug) |
| 分析数据和算法 | [8. uORB 与参数](#data)、[9. ULog 与埋点](#logging)、[10. 单元测试与分析工具](#testing)、[11. 性能与资源](#performance) |
| 调试集成与仿真 | [12. MAVLink/ROS 2/Offboard](#integration)、[13. 仿真、故障与 Replay](#simulation) |
| 上实体飞控 | [14. 固件与设备部署](#deployment)、[15. 板端控制台、SWD 与 HardFault](#hardware-debug) |
| 形成自己的能力 | [16. 开发不同功能](#extension)、[17. 故障速查](#triage)、[18. 保存成果与练习顺序](#evidence) |

<a id="first-debug"></a>
## 0. 第一次调试：从一个问题到断点和消息

当前统一操作入口是[第一次源码调试实验](WORKBOOK.md#debug-lab)。它给出构建、终端 GDB 启动、断点、变量、单步、继续、消息核对和退出的完整步骤，并提供 VS Code 等价操作。没有 `px4-sih-debug.code-workspace` 也可以完成。

### 0.1 先确定本次问题

以教学模块 `learn_status` 为例：参数 100 ms 怎样变为 100000 us，哪个线程执行 Run，停止时如何清理？源码与可运行实现见[第 06 课](WORKBOOK.md#c06)，构建命令见[实验入口](WORKBOOK.md#build-example)。

### 0.2 软件和窗口的职责

| 对象 | 做什么 | 不能由它单独证明什么 |
|---|---|---|
| Linux/容器 Shell | 编译、启动 GDB、运行检查脚本 | 某算法分支执行 |
| VS Code | 编辑、索引、组织任务与调试界面 | 真实运行用了哪份二进制 |
| GDB | 加载符号、断点、调用栈、单步 | 不暂停时的连续性能 |
| PX4 Shell | 调用模块、参数、listener、状态命令 | 输出一定已被下游消费 |
| QGC | 地面站遥测、配置和任务协议 | 源码是否重新编译 |
| SIH / Gazebo | 提供动力学和模拟测量 | 真机总线与硬件时序 |

### 0.3 第一次操作应得到什么

按[调试实验](WORKBOOK.md#debug-lab)在 `LearnStatus::Run` 命中，读取 `_sample_count`，沿发布行单步，再禁用断点继续。用 listener 比较相同字段；结束模块和程序后重新构建一个可预测改动，再复现。源码保存不会改变旧进程。

先看提示符：`(gdb)` 输入调试命令，`pxh>` 输入 PX4 命令，`$` 输入系统 Shell 命令。VS Code Debug Console 也不是 PX4 命令终端。F5 在暂停状态表示继续，不会自动重建正在运行的进程。

### 0.4 卡住时的下一步

| 画面/现象 | 检查与下一步 |
|---|---|
| 找不到 gdb | 在实际构建环境检查工具，避免宿主/容器错位 |
| 断点为空心/符号找不到 | program、Debug 产物、源路径、是否编入 |
| Run 不命中 | 模块是否启动、调度是否推进、是否等待消息 |
| 发布行不命中 | Run 中提前退出/条件分支；不要一律重装环境 |
| 高频反复停止 | 禁用已观察断点后继续，用日志测时间序列 |
| 工作区菜单不存在 | 使用当前 GDB 实验或已有生成配置，无需恢复历史文件 |

完成记录要包含实际程序、调用栈、变量、消息和恢复。作者本次运行范围在[实施记录](records/IMPLEMENTATION.md)，板端与其他环境另行验证。

<a id="workflow"></a>
## 1. 一次有效开发应当形成什么闭环

### 1.1 先写行为，再改代码

把“位置控制不对”改写成具体问题，例如：“同一模型与初始状态下，目标切换后水平速度超过设定限制，持续约一秒”。需要明确触发、预期、实际结果和可观察字段。

然后沿以下闭环工作：

```text
确认版本与初始条件 → 复现 → 收集证据 → 定位最早异常层
 → 提出解释 → 修改对应实现 → 针对性验证 → 对比与归档
```

尽量先复现未修改代码的行为。没有基线时，很难知道改动解决了问题，还是模型、参数或环境恰好改变。

### 1.2 按问题选择工具

| 要回答的问题 | 优先工具 | 得到的证据 |
|---|---|---|
| 代码有没有进入目标 | Kconfig、CMake、编译数据库、符号表 | 构建选择与实际产物 |
| 模块有没有启动、为什么退出 | 启动输出、模块 `status`、事件 | 生命周期与状态 |
| 谁发布/消费什么数据 | `.msg`、源码搜索、`listener`、`uorb top` | 接口、实例、字段和更新活动 |
| 为什么走某个分支 | GDB、受控日志、最小测试 | 条件值、调用栈、对象状态 |
| 动态行为何时偏离 | ULog、外部程序日志与事件 | 同一时间线上的输入、状态与输出 |
| 为什么实时性变差 | `perf`、工作队列/任务统计、样本时间 | 执行间隔、耗时、负载与延迟 |
| 为什么板子重启或无响应 | 启动串口、SWD、HardFault、供电测量 | 系统停止层、寄存器、栈和电气状态 |

断点适合研究逻辑，但会暂停执行。估计延迟、控制性能和通信超时应在不被断点打断的场景中测量。

<a id="environment"></a>
## 2. 使用你的 lab 建立构建和运行基线

### 2.1 四种输入位置

| 位置 | 典型提示/路径 | 使用的命令 |
|---|---|---|
| WSL 宿主 Bash | `~/embedded-lab`、`~/Workspace/px4/PX4-Autopilot` | `./bin/lab`、宿主 Git、Docker 状态 |
| lab 容器 Bash | `/workspaces/PX4-Autopilot` | `make`、CMake、CTest、GDB、源码工具 |
| PX4 Shell | SITL 的 `pxh>` 或板上的相应控制台 | `listener`、`param`、模块命令、`shutdown` |
| GDB | `(gdb)` | `break`、`run`、`bt`、`print`、`continue` |

VS Code 的 Debug Console 主要与调试器交互，不能自动当作 PX4 Shell。出现“找不到命令”时，先检查执行位置。

### 2.2 检查工作区和已有任务

在 **WSL 宿主**：

```bash
cd ~/Workspace/px4/PX4-Autopilot
git status --short --branch
git remote -v
git rev-parse HEAD
git submodule status --recursive
```

保留当前未提交改动。子模块前缀 `-` 表示未初始化，`+` 表示检出提交偏离记录；有自己的依赖修改时先核对，再决定更新。正常恢复当前版本使用 `git submodule update --init --recursive`，不使用 `--remote` 代替版本恢复。

继续在 **WSL 宿主**：

```bash
cd ~/embedded-lab
./bin/lab info px4-main
./bin/lab doctor px4-main --runtime
```

应确认 source 指向当前工作区、容器目录是 `/workspaces/PX4-Autopilot`、工具镜像和挂载可用。当前任务资源是 jobs 6、6 CPU、8 GiB。Docker 无法连接时，先处理 Docker 和外部数据盘启动，而不是改 PX4 CMake。

### 2.3 编译、启动检查和交互运行

**WSL 宿主、lab 根目录**：

```bash
./bin/lab run px4-main build px4_sitl_default
./bin/lab run px4-main smoke
./bin/lab run px4-main sitl
```

按顺序检查每一步成功后再继续。第一步应生成 `build/px4_sitl_default/bin/px4` 并记录产物报告；第二步验证 SIH 启动与退出；第三步进入 `sihsim_quadx` 交互环境。

在 **PX4 Shell**：

```text
ver all
simulator_sih status
commander status
listener vehicle_attitude -n 1
shutdown
```

`ver all` 应与本次构建版本对应。仅启动脚本成功还不够，应检查仿真器、核心模块和数据。`shutdown` 正常退出后再重新编译或换模型。

### 2.4 Debug 与普通构建怎样切换

先在 **WSL 宿主**进入工具环境：

```bash
cd ~/embedded-lab
./bin/lab shell px4-main
```

然后在 **容器 Bash**：

```bash
cd /workspaces/PX4-Autopilot
make -j6 px4_sitl_default PX4_CMAKE_BUILD_TYPE=Debug PX4_MAKE_ARGS=-j6
rg '^CMAKE_BUILD_TYPE:' build/px4_sitl_default/CMakeCache.txt
```

查询应显示 Debug。该命令会重新配置当前目标并增量构建，不要同时运行另一构建或 SITL 写同一目录。切回日常类型时显式指定：

```bash
make -j6 px4_sitl_default PX4_CMAKE_BUILD_TYPE=RelWithDebInfo PX4_MAKE_ARGS=-j6
```

不带类型参数的下一次构建可能沿用缓存，不能假设自动恢复。某些模块还有自己的优化选项，Debug 也不保证所有变量都可见；最终以编译条目和符号为准。

### 2.5 编译失败先识别阶段

| 失败阶段 | 典型表现 | 下一步 |
|---|---|---|
| 子模块/网络 | git checkout/download 失败、目录为空 | 核对依赖路径、提交和下载错误 |
| CMake 配置 | 找不到工具、依赖或目标 | 核对容器、目标、包版本和配置 |
| 消息/参数生成 | Python 异常、定义无效 | 查首个生成错误、输入格式和依赖 |
| C/C++ 编译 | 类型、头文件、语法错误 | 查第一处实际错误及该文件编译参数 |
| 链接 | undefined reference、内存区域溢出 | 查实现是否编入、库依赖与容量 |
| 打包 | 板信息、产物或资源错误 | 查目标打包元数据和路径 |
| 运行 | 动态库、脚本、模块或数据错误 | 构建结束，进入启动与运行诊断 |

并行构建最后打印的 `ninja: build stopped` 通常是汇总，不是根因。先找到前面的第一条失败命令，保留上下文。

<a id="editor"></a>
## 3. VS Code、编译数据库与源码索引

### 3.1 打开正确环境

**WSL 宿主**：

```bash
cd ~/embedded-lab
./bin/lab code px4-main
```

在新窗口执行 **Dev Containers: Reopen in Container**。确认终端工作目录、编译器和源码都在同一容器里。源码自身的 `.devcontainer` 使用另一套较旧镜像，当前学习应优先从 lab 入口打开。

### 3.2 `compile_commands.json` 为什么重要

PX4 的宏、包含路径和生成头随目标变化。编译数据库记录每个 C/C++ 翻译单元的真实编译命令，比手动把所有目录加入 include path 更准确。

**容器 Bash**检查某个文件是否被当前目标编入：

```bash
python3 - <<'PY'
import json
from pathlib import Path
p = Path('build/px4_sitl_default/compile_commands.json')
entries = json.loads(p.read_text())
matches = [e for e in entries if e['file'].endswith('/WorkItemExample.cpp')]
print('matching entries:', len(matches))
for e in matches:
    print('directory:', e['directory'])
    print('file:', e['file'])
    print(e.get('command', e.get('arguments')))
PY
```

应找到示例的编译条目。没有条目时先检查目标选择与构建是否完成，不立即认定编辑器坏了。

### 3.3 红色波浪线的排查顺序

1. 实际命令行能否编译当前文件所在目标。
2. 编辑器是否在容器中，`/workspaces/...` 是否可访问。
3. 对应编译数据库和生成头是否存在、是否来自当前目标。
4. C/C++ 扩展使用哪个 configuration provider、是否匹配该构建。
5. 文件是否被目标排除，是否应换硬件目标分析。

当前 `.vscode/c_cpp_properties.json` 配置了 CMake Tools provider 和容器内生成头；lab 入口还指定了 SITL 编译数据库。使用扩展的诊断日志查看**最终生效的配置**，不要仅看某个 JSON 字段就判断索引来源。

### 3.4 调试配置可能被 CMake 重生成

当前 [platforms/posix/CMakeLists.txt](../platforms/posix/CMakeLists.txt) 在 SITL 配置阶段把调试模板复制到源码的 `.vscode/launch.json`。所以手工加入的配置可能在重新配置后被覆盖。

保存自定义配置的原始片段；可以在自己的 VS Code 工作区配置中管理独立 `launch` 设置，或者在完成构建后重新合并需要的配置。调整上游模板属于另一类源码改动，应有明确目的并单独审阅。

<a id="workspace-explained"></a>
### 3.5 `px4-sih-debug.code-workspace` 到底是什么

**它曾是教程提供的可选 VS Code 配置，目前已经删除，不需要恢复。** 下面保留历史字段设计的解释，便于理解同类文件。删除只移除该文件提供的设置、任务与调试入口；当前操作使用[终端实验](WORKBOOK.md#debug-lab)或已有 VS Code 配置。

文件名中的 `px4` 表示项目，`sih` 表示这套配置选择的仿真方式，`debug` 表示用途；这些都是人为命名。`.code-workspace` 是 VS Code 识别的工作区文件扩展名，内容按 JSON/JSONC 格式组织。历史示例使用不带注释的 JSON。它可以保存多个源码目录，本例只登记一个 PX4 根目录，并没有复制源码或创建第二套 PX4。

**谁读取它？** VS Code 读取工作区、设置和任务；Microsoft C/C++ 扩展处理 `cppdbg` 调试配置；CMake Tools 和 C/C++ 扩展读取各自的设置。PX4 的 CMake 构建入口、`bin/px4` 和飞控板都不读取这个文件。

**什么时候生效？** 在 VS Code 中使用 **File → Open Workspace from File…** 打开它以后，工作区设置与任务才进入相应作用范围。仅在编辑器中打开文件查看 JSON，不等于切换到该工作区；仅打开源码文件夹，也不会自动加载 `learn/` 下这个文件的配置。

**为什么新增它？** 当前仓库的 [POSIX CMake 配置](../platforms/posix/CMakeLists.txt) 会根据目标，把 [SITL 调试模板](../platforms/posix/Debug/launch_sitl.json.in)等复制到 `.vscode/launch.json`。将教学配置单独保存在工作区中，可以保留明确的构建目标、程序路径与模型选择，并避开这一步生成操作对自定义调试条目的覆盖。

### 3.6 五个顶层部分分别控制什么

以下表格解释历史示例采用的字段，不要求打开一个当前不存在的文件。路径里的 `${workspaceFolder:PX4-Autopilot}` 由 VS Code 展开，不是 Bash 环境变量，也不是 C++ 代码。

| 部分和字段 | 当前含义 | 修改时影响什么 |
|---|---|---|
| `folders` → `name: PX4-Autopilot` | 给源码目录一个工作区名称，供资源管理器和变量引用使用 | 改名后要同时修改 `${workspaceFolder:PX4-Autopilot}` 引用 |
| `folders` → `path: ..` | 相对于这个工作区文件所在的 `learn/`，向上一层得到仓库根目录 | 移动文件位置后要重新计算相对路径；与终端当前目录无关 |
| `settings` → `cmake.configureOnOpen: false` | 关闭 CMake Tools 打开工程时自动配置的行为 | 不影响手动 `make`、CMake 配置或 F5 的构建任务 |
| `settings` → `C_Cpp.default.compileCommands` | 为 C/C++ 扩展提供当前 SITL 的编译数据库位置，帮助解析宏和头文件 | 改变索引配置，不会因此改变实际编译目标；provider 等配置仍需按第 3.3 节核对 |
| `extensions` → `recommendations` | 推荐安装 `ms-vscode.cpptools`，即 Microsoft C/C++ 扩展 | 推荐项不等于扩展已经安装，也不负责安装编译器和 GDB |
| `tasks` | 定义一个可重复执行的构建任务 | 决定构建时运行什么命令，以及怎样展示输出 |
| `launch` | 定义一项由调试器启动 PX4 的配置 | 决定运行哪个程序、带哪些参数和环境变量 |

历史示例没有 Docker 镜像、容器服务或 Dev Containers 连接定义。在已连接的容器窗口中，这些路径和工具在容器中解析；在宿主窗口打开时，就按宿主环境解析。工作区文件本身不会把宿主自动变成开发容器。只有打开文件，也不会自动启动 PX4；当前配置需要显式运行任务或开始调试。

`cmake.configureOnOpen` 还在源码目录的 `.vscode/settings.json` 中设为 `false`。工作区和文件夹设置有不同作用范围；支持文件夹覆盖的同名设置要考虑文件夹层，不能只读工作区中的值判断最终结果。

### 3.7 构建任务与调试配置逐项解释

下面按历史示例的任务说明字段；你现在使用的条目以实际配置为准：

| 字段 | 作用 |
|---|---|
| `version: 2.0.0` | VS Code 任务配置格式版本，与 PX4 版本无关 |
| `label: PX4: build SITL Debug` | 任务的名称；调试配置通过这个名称找到它 |
| `type: process`、`command: make` | 直接启动 `make` 进程，并把 `args` 中每一项作为参数传入 |
| `args` 中的 `-j6` | 请求并行构建；本教程按当前 lab 的资源约定选取 6 |
| `args` 中的 `px4_sitl_default` | 选择电脑端 SITL 默认构建目标，决定编入的模块和平台 |
| `PX4_CMAKE_BUILD_TYPE=Debug` | 由仓库 Makefile 传给 CMake，选择 Debug 构建类型 |
| `PX4_MAKE_ARGS=-j6` | 向底层构建工具传递并行参数，含义以当前 Makefile 的生成器分支为准 |
| `options.cwd` | 让 `make` 在 PX4 仓库根目录执行 |
| `problemMatcher: $gcc` | 把可识别的编译器错误、警告关联到 VS Code Problems 面板；不负责验证飞行行为 |
| `presentation.reveal: always` | 运行任务时显示任务终端 |
| `presentation.panel: dedicated`、`clear: false` | 为这个任务复用专用终端，并保留已有输出 |

因此它的主要动作等同于在仓库根目录执行：

```bash
make -j6 px4_sitl_default PX4_CMAKE_BUILD_TYPE=Debug PX4_MAKE_ARGS=-j6
```

**构建目标和仿真模型属于不同层。** 这条命令构建 SITL 程序；下面 `launch.environment` 再选择运行时使用 SIH 四旋翼。只修改显示名称中的 “SIH” 不会更换模拟器。

再看 `launch` 中的配置：

| 字段 | 作用 |
|---|---|
| `version: 0.2.0` | VS Code 调试配置格式版本，与 PX4 版本无关 |
| `name: PX4 first debug: SIH` | 显示在 Run and Debug 下拉框中的名称，可用于区分不同启动方式 |
| `type: cppdbg` | 使用 Microsoft C/C++ 扩展提供的 GDB/LLDB 调试适配器 |
| `request: launch` | 由调试器新启动一个程序；这里没有附加到其他终端已经运行的 PX4 |
| `preLaunchTask` | 启动调试前执行同名构建任务，名称须与 `tasks` 中的 `label` 对应 |
| `program` | 指定实际被调试的 `build/px4_sitl_default/bin/px4` 可执行文件 |
| `args` | 把 `ROMFS/px4fmu_common` 路径传给 PX4，提供启动脚本等资源；这里不传编译器参数 |
| `cwd` | PX4 的运行目录，当前为目标的 `rootfs`；相对参数文件、日志等受运行目录影响 |
| `MIMode: gdb`、`miDebuggerPath: /usr/bin/gdb` | 选择 GDB，并明确调试器的可执行文件位置 |
| `miDebuggerArgs: -nx` | 让 GDB 忽略自动初始化文件，避免当前生成的 `.gdbinit` 提前执行 `run` |
| `externalConsole: false` | 使用 VS Code 内部的终端承载方式；交互能力还取决于扩展和远程环境 |
| `internalConsoleOptions: neverOpen` | 不自动弹出 Debug Console，不会关闭 PX4 终端 |
| `stopAtEntry: false` | 不要求在程序入口自动暂停；显式设置的断点仍然生效 |
| `environment` → `PX4_SIM_MODEL=sihsim_quadx` | 选择模型名称；正常未被其他入口覆盖时，对应机型编号 10040 |
| `environment` → `PX4_SIMULATOR=sihsim` | 选择 SIH 仿真启动分支；其他继承环境和保存参数仍应按启动脚本核对 |
| `setupCommands` → `-enable-pretty-printing` | 启用 GDB 支持的美化显示，便于查看复杂变量；此项失败允许继续 |
| `setupCommands` → `handle SIGCONT nostop noprint nopass` | 按 PX4 的调试约定，不因该信号暂停、打印或把它传给被调试程序；当前配置不忽略此命令执行失败 |

`setupCommands` 中的 `text` 是交给调试器的命令，`ignoreFailures` 决定适配器是否忽略该条命令执行失败。这里的设置不会自动忽略 PX4 模块报错。

这个文件也没有 `work_item_example start`：F5 启动的是整套选定的 PX4 仿真，教学模块由你之后在 `pxh>` 中手动启动。

### 3.8 按 F5 时，各软件实际怎样配合

当没有活动调试会话，并选中 `PX4 first debug: SIH` 时：

```text
VS Code 读取当前选中的 launch 配置
  → 找到 preLaunchTask 指定的构建任务
  → 在仓库根目录启动 make
  → PX4 的 Makefile/CMake/底层构建工具生成 bin/px4
  → C/C++ 调试适配器启动 /usr/bin/gdb
  → 设置调试参数、程序参数、运行目录与环境变量
  → GDB 启动 bin/px4
  → PX4 执行 main()、启动脚本并启动模块
```

调试器负责断点和单步，PX4 的脚本与模块实现负责运行行为。这份配置把这些工具连接起来，没有把它们的功能实现放进 JSON 中。调试已经暂停时，**F5 表示继续运行**，不会重新编译；修改源码后要结束当前会话，再启动新会话才能使用新产物。

仓库自带的 `.vscode/launch.json` 也提供 SIH 入口。当前生成的配置名为 `SITL (sihsim_quadx SYS_AUTOSTART=10040)`，通过 `${command:cmake.launchTargetPath}` 和 `${command:cmake.buildDirectory}` 获取路径，并设置 `PX4_SYS_AUTOSTART=10040`。使用它需要核对 CMake Tools 当前选择的目标、构建目录以及对应任务。它与本教程工作区是两种可选入口，名称和设置不能混用。

### 3.9 能否删除，删除以后怎样操作

**可以删除，当前已经删除。** 本节解释删除影响；当前推荐的完整替代操作在[调试实验](WORKBOOK.md#debug-lab)。

| 对象 | 删除工作区文件后的影响 |
|---|---|
| 这份文件定义的目录集合、设置、任务与调试入口 | 文件无法再被打开加载；正在打开的窗口可能暂时保留内存状态，关闭工作区再按文件夹打开后核对 |
| `PX4 first debug: SIH` 和 `PX4: build SITL Debug` | 失去这份文件提供的定义；同名条目若被另存到其他配置，则按那份配置生效 |
| PX4 源码、Kconfig、CMake、启动脚本、SIH 模块 | 保留，仍可通过现有构建与运行命令使用 |
| `build/px4_sitl_default` 中的程序、缓存、参数和日志 | 保留；删除工作区不会清理它们，也不会把 Debug 构建自动切回其他类型 |
| `.vscode/settings.json`、`.vscode/launch.json`、`.vscode/tasks.json` | 保留；特别是文件夹中的 `cmake.configureOnOpen: false` 仍然生效 |
| Docker、lab 配置与已安装扩展 | 保留；工作区文件不是这些工具的安装或卸载入口 |
| 已启动的 PX4/GDB 进程 | 删除文件本身不能当作正常退出操作；先按当前会话的退出步骤结束调试 |
| 当前课程与第 0 章 | 已使用终端 GDB 和现存配置，不依赖历史文件 |

如果选择删除，先正常结束当前调试，再关闭这个工作区，使用 **File → Open Folder…** 打开 PX4 源码目录；远程开发时仍需保留正确的容器连接。随后选用已有入口，或者按下面的终端方式完成相同的构建和 GDB 启动。移动或改名工作区文件时，也要更新文档链接与 `folders.path` 等路径引用。

**不依赖这个文件的终端方式**：在已具备工具的 **lab 容器 Bash** 中执行，启动前结束其他占用同一实例和运行目录的 PX4：

```bash
cd /workspaces/PX4-Autopilot
make -j6 px4_sitl_default PX4_CMAKE_BUILD_TYPE=Debug PX4_MAKE_ARGS=-j6
```

构建成功后再执行：

```bash
cd /workspaces/PX4-Autopilot/build/px4_sitl_default/rootfs
env PX4_SIM_MODEL=sihsim_quadx PX4_SIMULATOR=sihsim \
  gdb -nx --args ../bin/px4 /workspaces/PX4-Autopilot/ROMFS/px4fmu_common
```

在 **GDB 的 `(gdb)` 提示符**下输入：

```text
handle SIGCONT nostop noprint nopass
break WorkItemExample::Run
run
```

出现 **PX4 的 `pxh>` 提示符**后输入 `work_item_example start`，再按[第 6 章](#gdb)查看调用栈、单步、继续和退出。这里给出的是与工作区相同的主要构建、模型和调试器选择；界面显示方式不同，也不代表已经在本次会话实测成功。

字段解释依据本基线 Makefile、CMake 和 VS Code 配置语义；当前实验结果与限制统一见[实施记录](records/IMPLEMENTATION.md)。

<a id="first-change"></a>
## 4. 第一次源码修改：验证“修改、编译、运行”真的连起来

### 4.1 为什么选择现有工作项示例

当前 SITL 默认配置启用了 `CONFIG_EXAMPLES_WORK_ITEM=y`，运行命令为 `work_item_example`。源码在 [WorkItemExample.cpp](../src/examples/work_item/WorkItemExample.cpp)，它订阅 `sensor_accel` 触发执行，发布 `orb_test`，并具有性能计数和状态命令。

这个练习不需要接管飞行控制，可以观察模块生命周期、消息和工作队列。

### 4.2 先观察未修改行为

启动 SIH 后，在 **PX4 Shell**逐条执行：

```text
work_item_example start
work_item_example status
listener orb_test -n 1
work_queue status
work_item_example stop
shutdown
```

当前源码发布的 `orb_test.val` 为 `314159`，时间戳随运行更新。若提示 `never published`，先确认模块启动成功、`sensor_accel` 有更新、没有其他发布者造成混淆。停止后保留的最后一条消息也可能仍可读取，应看时间是否继续更新。

### 4.3 做一次最小修改

在编辑器打开 `WorkItemExample.cpp`，找到：

```cpp
data.val = 314159;
```

仅为这个练习，把它改为：

```cpp
data.val = 271828;
```

保存文件，确保旧实例已退出，在容器内重新编译相同目标，或者回到宿主使用 lab build。再次启动 SIH 和该示例，读回 `orb_test`，应看到 `271828`。

如果仍是旧值，依次检查：文件是否保存 → 构建是否成功 → 编译目标是否包含它 → 启动的是哪个二进制 → 是否仍连接旧进程 → 是否存在其他发布者。

### 4.4 从这个实验学会读生命周期

把读到的行为对应到构造函数、`init()`、`Run()`、`print_status()` 和停止路径。`init()` 注册回调，`Run()` 消费数据并发布，`status` 打印已有计数器。

练习结束后可手工恢复这一行或在自己的学习分支保存实验。不要用整仓库重置来撤销一个行级练习。后续真正新增功能时再按行为要求设计测试，不需要为常量替换编写镜像式测试。

<a id="runtime"></a>
## 5. 控制台诊断：模块、主题、状态和执行上下文

### 5.1 一组基础观测命令

以下在 **PX4 Shell**运行，相关命令以当前目标编入且模块已启动为前提：

```text
ver all
commander status
ekf2 status
mc_pos_control status
control_allocator status
mavlink status
logger status
work_queue status
uorb top -1
perf
```

这些命令分别描述版本、模块状态、队列、消息活动和已有计数器。它们不能互相替代：模块 running 不代表算法输入有效，主题发布频率正常也不代表字段正确。

### 5.2 某模块没有输出时的检查链

```text
构建是否选中
 → 运行命令是否存在
 → 是否启动成功
 → 回调/周期是否触发
 → 输入实例是否正确且新鲜
 → 模式和有效性分支是否允许
 → 算法是否产生有效结果
 → 发布者与消费者是否连接一致
```

读模块 `Run()` 时，特别标出所有提前返回、数据检查和发布点。很多“算法没工作”实际停在输入或模式条件。

### 5.3 工作项与线程的对应

WorkItem 的对象不是独立线程。相同队列上的工作共享执行上下文；GDB 中看到的线程名可能是队列名，而不是模块类名。

同一队列某项阻塞，其他项可能停止更新。结合 `work_queue status`、性能计数和 `thread apply all bt` 判断哪个调用占用了队列，而不是只重启表现异常的消费者。

<a id="gdb"></a>
## 6. 终端 GDB：完整调试一套 SIH

### 6.1 准备 Debug 二进制与运行目录

按第 2.4 节完成 Debug 构建，退出其他使用同一实例和运行目录的 PX4。**容器 Bash**：

```bash
cd /workspaces/PX4-Autopilot
test -x build/px4_sitl_default/bin/px4
test -d build/px4_sitl_default/rootfs
```

两项都成功后进入运行目录：

```bash
cd /workspaces/PX4-Autopilot/build/px4_sitl_default/rootfs
env PX4_SIM_MODEL=sihsim_quadx PX4_SIMULATOR=sihsim \
  gdb -nx --args ../bin/px4 /workspaces/PX4-Autopilot/ROMFS/px4fmu_common
```

参数指定 ROMFS 资源，cwd 为该目标 rootfs，环境选择 SIH 四旋翼。`-nx` 忽略自动初始化文件：当前生成的 `.gdbinit` 自带 `run`，忽略它可以先设置断点再启动。

### 6.2 从模块断点观察执行

在 **GDB**中：

```text
set pagination off
handle SIGCONT nostop noprint nopass
break WorkItemExample::Run
run
```

SIGCONT 处理与本仓库 POSIX 调试配置一致，避免工作队列信号频繁中断调试。程序启动后出现 `pxh>`，此时在 **PX4 Shell**输入：

```text
work_item_example start
```

命中断点后提示回到 `(gdb)`，现在输入：

```text
bt
info threads
info args
info locals
print this
print _armed
next
```

`bt` 展示调用链，`this` 是当前对象，`_armed` 是它缓存的状态。函数刚进入时，后面声明的局部变量还没有进入作用域；需要单步到对应位置再读。

### 6.3 高频断点怎样使用

每个样本触发的函数可能每秒运行很多次。第一次命中确认路径后，可在 GDB 执行：

```text
disable
continue
```

`disable` 在这个练习中禁用所有当前断点，避免继续运行时立即再次停下。需要只处理某个断点时先 `info breakpoints`，按其编号操作。

条件断点适合锁定少见情况，例如某函数可见的 `dt` 超出预期；表达式必须在该断点作用域有效。条件尽量直接读值，不调用可能改变程序状态的函数。

恢复运行后，在 PX4 Shell 正常停止示例并 `shutdown`，进程结束后在 GDB `quit`。`Ctrl+C` 在 GDB 管理的前台进程中通常用于暂停回到调试器，不等于 PX4 正常退出。

### 6.4 常用命令与适用场景

| GDB 命令 | 用途 | 使用要点 |
|---|---|---|
| `break 类名::函数` | 函数断点 | C++ 名称和实际符号需匹配 |
| `break 文件:行号` | 源码行断点 | 行应有可执行指令，优化可能改变映射 |
| `next` / `step` / `finish` | 越过、进入、运行到当前函数返回 | 高并发下其他线程也可能运行 |
| `print 表达式` | 读变量/对象 | 对象生命周期、作用域和优化影响可读性 |
| `watch 表达式` | 监视写入变化 | 硬件支持、数量和大小有限；对象重建后应重设 |
| `bt full` | 当前栈及局部信息 | 先看发生问题的线程 |
| `thread apply all bt` | 所有线程调用栈 | 找死锁、阻塞和队列占用 |
| `info registers` | 寄存器 | 崩溃和板端调试常用 |
| `info line` / `disassemble` | 源码与机器指令对应 | 用于理解跳行、内联和优化 |

### 6.5 断点不命中怎样查

| 可能原因 | 检查方式 |
|---|---|
| 加载了旧二进制 | GDB `info files`、文件时间、构建输出与 `ver all` |
| 函数未编入 | 编译数据库、CMake/Kconfig、`info functions`、符号表 |
| 模块未启动或无触发 | Shell 状态、回调、输入主题和模式条件 |
| 源码与符号不一致 | 构建提交、实际路径、源码映射 |
| 优化/内联改变位置 | 构建类型、单文件选项、反汇编 |
| 条件从未成立 | 先使用普通断点观察值，再收紧条件 |

`optimized out` 表示当前调试信息无法提供值，不能解释为零。

### 6.6 attach 与 core 分析

启动自己的调试子进程和附加已有进程的权限条件不同。attach 前核对 PID、用户、容器 PID namespace 和实际可执行文件。当前 lab 配置不自动授予所有 ptrace 能力，遇到明确权限错误后再检查 `SYS_PTRACE`、seccomp 和宿主策略。

SITL 崩溃时保存匹配二进制、源码、配置和输入。在启动它的 **容器 Bash**检查：

```bash
ulimit -c
cat /proc/sys/kernel/core_pattern
```

必要时对该终端设置 `ulimit -c unlimited`。core 的生成位置取决于系统 handler，不能假设当前目录必有文件。拿到实际文件后：

```bash
gdb -nx /absolute/path/to/matching/px4 /absolute/path/to/core
```

路径是占位示例，替换为真实文件。随后用 `bt full`、所有线程栈、寄存器和对象状态分析；不要先重编译覆盖唯一匹配的可执行文件。

<a id="vscode-debug"></a>
## 7. VS Code 图形调试配置

### 7.1 一个与本机路径匹配的 SIH 配置

第一次使用时先完成[终端调试实验](WORKBOOK.md#debug-lab)，再用下方对象理解或整合图形配置。当前不存在专用教学 workspace；检查真实 program、cwd、模型与构建任务。

先完成 Debug 构建，并按第 3.4 节处理配置重生成问题。将下面对象加入调试配置的 `configurations` 数组，保留已有条目：

```json
{
  "name": "PX4 lab SIH Debug",
  "type": "cppdbg",
  "request": "launch",
  "program": "${workspaceFolder}/build/px4_sitl_default/bin/px4",
  "args": ["${workspaceFolder}/ROMFS/px4fmu_common"],
  "cwd": "${workspaceFolder}/build/px4_sitl_default/rootfs",
  "MIMode": "gdb",
  "miDebuggerPath": "/usr/bin/gdb",
  "miDebuggerArgs": "-nx",
  "externalConsole": false,
  "stopAtEntry": false,
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

此配置面向容器里的 Microsoft C/C++ 调试扩展；不是宿主 Windows 的 GDB 配置。它只启动当前产物，没有自动编译步骤，因此修改后先重新构建。

### 7.2 怎样使用界面

在 `WorkItemExample::Run()` 内放一个断点，选择此配置启动，在承载 PX4 进程输入的终端执行示例启动命令。命中后用 Variables、Watch、Call Stack 和 Threads 查看同样的信息。

部分扩展/终端组合的交互输入有差异。若无法给 `pxh>` 输入，不要把 PX4 命令发到 Debug Console；先用第 6 章终端 GDB 确认程序正常，再调整终端承载。

暂停后分析代码，恢复后再评估时序。断点触发造成的 Offboard 断链或外部程序超时，不能作为正常运行的故障结论。

<a id="data"></a>
## 8. 调试 uORB、参数和跨模块数据

### 8.1 从主题定义追到生产与消费

在 **源码根目录 Bash**：

```bash
rg --files msg | rg 'VehicleLocalPosition\.msg$'
rg -n 'ORB_ID\(vehicle_local_position\)' src/modules src/lib
```

先读 `.msg` 的单位、坐标、时间和有效性，再读发布条件和消费条件。不能仅凭变量名断定它是测量值、估计值还是目标值。

在 **PX4 Shell**观察：

```text
listener vehicle_local_position -n 1
listener sensor_gyro -i 0 -n 5 -r 2
uorb top -1
```

第二条指定实例 0、最多观察 5 条、限制读取速率为 2 Hz；它不改变传感器生产速率，也不保证实例 0 就是当前被选中的主设备。

### 8.2 一条数据需要核对的维度

| 维度 | 具体检查 |
|---|---|
| 来源 | 哪个发布者、哪个设备/估计器实例 |
| 时效 | 采样时间、发布时间、当前年龄、外部时钟换算 |
| 单位 | rad/deg、m/mm、积分量/瞬时量、归一化值/物理量 |
| 坐标 | NED/ENU、FRD/FLU、原点、旋转方向 |
| 有效性 | bool 标志、质量、NaN、初始化状态 |
| 连续性 | reset counter、delta、实例切换 |
| 速率/队列 | 生产消费关系、丢样、批量处理与积压 |
| 权限 | 当前模式是否允许消费者使用该目标 |

能够读取主题不意味着数据新鲜。`copy()` 返回成功后仍需检查语义；停止发布后留存的旧值尤其容易误导。

### 8.3 参数为什么“改了却没效果”

以 `MPC_XY_P` 为例，在 Bash 搜索定义与读取：

```bash
rg -n 'MPC_XY_P' src ROMFS
```

在 PX4 Shell 查询当前值：

```text
param show MPC_XY_P
```

实验修改前记录原值，选择明确目标和输入，修改后观察模块更新、派生量、目标和反馈。可能的原因包括：改错参数、当前模式没走该路径、模块尚未刷新、参数要求重启、限幅掩盖作用、保存配置或其他设置影响行为。

参数重置不是通用诊断动作。需要恢复时尽量恢复本次改动项，并保留校准、airframe 和通信等配置。

<a id="logging"></a>
## 9. ULog、实时曲线与开发埋点

### 9.1 实验前确定要记录什么

记录集应能覆盖因果链。例如位置跟踪问题至少包含目标、估计位置/速度、控制模式、姿态/角速度目标与反馈、推力/力矩、分配和相关事件。估计问题还需要输入测量、融合状态、创新和重置信息。

当前默认主题在 [logged_topics.cpp](../src/modules/logger/logged_topics.cpp)。主题是否记录、记录速率和实例数受配置影响，不能假设所有 uORB 都进入 ULog。

### 9.2 启动并检查记录

在 **PX4 Shell**先查询：

```text
logger status
param show SDLOG_MODE
param show SDLOG_PROFILE
```

若本次静止实验需要立即记录，且 Logger 已运行，可使用：

```text
logger on
```

完成需要的观测后使用 `logger off`，并正常结束实验。`on/off` 会覆盖本次运行中的自动启停判断；按当前帮助和实验需求使用，不把它当作永久参数设置。

在 **容器 Bash**查找实际文件：

```bash
cd /workspaces/PX4-Autopilot
rg --files --hidden --no-ignore build/px4_sitl_default -g '*.ulg'
```

板端文件通常在相应日志存储目录，具体以后端配置为准。保存记录时附上版本、参数、操作步骤和时间范围。

### 9.3 自定义日志主题要避免丢掉基线信息

当前实现读取 `PX4_STORAGEDIR/etc/logging/logger_topics.txt`；`PX4_STORAGEDIR` 取决于平台与目标配置，先确认本次实际运行存储位置。文件行格式为 `主题名 间隔毫秒 [实例]`，例如下面只展示语法，并不是完整实验记录集：

```text
vehicle_status 200
vehicle_local_position 20
vehicle_attitude 20
```

这里 `20` 是 20 毫秒，限制记录间隔，不是要求生产者以 20 Hz 发布。实际可记录频率还受主题更新速率等条件限制。

**当前代码在从自定义文件成功加入至少一个主题后，会跳过按 `SDLOG_PROFILE` 初始化默认主题集合。** 因此不能只写一个新主题后就假设原有集合全部保留。自定义时明确包含需要的模式、目标、反馈与诊断主题，并在重启 Logger/新实验后检查实际日志；若只想扩展源码默认集，可研究 `add_default_topics()` 或合适的 profile，记录相应配置/源码改动。

若提高记录速率或增加大量主题，要观察 Logger buffer、dropout 和磁盘吞吐。高频率不等于高质量，写不下的记录会破坏时间分析。

### 9.4 分析顺序和输出结论

1. 检查日志覆盖的起止时间、模式、版本与参数。
2. 确认 dropout、关键主题是否齐全以及记录频率。
3. 用事件和状态变化对齐操作时刻。
4. 从最早异常的输入或状态开始，沿目标与控制输出追踪。
5. 比较同条件的基线与修改后结果，量化差异。

Flight Review、PlotJuggler、pyulog 等是常见分析工具，是否安装以实际环境为准。主题时间不同需要按含义对齐；不要把不同采样点直接逐行相减。

有效结论应形如：“目标切换后，速度目标在时刻 A 被限幅，反馈在时刻 B 开始跟随，执行器同时饱和，修改后误差减少但饱和持续时间不变。”避免仅以“曲线好看了”判断。

### 9.5 怎样添加诊断信息

临时分支判断可用低频文本日志，持续变量优先使用合适的诊断主题和 Logger，耗时使用 `perf` 计数器。高频控制循环中的逐样本打印会改变调度和 I/O 负载。

埋点应记录足够上下文，例如时间、实例、模式、输入有效性与状态转换原因。调试后审阅是否应保留，避免把临时大量输出带入常规运行。

<a id="testing"></a>
## 10. 算法测试、Sanitizer、格式与静态分析

### 10.1 先测试确定输入，再测试完整系统

算法测试适合数值关系、限幅、异常输入、状态初始化和恢复；模块功能测试适合参数/uORB 等局部集成；仿真测试适合模式、任务和完整闭环。一次启动成功只覆盖启动路径。

当前位置控制测试在 [PositionControlTest.cpp](../src/modules/mc_pos_control/PositionControl/PositionControlTest.cpp)，命名规则见 [px4_add_gtest.cmake](../cmake/px4_add_gtest.cmake)。

### 10.2 跑已有位置控制测试

在 **容器 Bash、源码根目录**：

```bash
make tests TESTFILTER='^unit-PositionControl$' PX4_MAKE_ARGS=-j6
```

它配置 `px4_sitl_test` 并运行测试目标。筛选主要限制 CTest 的执行集合，不保证只编译一个测试，初次依赖准备可能较多。

配置完成后检查注册项并复跑：

```bash
ctest --test-dir build/px4_sitl_test -N
ctest --test-dir build/px4_sitl_test -R '^unit-PositionControl$' --output-on-failure --no-tests=error
```

应看到至少一个匹配测试且通过。`--no-tests=error` 避免把“零测试”解释为成功。修改后如果直接使用 CTest，必须先重建相应测试二进制。

### 10.3 用 GDB 调试单个测试

为了观察变量，可在 **容器 Bash**为测试配置 Debug，并只构建该目标：

```bash
cmake -S . -B build/px4_sitl_test -G Ninja -DCONFIG=px4_sitl_test -DCMAKE_BUILD_TYPE=Debug
cmake --build build/px4_sitl_test --target unit-PositionControl --parallel 6
gdb -nx --args build/px4_sitl_test/unit-PositionControl
```

如果该测试目录已有不同生成器或专门的分析配置，应先记录并处理配置差异，不直接复用不兼容缓存。在 GDB 中可 `break PositionControl::update` 后 `run`，查看输入、状态和限幅过程。测试可通过 GoogleTest 参数筛选具体用例，名称先由 `--gtest_list_tests` 查询。

### 10.4 什么测试值得写

新增或修改算法时，优先验证有独立意义的行为：饱和解除后积分恢复、无效输入不会生成错误有效输出、状态重置不造成不期望跳变、时间间隔变化受限、停止后不继续发布新数据等。

不要用同一段实现公式重新算一次“期望值”而没有独立依据。简单文档改动、常量练习不需要额外堆砌测试。

### 10.5 内存与未定义行为分析

当前 Makefile 提供 AddressSanitizer、UndefinedBehaviorSanitizer 等构建类型入口。ASan 适合越界和释放后访问，UBSan 适合部分未定义行为；它们不能代替线程/实时性分析，也不支持所有部署平台。

例如在已确认兼容的 **容器工具链**中，针对测试目标使用：

```bash
make tests TESTFILTER='^unit-PositionControl$' PX4_CMAKE_BUILD_TYPE=AddressSanitizer PX4_MAKE_ARGS=-j6
```

分析配置会影响 CMake cache 和产物。当前 [sanitizers.cmake](../cmake/sanitizers.cmake) 会把部分链接选项写入缓存，只改变构建类型可能留下旧选项。保存报告与自定义 CMake 配置后，在支持 `--fresh` 的 CMake 中刷新这一个测试目录，再重建所需目标：

```bash
cmake --fresh -S . -B build/px4_sitl_test -G Ninja -DCONFIG=px4_sitl_test -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build/px4_sitl_test --target unit-PositionControl --parallel 6
ctest --test-dir build/px4_sitl_test -R '^unit-PositionControl$' --output-on-failure --no-tests=error
```

`--fresh` 重置该目录的 CMake 配置状态；已有其他自定义选项时应一并重新传入。这里不清空整个工程。工具运行失败应先区分环境兼容和业务缺陷。

### 10.6 格式、覆盖率与 CI

对当前修改文件运行范围明确的检查，例如 **容器 Bash**：

```bash
Tools/astyle/check_code_style.sh src/examples/work_item/WorkItemExample.cpp
git diff --check
```

`make format` 可能修改更大范围。静态分析需要正确的编译数据库和生成头；报告应结合实际路径解释。

覆盖率帮助发现未经过的分支，不证明断言有效。当前 `make tests_coverage` 会先清理构建，不宜作为随手检查命令；需要覆盖率时使用独立记录的测试配置。`tests_integration` 走 Gazebo Classic/MAVSDK 链，与当前 Harmonic 依赖不同。

提交前按修改范围选择相关目标和测试，再参照 [.github/workflows](../.github/workflows) 的实际检查。个人分支的验证结果应由自己的运行记录说明。

<a id="performance"></a>
## 11. 性能、调度、内存与固件体积

### 11.1 分开测量三种时间

| 时间 | 回答的问题 | 典型证据 |
|---|---|---|
| 输入年龄 | 处理时数据已经多旧 | 当前时间与采样时间、发布时刻 |
| 执行间隔 | 模块多久运行一次，抖动多大 | interval 计数、时间戳序列、调度统计 |
| 执行耗时 | 一轮处理占用多久 | elapsed 计数、性能分析 |

一个算法只耗时 100 微秒，但等了 10 毫秒才被调度，闭环延迟仍然很大。这里的数值用于解释区别，并非本机测量结果。

### 11.2 使用已有性能计数

在 **PX4 Shell**：

```text
work_queue status
perf
perf latency
uorb top -1
```

`perf` 显示代码已有的计数器，不会自动给所有函数计时。查看调用次数、平均/最大耗时和间隔；先保留基线输出，再按实验需要使用 `perf reset` 开始新的窗口。

在支持的板端目标上，可结合 `top`、任务/栈统计与系统负载模块。不同平台支持的命令和统计含义不同，不能把 Linux 进程 CPU 与某个 PX4 工作项耗时直接等同。

### 11.3 新增性能计数应成对覆盖路径

参考 [WorkItemExample](../src/examples/work_item/WorkItemExample.cpp)：`PC_ELAPSED` 配合 `perf_begin/perf_end` 统计处理时间，`PC_INTERVAL` 配合 `perf_count` 统计执行间隔，生命周期结束时释放。

若中间提前返回漏掉 `perf_end`，统计可能失真。计数范围应明确：是整个 `Run()`、算法调用，还是 I/O 等待。分别计量才能区分计算慢和等待久。

### 11.4 内存和栈怎样检查

先区分静态数据、堆、任务栈、队列/消息缓冲和算法历史缓冲。新数组放在局部函数里可能增加线程栈，放在对象里则影响对象存储，增加主题队列会持续消耗内存。

在 **容器 Bash**可读取实际产物大小与符号，例如 FMUv6X 构建完成后：

```bash
arm-none-eabi-size build/px4_fmu-v6x_default/px4_fmu-v6x_default.elf
arm-none-eabi-nm --print-size --size-sort --radix=d build/px4_fmu-v6x_default/px4_fmu-v6x_default.elf | tail -30
```

结合 map 文件、段放置和板端运行统计分析。ELF 文件的磁盘大小可能包含调试信息，不等于 MCU 实际 Flash 占用；静态 RAM 也不能单独表示运行峰值。

### 11.5 性能实验的基本条件

使用同样的模型、时长、输入、日志配置和仿真速度，停止高频打印与断点影响。容器 CPU/内存限制与宿主负载也应记录。SITL 结果用于该软件环境，MCU 最坏耗时和总线时序需要板端测量。

<a id="integration"></a>
## 12. MAVLink、DDS/ROS 2 与 Offboard 集成调试

### 12.1 从传输层向行为层排查

```text
进程存在 → 设备或网络可达 → 正确端口/协议
 → 收到对应消息 → 内部主题更新
 → 字段、时间和质量有效 → 请求被接受
 → 实际模式与控制层启用 → 运动/任务结果符合预期
```

连接成功只证明这一链中的某一段。每层应留一份可以观测的证据。

### 12.2 MAVLink 的常见问题

先在 PX4 Shell 查 `mavlink status`，确认链路实例、模式、发送/接收情况和实际端口。本机默认实例的 GCS 链路通常是 PX4 本地 18570、对端 14550；Offboard 常见本地 14580、对端 14540。依据为 [px4-rc.mavlink](../ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink)，覆盖参数与多实例会改变安排。

在 **WSL 或容器 Bash**可用：

```bash
ss -lunp
```

它帮助检查 UDP socket，不证明应用已经解析消息。本机 lab 使用 host 网络，意味着共享 WSL Linux 网络；Windows QGC 的地址与可达性仍受 WSL 网络模式和防火墙影响。

任务/命令问题再检查 system/component ID、ACK 的 result、实际模式和任务结果。MAVLink 参数写入成功与模块实际应用参数也应分开观察。

### 12.3 DDS/ROS 2 怎样逐层检查

第一层是 PX4 中的 `uxrce_dds_client`，第二层是独立 Agent，第三层是 DDS 网络与外部节点，第四层是控制/观测契约。

在 **PX4 Shell**查询客户端状态；在 **已安装并加载相应 ROS 2 环境的终端**查看：

```bash
ros2 topic list
ros2 topic info /fmu/out/vehicle_status -v
```

主题名是常见基名；若启用了多实例命名空间或版本化命名，应从实际列表选取。消息存在但收不到时检查 QoS、Domain ID、消息版本、Agent 连接和话题映射；发布到输入主题后还需在 PX4 Shell 观察相应 uORB。

映射表是 [dds_topics.yaml](../src/modules/uxrce_dds_client/dds_topics.yaml)。ROS 2、Agent 和消息包是单独的环境组成，当前 PX4 构建镜像不等于已经安装它们。

### 12.4 Offboard 不执行时检查什么

| 现象 | 优先检查 |
|---|---|
| 请求被拒绝 | ACK 结果、当前估计有效性、控制层级、持续输入条件 |
| 显示 Offboard 但不按目标运动 | 目标字段、NaN、坐标、限幅、实际控制启用标志 |
| 很快退出 Offboard | 输入时间、新鲜度、应用线程/网络暂停、丢失保护配置 |
| 高度方向相反 | NED/ENU、局部原点和单位 |
| 偶发大跳变 | 估计重置、外部参考变化、时间戳和轨迹连续性 |
| 单机正常、多机异常 | system ID、端口、命名空间、模型和运行目录冲突 |

当前条件入口见 [offboardCheck.cpp](../src/modules/commander/HealthAndArmingChecks/checks/offboardCheck.cpp)。MAVLink 与 ROS 2 的活动/目标处理不同，不应机械共用一套保活实现。

记录外部程序的实际发送时间与 PX4 收到的数据时间。对外部算法线程设置断点时，可能人为造成丢失保护；这是实验干扰，需要与真实运行故障区分。

<a id="simulation"></a>
## 13. 仿真、故障注入、自动场景与 Replay

### 13.1 按问题选择仿真

SIH 适合快速学习控制链和状态机；Gazebo 适合场景、模型和模拟传感器；单元测试适合确定输入和精确边界。真机时序和电气问题最终仍要用硬件证据。

当前 **容器 Bash、源码根目录**的无图形 Gazebo 示例：

```bash
HEADLESS=1 make -j6 px4_sitl_default gz_x500 PX4_MAKE_ARGS=-j6
```

它需要当前源码所需的 Gazebo 依赖、模型和插件，且没有冲突实例。完整方案比较见 [ARCHITECTURE](ARCHITECTURE.md#simulation)。

### 13.2 故障注入不能只看命令返回

先在 **源码根目录 Bash**找到处理者：

```bash
rg -n 'VEHICLE_CMD_INJECT_FAILURE|FAILURE_UNIT_SENSOR_GPS' src/modules/simulation src/modules/commander
```

`failure gps off` 是否有效取决于正在使用的后端有没有实现对应请求。`simulator_mavlink` 的处理不能自动套到 SIH 或 gz_bridge。`SYS_FAILURE_EN` 等条件也需要核对。

设计故障实验时先记录原状态与恢复方式，再观察输入质量/速率确实变化，最后看估计、模式和输出响应。质量降低、停止发布和冻结值是三种不同实验。

### 13.3 自动化场景怎样判断通过

每个场景包含有界状态机：启动 → 等待明确就绪条件 → 操作 → 等待明确完成/失败条件 → 保存数据 → 正常退出。使用总超时和分阶段超时。

不要用固定等待时间或进程返回 0 代替行为判定。至少保存版本、参数、输入序列、关键状态、日志位置和失败原因。多实例要隔离参数、dataman、日志、端口和身份，不能只换 UDP 端口。

### 13.4 Replay 适合验证什么

Replay 使用记录输入重算支持的算法，适合比较同一批测量下的估计器变化。新的控制输出不会自动改变旧日志中的运动，因此它不能单独验证修改控制器后的闭环飞行轨迹。

普通 ULog 可能缺少回放需要的输入、时间信息或实例条件。EKF 专用流程应按当前实现核对单实例、`sensor_combined`、`ekf2_timestamps` 和实际辅助观测。

具体准备和运行步骤见 [OPERATIONS 的 Replay 实验](OPERATIONS.md#e4)。该节还说明了本分支 `rc.replay` 中旧 `ulog_params` 参数与当前 pyulog 版本的差异。照搬网络上的“一条 replay 命令”可能停在工具参数解析，而不是算法本身。

### 13.5 怎样比较回放结果

先不改算法和参数验证回放基线，再改变一个因素。对齐状态、创新、拒绝/融合标志、参考重置和有效性；检查是否残留上一轮的 `replay_params.txt` 或动态覆盖文件。

把原始日志与新产生日志分别保存。回放配置、源代码和参数覆盖都属于实验输入，不只记录 `.ulg` 文件名。

<a id="deployment"></a>
## 14. 实体飞控的固件、设备透传与部署

### 14.1 第一份硬件记录

在烧录前填写：准确产品和硬件修订、MCU、板 ID/bootloader、对应构建目标、供电方式、USB/调试口、电平、使用的传感器与输出。型号相似不代表固件可互换。

FMUv6X 只是本文的示例目标，不代表已经知道你实际使用的飞控型号。

### 14.2 编译并识别产物

**WSL 宿主、lab 根目录**：

```bash
./bin/lab run px4-main build px4_fmu-v6x_default
```

成功后应有固件包 `build/px4_fmu-v6x_default/px4_fmu-v6x_default.px4`；调试使用同次构建的 `.elf`，布局分析还可使用 map 文件。`.px4` 是打包产物，ELF 是符号和机器代码的重要依据。

需要 Debug 时，在 **容器源码根目录**显式构建对应目标：

```bash
make -j6 px4_fmu-v6x_default PX4_CMAKE_BUILD_TYPE=Debug PX4_MAKE_ARGS=-j6
```

Debug 可能改变体积和实时表现；硬件目标要检查容量，并在正常部署类型下再验证性能。

### 14.3 Windows、WSL、Docker 是三道设备边界

| 边界 | 检查内容 |
|---|---|
| Windows 识别实体设备 | 电缆、端口、驱动和设备身份 |
| WSL 获得设备 | 使用当前 usbipd/WSL 部署流程转交设备；重枚举后身份可能变化 |
| 容器可访问设备 | 设备映射、USB 总线节点、用户/组权限和驱动工具 |

宿主看得到 USB，不代表容器看得到；容器 host 网络也不等于共享所有 USB 设备。当前 lab 配方没有默认映射所有飞控和调试探针，应按实际设备补充配置。

在有设备的 **WSL/容器 Bash**分别检查实际节点，例如：

```bash
ls -l /dev/serial/by-id
ls -l /dev/bus/usb
id
```

某些系统没有这些目录属于需要继续定位的线索。优先使用可稳定识别的设备路径，避免多个串口时仅凭 `ttyACM0` 猜设备。QGC 或另一个终端占用串口，也会影响刷写和控制台连接。

### 14.4 刷写流程与验证

使用对应版本 QGC 的固件界面选择匹配的自定义 `.px4`，或者使用已核实的板级 upload 入口。**只有实际硬件确认为 FMUv6X、设备已透传且准备执行写入时**，容器中的示例为：

```bash
make px4_fmu-v6x_default upload
```

该命令会尝试写入设备，和普通 build 不同。具体重新连接/进入 bootloader 的时机依据上传器提示和板卡流程。

刷写后重新读取版本，核对传感器、airframe、参数、输出映射和日志。保存旧配置及对应版本，注意新固件可能有参数迁移；升级成功不代表配置天然正确。

<a id="hardware-debug"></a>
## 15. 板端控制台、SWD、GDB 与 HardFault

### 15.1 选哪种观察入口

| 入口 | 适合观察 | 前提与限制 |
|---|---|---|
| 系统启动 UART | bootloader/RTOS/PX4 早期输出 | 正确引脚、电平、波特率和供电；需上电前开始捕获 |
| MAVLink Console / Shell | 正常运行模块和系统命令 | PX4 和 MAVLink 链路已运行 |
| SWD/JTAG 与探针 | 早期启动、异常栈、寄存器、内存和断点 | 匹配架构、引脚、探针和调试服务器 |
| 示波器/逻辑分析仪 | 供电、总线、电气输出、时序 | 正确测量点、电平和参考地 |
| ULog/故障记录 | 运行历史与故障前后状态 | 记录功能启用且数据保存完整 |

MAVLink Shell 无法查看尚未建立 MAVLink 时的早期启动故障。仓库工具为 [Tools/mavlink_shell.py](../Tools/mavlink_shell.py)，用其 `--help` 核对串口/网络参数和 Python 依赖。

### 15.2 SWD 接线与运行条件

典型 SWD 包含 SWDIO、SWCLK、GND、目标电压参考，按需求还有复位线。具体引脚和电平按板卡文档；目标参考电压不应随意当作探针供电输出。

断点会停止控制或打乱系统时序。实体载具调试应使用动力受控的台架状态；涉及电机时拆桨，并明确 ESC、舵机和外部系统在处理器暂停后会怎样响应。

### 15.3 调试服务器和 GDB 的角色

```text
GDB 读取匹配 ELF / 源码
       ↕ GDB Remote Protocol
OpenOCD、J-Link Server 等
       ↕ USB / 探针接口
SWD / JTAG
       ↕
飞控 MCU
```

先用实际探针与 MCU 配置启动服务器，确认它识别目标，并明确服务器是否会复位或暂停 MCU。OpenOCD 配置、J-Link 的设备选择和端口各依环境不同，不能对全部板卡套一个芯片脚本。

服务器已监听本地 3333、实际目标与 ELF 确认匹配后，在 **能访问服务器的容器 Bash**示例：

```bash
gdb-multiarch -nx build/px4_fmu-v6x_default/px4_fmu-v6x_default.elf
```

也可使用已安装且支持该目标的 `arm-none-eabi-gdb`。在 **GDB**：

```text
target remote localhost:3333
info registers
bt
```

端口与 `localhost` 是示例，必须对应服务器真正的位置。先观察和保存证据，再决定是否复位、继续或写入；`load` 会写入目标，`monitor reset` 的行为依服务器实现，不是无副作用查询。

### 15.4 PX4/NuttX 调试辅助

[platforms/nuttx/Debug](../platforms/nuttx/Debug) 提供相应辅助。加载适合当前架构的脚本后，通过 `help px4`、`help nuttx` 或对应命令组查看任务、栈和寄存器工具。

`make 对应板型 debug` 的目标用于启动配置过的 GDB，不等于自动完成探针连接。[jlink.cmake](../platforms/nuttx/cmake/jlink.cmake) 中的 `jlink_debug_gdb` 明确包含刷写和调试，不能只根据名称中的 debug 就把它当作只读 attach。

### 15.5 HardFault 的证据链

先保存启动串口和运行条件。若目标启用了故障记录，重启后在 **板端 PX4 Shell**：

```text
hardfault_log check
```

若有未提交记录，再按当前帮助使用：

```text
hardfault_log commit
```

该操作将记录保存到相应故障文件并重新准备记录。`rearm` 会丢弃未提交记录，主动制造 fault 也不是初始排查步骤。

取得 PC、LR、寄存器、任务/栈等后，在 **容器 Bash**用匹配 ELF 解析地址：

```bash
arm-none-eabi-addr2line -e build/px4_fmu-v6x_default/px4_fmu-v6x_default.elf -f -C 0x08000000
```

`0x08000000` 仅表示地址格式，必须替换为故障中的真实地址。结合优化、内联和栈损坏解释结果；使用另一版 ELF 得到的行号不可信。

### 15.6 无故障日志的重启也需要调查

可能原因包括供电跌落、看门狗、复位线、早期异常、存储不可写等。没有 HardFault 文件不能排除故障，也不能自动认定是软件问题。结合复位原因、供电波形、启动阶段和重现条件定位。

<a id="extension"></a>
## 16. 开发不同功能时怎样安排实现和验证

### 16.1 新增运行模块

从现有模板建立类、入口、CMake 和 Kconfig，选择目标启用；先手动启动验证，再考虑加入启动脚本。确定任务/队列、触发频率、输入时效、状态、参数和输出契约。

至少检查：正常启动、初始化失败、无输入、输入恢复、停止、再次启动和资源释放。把算法与 uORB/平台包装适当分开，便于独立测试。完整示例见 [DEVELOPMENT](DEVELOPMENT.md#d9)。

### 16.2 新增消息或参数

消息定义需明确单位、坐标、采样/发布时间、无效值、队列/实例和版本。接通生产者、消费者、日志及需要的外部桥接；不要只看到生成头就算完成。

参数需定义范围、单位、默认值、重启要求和应用时机。测试更新后算法派生状态是否正确，恢复原值后行为能否恢复。当前生成物应通过源码输入生成，不直接编辑 `build/` 下文件。

### 16.3 修改控制或估计算法

明确数学约定与状态，针对正常、边界、失效和恢复输入写有意义的测试。用同一日志回放估计改动，再用闭环场景评估控制影响；最后按需进行硬件资源和动力验证。

记录改动对噪声、延迟、限制和状态连续性的影响。平均误差降低不一定代表最大偏差、稳定裕度或执行器饱和也改善。

### 16.4 新驱动和新板卡

驱动先完成识别、复位和静态读数，再加入正确采样时间、单位、方向、质量/错误计数、重试和恢复。验证顺序是设备/总线 → 原始主题 → 校准/选择 → 消费者/融合。

板卡还要完成 bootloader、RTOS、时钟、引脚、DMA、存储、电源、输出和打包。每次 bring-up 只引入可辨认的一部分，保留启动串口和板级测量证据。硬件结构与源码映射见 [ARCHITECTURE](ARCHITECTURE.md#hardware)。

<a id="triage"></a>
## 17. 常见故障速查

| 现象 | 首先收集的证据 | 常见原因或下一步 |
|---|---|---|
| lab 无法运行 | `lab info`、doctor 错误、Docker/数据盘状态 | 先处理环境层，不改源码 |
| 编辑器大量报红 | 容器路径、编译数据库、命令行构建结果 | 路径或目标不匹配、生成头缺失 |
| 找不到模块命令 | `.px4board`、Kconfig、CMake、实际版本 | 未编入或运行旧产物 |
| 模块启动后无输出 | 状态、输入实例/时间、提前返回条件 | 没有触发、无效输入或模式门控 |
| listener 一直是旧值 | 时间戳、发布活动、实例和进程身份 | 发布停止、连接旧进程或看错实例 |
| 修改参数没有效果 | 读回值、读取代码、更新与模式 | 应用时机、重启要求或限制掩盖 |
| GDB 断点灰色/不命中 | `info files`、符号、构建条目 | 文件或目标不一致、无执行路径 |
| attach 被拒绝 | PID/用户/namespace、实际错误 | ptrace、seccomp 或权限条件 |
| QGC 无法连接 | `mavlink status`、socket、WSL/Windows 地址 | 端口方向、网络、广播与防火墙 |
| ROS 2 看不到主题 | Agent、映射、Domain/QoS、版本与名称 | 桥接或外部环境问题 |
| Offboard 请求拒绝 | ACK、健康条件、输入时效与控制标志 | 状态不足、输入不满足或模式条件 |
| 控制振荡 | 目标/反馈、振动、延迟、饱和 | 从最早异常层查，避免只调增益 |
| 日志缺失或断续 | Logger 状态、记录集合、buffer/dropout | 未触发记录、主题没选中或吞吐不足 |
| 仅硬件不稳定 | 栈/负载、总线、供电、错误计数 | 资源或物理路径与 SITL 不同 |
| 板子反复重启 | 启动串口、复位原因、故障记录、供电 | 结合软件和电气证据定位 |

这个表是入口，不是自动诊断结论。每次修复应能说明证据如何排除其他解释。

<a id="evidence"></a>
## 18. 保存开发成果与建议练习顺序

### 18.1 一份可复现记录至少包含什么

```text
实验名称与要回答的问题
主仓库提交、工作区差异、子模块提交
lab 镜像/lock、目标、构建类型和产物标识
airframe、参数、模型/硬件身份与初始条件
操作步骤、输入和触发时刻
预期行为、实际行为、通过/失败判据
终端输出、ULog、测试报告或故障文件位置
结论、未覆盖条件、恢复与下一步
```

在 **源码根目录 Bash**审阅代码：

```bash
git status --short --branch
git diff --stat
git diff --check
git diff --submodule=log
```

新增未跟踪文件需要额外查看。选择本次相关的源码、配置与文档保存；大日志、缓存和固件按成果归档方式处理。保留能匹配故障的 ELF，避免重编译后失去唯一符号依据。

### 18.2 从简单到复杂的六次练习

| 次序 | 实验 | 完成标准 |
|---|---|---|
| 1 | 当前分支 build、smoke、状态查询 | 能说明运行版本、环境、目标和实际观察结果 |
| 2 | 第 4 章示例修改 | 能证明源码修改进入新产物并产生预期消息 |
| 3 | 第 6 章 GDB | 能解释线程/队列、调用栈、对象状态和触发路径 |
| 4 | 一条完整数据链与 ULog | 能把目标、反馈、模式和输出放到同一时间线上 |
| 5 | 一个算法边界或模式场景 | 有明确输入、判据、针对性测试和回归结果 |
| 6 | 外部应用或实体板专题 | 能定位接口/硬件边界，保存可重复的验证记录 |

完成每次练习后，用自己的话写出“改了什么、为什么会生效、如何证明”。当你能稳定回答这三点，再逐步扩大到新模块、复杂仿真、驱动或板卡移植。
