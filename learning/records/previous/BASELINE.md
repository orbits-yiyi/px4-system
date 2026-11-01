# 学习基线

[总入口](../../README.md) · [环境手册](../../OPERATIONS.md) · [操作](../../OPERATIONS.md)

## 当前已知信息

| 项目 | 已知值 |
|---|---|
| 学习版本 | PX4 v1.17.0 |
| 基线提交 | d6f12ad1c4f70ad3230afd7d86e971421e02fef4 |
| 本地分支 | learn/px4-v1.17.0 |
| 官方远程 | origin → https://github.com/PX4/PX4-Autopilot.git |
| 个人仓库 | https://github.com/orbits-yiyi/px4-system.git，待按 REMOTE 核对接入 |
| WSL 源码 | /home/yiyi/Workspace/px4/PX4-Autopilot |
| lab 任务 | px4-main；名称不等于源码分支 |
| 容器路径 | /workspaces/PX4-Autopilot |
| 工具环境仓库 | ~/embedded-lab，独立于本仓库 |
| 基础镜像 | ghcr.io/px4/px4-dev:v1.18.0-beta2，摘要见 embedded-lab 配方/任务锁 |
| 已验证能力 | v1.17.0 基础构建、SIH 四旋翼完整起降；Gazebo 未验收 |

基础镜像标签与源码版本独立；是否兼容由实际构建和测试记录说明。当前源码补上课程提交后，git describe 会出现 v1.17.0-N-g...，保留基线祖先关系即可。

## 每次阶段验收保存的信息

在 WSL 项目根目录执行：

```bash
git rev-parse HEAD
git status --short --branch
git submodule status --recursive
~/embedded-lab/bin/lab info px4-main
~/embedded-lab/bin/lab snapshot px4-main
```

`snapshot` 会启动临时容器采集实际工具版本，结果位置由 lab 输出；把镜像摘要、环境锁标识和环境仓库提交记录到实验文档，不提交凭据或完整缓存。

## 复现实验需要区分的状态

- 源码状态：PX4 提交、分支、修改及各子模块提交。
- 工具状态：基础/开发镜像、embedded-lab 配方、任务是否锁定。
- 实验状态：机型、仿真器、初始参数、任务和日志时间段。
- 硬件状态：板版本、传感器、接线、校准和设备固件。

大日志、镜像和编译缓存留在本地状态目录；学习仓库保存必要的摘要、参数差异、图表及复现依据。
