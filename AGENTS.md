# GKD.Roboctrl Agent 指南

本文件适用于整个仓库。GKD.Roboctrl 是面向 RoboMaster 机器人的 Linux 电控程序，使用 C++23、Asio 协程和 xmake，将通信、设备驱动与机器人控制分层组织。

## 修改代码前必须阅读

1. 先阅读 [`docs/README.md`](docs/README.md)，由文档索引定位任务涉及的模块。
2. 阅读 [`docs/architecture.md`](docs/architecture.md) 和对应的 `docs/modules/*.md`。涉及跨层调用、初始化、公共接口或配置时，同时阅读 [`docs/development.md`](docs/development.md)。
3. 再检查真实源码、调用点和当前 `git diff`。源码是行为事实，文档是设计约束；若二者不一致，先在任务中指出，再让代码与文档恢复一致。
4. 不要把“已声明”视为“已实现”。先查看模块文档中的成熟度标记，并检查 `.cpp`、初始化入口和实际调用链。

## 修改代码后必须更新文档

代码改动与相关文档更新必须在同一次变更中完成。至少遵守以下对应关系：

| 代码变更 | 必须检查并更新 |
| --- | --- |
| 模块职责、依赖方向、初始化顺序 | `docs/architecture.md`、对应模块文档 |
| 公共类、接口、协议、报文、设备行为 | 对应的 `docs/modules/*.md` |
| 机器人类型、设备名、CAN ID、串口或 PID 参数 | `docs/modules/config-build.md` 及相关设备/控制文档 |
| 构建命令、依赖、运行参数、验证方式 | `docs/modules/config-build.md`、`docs/development.md` |
| 新增/删除/重命名模块或文档 | `docs/README.md`、本文件中的文档导航 |
| 完成 TODO、引入限制或改变成熟度 | 对应模块文档的“当前状态/限制”部分 |

仅改注释或纯格式且不改变语义时，可以不改模块文档；但提交前仍要确认文档没有因此失真。

## 架构约束

- 依赖方向保持为 `utils/core → io → device → ctrl`，`config` 只组装具体实例，`main` 只负责参数、初始化和启动事件循环。避免底层模块反向依赖业务控制层。
- 项目采用单线程异步模型。长任务必须通过协程主动让出执行权，禁止在事件循环中加入阻塞等待或无让出的死循环。
- IO 层处理字节传输与分发；设备层负责协议解释、物理量和在线状态；控制层只表达机器人行为。不要把报文解析散落到控制层。
- 多例对象必须定义合规的 `info_type`（`owner_type`、`key_type`、`key()`）并由依赖顺序初始化；全局唯一服务使用 `singleton_base` 和 `init(info)`。
- 报文直接转结构体只适用于平凡可复制类型，并且必须明确长度、字节序、对齐和协议来源。外部输入不能只依赖 `assert` 做运行时校验。
- 配置中的字符串 key 是对象连接关系的一部分。重命名 CAN、串口或电机时，必须全仓检索并同步所有引用。
- 保留用户已有的未提交修改。不要顺手修复无关问题，也不要把当前骨架模块描述成生产可用。

## 验证要求

- 文档变更至少检查 Markdown 链接、文件覆盖和 `git diff --check`。
- C++ 或构建配置变更应运行与目标机器人类型对应的 xmake 配置和构建；命令见 [`docs/modules/config-build.md`](docs/modules/config-build.md)。
- 本项目直接运行会访问 SocketCAN、串口并可能下发电机命令。没有明确的硬件环境和用户授权时，只构建，不运行二进制。
- 无硬件逻辑优先运行 `unit-tests`，但它不能覆盖 Linux 设备、总线时序和真实执行器。若无法完成硬件验证，明确记录“已构建/已测试但未上机”，不要声称实车功能已验证。

## 文档导航

- [`docs/README.md`](docs/README.md)：Agent 阅读入口和文档索引
- [`docs/architecture.md`](docs/architecture.md)：总体架构、生命周期和数据流
- [`docs/modules/core.md`](docs/modules/core.md)：异步上下文、多例/单例、日志
- [`docs/modules/io.md`](docs/modules/io.md)：IO 抽象及 CAN/串口/UDP/TCP
- [`docs/modules/device.md`](docs/modules/device.md)：设备、IMU、电机、遥控器、超级电容
- [`docs/modules/control.md`](docs/modules/control.md)：机器人、底盘、云台、发射和功率控制
- [`docs/modules/config-build.md`](docs/modules/config-build.md)：机器人配置、构建、运行和部署边界
- [`docs/modules/utils.md`](docs/modules/utils.md)：控制器、回调、数学与通用工具
- [`docs/development.md`](docs/development.md)：修改流程、扩展模板和文档维护规则
