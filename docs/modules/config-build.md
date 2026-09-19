# 配置、构建与验证

路径：`configs/`、`include/config/runtime.hpp`、`include/config/validate.hpp`、`src/main.cpp`、`xmake.lua`、`.github/workflows/`

配置层把“同一套驱动/控制代码”实例化为具体机器人。运行时配置只描述构造信息、对象连接和子系统开关；它不打开设备、不启动任务，也不实现业务状态机。

## 车型配置

车型完全在运行时选择，不再通过 xmake 编译参数生成不同二进制。未指定 `--config` 时使用 `configs/infantry.yaml`；通过 `--config configs/<type>.yaml` 可选择 `hero`、`sentry`、`project`，也可传入自定义 YAML/JSON。四份 YAML 都直接组合 CAN、串口、DJI 电机、ControlPad、IMU 和 Robot 的既有 `info_type`；Project 当前只启用底盘控制，Infantry/Hero/Sentry 同时声明云台与发射。Sentry 配置还保留旧工程副云台电机拓扑，但当前运行时只绑定一个主云台实例。车型参数只维护在运行时 YAML 中，避免同一配置存在两份来源。

底盘配置声明四个电机 key、控制周期和底盘最高旋转速度 `max_rotate_speed`；Infantry/Hero/Sentry 的 Gimbal 声明 IMU/电机 key、角度 PID 和 1 ms 周期，Shoot 声明摩擦轮斜坡、最大速度与车型相关的拨弹速度。配置预检会验证这些 key、方向、范围、有限浮点值、PID 输出界限、设备路径和周期。

| 类型 | CAN | 串口设备 | 电机集合 | 启用控制 |
| --- | --- | --- | --- | --- |
| `infantry` | `can0`、`can1` | `/dev/IMU_HERO` | 4 底盘 + 2 云台 + 2 摩擦轮 + 1 拨弹 | chassis、gimbal、shoot |
| `hero` | `CAN_CHASSIS`、`CAN_GIMBAL` | `/dev/IMU_HERO` | 4 底盘 + 2 云台 + 2 摩擦轮 + 1 拨弹 | chassis、gimbal、shoot |
| `sentry` | `CAN_CHASSIS`、`CAN_BULLET`、`CAN_GIMBAL` | `/dev/IMU_BIG_YAW`、`/dev/IMU_SMALL_YAW` | 4 底盘 + 4 主/副云台 + 2 摩擦轮 + 1 拨弹 | chassis、gimbal、shoot |
| `project` | `CAN_CHASSIS` | `/dev/IMU_HERO` | 4 底盘 | chassis |

这些名称是当前源码配置，不保证部署机器已经创建同名 SocketCAN 接口或串口软链接。修改任何名称时需全仓检索引用，并在目标主机上独立验证 udev/网络配置。

## 运行时 YAML / JSON

`runtime_config` 不再维护 `MotorSpec`、`GimbalSpec` 等平行结构；根对象直接持有 `std::vector<io::can::info_type>`、`std::vector<io::serial::info_type>`、`std::vector<device::dji_motor::info_type>` 以及 ControlPad、IMU、Robot 的原始 `info_type`。因此 YAML 字段名就是头文件中的成员名，例如电机使用 `type_`、`can_name`、`pid_params`、`control_time`。

```yaml
schema_version: 1
profile: infantry
cans:
  - {name: can0, interface_name: can0}
serials:
  - {name: serial1, device: /dev/IMU_HERO, baud_rate: 115200}
dji_motors:
  - {type_: M6020, id: 1, name: gimbal_yaw_motor, can_name: can0,
     radius: 1.0, pid_params: {kp: 8000.0, ki: 0.0, kd: 0.0, max_out: 20000.0, max_iout: 5000.0},
     control_time: {count: 2, unit: milliseconds}}
```

`std::chrono::steady_clock::duration` 必须写成 `{count: <整数>, unit: milliseconds}`。DJI 型号使用既有枚举名 `M2006`、`M3508` 或 `M6020`。加载时使用严格的未知字段检查；可选/关闭子系统可依赖 `info_type` 的默认成员省略字段，但所有启用子系统和跨对象 key 都必须通过随后语义校验。JSON 具有相同结构。

每次启动都会先完整打印所选配置文件父目录下的 YAML/JSON 文件，再加载 `--config` 指定文件或默认文件。打印仅用于核对实际参数；文件中不得存放口令、令牌或其他秘密。

`robot.chassis_type` 与 `robot.gimbal_type` 用字符串声明预期的具体设备类型。配置层只检查启用时字符串非空，不维护类型白名单；启动时由对应注册表工厂查找并创建抽象底盘/云台实例。未知类型由工厂返回失败并终止初始化。

启动前 `validate_configuration()` 检查：

- 各类 key 非空且唯一；
- `interface_name` 和串口 `device` 配置字符串各自唯一，禁止两个逻辑对象直接复用同一字符串；路径别名或符号链接是否指向同一物理设备仍需在目标主机检查；
- 电机、IMU、ControlPad 引用的 CAN/串口存在；
- DJI 型号仅限 `M2006`、`M3508`、`M6020`，且 ID、型号级命令上限、半径、控制周期有效；
- 按物理 CAN 接口计算的反馈 ID 和发送指令槽不冲突；
- 已启用控制子系统所需的语义化电机名存在，且一个电机不能同时承担两个底盘、云台或发射执行器角色。

校验发生在硬件构造前，目的是让静态配置错误在无硬件测试和 CI 中就失败。它不能验证设备文件是否存在、CAN 是否 up、接线方向、PID 稳定性或实车安全。

新增配置项时，应同时完成三件事：为所有受影响的 `configs/<type>.yaml` 给出明确值或禁用策略；在 `validate.hpp` 增加跨字段约束；在 `tests/unit_tests.cpp` 增加至少一个能覆盖新约束的测试。

## 主程序参数与运行

目标名为 `gkd-roboctrl`，产物 basename 固定为 `gkd-roboctrl`。命令行参数：

- `-h, --help`：打印帮助。
- `-l, --log <debug|info|warn|error>`：设置最低日志等级。
- `-f, --filter <text>`：设置日志 role 过滤字符串。
- `-c, --config <path>`：加载指定的 YAML 或 JSON 文件；省略时加载 `configs/infantry.yaml`。

`start.sh` 只是 `xmake run gkd-roboctrl` 的参数转发包装。主程序启动会立即打开配置中的 CAN 和串口，事件循环开始后电机组持续发送帧；未经授权不要用它做“验证命令”。

## 构建

```sh
xmake f -y -m debug
xmake build -y gkd-roboctrl
xmake build -y unit-tests
xmake run unit-tests
```

xmake 强制 LLVM、C++23 和 libc++，依赖 `asio`、`cxxopts`、启用 YAML 的 `reflect-cpp`，主程序链接 pthread。车型不参与编译，运行时通过 `--config` 选择；`unit-tests` 除测试入口外只链接 Core 异步/日志、发射互锁纯逻辑和设备基类等无硬件源码，不打开 CAN 或串口。

若变更公共模板、配置结构、concept 或跨车型语义，至少执行：

```sh
xmake f -y -m debug
xmake build -y gkd-roboctrl unit-tests
xmake run unit-tests
```

Release 相关、编译器优化敏感或准备合并的改动还应重复 `-m release`。不要用循环运行主程序。直接依赖固定为 Asio 1.36.0、cxxopts v3.3.1、reflect-cpp v0.25.0，CI 固定 xmake 3.1.1。

英雄配置的 M3508 云台俯仰电机命令上限已从 30000 修正为该型号允许的 16384；该变化会限制原先越界的输出，仍需在台架重新确认控制响应。

`.github/workflows/build-test.yml` 在 Ubuntu 上分别构建 Debug/Release，并运行单元测试；车型配置作为独立运行时数据，不再形成构建矩阵。工作流中的 xmake 通用选项统一放在目标名之前（例如 `xmake build -y gkd-roboctrl`），兼容当前使用的 xmake 3.1.1。文档部署工作流监听默认分支 `master`。

Doxygen 使用根目录 `Doxyfile`、`DoxygenLayout.xml` 和 `mainpage.dox`，把 `include/`、`src/` 以及 `docs/` 中的 API/架构文档统一生成到 `docs/html/`；`docs/html/` 被忽略，`docs/*.md` 与 `docs/modules/*.md` 则进入版本控制。生成首页由 `mainpage.dox` 提供，包含构建命令、生命周期和模块导航；API 文档和 Agent 架构文档用途互补，公共 API 注释和对应模块文档都要随行为更新。

本地生成 API 文档需要 Doxygen、Graphviz 和已初始化的 `doxygen-awesome-css` 子模块：

```sh
git submodule update --init --recursive
doxygen Doxyfile
```

GitHub Actions 的 `doxygen-pages.yml` 会在 `master` 上自动生成并部署 GitHub Pages；文档生成不需要访问 CAN、串口或其他机器人硬件。

主程序是 Linux SocketCAN/串口程序。无明确硬件环境和授权时只构建及运行单元测试，不运行 `gkd-roboctrl`。当前结果只能标记为“已构建/已测试”，不能替代实车或硬件在环验证。

## 验证结果分级

- **静态检查**：配置预检、Markdown 链接、`git diff --check`。
- **单元测试**：无硬件纯逻辑与类型抽象。
- **Linux 构建**：证明目标平台可编译链接，不证明设备存在。
- **硬件在环/台架**：证明协议、ID、方向、单位和时序符合具体硬件。
- **实车验证**：在安全场地验证整机状态机与故障行为。

交付说明必须明确实际完成到哪一级，不把低一级结果写成更高一级结论。
