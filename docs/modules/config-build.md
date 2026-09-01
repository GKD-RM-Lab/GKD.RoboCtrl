# 配置、构建与验证

路径：`include/config/`、`src/main.cpp`、`xmake.lua`、`.github/workflows/`

配置层把“同一套驱动/控制代码”实例化为具体机器人。它只声明静态构造信息与子系统开关，不应在头文件中打开设备、启动任务或实现业务状态机。

## 车型配置

`xmake f --type=<infantry|hero|sentry|project>` 选择配置。四种配置都声明 CAN、串口、DJI 电机、ControlPad、IMU 和 Robot 参数；Project/Sentry 当前只启用底盘控制，Infantry/Hero 同时声明云台与发射。

底盘配置还声明云台跟随 PID、方向与 2 ms 周期；Infantry/Hero 的 Gimbal 声明 IMU/电机 key、角度 PID 和 1 ms 周期，Shoot 声明摩擦轮斜坡、最大速度与车型相关的拨弹速度。配置预检会验证这些 key、方向、范围和周期。

| 类型 | CAN | 串口设备 | 电机集合 | 启用控制 |
| --- | --- | --- | --- | --- |
| `infantry` | `can0`、`can1` | `/dev/IMU_HERO` | 4 底盘 + 2 云台 + 2 摩擦轮 + 1 拨弹 | chassis、gimbal、shoot |
| `hero` | `CAN_CHASSIS`、`CAN_GIMBAL` | `/dev/IMU_HERO` | 4 底盘 + 2 云台 + 2 摩擦轮 + 1 拨弹 | chassis、gimbal、shoot |
| `sentry` | `CAN_CHASSIS` | `/dev/IMU_SENTRY` | 4 底盘 | chassis |
| `project` | `CAN_CHASSIS` | `/dev/IMU_HERO` | 4 底盘 | chassis |

这些名称是当前源码配置，不保证部署机器已经创建同名 SocketCAN 接口或串口软链接。修改任何名称时需全仓检索引用，并在目标主机上独立验证 udev/网络配置。

启动前 `validate_configuration()` 检查：

- 各类 key 非空且唯一；
- 电机、IMU、ControlPad 引用的 CAN/串口存在；
- DJI ID、半径、控制周期有效；
- 反馈 ID 和发送指令槽不冲突；
- 已启用控制子系统所需的语义化电机名存在。

校验发生在硬件构造前，目的是让静态配置错误在无硬件测试和 CI 中就失败。它不能验证设备文件是否存在、CAN 是否 up、接线方向、PID 稳定性或实车安全。

新增配置项时，应同时完成三件事：为所有四个 `config.<type>.hpp` 给出明确值或禁用策略；在 `validate.hpp` 增加跨字段约束；在 `tests/unit_tests.cpp` 增加至少一个能覆盖新约束的测试。

## 主程序参数与运行

目标名为 `gkd-roboctrl`，配置后产物 basename 为 `gkd.roboctrl.<type>`。命令行参数：

- `-h, --help`：打印帮助。
- `-l, --log <debug|info|warn|error>`：设置最低日志等级。
- `-f, --filter <text>`：设置日志 role 过滤字符串。

`start.sh` 只是 `xmake run gkd-roboctrl` 的参数转发包装。主程序启动会立即打开配置中的 CAN 和串口，事件循环开始后电机组持续发送帧；未经授权不要用它做“验证命令”。

## 构建

```sh
xmake f -y -m debug --type=project
xmake build -y gkd-roboctrl
xmake build -y unit-tests
xmake run unit-tests
```

xmake 强制 LLVM、C++23 和 libc++，依赖 `asio`、`cxxopts`，主程序链接 pthread。`unit-tests` 只编译 `tests/unit_tests.cpp` 与 `src/device/base.cpp`，不打开 CAN/串口。切换车型前必须重新执行 `xmake f`，否则可能仍在验证上一次缓存的 `BUILD_TYPE`。

若变更公共模板、配置结构、concept 或跨车型语义，至少执行：

```sh
for type in infantry hero sentry project; do
  xmake f -y -m debug --type="$type"
  xmake build -y gkd-roboctrl unit-tests
  xmake run unit-tests
done
```

Release 相关、编译器优化敏感或准备合并的改动还应重复 `-m release`。不要用循环运行主程序。

`.github/workflows/build-test.yml` 的目标是在 Ubuntu 上对四种车型分别构建 Debug/Release，并运行单元测试。但当前 diff 把通用选项写在目标名之后（`xmake build gkd-roboctrl -y`），xmake 3.1.1 会把 `-y` 当作目标名；在改成 `xmake build -y gkd-roboctrl` 等正确顺序前，不能把该工作流标记为已验证可用。文档部署监听默认分支 `master`。

Doxygen 使用根目录 `Doxyfile`、`DoxygenLayout.xml` 和 `mainpage.dox`，把生成结果写入 `docs/html/`；该生成目录被忽略，`docs/*.md` 与 `docs/modules/*.md` 则进入版本控制。API 文档和 Agent 架构文档用途互补，公共 API 注释和对应模块文档都要随行为更新。

主程序是 Linux SocketCAN/串口程序。无明确硬件环境和授权时只构建及运行单元测试，不运行 `gkd-roboctrl`。当前结果只能标记为“已构建/已测试”，不能替代实车或硬件在环验证。

## 验证结果分级

- **静态检查**：配置预检、Markdown 链接、`git diff --check`。
- **单元测试**：无硬件纯逻辑与类型抽象。
- **Linux 构建**：证明目标平台可编译链接，不证明设备存在。
- **硬件在环/台架**：证明协议、ID、方向、单位和时序符合具体硬件。
- **实车验证**：在安全场地验证整机状态机与故障行为。

交付说明必须明确实际完成到哪一级，不把低一级结果写成更高一级结论。
