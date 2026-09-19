# 本次迁移验证记录

环境：macOS arm64，Clang 20.1.8，C++23；来源主提交 `cbe8a8a4d419fa5e87cfe00c8cb525ef748356cc`。本记录只描述本次实际软件证据。

## 已执行

| 检查 | 结果与范围 |
| --- | --- |
| 无硬件 aggregate | **PASS**；原 unit-tests 与新增配置、驱动/网络/裁判 codec、真实 shoot/gimbal 协程配模拟设备、功率分配/最终电流门、弹道测试一起链接运行 |
| 配置 YAML/JSON | **PASS**；infantry/hero/project/M9025 示例通过；严格未知字段、JSON roundtrip、引用/方向/周期/驱动能力/物理别名/协议槽/裁判依赖等负例被拒绝 |
| 哨兵来源拓扑 | **预期拒绝**；结构可解析，J6006 命令 0x201 与 DJI 拨弹反馈冲突；只在测试副本使用合成独立总线验证余下拓扑，不改部署 ID |
| 协议和控制边界 | **PASS**；CRC golden vectors/所有拆分点/重同步、IMU换算溢出、云台回中/NoForce/故障/失联、非电流电机失效禁用、发射裁判/弹种/精确速度阈值、主副执行器实例隔离 |
| 功率 | **PASS**；2000 组确定随机输入、转子/输出轴模型等价、能量/RLS、零裁判预算、后续 PID 输出不能突破安装的绝对上限 |
| UDP localhost | **PASS**；双向视觉/导航、来源/header分流、目标过期、日志样本、callback临时对象销毁/挂起后的缓冲所有权；单独 UBSan 运行通过 |
| 弹道与 Python 工具 | **PASS**；数值/失败路径、CLI 样例、Python 4 组测试、实际 C++ logger 编码到 Python 解码、读取日志 SHA 保持不变；生成 HTML 的 JS 静态语法通过 |
| C++ 语法检查 | **PASS**；`src/` 中除 `src/io/can.cpp` 外的全部 27 个翻译单元，包括 main.cpp；不等于链接通过 |
| 文档/差异 | 相对 Markdown 链接、文档覆盖、`git diff --check` 已检查；旧源 J6006 两文件及 gimbal_sentry.hpp SHA 与开工快照相同 |

期望负例会打印 `unsupported current control mode`、注入 writer 错误和注入异步异常日志，但测试返回 0；不把该预期日志当作运行失败。

## 直接测试复现

用户要求不执行 xmake 后已停止其配置过程，未再调用 xmake。使用固定依赖的临时副本进行无硬件编译：Asio 1.36.0、reflect-cpp 0.25.0、yaml-cpp 0.8.0；只把依赖编译到 `/tmp`，没有安装或替换项目依赖。

```sh
python3 -B tools/run_software_tests.py \
  --asio-include /tmp/gkd-review-4pJ1uU/asio-asio-1-36-0/asio/include \
  --reflect-include /tmp/reflect-cpp-0.25.0/include \
  --yaml-include /tmp/yaml-cpp-0.8.0/include \
  --reflect-library /tmp/gkd-reflect-build/libreflectcpp.a \
  --yaml-library /tmp/gkd-yaml-build/libyaml-cpp.a
```

实际输出包括 `C++ hardware-free aggregate: PASS`、`Ran 4 tests ... OK`、`Python tuning tools: PASS`。临时路径仅对应本次环境，复现时替换为自己的依赖位置。其他分项测试和 localhost 命令见各模块文档；网络测试只绑定本机 UDP，需要允许本机 socket。

## 未执行/未通过验证的范围

- 未执行完整 Linux 目标构建、链接或 CI。缺少 Linux CAN 头文件；遵从用户指示没有用假 CAN 头文件掩盖此限制。
- 未运行 `gkd-roboctrl`、`start.sh`、真实 CAN/串口、台架或实车。
- ASan 进程在环境中无输出挂起后被中止，不能报告 ASan 通过；这不替代一次可用环境下的内存检查。
- HTML 浏览器打开超时，未验证视觉渲染与交互。
- 哨兵 CAN 映射、J6006 机械零位、M9025 固件单位、IMU上游单位/方向、PID、功率模型、实际网络对端、裁判版本仍需硬件/部署核实。
- 原目录并行修改已提交为 `e6a71f4`，在用户授权后与迁移提交整合。[开工基线](migration-baseline.json) 与 [后续差异快照](migration-upstream-drift.json) 保留历史来源。未推送远端。

## 与主分支安全修复整合

迁移提交 `cf2c037` 与主分支 `e6a71f4` 合并，保留异步回调/typed IO/写队列所有权、异常停机、遥控校验和重新解锁边沿、DJI 型号电流上限、Matrix/RLS 修复。停机零输出覆盖扩展到 J6006/M9025/超容；CAN 队列按 ID+DLC 合并，避免 J6006 的 DLC4 速度覆盖 DLC8 使能命令。无硬件队列测试覆盖积压时保留两类帧且失能替换旧使能；SocketCAN 实现仍因平台限制未编译。

合并后的 aggregate/Python 测试、localhost UDP/UBSan、27 个可检查翻译单元语法检查、81 个相对文档链接与 `git diff --check` 均重新执行并通过。没有执行 xmake。
