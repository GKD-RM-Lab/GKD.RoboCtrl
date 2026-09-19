# 配置、构建与验证

配置直接组合组件的拥有字符串 `info_type`，通过 reflect-cpp v0.25.0 严格解析 YAML/JSON；无平行 `*Spec`。根字段除了原 CAN/Serial/DJI/ControlPad/IMU/Robot，还包含 `j6006_motors`、`m9025_motors`、`additional_imus`、`udp_servers`、`aim_links`、`navigation_links`、`remote_loggers` 和可选 `referee`、`super_cap`、`power`。

## 配置与状态

| 文件 | 实际软件连接 | 当前边界 |
| --- | --- | --- |
| `configs/infantry.yaml` | 四轮、DJI 双轴、17 mm 发射、视觉、裁判/UI、超容/功率、日志 | 参数由旧源码迁移，未上机 |
| `configs/hero.yaml` | 四轮、DJI 双轴、42 mm 发射、视觉、裁判/UI、超容/功率、日志 | 同上 |
| `configs/sentry.yaml` | J6006 大 yaw、DJI 小头、两 IMU、17 mm 发射、导航/搜索、裁判/超容/功率、日志 | **结构解析成功但语义拒绝**：来源中 J6006 命令和拨弹反馈冲突；用户要求暂不处理硬件映射。零位也未确认 |
| `configs/project.yaml` | 四轮和功率策略、遥控；不启用云台/发射 | 未上机 |
| `configs/examples/m9025-bench.yaml` | 独立 M9025 单轴接入示例 | 不是哨兵硬件替代；速度缩放为显式旧标签假设，零位未标定且默认零增益 |

这些路径、CAN 名称和 `/dev/REFEREE` 来自旧源码，并不保证部署机器上存在。UDP 默认仅监听 `127.0.0.1:11451`，视觉对端 `11453`，导航 `11456`，日志 `8080`。接收源 IP/端口必须匹配配置；跨机器使用必须修改双方配置。

`robot.gimbal_info` 为主小头；可选 `large_yaw_info` 是哨兵 yaw-only，`secondary_gimbal_info` 与 `secondary_shoot_info` 支持第二小头及独立发射器。主/副发射只接受对应头的许可，重复绑定电机被预检拒绝。实际旧哨兵只有一套已知共享硬件，因此不虚构第二发射器。

## 参数与语义预检

持续时间格式 `{count: 1, unit: milliseconds}`；DJI 枚举名为 M2006/M3508/M6020；视觉格式 `legacy_14` 或明确选用 `compact_10`。未知字段失败，可选组件默认缺省。电机 ID、半径、DJI 类型/周期和串口波特率缺失时初始化为无效零值，由预检拒绝，避免读取未初始化数值。

硬件打开前校验 key、物理 CAN/串口别名、串口 raw/keyed 模式、反馈/命令槽、跨驱动 CAN 冲突、驱动能力、控制对象唯一所有权、IMU 引用、协议 header/端点、有效超时、所有相关方向/有限数/PID、裁判发射依赖和功率配置。J6006 允许同一 master ID 下不同控制器复用反馈，禁止同总线重复控制器 ID。

`yaw_zero` 单位 rad，`yaw_zero_calibrated` 决定可否机械回中，未标定不能宣告 FinishInit 完成。`yaw_angle_direction` 与 `yaw_recenter_direction` 分别保留旧绝对/相对外环符号；输出方向另行配置。IMU 姿态和 rate 各轴符号独立，迁移值保留 pitch 取负。轮方向 LF/RF/LR/RR 明确 `[-1,1,-1,1]`。摩擦轮 m/s、拨弹 rad/s。

秒制 PID 使用 `ki_new=ki_old/dt0`、`kd_new=kd_old*dt0`：底盘2ms、云台/发射1ms。固定周期代数等价不代表硬件稳定。四车型 `power` 默认35W、boost0、RLS false是保守软件配置，非额定功率声明。裁判门开启时失联/非比赛阶段禁止发射；热量/弹速闭环未实现。全部标定边界见 [迁移记录](../migration-gkd-control.md)。

## 程序与调试入口

主程序保留 `--help/--log/--filter/--config`；新增 `--check-config` 在构造任何 IO 前校验并退出。独立 `config-check` 也只校验文件，更适合无硬件环境。普通主程序会打开总线/串口并发送命令，未经硬件授权禁止运行 `gkd-roboctrl` 或 `start.sh`。

新增 `ballistics-cli` 是独立离线数值入口，Python 日志接收/绘图见 [弹道与工具](ballistics-tools.md)。不再将旧无限调参任务放到正常初始化中。

## 构建目标

项目仍是 LLVM/C++23/libc++、Asio、cxxopts、reflect-cpp(YAML)。`gkd-roboctrl` 使用 `src/**.cpp`，依赖 Linux SocketCAN。`unit-tests` 汇总协议、配置、模拟执行器、功率和弹道测试，不打开真实设备，并显式 `-UNDEBUG` 保留 Release 断言。`network-transport-tests` 只使用 localhost UDP。`config-check`、`ballistics-cli` 不初始化硬件。

目标 Linux 环境的常规命令（本次未执行）：

```sh
xmake f -y -m debug
xmake build -y gkd-roboctrl unit-tests config-check ballistics-cli network-transport-tests
xmake run unit-tests
xmake run network-transport-tests
xmake run config-check configs/infantry.yaml configs/hero.yaml configs/project.yaml
```

哨兵文件预期非零退出，不应将它加入“全部配置均可部署”的断言。CI/Linux完整链接仍需要后续执行；修改 xmake 目标声明本身不代表构建通过。

用户本次明确要求不调用 xmake 后，使用直接 Clang 的无硬件测试。可复用以下脚本传入已存在依赖（不下载依赖、不构建机器人主程序）：

```sh
python3 tools/run_software_tests.py \
  --asio-include /path/to/asio/include \
  --reflect-include /path/to/reflect-cpp/include \
  --yaml-include /path/to/yaml-cpp/include \
  --reflect-library /path/to/libreflectcpp.a \
  --yaml-library /path/to/libyaml-cpp.a
```

## 验证层级

配置解析/预检、协议单测、模拟控制集成、localhost通信、源码语法、完整Linux链接、台架和实车是不同证据。本次没有后面三项。图表浏览器与 ASan 也未验证，不用静态检查冒充通过。实际执行记录见 [验证清单](../migration-verification.md)。

Doxygen/Graphviz 的生成与部署配置保持既有方式：`doxygen Doxyfile` 输出 `docs/html/`，不提交生成目录。新增模块已加入文档索引。
