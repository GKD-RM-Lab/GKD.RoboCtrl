# GKD_Control 剩余迁移记录

来源 `/Users/junity/code/GKD_Control` 只读；目标是本独立工作树。既有用户变更先完整带入，摘要与 SHA256 见 [`migration-baseline.json`](migration-baseline.json)：DJI/M9025 私有解析、通用 motor_base 去协议结构、三参数 PID、shoot 私有辅助函数、相关测试/文档均保留语义；未恢复删除的 `shoot_logic.hpp`。后续驱动修正是本次新增。

迁移期间原目录又出现并行修改；本工作树没有覆盖它们，仍以开工快照为基线。后续合并必须协调这些新增差异，记录见 [`migration-upstream-drift.json`](migration-upstream-drift.json)。

原文将底盘跟随、回中和相对 yaw 同步提前写成“已迁移”，与起始源码不符。本次实际新增这些路径。下面“接入”表示代码、配置入口和调用链存在，不表示 Linux 构建、台架或实车已经通过。

## 迁移清单

| 项目 | 实现与调用链 | 配置/初始化 | 软件验证 |
| --- | --- | --- | --- |
| 1 底盘跟随 | `ctrl/motion_logic.hpp` 坐标旋转、跟随 PID、自旋停止同向回中；motion → chassis → 最终轮速 | `robot.motion_info`、`chassis_info.wheel_directions` | 四分之一转、反变换、自旋过零、x/y/w 向量与比例限速 |
| 2 云台机械回中 | `gkd_sentry_gimbal::update`、机械相对 yaw、连续稳定时间；Robot `FinishInit` 门 | 显式 rad 零位、有效标志、容差/稳定时间，先 NoForce 再受控回中 | 实际协程配模拟 IMU/电机，未标定/离线/禁用/稳定时间 |
| 3 IMU 内环 | 补偿 `cos(pitch)*gyro.z-sin(pitch)*gyro.x`，角度外环 → IMU 角速度内环 → 原始电流 | 显式姿态/速度符号、秒制 PID；J6006 使用已确认固件速度模式 | 电机轴反馈不同于 IMU 时仍由 IMU 内环产生电流；模式/故障门 |
| 4 视觉 | `udp_server → aim_link → motion → gimbal/shoot`；姿态/阵营返回 | UDP 服务、固定对端、14/10 字节格式、超时直接用 info_type | 长度/端序/非有限值/过期、真实 localhost 双向收发与来源分流 |
| 5 哨兵 | 大 yaw + 小头分别实例化；两 IMU；大 yaw 卸载小头相对角；视觉失效巡扫与导航速度/状态 | `large_yaw_info`、`additional_imus`、navigation；可选第二小头/独立发射实例 | 多实例 API、安全门/目标同步、导航协议；**原始哨兵配置因 CAN 冲突被预检拒绝** |
| 6 电机驱动 | `j6006`、`m9025` 的 codec/反馈/使能/周期发送；DJI 最终输出门 | runtime 向量、construct/connect/start、类型化绑定；M9025 独立示例 | 协议向量、边界/非有限值/符号/输出限流，源文件语法 |
| 7 裁判/UI | raw serial → CRC8/16 增量 parser → 各命令独立时间戳状态；UI 编码与显示；shoot 弹种/比赛状态门 | `/dev/REFEREE` 来自旧源码；三作战车型明确 `enforce_referee: true` | 每个分片点、CRC/重同步、图元位域、真实发射协程的弹量/离线/阈值 |
| 8 超容 | codec/有效反馈/故障、周期重发、命令看门狗；motion 的 power tick 刷新命令 | `super_cap` 可选项、init/connect/start，作战车型接入 | 字节向量、限幅和安全输出门；未上机 |
| 9 功率 | 模型、按轮分配、能量环、可选 RLS；绝对电流上限进入 DJI `current()` 后被组发送使用 | `power` 可选项；四车型启用 35 W 保守软件限制，RLS/boost 默认关闭 | 2000 组确定随机边界、单位换算、后续 PID 不能旁路输出上限 |
| 10 辅助资产 | 独立 `solve_ballistics`/CLI，RemoteLogger、非破坏日志接收/HTML 绘图工具 | 无需把无调用点弹道强接入机器人；遥测 key/周期可配置 | 数值/失败路径、Python 四组测试、C++ 编码→Python 解码、源日志 SHA 保留 |

分模块详细接口和限制：[`motion-migration`](modules/motion-migration.md)、[`motor-protocols`](modules/motor-protocols.md)、[`network-control`](modules/network-control.md)、[`referee`](modules/referee.md)、[`power-control`](modules/power-control.md)、[`ballistics-tools`](modules/ballistics-tools.md)。

## 坐标、单位与参数

- 底盘由云台坐标到车体：`vx'=cos(yaw)vx+sin(yaw)vy`、`vy'=-sin(yaw)vx+cos(yaw)vy`。轮序统一 LF/RF/LR/RR；各 YAML 显式 `[-1,1,-1,1]` 恢复旧最终电机目标整体符号。旋转输入仍是轮速合成量，没有虚构底盘几何半径，不能声称其为已标定车体 rad/s。
- 旧 IMU pitch 取负、pitch_rate 不取负；迁移配置明确 `pitch_sign: -1` 和独立 rate_sign。旧角速度换算 `pi/180/1000` 显式保留为 gyro_scale，上游字段物理单位仍需实测，未把它猜成正确 SI 来源。
- 摩擦轮 m/s，拨弹 `trigger_speed` 为输出轴 rad/s，分别用 `set` 与 `set_angle_speed`。Hero 的 1.5 m/s 就绪边界改为包含等号，防止恰好达到目标时永不允许发射；就绪阈值必须为正且不大于目标。
- 旧 PID `I+=ki*e`、`D=kd*delta_e`。在原采样周期 `dt0` 下迁移为 `ki_new=ki_old/dt0`、`kd_new=kd_old*dt0`，kp/输出限幅不变。底盘取 2 ms，云台/发射取 1 ms；配置使用换算结果，不机械复用旧数值。该换算只保证固定原周期代数等价，不保证变周期或真实硬件稳定。
- 小头 DJI 零位按源码编码器值 `2*pi/8192` 转成 rad。J6006 是 `[-position_max,+position_max]` 的 16 位量化位置，旧哨兵沿用 M9025 convenience ecd 公式，无法据此确认机械零位，因此显式 `yaw_zero_calibrated: false`，不虚构标定。
- J6006 厂商协议核对修正旧 DLC：速度命令为 4 字节 float32 小端，并补齐 FC/FD 使能/失能。M9025 速度单位来源矛盾，必须显式填写 `speed_rad_per_count`；示例按旧 RPM 标签解释，不能当作固件确认。

## 未确认的硬件事实与旧工程边界

哨兵旧源码同时声明 J6006 `CAN_GIMBAL/id1` 和拨弹 M3508 `CAN_GIMBAL/id1`，前者速度命令与后者反馈均为 `0x201`。用户要求暂不处理此映射，本次保留原值，`sentry.yaml` 能完成结构解析但语义预检明确拒绝。测试用独立虚拟总线证明其余完整拓扑可校验，不把该虚拟映射写入车型文件。大 yaw 零位也仍待标定。

旧裁判/UI、超容启动被注释；旧功率计算被原始 PID 输出旁路；BulletSolver 无主流程调用点；旧 `shoot_heat` 恒 true。迁移没有据此宣称旧功能已验证。当前新增裁判许可并不提供热量预测、弹速闭环或单发节拍保证。旧哨兵无限调参循环未迁入启动路径，调参改为可选遥测和离线工具。

旧主/头配置共享发射电机，并非两套已知独立执行器。实际哨兵配置绑定一套小头发射机构；可选第二发射实例必须有独立电机，跨头视觉许可不能授权另一头发射。网络接收采用明确对端 IP/端口，旧对端发送程序不在仓库，其源端口兼容性仍需部署时确认。

## 验证与交付边界

用户要求不执行 xmake 后已停止配置过程，后续未执行任何 xmake 命令。直接 Clang 编译的无硬件 aggregate 单测与分项协议/模拟测试通过；主程序只做可行的语法检查，不链接/运行。完整 Linux 目标构建、CAN/串口台架、实车均未执行。

本次没有运行 `gkd-roboctrl` 或 `start.sh`，没有操作真实硬件，没有合并或推送。图表 HTML 的 JS 静态检查通过，浏览器运行时未验证；ASan 在本环境挂起，未计为通过，网络 UBSan 测试通过。最终执行证据见 [`migration-verification.md`](migration-verification.md)。
