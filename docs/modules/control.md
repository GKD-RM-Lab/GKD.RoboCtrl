# Control 模块

路径：`include/ctrl/`、`src/ctrl/`

Control 层组合设备引用并表达整机行为。它不解析总线报文。

设计理念是让“机器人想做什么”与“报文怎样编码”分离：Control 只读取设备提供的物理量、设置目标并维护状态机；具体总线、ID、端序和反馈结构留在 Device/IO。控制模块是全局唯一单例，通过车型配置决定哪些子系统启用。

## Robot 与安全状态

`robot::info_type` 分别声明是否启用底盘、云台和发射。初始化只启动启用的子系统，Robot 默认进入 `NoForce`。状态切换会统一禁用或启用所有 DJI 电机；这只是软件安全门，不能替代硬件急停和台架验证。

`robot_state` 当前包含 `NoForce`、`FinishInit`、`FollowGimbal`、`Search`、`Idle`、`NotFollow`。除 `NoForce` 与“其他状态”的电机使能差异外，尚没有完整状态转移规则。`robot` 还提供底盘速度、云台相对角和旋转速度的转发接口；调用这些接口前必须确认对应子系统在当前车型启用。

切换到 `NoForce` 会调用每台 DJI 电机的 `disable()`，清空 PID、当前输出并关闭 `enabled_`；切换到任意其他枚举值会使能电机。当前代码没有遥控器或裁判系统主动离开 `NoForce`，因此正常启动后应保持零输出。

成熟度：整机初始化和统一安全门 **已接入**；业务状态机 **部分实现**。

## Chassis

底盘在初始化时将四个具体 DJI 电机绑定为 `motor_ref`，周期循环不再重复写具体电机模板参数。麦轮分解与限速位于纯函数 `mecanum_wheel_speeds()`，便于无硬件测试。`NoForce` 分支只设置零目标。

输入 `velocity_` 先按 `gimbal_yaw_` 旋转到车体坐标，再与 `rotate_speed_` 合成四轮目标；若任一绝对轮速超过 `max_wheel_speed_`（当前 2.5），四轮按统一比例缩放以保持方向关系。右前、右后电机在实际下发时取反，属于底盘装配方向约定。

控制循环周期为 1 ms。NoForce 时每轮为四台电机设置零目标并提前返回；由于 DJI 组层还有禁用/离线置零，这形成两层软件保护。`chassis_kinematics.hpp` 是无硬件纯函数，运动学或限速变化应优先在这里添加测试。

成熟度：**部分实现**。运动学和比例限速已接入；尚缺控制周期抖动监控、功率限制和硬件符号标定。

## Shoot

摩擦轮和拨弹电机同样保存为 `motor_ref`。`firing_` 默认 false；`NoForce` 会重置斜坡、设置三台电机为零并跳过本轮后续命令，避免零命令被覆盖。

非 NoForce 状态下，`friction_ramp_` 以配置的最大变化率逐渐逼近 `friction_max_speed`，左右摩擦轮目标符号相反。当前 `firing_` 同时被当作“摩擦轮运行”开关，拨弹 `trigger` 在正常开火路径没有非零命令，也没有单发/连发、堵转恢复、热量或弹速限制。

成熟度：**部分实现**。摩擦轮斜坡存在，实际拨弹/连发状态机尚未完成。

## Gimbal、Power、Referee

Gimbal 初始化会保存配置传入的 yaw、初始 yaw 和 pitch 三组 `controlled_motor<dji_motor, rad_pid>`，并启动 1 ms 循环；循环当前没有读取姿态、更新 PID 目标或下发电机。因此 Infantry/Hero 虽配置为启用 Gimbal，也不能据此宣称云台可控。

`power_manager.h` 是尚未迁移到当前命名空间和设备类型的旧接口，引用 `Hardware::DJIMotor`、`ControllerList`、`Robot::Robot_set` 等当前仓库未定义类型；对应 `.cpp` 为空，且头文件不在主程序编译路径中。`referee.h` 只有 `#pragma once`。这两者都是**接口骨架/遗留设计材料**，不能直接 include 后使用。

## 控制参数与单位

- `robot::set_velocity(x, y)` 与底盘轮速上限使用的最终单位需由机械和电机半径标定确认；代码以 `motor_base::linear_speed()` 的 m/s 语义设计。
- `gimbal_yaw` 和云台 PID 使用 rad。
- `rotate_speed` 直接参与轮速合成，当前未乘几何半径；其比例含义需通过底盘模型/标定固定。
- 控制周期写死为 1 ms，但 PID 实现没有显式 `dt`，参数与周期耦合。

## 修改 Control 时的检查清单

- NoForce、离线、异常和子系统禁用时是否始终产生安全零输出。
- 状态转移是否有唯一入口、合法条件、退出动作和日志。
- 语义化电机名是否由配置校验保证，符号与单位是否有台架依据。
- 运动学/状态机中可提取的纯逻辑是否具有无硬件测试。
- 不在 Control 中解析字节报文或硬编码 CAN/串口接口。
- 对外报告分别说明单测、构建、台架和实车验证状态。
