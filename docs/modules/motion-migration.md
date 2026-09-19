# 运动、云台与哨兵迁移

本模块对应 [`control.md`](control.md)、[`device.md`](device.md) 和
[`config-build.md`](config-build.md)。代码位于 `ctrl/motion_control.*`、
`ctrl/motion_logic.hpp`、`device/gimbal/` 和 `device/imu/`。

## 生命周期与安全门

云台注册表现在拥有独立的 `unique_ptr<gimbal_base>` 实例。主云台保留 `current()`
接口，`robot.secondary_gimbal_info` 和 `robot.large_yaw_info` 可创建独立实例，
不会反复初始化同一个单例。每个云台先绑定 IMU/电机并初始化控制器，再调用幂等
`start()`；控制器和设备均由进程生命周期持有。Robot 只使能实际绑定的设备。

解锁进入 `FinishInit`：只使能云台，底盘和发射器仍禁用。所有云台完成初始化后
进入 `FollowGimbal`。初始化回中也必须同时满足解锁、在线和机械零位已标定，
禁止在启动或 `NoForce` 中自动回中。遥控器失联进入 `NoForce`；正常运行中
任一绑定云台依赖失联也进入 `NoForce`，要求重新解锁。初始化期间等待反馈，
不会忙等、退出进程或运行旧工程无限调参循环。

每台云台的 `set_enabled(false)` 立即禁用其绑定电机、清 PID 和初始化状态。
周期更新在禁用、IMU/电机离线或回中缺少标定时，电流控制发送零电流；
原生速度驱动同时禁用，避免把“零速度闭环仍有保持力矩”误当作零输出。
原生速度驱动仅在依赖和标定门均允许后才使能，不能在解锁调用中提前使能。
驱动故障即使仍有 CAN 心跳也不满足在线/初始化条件；控制器锁存该故障直到
Robot 显式禁用，再由重新解锁恢复，内部依赖门的临时禁用不会吞掉故障。
`initialized()` 同时检查在线状态。

## 坐标、单位与回中

`yaw()`/`pitch()` 返回测量姿态，`target_yaw()`/`target_pitch()` 返回目标。
所有角度使用 rad、角速度使用 rad/s；不再将角度外环输出隐式当作轮缘 m/s。
机械相对 yaw 为 `rad_format(motor.angle() - yaw_zero)`，且只有在线、
`yaw_zero_calibrated=true` 才能作为底盘方向依据。零位配置必须与实际驱动反馈
角度定义一致。DJI 的旧 `YawOffSet` 可依据 8192 计数每圈转换；不能把 DJI
或 M9025 的计数常数直接套入 J6006 的位置映射。

`recenter_on_enable=true` 时，`FinishInit` 用 `yaw_relative_pid` 将相对 yaw
回到零，并用 pitch 角度环回到 `init_pitch`；两轴连续处于 `init_tolerance`
内达 `init_settle_time` 才完成。任意超差、失联或禁用会清除已累计时间。
`yaw_only` 的大 yaw 不创建虚构 pitch 电机，也不检查 pitch 收敛。

没有可确认零位时应显式保留 `yaw_zero_calibrated=false`；软件会等待标定并保持
该阶段零输出。如果明确配置 `recenter_on_enable=false`，可仅锁定首次在线
世界姿态，但缺少机械零位仍禁止底盘坐标变换输出。这是配置中的可验证边界，
不表示零位已经通过上机验证。

控制层的底盘变换保留旧源码约定：

```text
vx_chassis = cos(relative_yaw) * vx_gimbal + sin(relative_yaw) * vy_gimbal
vy_chassis = -sin(relative_yaw) * vx_gimbal + cos(relative_yaw) * vy_gimbal
```

无自旋请求时 `follow_pid` 跟随零相对角；停止自旋后先保留最后旋转方向，
以 `spin_recenter_speed` 回到零附近再交给跟随 PID。离散采样跨过零点也会
结束此阶段，跨过正负 pi 边界不会被误认为过零。`NotFollow` 不使用跟随 PID。
四轮最终装配方向由底盘配置单独规定，不能用坐标变换掩盖符号差异。

## IMU 角速度内环

DJI/M9025 云台采用角度外环与 IMU 角速度内环：

```text
姿态目标 → rad_pid → 角速度目标
角速度目标 - IMU角速度 → linear_pid → motor.set_current(raw)
yaw_rate = cos(pitch) * gyro.z - sin(pitch) * gyro.x
pitch_rate = gyro.y
```

直接电流接口绕过驱动原有轮缘速度 PID。`yaw_angle_direction` 和
`yaw_recenter_direction` 作用于外环角速度目标；`yaw_direction` 和
`pitch_direction` 作用于最终电机命令。旧源码绝对 yaw 外环有 `Invert(-1)`，
机械回中外环没有该步骤，必须分别配置，不能用一个统一符号静默替换二者。

J6006 当前核实的是原生速度指令，因此大 yaw 使用
`yaw_current_control=false`，下发 `set_angle_speed(rad/s)`；它没有被伪装成
支持电流内环。IMU 姿态外环仍工作，速度环由设备内部执行。这与旧控制器在
原生速度指令前又串联软件速率 PID 的结构不同，外部电流模式尚无可确认协议，
不可宣称两种内环完全等价。

串口 IMU 的 key-1 负载现在明确为 24 字节、6 个 LE float32：
`yaw,pitch,roll,yaw_v,pitch_v,roll_v`；短帧、原始非有限数或单位换算溢出均不
刷新在线状态。云台另行检查完整姿态/角速度及补偿结果的有限性；无效快照会
清除在线与初始化条件，保持最后有限姿态供显示，并清零或禁用执行器。
`*_sign` 与 `*_rate_sign` 分别配置姿态和角速度符号。旧 IMU 对 pitch 姿态
取负，但未对 pitch_rate 取负；迁移配置须保留并明确记录这个事实。
`gyro_scale` 默认保留源码数值 `pi/180/1000`；其上游原始单位仍需测量确认，
不能仅凭除以 1000 就声称上游发送“度/毫秒”。

PID 以秒制 dt 更新，旧逐次 PID 在固定旧周期 `T` 的代数等价转换是
`kp_new=kp_old`、`ki_new=ki_old/T`、`kd_new=kd_old*T`，还需确认反馈单位与
输出单位相同；更换单位或内环结构后不能靠此公式保证硬件稳定性。

## 视觉、导航、多云台与巡扫

`motion_info.primary_aim_key`/`secondary_aim_key` 绑定设备层的视觉目标；控制层
不解析报文。仅在操作员 `auto_aim` 请求成立时使用新鲜目标。切换手动/视觉
先锁测量姿态；目标失效停止发射并保持姿态，或在显式启用
`search_when_vision_stale` 时进入 `Search`。搜索使用秒制 yaw 匀速与 pitch
正弦扫描，受同一解锁/在线门控制。收到视觉报文本身不会改变 Robot 安全状态。

每台云台分别计算视觉 fire 许可。主发射器只使用主云台许可；存在独立
`secondary_shoot_info` 时副发射器只使用副云台许可。副云台没有独立发射器时，
其许可不会被合并到主发射器，避免不同瞄准方向授权同一个拨弹电机。
所有发射仍需摩擦轮、堵转、裁判等安全门。
Sentry 源码实际使用 J6006 大 yaw 加 DJI 小头及两个 IMU，不能从旧配置表中
额外制造一套主 pitch 或第二发射器。附加实例接口是可选扩展，不等于有该硬件。

大 yaw 目标以“自身测量 yaw + 配置方向 × 小头机械相对 yaw”释放小头偏转，
不假设两个独立 IMU 上电后具有相同绝对 yaw 原点。
`large_yaw_follow_direction` 需要按轴向标定。

导航必须同时满足 `enable_navigation`、操作员自动瞄准请求和新鲜导航指令；
过期时平移速度置零。底盘相对角依据大 yaw（若存在），否则依据主云台。
姿态/导航状态每 20 ms 发送，阵营优先使用新鲜裁判身份，否则使用显式
`red_team` 回退配置；缺少新鲜裁判时 hp=0、match_started=false。
`remote_logger_key` 启用按 `telemetry_period` 下发目标/测量姿态、yaw 角速度、
功率分配和发射状态，所有任务都在既有 Asio 事件循环协作运行。

功率管理每个 motion tick 更新，包括 `NoForce`；其输出限流确实作用于底盘
绑定电机。裁判 UI 同步摩擦轮就绪、自动瞄准、自旋、发射许可及超容能量。
超容能量是旧协议索引按 255 归一化后的显示值，不是通过标定的物理荷电百分比。

## 无硬件验证与边界

`tests/motion_migration_tests.cpp` 覆盖坐标逆变换、跟随与自旋停止过零、IMU
长度/端序/非有限数、pitch 补偿投影、秒制 PID 参数换算，以及用模拟 IMU/电机
驱动真实云台协程单步 `update(dt)` 的集成验证：禁用零输出、IMU 失联、
相对零位、持续回中判据、未知零位阻断、直接电流使用 IMU 反馈而非电机速度、
原生 yaw-only 速度路径、失效时禁用保持力矩、恢复使能、故障心跳锁存以及不支持电流模式的拒绝。

本次按用户要求不执行 xmake；上述测试使用本机 clang++ 直接构建并运行，
并对受影响控制、云台和 IMU 源文件执行 `-fsyntax-only`。未启动主程序，未访问
CAN/串口，未验证 Linux 主目标、总线时序、实际 PID 稳定性、零位和轴向。
