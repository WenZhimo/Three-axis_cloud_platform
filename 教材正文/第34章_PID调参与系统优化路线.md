# 第34章 PID调参与系统优化路线

> 导航：上一章：[第33章_下载调试与电机硬件诊断](第33章_下载调试与电机硬件诊断.md) ｜ 下一章：无

## 1. 本章目标

前33章已经完成从 MCU 平台、外设、传感器、姿态解算、PID、电机输出、系统门控到下载诊断的主线。本章作为收束章，把前序知识转化为一条可执行的调参和优化路线。

本章目标是：

- 理解 `PID调参仿真工具` 在项目中的位置。
- 明确仿真工具能复用哪些固件参数，不能替代哪些仓库外实测验证。
- 建立从离线仿真、固件参数、仓库外实测观察到系统优化的闭环流程。
- 汇总移植、扩展和类似控制系统设计时应保留的关键检查点。

本章知识链为：

`知识点总表` 中的 `PID调参仿真工具`
-> `知识依赖图` 中它依赖 `PID控制`、`PID微分项滤波`、`输出限幅与速率限制`、`积分抗饱和` 和 `电机硬件诊断`
-> `学习优先级` 中的 P3 调试方法知识
-> `教学顺序` 第62项
-> `教材章节` 第34章。

本章阅读分层：

| 阅读层次 | 建议范围 | 适合读者 |
|---|---|---|
| 【必须掌握】 | 第1节到第5.4节，第13节总结 | 需要建立“先诊断、再仿真、再实测调参”主线的读者 |
| 【工程深化】 | 第5.5节到第8.16节，第9节调试方法 | 需要维护仿真脚本、固件参数解析、限幅观察和指标记录的读者 |
| 【拓展阅读】 | 第4.7节到第4.10节，第5.9节到第5.13节，第12节思考题 | 需要进一步理解仿真一致性、对象参数、状态重置和可计算指标的读者 |
| 【证据与验证】 | 第4.5节到第4.10.1节、第5.7节到第5.13节、第8节、第9节、章节尾部固定检查，以及所有 `【待验证】` 项 | 需要审查固件参数解析、仿真一致性、当前PID来源、`windupGuard` 与限速边界、D项差异、指标记录、`.map/.list/.su/.cyclo` 构建证据或准备仓库外实测记录的读者 |

如果只是建立调参与优化路线，可以先抓住“固件证据 -> 仿真趋势 -> 目标板实测 -> 参数回写和回滚”这条链；要解释异常曲线或复现实验时，再回到仿真对象、限幅主导和指标记录小节。

## 2. 前置知识

本章正式前置知识包括：

- `PID控制`
- `PID微分项滤波`
- `输出限幅与速率限制`
- `积分抗饱和`

这些知识分别来自第27章、第28章和第33章。第33章的下载调试与电机硬件诊断提供仓库外实测验证基础，避免在底层输出未确认前直接进入参数优化。

本章内部顺序为：

```text
固件PID证据
-> 仿真工具结构
-> 仿真与仓库外实测差异
-> 调参路线
-> 移植与扩展路线
```

原因是，调参不能先从“调哪个数”开始，而要先确认这些数在固件中如何被使用。

## 3. 问题背景

PID 调参最容易犯的错误，是把调参当成孤立改 P、I、D 三个数。

在本项目中，电机最终表现同时受这些因素影响：

- 姿态角是否可靠。
- `dt500Hz` 是否稳定。
- PID 的 P/I/D 参数。
- D 项滤波是否合适。
- 积分项是否被允许累加。
- 输出限幅是否过小或过大。
- 速率限制是否限制了指令变化。
- 机械角到电角的换算是否正确。
- 三相 PWM 和硬件映射是否正确。
- 开机门控和回中状态是否已经切换到目标阶段。

因此，本章的核心不是给一组“万能 PID 参数”，而是建立一条工程调试路线，让读者知道每一步改动应该验证什么、不能用什么现象过度推断。

## 4. 核心概念

### 4.1 PID调参仿真工具

项目中提供了：

```text
tools/pid_tuning_sim.py
```

它使用 `tkinter` 和 `matplotlib` 构建三轴 PID 调参界面，支持 Roll、Pitch、Yaw 三轴独立输入 P/I/D 参数，并绘制目标角、实际角和控制输出。

工具支持三种测试模式：

- 阶跃。
- 扰动。
- 阶跃和扰动同时存在。

它还会尝试从固件源码中读取默认参数，而不是完全依赖手工输入。

### 4.2 固件参数解析

`parse_firmware_defaults()` 会读取：

- `Core/Src/main.c`
- `Drivers/SRC/Src/computeMotorCommands.c`
- `Drivers/SRC/Src/config.c`

它从 `main.c` 解析三轴 P/I/D 覆盖值，从 `computeMotorCommands.c` 解析 `ROLL/PITCH/YAW_CMD_LIMIT_RAD`，从 `config.c` 解析 `eepromConfig.rateLimit`。

这说明仿真工具不是孤立脚本，它和固件之间存在参数证据链。

### 4.3 仿真对象模型

`run_simulation()` 使用简化二阶对象：

```text
theta_ddot + 2*zeta*wn*theta_dot + wn^2*theta = plant_gain*u + disturbance
```

这里的 `wn`、`zeta` 和 `plant_gain` 是仿真对象参数，用于模拟一个角度控制对象的响应趋势。

这不是项目真实电机、真实 IMU、真实驱动、真实机械结构的完整模型。它适合观察参数变化趋势，不适合直接替代仓库外实测调试。

### 4.4 调参闭环

本章建议的调参闭环是：

```text
确认硬件输出
-> 确认姿态输入
-> 确认门控状态
-> 离线仿真预演
-> 小幅修改固件参数
-> 仓库外低风险实测验证
-> 记录现象并回退或继续
```

这个顺序比直接修改 PID 更稳，因为它先排除硬件、姿态和门控问题。

### 4.5 证据层级

本章后续所有调参结论都按三层证据理解：

```text
离线仿真 -> 只能预演趋势
仓库源码 -> 只能确认软件路径和参数来源
仓库外实测 -> 才能确认方向、温升、稳定性和真实闭环效果
```

缺少仓库外实测记录时，真实电机响应、方向正确性、不过热和闭环稳定性都保持【待验证】。

#### 4.5.1 调参证据分层

Åström 和 Hägglund 的 `Advanced PID Control` 把 PID 作为反馈控制器的工程基础来讨论：
P 项响应当前误差，I 项用于消除长期偏差，D 项利用变化趋势改善动态响应。
这能支撑本章“先确认反馈方向，再逐步调 P、D、I”的理论背景。
但经典 PID 调参理论不能直接证明本项目某组参数已经适合当前三轴云台。

本项目调参应把证据拆成五层：

| 证据层级 | 本章对应证据 | 能证明什么 | 不能证明什么 |
|---|---|---|---|
| 控制理论层 | Åström/Hägglund 的 PID 资料、反馈控制基本原理 | PID 的 P/I/D 分工、负反馈方向优先、积分与微分的典型风险 | 不能证明本项目电机方向、姿态符号、限幅和速率限制已经正确 |
| 离线仿真层 | `tools/pid_tuning_sim.py` 的二阶对象、阶跃/扰动曲线、`update_pid()` | 能在同一假设对象下比较参数趋势，发现饱和、限速和 D 项尖峰风险 | 不能证明真实云台惯量、摩擦、噪声、供电和结构共振与仿真一致 |
| 源码/构建层 | `pid.c`、`computeMotorCommands.c`、`.map`、`.list`、`.su`、`.cyclo` | 能证明某个 Debug 构建中的 PID 计算、限幅、限速、状态清零和 PWM 输出路径存在 | 不能证明板上已经烧录同一镜像，也不能证明运行时满足 500Hz 预算 |
| 板上观测层 | 调试器读取 `eepromConfig.PID[]`、`pidCmd[]`、`outputRate[]`、`rollDiag`、`dt500Hz`、`executionTime500Hz` | 能证明目标板 RAM 中的参数、状态和耗时观测值【待验证】 | 不能单独证明电机不过热、机械无撞限或长期稳定 |
| 硬件安全层 | 电流、温升、振动、机械限位、物理轴方向和长时间运行记录【待验证】 | 能证明参数在特定硬件、负载和环境下的安全性与稳定性【待验证】 | 不能外推到不同电机、负载、供电、安装方向或固件版本 |

所以，本章不把“仿真曲线更好”写成“实物参数更优”，也不把
`.map/.list` 中的调用路径写成“闭环已经调好”。调参结论必须说明自己停在哪一层证据上。

### 4.6 调参变量与观测变量

调参不是只修改 `P/I/D`。在本项目中，至少要同时区分四类量：

- 设定量：`pointingCmd[]`、阶跃目标、回中目标。
- 反馈量：`sensors.margAttitude500Hz[]`、经过符号修正后的轴角。
- 控制量：`pidCmd[]`、`rollDiag.pidRaw`、`rollDiag.pidApplied`。
- 约束量：`cmd_limit`、`rateLimit`、`iHold`、`return_state` 和轴使能。

如果只记录“电机动了/没动”，读者无法判断问题来自 PID 参数、姿态方向、限幅、速率限制、门控状态还是底层 PWM。调参记录表必须把这些量分开。

### 4.7 仿真一致性边界

`tools/pid_tuning_sim.py` 尽量贴近固件，但它不是固件的逐行复刻。当前至少有三类边界：

- 对象模型边界：脚本用统一二阶模型，固件面对真实三轴机械结构。
- 限制器边界：脚本对三轴使用统一每步 `rate_limit`，固件 Roll 与 Pitch/Yaw 用法不同。
- D 项边界：脚本和固件在 D 项初值处理、角度跨界时的差分细节不完全一致。

因此，仿真曲线只适合作为趋势预演。凡是涉及“最终参数”“真实稳定”“真实不过热”的结论，都必须回到仓库外实测。

### 4.8 仿真对象参数边界

仿真界面中的 `wn`、`zeta` 和 `plant_gain` 不是固件参数，也不是仓库已经标定出的真实电机参数。它们只属于 `run_simulation()` 中的简化对象模型：

- `wn` 表示模型的固有响应速度。
- `zeta` 表示模型阻尼强弱。
- `plant_gain` 表示控制指令转化为角加速度的比例。

当前脚本对 Roll、Pitch、Yaw 使用同一组对象参数。这适合教学演示“同一组 PID 在不同对象假设下会有不同响应”，但不能证明三轴真实机械结构相同。

如果要让仿真更接近实物，必须先有仓库外记录：输入指令、姿态响应、时间戳、限幅状态和安全条件。缺少这些记录时，`wn/zeta/plant_gain` 只能作为仿真假设，不能作为实物辨识结果。

### 4.9 参数修改与PID内部状态

PID 参数不是唯一会影响下一帧输出的量。当前固件的 `PIDdata_t` 还保存：

```text
iTerm
lastDcalcValue
lastDterm
lastLastDterm
```

这些是运行时状态，不是静态参数。它们会随控制循环持续更新。
如果调参时只修改 `P/I/D`，但不处理这些历史状态，那么下一次输出同时包含：

```text
新P/I/D参数
+ 旧积分累加值
+ 旧D项滤波历史
+ 旧输出速率限制状态
```

因此，参数修改后的第一段响应不一定只反映新参数本身。
仿真工具每次点击“运行仿真”都会重新构造 `PIDRuntime`，默认从零状态开始；
固件除非调用 `zeroPIDintegralError()`、`zeroPIDstates()` 或轴级重置逻辑，
否则 PID 历史状态会继续保留。

调参记录必须区分：

- 冷启动后第一次响应。
- 清零 PID 状态后的响应。
- 不清零状态、只在线修改参数后的响应。

这三者的曲线和电机表现可能不同。缺少状态记录时，不应直接判断某组 P/I/D 的优劣。

### 4.10 调参试验基线与回滚点

调参不是“看到不好就继续改”，而是一组可复现实验。每一次调参都应先定义基线：

- 固件版本和参数来源。
- 被测轴和被测模式。
- 阶跃目标、扰动条件或回中条件。
- `dt500Hz`、`executionTime500Hz` 和门控状态。
- PID 历史状态、`pidCmdPrev[]` 和限幅状态。
- 仓库外安全条件，例如供电、电流、温升和机械限位【待验证】。

基线存在的意义，是让下一次改动只多一个变量。没有基线时，响应变化可能来自 P/I/D，
也可能来自初始角度、旧积分、D 项历史、速率限制、轴使能或测试姿态不同。

回滚点也必须在改参数前定义。最低要求是保存上一组有记录支撑的 P/I/D、
限幅设置、`rateLimit`、是否清零状态、以及触发回滚的异常现象。
如果出现方向反转、持续饱和、执行时间逼近周期预算、明显震荡、温升或电流异常【待验证】，
应先回退到最近的记录点，而不是继续叠加新参数。

#### 4.10.1 调参记录的可复现性边界

一条可复现的调参记录至少要拆成四张快照，不能只保存 P/I/D 三个数。

| 快照 | 仓库内证据 | 必须记录的最小字段 | 不能替代什么 |
|---|---|---|---|
| 参数快照 | `main.c` 覆盖 `eepromConfig.PID[axis].P/I/D`，`config.c` 写入 `rateLimit` | 固件提交号、被测轴、P/I/D、`cmd_limit`、`rateLimit` | 不能证明板上 RAM 已经是这些值 |
| 状态快照 | `pid.c` 持续更新 `iTerm`、`lastDcalcValue`、`lastDterm`、`lastLastDterm` | 是否调用 `zeroPIDintegralError()` / `zeroPIDstates()`，PID 历史和 `pidCmdPrev[]` | 不能只用新 P/I/D 解释响应差异 |
| 时序快照 | `main.c` 用 `dt500Hz` 和 `executionTime500Hz` 记录 500Hz 周期与耗时 | 实际 `dt500Hz`、本帧耗时、是否打印或阻塞 | 不能证明长期实时性和最坏情况栈余量【待验证】 |
| 安全快照 | 源码只能给出限幅、限速和门控变量 | 轴使能、限幅触发、限速触发、回滚条件 | 不能证明温升、电流、机械撞限和长期稳定【待验证】 |

因此，“同一组 P/I/D 表现更好”只能在这四类快照都可比时成立。
如果状态、时序或硬件条件不同，结论应降级为现象记录，而不是参数优劣判断。

## 5. 工作原理

### 5.1 仿真工具如何贴近固件

仿真工具和固件保持一致的地方包括：

- `DT_DEFAULT = 0.002`，对应 500Hz 实时控制周期。
- `RC_D_FILTER = 1/(2*pi*20)`，对应 `pid.c` 中 `F_CUT=20Hz`。
- `wrap_pi()` 对应固件中的角度归一化思想。
- `update_pid()` 对异常 `dt` 回退到 0.002。
- 积分项限制到 `[-10, 10]`。
- D 项限制到 `[-300, 300]`。
- 输出使用 `cmd_limit` 限幅。
- 控制输出使用 `rate_limit` 做变化限制。

这些一致性让它适合作为调参预演工具。

### 5.2 仿真工具和固件的差异

差异同样重要。

第一，仿真工具使用统一的二阶对象模型；固件项目中，Roll、Pitch、Yaw 的机械结构、映射和门控路径并不完全一致。

第二，脚本中的速率限制对三轴采用统一的每步 `rate_limit` 差值限制。

固件中 Roll 分支使用 `eepromConfig.rateLimit * safeDt` 并带最小步长。

Pitch/Yaw 当前直接用 `eepromConfig.rateLimit` 作为每帧差值限制。

第三，仿真工具不模拟三相 PWM、供电、电机发热、传感器噪声、安装方向误差和回中状态机。

第四，仿真工具不参与固件运行路径。它是离线工具，不会自动改变板子上的参数。

因此，本章必须把它写成“调参辅助”，不能写成“调参验证完成”。

### 5.3 固件中真正影响PID输出的因素

`pid.c` 中 `updatePID()` 的核心路径为：

```text
检查deltaT
-> 计算误差
-> 角度PID执行归一化
-> 根据iHold决定是否累加积分
-> 计算D项
-> D项低通与三点平均
-> 计算P/I/D输出
-> NaN/Inf输出保护
```

`computeMotorCommands.c` 在 `updatePID()` 外面又增加了轴级限制：

- Roll 计算电角误差，误差过大时暂停积分。
- Roll 对 PID 输出执行 `ROLL_CMD_LIMIT_RAD` 限幅。
- Roll 使用 `rateLimit * safeDt` 得到步长限制，并设置最小步长。
- Pitch/Yaw 对 `pidCmd[]` 执行各自限幅。
- Pitch/Yaw 当前对 `outputRate` 使用 `eepromConfig.rateLimit` 直接限制每帧变化。

这说明调参时不能只看 `P/I/D`，还要同时看 `iHold`、限幅和速率限制。

### 5.4 为什么先诊断再调参

第33章已经说明，电机硬件诊断可以绕过 PID 和姿态解算检查底层 PWM 输出。如果底层某个通道没有输出，继续调整 PID 只会把问题藏得更深。

推荐顺序是：

```text
下载调试正常
-> PWM初始化正常
-> 底层通道可点测
-> 姿态角输出正常
-> 门控状态正确
-> 再进入PID调参
```

这条顺序能让调参建立在已确认的可控对象上，而不是拿 PID 参数补偿硬件或方向错误。

进一步说，进入 PID 参数优化前，还要把方向链路拆成五段：

```text
姿态传感器符号正确
-> 机械误差方向正确
-> 机械角到电角的符号、比例和零位偏置正确
-> PWM_Motor_SetAngle()的枚举到CCR映射正确
-> 物理轴响应方向正确【待验证】
```

这里的“方向正确”不是一句口头判断，而是闭环能否形成负反馈的必要条件。以单轴小角度近似为例，设目标角暂时不变，机械误差为：

```text
e[k] = target - state[k]
```

若只启用很小的 P 项，有：

```text
u[k] = Kp * e[k]
state[k+1] ~= state[k] + G * sigma * u[k]
e[k+1] ~= e[k] - G * sigma * Kp * e[k]
```

其中 `G` 表示被控对象对控制量的实际响应增益，`sigma` 表示传感器符号、电角换算符号、定子方向和电机接线映射合成后的方向因子。若 `G * sigma > 0`，小 P 会让误差变小；若 `G * sigma < 0`，误差会被推大，系统进入正反馈。此时继续增大 P、加入 D 或加入 I，通常只会放大错误方向，不能把方向链路“调正确”。

### 5.5 固件离散PID公式

把 `updatePID()` 拆成离散形式，可以得到当前固件的核心计算链。

角度误差为：

```text
e[k] = wrap(command[k] - state[k])
```

当 `iHold == false` 时，积分项更新为：

```text
Iacc[k] = clip(Iacc[k-1] + e[k] * dt, -10, 10)
```

当 `iHold == true` 时，积分项保持上一帧，不继续累加。当前 `PIDdata.windupGuard`
字段虽然存在，并在 `main.c` 中被重新赋值，但 `pid.c` 的实际积分限幅使用硬编码
`[-10, 10]`，没有使用 `windupGuard` 字段。这是调参时必须知道的源码边界。

D 项按 `dErrorCalc` 分两种路径。当前 `config.c` 初始化三轴为 `D_ERROR`：

```text
d_raw[k] = wrap(e[k] - e[k-1]) / dt
```

如果使用 `D_STATE`，则按状态变化计算：

```text
d_raw[k] = wrap(state[k-1] - state[k]) / dt
```

最后输出为：

```text
u_pid[k] = P * e[k] + I * Iacc[k] + D * d_avg[k]
```

其中 `d_avg[k]` 不是原始差分，而是经过限幅、低通和三点平均后的 D 项。

### 5.6 D项低通与三点平均

`pid.c` 使用：

```text
F_CUT = 20Hz
rc = 1 / (2*pi*F_CUT)
alpha = dt / (rc + dt)
d_filtered[k] = d_filtered[k-1] + alpha * (d_raw[k] - d_filtered[k-1])
d_avg[k] = (d_filtered[k] + d_filtered[k-1] + d_filtered[k-2]) / 3
```

在理想 500Hz 下，`dt = 0.002s`，因此：

```text
rc = 1 / (2*pi*20) ≈ 0.00796s
alpha = 0.002 / (0.00796 + 0.002) ≈ 0.201
```

这说明当前 D 项并不是直接使用相邻两帧差分，而是先用一阶低通削弱高频噪声，再用三点平均进一步平滑。调 D 时如果只看公式 `D * de/dt`，会漏掉这两级动态。

D 项还有原始限幅：

```text
d_raw ∈ [-300, 300]
```

因此，当姿态噪声或 `dt` 异常导致差分过大时，固件会先限制 D 项输入，再进入低通。仿真中看到的尖峰，应同时检查 `dt`、姿态噪声、D 项限幅和滤波状态。

### 5.7 deltaT回退的证据边界

调参记录里的“500Hz”还要再拆成三层。`Core/Src/main.c` 用 `micros()` 计算
`deltaTime500Hz`，再得到 `dt500Hz = deltaTime500Hz / 1000000.0f`，并把这个值传入
`computeMotorCommands(dt500Hz)`。进入 `updatePID()` 后，源码又有一层防护：

```text
if deltaT > 0.01 or deltaT < 0.0001 or deltaT is NaN/Inf:
    deltaT = 0.002
```

因此，`dt500Hz` 的观测值、`computeMotorCommands()` 收到的 `dt`、`updatePID()` 最终用于积分和微分的
`deltaT` 不是同一层证据。当前源码没有额外变量记录“本次是否触发了 `deltaT=0.002f` 回退”，
所以仅凭外部记录的 `dt500Hz` 不能反推出每一次 PID 内部都使用了同一个 `deltaT`。若调参时出现 I 项累加速度异常、
D 项尖峰或 D 项突然变钝，应同时记录 `dt500Hz`、`executionTime500Hz`、是否存在超长帧/过短帧，
并在需要时临时增加调试计数或断点确认 `updatePID()` 是否进入回退分支【待验证】。

这个边界也解释了仿真工具的价值和限制。`tools/pid_tuning_sim.py` 中 `update_pid()` 使用同样的
`dt > 0.01`、`dt < 0.0001` 和非有限值回退逻辑，默认 `DT_DEFAULT = 0.002`。这能复现固件的异常
`dt` 防护语义，但仿真界面中的 `dt (s)` 是用户输入的离线步长，不是板上 `micros()` 实测结果。仿真曲线可以展示
`deltaT` 变化对积分、微分和滤波系数的影响，不能证明目标板真实循环周期稳定。

### 5.8 输出限幅与速率限制的单位

`computeMotorCommands.c` 在 `updatePID()` 之后还有轴级约束。它们决定 PID 输出能不能真正变成电角指令。

Roll 路径：

```text
pidCmd[ROLL] = clip(rollPidRaw, -ROLL_CMD_LIMIT_RAD, ROLL_CMD_LIMIT_RAD)
rollStepLimit = max(eepromConfig.rateLimit * safeDt, AXIS_MIN_STEP_LIMIT_RAD)
pidCmd[ROLL] = pidCmdPrev[ROLL] +/- rollStepLimit
```

以当前 `rateLimit = 45deg/s = 0.7854rad/s`、`safeDt = 0.002s` 估算：

```text
rateLimit * safeDt = 0.7854 * 0.002 ≈ 0.00157rad/frame
```

该值大于 `AXIS_MIN_STEP_LIMIT_RAD = 0.001`，因此理想 500Hz 下 Roll 每帧最大变化约 0.00157rad。
如果 `safeDt` 更小、`rateLimit` 更低，或调试者把 `rateLimit` 调到很小，
`AXIS_MIN_STEP_LIMIT_RAD` 会成为实际步长下限。此时继续降低 `rateLimit`
不一定继续降低 Roll 的每帧输出变化，调参记录必须把“计算步长”和“最小步长钳位后的有效步长”分开。

Pitch/Yaw 当前写法不同：

```text
if outputRate[PITCH] > eepromConfig.rateLimit:
    pidCmd[PITCH] = pidCmdPrev[PITCH] + eepromConfig.rateLimit
```

也就是说，Pitch/Yaw 把 `rateLimit` 直接当作每帧变化上限，而不是乘以 `dt`。若 500Hz 稳定运行，等效变化率约为：

```text
0.7854rad/frame * 500frame/s ≈ 392.7rad/s
```

这与 Roll 的约束强度不是一个量级。教材不能把三轴速率限制写成完全一致；仿真脚本采用统一 `rate_limit` 时，也不能证明三轴固件实际限速行为一致。

### 5.9 仿真脚本与固件D项差异

`tools/pid_tuning_sim.py` 的 `update_pid()` 与固件总体相似，但当前存在两个细节差异。

第一，D 项初值。固件在 `D_ERROR` 路径中，如果 `lastDcalcValue == 0.0f`，
会先把它设为当前 `error`，从而减小第一次 D_ERROR 差分尖峰。脚本中
`pid.last_d_calc == 0.0` 时统一设为 `state`，在阶跃刚出现时可能产生与固件不同的 D 项尖峰。

第二，角度跨界。固件在 `D_ERROR` 路径中对 `error - lastDcalcValue` 再做
`standardRadianFormat()`，脚本先把 `error` 包裹到 `[-pi, pi]`，但没有对
`error - last_d_calc` 再单独包裹。跨过 `±pi` 边界时，仿真 D 项可能与固件不同。

这些差异不否定工具价值，但它们限制了工具的证明力。工具适合比较趋势，不适合直接证明某组 D 参数在固件里一定无尖峰。

### 5.10 二阶对象的离散积分

`run_simulation()` 中的对象模型不是连续运行的模拟器，而是在每个 `dt` 步长上做离散更新。脚本中的核心计算等价于：

```text
a[k] = plant_gain * u[k] + disturbance[k]
     - 2*zeta*wn*v[k]
     - wn^2*theta[k]

v[k+1] = v[k] + a[k] * dt
theta[k+1] = wrap(theta[k] + v[k+1] * dt)
```

这里的 `v` 对应 `theta_dot`，`a` 对应 `theta_ddot`。脚本先更新速度，再用更新后的速度更新角度，因此是半隐式欧拉形式。

这个细节会影响仿真解释。若把 `dt` 调得过大，或者把 `wn` 调得很高，曲线震荡可能来自数值离散误差，而不一定来自 PID 参数本身。默认 `dt=0.002s`、`wn=7` 时：

```text
dt * wn = 0.002 * 7 = 0.014
```

该值很小，适合作为教学预演起点。若读者修改 `dt` 或对象参数，应先确认仿真步长仍足够细，再把曲线趋势用于调参判断。

### 5.11 限幅主导与调参误读

脚本中的输出链路为：

```text
raw PID output
-> cmd_limit限幅
-> rate_limit限速
-> control曲线
-> 二阶对象模型
```

当前图形界面绘制的是最终 `control`，也就是经过命令限幅和速率限制后的结果。它没有单独绘制原始 PID 输出、命令限幅后的输出和速率限制触发状态。

因此，看到响应慢时不能立刻判断“P 太小”；看到控制曲线平滑时，也不能立刻判断“没有饱和”。更严谨的判断是分层记录：

- `u_pid`：`update_pid()` 原始输出。
- `u_clamped`：经过 `cmd_limit` 后的输出。
- `u_applied`：经过 `rate_limit` 后的最终输出。
- `limit_active`：是否触及命令限幅。
- `rate_active`：是否触及速率限制。

固件侧 Roll 已经有 `rollDiag.pidRaw`、`rollDiag.pidClamped`、
`rollDiag.pidApplied` 和 `rollDiag.stepLimit`，适合作为分层观察样例。
Pitch/Yaw 当前没有同等级诊断结构，调参时不能假定三轴都有相同可观测性。

### 5.12 调参状态重置与可比性

一次调参实验要能比较，除了 P/I/D 本身，还要让初始状态尽可能一致。

仿真工具的做法比较简单。`run_and_plot()` 每次运行时都会重新创建：

```text
PIDRuntime(p, i, d)
AxisPlantState()
```

这意味着每次仿真默认从：

```text
i_term = 0
last_d_calc = 0
last_d_term = 0
last_last_d_term = 0
theta = 0
theta_dot = 0
cmd_prev = 0
```

开始。它适合比较“同一初始条件下，不同 P/I/D 的趋势”。

固件运行时不同。`updatePID()` 会持续更新 `iTerm` 和 D 项历史；
`computeMotorCommands()` 还会更新 `pidCmdPrev[]`，用于下一帧速率限制。
如果在不中断控制链的情况下直接修改 P/I/D，旧状态仍会参与后续计算。

因此，固件调参建议至少记录三件事：

- 改参数前的 `iTerm`、`lastDcalcValue`、`lastDterm`、`lastLastDterm`。
- 改参数后是否调用过 `zeroPIDintegralError()`、`zeroPIDstates()` 或轴级状态清零。
- `pidCmdPrev[]` 是否保留旧输出，导致速率限制继续按旧输出差值工作。

这不是要求每次都必须清零。清零能提高实验可比性，但也可能不代表真实连续运行场景。
更稳妥的写法是：调参报告必须说明采用“清零后测试”还是“连续运行在线修改”。
没有这个说明时，响应差异只能作为现象记录，不能直接归因到某个 PID 参数。

### 5.13 调参指标的可计算口径

为了避免“感觉更稳”这类模糊结论，调参记录至少应把曲线转换成指标。
对阶跃目标 `r[k]` 和角度反馈 `y[k]`，角度误差可写为：

```text
e[k] = wrap(r[k] - y[k])
```

常用指标包括：

```text
峰值误差     = max(|e[k]|)
稳态误差     = 观测窗口末段 mean(|e[k]|)
进入误差带时间 = 首次满足 |e[k]| < eps 并保持的时间
控制峰值     = max(|u_applied[k]|)
饱和占比     = 饱和帧数 / 总帧数
限速占比     = 限速帧数 / 总帧数
```

这些指标不是越小越好就结束，还要结合安全和实时性解释。
例如，峰值误差变小但控制峰值长期撞限幅，说明限幅可能正在主导响应；
稳态误差变小但 `iTerm` 贴近限幅，说明 I 项可能在用积分累积补偿其它问题；
响应变快但 `executionTime500Hz` 接近 2000us，则实时裕量变窄。

当前 `tools/pid_tuning_sim.py` 返回 `time`、`target`、`actual` 和最终 `control`，
适合计算误差和最终控制量指标；但它没有单独返回原始 PID 输出、
命令限幅后输出和限速触发标志。固件侧 Roll 有 `rollDiag.pidRaw`、
`rollDiag.pidClamped`、`rollDiag.pidApplied` 和 `rollDiag.stepLimit`，
更适合做分层指标记录。Pitch/Yaw 如果只观察 `pidCmd[]` 和 `outputRate[]`，
饱和与限速判断的证据会弱一些，结论应保守。

## 6. STM32实现机制

从 STM32 角度看，第34章不是新增外设章节，而是把已有机制组合成优化路线。

第一，实时周期来自第26章。PID 的 `deltaT` 由 500Hz 实时控制分支提供，异常时回退到 0.002s。

第二，调试观察来自第33章。ST-LINK/SWD 可以观察 `pidCmd[]`、`outputRate[]`、`sensors.margAttitude500Hz[]`、`eepromConfig.PID[]` 和 `rollDiag`。

第三，串口输出来自第11章和第32章。低频打印适合观察慢变量，但不适合在 500Hz 内大量输出。

第四，电机输出来自第30章。仓库源码可以确认 `PWM_Motor_SetAngle()` 的输出路径和 CCR 写入关系；仓库外电机响应仍按 4.5 节的证据层级处理。

第五，方向链路来自 `computeMotorCommands.c` 和 `drv_pwmMotors.c` 的组合。当前源码中，
Roll 使用 `ROLL_SENSOR_SIGN (-1.0f)` 修正姿态输入，再用 `rollTargetSlew - roll_angle`
形成机械误差，乘以 `mechanical2electricalDegrees[ROLL]` 得到电角误差，最后通过
`ROLL_STATOR_SIGN (-1.0f)` 合成定子电角；Pitch 使用 `PITCH_SENSOR_SIGN (-1.0f)`，
并在 `return_state == false` 时把 `current_elec` 写成 `-pitch_angle * mechanical2electricalDegrees[PITCH]`，
随后设置 `mechanical2electricalDegrees[PITCH] = 20.0f`；Yaw 没有单独的
`YAW_SENSOR_SIGN` 宏，而是用原始 `sensors.margAttitude500Hz[YAW]`、包裹后的机械误差、
`-current_elec` 和 `YAW_STATOR_SIGN (+1.0f)` 合成电角。调参时不能把三轴方向链路当成同一种公式。

第六，性能观察来自 DWT 和 `micros()`。`main.c` 中启用了 `DWT->CYCCNT`，并用
`executionTime500Hz = micros() - currentTime` 记录 500Hz 分支耗时。调参时如果加入
更多打印、矩阵运算或复杂滤波，必须观察 500Hz 分支是否仍有周期裕量。

第七，当前 Debug 构建使用 `-mcpu=cortex-m3`、`-mfloat-abi=soft` 和 `-O0`。
STM32F103/Cortex-M3 没有硬件 FPU，浮点 PID、`sinf/fmodf`、AHRS 和调试打印都可能
增加耗时。教材不能只从算法公式推断实时性，必须用 `dt500Hz` 和 `executionTime500Hz` 验证。

第八，调参停止条件也要依赖 MCU 侧证据。若 `dt500Hz` 超出合理范围、
`executionTime500Hz` 接近 2000us、`pidCmd[]` 长期饱和、或 `outputRate[]`
长期受限，当前现象不应继续简单归因于 P/I/D。硬件温升、电流和机械撞限仍属于仓库外证据【待验证】。

## 7. 项目中的应用

本章涉及的项目证据文件包括：

- `tools/pid_tuning_sim.py`
- `Drivers/SRC/Src/pid.c`
- `Drivers/SRC/Inc/pid.h`
- `Drivers/SRC/Src/computeMotorCommands.c`
- `Drivers/SRC/Src/config.c`
- `Core/Src/main.c`

项目中的调参证据链是：

```text
main.c中的当前PID覆盖值
-> pid_tuning_sim.py解析P/I/D默认值
-> pid_tuning_sim.py运行离线响应预演
-> pid.c在固件中实际计算PID输出
-> computeMotorCommands.c执行限幅、速率限制和电角合成
-> PWM_Motor_SetAngle()输出到电机
```

其中，仿真工具用于“预演趋势”，仓库源码用于“确认软件路径”，仓库外实测用于“确认实际效果”。

方向正确性还要另列一条证据链：

```text
sensors.margAttitude500Hz[]
-> ROLL/PITCH_SENSOR_SIGN或Yaw原始角
-> target - state或rollTargetSlew - roll_angle
-> mechanical2electricalDegrees[]、显式负号和电角偏置
-> ROLL/PITCH/YAW_STATOR_SIGN
-> PWM_Motor_SetAngle(MOTOR_*)
-> TIMx->CCRn
-> 物理轴响应方向【待验证】
```

当前 `PWM_Motor_SetAngle()` 的实际写寄存器关系为：`MOTOR_PITCH` 写
`TIM3->CCR2/CCR3/CCR4`，`MOTOR_ROLL` 写 `TIM3->CCR1`、`TIM2->CCR4`、`TIM2->CCR3`，
`MOTOR_YAW` 写 `TIM4->CCR4`、`TIM2->CCR2`、`TIM4->CCR3`。`drv_pwmMotors.h`
中的枚举注释可作为线索，但当注释和实现细节需要核对时，应以 `drv_pwmMotors.c`
中的实际 CCR 写入为硬证据。

还要注意，`PITCH_STATOR_SIGN` 当前取值是 `2.0f`，它不只是正负方向标志，还改变了
PID 输出进入电角合成时的比例。因此教材中把它笼统称为“方向符号”是不精确的；更准确的说法是
“定子电角合成系数”，其中包含方向和幅值缩放两层含义。

## 8. 代码分析

### 8.1 parse_firmware_defaults()

该函数先给出内置默认值，然后尝试从源码覆盖。

它在 `main.c` 中用正则寻找：

```text
eepromConfig.PID[ROLL_PID].P/I/D = ...
eepromConfig.PID[PITCH_PID].P/I/D = ...
eepromConfig.PID[YAW_PID].P/I/D = ...
```

这点很重要，因为当前项目在 `init_eepromConfig(true)` 之后又在 `main.c` 中覆盖了部分 PID 参数。仿真工具读取 `main.c`，比只读取 `config.c` 更接近当前运行值。

但这里要再拆出一个边界：`parse_firmware_defaults()` 当前解析的是 PID 数值参数，而不是完整 PID 语义配置。

`config.c` 中三轴还设置了：

```text
eepromConfig.PID[ROLL_PID].dErrorCalc  = D_ERROR;
eepromConfig.PID[ROLL_PID].type        = ANGULAR;
eepromConfig.PID[PITCH_PID].dErrorCalc = D_ERROR;
eepromConfig.PID[PITCH_PID].type       = ANGULAR;
eepromConfig.PID[YAW_PID].dErrorCalc   = D_ERROR;
eepromConfig.PID[YAW_PID].type         = ANGULAR;
```

这两个字段不是普通显示参数。`dErrorCalc` 决定 D 项按误差变化还是状态变化计算，`type` 决定误差和差分是否按角度跨界包裹。当前脚本中的 `PIDRuntime.d_error_calc = True` 对应 `D_ERROR`，`update_pid()` 默认对误差使用 `wrap_pi()`，因此与当前 `config.c` 的三轴默认语义相符。

边界在于：这种相符来自脚本默认行为，而不是来自解析结果。如果以后固件把某个轴改为 `D_STATE`，或者把 `type` 改成非 `ANGULAR`，当前仿真工具不会自动跟随这个语义变化。尤其是非 `ANGULAR` 输出分支还会使用 `B`，而当前脚本没有建模这个分支。教材中说“加载固件默认值”时，必须限定为加载 P/I/D、命令限幅和速率限制等已解析字段，不能理解为已经加载了所有 PID 行为开关。

#### 仿真默认值解析的新鲜度边界

`parse_firmware_defaults()` 读取的是当前工作区源码文本，而不是当前 Debug ELF、`.map`、
`.list` 或目标板 RAM。脚本入口中：

```text
project_root = Path(__file__).resolve().parents[1]
```

随后用 `Path.read_text()` 分别读取 `Core/Src/main.c`、`Drivers/SRC/Src/computeMotorCommands.c`
和 `Drivers/SRC/Src/config.c`，再用正则表达式提取 P/I/D、命令限幅和 `rateLimit`。
这意味着“加载固件默认值”按钮实际加载的是仓库文件系统中的当前文本快照。

这个机制有三个调试边界。

第一，它比只看 `config.c` 更贴近当前源码意图，因为它能读到 `main.c` 在
`init_eepromConfig(true)` 之后写入的 PID 覆盖值；但它不能证明这些值已经进入
`Debug/Three-axis_cloud_platformV2.elf`，也不能证明板上 Flash 或 RAM 已经是这些值。

第二，它不是 C 编译器，也不运行预处理器。正则只按文本查找赋值形态，不理解条件编译、
宏展开、函数是否实际调用、赋值是否位于不可达分支，也不自动识别注释中的历史赋值。
当前仓库可用它快速读取 P/I/D，但若以后有人在注释或调试分支中保留同形态赋值，
仿真工具可能读到并不代表当前固件路径的数值【待验证】。

第三，它的结果和 Debug `.list` 可能不一致。出现不一致时，调参记录应拆成：

```text
脚本解析值
-> 当前源码文本值
-> 当前构建产物反汇编值
-> 板上 RAM 观测值【待验证】
```

只有这四层证据对齐，才能把“仿真使用的默认值”和“目标板正在运行的 PID 参数”
写成同一组数。否则，仿真曲线只能代表当前工作区文本对应的离线预演。

当前仓库已经能看到一个具体的新鲜度样例：

| 轴 | 当前 `main.c` 覆盖值 | Debug `.list` 中历史构建值 | 教学边界 |
|---|---|---|---|
| Yaw | `P=0.03f, I=0.01f, D=0.016f` | `P=2.0f, I=0.0f, D=0.016f` | 不能把 `.list` 中的 Yaw P/I 当成当前源码文本值 |
| Pitch | `P=0.01f, I=0.0f, D=0.008f` | `P=0.5f, I=0.05f, D=0.008f` | 不能把旧构建反汇编直接当成当前调参基线 |
| Roll | `P=0.03f, I=0.0f, D=0.008f` | `P=0.03f, I=0.0f, D=0.008f` | 单个轴碰巧一致，也不能证明整份构建产物新鲜 |

这个表的作用不是判断哪组参数更好，而是提醒读者：调参记录必须写清楚“参数来自当前源码、脚本解析、构建产物，还是板上 RAM”。如果 `.list` 与源码不一致，应先重新构建并下载，或把结论降级为“历史 Debug 产物证据”。

### 8.2 命令限幅解析

工具从 `computeMotorCommands.c` 中读取：

```text
ROLL_CMD_LIMIT_RAD
PITCH_CMD_LIMIT_RAD
YAW_CMD_LIMIT_RAD
```

这对应固件中 `pidCmd[]` 的轴级限幅。调参时，如果仿真输出长期撞到 `cmd_limit`，说明继续增大 P 或 D 可能不会带来更大输出，只会让控制长期饱和。

这里还要区分“脚本已经解析”和“界面允许直接修改”。当前 `PIDSimulatorApp` 会把 `cmd_limit` 作为内部约束传入 `run_simulation()`，但图形界面没有为 Roll/Pitch/Yaw 的 `cmd_limit` 提供单独输入框。因此读者可以从源码证据确认命令限幅，也可以通过扩展脚本或临时调整被解析的限幅常量做离线对照；但不能把当前界面理解为已经支持直接编辑三轴命令限幅。

### 8.3 rateLimit解析

工具从 `config.c` 中读取：

```text
eepromConfig.rateLimit = 45.0f * D2R
```

并把它换算为弧度值。

边界是：脚本把 `rate_limit` 作为每一步的通用变化限制；固件里 Roll 与 Pitch/Yaw 的具体用法存在差异。因此脚本可以帮助理解速率限制效果，但不能证明三轴固件行为完全一致。

### 8.4 update_pid()

脚本中的 `update_pid()` 与固件 `updatePID()` 保持相似：

- 异常 `dt` 回退到 `DT_DEFAULT`。
- 误差归一化到 `[-pi, pi]`。
- `i_hold` 为 false 时累加积分。
- 积分限制到 `[-10, 10]`。
- D 项使用 20Hz 对应的 RC 低通。
- D 项限制到 `[-300, 300]`。
- 输出异常时置 0。

这让读者可以在离线环境中先理解 P/I/D 对响应趋势的影响。

### 8.5 run_simulation()

`run_simulation()` 对每个轴循环执行：

```text
生成目标角
-> update_pid()
-> cmd_limit限幅
-> rate_limit限制变化
-> 加入扰动
-> 二阶对象积分
-> 保存target/actual/control
```

它输出目标、实际和控制量曲线。曲线适合比较不同参数的响应速度、超调、震荡和扰动恢复趋势。

### 8.6 图形界面

`PIDSimulatorApp` 提供：

- 三轴 P/I/D 输入。
- 模式选择：阶跃、扰动、两者。
- 阶跃轴选择：横滚、俯仰、航向、全部。
- 仿真时间、dt、阶跃幅值、扰动强度、二阶对象参数、速率限制等输入。
- “运行仿真”和“加载固件默认值”按钮。

当前界面暴露了 `rate_limit` 输入，但没有暴露 `cmd_limit` 输入。`cmd_limit` 已经参与仿真计算，却属于“解析后内部使用”的约束量；若要把它也作为交互式变量，需要先给脚本增加三轴命令限幅输入或记录字段。

这说明它更像一个教学和调参辅助台，而不是自动调参器。它不会根据仓库外实测数据自动反推最优 PID。

### 8.7 固件中的当前PID来源

`config.c` 中有一组默认 PID 参数，但 `main.c` 初始化阶段又覆盖 Yaw、Pitch、Roll 的 P/I/D、D 项历史和 `windupGuard`。

因此，判断当前固件运行 PID 时，应以实际初始化后最终值为准。仿真工具读取 `main.c` 的覆盖值，是为了贴近这一点。

当前工具按仓库源码解析出的值为：

```text
ROLL  : P=0.03, I=0.0,  D=0.008
PITCH : P=0.01, I=0.0,  D=0.008
YAW   : P=0.03, I=0.01, D=0.016
```

命令限幅为：

```text
ROLL_CMD_LIMIT_RAD  = 0.15
PITCH_CMD_LIMIT_RAD = 1.5
YAW_CMD_LIMIT_RAD   = 1.0
```

`rateLimit` 从 `config.c` 中的 `45.0f * D2R` 解析得到：

```text
rateLimit = 45deg/s = 0.7854rad/s
```

这些数值来自源码解析，不等于已经通过实物调参验证。

#### PID参数值证据边界

调参时最容易混淆的是“看到某个 P/I/D 数值”到底来自哪一层证据。本项目至少要分成以下几层：

| 层级 | 当前证据 | 能证明什么 | 不能证明什么 |
|---|---|---|---|
| 配置默认值 | `Drivers/SRC/Src/config.c` 中 `init_eepromConfig(true)` 写入三轴默认 PID | 复位配置对象时的默认参数来源 | 不能证明启动后最终仍使用这些默认值 |
| 当前源码覆盖值 | `Core/Src/main.c` 在 `init_eepromConfig(true)` 之后覆盖 Yaw/Pitch/Roll 的 P/I/D 与部分状态字段 | 当前源码文本中的最终覆盖意图；仿真工具按这一层解析 P/I/D | 不能证明 Debug 产物已经重新构建，也不能证明板上 RAM 已是这些值 |
| 构建产物反汇编值 | `Debug/Three-axis_cloud_platformV2.list` 中也能看到 PID 赋值指令和源码注释片段 | 能证明某次 Debug 构建里曾经包含一组 PID 覆盖路径 | 当 `.list` 显示的数值与当前 `main.c` 不一致时，不能用它证明当前源码值已经进入固件 |
| 板上 RAM 观测值 | 调试器读取 `eepromConfig.PID[axis].P/I/D`【待验证】 | 能证明目标板当前运行镜像中的实际参数值【待验证】 | 不能单独证明控制方向、稳定性、温升或长期安全 |
| 闭环效果证据 | 阶跃响应、超调、振荡、电流、温升和机械限位记录【待验证】 | 才能评价这组参数是否适合当前硬件条件【待验证】 | 不能外推到不同负载、不同固件或不同安装方向 |

当前复查时还要注意一个具体边界：`Core/Src/main.c` 当前文本中的覆盖值与
`Debug/Three-axis_cloud_platformV2.list` 中显示的部分 PID 覆盖值并不完全一致。
例如当前源码文本中 Yaw/Pitch/Roll 的覆盖值是：

```text
YAW   : P=0.03, I=0.01, D=0.016
PITCH : P=0.01, I=0.0,  D=0.008
ROLL  : P=0.03, I=0.0,  D=0.008
```

而当前 `.list` 对应片段仍能看到另一组历史赋值，例如 Yaw `P=2.0f`、`I=0.0f`，
Pitch `P=0.5f`、`I=0.05f` 等。教材因此只能把 `.list` 用作“某次 Debug 产物路径证据”，
不能把它直接写成当前源码已经重新构建并烧录的参数证据。若要确认板上实际值，应重新构建、
记录 ELF/map/list 时间戳和提交号，并在调试器中读取 `eepromConfig.PID[]`。

#### 参数新鲜度复核顺序

把上表用于真实调参时，还要再拆出“新鲜度”这一层。因为同一组 P/I/D 可能同时存在于 `config.c` 默认值、`main.c` 覆盖值、仿真脚本解析结果、Debug `.list` 历史构建产物和板上 RAM 观测值中，任何一层没有对齐，都不能把调参记录写成“当前固件参数已经确认”。

| 复核顺序 | 应检查的对象 | 通过后可以写入的结论 | 未通过时的处理 |
|---|---|---|---|
| 1 | `Core/Src/main.c` 中 `init_eepromConfig(true)` 之后的 P/I/D 覆盖语句 | 当前源码文本的启动覆盖意图已经明确 | 只记录源码草稿值，不进入构建或实测结论 |
| 2 | `tools/pid_tuning_sim.py` 的 `parse_firmware_defaults()` 输出 | 离线仿真使用的 P/I/D、`cmd_limit` 和 `rateLimit` 与当前工作区文本一致 | 回到脚本正则和源码赋值形式，避免把解析失败当成控制效果 |
| 3 | 重新构建后的 `.list` / `.map` | `.list` 可核对赋值反汇编路径，`.map` 可核对 PID 函数和相关全局对象进入链接 | 不把旧 `.list` 中的历史赋值写成当前固件证据 |
| 4 | 下载记录和调试器 RAM 观察【待验证】 | 目标板当前运行镜像中的 `eepromConfig.PID[]` 数值已经被观察到【待验证】 | 只能停留在构建产物层，不能宣称板上已经运行同一参数 |
| 5 | 阶跃响应、电流、温升、机械限位和长时间运行记录【待验证】 | 参数对当前硬件负载的效果和安全性已有实测支撑【待验证】 | 只形成调参计划或风险清单，不能写成稳定性结论 |

这个顺序的工程意义是：调参记录不能只写“P 改成了多少”。更可靠的写法应同时标注“源码值、仿真值、构建值、板上 RAM 值、硬件效果值”各自是否已经对齐。尤其当前章节已经发现 `main.c` 文本值与 Debug `.list` 中部分历史赋值不完全一致，因此第 3 层之前的结论都不能越级变成第 4 层或第 5 层结论。

### 8.8 windupGuard字段边界

`PIDdata_t` 中有：

```text
float windupGuard;
```

`config.c` 和 `main.c` 都会给它赋值。但当前 `pid.c` 的 `updatePID()` 没有使用 `PIDparameters->windupGuard`，而是把积分累加值硬限制到：

```text
[-10.0, 10.0]
```

因此，修改 `eepromConfig.PID[...].windupGuard` 不能按当前源码直接改变积分限幅。调参时如果希望改变积分抗饱和边界，应先确认 `pid.c` 是否已改为使用该字段；否则结论保持【待验证】。

### 8.9 holdIntegrators传播边界

`computeMotorCommands()` 开头设置：

```text
holdIntegrators = false
```

Roll 闭环分支中，如果电角误差超过 `ROLL_I_ENABLE_ERR_RAD`，会设置：

```text
rollHoldIntegrators = true
holdIntegrators = true
```

Roll 自己调用 `updatePID()` 时传入 `rollHoldIntegrators`。Pitch/Yaw 调用 `updatePID()` 时传入全局 `holdIntegrators`。

这意味着 Roll 分支的积分暂停状态可能影响同一帧后续 Pitch/Yaw 的积分允许状态。第31章已经提到这种状态耦合；第34章调参时要把它作为观察项，而不是把 Pitch/Yaw 的积分行为看成完全独立。

### 8.10 速率限制差异

Roll 分支使用：

```text
rollStepLimit = eepromConfig.rateLimit * safeDt
```

Pitch/Yaw 分支使用：

```text
pidCmd += eepromConfig.rateLimit
```

当前 `rateLimit=0.7854rad/s`，理想 500Hz 下 Roll 每帧步长约 `0.00157rad`。Pitch/Yaw 若按当前代码直接每帧加减 `0.7854rad`，等效变化率约 `392.7rad/s`。

还有一个诊断字段边界：源码中 `rollDiag.stepLimit` 记录的是
`eepromConfig.rateLimit * safeDt`，而不是经过 `AXIS_MIN_STEP_LIMIT_RAD`
钳位后的最终 `rollStepLimit`。在当前默认值下二者相同量级，但当
`rateLimit * safeDt < 0.001rad` 时，真正参与限速的是 0.001rad，
`rollDiag.stepLimit` 会低估有效步长。判断 Roll 是否被限速时，应同时计算
`max(rollDiag.stepLimit, AXIS_MIN_STEP_LIMIT_RAD)`，并对照
`rollDiag.pidClamped - rollDiag.pidApplied` 或 `pidCmdPrev[]` 的变化。

因此，当仿真工具用统一 `rate_limit` 对三轴限速时，它只能帮助理解“速率限制会改变响应”，不能证明三轴固件限速强度相同。

### 8.11 仿真D项与固件D项差异

脚本的 `update_pid()` 与固件 `updatePID()` 都有 D 项低通和三点平均，但有三个细节边界：

- 固件 `D_ERROR` 初值用当前 `error` 初始化，脚本初值用 `state` 初始化。
- 固件对 `error - lastDcalcValue` 再做角度包裹，脚本没有对这个差分再次包裹。
- 固件的 `dErrorCalc` 和 `type` 来自 `config.c` 的轴级配置，当前脚本没有解析这两个字段，而是用 `PIDRuntime.d_error_calc=True` 和角度误差包裹作为默认行为；若进入非 `ANGULAR` 分支，固件还会使用 `B`，脚本当前也没有建模这条输出路径。

这意味着阶跃瞬间或跨过 `±pi` 边界时，脚本 D 项可能比固件更容易出现差分尖峰。调 D 时，仿真尖峰应视为风险提示，而不是固件必然表现。
当前仓库默认配置中，三轴都是 `D_ERROR` 和 `ANGULAR`，所以脚本默认语义与当前固件默认语义一致；如果固件以后修改这些字段，仿真结论要先复核脚本是否已经同步。

### 8.12 性能证据

当前 `main.c` 启用 DWT 计数器，并记录：

```text
executionTime500Hz = micros() - currentTime
```

构建文件显示 Debug 配置使用 `-mcpu=cortex-m3`、`-mfloat-abi=soft`、`-O0`。因此，PID、AHRS、三角函数和串口打印的耗时都应以板上测量为准。

调参阶段建议同时观察：

```text
dt500Hz
executionTime500Hz
pidCmd[]
outputRate[]
rollDiag.stepLimit
```

如果 `dt500Hz` 明显偏离 0.002s，先排查任务阻塞和性能问题，再解释 PID 曲线。

### 8.13 仿真对象参数的代码证据

`PIDSimulatorApp` 默认对象参数为：

```text
wn = 7.0
zeta = 0.85
plant_gain = 28.0
```

这些参数只在 `tools/pid_tuning_sim.py` 的界面和 `run_simulation()` 中使用。
当前没有证据表明固件会读取它们，也没有仓库内实测记录证明它们已经由真实云台辨识得到。

`run_simulation()` 对三轴循环时复用同一组 `wn/zeta/plant_gain`。
因此，若仿真中三轴响应相似，那首先说明三轴被放进了同一个对象假设；
不能据此推出真实 Roll、Pitch、Yaw 的惯量、摩擦、驱动能力或安装方向一致。

### 8.14 限幅观察的代码证据

脚本中输出先经过：

```text
u = update_pid(...)
u = clamp(u, -cmd_limit[axis], cmd_limit[axis])
du = u - state.cmd_prev
```

之后才按 `rate_limit` 修改最终 `u`。脚本保存到 `control[axis][i]`
的是最终 `u`，不是原始 PID 输出。

固件侧 Roll 分支提供更细的诊断字段：

```text
rollDiag.pidRaw
rollDiag.pidClamped
rollDiag.pidApplied
rollDiag.stepLimit
rollDiag.holdI
```

这使 Roll 更适合做“原始输出 -> 限幅输出 -> 最终输出”的分层调试。
Pitch/Yaw 当前只容易观察到 `pidCmd[]` 和 `outputRate[]`，缺少与 Roll 完全对称的诊断字段。
教材在调参任务中应提醒读者：可观测性不同会影响调参结论的可信度。

### 8.15 PID状态清零与可比性

`pid.c` 中 `initPID()` 会在初始化时清零：

```text
iTerm
lastDcalcValue
lastDterm
lastLastDterm
```

启动收敛结束时，`main.c` 还会调用：

```text
zeroPIDintegralError()
zeroPIDstates()
```

这说明项目知道 PID 历史状态会影响接通瞬间的控制输出。

运行过程中，`updatePID()` 会继续更新同一组状态。若调试者在线修改
`eepromConfig.PID[...].P/I/D`，这些历史状态不会因为参数改变而自动清零。
此外，`computeMotorCommands.c` 中的 `pidCmdPrev[]` 也会保留上一帧最终输出，
继续影响下一帧速率限制。

源码提供了清零入口：

```text
setPIDintegralError()
zeroPIDintegralError()
setPIDstates()
zeroPIDstates()
```

但是否调用它们属于调参策略。若目标是比较两组参数本身，清零积分和 D 项历史能减少历史状态干扰。
若目标是观察在线修改参数时的连续运行表现，则不清零也有意义。
关键是记录清楚实验条件，不能把“清零后响应”和“在线连续响应”混在一起比较。

### 8.16 指标记录能力边界

`tools/pid_tuning_sim.py` 的 `run_simulation()` 返回：

```text
time
target
actual
control
```

这说明脚本内部已经具备计算目标、反馈误差和最终控制输出指标的基础。
但当前 GUI 只绘制曲线，没有自动给出超调、稳态误差、进入误差带时间、
饱和占比或限速占比。

脚本中 `control` 保存的是经过 `cmd_limit` 和 `rate_limit` 后的最终控制量。
因此，若要判断“PID 原始输出是否过大”，当前脚本需要额外记录 `update_pid()`
刚返回的 `u_pid`，以及经过命令限幅后的 `u_clamped`。
在未增加这些记录前，仿真曲线只能证明最终控制量形状，不能完整证明限幅是否未参与。

固件侧 Roll 的 `rollDiag` 更适合做指标化记录，因为它同时保留原始、限幅后和最终应用输出。
Pitch/Yaw 没有同等级诊断字段，若只靠 `pidCmd[]` 和 `outputRate[]`，
应把“限幅是否长期主导”和“限速是否长期主导”写成较弱结论。

### 8.17 构建产物证据边界

源码能说明设计意图和当前文本状态，`.map` / `.list` 能进一步说明某个 Debug 构建产物中实际进入镜像的符号和反汇编路径。

当前仓库的 `Debug/Three-axis_cloud_platformV2.map` 中，`computeMotorCommands`、`updatePID()`、
`zeroPIDintegralError()`、`zeroPIDstates()` 和 `PWM_Motor_SetAngle()` 都有最终 Flash 地址；
`eepromConfig`、`sensors`、`outputRate`、`pidCmd` 和 `pidCmdPrev` 都有最终 SRAM 地址。
这能证明该 Debug 构建产物没有把 PID 计算、PID 状态清零、传感器对象、配置对象和电机输出链路整体裁掉。

`Debug/Three-axis_cloud_platformV2.list` 提供了更细的路径证据：

- `main()` 在启动收敛计数到达阈值后调用 `zeroPIDintegralError()` 和 `zeroPIDstates()`。
- `main()` 在 500Hz 分支中把 `dt500Hz` 传入 `computeMotorCommands()`。
- `computeMotorCommands()` 内部可以看到 Roll、Pitch、Yaw 轴分别调用 `updatePID(..., &eepromConfig.PID[axis])` 的反汇编调用。
- 同一函数内还能看到 `pidCmd[]`、`pidCmdPrev[]`、`outputRate[]`、`eepromConfig.rateLimit` 和 `PWM_Motor_SetAngle()` 相关路径。

除了地址和调用路径，Debug 目录中的 `.su` / `.cyclo` 还能给出该次编译的静态栈估计和圈复杂度。对调参来说，它们的价值不是证明“已经实时稳定”，而是帮助读者知道哪些函数在后续实测中更值得优先观察耗时、栈余量和分支覆盖。

| 函数 | 静态栈估计 | 圈复杂度 | 证据文件 | 调参含义 |
|---|---:|---:|---|---|
| `main()` | 56 字节 | 7 | `Debug/Core/Src/main.su` / `Debug/Core/Src/main.cyclo` | 500Hz 分支、门控和调参入口所在的主循环框架。 |
| `SysTick_Handler()` | 16 字节 | 5 | `Debug/Core/Src/stm32f1xx_it.su` / `Debug/Core/Src/stm32f1xx_it.cyclo` | HAL tick 与时间基准相关，但不能单独证明 `dt500Hz` 抖动。 |
| `MPU6050_Read_And_Process()` | 64 字节 | 3 | `Debug/Drivers/CustomDrivers/Src/mpu6050.su` / `Debug/Drivers/CustomDrivers/Src/mpu6050.cyclo` | PID 输入链路上游的传感器读取与处理入口。 |
| `MargAHRSupdate()` | 80 字节 | 33 | `Debug/Drivers/SRC/Src/MargAHRS.su` / `Debug/Drivers/SRC/Src/MargAHRS.cyclo` | 姿态估计分支较多，调参前应把姿态输入异常与 PID 参数问题分开。 |
| `standardRadianFormat()` | 16 字节 | 3 | `Debug/Drivers/SRC/Src/pid.su` / `Debug/Drivers/SRC/Src/pid.cyclo` | 角度误差包裹路径进入 PID 计算前的基础处理。 |
| `updatePID()` | 64 字节 | 34 | `Debug/Drivers/SRC/Src/pid.su` / `Debug/Drivers/SRC/Src/pid.cyclo` | P/I/D、积分限幅、D 项处理和状态更新集中在这里，复杂度提示调参记录应拆成内部指标。 |
| `zeroPIDintegralError()` | 16 字节 | 2 | `Debug/Drivers/SRC/Src/pid.su` / `Debug/Drivers/SRC/Src/pid.cyclo` | 可比性验证中要记录是否清过积分状态。 |
| `zeroPIDstates()` | 16 字节 | 2 | `Debug/Drivers/SRC/Src/pid.su` / `Debug/Drivers/SRC/Src/pid.cyclo` | 可比性验证中要记录是否清过 D 项和历史误差状态。 |
| `computeMotorCommands()` | 120 字节 | 48 | `Debug/Drivers/SRC/Src/computeMotorCommands.su` / `Debug/Drivers/SRC/Src/computeMotorCommands.cyclo` | 三轴限幅、限速、电角合成和 PWM 输出调用都集中在这里，是调参链路中最需要实测剖分的函数之一。 |
| `PWM_Motor_SetAngle()` | 80 字节 | 13 | `Debug/Drivers/CustomDrivers/Src/drv_pwmMotors.su` / `Debug/Drivers/CustomDrivers/Src/drv_pwmMotors.cyclo` | 能提示电机输出映射分支规模，但不能证明物理轴方向或三相波形正确。 |

这些数字只能作为本次 Debug 构建、当前编译选项和当前源码状态下的静态证据。它们不能简单相加当作最坏栈深，也不能替代板上 `dt500Hz`、`executionTime500Hz`、DWT / GPIO 翻转或日志测量。尤其在 Cortex-M3 软浮点、Debug `-O0` 条件下，`computeMotorCommands()`、`updatePID()` 和 `MargAHRSupdate()` 的静态复杂度只能说明“值得测”，不能说明“已经超时”或“不会超时”；真实运行时结论仍为【待验证】。

因此，本节可以把“PID 输出链路已经进入该 Debug 构建产物”作为构建证据。
但它不能证明 PID 参数已经调好、负反馈方向一定正确、仿真对象与实物一致、闭环稳定、不会过热或调参过程安全。
如果 `.list` 与当前源码文本或板上现象不一致，还必须重新构建并记录 ELF / map / list 的时间戳、提交号和烧录证据；否则只能说仓库中存在历史 Debug 构建证据，不能说板上正在运行同一镜像。

## 9. 调试方法

本节按统一调试结构组织：现象 -> 可能原因 -> 定位方法 -> 验证步骤 -> 解决方案 -> 经验总结。对第34章而言，核心是把“能不能调参”和“怎么调参”分开：先确认下载诊断、姿态方向、PWM 输出和门控状态，再建立参数基线、状态基线、指标口径和回滚点，最后按 P -> D -> I 的顺序做单轴单参数验证。

### 9.1 现象与可能原因

常见现象包括小 P 后误差反而变大、仿真曲线看似变好但板上无改善、输出长期饱和、响应被速率限制主导、加入 I 后偏置或震荡加重、在线改参前后不可比较，或 500Hz 周期不稳定导致调参结论漂移。可能原因包括第33章下载/底层输出尚未确认、姿态符号或电角方向错误、`PIDdata_t` 历史状态未清零、`pidCmdPrev[]` 继续影响限速、`cmd_limit` 或 `rateLimit` 已接管响应、脚本与固件边界不同，或缺少仓库外安全实测记录。

### 9.2 定位方法：调参前提、方向链和默认参数

第一步，确认进入调参前的前提。

- 第33章的下载调试链路正常。
- 电机底层输出至少有可验证通道。
- `sensors.margAttitude500Hz[]` 姿态角合理。
- `startup_delay_counter`、`return_state`、`return_state_roll` 等门控状态清楚。

第一步内部还要完成单轴方向链验证。

- 用手轻微改变被测轴姿态，记录 `sensors.margAttitude500Hz[]` 原始变化方向。
- 记录符号修正后的控制角，例如 Roll 的 `ROLL_SENSOR_SIGN * sensors.margAttitude500Hz[ROLL]` 和 Pitch 的 `PITCH_SENSOR_SIGN * sensors.margAttitude500Hz[PITCH]`。
- 给很小目标阶跃，只保留很小 P，I/D 保持 0 或接近 0，确认误差绝对值先变小再继续调参。
- Roll 优先记录 `rollDiag.errMechRad`、`rollDiag.errElecRad`、`rollDiag.sensorSign`、`rollDiag.statorSign` 和 `rollDiag.pidApplied`。
- Pitch/Yaw 没有同等级 `rollDiag` 字段时，至少记录目标、当前姿态、机械误差、`pidCmd[]`、`PWM_Motor_SetAngle()` 的 `MotorAxis_t` 枚举和对应 CCR 组。
- 若小 P 后误差变大，先停止调参，回到传感器符号、电角合成系数、`PWM_Motor_SetAngle()` 映射和仓库外物理轴响应检查；此时不能把问题写成“P 太小”。

第二步，加载固件默认参数。

- 运行 `tools/pid_tuning_sim.py`。
- 点击“加载固件默认值”。
- 确认三轴 P/I/D 与当前 `main.c` 覆盖值一致。
- 当前应读到 Roll `0.03/0/0.008`，Pitch `0.01/0/0.008`，Yaw `0.03/0.01/0.016`。
- 通过 `computeMotorCommands.c` 或脚本解析结果确认命令限幅为 Roll `0.15`、Pitch `1.5`、Yaw `1.0`；当前 GUI 不单独显示或编辑这三个 `cmd_limit`。

### 9.3 定位方法：时间基准、状态基线和回滚条件

第三步，确认时间基准。

- 在固件中观察 `dt500Hz` 是否接近 0.002s。
- 观察 `executionTime500Hz` 是否明显接近或超过 2000us。
- 如果 500Hz 周期不稳定，先排查 I2C、串口打印、阻塞调用和性能问题。

第四步，确认调参初始状态。

- 记录 `iTerm`、`lastDcalcValue`、`lastDterm`、`lastLastDterm`。
- 记录 `pidCmdPrev[]`，判断速率限制是否会受上一帧输出影响。
- 如果要比较两组参数本身，应说明是否调用 `zeroPIDintegralError()` 和 `zeroPIDstates()`。
- 如果要观察在线修改效果，应明确不清零状态，并把前一组参数的残留状态写入记录。

第五步，建立本轮试验基线。

- 固定被测轴、目标阶跃幅度、测试时长和初始姿态。
- 固定是否清零 PID 状态，以及是否保留 `pidCmdPrev[]`。
- 记录上一组可回退参数，包括 P/I/D、`cmd_limit`、`rateLimit` 和门控状态。
- 先写下停止条件，例如方向反、持续饱和、明显震荡、周期超预算或仓库外安全异常【待验证】。

### 9.4 验证步骤：按 P、D、I 顺序调参

第六步，先只调 P。

- I 和 D 先保持 0 或很小。
- 观察阶跃响应是否方向正确。
- 若方向错误，不要继续增大 P，应先回到姿态符号、电机映射和电角方向检查。
- 记录 `pidCmd[]` 是否还没有撞到轴级 `cmd_limit`。

第七步，再加入 D。

- D 用于抑制超调和震荡。
- 观察控制输出是否出现尖峰。
- 结合第28章的 D 项滤波和第32章的异常防护检查 `dTerm`。
- 若只在仿真中出现尖峰，应复核 8.11 节的脚本/固件 D 项差异。
- 修改 D 后尤其要记录是否清零 `lastDcalcValue`、`lastDterm` 和 `lastLastDterm`。

第八步，最后谨慎加入 I。

- I 用于消除长期静差。
- 如果系统还存在方向错误、门控不清或输出长期饱和，先不要加入 I。
- 仓库外实测中加入 I 后要观察 `iTerm` 是否持续贴近限幅。
- 不要把 `windupGuard` 修改当成已经改变积分限幅；当前 `pid.c` 使用硬编码 `[-10, 10]`。
- 修改 I 后要记录原有 `iTerm` 是否保留；否则静差改善或偏转不能只归因于新 I 参数。

### 9.5 验证步骤：限幅、限速、输出分层和指标化

第九步，检查限幅和速率限制。

- 如果 `pidCmd[]` 长期等于 `ROLL/PITCH/YAW_CMD_LIMIT_RAD`，说明输出饱和。
- 如果 `outputRate[]` 长期被限制，说明变化率约束正在主导响应。
- Roll 还应观察 `rollDiag.stepLimit`，并手动与 `AXIS_MIN_STEP_LIMIT_RAD` 取较大值，确认是否由有效 `rollStepLimit` 限制。
- Pitch/Yaw 要注意当前 `rateLimit` 是直接每帧使用，不能简单等同 Roll。
- 不要把限幅导致的“响应慢”误判为 P 太小。

第十步，分层记录 PID 输出。

- 仿真中至少区分原始 PID 输出、命令限幅后输出和最终控制输出。
- 固件中 Roll 优先记录 `rollDiag.pidRaw`、`rollDiag.pidClamped` 和 `rollDiag.pidApplied`。
- Pitch/Yaw 若只能观察 `pidCmd[]` 和 `outputRate[]`，应在记录表中标明可观测性不足。
- 当最终输出很平滑时，仍要检查是否已经被限幅或速率限制接管。

第十一步，指标化判定。

- 记录峰值误差、稳态误差、进入误差带时间和控制峰值。
- 记录饱和占比、限速占比、`iTerm` 是否贴近限幅。
- 记录 `dt500Hz` 和 `executionTime500Hz`，判断实时性是否仍有裕量。
- 没有这些指标时，只能写“观察到某种现象”，不能写“参数更优”。

### 9.6 解决方案：仓库外实测和调试记录

第十二步，仓库外实测验证。

- 每次只改一个轴、一个参数。
- 每次改动后记录参数、现象、是否震荡、是否过热、是否撞限幅。
- 出现异常时先回退到上一组有记录支撑的可用参数。
- 若没有仓库外实测条件、安全测试条件或测量记录，只能形成调参计划和风险清单；真实效果结论保持【待验证】。

调试记录建议如下。

- 记录调参前提、固件参数来源、仿真参数、单轴单参数改动和回退点。
- 每次调参应记录 P/I/D、PID历史状态、限幅、速率限制、阶跃响应、饱和状态和异常现象。
- 每次记录都应包含基线、停止条件、实际触发条件和是否回滚。
- 仿真结果、源码变量、`.map` / `.list` / ELF 构建产物、烧录记录和仓库外实测结果应分列记录，不能互相替代。
- 方向正确性、不过热、无振荡和闭环稳定性必须有安全实测证据，缺失时保持【待验证】。

### 9.7 经验总结

第34章的调参结论应先证明调参对象可信，再证明参数更优。证据层级回到第4.5节判断，状态基线和可复现快照回到第4.10.1节判断；缺少方向链验证、指标口径和回滚点时，不把某组 P/I/D 写成“已优化参数”。

## 10. 常见问题

问题一：仿真工具能否直接给出最终 PID。

不能。它使用简化对象模型，只适合观察趋势和预演参数影响。

问题二：为什么工具要读取 `main.c` 而不是只读 `config.c`。

因为当前项目初始化后在 `main.c` 中又覆盖了部分 PID 参数。实际运行值要看最终覆盖结果。

问题三：仿真曲线稳定，仓库外实测是否一定稳定。

不一定。仓库外实测还受传感器噪声、安装方向、电机映射、供电、摩擦、结构共振和门控状态影响。

问题四：调 P 以后电机方向明显反了，是否继续调 D。

不应该。方向错误优先检查姿态符号、`PITCH/ROLL/YAW_STATOR_SIGN`、机械角到电角换算和三轴映射。

问题五：响应很慢是否一定是 P 太小。

不一定。可能是输出限幅、速率限制、轴未使能、回中状态未切换或电机功率限制导致。

问题六：为什么 I 要最后加。

因为 I 会积累历史误差。如果系统方向、限幅或门控还没确认，积分可能把问题放大。

问题七：为什么仿真工具不能替代第33章硬件诊断。

仿真工具不接触真实 PWM 引脚和电机驱动，无法验证底层输出链路。

问题八：调参路线是否可以一次性套到所有轴。

不建议。
三轴的传感器符号、机械负载、电机映射、功率限制和门控条件不同。
即使使用同一套方法，也应逐轴记录初始参数、修改项、现象和回退点。
教材给出的是调试顺序，不是保证所有轴共用同一组参数。

问题九：修改 `windupGuard` 是否会改变当前积分限幅。

按当前 `pid.c`，不会直接改变。`updatePID()` 使用硬编码 `[-10, 10]` 限制积分累加值，没有读取 `PIDparameters->windupGuard`。

问题十：仿真中 D 项出现尖峰，是否说明固件一定也会尖峰。

不能直接这样判断。脚本和固件在 D_ERROR 初值处理与角度差分包裹上存在差异。仿真尖峰应作为风险提示，固件表现还要观察 `lastDcalcValue`、`lastDterm`、`lastLastDterm` 和实际输出。

问题十一：Roll、Pitch、Yaw 的速率限制是否完全一样。

当前不是。Roll 使用 `rateLimit * safeDt`，Pitch/Yaw 直接使用 `rateLimit` 作为每帧变化上限。因此响应慢或不慢，必须按轴分别解释。

问题十二：为什么调参时要同时看 `dt500Hz` 和 `executionTime500Hz`。

因为 PID 的 I 项和 D 项都依赖 `dt`。如果 500Hz 周期被 I2C、串口或计算耗时拉长，调参现象可能来自时间基准变化，而不是 P/I/D 本身。

问题十三：在线修改 P/I/D 后，是否应该立即清零 PID 状态。

没有唯一答案，取决于实验目的。
如果要公平比较两组参数本身，应考虑清零 `iTerm` 和 D 项历史，减少旧状态干扰。
如果要观察真实连续运行中的在线修改效果，则可以不清零，但必须记录修改前的
`iTerm`、D 项历史和 `pidCmdPrev[]`。缺少状态记录时，不应把响应差异完全归因于新参数。

问题十四：为什么调参记录必须有回滚点。

因为 PID 输出还受门控、限幅、速率限制、旧积分、D 项历史和硬件状态影响。
如果没有上一组可复现参数，异常出现后只能继续猜测。
回滚点让调试者先恢复到已知状态，再判断新问题是否由本次改动引入。

问题十五：什么时候应该停止继续加大 P/I/D。

当方向错误、`pidCmd[]` 长期饱和、`outputRate[]` 长期受限、`iTerm` 贴近限幅、
`executionTime500Hz` 接近周期预算，或仓库外安全指标异常【待验证】时，
应停止继续加参数，先定位约束或安全问题。

问题十六：为什么方向错不能靠继续调 D 或 I 解决。

因为方向错本质上会把闭环从负反馈变成正反馈。P 项会把误差推大，D 项可能放大错误方向上的快速变化，I 项还会持续累积同一方向的历史误差。正确处理顺序是先验证姿态符号、机械误差符号、电角合成、PWM 映射和物理响应，再回到 P/I/D。

问题十七：为什么 `PITCH_STATOR_SIGN = 2.0f` 不能只理解成“方向符号”。

因为它既决定 `pidCmd[PITCH]` 加到定子电角时的正负方向，也把 PID 输出放大了 2 倍。若只把它当作符号，读者会漏掉 Pitch 轴控制量比例被改变这一事实，后续比较 Pitch 和 Roll 响应速度时就可能误把合成系数差异归因于 PID 参数。

## 11. 实践任务

开始任务前，先回到本章第8节定位源码证据：参数读取看 8.1 至 8.3，仿真计算看 8.4 和 8.5，界面操作看 8.6。
固件当前 PID 来源看 8.7，积分、限速、D 项、性能、对象参数、限幅观察、PID 状态、指标记录和构建产物边界看 8.8 至 8.17。
第9节提供从仿真到仓库外实测的调试顺序。

任务一至任务十一属于仿真、参数、代码证据、构建产物证据和调参基线；任务十二至任务十四属于仓库外调参记录和移植综合设计。

任务一：比较固件参数来源。

分别在 `config.c` 和 `main.c` 中找到 PID 设置，说明为什么最终运行值可能以 `main.c` 覆盖值为准。
验收依据是参数来源表包含 `config.c` 默认值、`main.c` 覆盖值和最终参与运行值三列。

任务二：运行仿真工具。

启动 `tools/pid_tuning_sim.py`，加载固件默认值，记录三轴 P/I/D、`cmd_limit` 和 `rate_limit`。
验收依据是能记录工具读取到的参数，并与任务一中的 `config.c` 默认值和 `main.c` 覆盖值做一次对照；同时说明当前工具没有解析 `dErrorCalc` 和 `type`，只是默认按 `D_ERROR`、角度误差包裹路径运行，也没有建模非 `ANGULAR` 分支中的 `B` 项。

任务三：做单轴阶跃仿真。

选择 Roll，设置阶跃输入，只改变 P，观察响应速度和超调变化。
验收依据是每次只改变一个参数，并记录 P 值、响应速度、超调和输出是否触及限幅。

任务四：加入 D 项。

在保持 P 不变的基础上逐步增加 D，观察曲线中的震荡和控制输出尖峰。
验收依据是记录 D 值、超调、震荡、控制输出尖峰四列，并在备注栏回指第28章 D 项滤波边界。

任务五：观察限幅影响。

优先通过当前 GUI 降低 `rate_limit`，观察响应变慢是否来自速率限制，而不是 PID 参数本身。若要观察 `cmd_limit`，需要先明确当前 GUI 没有三轴命令限幅输入；可以只做源码证据分析，也可以在离线仿真脚本中增加输入字段或临时调整被解析的限幅常量，但必须把这种工具改动写入实验记录。
验收依据是表格分别记录原始 PID 输出、命令限幅后输出、最终控制输出、`outputRate[]`
变化率限制、工具模型限制和固件三轴速率限制差异。当前脚本未单独绘制原始 PID 输出时，
应在结论中标明观察不足，不能直接判断“没有饱和”。
若记录 Roll 轴限速，还必须列出 `rollDiag.stepLimit`、`AXIS_MIN_STEP_LIMIT_RAD`、
有效 `rollStepLimit = max(rollDiag.stepLimit, AXIS_MIN_STEP_LIMIT_RAD)`，
避免把诊断字段误当成最终限速步长。

任务六：复核积分与D项边界。

在 `pid.c` 中找到积分限幅、D 项限幅、低通滤波和三点平均代码。
验收依据是写出 `Iacc` 限幅、`d_raw` 限幅、`alpha` 计算和 `d_avg` 计算四项；同时说明 `windupGuard` 当前没有被 `updatePID()` 使用。
进一步复核 `config.c` 中三轴 `dErrorCalc`、`type` 和 `B` 的当前取值，并判断它们是否与脚本中的 `PIDRuntime.d_error_calc`、误差包裹路径和输出公式一致。

任务七：复核参数修改前后的PID状态。

在 `pid.c` 中找到 `iTerm`、`lastDcalcValue`、`lastDterm`、`lastLastDterm`
的更新位置，以及 `zeroPIDintegralError()`、`zeroPIDstates()` 的清零路径。
验收依据是记录表能区分“清零后测试”和“在线连续修改测试”；
同时记录 `pidCmdPrev[]` 是否会继续影响速率限制。

任务八：设计调参基线和回滚表。

表格至少包含被测轴、固件版本、P/I/D、`cmd_limit`、`rateLimit`、
清零状态、`pidCmdPrev[]`、门控状态、停止条件、回滚参数和异常触发原因。
验收依据是任意一次调参失败后，都能根据表格恢复到上一组有记录支撑的参数。

任务九：复核500Hz时间预算。

在固件中观察 `dt500Hz` 和 `executionTime500Hz`，并记录调参前后的变化。
验收依据是记录表包含目标周期 0.002s、实际 `dt500Hz`、500Hz 分支耗时和是否存在串口/I2C阻塞怀疑项；没有板上记录时保持【待验证】。

任务十：复核仿真对象参数。

在 `tools/pid_tuning_sim.py` 中找到 `wn`、`zeta` 和 `plant_gain` 的默认值，
并说明它们只属于简化二阶模型。
验收依据是记录表明确区分“仿真假设参数”和“仓库外辨识参数”；
没有输入响应日志时，不能把这三个数写成实物标定结果。

任务十一：复核固件 PID 构建产物证据链。

在 `Debug/Three-axis_cloud_platformV2.map` 中找到 `computeMotorCommands`、`updatePID()`、
`zeroPIDintegralError()`、`zeroPIDstates()`、`PWM_Motor_SetAngle()`、
`eepromConfig`、`sensors`、`pidCmd`、`pidCmdPrev` 和 `outputRate` 的最终地址。
再在 `Debug/Three-axis_cloud_platformV2.list` 中找到 `main()` 调用 PID 清零函数、
`main()` 调用 `computeMotorCommands(dt500Hz)`、`computeMotorCommands()` 调用 `updatePID(..., &eepromConfig.PID[axis])`
和调用 `PWM_Motor_SetAngle()` 的片段。
验收依据是记录表能区分源码证据、构建产物证据、烧录证据和仓库外实测证据；没有烧录记录时，不能把 `.map` / `.list` 直接写成板上正在运行的固件。

任务十二：制定仓库外实测调参记录表。

记录轴、P/I/D、PID历史状态、限幅、速率限制、姿态角表现、`pidCmd[]`、`outputRate[]`、电机温升和是否震荡。
验收依据是记录表明确每次只改一个轴、一个参数；若没有仓库外实测条件或安全测试条件，只形成记录模板和风险清单，仓库外实测结论保持【待验证】。

任务十三：设计移植检查清单。

把本教材中的平台、时钟、I2C、姿态、PID、PWM、门控、诊断和调参步骤整理成移植到新板子的检查顺序。
验收依据是清单按平台、时钟、I2C、姿态、PID、PWM、门控、诊断、调参顺序排列，并标出下载调试、电机诊断和异常门控三个不可跳过节点。

任务十四：验证单轴方向闭环证据链。

选择 Roll、Pitch 或 Yaw 中的一轴，建立方向验证表。表格至少包含原始姿态角、符号修正后姿态角、目标角、机械误差、PID 输出符号、电角合成表达式、`MotorAxis_t` 枚举、CCR 组和物理响应方向【待验证】。
验收依据是能够说明该轴从 `sensors.margAttitude500Hz[]` 到 `TIMx->CCRn` 的软件链路，并明确指出哪一步仍需要仓库外实测证明。若小 P 阶跃后误差变大，任务结论应写成“方向链路未通过”，而不是继续给出新 PID 参数。

## 12. 思考题

1. 为什么 PID 调参前必须先确认姿态输入和 PWM 输出，而不是直接调 P/I/D。
2. 为什么仿真工具读取 `main.c` 的覆盖值比只读取默认配置更接近当前固件。
3. 如果仿真中增大 P 后响应变快，但仓库外实测中没有变化，可能被哪些限幅或门控限制了。
4. 为什么 Roll 的速率限制实现不能简单等同于 Pitch/Yaw。
5. 如果 D 项让控制输出尖峰变大，应从滤波、限幅还是姿态噪声开始排查。
6. 如果 I 项导致开机后缓慢偏转，应如何结合启动门控和积分清零排查。
7. 如果要设计类似三轴控制系统，哪些模块可以复用，哪些必须重新标定和验证。
8. 如果一组参数在仿真和实物上的表现不同，教材应先怀疑哪几类证据缺失。
9. 为什么 `windupGuard` 字段存在，不等于当前积分限幅一定由它决定。
10. 为什么 D 项跨过 `±pi` 边界时，要特别检查角度差分包裹。
11. 为什么 Cortex-M3 软浮点调试构建下，调参还要观察 500Hz 执行耗时。
12. 如果 Roll 和 Pitch 的响应速度不同，如何区分 PID 参数差异和速率限制实现差异。
13. 为什么 `wn/zeta/plant_gain` 不能在没有输入响应记录时写成实物标定参数。
14. 为什么只看仿真工具的最终 `control` 曲线，不能完整判断原始 PID 是否已经饱和。
15. 为什么在线修改 PID 参数时，要同时记录 `iTerm`、D 项历史和 `pidCmdPrev[]`。
16. 为什么没有调参基线时，不能说某一组参数一定比上一组更好。
17. 为什么 `.map` / `.list` 能证明 PID 相关符号进入某个 Debug 构建产物，却不能证明参数已经调好或板上正在运行同一镜像。
18. 为什么持续撞限幅时，继续增大 P 可能让现象更难解释。
19. 为什么回滚点属于调参安全设计，而不只是文档记录习惯。
20. 为什么小 P 阶跃后误差变大时，应先验证方向链路，而不是继续增加 D。
21. 为什么 `PITCH_STATOR_SIGN = 2.0f` 同时影响方向判断和控制量比例判断。

## 13. 本章总结

本章把三轴云台教材收束到调参和系统优化路线。

当前项目中：

- `tools/pid_tuning_sim.py` 是离线 PID 调参辅助工具。
- 工具会从 `main.c`、`computeMotorCommands.c` 和 `config.c` 解析 PID、命令限幅和速率限制。
- PID 参数值要区分 `config.c` 默认值、`main.c` 当前覆盖值、仿真工具解析值、Debug `.list` 历史构建值、板上 RAM 值和闭环效果证据。
- 当前工具没有解析 `dErrorCalc` 和 `type`，也没有建模非 `ANGULAR` 分支中的 `B` 项；脚本默认 `D_ERROR` 和角度误差包裹，与当前 `config.c` 默认语义一致，但不是完整固件语义解析。
- 工具的 D 项滤波、积分限幅、D 项限幅和异常回退与固件有对应关系。
- 当前 GUI 暴露 `rate_limit`，但不暴露三轴 `cmd_limit` 输入；命令限幅已参与仿真计算，却不能在现有界面中直接调节。
- 工具使用简化二阶对象模型，只能预演趋势，不能替代仓库外实测验证。
- 固件 `updatePID()` 的积分限幅当前是硬编码 `[-10, 10]`，不是由 `windupGuard` 字段直接决定。
- Roll 的速率限制使用 `rateLimit * safeDt`，Pitch/Yaw 当前直接使用 `rateLimit`，三轴限速强度不能混为一谈。
- 仿真脚本与固件在 D 项初值和角度差分包裹上存在细节差异，D 项尖峰要回到固件变量验证。
- `wn`、`zeta` 和 `plant_gain` 是仿真对象参数，不是固件参数，也不是仓库内已证明的实物辨识结果。
- 脚本当前绘制最终 `control`，调参时应区分原始 PID 输出、命令限幅后输出和最终应用输出。
- 仿真每次运行默认从干净 `PIDRuntime` 和对象状态开始，固件在线修改参数时 PID 历史状态不会自动清零。
- `zeroPIDintegralError()`、`zeroPIDstates()` 和轴级状态清零会影响调参可比性，记录中必须说明是否使用。
- 每轮调参应有基线、单变量改动、指标化判定、停止条件和回滚点。
- 当前脚本可提供目标、反馈和最终控制输出；原始 PID、限幅后输出和限速触发需要额外记录。
- Roll 的 `rollDiag.stepLimit` 记录的是 `rateLimit * safeDt`，最终有效限速还要与 `AXIS_MIN_STEP_LIMIT_RAD` 取较大值后判断。
- Debug 构建使用 Cortex-M3 软浮点和 `-O0`，调参时要同时观察 `dt500Hz` 与 `executionTime500Hz`。
- `.map/.list/.su/.cyclo` 的构建产物结论统一回到第8.17节判断：它们能证明 PID、限速、状态清零、PWM 输出路径和静态资源边界进入某次 Debug 构建，但不能替代烧录记录、板上耗时、栈余量、方向、稳定性或仓库外实测。
- 固件中的 PID 效果不仅由 P/I/D 决定，还受门控、限幅、速率限制、电角换算和硬件映射影响。
- PID 调参前必须先验证负反馈方向链路：姿态符号、机械误差、电角合成、PWM 枚举到 CCR 映射和物理轴响应缺一不可。
- `PITCH_STATOR_SIGN = 2.0f` 不是单纯符号宏，它同时改变方向和控制量比例，Pitch/Roll/Yaw 的调参现象不能只按 P/I/D 数值直接类比。
- 推荐路线是先确认硬件和姿态，再用仿真预演，最后进行小步仓库外实测验证。
- 经典 PID 理论、离线仿真、源码/构建产物、板上观测和硬件安全记录属于不同证据层级，不能互相替代。

本章边界：

- 本章提供调参路线和优化顺序，不给出可直接套用到实物的最终参数。
- 仿真结果、单轴点测和短时稳定都不能替代逐轴记录、回退点和仓库外长期验证。
- 构建产物只能证明符号、地址、反汇编调用路径以及静态栈 / 复杂度边界，不能证明运行时方向、稳定性、温升、安全性或板上镜像版本。
- 静态资源证据不能替代板上时序、栈余量、热状态、物理方向和闭环稳定性验证。
- 真实闭环稳定性、方向正确性、不过热和抗扰性能仍需仓库外安全实测证据。

至此，第01章到第34章形成了从平台、外设、传感器、姿态、控制、电机、调试到优化的完整学习主线。后续继续打磨时，应优先复查事实边界、证据链和实践任务，而不是盲目扩写新主题。

### 章节尾部固定检查

知识链路：

`知识点总表`
-> `知识依赖图`
-> `学习优先级`
-> `教学顺序`
-> `教材章节`

项目证据：

- 文件：`tools/pid_tuning_sim.py`
- 文件：`Drivers/SRC/Src/pid.c`
- 文件：`Drivers/SRC/Inc/pid.h`
- 文件：`Drivers/SRC/Src/computeMotorCommands.c`
- 文件：`Drivers/SRC/Src/config.c`
- 文件：`Core/Src/main.c`
- 文件：`Debug/makefile`
- 文件：`Debug/Three-axis_cloud_platformV2.map`
- 文件：`Debug/Three-axis_cloud_platformV2.list`
- 文件：`Debug/Core/Src/main.su`
- 文件：`Debug/Core/Src/main.cyclo`
- 文件：`Debug/Core/Src/stm32f1xx_it.su`
- 文件：`Debug/Core/Src/stm32f1xx_it.cyclo`
- 文件：`Debug/Drivers/CustomDrivers/Src/mpu6050.su`
- 文件：`Debug/Drivers/CustomDrivers/Src/mpu6050.cyclo`
- 文件：`Debug/Drivers/SRC/Src/MargAHRS.su`
- 文件：`Debug/Drivers/SRC/Src/MargAHRS.cyclo`
- 文件：`Debug/Drivers/SRC/Src/pid.su`
- 文件：`Debug/Drivers/SRC/Src/pid.cyclo`
- 文件：`Debug/Drivers/SRC/Src/computeMotorCommands.su`
- 文件：`Debug/Drivers/SRC/Src/computeMotorCommands.cyclo`
- 文件：`Debug/Drivers/CustomDrivers/Src/drv_pwmMotors.su`
- 文件：`Debug/Drivers/CustomDrivers/Src/drv_pwmMotors.cyclo`
- 函数：`parse_firmware_defaults()`
- 函数：`update_pid()`
- 函数：`run_simulation()`
- 函数：`updatePID()`
- 函数：`computeMotorCommands()`
- 函数：`micros()`
- 函数：`zeroPIDintegralError()`
- 函数：`zeroPIDstates()`
- 函数：`setPIDintegralError()`
- 函数：`setPIDstates()`
- 变量：`DT_DEFAULT`
- 变量：`RC_D_FILTER`
- 变量：`F_CUT`
- 变量：`rc`
- 变量：`iTerm`
- 变量：`lastDcalcValue`
- 变量：`lastDterm`
- 变量：`lastLastDterm`
- 变量：`windupGuard`
- 变量：`holdIntegrators`
- 变量：`cmd_limit`
- 变量：`rate_limit`
- 变量：`wn`
- 变量：`zeta`
- 变量：`plant_gain`
- 变量：`theta`
- 变量：`theta_dot`
- 变量：`theta_ddot`
- 变量：`control`
- 变量：`target`
- 变量：`actual`
- 变量：`pidCmd[]`
- 变量：`pidCmdPrev[]`
- 变量：`outputRate[]`
- 变量：`rollDiag.pidRaw`
- 变量：`rollDiag.pidClamped`
- 变量：`rollDiag.pidApplied`
- 变量：`dt500Hz`
- 变量：`executionTime500Hz`
- 变量：`rollDiag.stepLimit`
- 配置项：`eepromConfig.PID[]`
- 配置项：`eepromConfig.rateLimit`
- 配置项：`-mcpu=cortex-m3`
- 配置项：`-mfloat-abi=soft`
- 配置项：`-O0`

参考资料：

- K. J. Åström, T. Hägglund, `Advanced PID Control`, ISA - The Instrumentation, Systems, and Automation Society, 2006。
- K. J. Åström, T. Hägglund, `PID Controllers: Theory, Design, and Tuning`, Instrument Society of America, 1995。

质量自检：

- P0 事实错误：通过。
- P1 依赖断层：通过。
- P2 逻辑连贯：通过。
- P3 项目证据：通过。
- P4 原理展开：通过。
- P5 调试实践：通过。
- P6 表达统一：通过。

---
> 导航：上一章：[第33章_下载调试与电机硬件诊断](第33章_下载调试与电机硬件诊断.md) ｜ 下一章：无
