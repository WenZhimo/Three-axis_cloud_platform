# 第27章 PID控制核心

> 导航：上一章：[第26章_500Hz实时控制循环](第26章_500Hz实时控制循环.md) ｜ 下一章：[第28章_PID细节与输出约束](第28章_PID细节与输出约束.md)

## 1. 本章目标

第26章已经说明 500Hz 主循环如何按顺序调用传感器读取、姿态解算和 `computeMotorCommands(dt500Hz)`。本章进入闭环控制核心：项目如何把目标角、当前姿态角和 PID 参数组合成控制补偿量。

本章目标是：

- 理解 `PID控制` 在云台闭环中的作用。
- 看懂 `PIDdata_t` 中 `P`、`I`、`D`、`iTerm`、`type`、`dErrorCalc` 等字段的职责。
- 明确 `initPID()`、`updatePID()`、`zeroPIDintegralError()` 和 `zeroPIDstates()` 的主线作用。
- 追踪 `computeMotorCommands(dt500Hz)` 中 Pitch、Yaw 和 Roll 相关 PID 调用入口。
- 区分第27章的 PID 核心与第28章的 D 项滤波、目标斜坡、输出限幅和积分抗饱和细节。

本章知识链为：

`知识点总表` 中的 `PID控制`
-> `知识依赖图` 中 `PID控制` 依赖 `角度归一化`、`传感器数据结构sensors_t` 和 `500Hz实时控制循环`
-> `学习优先级` 中 P1 核心基础知识
-> `教学顺序` 第45项
-> `教材章节` 第27章。

## 2. 前置知识

本章正式前置知识包括：

- `角度归一化`
- `传感器数据结构sensors_t`
- `500Hz实时控制循环`

第23章已经说明 `sensors.margAttitude500Hz` 是当前主线姿态输出，第25章说明它被控制链路读取。

第26章说明 `computeMotorCommands(dt500Hz)` 在 500Hz 实时控制分支中被调用。第39项知识 `角度归一化` 虽然在第23章内已经出现，但在控制中还会继续发挥作用：角度误差必须按圆周最短路径计算，不能简单按普通浮点差值理解。

本章只讲 PID 控制的核心框架：目标值、当前值、误差、时间步长、P/I/D 参数和输出。第28章会继续展开 D 项滤波、输出限幅、目标角斜坡逼近和积分抗饱和。

## 3. 问题背景

三轴云台的目标不是只知道“当前姿态是多少”，而是要让姿态朝目标角移动并稳定下来。姿态解算给出的是当前角度，控制器要回答的是：

- 当前角度和目标角差多少。
- 差得越多，电机补偿是否应该越大。
- 误差持续存在时，是否要逐渐累积补偿。
- 姿态变化太快时，是否要用阻尼抑制震荡。
- 控制计算应该使用哪一帧的 `dt`。

PID 控制就是项目中承担这些问题的核心算法。它把误差拆成三个方向：

```text
P：看当前误差
I：看误差累计
D：看误差或状态变化趋势
```

在本项目中，PID 不是孤立函数。它嵌在 500Hz 实时控制链路中：

```text
sensors.margAttitude500Hz
-> computeMotorCommands(dt500Hz)
-> updatePID(command, state, dt, iHold, PID参数)
-> pidCmd[ROLL/PITCH/YAW]
```

这一章的重点是把“算法公式”和“项目调用位置”扣在一起。

## 4. 核心概念

### 4.1 PID控制

PID 控制器把目标值 `command` 和当前值 `state` 的误差转换为输出：

```text
error = command - state
output = P * error + I * integral(error) + D * derivative(error or state)
```

其中：

- `P` 是比例项，误差越大，输出越大。
- `I` 是积分项，误差持续存在时逐渐累积。
- `D` 是微分项，观察误差或状态的变化趋势，用于抑制快速变化。

这个公式是理解 PID 的入口。项目源码中还包含角度归一化、D 项滤波、输出限幅和 NaN 防护，本章先建立主干，细节放到后续章节逐步拆开。

### 4.2 command与state

`updatePID()` 的函数签名是：

```c
float updatePID(float command,
                float state,
                float deltaT,
                uint8_t iHold,
                struct PIDdata *PIDparameters)
```

在姿态控制中：

- `command` 表示目标角或目标电角。
- `state` 表示当前姿态角或当前电角。
- `deltaT` 是第26章传入的控制周期时间。
- `iHold` 控制是否暂停积分更新。
- `PIDparameters` 指向当前轴的 PID 参数结构。

### 4.3 PIDdata_t

`Drivers/SRC/Inc/pid.h` 定义 `PIDdata_t`：

```c
typedef struct PIDdata
{
    float B, P, I, D;
    float iTerm;
    float windupGuard;
    float lastDcalcValue;
    float lastDterm;
    float lastLastDterm;
    uint8_t dErrorCalc;
    uint8_t type;
} PIDdata_t;
```

这里的 `P/I/D` 是控制参数，`iTerm` 是积分状态。

`lastDcalcValue`、`lastDterm` 和 `lastLastDterm` 是 D 项历史状态。`type` 用于区分角度 PID 和其他形式，`dErrorCalc` 用于选择 D 项来自误差变化还是状态变化。

`windupGuard` 字段虽然存在，并且 `config.c` 与 `main.c` 都会写入它，
但当前 `updatePID()` 的积分限幅没有读取该字段，而是使用函数内部的固定 `[-10, 10]`。
源码中的 PITCH 额外限幅分支当前也重复使用同一组常数，并没有比全局限幅更严格。
教材后续分析积分抗饱和时必须以实际代码为准，不能只根据字段名或源码注释推断行为。

### 4.4 ANGULAR与角度误差

`pid.h` 中定义：

```c
#define OTHER   0
#define ANGULAR 1
```

当 `PIDparameters->type == ANGULAR` 时，`updatePID()` 会对误差调用 `standardRadianFormat()`。

它把角度误差约束到接近 `[-PI, PI]` 的范围。这样可以避免目标角从 `+179°` 到 `-179°` 时被误认为需要转过接近 358°。

### 4.5 ROLL/PITCH/YAW PID索引

`Drivers/BGC/bgc32.h` 中定义：

```c
#define NUMBER_OF_PIDS 3
#define ROLL_PID  0
#define PITCH_PID 1
#define YAW_PID   2
```

项目通过 `eepromConfig.PID[ROLL_PID]`、`eepromConfig.PID[PITCH_PID]`、`eepromConfig.PID[YAW_PID]` 保存三轴 PID 参数和历史状态。

### 4.6 连续PID与离散PID

教材中常见的 PID 写成连续形式：

```text
u(t) = Kp * e(t) + Ki * ∫e(t)dt + Kd * de(t)/dt
```

但 STM32 代码每 500Hz 执行一次，只能按离散采样更新。设第 `k` 帧时间步长为 `dt[k]`，
则项目中的核心近似可以理解为：

```text
e[k] = command[k] - state[k]
I_state[k] = I_state[k-1] + e[k] * dt[k]
D_raw[k] = (e[k] - e[k-1]) / dt[k]
u[k] = P * e[k] + I * I_state[k] + D * D_filtered[k]
```

源码中的 `iTerm` 对应这里的 `I_state`，它保存的是误差积分状态，而不是已经乘过
`I` 参数的最终积分输出。

因此三项的工程单位也不同。若误差单位是弧度、输出也按弧度补偿理解，则：

```text
P: 近似无量纲
I: 约为 1/s
D: 约为 s
```

这解释了为什么 `dt500Hz` 不只是一个辅助变量。它同时进入积分累积和微分斜率计算，
时间步长异常会改变 I 项和 D 项的实际强度。

### 4.7 B参数与当前姿态PID边界

`PIDdata_t` 中还有 `B` 字段。它不是无用字段，但当前三轴姿态 PID 主线并不直接使用它。

`pid.c::updatePID()` 中有两条输出分支。`ANGULAR` 分支为：

```text
output = P * error + I * iTerm + D * dAverage
```

`OTHER` 分支为：

```text
output = P * B * command + I * iTerm + D * dAverage - P * state
```

从公式形态推断，`OTHER` 分支里的 `B` 可理解为目标权重或设定值权重：当 `B=1` 时，
比例项等价于 `P * (command - state)`；当 `B<1` 时，目标值变化对比例项的直接影响会减小。

但 `config.c` 当前把 ROLL/PITCH/YAW 的 `type` 都设置为 `ANGULAR`。所以分析当前云台姿态
PID 时，应以 `ANGULAR` 分支为主，不能把 `B` 写成当前三轴姿态控制中实际生效的参数。

### 4.8 D_ERROR与D_STATE

`dErrorCalc` 决定 D 项看“误差变化”还是看“状态变化”。

当 `dErrorCalc == D_ERROR` 时：

```text
D_raw = (error[k] - error[k-1]) / dt
```

当 `dErrorCalc == D_STATE` 时：

```text
D_raw = (state[k-1] - state[k]) / dt
```

二者在目标值平稳时可能接近，但在目标值突变时不同。`D_ERROR` 会把目标突变也计入微分项，
可能产生更强的瞬态响应；`D_STATE` 主要对测量状态变化提供阻尼，通常能减少目标阶跃带来的
微分冲击。

当前 `config.c` 中三轴默认 `dErrorCalc` 均为 `D_ERROR`。是否适合实物云台，还需要结合目标斜坡、
输出限幅、实际振动和电机响应记录验证；缺少实测时保持【待验证】。

### 4.9 角度包裹函数边界

第23章和第31章已经介绍角度归一化，但第27章还需要补一层源码边界：
`pid.c::standardRadianFormat()` 不是任意角度都能完全归一化的通用函数。

它的实现逻辑是：

```text
if angle >= PI:  return angle - 2*PI
if angle < -PI:  return angle + 2*PI
else:            return angle
```

也就是说，它只做一次加减 `2π`。如果输入已经接近 `[-π, π]`，
或者最多只跨过一个圆周边界，这种轻量处理足够贴合当前 PID 用法。
但如果输入是 `5π` 这类多圈角度，一次相减后得到 `3π`，
仍然没有回到 `[-π, π]`。

控制层中的 `wrapToPif()` 使用 `while` 循环反复加减 `2π`，
更像通用角度包裹函数。因此教材应区分：

- `standardRadianFormat()`：PID 内部轻量单次包裹。
- `wrapToPif()`：控制层更通用的循环包裹。

还有一个边界值差异：`standardRadianFormat(PI)` 会返回 `-PI`，
而 `wrapToPif(PI)` 会保留 `PI`。二者都可表示同一反向角度边界，
但调试跨越 `±π` 时，日志中看到 `PI` 或 `-PI` 不应被误判为方向错误。

### 4.10 闭环控制与开环控制

PID 属于闭环控制。闭环的关键不是“计算一个输出”，而是把输出造成的结果再测回来，
用新一帧测量值继续修正下一帧输出。

在本项目中，闭环可以写成：

```text
目标角 pointingCmd
-> PID 计算补偿量
-> 电角/PWM 输出
-> 云台姿态变化
-> IMU/AHRS 重新测得 sensors.margAttitude500Hz
-> 下一帧 PID 继续修正
```

如果没有反馈，就变成开环控制：程序只按预设电角或功率输出，无法知道云台是否真的到达目标角。
开环适合硬件自检、单相输出测试或固定波形验证，不适合抗扰、抗负载变化和姿态稳定。
这也是为什么第33章的电机硬件诊断可以绕过 PID，而本章的姿态稳定必须进入闭环。

### 4.11 位置式PID与增量式PID

离散 PID 常见两种实现形态：位置式和增量式。

位置式 PID 每帧直接计算当前控制量：

```text
u[k] = P * e[k] + I * sum(e * dt) + D * de/dt
```

增量式 PID 每帧计算控制量变化：

```text
du[k] = u[k] - u[k-1]
u[k] = u[k-1] + du[k]
```

当前 `pid.c::updatePID()` 返回的是 `output`，公式直接由 `error`、`iTerm`
和 `dAverage` 组合得到。它没有保存上一帧 `output` 并返回 `du`。
因此本章应把当前源码解释为位置式离散 PID，而不是增量式 PID。

这一区分影响调试方法。位置式 PID 的输出大小直接反映当前误差、积分状态和 D 项状态；
增量式 PID 则更强调“本帧相对上一帧改多少”。当前项目后续的 `pidCmdPrev` 和
`outputRate` 属于轴控制层的输出速率限制，不等于 `updatePID()` 本身采用增量式 PID。

### 4.12 为什么当前项目采用PID

对三轴云台这种小型嵌入式姿态稳定任务，PID 的优势是模型要求低、计算量小、
参数含义直观，并且能直接和 500Hz 主循环、姿态角误差、输出限幅和调试日志结合。

更复杂的控制方法并不是“更高级就一定更适合”：

- LQR 需要较明确的线性状态空间模型、状态权重和控制权重。
- MPC 需要在线求解带约束优化问题，对模型、计算量和调试工具要求更高。
- 自适应控制或鲁棒控制需要额外的模型假设、稳定性证明和故障边界。

当前仓库没有机械惯量、摩擦、力矩常数、状态空间模型或在线优化求解器的证据。
所以教材不能把项目解释成 LQR/MPC 架构。按现有证据，PID 是当前工程中
最可追踪、最容易在 STM32F103 上落地、也最符合源码结构的闭环控制方案。

## 5. 工作原理

一个 PID 控制周期可以分成六步。

第一步，取得目标值和当前值。

在 `computeMotorCommands()` 中，当前姿态主要来自：

```text
sensors.margAttitude500Hz[ROLL/PITCH/YAW]
```

目标值主要来自：

```text
pointingCmd[ROLL/PITCH/YAW]
```

不同轴会先做符号变换、角度归一化或电角转换。本章只确认目标和当前值会进入 PID 控制路径。

第二步，计算误差。

`updatePID()` 内部先执行：

```c
error = command - state;
```

如果当前 PID 类型是 `ANGULAR`，则继续执行角度归一化：

```c
error = standardRadianFormat(error);
```

第三步，处理积分状态。

当 `iHold == false` 时，函数用 `error * deltaT` 更新 `iTerm`。当 `iHold == true` 时，积分状态保持不变。这样可以在某些误差过大或启动阶段暂时禁止积分累积。

这里要注意 `iTerm` 的含义。它保存的是历史误差面积：

```text
iTerm[k] = iTerm[k-1] + error[k] * deltaT
```

所以如果 `deltaT` 突然变大，同样的误差会一次性累积更多积分。源码中对异常 `deltaT` 有回退，
但这只能说明 PID 内部做了时间步长保护，不能说明姿态解算入口也完成了同样保护。

第四步，计算 D 项。

源码支持两种 D 项来源：

- `D_ERROR`：根据误差变化计算。
- `D_STATE`：根据状态变化计算。

当前默认配置中 ROLL/PITCH/YAW 的 `dErrorCalc` 都设置为 `D_ERROR`。第28章会继续展开 D 项滤波和历史状态。

从控制效果看，D 项可以理解为阻尼。它抑制误差变化过快或状态变化过快造成的摆动。
但它也最怕噪声和时间步长异常，因为除以 `deltaT` 会放大高频抖动。
这就是为什么项目在 D 项后面又叠加滤波和平均历史状态。

还要注意 `D_ERROR` 与 `D_STATE` 的角度包裹位置不同。当前源码中，
`D_ERROR` 先计算 `error - lastDcalcValue`，再对这个角度差做
`standardRadianFormat()`，最后除以 `deltaT`。而 `D_STATE` 分支
先把 `(lastDcalcValue - state)` 除以 `deltaT` 得到变化率，再对这个
变化率调用 `standardRadianFormat()`。

因此不能把 `D_STATE` 简化写成“和 `D_ERROR` 只是换了输入变量”。
当前三轴默认使用 `D_ERROR`，所以 `D_STATE` 属于源码存在但当前主线未启用的边界。
如果未来改用 `D_STATE`，应单独验证跨 `±π` 和高速变化时的 D 项表现【待验证】。

第五步，组合输出。

对于 `ANGULAR` 类型，`updatePID()` 的核心输出形式是：

```text
output = P * error + I * iTerm + D * dAverage
```

这就是本章最重要的 PID 主公式。

对于 `OTHER` 类型，源码使用另一种形式：

```text
output = P * B * command + I * iTerm + D * dAverage - P * state
```

当前三轴姿态 PID 是 `ANGULAR` 类型，所以主线公式不包含 `B`。这个边界很重要：
教材可以解释 `B` 字段，但不能把它误写成当前 ROLL/PITCH/YAW 姿态 PID 的生效参数。

第六步，交还给轴控制逻辑。

`updatePID()` 只返回一个浮点补偿量。它不直接写 PWM。`computeMotorCommands()` 接收这个结果后，后续还会做限幅、速率限制、电角合成和 `PWM_Motor_SetAngle()` 输出。那些内容属于第28章到第30章。

## 6. STM32实现机制

PID 本身不是 STM32 外设，而是运行在 500Hz 主循环中的 C 算法。它和 STM32 的关系体现在三个方面。

第一，PID 运行在第26章建立的控制周期中。`main()` 在 500Hz 实时控制分支中调用：

```c
computeMotorCommands(dt500Hz);
```

`dt500Hz` 继续传入 `updatePID()`，成为积分和微分计算的时间步长。

第二，PID 参数存放在全局配置结构中。`eepromConfig.PID[]` 保存三轴的 `P/I/D`、积分状态和 D 项状态。`config.c` 给出默认值，`main.c` 又在当前调试初始化阶段覆盖了部分 P/I/D 参数。

第三，PID 输出最终会影响 PWM 电机驱动。但第27章只讲到 `pidCmd[]` 和补偿量。PWM 定时器、机械角到电角转换、三相输出和电机硬件映射会在后续章节展开。

因此，PID 是“软件控制核心”，不是“外设初始化章节”。理解它要同时看算法公式、配置结构和运行调用位置。

从 STM32 工程实现角度，PID 还有三个隐含约束。

第一，它在 Cortex-M3 上以单精度浮点形式运行。STM32F103 这类 Cortex-M3 内核没有硬件 FPU，
浮点乘除和 `isnan()`、`isinf()` 检查都属于软件执行成本。当前单次 PID 计算不重，
但三轴、D 项滤波、角度包裹和电角转换叠加后，仍应和第26章的 2ms 预算一起观察。

第二，`updatePID()` 修改 `PIDparameters->iTerm`、`lastDcalcValue`、`lastDterm` 和
`lastLastDterm`。这些不是临时变量，而是跨帧状态。调试 PID 时如果只看本帧输入输出，
容易漏掉历史状态对当前输出的影响。

第三，当前 PID 状态主要在主循环 500Hz 分支中读写。若未来把控制迁移到中断、DMA 回调或 RTOS
任务，就需要重新审查 PID 状态的并发访问、临界区和数据发布边界。当前仓库没有这种迁移证据。

## 7. 项目中的应用

本章涉及的项目证据文件包括：

- `Drivers/SRC/Inc/pid.h`
- `Drivers/SRC/Src/pid.c`
- `Drivers/SRC/Src/config.c`
- `Core/Src/main.c`
- `Drivers/BGC/bgc32.h`
- `Drivers/SRC/Src/computeMotorCommands.c`

`pid.h` 定义 PID 参数结构和函数接口。

`pid.c` 实现 `initPID()`、`updatePID()`、`zeroPIDintegralError()` 和 `zeroPIDstates()`。

`config.c` 设置三轴 PID 默认参数，包括 `P/I/D`、`dErrorCalc` 和 `type`。

`Core/Src/main.c` 在初始化阶段调用 `initPID()`，随后覆盖当前调试使用的部分 P/I/D 参数，并在 AHRS 收敛后调用 `zeroPIDintegralError()` 和 `zeroPIDstates()`。

`bgc32.h` 定义 `ROLL_PID`、`PITCH_PID`、`YAW_PID` 和 `NUMBER_OF_PIDS`。

`computeMotorCommands.c` 在轴使能和状态分支满足时调用 `updatePID()`，并把返回值放入对应轴的 `pidCmd[]`。

## 8. 代码分析

### 8.1 initPID

`initPID()` 的职责是初始化 PID 运行状态，而不是设置全部 P/I/D 默认参数。它主要做两件事：

```text
rc = 1 / (2π * F_CUT)
清零每个 PID 的 iTerm 和 D 项历史状态
```

`rc` 服务于 D 项滤波，第28章会继续解释。本章只需知道：`initPID()` 为后续 `updatePID()` 准备状态变量。

### 8.2 PID参数来源

`config.c` 中 `init_eepromConfig(true)` 为 ROLL/PITCH/YAW 设置默认 `P/I/D`、`dErrorCalc` 和 `type`。

随后 `Core/Src/main.c` 又覆盖了当前调试使用的 PID 参数，例如：

```text
YAW:   P=0.03, I=0.01, D=0.016
PITCH: P=0.01, I=0.0,  D=0.008
ROLL:  P=0.03, I=0.0,  D=0.008
```

因此阅读当前运行参数时，不能只看 `config.c` 的默认值，还要继续看 `main.c` 初始化段后续覆盖。

还要注意，`windupGuard` 当前更像保留/配置字段。虽然它被赋值，但 `updatePID()` 的积分限幅没有直接使用它。

#### PID参数来源与生效证据边界

分析 `eepromConfig.PID[]` 时，建议把“默认配置”“源码覆盖”“当前构建”和“目标板运行观测”
分成四层证据，不要把它们合并成一句“当前 PID 参数就是某组数值”。

| 证据层级 | 当前可用证据 | 能证明什么 | 不能证明什么 |
|---|---|---|---|
| 默认配置层 | `Drivers/SRC/Src/config.c::init_eepromConfig(true)` 写入三轴默认 `P/I/D`、`B`、`dErrorCalc`、`type` 和 `rateLimit` | 证明复位初始化分支会给 `eepromConfig.PID[]` 准备一组默认参数 | 不能证明这些默认值会作为进入闭环前的最终参数，因为 `main()` 后面还有覆盖写入 |
| 源码覆盖层 | `Core/Src/main.c` 在 `initPID()` 和 MPU6050 初始化之后，继续写入 Yaw/Pitch/Roll 的 `P/I/D` 与 D 项历史状态 | 证明当前源码层面的调试意图和写入顺序：覆盖发生在 `init_eepromConfig(true)` 之后 | 不能证明现有 Debug ELF 一定已经反映这些源码值，除非重新构建后的 `.list` 与源码一致 |
| 当前构建层 | `Debug/Three-axis_cloud_platformV2.map` 包含 `.bss.eepromConfig`、`init_eepromConfig`、`initPID`、`updatePID`；`.list` 可见 `main()` 中的 PID 写入指令 | 证明某一次 Debug 构建中的对象、函数和写入路径进入了镜像 | 不能证明该镜像一定来自当前源码；当 `.list` 注释值与 `main.c` 数值不一致时，只能说明存在构建同步边界 |
| 运行观测层 | 通过调试器在第一次 `computeMotorCommands()` 前读取 `eepromConfig.PID[ROLL/PITCH/YAW]` | 能证明目标板 RAM 中实际参与第一次 PID 计算的参数值【待验证】 | 仍不能证明闭环稳定、EEPROM 持久化恢复或电机实际输出方向正确【待验证】 |

本仓库当前就能看到这个边界：`Core/Src/main.c` 中 Yaw 覆盖为 `P=0.03, I=0.01`，
Pitch 覆盖为 `P=0.01, I=0.0`；而现有 `Debug/Three-axis_cloud_platformV2.list`
对应注释仍可见 Yaw `P=2.0, I=0.0`、Pitch `P=0.5, I=0.05`。
因此，本章只能把前者称为“当前源码写法”，把后者称为“现有 Debug 构建产物证据”。
在没有重新构建、下载并读取目标板 RAM 前，不能把两者混合成唯一运行参数结论【待验证】。

### 8.3 updatePID输入

`updatePID()` 的输入把控制问题压缩成五个量：

```text
command：目标
state：当前状态
deltaT：时间间隔
iHold：是否暂停积分
PIDparameters：当前轴参数和历史状态
```

其中 `deltaT` 来自第26章的 `dt500Hz` 或轴内的 `safeDt`。
源码中如果 `deltaT` 超过 `0.01f`、小于 `0.0001f`、为 NaN 或 Inf，
会回退为 `0.002f`。这个防护属于后续异常防护章节，
本章只记录它会影响 PID 时间步长。

从代码执行顺序看，`updatePID()` 先修正 `deltaT`，再计算误差、积分、D 项和输出。
这意味着异常 `deltaT` 不会继续直接进入 PID 的积分和微分计算。
但第26章已经指出，AHRS 使用的是进入姿态函数前的原始 `dt500Hz`，所以不要把 PID 内部保护扩大为
整条控制链路的全局保护。

### 8.3.1 deltaT阈值的时间尺度

`updatePID()` 中的 `0.01f`、`0.0001f` 和 `0.002f` 不是普通无意义常数，它们对应明确的时间尺度：

```text
deltaT > 0.01s    -> 控制周期低于 100Hz，认为过慢
deltaT < 0.0001s  -> 控制周期高于 10kHz，认为过快或时间戳异常
fallback 0.002s   -> 回退到 500Hz 标称周期
```

这条保护的设计意图可以从第26章的 500Hz 主循环反推：正常 `dt500Hz` 应接近
`0.002s`，也就是 2ms。如果主循环阻塞、时间戳异常或调试暂停导致 `deltaT`
落到 `[0.0001s, 0.01s]` 之外，PID 内部不再使用这个异常值，而是回退到标称
500Hz 步长。

但这只是“局部净化”。它不能证明异常帧没有发生，也不能证明控制周期仍然稳定。
例如 `deltaT = 0.02s` 时，`updatePID()` 会用 `0.002s` 计算积分和微分，
这能避免 I/D 项被 20ms 步长直接放大或削弱；但实际主循环已经经历了一次约 20ms
的消费间隔，姿态解算、目标斜坡、输出更新节奏和电机响应仍需要单独验证【待验证】。
因此调试报告中应同时记录“进入 `updatePID()` 的原始 `deltaT`”和“回退后实际用于
I/D 计算的 `deltaT`”，不要只记录 PID 输出值。

#### 8.3.1.1 deltaT回退对P/I/D三项的影响

`deltaT` 回退发生在 `updatePID()` 开头，所以后续 P/I/D 三项看到的是回退后的局部时间步。它对三项的影响并不相同：

| 项 | 源码位置 | `deltaT` 的直接影响 | 调试边界 |
|---|---|---|---|
| P 项 | `error = command - state`，输出中 `P * error` | P 项公式不直接乘除 `deltaT` | 若异常 `deltaT` 来自主循环阻塞，P 项仍可能因为姿态或目标已经变化而间接受影响 |
| I 项 | `temp_iTerm = iTerm + error * deltaT` | 回退后的 `deltaT` 决定本帧积分面积增量 | `iHold == true` 时本帧不更新 `iTerm`；若该轴 `I=0.0f`，`iTerm` 变化也不会形成积分输出 |
| D 项 | `dTerm = dInput / deltaT`，随后 `deltaT / (rc + deltaT)` 进入滤波 | 回退后的 `deltaT` 同时影响微分斜率和 D 项低通滤波系数 | D 项是否抖动还取决于姿态噪声、`lastDcalcValue`、`lastDterm` 和 `dErrorCalc` |
| 输出组合 | `P * error + I * iTerm + D * dAverage` | 输出使用已经更新或滤波后的状态量 | 不能只看输出有限值就断言真实控制周期稳定 |

所以，`deltaT` 回退更像 PID 内部的“数值保险丝”，不是全系统实时性的证明。若要复盘一次异常帧，应同时记录回退前 `deltaT`、回退后局部 `deltaT`、`iHold`、`iTerm` 增量、原始 `dTerm`、滤波后 `dAverage` 和最终输出。

### 8.3.2 deltaT回退与safeDt分层

`deltaT` 回退和 `safeDt` 不是同一个保护点。前者在 `pid.c` 的 `updatePID()` 内部发生，
保护的是 PID 本体的积分、微分和 D 项滤波计算；后者出现在 `computeMotorCommands.c` 的部分轴分支中，
保护的是分支内目标斜坡、PID 实参或后级输出速率限制。按当前源码可以拆成四条路径：

| 路径 | 传给 `updatePID()` 的时间步 | PID 外部是否继续使用 `safeDt` | 教学边界 |
|---|---|---|---|
| Roll 回中/回零分支 | 不调用 `updatePID()` | 使用 `safeDt` 计算 `ROLL_SLEW_RAD_S * safeDt` | 这是目标斜坡保护，不是 PID 回退 |
| Roll 后续 PID 分支 | 先在轴分支生成 `safeDt`，再传给 `updatePID()` | 使用 `rateLimit * safeDt` 约束单帧输出变化 | PID 本体和 Roll 后级限速都使用净化后的时间步 |
| Pitch PID 分支 | 直接传入 `dt` | 输出速率限制直接比较 `eepromConfig.rateLimit` | PID 内部可能回退 `deltaT`，但后级限速没有乘 `dt` |
| Yaw PID 分支 | 直接传入 `dt` | 输出速率限制直接比较 `eepromConfig.rateLimit` | 与 Pitch 相同，不能推导为全轴统一的每秒速率限制 |

因此，`updatePID()` 的异常时间步回退只能证明 PID 内部的 I/D 计算不会直接使用异常 `deltaT`。
它不能证明 AHRS、Roll 目标斜坡、Pitch/Yaw 后级输出速率限制都使用同一个受保护时间步。
若要判断这些差异是否为有意设计，需要结合轴使能状态、连续运行日志和电机响应记录，当前保持【待验证】。

### 8.4 误差和角度归一化

`updatePID()` 先计算：

```c
error = command - state;
```

如果 `PIDparameters->type == ANGULAR`，再调用：

```c
error = standardRadianFormat(error);
```

这说明当前三轴姿态 PID 是角度 PID。第39项知识 `角度归一化` 在这里成为控制误差计算的前置条件。

`standardRadianFormat()` 的源码只执行一次边界修正：

```text
angle >= PI  -> angle - 2*PI
angle < -PI  -> angle + 2*PI
```

因此它适合已经被控制链路限制在合理范围附近的角度误差，
但不是任意大角度的完整归一化器。对比 `computeMotorCommands.c`
中的 `wrapToPif()`，后者使用 `while` 循环，能反复处理多圈角度。

调试时建议增加三个边界样例：

```text
standardRadianFormat( PI) -> -PI
standardRadianFormat(-PI) -> -PI
standardRadianFormat(5PI) ->  3PI
```

这些样例说明，PID 内部的函数更像“单圈附近的误差校正”，
不能替代控制层对目标角、电角和斜坡目标的完整包裹。

### 8.5 P/I/D输出

在 `ANGULAR` 分支中，输出组合为：

```text
P * error + I * iTerm + D * dAverage
```

其中：

- `error` 来自目标和当前角度。
- `iTerm` 来自历史误差累计。
- `dAverage` 来自 D 项计算和滤波历史。

本章不展开 `dAverage` 的完整滤波推导，也不把输出约束提前并入本章主线。
但积分暂停、固定积分限幅和 D 项数值防护已经影响 `updatePID()` 的核心输入输出，
因此本章会记录它们的源码边界；第28章再继续展开抗饱和、滤波和输出保护的系统意义。

### 8.6 连续公式到源码状态

把连续 PID 对照到当前源码，可以得到下表：

| 连续PID概念 | 离散源码变量 | 项目含义 |
| --- | --- | --- |
| `e(t)` | `error` | 当前目标与当前状态的差 |
| `∫e(t)dt` | `PIDparameters->iTerm` | 跨帧累计的误差面积 |
| `de/dt` | `dTerm` | 误差或状态变化率 |
| D项滤波结果 | `dAverage` | 滤波并平均后的 D 项输入 |
| 控制输出 | `output` | 返回给轴逻辑的 PID 补偿量 |

这张表的关键是：`iTerm` 和 D 项历史状态都保存在 `eepromConfig.PID[]` 中。
它们会跨帧延续，所以 PID 调试不是单帧代数题，而是带状态的离散控制问题。

#### 8.6.1 PIDdata_t的配置字段与跨帧状态字段

`PIDdata_t` 容易被误读成一张“PID 参数表”。从源码看，它同时承担两类职责：

| 字段类别 | 典型字段 | 写入来源 | 教学含义 |
|---|---|---|---|
| 配置字段 | `P/I/D/B/type/dErrorCalc/rateLimit/windupGuard` | `config.c::init_eepromConfig(true)` 默认写入，`main.c` 当前调试代码再次覆盖部分字段 | 决定公式分支、比例/积分/微分增益和后级约束意图 |
| 运行状态字段 | `iTerm/lastDcalcValue/lastDterm/lastLastDterm` | `initPID()`、`zeroPIDintegralError()`、`zeroPIDstates()` 和每次 `updatePID()` 写回 | 保存跨帧积分面积、D 项历史输入和滤波历史 |

这一区分很重要。`computeMotorCommands.c` 传给 `updatePID()` 的不是一份临时副本，而是
`&eepromConfig.PID[ROLL_PID]`、`&eepromConfig.PID[PITCH_PID]` 或
`&eepromConfig.PID[YAW_PID]`。因此 `updatePID()` 内部执行：

```c
PIDparameters->iTerm = temp_iTerm;
PIDparameters->lastDcalcValue = error;
PIDparameters->lastLastDterm = PIDparameters->lastDterm;
PIDparameters->lastDterm = dTermFiltered;
```

时，写回的是对应轴 `eepromConfig.PID[]` 中的跨帧状态。也就是说，`updatePID()` 不是“只读参数、返回输出”的纯函数；它会改变下一帧 PID 计算的初始条件。

这也解释了为什么启动门控达到 1000 帧时要同时调用 `zeroPIDintegralError()` 和
`zeroPIDstates()`。前者清除积分面积，后者清除 D 项历史状态。否则即使当前帧 `command`
和 `state` 看起来正常，上一阶段残留的 `iTerm`、`lastDterm` 或 `lastDcalcValue`
仍可能影响接通后的第一批 PID 输出。

构建产物能证明这条链路的边界：`.map` 中 `eepromConfig`、`updatePID()`、
`zeroPIDintegralError()` 和 `zeroPIDstates()` 都进入当前 Debug 镜像；`.list`
中也能看到三轴分支调用 `updatePID()` 的路径。但 `.map/.list` 不能证明某一帧写回的
`iTerm` 或 D 项历史值合理，也不能证明这些运行状态已经被 EEPROM 持久化保存。
当前 `config.c::init_eepromConfig(true)` 仍以默认写入分支为主，缺少 EEPROM 恢复链路证据时，
PID 状态持久化继续保持【待验证】。

### 8.7 B参数分支边界

`pid.c::updatePID()` 在 `ANGULAR` 和 `OTHER` 两类 PID 上使用不同输出公式。

`ANGULAR` 分支：

```c
output = PIDparameters->P * error
       + PIDparameters->I * PIDparameters->iTerm
       + PIDparameters->D * dAverage;
```

`OTHER` 分支：

```c
output = PIDparameters->P * PIDparameters->B * command
       + PIDparameters->I * PIDparameters->iTerm
       + PIDparameters->D * dAverage
       - PIDparameters->P * state;
```

`config.c` 中当前 ROLL/PITCH/YAW 均为 `ANGULAR`。因此本项目当前姿态闭环使用的是第一条公式。
`B` 字段虽然在配置中被赋值为 `1.0f`，但不会影响当前三轴 `ANGULAR` 分支输出。

### 8.8 computeMotorCommands中的Pitch入口

Pitch 轴在 `eepromConfig.pitchEnabled == true` 时读取：

```text
pitch_angle = PITCH_SENSOR_SIGN * sensors.margAttitude500Hz[PITCH]
target_angle = pointingCmd[PITCH]
```

随后调用：

```text
updatePID(target_angle, pitch_angle, dt, holdIntegrators, &eepromConfig.PID[PITCH_PID])
```

这是一条较直观的姿态角闭环：目标 Pitch 角减当前 Pitch 角，得到 PID 补偿量。

这里有一个容易误读的源码边界。Pitch 分支在调用 `updatePID()` 前也计算了：

```c
float error_mech = wrapToPif(target_angle - pitch_angle);
```

但当前后续调用仍然把 `target_angle` 和 `pitch_angle` 作为 `command/state` 实参传入
`updatePID()`，没有把 `error_mech` 本身作为 PID 输入，也没有像 Yaw 分支那样用它构造
`safe_target`。因此，当前 Pitch 的角度包裹主要依赖 `updatePID()` 内部
`ANGULAR` 分支的 `standardRadianFormat(error)`。教材和调试记录不能只因为附近存在
`error_mech = wrapToPif(...)`，就写成 Pitch 已经在控制层用 `wrapToPif()` 预处理了 PID 输入。

### 8.9 computeMotorCommands中的Yaw入口

Yaw 轴在 `eepromConfig.yawEnabled == true` 时读取：

```text
yaw_angle = sensors.margAttitude500Hz[YAW]
target_angle = pointingCmd[YAW]
```

代码先用 `wrapToPif()` 计算机械角误差，再构造 `safe_target`，随后调用：

```text
updatePID(safe_target, yaw_angle, dt, holdIntegrators, &eepromConfig.PID[YAW_PID])
```

这里的重点是：Yaw 不是直接把原始目标角丢给 PID，而是先做角度包裹，避免跨越 `±PI` 时出现长路径误差。
这也和 Pitch 形成对照：两处都出现 `error_mech` 局部变量，但只有 Yaw 把包裹后的误差重新组合进
`safe_target` 并作为 `updatePID()` 的 `command` 实参。

### 8.10 computeMotorCommands中的Roll入口

Roll 轴的当前实现有明显状态分支。

当 `eepromConfig.rollEnabled == true` 且 `return_state_roll == true` 时，
代码主要通过 `rollTargetSlew` 和 `PWM_Motor_SetAngle()` 做回中/过渡输出，
没有进入 `updatePID()`。

当进入后续分支时，代码会计算当前电角、目标电角和电角误差，并调用：

```text
updatePID(target_electrical_angle,
          current_electrical_angle,
          safeDt,
          rollHoldIntegrators,
          &eepromConfig.PID[ROLL_PID])
```

所以不能简单写成“Roll 每帧都与 Pitch/Yaw 一样调用 PID”。更准确的说法是：Roll 的 PID 调用受轴使能和回中状态分支影响。

### 8.11 PID状态清零

`main.c` 在 AHRS 收敛计数达到 1000 帧后调用：

```text
zeroPIDintegralError()
zeroPIDstates()
```

这说明控制接通前会清空积分项和 D 项历史状态，降低开机瞬间历史状态造成的冲击。第32章运行门控和异常防护会继续分析这个行为的系统意义。

### 8.12 D_STATE角度处理边界

`D_ERROR` 分支的源码顺序是：

```text
dInput = error - lastDcalcValue
dInput = standardRadianFormat(dInput)
dTerm = dInput / deltaT
```

这表示角度差先被包裹，再换算成角速度量。

`D_STATE` 分支的源码顺序是：

```text
dTerm = (lastDcalcValue - state) / deltaT
dTerm = standardRadianFormat(dTerm)
```

这表示先计算变化率，再把变化率当作角度值做一次包裹。
从量纲上看，这与“先包裹角度差、再除以时间”的数学含义不同。

由于当前 `config.c` 把三轴 `dErrorCalc` 都设为 `D_ERROR`，
第27章只把它作为源码边界记录，不把它写成当前运行主线。
若未来切换到 `D_STATE`，应补充专门测试：目标不变、状态跨越 `±π`、
`deltaT` 接近 0.002s 和异常 `deltaT` 回退时，D 项是否符合预期【待验证】。

### 8.13 位置式PID源码证据

`updatePID()` 中最后组合输出的位置是：

```c
output = PIDparameters->P * error
       + PIDparameters->I * PIDparameters->iTerm
       + PIDparameters->D * dAverage;
```

这个输出是当前帧控制量，而不是相对上一帧控制量的增量。源码中没有类似：

```text
deltaOutput = ...
output = lastOutput + deltaOutput
```

的 PID 内部状态。因此，第27章把它归类为位置式离散 PID。

需要避免一个误判：`computeMotorCommands.c` 中确实有 `pidCmdPrev[]`
和 `outputRate[]`。但它们用于 PID 输出之后的速率限制，不是 `updatePID()`
内部的增量式 PID 实现。换句话说：

```text
updatePID()      -> 位置式 PID 输出
computeMotor...  -> 对输出再做限幅和速率约束
```

复查输出速率约束时，还要单独检查 `rateLimit` 的单位语义。当前
`config.c` 中默认：

```text
eepromConfig.rateLimit = 45.0f * D2R
```

如果把这个字段按变量名理解为“每秒允许变化的弧度量”，Roll 后续 PID 分支的写法是匹配的：

```text
rollStepLimit = eepromConfig.rateLimit * safeDt
```

在 500Hz、`safeDt ~= 0.002s` 时，默认每帧步长约为：

```text
45 deg/s * 0.002 s = 0.09 deg/frame ~= 0.00157 rad/frame
```

但 Pitch/Yaw 分支当前直接比较：

```text
outputRate[axis] = pidCmd[axis] - pidCmdPrev[axis]
if outputRate[axis] > eepromConfig.rateLimit:
    pidCmd[axis] = pidCmdPrev[axis] + eepromConfig.rateLimit
```

这等价于把同一个 `rateLimit` 当作“每帧最大变化量”使用。按默认值计算，
Pitch/Yaw 每帧允许变化约 `45 deg/frame`，而 Roll 后续 PID 分支约
`0.09 deg/frame`。教材不能仅凭这段源码断言它一定是错误实现，因为不同轴可能有调试阶段的临时意图；
但必须把它记录为输出速率限制的单位边界：同一配置字段在不同轴上呈现了
“乘以 `dt`”和“不乘以 `dt`”两种语义，需要日志、实物响应或设计意图确认【待验证】。

### 8.14 非LQR/MPC架构证据

如果一个项目采用 LQR，通常会看到状态向量、系统矩阵、反馈增益矩阵或类似
`u = -Kx` 的计算结构。当前仓库的主控制路径没有这些证据。

如果一个项目采用 MPC，通常会看到预测模型、预测时域、约束、代价函数和在线优化求解器。
当前仓库也没有这些结构。

当前可见证据是：

```text
姿态误差/电角误差
-> updatePID()
-> pidCmd[]
-> 限幅/速率限制/电角合成
-> PWM_Motor_SetAngle()
```

因此本章按 PID 控制解释当前项目，而不是把它扩展成现代状态空间控制或预测控制教材。
后续如果要引入 LQR/MPC，只能作为架构迁移方案重新设计，不能从当前源码直接推出。

### 8.15 iHold与holdIntegrators顺序边界

`updatePID()` 的 `iHold` 参数是每次调用传入的积分暂停标志。
但 `computeMotorCommands()` 中并不是三轴完全独立地生成这个标志。

`pid.c` 定义了全局变量：

```text
holdIntegrators = true
```

而 `computeMotorCommands()` 每帧一开始会执行：

```text
holdIntegrators = false
```

随后 Roll 的后续 PID 分支会根据电角误差计算：

```text
rollHoldIntegrators = fabsf(roll_error_electrical) > ROLL_I_ENABLE_ERR_RAD
```

Roll 调用 `updatePID()` 时传入的是 `rollHoldIntegrators` 本身。
如果它为真，代码还会把全局 `holdIntegrators` 置为 `true`。

Pitch 和 Yaw 的 `updatePID()` 调用则传入全局 `holdIntegrators`。
由于当前文件中 Roll 分支位于 Pitch/Yaw 分支之前，
如果 Roll 已经进入后续 PID 分支且 `rollHoldIntegrators == true`，
同一帧内后面的 Pitch/Yaw 积分也可能被暂停。

这个边界说明当前积分暂停不是完全轴内局部状态：

- Roll PID 分支：使用 `rollHoldIntegrators`。
- Pitch/Yaw PID 分支：使用当前帧全局 `holdIntegrators`。
- `return_state_roll == true` 的 Roll 回中阶段不调用 Roll PID，也不会设置这个全局暂停。
- Roll 轴未使能时，Pitch/Yaw 通常看到的是本帧开头写入的 `false`。

实际运行中是否出现同帧跨轴积分暂停，还取决于 Roll 是否使能、是否退出回中阶段、
电角误差是否超过 `ROLL_I_ENABLE_ERR_RAD`，以及 Pitch/Yaw 是否已使能。
这些组合需要断点或日志确认，不能只凭源码顺序断言已经发生【待验证】。

### 8.16 固定积分限幅与windupGuard边界

`main.c` 中给 `windupGuard` 赋值时，注释写成积分限幅。
但 `pid.c::updatePID()` 实际更新积分状态时没有读取 `PIDparameters->windupGuard`。
当前积分更新流程是：

```text
temp_iTerm = iTerm + error * deltaT
temp_iTerm clamp to [-10, 10]
if PIDparameters == PITCH_PID:
    temp_iTerm clamp to [-10, 10]
iTerm = temp_iTerm
```

因此有三个教学边界：

- `windupGuard` 是结构体字段和配置字段，但不是当前积分限幅的执行来源。
- PITCH 额外限幅分支当前与全局限幅数值相同，源码注释中的“更严格”没有由代码体现。
- `iTerm` 的限幅是积分状态限幅，不是最终输出限幅；最终积分输出还要乘以 `I` 参数。

结合当前 `main.c` 覆盖参数看，Yaw 的 `I=0.01`，Pitch 和 Roll 的 `I=0.0`。
这意味着 Pitch/Roll 即使更新了 `iTerm`，当前 PID 输出中的 `I * iTerm` 仍为 0。
如果后续调参把 Pitch/Roll 的 `I` 改成非零值，既有 `iTerm` 状态就会重新影响输出。
实际调参时应同时记录 `I`、`iTerm` 和 `I * iTerm`，不能只看 `windupGuard`。

### 8.17 lastDcalcValue的0值哨兵边界

`updatePID()` 用 `lastDcalcValue == 0.0f` 判断 D 项历史是否需要初始化：

```text
if lastDcalcValue == 0.0:
    lastDcalcValue = error or state
```

这种写法简单，但把“尚未初始化”和“上一帧值刚好为 0”放在同一个数值状态里。
在 `D_ERROR` 分支中，如果上一帧误差确实为 0，下一帧进入函数时仍可能再次触发初始化判断。
若下一帧误差刚从 0 变为非零，代码会先把 `lastDcalcValue` 设成当前误差，
再计算 `error - lastDcalcValue`，于是这一帧的 D 项变化可能被清成 0。

同理，在 `D_STATE` 分支中，若上一帧状态值恰好为 0，也存在类似的哨兵值混淆边界。
更明确的设计通常会使用独立的 `valid` 标志或初始化阶段标志，
把“历史值是否有效”和“历史值是多少”分开。

当前三轴默认使用 `D_ERROR`，并且真实运行中误差是否会长期精确等于 0 需要日志确认。
所以这里不能写成一定导致控制问题，只能写成 D 项初始化逻辑的源码边界【待验证】。

### 8.18 D项限幅与输出有限值边界

`updatePID()` 对 D 项和最终输出都有数值防护。D 项原始值计算后：

```text
NaN/Inf -> 0
dTerm clamp to [-300, 300]
```

随后 D 项还会进入低通滤波和三点平均，最终参与：

```text
output = P * error + I * iTerm + D * dAverage
```

如果把源码中的滤波部分单独展开，可以得到：

```text
rc = 1 / (2π * F_CUT)
alpha[k] = deltaT[k] / (rc + deltaT[k])
D_f[k] = D_f[k-1] + alpha[k] * (D_raw[k] - D_f[k-1])
D_avg[k] = (D_f[k] + D_f[k-1] + D_f[k-2]) / 3
```

其中 `D_raw` 对应限幅后的 `dTerm`，`D_f` 对应 `dTermFiltered`，
`D_avg` 对应最终进入输出公式的 `dAverage`。这说明当前 D 项不是“误差差分后立刻乘以 D”，
而是先经过一阶低通滤波，再与前两帧滤波结果做三点平均。

当前 `pid.c` 中 `F_CUT = 20.0f`。若 `deltaT` 约为 0.002s，则：

```text
rc = 1 / (2π * 20) ~= 0.00796s
alpha = 0.002 / (0.00796 + 0.002) ~= 0.201
```

`alpha` 越接近 0，滤波输出越依赖上一帧；`alpha` 越接近 1，滤波输出越接近当前原始 D 项。
因此 `deltaT` 不只影响 `dTerm = Δerror / deltaT`，也会改变 D 项低通滤波的响应速度。
当 `updatePID()` 把异常 `deltaT` 回退为 `0.002f` 时，D 项斜率计算和滤波系数会同时使用这个回退值。
调试 D 项噪声或迟滞时，必须同时记录 `deltaT`、`dTerm`、`dTermFiltered`、`lastDterm`、
`lastLastDterm` 和 `dAverage`，否则容易把“微分增益太大”和“滤波状态滞后”混为一谈。

最终 `output` 如果是 NaN 或 Inf，也会被置为 0。
这些防护能降低异常数值继续传播的风险，但它们不等于闭环稳定性证明。

例如，D 项被夹到 `300` 只能说明微分输入被限制在一个数值范围内；
最终输出是有限数字，也只能说明本次浮点计算没有继续发散成 NaN/Inf。
电机方向、机械耦合、输出限幅后的实际力矩、姿态是否稳定，
仍然需要第28章到第30章的链路分析和硬件实测证据【待验证】。

### 8.19 PID状态写回证据边界

经典 PID 不是只由当前一帧误差决定的无状态公式。积分项需要保存历史误差累积，
微分滤波也需要保存上一帧或上几帧的中间值。当前项目的 `updatePID()` 也符合这个特征：
它接收的是 `struct PIDdata *PIDparameters` 指针，并在计算过程中写回该结构体。

可以把 `updatePID()` 的状态写回拆成四类证据：

| 状态字段 | 写回条件 | 源码证据 | 教材解释边界 |
| --- | --- | --- | --- |
| `iTerm` | `iHold == false` 时更新，随后夹到 `[-10, 10]` | `PIDparameters->iTerm = temp_iTerm` | 证明积分状态会跨帧保存；若对应轴 `I=0.0f`，状态更新不等于积分输出已经生效 |
| `lastDcalcValue` | D 项初始化和每次 D 项输入更新时写回 | `PIDparameters->lastDcalcValue = error/state` | 证明下一帧微分输入依赖本帧记录；`0.0f` 哨兵不能证明已有独立有效标志 |
| `lastLastDterm` | 每次 D 项滤波后写回上一帧 `lastDterm` | `PIDparameters->lastLastDterm = PIDparameters->lastDterm` | 证明三点平均使用跨帧 D 项历史；不能只看当前 `dTerm` 判断 D 输出 |
| `lastDterm` | 每次 D 项滤波后写回当前滤波结果 | `PIDparameters->lastDterm = dTermFiltered` | 证明低通滤波状态会延续到下一帧；调试噪声时必须记录连续帧 |

三轴之所以共享同一个 `updatePID()` 实现，是因为 `computeMotorCommands.c`
分别传入 `&eepromConfig.PID[ROLL_PID]`、`&eepromConfig.PID[PITCH_PID]`
和 `&eepromConfig.PID[YAW_PID]`。也就是说，函数代码共享，但运行状态按 PID 参数对象分开保存。
这类设计降低了重复实现，但也要求调试时明确“当前断点看到的是哪一个轴的 `PIDdata_t`”。

构建产物可以补一层证据：`.map` 证明 `.bss.eepromConfig`、`updatePID` 和状态清零 helper
进入镜像，`.list` 证明 `computeMotorCommands()` 的 Roll、Pitch、Yaw 分支都存在调用
`updatePID()` 的路径，并能看到 `updatePID()` 内部对 `iTerm`、`lastDcalcValue`、
`lastLastDterm` 和 `lastDterm` 的写回语句。
但这些证据仍不能证明某一轴在真实运行中每帧都执行，也不能证明连续帧状态演化符合预期。
若要证明状态写回真正按预期工作，需要在同一轴上连续记录“调用前字段值、输入误差、`deltaT`、
写回后字段值和返回输出”，缺少这类日志时保持【待验证】。

### 8.20 构建产物证据边界

前面的小节主要根据源码说明 PID 的数据结构、公式分支和调用关系。
如果要确认这些路径是否真的进入当前 Debug 镜像，需要再看 `Debug/Three-axis_cloud_platformV2.map`
和 `Debug/Three-axis_cloud_platformV2.list`，不能只停留在“源码里存在”。

当前 `.map` 至少能证明以下符号已经被链接进最终镜像：

| 符号或段 | 地址 | 证据含义 |
|---|---:|---|
| `wrapToPif` | `0x080042f4` | `computeMotorCommands.c` 中的多圈角度包裹 helper 进入镜像。 |
| `computeMotorCommands` | `0x080043b4` | 500Hz 控制输出函数进入镜像。 |
| `standardRadianFormat` | `0x08005250` | `pid.c` 中的 PID 内部角度包裹函数进入镜像。 |
| `initPID` | `0x080052a4` | PID 状态初始化函数进入镜像。 |
| `updatePID` | `0x08005334` | PID 主计算函数进入镜像。 |
| `zeroPIDintegralError` | `0x0800578c` | 积分清零 helper 进入镜像。 |
| `zeroPIDstates` | `0x08005814` | D 项历史状态清零 helper 进入镜像。 |
| `.data.holdIntegrators` | `0x2000002c` | 跨轴积分暂停开关以全局数据进入 RAM 映像。 |
| `.bss.eepromConfig` | `0x200003e0` | PID 参数所在配置对象进入 RAM 映像。 |
| `.bss.outputRate` | `0x20000d8c` | 后级输出变化量数组进入 RAM 映像。 |
| `.bss.pidCmd` | `0x20000d98` | PID 输出命令数组进入 RAM 映像。 |
| `.bss.pidCmdPrev` | `0x20000da4` | 上一帧 PID 输出数组进入 RAM 映像。 |
| `.bss.rollDiag` | `0x20000db0` | Roll 诊断结构体进入 RAM 映像。 |

当前 `.list` 则能把“进入镜像”的符号继续落到反汇编调用点上：

- `main()` 在 `0x08001550` 调用 `initPID()`。
- AHRS 收敛后的启动门控路径在 `0x080018d4` 调用 `zeroPIDintegralError()`，
  并在 `0x080018d8` 调用 `zeroPIDstates()`。
- 500Hz 分支在 `0x08001900` 调用 `computeMotorCommands(dt500Hz)`。
- `computeMotorCommands()` 的 Roll 分支在 `0x0800458e` 调用 `updatePID()`。
- `computeMotorCommands()` 的 Pitch 分支在 `0x08004854` 调用 `updatePID()`。
- `computeMotorCommands()` 的 Yaw 分支在 `0x08004a82` 调用 `updatePID()`。
- `updatePID()` 内部可见多处 `standardRadianFormat()` 调用，例如 `0x080053be`、
  `0x0800548c` 和 `0x080054ea`。
- `computeMotorCommands()` 内部可见多处 `wrapToPif()` 调用，例如 `0x0800448c`、
  `0x08004840` 和 `0x08004a62`。

`.su/.cyclo` 还能把 PID 主链路的静态审查边界补得更细。当前 Debug 构建中：

| 函数 | 静态栈用量 | 圈复杂度 | 证据文件 |
| --- | --- | --- | --- |
| `standardRadianFormat()` | 16 字节 | 3 | `Debug/Drivers/SRC/Src/pid.su` / `.cyclo` |
| `initPID()` | 16 字节 | 2 | `Debug/Drivers/SRC/Src/pid.su` / `.cyclo` |
| `updatePID()` | 64 字节 | 34 | `Debug/Drivers/SRC/Src/pid.su` / `.cyclo` |
| `zeroPIDintegralError()` | 16 字节 | 2 | `Debug/Drivers/SRC/Src/pid.su` / `.cyclo` |
| `zeroPIDstates()` | 16 字节 | 2 | `Debug/Drivers/SRC/Src/pid.su` / `.cyclo` |
| `computeMotorCommands()` | 120 字节 | 48 | `Debug/Drivers/SRC/Src/computeMotorCommands.su` / `.cyclo` |
| `main()` | 56 字节 | 7 | `Debug/Core/Src/main.su` / `.cyclo` |

这些结果说明当前 PID 链路在 Debug 构建中确实保留了静态栈报告和较高圈复杂度的控制函数。
其中 `updatePID()` 和 `computeMotorCommands()` 仍是分支密集点；但静态栈数字不能直接相加成单帧峰值，
也不能把圈复杂度当成实时耗时。`updatePID()` 的 64 字节静态栈只说明编译器对该函数生成的
静态报告如此，不代表实际 500Hz 单次调用的运行时栈水位。真实 2ms 预算仍要由 `executionTime500Hz`
日志、GPIO 翻转、逻辑分析仪或调试器测量来验证。

因此，`.map/.list` 能证明当前构建中确实存在 PID 初始化、状态清零、500Hz 调用、
三轴 `updatePID()` 路径、角度包裹 helper 和后级输出状态对象。
它们不能证明 PID 参数合理、闭环稳定、电机方向安全、机械耦合正确、D 项调参效果符合预期，
也不能证明 `holdIntegrators` 的跨轴积分暂停在真实运行中一定触发。
这些结论仍需要断点、日志、波形、硬件响应或仓库外实测记录支撑【待验证】。

### 本节证据边界

本节只根据当前仓库说明文件、函数、宏、变量、调用关系和当前 Debug 构建产物说明 PID 主链路。
`.map/.list` 可以证明符号进入镜像和反汇编调用点存在，但不能证明运行时频率、外部硬件表现、
传感器方向、电机响应、闭环稳定性或真实控制效果；这些结论仍需调试记录、日志或仓库外实测证据。
缺少证据时保持【待验证】。

## 9. 调试方法

第一步，确认 PID 是否初始化。

- 在 `Core/Src/main.c` 中确认 `initPID()` 已经在进入主循环前调用。
- 在 `Debug/Three-axis_cloud_platformV2.list` 中确认 `main()` 对 `initPID()` 的反汇编调用点仍存在。
- 观察 `eepromConfig.PID[ROLL/PITCH/YAW]` 的 `iTerm` 和 D 项历史状态是否清零。

第二步，确认当前运行参数。

- 先看 `Drivers/SRC/Src/config.c` 中默认 P/I/D。
- 再看 `Core/Src/main.c` 中是否覆盖了这些参数。
- 调试当前工程时，以实际运行前最后一次写入的参数为准。
- 同时记录 `type`、`dErrorCalc` 和 `B`，避免只看 P/I/D 而漏掉公式分支。

第三步，确认输入来源。

- Pitch/Yaw 调试时观察 `sensors.margAttitude500Hz[PITCH/YAW]`。
- Pitch 分支要同时确认 `error_mech` 是否只是局部计算值；当前 PID 实参仍是
  `target_angle` 和 `pitch_angle`。
- Yaw 分支要确认 `error_mech` 是否进入 `safe_target`，再进入 `updatePID()` 的
  `command` 实参。
- Roll 调试时同时观察 `return_state_roll`，确认当前是否已经进入 Roll PID 分支。
- 观察 `pointingCmd[]` 是否符合目标角预期。

第四步，确认时间步长。

- 观察传入 `computeMotorCommands(dt500Hz)` 的 `dt500Hz`。
- 对照 `.list` 中 `main()` 到 `computeMotorCommands(dt500Hz)` 的调用点，确认当前分析针对的是已链接的 500Hz 分支。
- 观察 `updatePID()` 内部是否把异常 `deltaT` 回退为 `0.002f`。
- 分别记录积分增量 `error * deltaT` 和微分斜率变化，确认时间步长对 I/D 两项的影响。

第五步，确认角度包裹边界。

- 对比 `standardRadianFormat()` 和 `wrapToPif()` 的源码实现。
- 用 `PI`、`-PI`、`5PI` 三个样例验证单次包裹与循环包裹的差异。
- 若目标角、电角或误差可能超过一圈，优先确认控制层是否已用 `wrapToPif()` 处理。
- 不要把 `standardRadianFormat()` 当成任意角度输入都有效的通用归一化器。

第六步，确认输出结果。

- 在 `updatePID()` 返回处观察 `output`。
- 在 `computeMotorCommands()` 中观察 `pidCmd[PITCH]`、`pidCmd[YAW]` 和进入 Roll PID 分支后的 `pidCmd[ROLL]`。
- 同时记录 `pidCmdPrev[]`、`outputRate[]`、`eepromConfig.rateLimit`、`dt` 或 `safeDt`。
- 分别确认 Roll 后续 PID 分支是否使用 `rateLimit * safeDt`，Pitch/Yaw 是否直接使用 `rateLimit`。
- 如果 PID 输出正常但电机输出异常，后续应进入电角转换、限幅和 PWM 输出章节，而不是只改 P/I/D 参数。

第七步，拆分 P/I/D 三项贡献。

- 记录 `P * error`。
- 记录 `I * iTerm`。
- 记录 `D * dAverage`。
- 对比三项之和与 `output` 是否一致。

如果某一项远大于其它项，优先检查对应来源：P 项看误差定义，I 项看积分状态和 `iHold`，
D 项看 `deltaT`、噪声和 D 项历史状态。

第八步，确认积分暂停的来源。

- 在 `computeMotorCommands()` 入口记录 `holdIntegrators` 被置为 `false`。
- 在 Roll 后续 PID 分支记录 `rollHoldIntegrators` 和全局 `holdIntegrators`。
- 分别记录 Roll/Pitch/Yaw 传给 `updatePID()` 的 `iHold` 实参。
- 若 Pitch/Yaw 积分被暂停，确认是否由同一帧前面的 Roll 分支触发。

第九步，确认积分限幅的真实来源。

- 记录 `windupGuard` 的配置值，但不要把它直接当成当前执行限幅。
- 在 `updatePID()` 中观察 `temp_iTerm` 被夹到 `[-10, 10]` 的过程。
- 确认 PITCH 额外限幅分支当前是否仍使用同一组 `[-10, 10]` 常数。
- 同时记录 `I`、`iTerm` 和 `I * iTerm`，区分积分状态与积分输出贡献。

第十步，确认 D 项初始化和限幅。

- 观察 `lastDcalcValue == 0.0f` 时是否触发历史值初始化。
- 在误差从 0 变为非零的场景下，记录第一帧 D 项是否被清成 0。
- 记录 `dTerm` 进入 `[-300, 300]` 限幅前后的数值。
- 若 `dTerm`、`dTermFiltered` 或 `output` 被 NaN/Inf 防护置零，保留触发前的输入证据。

调试记录建议：

- 记录 `initPID()` 调用、实际 P/I/D 参数来源、输入姿态、目标角和 `dt500Hz`。
- 对每个轴，应记录误差、P 项、I 项、D 项、`type`、`dErrorCalc`、积分暂停状态和 `updatePID()` 返回值。
- Roll 分支记录应包含 `return_state_roll`、目标斜坡和是否进入 Roll PID 调用。
- 积分暂停记录应包含 `holdIntegrators`、`rollHoldIntegrators`、三轴 `iHold` 实参和轴分支顺序。
- 积分限幅记录应同时包含 `windupGuard` 配置值、实际 `temp_iTerm` 限幅和最终 `I * iTerm`。
- D 项记录应包含 `lastDcalcValue` 初始化、原始 `dTerm`、限幅后 `dTerm`、滤波后 `dTermFiltered` 和 `dAverage`。
- 如果要分析 `B`，必须先确认该轴是否进入 `OTHER` 分支；当前三轴 `ANGULAR` 分支下 `B` 不参与输出。
- 如果要分析 `D_STATE`，必须先确认当前轴已经切换到该分支；当前三轴默认 `D_ERROR`。
- 角度包裹调试应同时记录输入值、包裹函数、输出值和是否存在多圈输入。
- 输出速率限制记录应同时包含 `pidCmdPrev[]`、`outputRate[]`、`rateLimit`、`dt/safeDt` 和最终应用的 `pidCmd[]`，避免把“每秒速率”和“每帧步长”混为一谈。
- PID 输出只证明控制量计算结果，不能直接证明电机方向、稳定性或安全性。

调试边界：

当前仓库能证明 PID 初始化、参数来源、输入输出变量和调用路径。闭环稳定性、方向正确性、电机温升和安全性必须依赖仓库外实测记录；缺少证据时保持【待验证】。

## 10. 常见问题

问题一：PID 是不是直接控制 PWM 占空比。

不是。`updatePID()` 返回的是补偿量。这个补偿量还会经过轴逻辑、限幅、速率限制、电角合成，最后才影响 `PWM_Motor_SetAngle()`。

问题二：当前三轴是不是每帧都同样调用 `updatePID()`。

不是。Pitch 和 Yaw 在对应轴使能时进入 PID 调用。Roll 还受 `return_state_roll` 分支影响，初始回中阶段可能不调用 `updatePID()`。

问题三：为什么 PID 需要 `dt`。

积分项需要用 `error * dt` 表示误差随时间累计，微分项需要用变化量除以 `dt` 表示变化速度。没有可靠 `dt`，PID 输出就会随周期抖动而变化。

问题四：为什么角度 PID 不能直接用普通差值。

角度有周期性。目标角和当前角跨越 `±PI` 时，普通差值可能得到一条很长的路径。`ANGULAR` 类型会让 `updatePID()` 对误差做角度归一化。

问题五：为什么 `config.c` 和 `main.c` 都设置 PID 参数。

`config.c` 给出默认配置，`main.c` 当前又在初始化阶段覆盖部分参数。教材以实际运行前最后写入的值为准，并记录这种“默认值与调试覆盖并存”的边界。

问题六：第27章为什么不详细讲 D 项滤波和输出限幅。

因为它们是 PID 控制之后的细节知识点，已经映射到第28章。本章先建立目标、状态、误差、时间步长和 P/I/D 输出的主干，避免读者还没理解闭环核心就被安全约束细节淹没。

问题七：PID 输出是有限数字，是否就表示控制一定稳定。

不能。
有限数字只说明当前计算没有直接溢出成 `NaN` 或 `Inf`。
稳定性还依赖姿态方向、时间步长、轴使能、输出限幅、速率限制、电机映射和实物响应。
本章只能证明 PID 计算链路存在，不能单独证明仓库外实测稳定。

问题八：`B` 在配置里被赋值，是否说明当前姿态 PID 使用了它。

不能。
源码中 `B` 只出现在 `OTHER` 分支公式里。
当前 `config.c` 把三轴 PID 的 `type` 设置为 `ANGULAR`，
所以 ROLL/PITCH/YAW 姿态 PID 主线使用的是 `P * error + I * iTerm + D * dAverage`。

问题九：为什么 D 项最容易受噪声和周期抖动影响。

因为 D 项本质上是变化量除以 `deltaT`。
当角度测量有抖动、目标突然变化，或者 `deltaT` 异常变小时，微分值可能被放大。
这也是项目需要 D 项滤波、历史平均和输出约束的原因。

问题十：`iTerm` 是不是积分输出本身。

不是。
`iTerm` 是误差随时间累计的状态，最终积分输出还要乘以 `I` 参数。
调试时要同时看 `iTerm` 和 `I * iTerm`，否则容易误判积分项实际贡献。

问题十一：`standardRadianFormat()` 是否能处理任意大的角度。

不能按通用归一化函数理解。
它只做一次 `2π` 加减，适合单圈附近的角度误差。
如果输入可能是多圈角度，应先在控制层使用类似 `wrapToPif()` 的循环包裹。

问题十二：`D_STATE` 是否只是把 D 项输入从 error 换成 state。

不能这样简化。当前源码中 `D_ERROR` 是先包裹角度差再除以 `deltaT`，
而 `D_STATE` 是先除以 `deltaT` 再调用 `standardRadianFormat()`。
当前三轴默认 `D_ERROR`，所以 `D_STATE` 的真实控制效果仍需单独验证【待验证】。

问题十三：当前 PID 是位置式还是增量式。

按 `updatePID()` 源码看，它是位置式离散 PID。
函数每次直接返回 `P * error + I * iTerm + D * dAverage` 组合出的控制量，
没有计算 `du`，也没有在 PID 内部用上一帧输出累加得到本帧输出。
`pidCmdPrev[]` 出现在后续轴控制层，服务输出速率限制，不表示 PID 本体是增量式。

问题十四：为什么当前项目不直接使用 LQR 或 MPC。

当前仓库没有状态空间模型、反馈增益矩阵、预测时域、约束优化器或在线求解器证据。
PID 对模型依赖更低、计算量更小、参数含义更直观，更符合当前 STM32F103 和现有源码结构。
如果未来采用 LQR/MPC，应作为新的控制架构设计，而不能从当前 PID 源码直接推出。

问题十五：开环输出和闭环 PID 有什么本质区别。

开环只按预设命令输出，不检查姿态是否真的到达目标。
闭环会把 `sensors.margAttitude500Hz` 反馈回来，持续修正目标角与当前角之间的误差。
本项目的硬件诊断可以使用开环式输出，但姿态稳定依赖闭环 PID。

问题十六：`windupGuard` 被赋值，是否说明当前积分限幅来自它。

不能。
当前 `updatePID()` 更新积分时没有读取 `windupGuard` 字段，
而是把 `temp_iTerm` 固定夹在 `[-10, 10]`。
PITCH 额外限幅分支当前也使用同样的 `[-10, 10]`，
所以不能把源码注释中的“更严格”直接写成已实现行为。

问题十七：Pitch/Yaw 的积分暂停是否完全由各自轴内误差决定。

不能这样理解。
Pitch/Yaw 调用 `updatePID()` 时传入的是全局 `holdIntegrators`。
当前 `computeMotorCommands()` 中 Roll 分支位于 Pitch/Yaw 前面，
Roll 后续 PID 分支在电角误差过大时可能把全局 `holdIntegrators` 置为 `true`。
同帧内 Pitch/Yaw 是否因此暂停积分，需要结合轴使能和分支状态用日志确认【待验证】。

问题十八：`lastDcalcValue == 0.0f` 为什么是一个需要记录的边界。

因为 0 既可能表示 D 项历史尚未初始化，也可能表示上一帧误差或状态真实为 0。
源码没有单独的有效标志来区分这两种情况。
当误差从精确 0 变为非零时，第一帧 D 项变化可能因为重新初始化而被清掉。
这不一定造成实际问题，但属于 D 项初始化逻辑必须说明的源码边界【待验证】。

问题十九：D 项被限制到 `[-300, 300]` 是否说明系统安全。

不能。
这只说明 `updatePID()` 对原始 D 项数值做了夹紧。
系统安全还取决于 PID 输出限幅、输出速率限制、电角映射、PWM 下发、
电机负载、供电和机械结构。缺少硬件记录时只能写成数值防护，不能写成安全证明。

问题二十：同一个 `rateLimit` 字段在三轴输出速率限制中是否一定具有相同单位语义。

不能直接这样假设。
Roll 后续 PID 分支把 `eepromConfig.rateLimit` 乘以 `safeDt` 后作为单帧步长；
Pitch/Yaw 分支则直接把 `pidCmd - pidCmdPrev` 与 `eepromConfig.rateLimit` 比较。
因此 Roll 的写法更像“每秒速率转换为每帧步长”，Pitch/Yaw 的写法更像“每帧最大变化量”。
这不属于 `updatePID()` 本体，而是 PID 后级输出约束；是否为有意的轴差异或遗留实现，
必须结合设计意图和实测日志确认【待验证】。

## 11. 实践任务

开始任务前，先回到本章第8节定位 `initPID()`、PID 参数来源、`updatePID()` 输入、P/I/D 输出和三轴分支入口；第9节提供 PID 调试顺序。

任务一：画出 PID 调用链。

从 `main()` 中 `computeMotorCommands(dt500Hz)` 开始，画到 `Pitch/Yaw/Roll` 分支中的 `updatePID()` 调用。
验收依据是 PID 调用链图包含入口函数、三轴分支、`updatePID()` 调用和 Roll 分支差异。

任务二：列出 PID 参数来源。

分别记录 `config.c` 和 `main.c` 中 ROLL/PITCH/YAW 的 P/I/D 值，并说明当前运行时应以哪一处为准。
验收依据是参数来源表包含 `config.c` 默认值、`main.c` 覆盖值、先后关系和运行结论。

任务三：验证 Pitch PID 输入。

观察 `pointingCmd[PITCH]`、`sensors.margAttitude500Hz[PITCH]`、`error_mech`、`dt`
和 `pidCmd[PITCH]`。
说明 `target_angle/pitch_angle` 如何对应 `command/state`，并说明当前 `error_mech`
没有作为 `updatePID()` 的实参。
验收依据是 PID 输入表包含变量名、实参位置、角色、观察值和“是否实际进入 PID 实参”。

任务四：验证 Yaw 角度包裹。

观察 `yaw_angle`、`target_angle`、`error_mech` 和 `safe_target`，说明为什么 Yaw 在进入 PID 前要处理角度环绕。
验收依据是 Yaw 误差表包含相关变量、`wrapToPif()` 调用位置和环绕结论。

任务五：验证 Roll 状态分支。

观察 `return_state_roll`。说明 Roll 在回中阶段和后续 PID 阶段的调用路径有什么不同。
验收依据是 Roll 分支表包含回中路径、后续 PID 路径和输出影响结论。

任务六：验证 PID 状态清零。

观察 `startup_delay_counter == 1000` 时 `zeroPIDintegralError()` 和 `zeroPIDstates()` 是否执行。
记录清零前后的 `iTerm` 和 D 项历史值。
验收依据是清零记录表包含触发条件、清零函数、清零前后值和门控顺序。

任务七：推导一帧 PID 输出。

任选一个轴，记录 `command`、`state`、`deltaT`、`error`、`iTerm`、`dAverage` 和 P/I/D 参数。
手算 `P * error + I * iTerm + D * dAverage`，与 `updatePID()` 返回值对比。
验收依据是手算表能解释三项贡献和总输出之间的关系。

任务八：验证 `B` 字段边界。

记录 ROLL/PITCH/YAW 的 `type` 和 `B` 值，再定位 `updatePID()` 中 `ANGULAR` 与 `OTHER` 输出分支。
说明为什么当前三轴姿态 PID 不受 `B` 影响。
验收依据是分支验证表包含 `type`、公式分支、`B` 是否参与输出和源码行证据。

任务九：验证 D 项来源。

记录 `dErrorCalc`，说明当前使用 `D_ERROR` 还是 `D_STATE`。
构造目标值变化和状态值变化两种场景，分析 D 项会响应哪一种变化。
验收依据是 D 项来源表包含 `dErrorCalc`、上一帧值、当前值、`deltaT` 和 D 项变化方向。

任务十：验证角度包裹函数边界。

分别用 `PI`、`-PI`、`5PI` 和 `-5PI` 对比 `standardRadianFormat()`
与 `wrapToPif()` 的输出。
验收依据是表格能说明 `standardRadianFormat()` 只做一次 `2π` 修正，
而 `wrapToPif()` 会循环修正；多圈输入不能只依赖 PID 内部单次包裹。

任务十一：验证 D_STATE 分支边界。

在不修改主线固件的前提下，阅读 `pid.c` 中 `D_STATE` 分支的代码顺序，
说明它是先除以 `deltaT` 再调用 `standardRadianFormat()`。
验收依据是写出 `D_ERROR` 与 `D_STATE` 的计算顺序差异，并说明当前三轴默认
`D_ERROR`，所以 `D_STATE` 行为属于待验证边界。

任务十二：验证位置式 PID 边界。

阅读 `updatePID()` 的返回公式，确认它直接返回当前帧 `output`。
再搜索是否存在 PID 内部 `lastOutput` 或 `deltaOutput` 累加逻辑。
验收依据是能说明当前 PID 本体属于位置式离散 PID，而 `pidCmdPrev[]`
属于后续输出速率限制，不是增量式 PID 证据。

任务十三：验证非 LQR/MPC 架构边界。

搜索状态空间矩阵、反馈增益矩阵、预测时域、约束代价函数和在线优化求解器相关代码。
若没有这些证据，说明当前项目不能被写成 LQR 或 MPC 控制架构。
验收依据是搜索记录能区分“当前 PID 证据”和“未来可能迁移到 LQR/MPC 的设计假设”。

任务十四：验证积分暂停跨轴边界。

在 `computeMotorCommands()` 中按执行顺序记录 `holdIntegrators`、`rollHoldIntegrators`
和三轴 `updatePID()` 的 `iHold` 实参。
验收依据是表格能说明 Roll 分支是否在同一帧影响 Pitch/Yaw 积分暂停，
并把没有实测日志的组合标为【待验证】。

任务十五：验证 `windupGuard` 与实际积分限幅。

记录 `main.c` 中三轴 `windupGuard`、`I` 参数和 `updatePID()` 内部 `temp_iTerm`。
然后说明当前积分状态为什么被固定限制在 `[-10, 10]`，
以及 PITCH 额外分支为什么没有体现更严格数值。
验收依据是能区分配置字段、源码注释、实际限幅常数和最终 `I * iTerm` 输出贡献。

任务十六：验证 D 项哨兵值和数值防护。

构造或记录误差为 0、误差从 0 变为非零、误差跨 `±PI` 三类场景。
观察 `lastDcalcValue`、原始 `dTerm`、限幅后 `dTerm`、`dTermFiltered` 和 `output`。
验收依据是能说明 `lastDcalcValue == 0.0f` 的初始化边界、
`[-300, 300]` 的 D 项限幅，以及 NaN/Inf 置零只属于数值防护。

任务十七：验证 D 项滤波系数和三点平均。

记录 `F_CUT`、`rc`、`deltaT`、`alpha = deltaT / (rc + deltaT)`、
`dTerm`、`dTermFiltered`、`lastDterm`、`lastLastDterm` 和 `dAverage`。
在 `deltaT ~= 0.002s` 时计算 `alpha` 的数量级，再观察异常 `deltaT`
回退为 `0.002f` 后滤波系数是否同步回退。
验收依据是能说明 D 项从原始差分到 `dAverage` 的完整路径，
并能区分“D 增益设置问题”“原始微分噪声问题”和“滤波状态滞后问题”。

任务十八：验证输出速率限制的单位语义。

分别记录 Roll 后续 PID 分支、Pitch 分支和 Yaw 分支中的
`pidCmdPrev[]`、`pidCmd[]`、`outputRate[]`、`eepromConfig.rateLimit`
以及 `dt/safeDt`。
对 Roll 计算 `rateLimit * safeDt`，对 Pitch/Yaw 记录直接比较 `rateLimit`
时的单帧允许变化量。
验收依据是表格能说明同一 `rateLimit` 字段在不同轴上是按“每秒速率”还是“每帧步长”生效，
并把没有设计意图或硬件日志支持的结论标为【待验证】。

实践边界：

当前任务优先形成表格、链路图、搜索记录和计算过程。涉及 IDE 现场、构建日志、断点数值、外部波形、主机侧结果或硬件响应时，若没有截图、日志或仓库外实测证据，结论保持【待验证】。

## 12. 思考题

1. 为什么 PID 控制必须同时知道目标值、当前值和时间步长。
2. P 项、I 项、D 项分别在云台控制中解决什么问题，又可能带来什么副作用。
3. 如果 `dt500Hz` 偶尔变大，积分项和微分项分别会受到什么影响。
4. 为什么角度误差必须考虑 `[-PI, PI]` 环绕。
5. 为什么 Roll 的 PID 路径不能只看 `eepromConfig.rollEnabled`，还要看 `return_state_roll`。
6. 如果 PID 输出正常但电机方向错误，为什么下一步应检查电角转换和电机映射，而不是只调 P/I/D。
7. 如果 PID 参数被覆盖后看起来“更稳”，为什么仍要记录覆盖来源和当前生效位置。
8. PID 输出正常时，还需要哪些第29章、第30章和仓库外实测证据，才能判断电机真实响应可信？
9. 为什么 `iTerm` 要保存误差积分状态，而不是每帧只根据当前误差计算。
10. 为什么 `D_ERROR` 在目标值阶跃变化时可能产生更大的瞬态响应。
11. 如果 `B=1.0f` 但当前轴是 `ANGULAR` 类型，为什么 `B` 仍然不影响输出。
12. 在没有硬件 FPU 的 Cortex-M3 上，为什么三轴 PID 的浮点计算仍要纳入 2ms 预算审查。
13. 为什么 `standardRadianFormat(5PI)` 不会得到 `PI` 或 `-PI` 范围内的结果。
14. 为什么 `D_STATE` 分支的角度处理顺序需要单独验证，而不能直接套用 `D_ERROR` 的结论。
15. 为什么 `pidCmdPrev[]` 不能证明 `updatePID()` 是增量式 PID。
16. 为什么没有模型和优化器证据时，教材不能把当前项目讲成 LQR/MPC 控制。
17. 为什么硬件诊断可以开环，而姿态稳定必须依赖闭环反馈。
18. 为什么 `windupGuard` 字段存在，不等于当前积分限幅一定由它决定。
19. 为什么 Roll 分支中的 `holdIntegrators` 可能影响同一帧后续 Pitch/Yaw 的积分暂停。
20. 为什么用 `0.0f` 作为 D 项历史初始化哨兵会带来边界歧义。
21. 为什么 D 项和输出的 NaN/Inf 防护只能证明数值被兜底，不能证明闭环稳定。
22. 为什么 D 项限幅之后仍然要理解低通滤波和三点平均，而不能只看原始 `dTerm`。
23. 为什么同一个 `rateLimit` 字段在 Roll 与 Pitch/Yaw 中是否乘以 `dt`，会影响你对“速率限制”的工程判断。

## 13. 本章总结

本章建立了三轴云台项目中的 `PID控制` 主干。

当前项目中：

- `Drivers/SRC/Inc/pid.h` 定义 `PIDdata_t` 和 PID 函数接口。
- `Drivers/BGC/bgc32.h` 定义 ROLL/PITCH/YAW 三个 PID 索引。
- `Drivers/SRC/Src/config.c` 设置默认 PID 参数。
- `Core/Src/main.c` 调用 `initPID()`，并在当前调试初始化中覆盖部分 PID 参数。
- `updatePID()` 接收 `command`、`state`、`deltaT`、`iHold` 和当前轴 PID 参数。
- `ANGULAR` 类型会让 PID 误差进入角度归一化。
- 当前三轴姿态 PID 的离散主公式是 `P * error + I * iTerm + D * dAverage`。
- `iTerm` 保存的是误差积分状态，不是已经乘以 `I` 的积分输出。
- `PIDdata_t` 同时包含配置字段和跨帧状态字段，不能把 `eepromConfig.PID[]` 只理解为静态参数表。
- `D_ERROR` 表示 D 项来自误差变化，`D_STATE` 表示 D 项来自状态变化。
- `standardRadianFormat()` 是 PID 内部单次角度包裹函数，不是多圈通用归一化器。
- `D_ERROR` 与 `D_STATE` 的角度包裹位置不同；当前三轴默认使用 `D_ERROR`。
- `B` 字段只在 `OTHER` 分支中参与输出，当前三轴 `ANGULAR` 主线不使用它。
- `windupGuard` 当前没有被 `updatePID()` 作为积分限幅来源读取。
- 当前积分状态实际被固定夹到 `[-10, 10]`，PITCH 额外分支当前没有更严格数值。
- Roll 后续 PID 分支可能通过全局 `holdIntegrators` 影响同一帧后续 Pitch/Yaw 的积分暂停【待验证】。
- `lastDcalcValue == 0.0f` 同时承担初始化判断和值存储角色，存在 D 项哨兵值边界。
- 原始 `dTerm` 会被限制到 `[-300, 300]`，最终输出 NaN/Inf 会被置 0。
- `dTerm` 还会经过 `alpha = deltaT / (rc + deltaT)` 的一阶低通滤波和三点平均，最终以 `dAverage` 进入输出公式。
- `updatePID()` 会通过 `PIDdata_t *` 写回 `iTerm`、`lastDcalcValue`、`lastDterm`
  和 `lastLastDterm`，因此它是带跨帧状态的控制器更新，不是只读当前误差的纯公式函数。
- 当前 `updatePID()` 是位置式离散 PID，不是增量式 PID。
- `pidCmdPrev[]` 和 `outputRate[]` 是 PID 后级输出约束，其中 Roll 后续 PID 分支使用 `rateLimit * safeDt`，Pitch/Yaw 当前直接使用 `rateLimit`，存在单位语义边界【待验证】。
- 当前控制链路是姿态反馈闭环，不是开环固定输出。
- 当前仓库没有 LQR/MPC 所需的模型、矩阵、预测时域或优化求解器证据。
- Pitch 和 Yaw 在轴使能时进入 PID 调用。
- Pitch 分支虽然计算 `error_mech = wrapToPif(target_angle - pitch_angle)`，但当前没有把
  `error_mech` 作为 PID 实参；Yaw 分支则用 `error_mech` 构造 `safe_target` 后进入 PID。
- Roll 的 PID 调用受 `return_state_roll` 状态分支影响，不能写成无条件每帧同样调用。
- `zeroPIDintegralError()` 和 `zeroPIDstates()` 在 AHRS 收敛后用于清空 PID 历史状态。
- 当前 `.map/.list` 能证明 `initPID()`、`updatePID()`、`zeroPIDintegralError()`、
  `zeroPIDstates()`、`computeMotorCommands()`、`standardRadianFormat()`、`wrapToPif()` 以及
  `eepromConfig`、`holdIntegrators`、`pidCmd[]`、`pidCmdPrev[]`、`outputRate[]`、`rollDiag`
  已进入当前 Debug 构建，并能看到 `main()` 与三轴 `updatePID()` 的反汇编调用点。
- `.su/.cyclo` 还能补充 `updatePID()`、`initPID()`、`zeroPIDintegralError()`、
  `zeroPIDstates()`、`standardRadianFormat()`、`computeMotorCommands()` 和 `main()` 的静态栈与圈复杂度，
  但不能替代运行时栈水位、真实耗时或闭环稳定性证据。

本章保留十二个边界：

- D 项滤波的调参效果、目标角斜坡、输出限幅和积分抗饱和留到第28章展开。
- 机械角到电角转换和三相 PWM 输出留到后续电机章节展开。
- 当前 PID 参数存在 `config.c` 默认值和 `main.c` 调试覆盖值并存的情况，分析运行行为时必须以实际写入顺序为准。
- `windupGuard` 是配置字段，但当前积分限幅执行来源是 `updatePID()` 内部固定常数。
- Pitch/Roll 当前 `I=0.0` 时，`iTerm` 状态更新不等于输出积分项已经生效。
- `holdIntegrators` 的同帧跨轴影响需要轴使能、Roll 分支状态和实测日志共同确认【待验证】。
- `lastDcalcValue == 0.0f` 的哨兵值写法不能替代独立初始化有效标志。
- D 项滤波系数由 `deltaT` 参与计算，异常 `deltaT` 回退会同时影响微分斜率和滤波响应。
- 三轴共享 `updatePID()` 代码但状态对象不同，断点和日志必须标明当前传入的是
  `&eepromConfig.PID[ROLL_PID]`、`&eepromConfig.PID[PITCH_PID]` 还是 `&eepromConfig.PID[YAW_PID]`。
- `pidCmdPrev[]` 和 `outputRate[]` 属于 PID 后级输出约束，不改变 `updatePID()` 的位置式 PID 属性。
- 同一 `rateLimit` 字段在 Roll 与 Pitch/Yaw 输出速率限制中的 `dt` 使用方式不同，是否为有意轴差异需要设计意图或实测日志确认【待验证】。
- LQR/MPC 只能作为未来架构迁移讨论，不能写成当前项目采用的控制方法。
- 当前仓库和 Debug 构建产物能证明公式分支、变量状态、符号进入镜像和反汇编调用点，不能单独证明闭环稳定性；稳定性仍需硬件日志或测试记录【待验证】。
- 当前 `.su/.cyclo` 不能写成运行时栈水位、单帧耗时或 2ms 预算达标证明。

下一章将进入 `PID细节与输出约束`，进一步分析 D 项平滑、积分暂停、目标变化平滑、输出限幅和速率限制如何保护闭环控制不因瞬态误差或参数不当而失控。

本章新增参考资料：

- 本仓库 `Drivers/SRC/Inc/pid.h`。
- 本仓库 `Drivers/SRC/Src/pid.c`。
- 本仓库 `Drivers/SRC/Src/config.c`。
- 本仓库 `Core/Src/main.c`。
- 本仓库 `Drivers/SRC/Src/computeMotorCommands.c`。
- 本仓库 `Debug/Three-axis_cloud_platformV2.map`。
- 本仓库 `Debug/Three-axis_cloud_platformV2.list`。
- 本仓库 `Debug/Drivers/SRC/Src/pid.su`。
- 本仓库 `Debug/Drivers/SRC/Src/pid.cyclo`。
- 本仓库 `Debug/Drivers/SRC/Src/computeMotorCommands.su`。
- 本仓库 `Debug/Drivers/SRC/Src/computeMotorCommands.cyclo`。
- 本仓库 `Debug/Core/Src/main.su`。
- 本仓库 `Debug/Core/Src/main.cyclo`。
- K. J. Astrom and T. Hagglund, PID Controllers: Theory, Design, and Tuning:
  https://portal.research.lu.se/en/publications/pid-controllers-theory-design-and-tuning/
- K. J. Astrom and T. Hagglund, Advanced PID Control, Chapter 1:
  https://www.isa.org/getmedia/fb0e41bc-e4f3-422a-9f67-b9bd31340e16/Advanced-PID-Control_AstromHagglund_Chapter1-Introduction.pdf
- K. J. Astrom and T. Hagglund, The future of PID control:
  https://scispace.com/pdf/the-future-of-pid-control-16e4ycgco7.pdf

### 章节尾部固定检查

知识链路：

`知识点总表`
-> `知识依赖图`
-> `学习优先级`
-> `教学顺序`
-> `教材章节`

项目证据：

- `Drivers/SRC/Inc/pid.h`
- `Drivers/SRC/Src/pid.c`
- `Drivers/SRC/Src/config.c`
- `Core/Src/main.c`
- `Drivers/SRC/Src/computeMotorCommands.c`
- `Debug/Three-axis_cloud_platformV2.map`
- `Debug/Three-axis_cloud_platformV2.list`
- `Debug/Drivers/SRC/Src/pid.su`
- `Debug/Drivers/SRC/Src/pid.cyclo`
- `Debug/Drivers/SRC/Src/computeMotorCommands.su`
- `Debug/Drivers/SRC/Src/computeMotorCommands.cyclo`
- `Debug/Core/Src/main.su`
- `Debug/Core/Src/main.cyclo`
- 函数：`initPID()`
- 函数：`updatePID()`
- 函数：`zeroPIDintegralError()`
- 函数：`zeroPIDstates()`
- 函数：`standardRadianFormat()`
- 函数：`wrapToPif()`
- 全局对象：`eepromConfig`
- 全局对象：`holdIntegrators`
- 全局对象：`pidCmd[]`
- 全局对象：`pidCmdPrev[]`
- 全局对象：`outputRate[]`
- 全局对象：`rollDiag`
- 宏：`PI`
- 宏：`TWO_PI`

质量自检：

- P0 事实错误：通过
- P1 依赖断层：通过
- P2 逻辑连贯：通过
- P3 项目证据：通过
- P4 原理展开：通过
- P5 调试实践：通过
- P6 表达统一：通过

---
> 导航：上一章：[第26章_500Hz实时控制循环](第26章_500Hz实时控制循环.md) ｜ 下一章：[第28章_PID细节与输出约束](第28章_PID细节与输出约束.md)
