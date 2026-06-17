# 第27章 PID控制核心

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

`windupGuard` 字段虽然存在，并且 `config.c` 与 `main.c` 都会写入它，但当前 `updatePID()` 的积分限幅没有读取该字段，而是使用函数内部的固定范围。教材后续分析积分抗饱和时必须以实际代码为准，不能只根据字段名推断行为。

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

第四步，计算 D 项。

源码支持两种 D 项来源：

- `D_ERROR`：根据误差变化计算。
- `D_STATE`：根据状态变化计算。

当前默认配置中 ROLL/PITCH/YAW 的 `dErrorCalc` 都设置为 `D_ERROR`。第28章会继续展开 D 项滤波和历史状态。

第五步，组合输出。

对于 `ANGULAR` 类型，`updatePID()` 的核心输出形式是：

```text
output = P * error + I * iTerm + D * dAverage
```

这就是本章最重要的 PID 主公式。

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

### 8.3 updatePID输入

`updatePID()` 的输入把控制问题压缩成五个量：

```text
command：目标
state：当前状态
deltaT：时间间隔
iHold：是否暂停积分
PIDparameters：当前轴参数和历史状态
```

其中 `deltaT` 来自第26章的 `dt500Hz` 或轴内的 `safeDt`。源码中如果 `deltaT` 超过 `0.01f`、小于 `0.0001f`、为 NaN 或 Inf，会回退为 `0.002f`。这个防护属于后续异常防护章节，本章只记录它会影响 PID 时间步长。

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

### 8.5 P/I/D输出

在 `ANGULAR` 分支中，输出组合为：

```text
P * error + I * iTerm + D * dAverage
```

其中：

- `error` 来自目标和当前角度。
- `iTerm` 来自历史误差累计。
- `dAverage` 来自 D 项计算和滤波历史。

本章不展开 `dAverage` 的滤波推导，也不展开积分何时暂停。它们分别属于第28章的 `PID微分项滤波` 和后续 `积分抗饱和`。

### 8.6 computeMotorCommands中的Pitch入口

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

### 8.7 computeMotorCommands中的Yaw入口

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

### 8.8 computeMotorCommands中的Roll入口

Roll 轴的当前实现有明显状态分支。

当 `eepromConfig.rollEnabled == true` 且 `return_state_roll == true` 时，代码主要通过 `rollTargetSlew` 和 `PWM_Motor_SetAngle()` 做回中/过渡输出，没有进入 `updatePID()`。

当进入后续分支时，代码会计算当前电角、目标电角和电角误差，并调用：

```text
updatePID(target_electrical_angle,
          current_electrical_angle,
          safeDt,
          rollHoldIntegrators,
          &eepromConfig.PID[ROLL_PID])
```

所以不能简单写成“Roll 每帧都与 Pitch/Yaw 一样调用 PID”。更准确的说法是：Roll 的 PID 调用受轴使能和回中状态分支影响。

### 8.9 PID状态清零

`main.c` 在 AHRS 收敛计数达到 1000 帧后调用：

```text
zeroPIDintegralError()
zeroPIDstates()
```

这说明控制接通前会清空积分项和 D 项历史状态，降低开机瞬间历史状态造成的冲击。第32章运行门控和异常防护会继续分析这个行为的系统意义。

### 本节证据边界

本节只根据当前仓库说明文件、函数、宏、变量和调用关系。运行时频率、外部硬件表现、主机侧现象、传感器方向、电机响应或真实控制效果仍需调试记录、日志或仓库外实测证据；缺少证据时保持【待验证】。

## 9. 调试方法

第一步，确认 PID 是否初始化。

- 在 `Core/Src/main.c` 中确认 `initPID()` 已经在进入主循环前调用。
- 观察 `eepromConfig.PID[ROLL/PITCH/YAW]` 的 `iTerm` 和 D 项历史状态是否清零。

第二步，确认当前运行参数。

- 先看 `Drivers/SRC/Src/config.c` 中默认 P/I/D。
- 再看 `Core/Src/main.c` 中是否覆盖了这些参数。
- 调试当前工程时，以实际运行前最后一次写入的参数为准。

第三步，确认输入来源。

- Pitch/Yaw 调试时观察 `sensors.margAttitude500Hz[PITCH/YAW]`。
- Roll 调试时同时观察 `return_state_roll`，确认当前是否已经进入 Roll PID 分支。
- 观察 `pointingCmd[]` 是否符合目标角预期。

第四步，确认时间步长。

- 观察传入 `computeMotorCommands(dt500Hz)` 的 `dt500Hz`。
- 观察 `updatePID()` 内部是否把异常 `deltaT` 回退为 `0.002f`。

第五步，确认输出结果。

- 在 `updatePID()` 返回处观察 `output`。
- 在 `computeMotorCommands()` 中观察 `pidCmd[PITCH]`、`pidCmd[YAW]` 和进入 Roll PID 分支后的 `pidCmd[ROLL]`。
- 如果 PID 输出正常但电机输出异常，后续应进入电角转换、限幅和 PWM 输出章节，而不是只改 P/I/D 参数。

调试记录建议：

- 记录 `initPID()` 调用、实际 P/I/D 参数来源、输入姿态、目标角和 `dt500Hz`。
- 对每个轴，应记录误差、P 项、I 项、D 项、积分暂停状态和 `updatePID()` 返回值。
- Roll 分支记录应包含 `return_state_roll`、目标斜坡和是否进入 Roll PID 调用。
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

## 11. 实践任务

开始任务前，先回到本章第8节定位 `initPID()`、PID 参数来源、`updatePID()` 输入、P/I/D 输出和三轴分支入口；第9节提供 PID 调试顺序。

任务一：画出 PID 调用链。

从 `main()` 中 `computeMotorCommands(dt500Hz)` 开始，画到 `Pitch/Yaw/Roll` 分支中的 `updatePID()` 调用。
验收依据是 PID 调用链图包含入口函数、三轴分支、`updatePID()` 调用和 Roll 分支差异。

任务二：列出 PID 参数来源。

分别记录 `config.c` 和 `main.c` 中 ROLL/PITCH/YAW 的 P/I/D 值，并说明当前运行时应以哪一处为准。
验收依据是参数来源表包含 `config.c` 默认值、`main.c` 覆盖值、先后关系和运行结论。

任务三：验证 Pitch PID 输入。

观察 `pointingCmd[PITCH]`、`sensors.margAttitude500Hz[PITCH]`、`dt` 和 `pidCmd[PITCH]`。
说明它们如何对应 `command/state/deltaT/output`。
验收依据是 PID 输入表包含变量名、实参位置、角色和观察值。

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

## 13. 本章总结

本章建立了三轴云台项目中的 `PID控制` 主干。

当前项目中：

- `Drivers/SRC/Inc/pid.h` 定义 `PIDdata_t` 和 PID 函数接口。
- `Drivers/BGC/bgc32.h` 定义 ROLL/PITCH/YAW 三个 PID 索引。
- `Drivers/SRC/Src/config.c` 设置默认 PID 参数。
- `Core/Src/main.c` 调用 `initPID()`，并在当前调试初始化中覆盖部分 PID 参数。
- `updatePID()` 接收 `command`、`state`、`deltaT`、`iHold` 和当前轴 PID 参数。
- `ANGULAR` 类型会让 PID 误差进入角度归一化。
- `windupGuard` 当前没有被 `updatePID()` 作为积分限幅来源读取。
- Pitch 和 Yaw 在轴使能时进入 PID 调用。
- Roll 的 PID 调用受 `return_state_roll` 状态分支影响，不能写成无条件每帧同样调用。
- `zeroPIDintegralError()` 和 `zeroPIDstates()` 在 AHRS 收敛后用于清空 PID 历史状态。

本章保留三个边界：

- D 项滤波、目标角斜坡、输出限幅和积分抗饱和留到第28章展开。
- 机械角到电角转换和三相 PWM 输出留到后续电机章节展开。
- 当前 PID 参数存在 `config.c` 默认值和 `main.c` 调试覆盖值并存的情况，分析运行行为时必须以实际写入顺序为准。

下一章将进入 `PID细节与输出约束`，进一步分析 D 项平滑、积分暂停、目标变化平滑、输出限幅和速率限制如何保护闭环控制不因瞬态误差或参数不当而失控。

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

质量自检：

- P0 事实错误：通过
- P1 依赖断层：通过
- P2 逻辑连贯：通过
- P3 项目证据：通过
- P4 原理展开：通过
- P5 调试实践：通过
- P6 表达统一：通过
