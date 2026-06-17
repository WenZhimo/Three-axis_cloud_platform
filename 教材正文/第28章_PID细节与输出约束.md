# 第28章 PID细节与输出约束

## 1. 本章目标

第27章已经建立 PID 控制主干：目标值、当前值、误差、`dt` 和 P/I/D 输出。本章继续进入 PID 周边的保护和约束逻辑：微分项如何平滑，目标角如何避免突变，PID 输出如何限制，积分为什么有时要暂停。

本章目标是：

- 理解 `PID微分项滤波` 如何降低 D 项对噪声和瞬态变化的敏感性。
- 理解 `目标角斜坡逼近` 如何让目标角逐步变化，而不是瞬间跳变。
- 理解 `输出限幅与速率限制` 如何限制补偿量大小和变化速度。
- 理解 `积分抗饱和` 如何避免误差过大时积分项继续累积。
- 明确当前源码中 Roll、Pitch、Yaw 三轴细节并不完全一致，教材必须按分支分别分析。

本章知识链为：

`知识点总表` 中的 `PID微分项滤波`、`目标角斜坡逼近`、`输出限幅与速率限制`、`积分抗饱和`
-> `知识依赖图` 中它们分别依赖 `PID控制`、`一阶低通滤波`、`角度归一化`、`目标角斜坡逼近` 和 `输出限幅与速率限制`
-> `学习优先级` 中 P1/P3 控制保护知识
-> `教学顺序` 第46到第49项
-> `教材章节` 第28章。

## 2. 前置知识

本章正式前置知识包括：

- `一阶低通滤波`
- `PID控制`
- `角度归一化`
- `目标角斜坡逼近`
- `输出限幅与速率限制`

这些前置主要对应第22章、第23章和第27章。

这里有两个“本章内前置”：

- `输出限幅与速率限制` 需要先理解 `目标角斜坡逼近`。
- `积分抗饱和` 需要先理解 `输出限幅与速率限制`。

因此本章的学习顺序是：先看 D 项滤波，再看目标角斜坡，再看输出限幅和速率限制，最后看积分暂停和积分限幅。

本章不展开机械角到电角转换，不展开三相 PWM 输出，不展开完整调参流程。那些内容分别属于后续电机章节和 PID 调参章节。

## 3. 问题背景

如果只使用第27章的 PID 主公式，控制系统仍然容易出现问题。

第一，D 项对变化速度敏感，也会放大噪声。如果姿态角有轻微抖动，未经处理的 D 项可能产生很尖的输出。

第二，目标角如果瞬间跳变，PID 会立刻看到很大的误差，输出也会突然变大。电机可能因此出现抽动。

第三，PID 输出如果没有限幅，参数稍大或误差稍大时就可能给出过大的补偿角。

第四，输出已经被限制住时，积分项如果继续累积，就可能在误差消失后仍然留下很大的历史补偿，造成过冲。

本项目在 `pid.c` 和 `computeMotorCommands.c` 中加入了多层保护：

```text
D项滤波
-> 目标角斜坡
-> PID输出限幅
-> 输出变化率限制
-> 积分暂停和积分限幅
```

这些保护不是装饰代码，而是让云台闭环能更稳定地运行。

## 4. 核心概念

### 4.1 PID微分项滤波

微分项关注“变化速度”。变化速度越快，D 项越大。问题是传感器噪声也会表现为快速变化，所以 D 项需要平滑。

`pid.c` 中定义：

```c
#define F_CUT 20.0f
static float rc;
```

`initPID()` 中计算：

```text
rc = 1 / (2 * PI * F_CUT)
```

`updatePID()` 中再使用一阶低通形式：

```text
dTermFiltered = lastDterm + deltaT / (rc + deltaT) * (dTerm - lastDterm)
```

这就是第22章的一阶低通思想在 PID D 项上的应用。

### 4.2 三点平均dAverage

滤波后的 D 项还会和历史 D 项做平均：

```text
dAverage = (dTermFiltered + lastDterm + lastLastDterm) / 3
```

随后更新历史：

```text
lastLastDterm = lastDterm
lastDterm = dTermFiltered
```

这进一步降低 D 项瞬时尖峰。

### 4.3 目标角斜坡逼近

目标角斜坡逼近是指目标值不一次跳到最终目标，而是每帧最多移动一个小步长。

`computeMotorCommands.c` 中有两个工具函数：

```c
moveTowardsf()
moveTowardsAnglef()
```

`moveTowardsAnglef()` 会先用 `wrapToPif()` 计算角度最短路径，再限制每次变化量。

当前实际调用 `moveTowardsAnglef()` 的路径集中在 Roll 轴。`PITCH_TARGET_SLEW_RAD_S` 和 `YAW_TARGET_SLEW_RAD_S` 当前只发现定义，没有发现参与当前 Pitch/Yaw 控制路径的调用证据。

### 4.4 输出限幅

输出限幅是把 PID 输出限制在允许范围内。例如：

```text
ROLL_CMD_LIMIT_RAD
PITCH_CMD_LIMIT_RAD
YAW_CMD_LIMIT_RAD
```

Roll 使用 `clampf()`，Pitch 和 Yaw 使用显式 `if` 判断。写法不同，但目的相同：防止 PID 输出过大。

### 4.5 速率限制

速率限制关注的是“本帧输出相对上一帧最多能变化多少”。

项目使用：

```text
outputRate[axis] = pidCmd[axis] - pidCmdPrev[axis]
pidCmdPrev[axis] = pidCmd[axis]
```

Roll 分支把 `eepromConfig.rateLimit` 乘以 `safeDt` 得到步长限制，并设置最小步长。

Pitch/Yaw 分支当前直接用 `eepromConfig.rateLimit` 作为每帧差值限制。

由于 `config.c` 注释称 `rateLimit = 45.0f * D2R` 是速率限制，Pitch/Yaw 当前是否也应乘以 `dt` 需要在后续调参或代码整理中继续验证，本章记录为【待验证】。

### 4.6 积分抗饱和

积分抗饱和用于防止积分项在不合适的时候继续累积。当前项目有两层相关逻辑：

- `updatePID()` 在 `iHold == false` 时才更新 `iTerm`。
- `updatePID()` 把 `iTerm` 限制到 `[-10, 10]`。

`windupGuard` 字段虽然存在并被赋值，但当前 `updatePID()` 没有使用它作为积分限幅来源。

## 5. 工作原理

### 5.1 D项从差分开始

`updatePID()` 先根据 `dErrorCalc` 选择 D 项来源。

当 `dErrorCalc == D_ERROR` 时：

```text
dInput = error - lastDcalcValue
dTerm = dInput / deltaT
lastDcalcValue = error
```

当 `dErrorCalc == D_STATE` 时：

```text
dTerm = (lastDcalcValue - state) / deltaT
lastDcalcValue = state
```

当前 `config.c` 默认把三轴 `dErrorCalc` 设置为 `D_ERROR`。这表示 D 项主要跟随误差变化。

### 5.2 D项先限幅再滤波

源码先对原始 `dTerm` 做异常和范围处理：

```text
NaN/Inf -> 0
dTerm 限制到 [-300, 300]
```

然后进入一阶低通：

```text
dTermFiltered = lastDterm + alpha * (dTerm - lastDterm)
alpha = deltaT / (rc + deltaT)
```

最后计算三点平均 `dAverage`。PID 输出中使用的是 `dAverage`，不是最原始的 `dTerm`。

### 5.3 Roll目标角斜坡

Roll 有两个斜坡相关路径。

在 `return_state_roll == true` 的回中阶段，如果 `pointingCmd[ROLL] == 0.0f`，代码用：

```text
ROLL_SLEW_RAD_S * safeDt
```

限制 `rollTargetSlew` 向目标角靠近。

在后续 Roll PID 分支中，代码用：

```text
ROLL_TARGET_SLEW_RAD_S * safeDt
```

限制 `rollTargetSlew` 向 `pointingCmd[ROLL]` 靠近。

这说明 Roll 的目标角并不总是直接跳到 `pointingCmd[ROLL]`，而是通过 `rollTargetSlew` 逐步过渡。

### 5.4 Pitch和Yaw目标角边界

Pitch/Yaw 当前定义了：

```text
PITCH_TARGET_SLEW_RAD_S
YAW_TARGET_SLEW_RAD_S
```

但在当前 `computeMotorCommands.c` 中，没有发现它们被用于 Pitch/Yaw 目标角过渡。Pitch 直接使用 `pointingCmd[PITCH]`，Yaw 先计算 `error_mech` 和 `safe_target`，再进入 PID。

所以第28章不能写成“三轴都使用目标角斜坡”。更准确的边界是：目标角斜坡机制当前在 Roll 路径中有运行证据，Pitch/Yaw 相关常量当前像是预留或未接入实现。

### 5.5 输出限幅和速率限制

PID 返回后，项目先限制输出大小，再限制输出变化速度。

Roll：

```text
pidCmd[ROLL] = clampf(rollPidRaw, -ROLL_CMD_LIMIT_RAD, ROLL_CMD_LIMIT_RAD)
rollStepLimit = eepromConfig.rateLimit * safeDt
pidCmd[ROLL] 相对 pidCmdPrev[ROLL] 限制在 ±rollStepLimit
```

Pitch：

```text
pidCmd[PITCH] 限制到 ±PITCH_CMD_LIMIT_RAD
pidCmd[PITCH] 相对 pidCmdPrev[PITCH] 限制在 ±eepromConfig.rateLimit
```

Yaw：

```text
pidCmd[YAW] 限制到 ±YAW_CMD_LIMIT_RAD
pidCmd[YAW] 相对 pidCmdPrev[YAW] 限制在 ±eepromConfig.rateLimit
```

三轴都有“幅值限制”和“变化限制”的思想，但实现细节不同。

### 5.6 积分暂停

`computeMotorCommands()` 每次进入时先执行：

```c
holdIntegrators = false;
```

Roll 后续 PID 分支根据电角误差计算：

```text
rollHoldIntegrators = fabs(roll_error_electrical) > ROLL_I_ENABLE_ERR_RAD
```

当 `rollHoldIntegrators` 为真时，传给 `updatePID()` 的 `iHold` 为真，积分项不更新。

Pitch/Yaw 当前传入的是全局 `holdIntegrators`。

由于函数开头把它置为 `false`，在没有 Roll 分支改写它的情况下，Pitch/Yaw 通常允许积分更新。

若 Roll 分支把 `holdIntegrators` 置为真，则同一帧后续 Pitch/Yaw 的积分也可能被暂停。

这个跨轴影响需要调试时注意。

## 6. STM32实现机制

本章内容仍然属于软件控制算法，不是外设配置。

STM32 相关性体现在三点：

第一，所有约束都运行在 500Hz 实时控制分支中。每帧的 `dt` 会影响 D 项计算、D 项滤波和 Roll 目标斜坡步长。

第二，输出约束最终会影响电机 PWM 指令。虽然本章不展开 `PWM_Motor_SetAngle()`，但 `pidCmd[]` 的限幅和变化率会决定后续电角输出的变化大小。

第三，调试时这些变量可以直接在 STM32 调试器中观察，例如 `dTermFiltered`、`pidCmd[]`、`pidCmdPrev[]`、`rollTargetSlew`、`holdIntegrators` 和 `eepromConfig.rateLimit`。

因此，本章是 PID 算法和电机输出之间的安全缓冲层。

## 7. 项目中的应用

本章涉及的项目证据文件包括：

- `Drivers/SRC/Src/pid.c`
- `Drivers/SRC/Src/computeMotorCommands.c`
- `Drivers/SRC/Src/config.c`
- `Core/Src/main.c`
- `tools/pid_tuning_sim.py`

`pid.c` 提供 D 项滤波、D 项限幅、积分更新和积分限幅。

`computeMotorCommands.c` 提供目标角斜坡、输出限幅、速率限制、积分暂停和三轴分支差异。

`config.c` 设置 `eepromConfig.rateLimit = 45.0f * D2R`。

`Core/Src/main.c` 设置部分 `windupGuard` 参数，并在 AHRS 收敛后调用 `zeroPIDintegralError()` 和 `zeroPIDstates()`。

`tools/pid_tuning_sim.py` 用 `RC_D_FILTER`、`cmd_limit` 和 `rate_limit` 模拟固件中的 PID 滤波和输出约束。它是调参辅助证据，不是固件运行路径。

## 8. 代码分析

### 8.1 D项滤波代码

`pid.c` 中 D 项滤波的主线是：

```text
F_CUT = 20Hz
rc = 1 / (2πF_CUT)
dTermFiltered = lastDterm + deltaT / (rc + deltaT) * (dTerm - lastDterm)
dAverage = (dTermFiltered + lastDterm + lastLastDterm) / 3
```

这段逻辑说明，PID 输出并不直接使用原始差分 D 项，而是使用经过低通和平滑后的 `dAverage`。

### 8.2 D项异常保护

在进入滤波前，源码处理了三类异常：

- `deltaT` 异常时回退到 `0.002f`。
- `dTerm` 是 NaN 或 Inf 时置 0。
- `dTerm` 被限制到 `[-300, 300]`。

这些逻辑让 D 项不至于因为异常 `dt` 或瞬时错误输入直接变成极端输出。

### 8.3 目标角斜坡代码

`moveTowardsAnglef()` 的核心思想是：

```text
delta = wrapToPif(target - current)
delta 限制到 [-maxStep, maxStep]
return wrapToPif(current + delta)
```

它比普通 `moveTowardsf()` 多了角度环绕处理，因此适合姿态角目标逼近。

当前运行证据显示它被 Roll 路径调用。`moveTowardsf()` 当前只看到定义，没有发现当前控制路径调用证据。

### 8.4 Roll输出约束

Roll 后续 PID 分支中：

```text
rollPidRaw -> clampf -> pidCmd[ROLL]
pidCmd[ROLL] - pidCmdPrev[ROLL] -> outputRate[ROLL]
rollStepLimit = eepromConfig.rateLimit * safeDt
```

如果变化超过 `rollStepLimit`，就把 `pidCmd[ROLL]` 限制到上一帧附近。`rollDiag` 还记录了 `pidRaw`、`pidClamped`、`pidApplied`、`dPidRaw` 和 `stepLimit`，适合调试 Roll 输出约束。

### 8.5 Pitch和Yaw输出约束

Pitch/Yaw 的限幅分两步。

第一步是幅值限幅：

```text
PITCH_CMD_LIMIT_RAD
YAW_CMD_LIMIT_RAD
```

第二步是相对上一帧的变化限制：

```text
outputRate[axis] = pidCmd[axis] - pidCmdPrev[axis]
if outputRate > eepromConfig.rateLimit:
    pidCmd = pidCmdPrev + eepromConfig.rateLimit
```

和 Roll 不同，Pitch/Yaw 当前没有乘以 `dt`。由于 `rateLimit` 在配置中按 `45.0f * D2R` 设置，这里是“每帧变化限制”还是遗漏了 `dt` 缩放，需要结合仓库外实测效果继续验证，本章标记为【待验证】。

### 8.6 积分限幅与windupGuard边界

`updatePID()` 中积分更新逻辑是：

```text
if iHold == false:
    iTerm += error * deltaT
    iTerm 限制到 [-10, 10]
```

代码中还写了 Pitch 轴额外限幅分支，但上下限仍然是 `[-10, 10]`，与全局限幅相同。

`PIDdata_t.windupGuard` 在配置中被写入，但当前没有被 `updatePID()` 读取。因此教材不能把 `windupGuard` 写成当前积分限幅实际来源。

### 8.7 仿真脚本边界

`tools/pid_tuning_sim.py` 中 `RC_D_FILTER = 1/(2π*20)`，与固件 `F_CUT=20Hz` 对应。脚本的 `update_pid()` 也包含：

```text
i_term 限制到 [-10, 10]
d_term 限制到 [-300, 300]
d_term_filtered 一阶低通
cmd_limit 限幅
rate_limit 限制输出变化
```

但脚本使用的是简化二阶对象模型，不等于真实电机、真实 IMU 和真实 PWM 输出。它适合做趋势理解和调参预演，不能替代仓库外实测验证。

## 9. 调试方法

第一步，观察 D 项滤波。

- 在 `updatePID()` 中观察 `dTerm`、`dTermFiltered` 和 `dAverage`。
- 轻微扰动云台时，比较原始 `dTerm` 和滤波后的 `dAverage` 是否有明显平滑效果。
- 若没有仓库外实测条件、安全夹具和可回退参数，只做源码路径与变量观察计划；实际扰动响应结论保持【待验证】。

第二步，观察 Roll 目标斜坡。

- 在 Roll 分支中观察 `pointingCmd[ROLL]`、`rollTargetSlew` 和 `safeDt`。
- 确认 `rollTargetSlew` 每帧按 `ROLL_SLEW_RAD_S` 或 `ROLL_TARGET_SLEW_RAD_S` 逐步接近目标。

第三步，确认 Pitch/Yaw 目标斜坡边界。

- 搜索 `PITCH_TARGET_SLEW_RAD_S` 和 `YAW_TARGET_SLEW_RAD_S`。
- 如果只发现定义，没有发现调用，就不要把它们写成当前运行逻辑。

第四步，观察输出限幅。

- 观察 `rollPidRaw` 与 `pidCmd[ROLL]` 的差异。
- 观察 `pidCmd[PITCH]` 是否被限制到 `±PITCH_CMD_LIMIT_RAD`。
- 观察 `pidCmd[YAW]` 是否被限制到 `±YAW_CMD_LIMIT_RAD`。

第五步，观察速率限制。

- 观察 `pidCmdPrev[]`、`pidCmd[]` 和 `outputRate[]`。
- Roll 重点观察 `rollDiag.stepLimit`。
- Pitch/Yaw 重点观察 `eepromConfig.rateLimit` 是否作为每帧差值限制生效。

第六步，观察积分暂停。

- 观察 `holdIntegrators` 和 `rollHoldIntegrators`。
- 观察 `iTerm` 是否在 `iHold == true` 时保持不变。
- 注意 Roll 分支修改全局 `holdIntegrators` 后可能影响后续 Pitch/Yaw 的积分更新。

调试记录建议：

- 记录 D 项原始值、滤波值、三点平均值和扰动前后变化，但不要越界断言实物响应。
- 对目标斜坡、限幅和速率限制，应分别记录输入、限制表达式、限制前值和限制后值。
- 积分暂停记录应包含全局门控、Roll 专用门控、`iHold` 入参和 `iTerm` 是否变化。
- 速率限制单位、扰动响应、震荡改善和稳定性结论缺少实测时保持【待验证】。

## 10. 常见问题

问题一：D 项滤波是不是改变 PID 的目标。

不是。D 项滤波只改变微分项的变化速度估计，不改变目标角本身。目标角处理属于目标斜坡和角度归一化。

问题二：三轴是否都使用目标角斜坡。

当前证据不是。Roll 路径有 `moveTowardsAnglef()` 调用证据，Pitch/Yaw 目标斜坡常量当前只发现定义，未发现运行路径调用。

问题三：`rateLimit` 是否一定表示“每秒限制”。

`config.c` 注释把它写作 rate limiting，但当前 Roll 使用 `rateLimit * safeDt`，
Pitch/Yaw 直接使用 `rateLimit`。
因此 Pitch/Yaw 的单位边界需要继续验证，教材暂不把它解释成完全一致的每秒限制。

问题四：`windupGuard` 是否控制当前积分限幅。

当前不是。`windupGuard` 被写入，但 `updatePID()` 实际使用硬编码 `[-10, 10]` 限制 `iTerm`。

问题五：仿真脚本能不能证明仓库外实测稳定。

不能。仿真脚本可以帮助理解趋势和参数影响，但它使用简化对象模型。真实稳定性仍要看仓库外实测中的传感器、机械结构、电机驱动和实际负载。

问题六：为什么输出限幅之后还要速率限制。

限幅只限制“输出最大有多大”，速率限制限制“输出变化有多快”。一个防止幅度过大，一个防止突变过快。

问题七：限幅、速率限制和异常值回退是否等于硬件保护。

不能这样表述。
它们是软件数值约束，用来减少过大指令、突变指令或异常浮点值继续传播。
是否足以保护真实电机、驱动器和机械结构，需要仓库外实测证据。
教材应把它们称为控制链路约束，而不是硬件层面的保证。

## 11. 实践任务

开始任务前，先回到本章第8节定位源码证据：D 项滤波看 8.1 和 8.2，目标斜坡看 8.3，输出约束看 8.4 和 8.5，积分暂停看 8.6。第9节提供调试观察顺序。

任务一：画出 D 项滤波链。

从 `dTerm` 画到 `dTermFiltered`，再画到 `dAverage` 和最终 PID 输出。
验收依据是 D 项链路表包含 `F_CUT=20Hz`、`rc`、三点平均和源码位置。

任务二：验证 Roll 目标斜坡。

观察 `rollTargetSlew` 是否按每帧最大步长接近 `pointingCmd[ROLL]`。
验收依据是 Roll 斜坡表分列回中阶段表达式、后续 PID 阶段表达式和触发条件。

任务三：验证 Pitch/Yaw 斜坡边界。

搜索 `PITCH_TARGET_SLEW_RAD_S` 和 `YAW_TARGET_SLEW_RAD_S`，说明为什么当前不能写成运行逻辑。
验收依据是宏边界表包含宏定义、调用搜索结果、运行证据结论和待接入边界。

任务四：列出三轴输出限幅。

分别记录 `ROLL_CMD_LIMIT_RAD`、`PITCH_CMD_LIMIT_RAD`、`YAW_CMD_LIMIT_RAD`，并说明它们限制的是哪个变量。
验收依据是限幅表包含常量名、目标轴、限制变量和代码位置。

任务五：比较三轴速率限制。

比较 Roll 的 `rateLimit * safeDt` 与 Pitch/Yaw 的 `rateLimit` 直接限制，记录该差异为什么需要【待验证】。
验收依据是速率限制表包含 Roll、Pitch、Yaw 三列表达式、源码结论和单位设计【待验证】项。

任务六：验证积分暂停。

观察 `rollHoldIntegrators` 为真时，Roll 的 `iTerm` 是否停止更新；再观察全局 `holdIntegrators` 是否影响同一帧后续轴。
验收依据是积分暂停表包含局部参数、全局变量、影响轴和观察结论。

## 12. 思考题

1. 为什么 D 项比 P 项更容易受噪声影响。
2. 一阶低通中的 `deltaT / (rc + deltaT)` 变大或变小时，滤波响应会怎样变化。
3. 为什么目标角斜坡需要角度归一化，而普通数值斜坡不一定需要。
4. 输出限幅和速率限制分别防止什么风险。
5. 为什么积分项在大误差或输出受限时可能带来过冲。
6. 如果 `windupGuard` 字段存在但未被使用，调参时应该如何避免误判。
7. 为什么仿真脚本只能作为调参辅助，不能替代仓库外实测验证。
8. 为什么“输出被限住了”不能直接等同于“系统已经安全”。
9. 第28章的限幅结论如何影响第34章调参顺序，为什么不能跳过证据记录直接改大 PID 参数？

## 13. 本章总结

本章把第27章的 PID 主干继续扩展为“可运行、可保护、可调试”的控制细节。

当前项目中：

- `pid.c` 使用 `F_CUT=20Hz` 和一阶低通形式平滑 D 项。
- `dAverage` 由当前滤波 D 项和两帧历史值平均得到。
- `dTerm` 被限制到 `[-300, 300]`。
- `updatePID()` 在 `iHold == false` 时更新积分项，并把 `iTerm` 限制到 `[-10, 10]`。
- `windupGuard` 字段当前不是 `updatePID()` 的积分限幅来源。
- Roll 路径有 `moveTowardsAnglef()` 目标斜坡运行证据。
- Pitch/Yaw 目标斜坡常量当前只见定义，未发现参与控制路径调用证据。
- 三轴都有输出幅值限制，但 Roll、Pitch、Yaw 的速率限制实现细节不同。
- `tools/pid_tuning_sim.py` 与固件共享 D 项滤波和限幅思想，但只作为调参辅助模型。

本章保留三个边界：

- Pitch/Yaw 的 `rateLimit` 当前未乘以 `dt`，与 Roll 分支不一致，需在调参或后续代码整理中继续验证。
- Pitch 轴“额外更严格限幅”代码当前上下限仍与全局 `[-10, 10]` 相同。
- 目标角斜坡机制当前不能写成三轴都已接入。

下一章将进入 `机械角、电角与正弦输出`，把 PID 输出继续连接到电机定子电角和三相 PWM 输出路径。

### 章节尾部固定检查

知识链路：

`知识点总表`
-> `知识依赖图`
-> `学习优先级`
-> `教学顺序`
-> `教材章节`

项目证据：

- `Drivers/SRC/Src/pid.c`
- `tools/pid_tuning_sim.py`
- `Drivers/SRC/Src/computeMotorCommands.c`
- `Drivers/SRC/Src/config.c`

质量自检：

- P0 事实错误：通过
- P1 依赖断层：通过
- P2 逻辑连贯：通过
- P3 项目证据：通过
- P4 原理展开：通过
- P5 调试实践：通过
- P6 表达统一：通过
