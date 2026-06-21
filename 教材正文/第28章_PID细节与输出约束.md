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

在当前默认 500Hz 附近运行时，可以估算 D 项低通系数。

```text
rc = 1 / (2π * 20) ≈ 0.00796s
alpha = dt / (rc + dt)
dt = 0.002s 时，alpha ≈ 0.002 / 0.00996 ≈ 0.201
```

这表示每一帧滤波值大约向新的原始 D 项移动 20%。它能削弱尖峰，但也引入响应滞后。
三点平均会继续降低尖峰，同时再增加一点相位滞后。D 项滤波不是“免费稳定”，
而是在抗噪声和响应速度之间取折中。

还可以用一个极端样例看清这层保护的真实强度。假设历史 D 项全为 0，
当前原始 `dTerm` 因目标阶跃或噪声被夹到上限 `300`，且 `dt=0.002s`。
第一帧低通输出约为：

```text
dTermFiltered[1] = 0 + 0.201 * (300 - 0) ≈ 60.3
dAverage[1] = (60.3 + 0 + 0) / 3 ≈ 20.1
```

如果该轴 `D=0.008`，第一帧 D 项输出贡献约为：

```text
D * dAverage[1] ≈ 0.008 * 20.1 ≈ 0.161
```

这比直接使用 `0.008 * 300 = 2.4` 小得多。第二帧如果原始 `dTerm` 仍为 300，
滤波值会继续向 300 靠近，三点平均也继续上升。因此，`[-300, 300]` 是原始 D 项限幅，
不是每一帧 D 输出贡献的直接上限。D 项会被低通和三点平均逐步释放，
这正是“抗尖峰”和“响应滞后”同时存在的原因。

### 4.3 目标角斜坡逼近

目标角斜坡逼近是指目标值不一次跳到最终目标，而是每帧最多移动一个小步长。

`computeMotorCommands.c` 中有两个工具函数：

```c
moveTowardsf()
moveTowardsAnglef()
```

`moveTowardsAnglef()` 会先用 `wrapToPif()` 计算角度最短路径，再限制每次变化量。

当前实际调用 `moveTowardsAnglef()` 的路径集中在 Roll 轴。`PITCH_TARGET_SLEW_RAD_S` 和 `YAW_TARGET_SLEW_RAD_S` 当前只发现定义，没有发现参与当前 Pitch/Yaw 控制路径的调用证据。

同一文件还定义了 `pitchTargetSlew`、`yawTargetSlew`、`yawCtrlAngle`、
`pitchAxisWasEnabled`、`yawAxisWasEnabled`、`pitchGateOpen` 和 `rollSettledTime`
等状态变量，以及 `YAW_CTRL_LPF_TAU_S`、`YAW_ERR_DEADBAND_RAD`、
`ROLL_SETTLE_ERR_RAD` 等宏。当前搜索结果显示，这些名字主要停留在定义处，
没有看到接入当前 Pitch/Yaw 控制路径或 Roll 收敛门控的运行代码。
因此它们应被写成“预留状态/未接入边界”，不能写成已经生效的目标斜坡或门控机制。

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

这里的 `outputRate[]` 名字容易造成误解。它记录的是“限速判断前的本帧差值”，
而不一定是最终实际应用的帧间变化量。因为代码先计算 `outputRate[]`，
再根据阈值修改 `pidCmd[]`，之后没有重新计算 `outputRate[]`。
所以调试最终变化量时，应比较当前受限后的 `pidCmd[]` 与上一帧受限输出，
不能只看 `outputRate[]`。

Roll 分支把 `eepromConfig.rateLimit` 乘以 `safeDt` 得到步长限制，并设置最小步长。

Pitch/Yaw 分支当前直接用 `eepromConfig.rateLimit` 作为每帧差值限制。

由于 `config.c` 注释称 `rateLimit = 45.0f * D2R` 是速率限制，Pitch/Yaw 当前是否也应乘以 `dt` 需要在后续调参或代码整理中继续验证，本章记录为【待验证】。

还要注意 `rateLimit` 限制的变量层级。`config.c` 的原注释写明它限制的是
electrical rotation，而不是 mechanical。当前代码也不是直接限制
`pointingCmd[]`、`rollTargetSlew` 或机械姿态角速度，而是在 PID 输出幅值限幅之后限制
`pidCmd[] - pidCmdPrev[]`。因此它更接近“PID 补偿量进入定子电角合成前的变化限制”。
若把它误写成“机械目标角速度限制”，就会把目标斜坡、PID 输出限速和机械角到电角映射三层概念混在一起。

### 4.6 目标斜坡与输出速率限制的区别

目标斜坡和输出速率限制都能让数值变化变慢，但它们不是同一个控制层。

目标斜坡限制的是进入 PID 前的目标值：

```text
pointingCmd -> target_slew -> updatePID()
```

输出速率限制限制的是 PID 之后的执行指令：

```text
updatePID() -> pidCmd限幅 -> pidCmd相对上一帧限速 -> 电角合成
```

Roll 后续 PID 分支当前同时具备这两层：`rollTargetSlew` 先限制目标角变化，
`pidCmd[ROLL]` 再限制输出变化。Pitch/Yaw 当前有输出速率限制，但没有发现目标斜坡进入
当前控制路径的运行证据。

这个区别会影响 D 项。当前默认 `dErrorCalc = D_ERROR`，D 项观察的是误差变化。
如果目标角突变，误差也会突变；目标斜坡能从源头减小每帧误差跳变，
输出速率限制只能在 PID 输出之后再限制执行指令，不能阻止原始 D 项先看到目标阶跃。

### 4.7 目标突变与D项冲击

在 `D_ERROR` 模式下：

```text
error[k] = command[k] - state[k]
D_raw = (error[k] - error[k-1]) / dt
```

展开后可以写成：

```text
D_raw = ((command[k] - command[k-1]) - (state[k] - state[k-1])) / dt
```

如果某一帧目标角 `command` 突然跳变，而实际姿态 `state` 还没有来得及变化，
那么 `command[k] - command[k-1]` 会直接进入原始 D 项。这类现象通常称为目标阶跃引起的
微分冲击，或 derivative kick。

当前 Roll 的目标斜坡可以限制 `command[k] - command[k-1]` 的大小。Pitch/Yaw 当前没有发现
同等目标斜坡调用证据，因此不能假设三轴对目标阶跃有相同的 D 项前级保护。
后续 D 项限幅、低通、三点平均、输出限幅和输出速率限制仍然有保护作用，
但它们属于阶跃进入 D 项之后的处理。是否造成真实电机抽动或稳定性变化，需要仓库外实测，
本章保持【待验证】。

### 4.8 积分抗饱和

积分抗饱和用于防止积分项在不合适的时候继续累积。当前项目有两层相关逻辑：

- `updatePID()` 在 `iHold == false` 时才更新 `iTerm`。
- `updatePID()` 把 `iTerm` 限制到 `[-10, 10]`。

`windupGuard` 字段虽然存在并被赋值，但当前 `updatePID()` 没有使用它作为积分限幅来源。

更准确地说，当前实现属于“积分暂停 + 积分状态硬限幅”，不是完整的反算 anti-windup。

完整反算方案通常会在输出饱和后，把“饱和前输出”和“饱和后输出”的差值反馈到积分状态，
减少积分继续推向饱和区。当前源码没有看到这种回灌路径。

因此，如果某轴输出已经被限幅，而传入 `updatePID()` 的 `iHold` 仍为 `false`，
`iTerm` 仍可能继续累积到 `[-10, 10]` 边界。是否造成实物过冲，需要结合该轴 I 参数、
输出限幅、机械负载和实测响应验证。

### 4.9 约束顺序

第28章还要关注保护层的执行顺序。当前主线可以概括为：

```text
deltaT 异常回退
-> error 计算与角度归一化
-> iHold 决定是否更新 iTerm
-> D 原始差分
-> D 原始值限幅
-> D 低通与三点平均
-> P/I/D 合成 output
-> output NaN/Inf 回退
-> 轴级输出幅值限幅
-> 轴级输出变化率限制
-> 电角合成与 PWM 输出
```

顺序不同，控制效果也不同。例如，如果先做输出限幅再做 D 项滤波，就无法抑制 D 项本身的尖峰。
如果只做幅值限幅而没有变化率限制，输出仍可能在两个允许幅值之间快速跳变。

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
它的好处是实现直接，也能同时反映目标和状态之间的误差变化；代价是目标阶跃本身也会进入
D 项。若希望 D 项只对测量状态变化敏感，常见替代做法是对测量值求微分，或使用设定值权重。
当前项目虽然在 `updatePID()` 中有 `D_STATE` 分支，但默认配置不是这条路径。
因此本章后续所有关于目标突变的分析，都以当前 `D_ERROR` 默认配置为前提。

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

这一步的顺序很重要。`dTerm` 先被限制到 `[-300, 300]`，再进入低通。
若当前调试参数中 `D=0.008`，单看 D 项原始上限，对输出的贡献上界约为：

```text
D * dTerm_max = 0.008 * 300 = 2.4
```

若 `D=0.016`，则约为：

```text
0.016 * 300 = 4.8
```

后续轴级输出限幅通常会进一步限制实际 `pidCmd[]`。所以 D 项限幅不是最终电机指令上限，
而是进入滤波和 PID 合成前的一层数值约束。

从滤波结构看，这里叠加了两类平滑：

```text
一阶 IIR 低通：使用 lastDterm 递推
三点 FIR 平均：使用 dTermFiltered、lastDterm、lastLastDterm
```

IIR 低通决定“每帧向新 D 项靠近多少”，三点平均决定“最近三帧滤波值如何分摊到输出”。
如果 `deltaT` 变大，`alpha = deltaT / (rc + deltaT)` 会变大，滤波响应变快；
如果 `deltaT` 变小，`alpha` 会变小，滤波响应变慢。
但同一个 `deltaT` 又出现在原始差分 `dInput / deltaT` 中。
因此时间步长异常会同时影响原始 D 项大小和滤波释放速度。
当前 `updatePID()` 会把异常 `deltaT` 回退为 `0.002f`，这只说明 PID 内部做了兜底，
不代表整条控制链的采样和调度都稳定。

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

同理，`pitchTargetSlew`、`yawTargetSlew`、`yawCtrlAngle`、
`pitchAxisWasEnabled` 和 `yawAxisWasEnabled` 当前也没有在 Pitch/Yaw 分支中形成
“首次使能时对齐当前角度、随后斜坡逼近目标”的完整链路。
这意味着 Pitch/Yaw 若在运行中重新使能，当前教材只能根据源码确认它们会使用
`pidCmdPrev[]` 的既有状态和当前 `pointingCmd[]` 进入输出约束；
是否存在重新使能瞬态，需要实际日志或断点记录【待验证】。

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

按 `config.c` 的 `rateLimit = 45.0f * D2R` 估算：

```text
rateLimit ≈ 45°/s * π/180 ≈ 0.7854 rad/s
```

这里的“45°”还不能直接写成云台机械角速度。`config.c` 注释明确说明它是
electrical rotation 的 rate limiting，而当前实际比较对象是 `pidCmd[axis] - pidCmdPrev[axis]`。
随后 `pidCmd[]` 才进入类似：

```text
stator_electrical_angle = current_electrical_angle + sign * pidCmd[axis]
```

这样的电角合成表达式。因此，调试 `rateLimit` 时应记录三层数值：

```text
机械角/目标角: pointingCmd、rollTargetSlew、pitch_angle、yaw_angle
PID补偿量: pidCmd、pidCmdPrev、outputRate
定子电角: stator_electrical_angle
```

只有把这三层分开，才能判断当前限制到底约束了目标变化、PID 补偿变化，还是最终电角变化。

### 5.5.1 PID补偿约束与定子电角边界

第28章讨论的“输出约束”主要发生在 `pidCmd[]` 这一层。它能限制 PID 补偿量的幅值和帧间变化，
但不能直接证明最终传入 `PWM_Motor_SetAngle()` 的定子电角也按同样步长变化。原因是三轴在
`computeMotorCommands.c` 中还会把当前电角、符号和固定偏移叠加进去：

| 轴向路径 | `pidCmd[]` 之后的定子电角表达式 | 本章边界 |
|---|---|---|
| Roll 后续 PID | `current_electrical_angle + ROLL_STATOR_SIGN * pidCmd[ROLL]` | `rateLimit` 只限制 PID 补偿项，不限制 `current_electrical_angle` 自身变化 |
| Pitch | `wrapToPif(current_elec + pitch_elec_offset + PITCH_STATOR_SIGN * pidCmd[PITCH])` | 固定偏移和角度包裹会参与最终电角，不能只看 `pidCmd[PITCH]` |
| Yaw | `wrapToPif(-current_elec + YAW_STATOR_SIGN * pidCmd[YAW])` | 当前电角前的负号属于轴向符号约定，真实方向仍需硬件验证【待验证】 |

因此，`pidCmd[]` 平滑只能说明“控制补偿项”被平滑；最终定子电角还受姿态角换算、符号、偏移和包裹影响。
如果调试时看到 `pidCmd[]` 变化平滑但电机响应仍突变，应继续进入第29章和第30章的电角合成、正弦查表和定时器通道映射，
不能仅凭第28章的限速逻辑判断硬件输出已经平滑【待验证】。

Roll 使用：

```text
rollStepLimit = rateLimit * safeDt
```

若 `safeDt=0.002s`，则：

```text
rollStepLimit ≈ 0.7854 * 0.002 ≈ 0.00157 rad/frame
```

同时 Roll 还有 `AXIS_MIN_STEP_LIMIT_RAD = 0.001` 的最小步长保护。

还要注意一个 Roll 诊断边界：`rollDiag.stepLimit` 记录的是
`eepromConfig.rateLimit * safeDt`，不一定等于最终实际使用的 `rollStepLimit`。
源码会在局部 `rollStepLimit` 小于 `AXIS_MIN_STEP_LIMIT_RAD` 时，把实际步长抬到
`0.001 rad/frame`，但 `rollDiag.stepLimit` 写入的仍是乘法结果。

例如 `safeDt=0.001s` 时：

```text
rateLimit * safeDt ≈ 0.7854 * 0.001 ≈ 0.000785 rad/frame
actual rollStepLimit = max(0.000785, 0.001) = 0.001 rad/frame
rollDiag.stepLimit ≈ 0.000785
```

若 `safeDt=0.002s`，乘法结果约为 `0.00157 rad/frame`，已经高于最小步长，
此时记录值和实际限制值才一致。调试时应重新计算
`max(eepromConfig.rateLimit * safeDt, AXIS_MIN_STEP_LIMIT_RAD)`，不能只看
`rollDiag.stepLimit` 就断言最终限速阈值。

Pitch/Yaw 当前直接使用 `eepromConfig.rateLimit` 作为每帧差值上限，
即约 `0.7854 rad/frame`。这个数量级远大于 Roll 的 `rateLimit * dt` 结果。
所以教材不能把三轴速率限制写成同一物理含义；Pitch/Yaw 当前更像“每帧最大变化量”，
Roll 更像“每秒速率换算到本帧步长”。设计意图和实物效果需要继续【待验证】。

还要注意，三轴的 `outputRate[]` 都是在速率限制前计算的。
如果本帧输出被速率限制改写，`outputRate[]` 仍保留限速前差值。
因此：

```text
outputRate[axis]          = 限速前差值
pidCmd[axis] - 上帧pidCmd = 限速后实际应用差值
```

调试“最终变化是否超过阈值”时，应以后者为准。

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

这里还有一个宏定义边界。源码定义了：

```text
ROLL_I_ENABLE_ERR_RAD
PITCH_I_ENABLE_ERR_RAD
YAW_I_ENABLE_ERR_RAD
```

当前运行证据显示，Roll 使用 `ROLL_I_ENABLE_ERR_RAD` 生成 `rollHoldIntegrators`。
Pitch/Yaw 虽然定义了各自的 `*_I_ENABLE_ERR_RAD`，但当前 Pitch/Yaw 分支传入
`updatePID()` 的是全局 `holdIntegrators`，没有看到基于本轴误差阈值的本地门控。
所以教材不能写成“三轴都按各自 I_ENABLE 阈值暂停积分”。

还要把当前调试参数纳入判断。`Core/Src/main.c` 当前覆盖了：

```text
ROLL:  I = 0.0
PITCH: I = 0.0
YAW:   I = 0.01
```

即使 `I=0`，`updatePID()` 仍可能更新 `iTerm`，只是 `I * iTerm` 对输出暂时没有贡献。
如果后续调参把 I 从 0 改为非零，已有 `iTerm` 状态可能突然开始影响输出。
这也是启动门控后清零积分状态有工程意义的原因。

## 6. STM32实现机制

本章内容仍然属于软件控制算法，不是外设配置。

STM32 相关性体现在三点：

第一，所有约束都运行在 500Hz 实时控制分支中。每帧的 `dt` 会影响 D 项计算、D 项滤波和 Roll 目标斜坡步长。

第二，输出约束最终会影响电机 PWM 指令。虽然本章不展开 `PWM_Motor_SetAngle()`，但 `pidCmd[]` 的限幅和变化率会决定后续电角输出的变化大小。

第三，调试时这些变量可以直接在 STM32 调试器中观察，例如 `dTermFiltered`、`pidCmd[]`、`pidCmdPrev[]`、`rollTargetSlew`、`holdIntegrators` 和 `eepromConfig.rateLimit`。

因此，本章是 PID 算法和电机输出之间的数值约束层。

从 Cortex-M3 执行成本看，本章保护逻辑主要消耗浮点乘除、`fabsf()`、`isnan()`、`isinf()`、
角度包裹循环和若干条件分支。单个操作不复杂，但它们位于 500Hz 分支内，并且与 AHRS、
I2C 读取和 PWM 输出同帧执行，所以仍应纳入第26章的 `executionTime500Hz` 预算观察。

从状态存储看，约束层依赖多个跨帧变量：

- `lastDcalcValue`
- `lastDterm`
- `lastLastDterm`
- `pidCmdPrev[]`
- `rollTargetSlew`
- `holdIntegrators`

这些状态让输出更平滑，但也让调试更像“时间序列问题”。如果只看当前帧输入，
无法解释历史 D 项、上一帧输出或目标斜坡对本帧输出的影响。

## 7. 项目中的应用

本章涉及的项目证据文件包括：

- `Drivers/SRC/Src/pid.c`
- `Drivers/SRC/Src/computeMotorCommands.c`
- `Drivers/SRC/Src/config.c`
- `Core/Src/main.c`
- `tools/pid_tuning_sim.py`

`pid.c` 提供 D 项滤波、D 项限幅、积分更新和积分限幅。

`computeMotorCommands.c` 提供目标角斜坡、输出限幅、速率限制、积分暂停和三轴分支差异。
其中 `rateLimit` 当前作用在 `pidCmd[]` 与 `pidCmdPrev[]` 的差值上，而不是直接作用在
机械目标角 `pointingCmd[]` 上。

`config.c` 设置 `eepromConfig.rateLimit = 45.0f * D2R`，源码注释说明该限制面向
electrical rotation，不是 mechanical。

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

### 8.3 D项历史初始化边界

`updatePID()` 中有一段特殊初始化：

```text
if lastDcalcValue == 0:
    lastDcalcValue = error 或 state
```

这可以降低第一次进入 D 项计算时的突变，但它也带来一个边界：
如果真实上一帧误差或状态恰好等于 0，代码无法区分“尚未初始化”和“历史值真实为 0”。

在当前项目中，启动门控后会调用 `zeroPIDstates()`，Roll 分支初次使能时也会清零 D 项历史。
因此调试初始若看到 D 项短暂偏小或重新建立历史，不应立刻解释为滤波失效。
应同时查看 `lastDcalcValue`、`lastDterm` 和 `lastLastDterm` 的清零时刻。

### 8.4 目标角斜坡代码

`moveTowardsAnglef()` 的核心思想是：

```text
delta = wrapToPif(target - current)
delta 限制到 [-maxStep, maxStep]
return wrapToPif(current + delta)
```

它比普通 `moveTowardsf()` 多了角度环绕处理，因此适合姿态角目标逼近。

当前运行证据显示它被 Roll 路径调用。`moveTowardsf()` 当前只看到定义，没有发现当前控制路径调用证据。

### 8.5 Roll输出约束

Roll 后续 PID 分支中：

```text
rollPidRaw -> clampf -> pidCmd[ROLL]
pidCmd[ROLL] - pidCmdPrev[ROLL] -> outputRate[ROLL]
rollStepLimit = eepromConfig.rateLimit * safeDt
```

如果变化超过 `rollStepLimit`，就把 `pidCmd[ROLL]` 限制到上一帧附近。`rollDiag` 还记录了
`pidRaw`、`pidClamped`、`pidApplied`、`dPidRaw` 和 `stepLimit`，适合调试 Roll 输出约束。

这些字段需要按约束阶段分开读：

- `pidRaw` 是 `updatePID()` 返回并经过 NaN/Inf 回退后的原始 PID 输出。
- `pidClamped` 是幅值限幅后的理论值，即限制到 `±ROLL_CMD_LIMIT_RAD` 后的结果。
- `pidApplied` 是速率限制后真正进入电角合成的 Roll 控制量。
- `dPidRaw` 是 `pidClamped - pidCmdPrev[ROLL]` 的差值，计算发生在速率限制之前。
- `stepLimit` 是 `rateLimit * safeDt` 的记录值，不一定包含 `AXIS_MIN_STEP_LIMIT_RAD` 抬升。

因此，当 `dPidRaw` 绝对值大于实际步长限制时，`pidApplied` 的变化量会被压到
`±actualStepLimit` 附近，但 `dPidRaw` 本身不会被重新计算成受限后的差值。
调试表应同时列出 `pidApplied - 上一帧pidApplied`，才可以判断最终实际变化量。

### 8.6 Pitch和Yaw输出约束

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

### 8.6.1 rateLimit字段的单位语义边界

`rateLimit` 是第28章里最容易被误读的字段之一。它的名字像“速率”，
`config.c` 注释也写着：

```text
rateLimit = 45.0f * D2R
Note this is rate limiting electrical degrees of rotation, not mechanical
```

这层证据只能说明配置者希望表达“电角度相关的变化限制”，不能直接证明三轴当前都按
“rad/s”解释。真正进入控制路径后，还要看 `computeMotorCommands.c` 中的比较对象和换算方式。

当前源码可以拆成三层：

| 层级 | 源码证据 | 可证明结论 | 不能证明的结论 |
| --- | --- | --- | --- |
| 配置字段 | `eepromConfig.rateLimit = 45.0f * D2R` | 默认数值约为 `0.7854`，注释指向 electrical rotation | 它一定是机械角速度上限 |
| 比较对象 | `outputRate[axis] = pidCmd[axis] - pidCmdPrev[axis]` | 限制对象是 PID 补偿量的帧间差值 | 限制对象是 `pointingCmd[]` 目标角速度 |
| 轴向换算 | Roll 使用 `rateLimit * safeDt`，Pitch/Yaw 直接使用 `rateLimit` | 三轴当前代码表达式不同 | 三轴具有同一物理单位和同一限速强度 |

用当前默认 500Hz 附近的 `safeDt = 0.002s` 做数量级检查：

```text
rateLimit = 45 deg/s * pi/180 ≈ 0.7854 rad/s
Roll:  rollStepLimit = 0.7854 * 0.002 ≈ 0.00157 rad/frame
Pitch: stepLimit = 0.7854 rad/frame
Yaw:   stepLimit = 0.7854 rad/frame
```

如果把 Pitch/Yaw 的 `0.7854 rad/frame` 再按 500Hz 换算成“等效每秒速率”，会得到：

```text
0.7854 rad/frame * 500 frame/s ≈ 392.7 rad/s
≈ 22500 deg/s
```

这个数量级说明：不能把 Pitch/Yaw 当前表达式直接解释成与 Roll 相同的
`45 deg/s` 每秒速率限制。更准确的教材表述应是：

```text
Roll: 当前源码有 rateLimit * safeDt 的每帧步长换算证据。
Pitch/Yaw: 当前源码有直接使用 rateLimit 作为每帧差值阈值的证据。
三轴单位语义是否符合设计意图，需要代码整理、调参日志或实物测试继续验证【待验证】。
```

这个边界还影响调试方法。若示波器或串口日志里看到 Pitch/Yaw 输出变化比 Roll 快，
不能只说“`rateLimit` 没生效”。应先确认：

```text
1. 本轴是否进入 updatePID() 后的幅值限幅路径。
2. outputRate[axis] 是否超过 eepromConfig.rateLimit。
3. pidCmd[axis] 是否被改写为 pidCmdPrev[axis] ± eepromConfig.rateLimit。
4. pidCmdPrev[axis] 更新前后的差值是否重新计算。
5. 最终 stator_electrical_angle 是否还叠加了 current_elec、offset、sign 或 wrapToPif()。
```

`.list` 构建产物能作为第二层证据：当前 Debug 快照中 Pitch 和 Yaw 分支在 `updatePID()`
之后仍能看到 `outputRate[PITCH/YAW]` 与 `eepromConfig.rateLimit` 的比较和加减；
Roll 分支则保留了 `eepromConfig.rateLimit * safeDt` 的步长计算。也就是说，
这不是单纯的源码阅读推测，而是当前 Debug 构建中仍然存在的轴向差异。

但 `.list` 仍然不能证明设计意图。它能证明“编译出来的代码保留了这种差异”，
不能证明“Pitch/Yaw 这样写就是正确单位”，也不能证明“真实电机响应安全”。因此，
本章把 `rateLimit` 写成一个需要按轴解释的约束字段，而不是三轴共享的统一物理速率参数。

### 8.7 积分限幅与windupGuard边界

`updatePID()` 中积分更新逻辑是：

```text
if iHold == false:
    iTerm += error * deltaT
    iTerm 限制到 [-10, 10]
```

代码中还写了 Pitch 轴额外限幅分支，但上下限仍然是 `[-10, 10]`，与全局限幅相同。

`PIDdata_t.windupGuard` 在配置中被写入，但当前没有被 `updatePID()` 读取。因此教材不能把 `windupGuard` 写成当前积分限幅实际来源。

当前代码也没有输出饱和反算。例如 `pidCmd[PITCH]` 被限制到 `±PITCH_CMD_LIMIT_RAD` 后，
没有把“限幅前后差值”反馈给 `eepromConfig.PID[PITCH_PID].iTerm`。

因此第28章应把当前积分保护限定为：

```text
iHold 暂停积分
iTerm 硬限幅到 [-10, 10]
启动/使能阶段清零积分状态
```

而不是写成完整 anti-windup 控制器。

### 8.8 I_ENABLE宏定义与实际接线边界

`computeMotorCommands.c` 中三轴都定义了积分使能误差阈值宏：

```text
ROLL_I_ENABLE_ERR_RAD
PITCH_I_ENABLE_ERR_RAD
YAW_I_ENABLE_ERR_RAD
```

但定义存在不等于运行接入。当前代码只有 Roll 分支使用：

```text
rollHoldIntegrators = fabsf(roll_error_electrical) > ROLL_I_ENABLE_ERR_RAD
updatePID(..., rollHoldIntegrators, ...)
```

Pitch/Yaw 分支没有使用 `PITCH_I_ENABLE_ERR_RAD` 或 `YAW_I_ENABLE_ERR_RAD`
计算本轴 `iHold`，而是把全局 `holdIntegrators` 传入 `updatePID()`。
这个全局变量在函数开头被清为 `false`，随后可能被 Roll 分支置为 `true`。

因此当前积分暂停接线应拆成三类：

- Roll：有本轴误差阈值门控。
- Pitch/Yaw：当前未发现本轴 `*_I_ENABLE_ERR_RAD` 门控接入。
- 跨轴影响：Roll 可能通过全局 `holdIntegrators` 影响后续 Pitch/Yaw。

调试或重构时，如果想让 Pitch/Yaw 使用各自误差阈值，需要显式增加本轴误差判断，
并确认这不会改变原有跨轴暂停语义。相关实物效果保持【待验证】。

### 8.9 约束顺序代码边界

从 `computeMotorCommands.c` 看，轴级输出顺序通常是：

```text
updatePID()
-> NaN/Inf 输出回退
-> 幅值限幅
-> 变化率限制
-> pidCmdPrev 更新
-> 电角合成
-> PWM_Motor_SetAngle()
```

这个顺序意味着 `pidCmdPrev[]` 保存的是约束后的输出，而不是 PID 原始输出。
调试变化率限制时，应比较 `pidCmdPrev[]` 与限幅后的 `pidCmd[]`，不要直接拿 `rollPidRaw`
和 `pidCmdPrev[]` 判断速率限制是否生效。

#### 输出变量生命周期证据边界

第28章最容易混淆的地方，是同一个轴在一帧内会出现多个“看起来都像输出”的变量。
按源码顺序，它们代表不同生命周期阶段：

| 阶段 | Roll 证据 | Pitch/Yaw 证据 | 能说明什么 | 不能说明什么 |
| --- | --- | --- | --- | --- |
| PID原始输出 | `rollPidRaw = updatePID(...)` | `pidCmd[axis] = updatePID(...)` 后、NaN/Inf 回退前的值 | PID 公式、积分状态和 D 项滤波给出的原始补偿量 | 已经满足幅值限制或变化率限制 |
| 幅值限幅后输出 | `pidCmd[ROLL] = clampf(rollPidRaw, ...)`，`rollDiag.pidClamped` | `pidCmd[axis]` 被限制到 `±*_CMD_LIMIT_RAD` 后的值 | 补偿量已经进入轴级最大幅值边界 | 本帧变化速度已经满足限制 |
| 速率判断前差值 | `outputRate[ROLL] = pidCmd[ROLL] - pidCmdPrev[ROLL]`，`rollDiag.dPidRaw` | `outputRate[PITCH/YAW] = pidCmd[axis] - pidCmdPrev[axis]` | 本帧限速前，约束后输出相对上一帧历史的差值 | 最终实际应用的帧间差值一定等于它 |
| 速率限制后输出 | `pidCmd[ROLL]` 被改写为 `pidCmdPrev[ROLL] ± rollStepLimit` 后，`rollDiag.pidApplied` | `pidCmd[PITCH/YAW]` 被改写为 `pidCmdPrev[axis] ± eepromConfig.rateLimit` 后 | 真正参与后续电角合成的 PID 补偿量 | 电角方向、PWM占空比和电机响应已经正确 |
| 下一帧历史 | `pidCmdPrev[ROLL] = pidCmd[ROLL]` | `pidCmdPrev[PITCH/YAW] = pidCmd[axis]` | 下一帧速率限制的历史基准已经更新 | 保存了原始 PID 输出或限速前差值 |

因此，`outputRate[]` 更适合作为“是否触发限速”的诊断入口，而不是最终执行变化量。
如果本帧发生速率限制，最终实际应用差值应重新用：

```text
actualDelta[axis] = pidCmd_after_rate_limit[axis] - pidCmdPrev_before_update[axis]
```

来判断。Roll 的 `rollDiag` 帮助把这些阶段拆开：`pidRaw` 是 PID 原始值，
`pidClamped` 是幅值限幅后值，`dPidRaw` 是限速前差值，`pidApplied` 是速率限制后真正应用的值。
Pitch/Yaw 当前没有同等细粒度诊断结构，调试时需要手动在限幅前、限幅后、限速后和
`pidCmdPrev[]` 更新前分别打点；否则容易把“限速前差值很大”误判为“最终电角变化同样很大”。

### 8.10 仿真脚本边界

`tools/pid_tuning_sim.py` 中 `RC_D_FILTER = 1/(2π*20)`，与固件 `F_CUT=20Hz` 对应。脚本的 `update_pid()` 也包含：

```text
i_term 限制到 [-10, 10]
d_term 限制到 [-300, 300]
d_term_filtered 一阶低通
cmd_limit 限幅
rate_limit 限制输出变化
```

但脚本和固件并不是逐行等价。

第一，固件在 `D_ERROR` 路径中，如果 `lastDcalcValue == 0.0f`，会先用当前 `error`
初始化历史值；脚本当前在 `pid.last_d_calc == 0.0` 时统一用 `state` 初始化。
因此阶跃刚出现时，脚本 D 项可能出现与固件不同的初始差分。

可以用一个数字样例看清差异。假设 `command=0`、`state=0.2rad`、
`dt=0.002s`，且 D 项历史刚清零，则当前误差为：

```text
error = command - state = -0.2rad
```

固件在 `D_ERROR` 路径中会先执行：

```text
lastDcalcValue = error = -0.2
dInput = error - lastDcalcValue = 0
dTerm = 0 / dt = 0
```

脚本当前会先执行：

```text
last_d_calc = state = 0.2
d_term = (error - last_d_calc) / dt
       = (-0.2 - 0.2) / 0.002
       = -200
```

因此，同样一组初始条件下，固件第一帧 D 项可能被初始化逻辑压到 0，
而脚本可能给出明显的负向 D 项。若继续代入 `F_CUT=20Hz` 的低通，第一帧
`dTermFiltered` 约为 `-40.2`，`dAverage` 约为 `-13.4`；
当 `D=0.008` 时，脚本第一帧 D 输出贡献约为 `-0.107`。
这个例子说明：脚本曲线可用于提示趋势和风险，但不能逐帧替代固件 D 项真值。

第二，固件会对 `error - lastDcalcValue` 再做一次角度包裹；脚本先包裹 `error`，
但没有对这次差分再次单独包裹。跨过 `±pi` 边界时，仿真 D 项可能与固件不一致。

例如上一帧误差接近 `+3.13rad`，当前误差接近 `-3.13rad`。固件会先得到：

```text
dInput = -3.13 - 3.13 = -6.26
standardRadianFormat(dInput) ≈ 0.023
dTerm ≈ 0.023 / 0.002 ≈ 11.5
```

脚本若直接使用未再次包裹的误差差分，则会得到：

```text
d_term = (-3.13 - 3.13) / 0.002 ≈ -3130
clamp -> -300
```

这会把一个跨 `±pi` 的小角度连续变化误判成接近 D 项下限的尖峰。
所以凡是用脚本讨论边界附近的 D 项，都必须回到固件 `updatePID()` 的
`standardRadianFormat(error - lastDcalcValue)` 顺序复核。

第三，脚本用统一 `rate_limit` 约束三轴每步输出变化；固件 Roll 使用
`rateLimit * safeDt`，Pitch/Yaw 直接使用 `rateLimit`。所以脚本只能说明限速趋势，
不能证明三轴固件限速强度完全相同。

但脚本使用的是简化二阶对象模型，不等于真实电机、真实 IMU 和真实 PWM 输出。它适合做趋势理解和调参预演，不能替代仓库外实测验证。

### 8.11 预留斜坡与门控变量边界

`computeMotorCommands.c` 中存在一组看起来与目标斜坡、Yaw 平滑和 Roll 收敛门控相关的变量：

```text
pitchTargetSlew
yawTargetSlew
yawCtrlAngle
pitchAxisWasEnabled
yawAxisWasEnabled
pitchGateOpen
rollSettledTime
```

也存在一组看起来与这些机制相关的宏：

```text
PITCH_TARGET_SLEW_RAD_S
YAW_TARGET_SLEW_RAD_S
YAW_CTRL_LPF_TAU_S
YAW_ERR_DEADBAND_RAD
ROLL_SETTLE_ERR_RAD
ROLL_SETTLE_CMD_RAD
ROLL_SETTLE_TIME_S
```

但当前搜索结果只证明这些名字被定义，不能证明它们已经接入当前运行路径。
本章只能把它们作为“预留设计痕迹”或“未接入边界”记录。

这个边界影响读者对控制保护层的理解。若只看宏名，容易误以为 Pitch/Yaw 已经有目标斜坡、
Yaw 已经有控制低通、Roll 已经有收敛后再放开 Pitch 的门控。按当前源码证据，
这些机制不能写成已经生效。若未来补齐这些逻辑，应同时更新第28章和第31章的控制架构说明。

#### PID保护机制的权威资料与项目实现边界

PID 细节不能只按“工程经验”讲。经典 PID 文献和控制教材通常会把三类问题分开讨论：微分项对噪声敏感、执行器饱和会引发积分 windup、设定值突变可能带来 derivative kick 或输出冲击。这些资料能证明第28章讨论的 D 项滤波、输出限幅、速率限制和抗饱和都属于 PID 工程实现中的常见保护主题。

但权威资料层不能替代项目源码层。资料中常见的 anti-windup 包括条件积分、积分暂停、积分限幅、tracking/back-calculation 等不同形式；当前项目源码只能证明 `updatePID()` 实现了 `iHold` 条件积分、`iTerm` 硬限幅到 `[-10, 10]`，并没有看到把输出饱和误差反馈回积分状态的反算路径。因此本章只能把当前实现写成“积分暂停 + 积分硬限幅”，不能因为资料中存在完整 anti-windup 概念，就把项目写成完整反算 anti-windup 控制器。

同理，资料层可以说明 D 项滤波和设定值权重是降低噪声敏感性与设定值冲击的常见方法；项目源码层只能证明当前 `updatePID()` 有 `dErrorCalc` 分支、一阶低通、三点平均、D 项原始限幅，以及 ANGULAR/非 ANGULAR 两类输出表达式。当前默认配置、三轴接线和实际响应仍必须回到 `config.c`、`pid.c`、`computeMotorCommands.c`、`.map/.list` 与调试日志验证；缺少现场数据时，稳定性和硬件安全结论保持【待验证】。

### 8.12 构建产物证据边界

前面的小节主要来自源码阅读。当前 Debug 构建还可以用
`Debug/Three-axis_cloud_platformV2.map` 和 `Debug/Three-axis_cloud_platformV2.list`
补充一层“确实进入最终构建”的证据。

`.map` 中能看到以下符号和对象：

| 构建产物证据 | 地址或段 | 能证明的范围 |
|---|---:|---|
| `PWM_Motor_SetAngle` | `0x08002748` | PWM 设角函数进入最终镜像 |
| `wrapToPif` | `0x080042f4` | `computeMotorCommands.o` 中的角度包裹辅助函数进入最终镜像 |
| `computeMotorCommands` | `0x080043b4` | 三轴电机指令计算主路径进入最终镜像 |
| `standardRadianFormat` | `0x08005250` | `pid.o` 中的角度归一化函数进入最终镜像 |
| `updatePID` | `0x08005334` | PID 更新函数进入最终镜像 |
| `zeroPIDintegralError` | `0x0800578c` | 积分清零辅助函数进入最终镜像 |
| `zeroPIDstates` | `0x08005814` | PID 状态清零辅助函数进入最终镜像 |
| `.data.holdIntegrators` | `0x2000002c` | 全局积分暂停标志被分配到 RAM 初始化数据段 |
| `.bss.eepromConfig` | `0x200003e0` | PID 参数所在配置对象被分配到 RAM |
| `.bss.outputRate` | `0x20000d8c` | 三轴限速判断前差值数组被分配到 RAM |
| `.bss.pidCmd` | `0x20000d98` | 三轴 PID 输出数组被分配到 RAM |
| `.bss.pidCmdPrev` | `0x20000da4` | 三轴约束后输出历史数组被分配到 RAM |
| `.bss.rollDiag` | `0x20000db0` | Roll 诊断结构体被分配到 RAM |

这些地址和段名只能证明符号、函数和对象被链接进当前 Debug 镜像，不能证明它们在真实硬件上一定按预期运行。

`.list` 能进一步把源码语句和反汇编位置对应起来：

- `main()` 中存在 `computeMotorCommands(dt500Hz)` 调用，反汇编调用目标为 `0x080043b4 <computeMotorCommands>`。
- Roll 分支在 `0x0800458e` 调用 `updatePID`，随后可见幅值限幅、`outputRate[ROLL]`、`pidCmdPrev[ROLL]`、`rollStepLimit` 比较、`PWM_Motor_SetAngle` 调用和 `rollDiag` 字段写入。
- Pitch 分支在 `0x08004854` 调用 `updatePID`，随后可见幅值限幅、`outputRate[PITCH]`、直接使用 `eepromConfig.rateLimit` 的差值限制、`pidCmdPrev[PITCH]` 更新和 `PWM_Motor_SetAngle` 调用。
- Yaw 分支在 `0x08004a82` 调用 `updatePID`，随后可见幅值限幅、`outputRate[YAW]`、直接使用 `eepromConfig.rateLimit` 的差值限制、`pidCmdPrev[YAW]` 更新和 `PWM_Motor_SetAngle` 调用。
- `updatePID()` 反汇编段能对应到 `dTermFiltered`、`dAverage`、`lastDterm`、`lastLastDterm` 更新，以及最终 P/I/D 合成表达式。

因此，构建产物可以证明“当前 Debug 固件包含 PID 细节和输出约束路径”，也可以证明 Roll、Pitch、Yaw 的限速表达式在编译后仍然保留了源码中的差异。

`.su/.cyclo` 还能补充输出保护链路的静态审查边界。当前 Debug 构建中：

| 函数 | 静态栈用量 | 圈复杂度 | 与本章的关系 |
| --- | --- | --- | --- |
| `clampf()` | 24 字节 | 3 | Roll 幅值限幅 helper |
| `moveTowardsf()` | 32 字节 | 3 | 浮点目标逼近 helper，当前构建中有静态报告 |
| `wrapToPif()` | 16 字节 | 3 | 电角、机械角和误差包裹 helper |
| `moveTowardsAnglef()` | 32 字节 | 3 | Roll 目标角斜坡 helper |
| `updatePID()` | 64 字节 | 34 | D 项滤波、积分更新和 P/I/D 合成 |
| `computeMotorCommands()` | 120 字节 | 48 | 三轴限幅、速率限制、`pidCmdPrev[]` 更新和电机设角主路径 |
| `PWM_Motor_SetAngle()` | 80 字节 | 13 | PID 输出进入三相 PWM 之前的电机设角函数 |

这些数值能提示输出保护路径中 `computeMotorCommands()` 和 `updatePID()` 的分支复杂度较高，
也能说明 `PWM_Motor_SetAngle()` 已有独立静态栈报告。但它们不能证明限幅后电机一定安全，
也不能证明一次 500Hz 帧的真实耗时或栈峰值。尤其是 `moveTowardsf()` 只有静态报告，
当前第28章仍要以 `.list` 调用点判断它是否进入运行路径；不能把“有 `.su/.cyclo`”
写成“当前目标斜坡已调用”。

#### 软件约束到硬件输出的证据边界

第28章讨论的是 PID 输出进入电机设角之前的数值约束。它会影响后续 PWM，但不能直接替代第29章和第30章的电角、三相 PWM 与硬件诊断证据。可以按下面四层拆开：

| 层级 | 当前构建证据 | 能证明什么 | 不能证明什么 |
|---|---|---|---|
| PID数值约束 | `.list` 中三轴 `updatePID()` 后可见幅值限幅、`outputRate[]`、`pidCmdPrev[]` 更新 | 当前 Debug 固件保留了 PID 输出限幅和变化率限制路径 | 限幅参数一定足以保护真实电机或机械结构 |
| 电角设定入口 | `.list` 中三轴分支都调用 `PWM_Motor_SetAngle()`，`.map` 中该函数有最终地址 | 受限后的 `pidCmd[]` 会参与后续定子电角合成并进入电机设角函数 | 电角方向、机械响应和负载状态已经正确 |
| PWM比较值写入 | `PWM_Motor_SetAngle()` 中计算三相 `ccr_val[]`，并按轴写入 TIM2/TIM3/TIM4 的 `CCR` 寄存器 | 软件路径最终会更新定时器比较寄存器，形成三相 PWM 占空比入口 | 实际相电流、驱动器温升、电机力矩和机械安全 |
| 现场验证 | 断点、`rollDiag`、`pidCmd[]`、`CCR` 读数、示波器波形、电机夹具和温升记录【待验证】 | 可以把数值约束推进到真实输出和安全边界验证 | 当前仓库没有这些现场记录，不能宣称闭环稳定或硬件安全已证明 |

所以本章可以说“软件链路限制了传给 `PWM_Motor_SetAngle()` 的补偿量和变化量”，但不能说“电机一定不会抽动、不会过热、不会失步”。软件限幅是安全设计的一层，不是硬件安全证明本身。

但它不能证明以下结论：

- 不能证明限幅和速率限制足以保护真实电机、驱动器和机械结构。
- 不能证明闭环稳定、震荡改善或扰动恢复效果。
- 不能证明 Pitch/Yaw 直接使用 `rateLimit` 的物理单位就是每秒机械角速度。
- 不能证明 `holdIntegrators` 在某次真实运动中一定被触发，触发时序仍需要日志或断点。
- 不能证明 `tools/pid_tuning_sim.py` 与固件逐帧等价。

所以，本章可以把 `.map/.list` 作为构建证据引用，但所有运行时、硬件安全和闭环调参结论仍保持【待验证】。

## 9. 调试方法

第一步，观察 D 项滤波。

- 在 `updatePID()` 中观察 `dTerm`、`dTermFiltered` 和 `dAverage`。
- 用 `dTerm=300`、历史 D 项为 0 的样例，手算第一帧 `dTermFiltered` 和 `dAverage`。
- 轻微扰动云台时，比较原始 `dTerm` 和滤波后的 `dAverage` 是否有明显平滑效果。
- 若没有仓库外实测条件、安全夹具和可回退参数，只做源码路径与变量观察计划；实际扰动响应结论保持【待验证】。

第二步，观察 Roll 目标斜坡。

- 在 Roll 分支中观察 `pointingCmd[ROLL]`、`rollTargetSlew` 和 `safeDt`。
- 确认 `rollTargetSlew` 每帧按 `ROLL_SLEW_RAD_S` 或 `ROLL_TARGET_SLEW_RAD_S` 逐步接近目标。

第三步，确认 Pitch/Yaw 目标斜坡边界。

- 搜索 `PITCH_TARGET_SLEW_RAD_S` 和 `YAW_TARGET_SLEW_RAD_S`。
- 如果只发现定义，没有发现调用，就不要把它们写成当前运行逻辑。
- 同时搜索 `pitchTargetSlew`、`yawTargetSlew`、`yawCtrlAngle` 和 `pitchAxisWasEnabled`。
- 若这些状态变量没有接入 Pitch/Yaw 分支，就不要把它们写成再使能保护或目标平滑证据。

第四步，观察输出限幅。

- 观察 `rollPidRaw` 与 `pidCmd[ROLL]` 的差异。
- 观察 `pidCmd[PITCH]` 是否被限制到 `±PITCH_CMD_LIMIT_RAD`。
- 观察 `pidCmd[YAW]` 是否被限制到 `±YAW_CMD_LIMIT_RAD`。

第五步，观察速率限制。

- 观察 `pidCmdPrev[]`、`pidCmd[]` 和 `outputRate[]`。
- Roll 重点观察 `rollDiag.stepLimit`，并重新计算
  `max(eepromConfig.rateLimit * safeDt, AXIS_MIN_STEP_LIMIT_RAD)`。
- 对 Roll 同时记录 `dPidRaw` 与 `pidApplied` 的帧间差值，区分速率限制前后的变化量。
- Pitch/Yaw 重点观察 `eepromConfig.rateLimit` 是否作为每帧差值限制生效。
- 注意 `pidCmdPrev[]` 保存的是上一帧约束后的输出，不是上一帧原始 PID 输出。
- 注意 `outputRate[]` 是限速判断前差值；最终应用差值应由当前 `pidCmd[]` 减上一帧受限输出得到。

第六步，观察积分暂停。

- 观察 `holdIntegrators` 和 `rollHoldIntegrators`。
- 观察 `iTerm` 是否在 `iHold == true` 时保持不变。
- 注意 Roll 分支修改全局 `holdIntegrators` 后可能影响后续 Pitch/Yaw 的积分更新。
- 单独搜索 `PITCH_I_ENABLE_ERR_RAD` 和 `YAW_I_ENABLE_ERR_RAD`，确认它们当前是否只定义未接入。

第七步，观察输出饱和后的积分状态。

- 人为设置较大目标误差时，观察 `pidCmd[]` 是否被限幅。
- 同时观察 `iTerm` 是否仍在更新。
- 如果 `iTerm` 继续增加，只能说明当前轴没有触发 `iHold` 暂停或已到硬限幅，
  不能把输出限幅误解为积分自动反算。

调试记录建议：

- 记录 D 项原始值、滤波值、三点平均值和扰动前后变化，但不要越界断言实物响应。
- 对目标斜坡、限幅和速率限制，应分别记录输入、限制表达式、限制前值和限制后值。
- 积分暂停记录应包含全局门控、Roll 专用门控、`iHold` 入参和 `iTerm` 是否变化。
- 输出约束记录应按顺序包含 `pidRaw`、幅值限幅后值、速率限制后值、`pidCmdPrev[]` 和电角输出。
- 目标斜坡记录应区分“宏/变量已定义”和“当前分支已调用”，不要用命名替代运行证据。
- 使用 `tools/pid_tuning_sim.py` 对比固件时，应单独记录 D 项历史初始化方式、误差差分是否二次角度包裹，以及脚本 `rate_limit` 是否对应当前轴固件表达式。
- 使用 `.map/.list` 佐证时，应把“符号进入镜像”和“真实运行效果”分开记录。
- 速率限制单位、扰动响应、震荡改善和稳定性结论缺少实测时保持【待验证】。

## 10. 常见问题

问题一：D 项滤波是不是改变 PID 的目标。

不是。D 项滤波只改变微分项的变化速度估计，不改变目标角本身。目标角处理属于目标斜坡和角度归一化。

问题二：三轴是否都使用目标角斜坡。

当前证据不是。Roll 路径有 `moveTowardsAnglef()` 调用证据，Pitch/Yaw 目标斜坡常量当前只发现定义，未发现运行路径调用。

问题三：`rateLimit` 是否一定表示“每秒限制”。

`config.c` 注释把它写作 rate limiting，但当前 Roll 使用 `rateLimit * safeDt`，
Pitch/Yaw 直接使用 `rateLimit`。
更细地说，源码注释还写明它限制的是 electrical rotation，而不是 mechanical。
当前比较对象是 `pidCmd[] - pidCmdPrev[]`，不是 `pointingCmd[]` 的机械目标变化。
因此 Pitch/Yaw 的单位边界需要继续验证，教材暂不把它解释成完全一致的每秒机械角速度限制。

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

问题八：D 项滤波越强是否越好。

不是。
更强的滤波会降低噪声和尖峰，但也会让 D 项响应更慢，增加相位滞后。
如果滞后过大，D 项提供的阻尼可能不再及时，甚至影响闭环稳定性。

问题九：输出被限幅后，积分项是否会自动反向修正。

当前源码没有这种证据。
`updatePID()` 有 `iHold` 和 `iTerm` 硬限幅，但轴级 `pidCmd[]` 被限制后，
没有把限幅误差反馈回 `iTerm`。所以不能把当前实现称为完整反算 anti-windup。

问题十：为什么 `pidCmdPrev[]` 应看作约束后输出历史。

因为代码在幅值限幅和速率限制后才更新 `pidCmdPrev[]`。
它用于下一帧计算输出变化量，因此代表上一帧实际送往电角合成路径的受限控制量。

问题十一：为什么 `rollDiag.stepLimit` 不能直接当作 Roll 最终限速阈值。

因为实际局部变量 `rollStepLimit` 会先按 `rateLimit * safeDt` 计算，
再和 `AXIS_MIN_STEP_LIMIT_RAD` 比较。若乘法结果小于 `0.001 rad/frame`，
最终限速阈值会被抬高，但 `rollDiag.stepLimit` 记录的仍是乘法结果。
所以 Roll 调试时应额外计算最终实际阈值。

问题十二：目标角斜坡和输出速率限制是不是一回事。

不是。目标角斜坡发生在 PID 输入侧，改变 `command` 的每帧变化量；
输出速率限制发生在 PID 输出侧，限制 `pidCmd[]` 相对上一帧的变化。
在 `D_ERROR` 模式下，只有输入侧目标斜坡能直接减小目标阶跃进入原始 D 项的幅度。

问题十三：`PITCH_I_ENABLE_ERR_RAD` 和 `YAW_I_ENABLE_ERR_RAD` 是否已经生效。

当前没有运行证据。它们在 `computeMotorCommands.c` 中有定义，
但 Pitch/Yaw 当前传给 `updatePID()` 的是全局 `holdIntegrators`，
没有看到用各自误差阈值计算本轴 `iHold` 的代码。

问题十四：为什么 `D_ERROR` 会受目标突变影响。

因为它对 `error = command - state` 求差分。目标 `command` 突变时，
即使 `state` 尚未变化，`error` 也会变化，从而进入原始 D 项。
后级滤波和限幅能抑制传播，但不能改变突变已经进入 D 项差分这一事实。

问题十五：`outputRate[]` 是否等于最终实际输出变化量。

不一定。
三轴代码都是先计算 `outputRate[] = pidCmd[] - pidCmdPrev[]`，
再根据速率阈值改写 `pidCmd[]`。如果发生限速，`outputRate[]` 保留的是限速前差值。
最终实际变化量应由限速后的当前 `pidCmd[]` 减去上一帧受限输出计算。

问题十六：看到 `pitchTargetSlew` 或 `yawTargetSlew` 定义，是否说明 Pitch/Yaw 已经有目标斜坡。

不能。
定义变量和宏只能说明代码里有预留名字，不能说明运行路径已经接入。
当前 Pitch/Yaw 分支没有看到使用这些斜坡状态生成 PID 目标的证据，
所以教材只能把它们写成预留状态或未接入边界。

问题十七：为什么 `[-300, 300]` 的 D 项限幅不等于每帧 D 输出贡献上限。

因为原始 `dTerm` 被夹紧后，还要经过一阶低通和三点平均。
在历史值为 0、`dt=0.002s` 的第一帧，即使原始 `dTerm=300`，
`dAverage` 也约为 20.1。最终 D 项输出贡献还要再乘以 `D` 参数。
因此应区分原始 D 项限幅、滤波后 D 项和最终输出贡献。

问题十八：为什么仿真脚本中的 D 项尖峰不能直接当作固件尖峰。

因为脚本和固件在两个关键细节上不完全等价。
固件的 `D_ERROR` 初始化会在 `lastDcalcValue == 0.0f` 时先用当前 `error`
建立历史值，而脚本当前用 `state` 初始化 `last_d_calc`。
此外，固件会对 `error - lastDcalcValue` 再做一次角度包裹，脚本没有对这次差分再次包裹。
因此脚本可能高估初始帧 D 项，也可能在跨 `±pi` 时把小角度连续变化放大成被夹紧的尖峰。
脚本结论应作为调参提示，固件结论必须回到 `pid.c::updatePID()` 和实测日志验证。

## 11. 实践任务

开始任务前，先回到本章第8节定位源码证据：D 项滤波看 8.1 到 8.3，
目标斜坡看 8.4，输出约束看 8.5、8.6 和 8.9，积分暂停看 8.7 和 8.8。
第9节提供调试观察顺序。

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
同时记录 `rateLimit` 当前比较的是 `pidCmd[] - pidCmdPrev[]`，并说明它如何进入后续定子电角合成。
验收依据是速率限制表包含 Roll、Pitch、Yaw 三列表达式、源码结论、机械角/电角层级和单位设计【待验证】项。

任务六：验证 Roll 诊断字段边界。

记录 `pidRaw`、`pidClamped`、`pidApplied`、`dPidRaw`、`rollDiag.stepLimit`、
`safeDt` 和 `AXIS_MIN_STEP_LIMIT_RAD`。重新计算实际步长限制，并说明
`rollDiag.stepLimit` 在什么 `safeDt` 下可能小于实际生效限制。
验收依据是诊断表能区分“记录的乘法值”和“最终用于限速的实际阈值”。

任务七：验证积分暂停。

观察 `rollHoldIntegrators` 为真时，Roll 的 `iTerm` 是否停止更新；再观察全局 `holdIntegrators` 是否影响同一帧后续轴。
验收依据是积分暂停表包含局部参数、全局变量、影响轴和观察结论。

任务八：验证滤波系数数量级。

用 `F_CUT=20Hz` 和 `dt=0.002s` 计算 `rc` 与 `alpha`。
说明为什么该滤波会降低尖峰，同时带来响应滞后。
验收依据是计算表包含 `rc`、`alpha`、当前帧权重和滞后结论。

任务九：验证输出约束顺序。

任选一个轴，记录 `pidRaw`、NaN/Inf 回退后值、幅值限幅后值、速率限制后值和 `pidCmdPrev[]`。
验收依据是顺序表能证明 `pidCmdPrev[]` 是约束后历史，而不是原始 PID 输出历史。

任务十：验证 anti-windup 边界。

制造或模拟输出限幅场景，观察 `pidCmd[]` 已经饱和时 `iTerm` 是否仍在变化。
若 `iHold == false` 且 `iTerm` 继续变化，应记录为“当前没有输出饱和反算”。
验收依据是表格包含 `iHold`、`iTerm`、限幅前输出、限幅后输出和结论。

任务十一：比较目标斜坡和输出速率限制。

以 Roll 为例，分别记录 `pointingCmd[ROLL] -> rollTargetSlew` 和
`pidCmdPrev[ROLL] -> pidCmd[ROLL]` 两条链路。
说明前者如何影响进入 PID 的目标，后者如何影响 PID 后的执行指令。
验收依据是表格能清楚区分输入侧限制和输出侧限制。

任务十二：验证 D_ERROR 下目标阶跃影响。

用源码或仿真构造目标角阶跃，比较 `command`、`state`、`error`、`dTerm` 和 `dAverage`。
如果没有硬件安全夹具，只做离线仿真或调试器观察计划，实物响应保持【待验证】。
验收依据是能说明 `command[k] - command[k-1]` 如何进入原始 D 项。

任务十三：验证 Pitch/Yaw I_ENABLE 宏边界。

搜索 `PITCH_I_ENABLE_ERR_RAD` 与 `YAW_I_ENABLE_ERR_RAD` 的使用位置，
并记录 Pitch/Yaw 传给 `updatePID()` 的 `iHold` 来源。
验收依据是结论明确区分“宏已定义”和“运行路径已接入”。

任务十四：手算 D 项滤波第一帧响应。

设历史 D 项均为 0，`dTerm=300`，`dt=0.002s`，`F_CUT=20Hz`。
计算 `rc`、`alpha`、第一帧 `dTermFiltered`、第一帧 `dAverage`，
再分别代入 `D=0.008` 和 `D=0.016` 计算输出贡献。
验收依据是能说明原始 D 项限幅、滤波后 D 项和最终 D 输出贡献不是同一个量。

任务十五：验证 `outputRate[]` 与最终帧间变化量。

任选 Roll、Pitch 或 Yaw，记录限速前 `outputRate[axis]`、限速后 `pidCmd[axis]`、
上一帧受限输出和最终帧间差值。
验收依据是能说明发生限速时，`outputRate[]` 不会自动更新为最终应用差值。

任务十六：验证预留斜坡和门控变量。

搜索 `pitchTargetSlew`、`yawTargetSlew`、`yawCtrlAngle`、`pitchGateOpen`、
`rollSettledTime`、`YAW_CTRL_LPF_TAU_S` 和 `ROLL_SETTLE_TIME_S`。
分别记录“定义位置”和“是否有运行路径读取/写入”。
验收依据是能区分预留设计痕迹、未接入边界和已生效控制逻辑。

任务十七：验证仿真脚本与固件 D 项边界。

构造两组离线输入：一组为 `command=0`、`state=0.2rad`、D 历史清零；
另一组为上一帧误差接近 `+3.13rad`、当前误差接近 `-3.13rad`。
分别按固件 `pid.c::updatePID()` 和 `tools/pid_tuning_sim.py::update_pid()` 的顺序手算
`lastDcalcValue/last_d_calc`、原始 D 项、包裹结果、限幅结果和第一帧 `dAverage`。
验收依据是能说明脚本在哪些条件下会高估或低估固件 D 项瞬态，并明确脚本只作为调参辅助。

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
10. 为什么 D 项滤波会在抗噪声和响应速度之间形成权衡。
11. 为什么输出限幅不能自动防止积分项继续积累。
12. 如果 Pitch/Yaw 的 `rateLimit` 不乘以 `dt`，调试时应如何验证它的真实物理含义。
13. 为什么目标斜坡能减轻 `D_ERROR` 的目标阶跃冲击，而输出速率限制不能阻止原始 D 项看到阶跃。
14. 如果要接入 Pitch/Yaw 的 `*_I_ENABLE_ERR_RAD`，应如何避免破坏现有 Roll 触发的跨轴暂停语义。
15. 为什么 `outputRate[]` 的名字可能误导调试者，它和最终帧间变化量有什么区别。
16. 为什么只看到斜坡变量或门控宏的定义，不能说明该保护机制已经在运行。
17. 为什么 D 项原始限幅、滤波后 D 项和 `D * dAverage` 必须分开记录。
18. 为什么离线仿真脚本与固件只要在 D 项初值和角度差分包裹上有一点差异，就不能逐帧等价比较。

## 13. 本章总结

本章把第27章的 PID 主干继续扩展为“可运行、可保护、可调试”的控制细节。

当前项目中：

- `pid.c` 使用 `F_CUT=20Hz` 和一阶低通形式平滑 D 项。
- `dAverage` 由当前滤波 D 项和两帧历史值平均得到。
- `dTerm` 被限制到 `[-300, 300]`。
- 500Hz、`F_CUT=20Hz` 附近运行时，D 项低通 `alpha` 约为 0.2。
- `updatePID()` 在 `iHold == false` 时更新积分项，并把 `iTerm` 限制到 `[-10, 10]`。
- 当前积分保护是积分暂停与硬限幅，不是完整输出饱和反算 anti-windup。
- `windupGuard` 字段当前不是 `updatePID()` 的积分限幅来源。
- 当前默认 `D_ERROR` 会让目标阶跃进入原始 D 项，Roll 目标斜坡可减小该输入侧跳变。
- Roll 路径有 `moveTowardsAnglef()` 目标斜坡运行证据。
- Pitch/Yaw 目标斜坡常量当前只见定义，未发现参与控制路径调用证据。
- `PITCH_I_ENABLE_ERR_RAD` 和 `YAW_I_ENABLE_ERR_RAD` 当前只发现定义，未发现本轴积分门控接入证据。
- 三轴都有输出幅值限制，但 Roll、Pitch、Yaw 的速率限制实现细节不同。
- `rateLimit` 当前限制的是 `pidCmd[]` 相对 `pidCmdPrev[]` 的变化，随后影响定子电角合成；
  不能直接写成机械目标角速度限制。
- `pidCmdPrev[]` 保存约束后的输出历史，用于下一帧速率限制。
- `rollDiag.pidRaw`、`pidClamped`、`pidApplied` 分别对应 PID 原始输出、幅值限幅后值和速率限制后值。
- `rollDiag.dPidRaw` 是速率限制前的输出差值，不等于受限后的最终帧间变化量。
- `rollDiag.stepLimit` 记录 `rateLimit * safeDt`，低于最小步长时不等于实际 Roll 限速阈值。
- `pidCmd[]` 在同一帧内会从 PID 原始输出逐步变成幅值限幅后值和速率限制后值；
  调试时必须记录阶段位置，不能只按变量名判断它代表哪一层输出。
- 三轴 `outputRate[]` 都是限速判断前差值，不一定等于最终实际应用的帧间变化量。
- `pitchTargetSlew`、`yawTargetSlew`、`yawCtrlAngle` 等状态当前只证明定义存在，不能证明 Pitch/Yaw 目标斜坡已接入。
- `YAW_CTRL_LPF_TAU_S`、`YAW_ERR_DEADBAND_RAD` 和 Roll 收敛门控宏当前不能写成已生效运行逻辑。
- `tools/pid_tuning_sim.py` 与固件共享 D 项滤波和限幅思想，但在 D 项初值、角度差分包裹和统一速率限制上不是逐帧等价模型。
- `Debug/Three-axis_cloud_platformV2.map` 能证明 PID、输出约束函数和关键全局对象进入当前 Debug 镜像。
- `Debug/Three-axis_cloud_platformV2.list` 能证明三轴 `updatePID()` 调用、幅值限幅、速率限制、`pidCmdPrev[]` 更新和 `PWM_Motor_SetAngle()` 调用的构建路径。
- `.su/.cyclo` 能补充 `clampf()`、`moveTowardsf()`、`wrapToPif()`、`moveTowardsAnglef()`、
  `updatePID()`、`computeMotorCommands()` 和 `PWM_Motor_SetAngle()` 的静态栈与圈复杂度线索，
  但不能替代真实耗时、栈水位、硬件安全或闭环稳定性证据。
- Åström/Hägglund、Åström/Murray 和 MathWorks PID 文档能支撑 D 项噪声、设定值冲击、执行器饱和与 anti-windup 的通用工程背景，但不能替代本项目源码和构建产物证据。

本章保留十二个边界：

- Pitch/Yaw 的 `rateLimit` 当前未乘以 `dt`，与 Roll 分支不一致，需在调参或后续代码整理中继续验证。
- `config.c` 注释说明 `rateLimit` 面向 electrical rotation，教材不能把它简化为机械角速度限制。
- Pitch 轴“额外更严格限幅”代码当前上下限仍与全局 `[-10, 10]` 相同。
- 目标角斜坡机制当前不能写成三轴都已接入。
- Pitch/Yaw 的本轴积分阈值宏当前不能写成已经生效的本地门控。
- Roll 的 `rollDiag.stepLimit` 不一定等于实际生效的最终步长限制。
- `pidCmdPrev[]` 更新发生在速率限制之后，它是下一帧历史基准，不是原始 PID 输出缓存。
- `outputRate[]` 不能直接当作最终实际输出变化量。
- 原始 D 项 `[-300, 300]` 限幅不能直接等同于每帧 D 输出贡献上限。
- 预留斜坡、Yaw 平滑和 Roll 收敛门控变量需要运行路径证据才能写成已生效机制。
- 仿真脚本的 D 项尖峰只能作为趋势提示，不能替代固件 `updatePID()` 的逐帧日志。
- `.map/.list` 可以证明符号、对象和调用路径进入当前构建，不能证明硬件安全或闭环稳定。
- `.su/.cyclo` 只能作为静态复杂度和静态栈线索，不能写成运行时 2ms 预算达标证明。
- 软件限幅和速率限制只能证明数值被约束，不能证明硬件安全或闭环稳定；相关结论保持【待验证】。

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
- `Core/Src/main.c`
- `Debug/Three-axis_cloud_platformV2.map`
- `Debug/Three-axis_cloud_platformV2.list`
- `Debug/Drivers/SRC/Src/computeMotorCommands.su`
- `Debug/Drivers/SRC/Src/computeMotorCommands.cyclo`
- `Debug/Drivers/SRC/Src/pid.su`
- `Debug/Drivers/SRC/Src/pid.cyclo`
- `Debug/Drivers/CustomDrivers/Src/drv_pwmMotors.su`
- `Debug/Drivers/CustomDrivers/Src/drv_pwmMotors.cyclo`

权威参考资料：

- Karl J. Astrom and Tore Hagglund, `PID Controllers: Theory, Design, and Tuning`
- Karl J. Astrom and Richard M. Murray, `Feedback Systems: An Introduction for Scientists and Engineers`, PID control chapter: `https://fbswiki.org/wiki/index.php/PID_Control`
- IEEE Control Systems Society, Karl Astrom PID lecture material: `https://ieeecss.org/CSM/library/2018/feb18/06-PID.pdf`
- MathWorks documentation, `Anti-Windup Control Using PID Controller Block`: `https://www.mathworks.com/help/simulink/slref/anti-windup-control-using-a-pid-controller.html`

质量自检：

- P0 事实错误：通过
- P1 依赖断层：通过
- P2 逻辑连贯：通过
- P3 项目证据：通过
- P4 原理展开：通过
- P5 调试实践：通过
- P6 表达统一：通过
