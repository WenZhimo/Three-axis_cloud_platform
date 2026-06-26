# 第12章 通用定时器PWM输出

> 导航：上一章：[第11章_Newlib适配与UART调试输出](第11章_Newlib适配与UART调试输出.md) ｜ 下一章：[第13章_TIM8与TIM6项目配置](第13章_TIM8与TIM6项目配置.md)

## 1. 本章目标

- 理解通用定时器如何通过计数周期和比较值产生 PWM 输出。
- 看懂项目中 TIM2、TIM3、TIM4 的 PSC、ARR、PWM 模式和通道配置。
- 理解 CNT、输出比较、更新事件和预装载机制在 PWM 中分别承担什么角色。
- 能从 `.ioc`、`tim.c` 和 `drv_pwmMotors.c` 追踪 PWM 从配置、GPIO 复用、通道启动到 CCR 更新的路径。
- 能解释为什么项目初始化阶段使用 HAL，而运行时高频更新直接写 `TIMx->CCR`。
- 明确通用定时器 PWM 输出在三轴电机驱动中的基础作用。
- 区分本章的 PWM 基础输出与后续 TIM8/TIM6、三相正弦驱动、多定时器相位同步章节边界。

## 2. 前置知识

- 系统时钟树
- HAL MSP初始化机制
- GPIO输出与复用
- CMSIS寄存器与内核访问

第06章已经说明 APB1 定时器时钟来源，第07章已经说明 GPIO 复用和 MSP 初始化，第05章已经说明项目中存在寄存器级访问。

本章在这些前置基础上解释 TIM2、TIM3、TIM4 如何产生 PWM 波形，并说明 `drv_pwmMotors.c` 如何启动和更新这些通道。

本章不展开 TIM8 高级定时器配置、不展开 TIM6 基本定时器用途、不展开三相正弦 PWM 电机驱动算法、不展开多定时器相位同步细节。它们已经安排在后续章节。

## 3. 问题背景

三轴云台最终要驱动电机。对 MCU 来说，电机驱动并不是直接输出一个“角度”或“力矩”变量，而是需要把控制计算转换成定时器通道上的 PWM 占空比。

项目中这条输出基础链路可以从三个位置看到：

- `.ioc` 把 TIM2、TIM3、TIM4 的多个通道配置为 PWM Generation。
- `Core/Src/tim.c` 生成 TIM2、TIM3、TIM4 的 PWM 初始化和 GPIO 复用配置。
- `Drivers/CustomDrivers/Src/drv_pwmMotors.c` 启动这些 PWM 通道，并在运行时写入 `TIMx->CCR` 改变占空比。

如果读者只看控制算法，就会看到 `PWM_Motor_SetAngle()` 这样的抽象接口，却不知道电机输出实际落在哪些定时器通道上。如果只看 `tim.c`，又只能看到通道被配置，却不知道项目什么时候启动它们、怎样更新比较值。本章把这两部分连接起来。

## 4. 核心概念

- 通用定时器：STM32 中可用于计数、比较和 PWM 输出的定时器资源。本项目主 PWM 输出使用 TIM2、TIM3、TIM4。
- PSC：预分频器，决定定时器计数频率。本项目 TIM2、TIM3、TIM4 的 Prescaler 均为 0。
- ARR：自动重装载值，决定一个 PWM 周期包含多少个计数。本项目 TIM2、TIM3、TIM4 的 Period 均为 3599。
- CNT：当前计数值，表示定时器当前处于 PWM 周期中的哪个位置。
- CCR：捕获比较寄存器，PWM 输出中通常决定翻转或有效电平保持到哪个计数位置。本项目运行时直接写 `TIMx->CCR` 改变占空比。
- APB1 定时器时钟：TIM2、TIM3、TIM4 位于 APB1 侧。当前 PCLK1 为 36MHz，但 APB1 定时器频率为 72MHz，这是因为 APB1 分频不为 1 时，定时器时钟会按规则加倍。
- 输出比较：定时器把 CNT 与 CCR 比较，并根据比较结果改变通道输出电平。
- `OCxREF`：输出比较单元内部生成的参考信号，GPIO 引脚最终看到的是它经过极性、通道使能和复用输出后的结果。
- `CCxE`：捕获/比较输出使能位。PWM 通道已配置并不等于已经输出，通道还需要被启动并使能输出。
- 更新事件：定时器计数溢出、重装载或软件触发时产生的内部事件，可用于让某些预装载值生效，也可作为中断或 DMA 触发源。
- `EGR.UG`：软件触发更新事件的位。当前 HAL 基础定时配置会在写入 ARR 和 PSC 后设置 `TIMx->EGR = TIM_EGR_UG`，用于让预分频器等配置立即装载到工作路径。
- `OCxPE`：输出比较预装载使能位。当前 HAL PWM 通道配置会设置该位，使 CCR 新值通过预装载路径在更新事件后进入有效比较路径。
- 预装载/影子寄存器：先把新值写入缓冲寄存器，再在更新事件时统一装入实际工作寄存器，用来减少周期中途改参数造成的瞬态不一致。
- PWM 模式 1：计数器从 0 向上计数时，比较值之前和之后输出不同电平，从而形成占空比。
- 占空比：一个 PWM 周期内有效电平所占比例。简化理解可用 `CCR / (ARR + 1)` 表示。
- 占空比分辨率：ARR 为 3599 时，一个 PWM 周期有 3600 个计数位置，单个 CCR 计数对应约 `1 / 3600 = 0.0278%` 的占空比步进。
- 边沿对齐：计数器单向计数，PWM 周期边界集中在计数回零处。本项目 TIM2、TIM3、TIM4 使用向上计数，属于边沿对齐思路。
- 中心对齐：计数器上下计数，PWM 边沿围绕周期中心展开；当前项目未采用。
- 定时器通道：一个定时器可以有多个输出通道，例如 TIM3 CH1、CH2、CH3、CH4。
- 复用推挽输出：GPIO 引脚由定时器外设驱动输出波形，本项目 PWM 引脚都配置为 `GPIO_MODE_AF_PP`。

这些概念服务于正式知识点 `通用定时器PWM输出`，不新增结构外知识点。

## 5. 工作原理

PWM 的基本思想是：用固定频率的周期波形表示一个连续强度。

周期由定时器计数决定，占空比由比较值决定。

对向上计数 PWM 来说，定时器从 0 开始计数，当前值保存在 CNT 中。

CNT 计到 ARR 后回到 0。每一轮计数就是一个 PWM 周期。

CCR 是周期内的比较点。定时器硬件不断比较 CNT 和 CCR，并根据 PWM 模式决定通道输出电平。

CCR 越大，有效电平持续时间越长；CCR 越小，有效电平持续时间越短。

对本项目的 `TIM_OCMODE_PWM1`、向上计数和高有效极性来说，可以按下面规则理解：

```text
当 CNT < CCR 时，通道处于有效电平。
当 CNT >= CCR 时，通道处于无效电平。
```

因此，在 ARR 为 3599 时，一个周期包含 3600 个计数位置。

若 CCR 为 0，几乎没有有效电平；若 CCR 为 1800，约为 50% 占空比。

若 CCR 为 3600，则在 CNT 为 0 到 3599 的整个周期内都满足 `CNT < CCR`，可理解为接近 100% 占空比。

这也解释了为什么项目运行时把 CCR 限幅到 `period = htim3.Init.Period + 1`，而不是只限到 `ARR`。教材要把“ARR 是最大计数值”和“ARR+1 是一个周期的计数个数”分开讲清楚。

本项目 TIM2、TIM3、TIM4 的配置很统一：

- Prescaler = 0
- Period = 3599
- PWM 模式 = `TIM_OCMODE_PWM1`
- 初始 Pulse = 500
- 输出极性 = 高有效
- 自动重装载预装载 = Disable

这里容易跳过一个 STM32 前置规则：APB1 外设总线频率不一定等于 APB1 定时器时钟。

当前 `.ioc` 中：

- `RCC.APB1Freq_Value=36000000`
- `RCC.APB1TimFreq_Value=72000000`
- `RCC.APB1CLKDivider=RCC_HCLK_DIV2`

也就是说，普通 APB1 外设时钟 PCLK1 是 36MHz，但 TIM2、TIM3、TIM4 使用的 APB1 定时器时钟是 72MHz。教材计算 PWM 频率时必须使用 `APB1TimFreq_Value`，不能误用 `APB1Freq_Value`。

TIM2/TIM3/TIM4 又都配置为 Prescaler 0、Period 3599。因此按当前工程配置，PWM 频率可按：

`72000000 / (0 + 1) / (3599 + 1) = 20000Hz`

也就是约 20kHz。初始 Pulse 500 对应的初始占空比约为：

`500 / (3599 + 1) = 13.9%`

占空比分辨率为：

```text
duty_step = 1 / (ARR + 1)
          = 1 / 3600
          ≈ 0.0278%
```

这说明 `CCR` 每改变 1，理想占空比只改变约 0.0278 个百分点。若只从百分比角度看，当前 PWM 分辨率约为 12 位以内；若从电机实际力矩看，还会受到驱动器、电源、电机绕组和控制算法的共同影响，本章不提前推断。

运行时，项目不会一直保持这个初始 Pulse。

`PWM_Motor_SetAngle()` 会根据上层传入的角度和功率计算三个 CCR 值，再写入对应定时器通道。

也就是说，`tim.c` 给出 PWM 的基本载波和通道，`drv_pwmMotors.c` 在运行时改变每个通道的占空比。

### 5.1 最小教学单元拆分

把 `通用定时器PWM输出` 拆到工程可教学粒度，可以得到以下链路：

```text
APB1定时器时钟
-> PSC分频
-> CNT计数
-> ARR周期边界
-> 更新事件
-> CCR比较点
-> 输出比较
-> OCxREF内部参考信号
-> CCxE通道使能
-> PWM模式1
-> GPIO复用输出
-> 运行时CCR更新
```

其中，APB1 定时器时钟和 PSC 决定“计数有多快”，ARR 决定“一个周期有多长”，CCR 决定“有效电平持续多久”。

更新事件是一个容易被跳过的隐藏前置。它说明定时器并不是靠软件循环翻转引脚，而是在硬件计数边界产生内部事件。

当启用预装载时，新 ARR 或 CCR 可以先进入缓冲，再在更新事件时统一生效。

更新事件也不只来自计数溢出。当前 HAL 源码中的 `TIM_Base_SetConfig()` 会先写入 `TIMx->ARR` 和 `TIMx->PSC`，随后执行 `TIMx->EGR = TIM_EGR_UG`。这说明初始化阶段也会主动制造一次软件更新事件，用来让预分频器等基础参数进入定时器工作路径。

因此，教材中要把三层证据分开：`.ioc` 和 `tim.c` 证明“希望配置什么值”，HAL 源码证明“这些值如何写入硬件寄存器并触发更新事件”，调试器读寄存器或波形测量才证明“运行时硬件确实按这些值工作”。

本项目要把 ARR 预装载和 CCR 预装载分开看。

`tim.c` 中 TIM2、TIM3、TIM4 的 `AutoReloadPreload` 是 Disable，因此 ARR 预装载没有打开。

但本地 HAL 源码显示，`HAL_TIM_PWM_ConfigChannel()` 在配置 PWM 通道时会设置对应的 `OCxPE` 位，例如 CH1 设置 `TIM_CCMR1_OC1PE`，CH2 设置 `TIM_CCMR1_OC2PE`，CH3/CH4 设置 `TIM_CCMR2_OC3PE/OC4PE`。

因此，项目运行时虽然是软件直接写 `TIMx->CCRn`，但通道比较值仍可能通过输出比较预装载机制在更新事件边界进入有效比较路径。更准确的说法是：项目直接写 CCR 寄存器接口，不重新配置 PWM 模式；CCR 生效时序还要看 `OCxPE` 和更新事件。

### 5.1.1 从CCR写入到波形变化的四层时序

`PWM_Motor_SetAngle()` 写 `TIMx->CCRn` 时，读者容易把“CPU 已经写寄存器”和“引脚波形已经改变”合成一步。出版级教材要把这一步拆开：

| 时序层 | 当前项目证据 | 能证明什么 | 不能证明什么 |
|---|---|---|---|
| 软件请求层 | `drv_pwmMotors.c` 在临界区内写 `TIM3->CCR2/3/4`、`TIM3->CCR1`、`TIM2->CCR4/3`、`TIM4->CCR4`、`TIM2->CCR2`、`TIM4->CCR3` | 控制算法已经把三相目标值写到对应 CCR 接口 | 外部引脚已经立刻改变 |
| 通道预装载层 | `HAL_TIM_PWM_ConfigChannel()` 设置 `TIM_CCMR1_OC1PE/OC2PE` 或 `TIM_CCMR2_OC3PE/OC4PE`；设备头文件把这些位定义为 Output Compare preload enable | CCR 写入要按输出比较预装载路径理解 | 三个定时器一定在同一硬件时刻接收新值 |
| 更新事件层 | `TIM_Base_SetConfig()` 初始化时写 `TIMx->ARR`、`TIMx->PSC` 后执行 `TIMx->EGR = TIM_EGR_UG`；运行时周期边界也会产生更新事件 | 初始化阶段存在软件更新事件，预装载值的生效需要结合更新事件理解 | 当前仓库没有证明运行时三组定时器共享同一个更新事件 |
| 外部波形层 | 需要观察 `CCER.CCxE`、`CR1.CEN`、`CCMRx.OCxPE`、`CNT` 与示波器波形【待验证】 | 能把寄存器更新推进到引脚电平变化 | 不能由 `.list` 中一条 `str` 写 CCR 指令直接证明 |

这四层解释了为什么本章反复区分“写 CCR”和“波形正确”。`.list` 中看到 `str` 写入 CCR，只能证明当前 Debug 构建保留了软件写入动作；若要证明新比较值在预期 PWM 周期边界生效，还要读回 `OCxPE`、观察更新事件附近的 `CNT`，或用示波器比较写入前后的通道波形【待验证】。

### 5.2 边沿对齐、中心对齐和项目选择

本项目 TIM2、TIM3、TIM4 使用 `TIM_COUNTERMODE_UP`。

这意味着 PWM 周期按向上计数组织，属于边沿对齐的典型使用方式。

边沿对齐的好处是理解和计算简单：频率可直接按 `TimerClock / (PSC + 1) / (ARR + 1)` 计算，占空比可近似按 `CCR / (ARR + 1)` 理解。

中心对齐需要计数器上下计数，周期和比较事件的理解更复杂。它在一些电机控制场景中有改善开关对称性的价值，但当前仓库没有使用中心对齐模式。

因此，本章先把边沿对齐讲透；中心对齐只作为替代方案和扩展方向，不写成当前项目路径。

### 5.3 DMA、NVIC和当前项目边界

STM32 定时器可以把更新事件或比较事件连接到中断或 DMA 请求，用于在固定时刻执行软件回调或搬运新的比较值。

这是一种更自动化的 PWM 更新方式，适合波形表连续输出、采样同步或更高频率的批量寄存器更新。

当前项目的主 PWM 输出没有这样做。

当前仓库证据显示：

- TIM2/TIM3/TIM4 用 HAL 初始化为 PWM 输出。
- `PWM_Motor_Init()` 调用 `HAL_TIM_PWM_Start()` 启动通道。
- `PWM_Motor_SetAngle()` 在 500Hz 控制路径中直接写 `TIMx->CCR`。
- 未发现 TIM2/TIM3/TIM4 的业务 DMA 配置。
- 未发现 TIM2/TIM3/TIM4 中断承担主 PWM 更新任务。

因此，本项目采用的是“硬件定时器持续输出 PWM，软件在 500Hz 控制帧中更新 CCR”的方案。

它比 DMA 波形输出更简单，也更容易把 PID 输出和三相 CCR 计算放在同一处调试；代价是软件写入 CCR 的时刻由主循环执行位置决定。虽然当前 HAL 通道配置启用了 `OCxPE`，让 CCR 生效倾向于落在更新事件边界，但三组定时器之间是否同边界更新，仍需要同步配置或波形证据证明。

### 5.4 PWM载波和控制帧的时间尺度

本项目按当前配置得到约 20kHz PWM 载波：

```text
T_pwm = 1 / 20000 = 50us
```

主控制循环目标是 500Hz：

```text
T_control = 1 / 500 = 2ms
```

也就是说，一个 500Hz 控制帧中大约包含：

```text
N = T_control / T_pwm = 2ms / 50us = 40
```

个 PWM 周期。

这个比例解释了项目结构：定时器硬件每 50us 自动产生一次 PWM 周期，软件控制帧约每 2ms 计算一次新的目标，并把新的三相 CCR 写入寄存器。

因此，PWM 输出不是软件逐周期翻转 GPIO；软件只是在较低频率上更新比较值，硬件负责持续输出高频载波。

## 6. STM32实现机制

STM32 HAL 中，PWM 输出至少需要四类配置。

第一类是定时器基础参数。TIM2、TIM3、TIM4 都设置向上计数、Prescaler 0、Period 3599。TIM2 和 TIM4 还显式配置内部时钟源；TIM3 通过 PWM 初始化路径生成。

第二类是输出比较参数。`TIM_OC_InitTypeDef` 中配置 PWM 模式、Pulse、输出极性和快速模式。本项目使用 `TIM_OCMODE_PWM1`、Pulse 500、`TIM_OCPOLARITY_HIGH` 和 `TIM_OCFAST_DISABLE`。

第三类是通道配置。TIM2 启用 CH2/CH3/CH4，TIM3 启用 CH1/CH2/CH3/CH4，TIM4 启用 CH3/CH4。`HAL_TIM_PWM_ConfigChannel()` 把输出比较参数绑定到具体通道。

第四类是 MSP 后初始化。`HAL_TIM_MspPostInit()` 把定时器通道连接到 GPIO 引脚：

- TIM2 CH2/CH3/CH4：PA1、PA2、PA3。
- TIM3 CH1/CH2/CH3/CH4：PA6、PA7、PB0、PB1。
- TIM4 CH3/CH4：PB8、PB9。

这些引脚都配置为复用推挽输出，速度为低速。第07章已经说明 GPIO 复用的含义，本章把它接到 PWM 波形输出。

第五类是通道启动。`HAL_TIM_PWM_ConfigChannel()` 只说明通道模式和初始比较值已经配置；项目还需要在 `PWM_Motor_Init()` 中调用 `HAL_TIM_PWM_Start()`。

HAL 源码显示，`HAL_TIM_PWM_Start()` 会调用 `TIM_CCxChannelCmd(..., TIM_CCx_ENABLE)` 使能对应通道输出，并在非触发从模式下使能定时器计数。

拆成位级动作，至少包含：

- `TIM_CHANNEL_STATE_GET()`：先检查目标通道是否处于 `HAL_TIM_CHANNEL_STATE_READY`。
- `TIM_CHANNEL_STATE_SET(..., HAL_TIM_CHANNEL_STATE_BUSY)`：启动成功路径会把该通道标记为 BUSY。
- `HAL_TIM_PWM_ConfigChannel()`：写入 OC 模式、初始 Pulse，并设置对应 `OCxPE`。
- `TIM_CCxChannelCmd(..., TIM_CCx_ENABLE)`：设置对应 `CCxE`，让通道输出被使能。
- `__HAL_TIM_ENABLE()`：设置 `TIMx->CR1.CEN`，让计数器开始运行。
- 对高级定时器还可能涉及 `MOE`，但 TIM2/TIM3/TIM4 是本章主线，不把 TIM8 的高级输出门控混入本章。

因此，`HAL_TIM_PWM_Start()` 的成功证据不能只看函数是否被调用。更完整的链路是：

```text
ChannelState == READY
-> HAL_TIM_PWM_Start() returns HAL_OK
-> ChannelState becomes BUSY
-> CCxE is set
-> CEN is set
-> CNT starts counting
-> GPIO AF output can present waveform
```

所以调试时要区分三层证据：`tim.c` 已配置通道、`PWM_Motor_Init()` 已启动通道、外部引脚上已经测到波形【待验证】。

项目还配置了 TIM8 和 TIM6，但它们不属于本章主线。TIM8 的高级定时器属性和中断配置放到第13章；TIM6 的基本定时器用途也放到第13章。

### 6.1 HAL、LL和裸寄存器的工程权衡

同一个 PWM 输出，可以有三种常见实现层级。

第一种是 HAL 初始化。它用 `HAL_TIM_PWM_Init()`、`HAL_TIM_PWM_ConfigChannel()` 和 `HAL_TIM_PWM_Start()` 完成结构体配置、参数检查和状态管理。

本项目用 HAL 完成初始化，因为初始化发生频率低，清晰性和 CubeMX 可追踪性比极限性能更重要。

第二种是 LL 或寄存器级运行时更新。它直接写 `TIMx->CCRn`，省去重新配置通道的函数开销。

本项目运行时更新 CCR 采用这种方式，因为 500Hz 控制循环中每次都要更新三相输出，直接写比较寄存器更短、更可预测。

从 HAL/LL 的角度看，直接写 `TIMx->CCRn` 与 `__HAL_TIM_SET_COMPARE()` 或 `LL_TIM_OC_SetCompareCHx()` 属于同一类动作：改变比较寄存器值。

区别在于项目代码显式写寄存器，少了一层宏调用；教学上更能看到最终落点，但也要求读者知道这些写入不是重新配置 PWM 模式，而只是更新比较值。

第三种是 DMA 或中断驱动更新。它可以把一串 CCR 值交给硬件节拍搬运，或者在定时器事件中执行软件更新。

当前项目没有采用这种方式。原因不是 STM32 不支持，而是当前控制输出来自 500Hz 姿态和 PID 计算，CCR 值需要先由软件实时计算出来；直接写 CCR 已能满足当前仓库主线表达。

如果后续要输出固定波形表、提高更新频率或减少主循环抖动，才需要重新评估 DMA 或定时器中断方案。

### 6.2 ARR预装载与CCR预装载

`Core/Src/tim.c` 中 TIM2、TIM3、TIM4 的 `AutoReloadPreload` 都是 `TIM_AUTORELOAD_PRELOAD_DISABLE`。

这说明 ARR 预装载没有开启。ARR 决定周期长度，当前项目运行时没有发现动态改 ARR 的业务路径，因此关闭 ARR 预装载不会影响当前 20kHz 固定载波的基本成立。

还要再拆一层：ARR 预装载关闭，不等于初始化阶段没有更新事件。`HAL_TIM_PWM_Init()` 会调用 `TIM_Base_SetConfig()`，该函数把 `Structure->Period` 写入 `TIMx->ARR`，把 `Structure->Prescaler` 写入 `TIMx->PSC`，随后写 `TIMx->EGR = TIM_EGR_UG`。在当前工程里，`Structure->Period` 来自 `htimx.Init.Period = 3599`，`Structure->Prescaler` 来自 `htimx.Init.Prescaler = 0`。

这条链路能解释一个常见疑问：`tim.c` 中的 `htimx.Init` 字段不是硬件寄存器本身，它先是 HAL 句柄中的配置意图；进入 `HAL_TIM_PWM_Init()` 或 `HAL_TIM_Base_Init()` 后，HAL 才把这些字段写到 `ARR/PSC`。构建产物中的 `TIM_Base_SetConfig()`、`TIMx->ARR`、`TIMx->PSC` 和 `TIMx->EGR = TIM_EGR_UG` 只能证明这条写寄存器路径进入了 ELF，不能替代断点下的寄存器读数或示波器频率测量。

但 CCR 是另一件事。`HAL_TIM_PWM_ConfigChannel()` 会在通道配置阶段设置对应 `OCxPE`，因此本章不能再把当前项目描述成“没有输出比较预装载”。

更准确的链路是：

```text
PWM_Motor_SetAngle()
  -> 写 TIMx->CCRn
  -> CCR 预装载路径
  -> 更新事件
  -> 新比较值进入有效比较逻辑
```

这对教学很关键：项目“直接写 CCR”指的是软件不再调用 HAL 重新配置通道，而不是一定代表新值在同一个 CPU 写周期立即改变外部电平。

项目用 20kHz PWM 载波承载 500Hz 控制更新。也就是说，控制层更新频率远低于 PWM 载波频率；这种比例让直接 CCR 更新在工程上更容易接受。

还要注意：`PWM_Motor_SetAngle()` 在三相 CCR 写入前后关闭和恢复全局中断。这能减少三相写入序列被中断打断的风险。

但关闭中断不能证明 TIM2、TIM3、TIM4 的更新事件严格同时发生，也不能单独证明外部三相波形严格同相同步。相位和边沿一致性仍需要定时器配置、CNT 观察或示波器证据【待验证】。

可以把这里的证据边界压成三层：

| 层级 | 项目证据 | 能证明什么 | 不能证明什么 |
|---|---|---|---|
| 软件写入连续性 | `PWM_Motor_SetAngle()` 在 `__disable_irq()` 与 `__enable_irq()` 之间连续写三路 `TIMx->CCRn` | 三相 CCR 写入序列不容易被普通中断打断，软件侧减少了半更新状态暴露给中断处理的风险 | CPU 写三个寄存器仍有先后顺序，不是硬件同时写入 |
| PWM生效边界 | `HAL_TIM_PWM_ConfigChannel()` 设置 `OCxPE`，CCR 新值需结合更新事件理解 | 新比较值倾向于在更新事件边界进入有效比较逻辑，而不是重新初始化通道 | 不同定时器的更新事件天然同相，或三相外部边沿严格同步 |
| 运行与波形证据 | `Debug/Three-axis_cloud_platformV2.list` 显示 CCR 写入和临界区进入当前构建 | 当前 Debug 镜像包含这条直接寄存器更新路径 | 实际 20kHz 波形、三相相位、电机相线电压和跨定时器同步效果；这些仍需 CNT 读数或示波器测量【待验证】 |

## 7. 项目中的应用

本章在项目运行流程中的位置如下：

1. `main()` 完成系统时钟配置后，依次调用 `MX_TIM3_Init()`、`MX_TIM2_Init()`、`MX_TIM4_Init()`。
2. 这些初始化函数在 `tim.c` 中配置 PWM 周期、通道和 GPIO 复用输出。
3. 用户初始化阶段调用 `PWM_Motor_Init()`。
4. `PWM_Motor_Init()` 调用 `HAL_TIM_PWM_Start()` 启动 TIM2、TIM3、TIM4 上用于电机输出的 9 个 PWM 通道。
5. 500Hz 实时控制循环中，`computeMotorCommands(dt500Hz)` 会在满足运行条件时调用 `PWM_Motor_SetAngle()`。
6. `PWM_Motor_SetAngle()` 根据目标轴、角度和功率计算三个比较值，并写入对应的 `TIMx->CCR`。

在项目“采集—处理—控制—输出”链路中，本章处于“输出”基础层。传感器采样和姿态计算决定控制输入，PID 和角度换算决定输出命令，而 TIM2/TIM3/TIM4 的 PWM 通道把命令转换成可送往电机驱动硬件的波形。

设计权衡可以概括为：

- 使用 TIM2/TIM3/TIM4，而不是软件翻转 GPIO：因为定时器硬件能持续产生 20kHz 载波。
- 使用 HAL 初始化，而不是全手写寄存器初始化：因为 CubeMX 配置可追踪，便于教学和移植。
- 运行时直接写 CCR，而不是反复调用 HAL 配置函数：因为控制循环只需要改变比较值，不需要重新配置通道模式。
- 当前不使用 DMA 更新 CCR：因为 CCR 来自实时姿态和 PID 计算，不是预先固定的波形表。
- 当前不使用 TIM2/TIM3/TIM4 中断更新 PWM：因为主循环 500Hz 控制帧已经组织了传感器、姿态、PID 和输出顺序。

## 8. 代码分析

### 1. `.ioc` 中的 PWM 通道证据

`.ioc` 明确记录了 TIM2、TIM3、TIM4 的 PWM 通道和引脚：

- PA1、PA2、PA3 对应 TIM2 CH2、CH3、CH4。
- PA6、PA7、PB0、PB1 对应 TIM3 CH1、CH2、CH3、CH4。
- PB8、PB9 对应 TIM4 CH3、CH4。

`.ioc` 还记录 TIM2、TIM3、TIM4 的 Prescaler 为 0，Period 为 3599，初始 Pulse 为 500。这是 `tim.c` 生成配置的上游证据。

### 2. `MX_TIM2_Init()`

TIM2 配置了内部时钟源、PWM 初始化、主从同步关闭和 CH2/CH3/CH4 三个 PWM 通道。它的项目意义是提供三路 PWM 输出，其中 `drv_pwmMotors.c` 后续会使用 TIM2 CH2、CH3、CH4 分别参与不同电机轴的输出。

### 3. `MX_TIM3_Init()`

TIM3 配置了 CH1/CH2/CH3/CH4 四个 PWM 通道。它是项目中使用通道最多的通用定时器。`PWM_Motor_Init()` 会启动 TIM3 的四个通道；`PWM_Motor_SetAngle()` 也会直接写 TIM3 的 CCR 寄存器。

### 4. `MX_TIM4_Init()`

TIM4 配置了 CH3/CH4 两个 PWM 通道。项目后续将它们用于某个电机轴的输出组合。本章只确认 TIM4 提供这两路 PWM 基础波形，具体三轴电机映射在后续章节展开。

### 5. `HAL_TIM_MspPostInit()`

`HAL_TIM_MspPostInit()` 是 PWM 引脚配置落到 GPIO 的位置。它启用 GPIOA 或 GPIOB 时钟，把对应引脚配置为 `GPIO_MODE_AF_PP`。

如果只执行 `HAL_TIM_PWM_ConfigChannel()`，但没有完成 GPIO 复用输出配置，波形就无法从对应引脚输出。

### 6. `PWM_Motor_Init()`

`PWM_Motor_Init()` 是项目业务代码启动 PWM 输出的入口。它先生成正弦查表，然后对 TIM3、TIM2、TIM4 的相关通道调用 `HAL_TIM_PWM_Start()`。

本章只分析启动通道这个事实。函数中还清零 TIM2/TIM3/TIM4 的计数器，这涉及多定时器相位同步，属于后续章节，本章只记录它位于 PWM 通道启动之后。

源码注释认为连续清零三个 `CNT` 可以保证三路定时器相位对齐。教材在本章需要更谨慎：这能说明项目有相位对齐意图，但“完美对齐”仍需要 CNT 读数、定时器同步配置或示波器波形作为更强证据。

当前 `PWM_Motor_Init()` 没有检查每次 `HAL_TIM_PWM_Start()` 的返回值，而是在启动调用后返回 `HAL_OK`。因此源码能证明项目尝试启动这些通道，但不能仅凭返回值链路证明每一路启动都成功；断点或寄存器状态仍是更强证据。

把这段启动过程拆成伪代码，可以看到证据缺口：

```text
for each PWM channel:
  status = HAL_TIM_PWM_Start(timer, channel)
  status is ignored

isInitialized = 1
return HAL_OK
```

这意味着 `PWM_Motor_Init()` 的 `HAL_OK` 更像“初始化流程执行到末尾”的返回值，而不是“9 路 PWM 通道全部由 HAL 确认启动成功”的聚合结果。若要把教材中的验证做扎实，应在调试时逐路观察 `HAL_TIM_PWM_Start()` 的返回值、`htim.ChannelState[]`、`TIMx->CCER`、`TIMx->CR1.CEN` 和 `TIMx->CNT`，而不是只看 `PWM_Motor_Init()` 最终返回值。

### 7. `PWM_Motor_SetAngle()`

`PWM_Motor_SetAngle()` 的输入是电机轴、角度和功率百分比。函数根据角度得到三个相位的输出值，再把结果写入对应 `TIMx->CCR`。

从本章角度看，关键不是正弦计算本身，而是最后的寄存器写入：CCR 值改变后，经由当前 HAL 配置打开的输出比较预装载路径，在更新事件后影响后续 PWM 周期占空比。这说明项目并不是反复重新初始化定时器，而是在运行时更新比较寄存器。

函数内部使用 `period = htim3.Init.Period + 1`，并把三相 `ccr_val[]` 限制在 0 到 `period` 之间。结合本章 PWM1 规则，可以把它理解为把占空比限制在 0% 到接近 100% 的范围内。

这里还要避免一个常见误读：`power_percent` 不是某一路 PWM 引脚的最终占空比百分数。
源码先把 `power_percent` 作为三相正弦幅值参数使用，计算 `current_amp = half_period * power_percent / 100`，
再把正弦表值叠加到 `half_period` 附近得到三相 `ccr_val[]`。此外当前实现中
`power_percent > 60.0f` 会被改写为 `40.0f`，不是简单饱和到 60%。所以第12章只能说
`PWM_Motor_SetAngle()` 最终通过写 CCR 改变占空比；至于 `power_percent` 如何变成三相正弦幅值，留到第29章和第30章展开。

由于 TIM2、TIM3、TIM4 当前 Period 都是 3599，所以使用 `htim3.Init.Period + 1` 作为统一周期在当前仓库内是成立的。若未来某个定时器 Period 不再一致，这个假设需要重新审查。

### 8. 权威资料对照

ST 的 STM32F10xxx 参考手册 RM0008 将通用定时器描述为具备预分频器、自动重装载寄存器、捕获/比较通道、输出比较、PWM 模式、更新事件、中断和 DMA 请求等能力。

ST 的 AN4776 通用定时器 cookbook 进一步把定时器用作时基、输出比较、PWM 生成、DMA burst 和波形生成等典型场景。

本章采用这些资料作为理论边界，但只把当前仓库中实际配置和调用的部分写入项目主线：

- 当前主线使用 TIM2/TIM3/TIM4 的 PWM 输出能力。
- 当前主线使用 PSC、ARR、CCR 和 PWM 模式 1。
- 当前 HAL 基础定时配置会通过 `TIM_Base_SetConfig()` 写入 `ARR/PSC`，并用 `TIM_EGR_UG` 触发一次软件更新事件。
- 当前 HAL PWM 通道配置会打开 `OCxPE`，因此 CCR 更新需要结合输出比较预装载和更新事件理解。
- 当前主线没有使用 TIM2/TIM3/TIM4 的业务 DMA 更新。
- 当前主线没有使用 TIM2/TIM3/TIM4 中断更新 CCR。
- TIM8 更新中断和 TIM6 配置留在第13章讨论。

### 8.1 构建产物证据边界

`Debug/Three-axis_cloud_platformV2.map` 可以证明本次构建把 `MX_TIM2_Init()`、`MX_TIM3_Init()`、`MX_TIM4_Init()`、`PWM_Motor_Init()`、`PWM_Motor_SetAngle()`、`HAL_TIM_PWM_Init()`、`HAL_TIM_PWM_Start()` 和 `HAL_TIM_PWM_ConfigChannel()` 等符号放入了最终镜像。它还显示 `HAL_TIM_PWM_Start_IT` 和 `HAL_TIM_PWM_Start_DMA` 这类函数存在于 HAL 对象代码中，但仅凭符号出现不能推断项目业务路径已经调用了中断或 DMA 方式更新 PWM。

`Debug/Three-axis_cloud_platformV2.list` 能进一步证明当前构建中的调用路径：`main()` 依次调用 `MX_TIM3_Init()`、`MX_TIM2_Init()`、`MX_TIM4_Init()`，随后调用 `PWM_Motor_Init()`；`PWM_Motor_Init()` 内部连续调用 9 次 `HAL_TIM_PWM_Start()`；`PWM_Motor_SetAngle()` 内部最终写入 `TIM3->CCR2/CCR3/CCR4`、`TIM3->CCR1`、`TIM2->CCR4/CCR3`、`TIM4->CCR4`、`TIM2->CCR2` 和 `TIM4->CCR3`。这类证据可以把“源码里写了”推进到“那次构建确实包含这些调用和寄存器写入”。

但是 `.map` 和 `.list` 仍有明确边界。它们不能证明外部引脚已经测到 20kHz 波形，不能证明 `HAL_TIM_PWM_Start()` 逐路返回 `HAL_OK`，不能证明 `TIMx->CCER`、`TIMx->CR1.CEN`、`TIMx->CNT` 和 `TIMx->CCMRx.OCxPE` 在板上运行时处于预期状态，也不能证明 TIM2/TIM3/TIM4 的边沿严格同步；这些结论仍需要断点寄存器读数或示波器证据【待验证】。

`.su/.cyclo` 可以把同一条 PWM 链路再补上一层静态资源边界。它们不能证明波形已经输出，也不能证明通道启动成功；它们只说明当前 Debug 编译中相关函数的静态栈估计和圈复杂度。

| 链路环节 | 函数 | 静态栈估计 | 圈复杂度 | 证据文件 | 证据边界 |
|---|---|---:|---:|---|---|
| TIM2基础配置 | `MX_TIM2_Init()` | 64 字节 | 8 | `Debug/Core/Src/tim.su` / `.cyclo` | 覆盖 TIM2 基础时基、PWM通道和 MSP 后初始化调用路径的函数级资源。 |
| TIM3基础配置 | `MX_TIM3_Init()` | 48 字节 | 7 | `Debug/Core/Src/tim.su` / `.cyclo` | 覆盖 TIM3 四路 PWM 配置路径的函数级资源。 |
| TIM4基础配置 | `MX_TIM4_Init()` | 64 字节 | 7 | `Debug/Core/Src/tim.su` / `.cyclo` | 覆盖 TIM4 两路 PWM 配置路径的函数级资源。 |
| GPIO复用输出 | `HAL_TIM_MspPostInit()` | 56 字节 | 5 | `Debug/Core/Src/tim.su` / `.cyclo` | 说明引脚复用配置函数进入静态分析，不能证明外部引脚已测到波形。 |
| HAL PWM初始化 | `HAL_TIM_PWM_Init()` | 16 字节 | 3 | `Debug/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.su` / `.cyclo` | 说明 HAL PWM 初始化函数有静态资源条目，不能替代寄存器读数。 |
| 基础寄存器装载 | `TIM_Base_SetConfig()` | 24 字节 | 16 | `Debug/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.su` / `.cyclo` | 对应 ARR、PSC 和软件更新事件路径，不能证明板上频率已经正确。 |
| PWM通道配置 | `HAL_TIM_PWM_ConfigChannel()` | 32 字节 | 6 | `Debug/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.su` / `.cyclo` | 对应 OC 模式和初始 Pulse 配置入口，不能证明通道已经输出。 |
| 通道启动 | `HAL_TIM_PWM_Start()` | 24 字节 | 18 | `Debug/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.su` / `.cyclo` | 对应通道状态检查、CCxE 使能和计数器启动路径，不能证明 9 路调用都返回 `HAL_OK`。 |
| 底层通道使能 | `TIM_CCxChannelCmd()` | 32 字节 | 1 | `Debug/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.su` / `.cyclo` | 对应 CCxE 写入辅助函数，不能替代 `TIMx->CCER` 运行时读数。 |
| 项目PWM启动 | `PWM_Motor_Init()` | 8 字节 | 2 | `Debug/Drivers/CustomDrivers/Src/drv_pwmMotors.su` / `.cyclo` | 说明项目启动入口本身很薄，但仍需要逐路检查 HAL 返回值。 |
| 运行时CCR更新 | `PWM_Motor_SetAngle()` | 80 字节 | 13 | `Debug/Drivers/CustomDrivers/Src/drv_pwmMotors.su` / `.cyclo` | 说明角度到三相 CCR 写入函数的静态资源规模，不能证明外部三相波形正确。 |

这张表的价值是帮助读者把“初始化阶段的 HAL 配置成本”和“运行阶段的直接 CCR 更新成本”分开看。`PWM_Motor_Init()` 自身只有 8 字节、圈复杂度 2，但它连续调用多路 `HAL_TIM_PWM_Start()`；`PWM_Motor_SetAngle()` 自身为 80 字节、圈复杂度 13，说明运行时占空比更新主要落在项目函数和寄存器写入路径上。上述静态数字仍不能简单相加成系统最大栈深，也不能换算成执行时间；真实 PWM 频率、通道启动状态、边沿同步和波形质量仍需寄存器断点、DWT/GPIO 计时或示波器证据【待验证】。

还要注意构建产物的新鲜度边界。当前 `.list` 中 `computeMotorCommands()` 对 Yaw 路径的反汇编旁注仍显示 `PWM_Motor_SetAngle(MOTOR_YAW, stator_electrical_angle, 40.0f)`，而当前 `Drivers/SRC/Src/computeMotorCommands.c` 源码已经是 `30.0f`。因此，本章引用 `.map/.list` 时只能把它们作为“该构建产物所记录的入口、符号和调用形状”证据；涉及具体常量、功率百分比或调参语义时，必须优先回到当前源码，并在重新构建后再用新的 `.list` 复核。

### 本节证据边界

本节只根据当前仓库说明文件、函数、宏、变量、调用关系和 Debug 构建产物。运行时频率、外部硬件表现、主机侧现象、传感器方向、电机响应、真实栈水位、函数耗时或仓库外控制效果仍需调试记录、日志或仓库外实测证据；缺少证据时保持【待验证】。

## 9. 调试方法

调试 PWM 输出时，应先确认配置链路，再确认运行链路。

配置链路观察点：

- `.ioc` 中 TIM2、TIM3、TIM4 是否配置了预期 PWM 通道。
- `tim.c` 中 Prescaler、Period、Pulse 是否与 `.ioc` 一致。
- `HAL_TIM_MspPostInit()` 是否把对应引脚配置为复用推挽输出。
- `MX_TIM3_Init()`、`MX_TIM2_Init()`、`MX_TIM4_Init()` 是否在 `main()` 中先于 `PWM_Motor_Init()` 调用。
- `HAL_TIM_PWM_Start()` 执行后，目标通道对应的 CCxE 位是否已经置位。

运行链路观察点：

- `PWM_Motor_Init()` 是否已经执行。
- `HAL_TIM_PWM_Start()` 是否覆盖项目需要的 9 个通道。
- 单步执行时，每次 `HAL_TIM_PWM_Start()` 返回值是否为 `HAL_OK`。
- 对应 `htim.ChannelState[]` 是否从 READY 进入 BUSY。
- `PWM_Motor_SetAngle()` 是否在电机使能后被调用。
- `TIMx->CCR` 是否会随输入角度或功率变化。
- `TIMx->CNT` 是否在运行中递增并周期性回绕。
- `TIMx->ARR` 是否等于初始化中的 Period。
- `TIMx->CR1.CEN` 是否置位，说明计数器已经使能。
- `TIMx->CCER` 中对应通道使能位是否保持有效。
- `TIMx->CCMR1/CCMR2` 中对应 `OCxPE` 是否置位，用于判断 CCR 预装载路径。
- 若怀疑更新时序问题，记录写 CCR 时 CNT 大致处于周期中的位置。

### 9.1 CCR 到外部波形的证据检查顺序

把 `TIMx->CCRn` 写入当成“已经有外部 PWM 波形”是一个常见跳步。当前项目可以把证据链拆成下面几个层次：

| 检查层次 | 项目证据 | 能证明什么 | 不能证明什么 |
|---|---|---|---|
| PWM 配置已进入工程 | `Core/Src/tim.c` 中 `MX_TIM2_Init()`、`MX_TIM3_Init()`、`MX_TIM4_Init()` 配置 `TIM_OCMODE_PWM1`、`Pulse = 500` 和目标通道 | CubeMX 生成的通道配置存在，且进入当前源码 | 不能证明通道已经启动，也不能证明引脚上有波形 |
| GPIO 复用路径存在 | `HAL_TIM_MspPostInit()` 把 TIM2/TIM3/TIM4 相关引脚配置为 `GPIO_MODE_AF_PP` | 定时器输出有对应的复用推挽引脚路径 | 不能证明外部驱动板接线正确 |
| 启动函数进入最终构建 | `.map` 中存在 `PWM_Motor_Init` 和 `HAL_TIM_PWM_Start`，`.list` 中 `PWM_Motor_Init()` 连续调用 9 次 `HAL_TIM_PWM_Start()` | Debug 构建包含 PWM 启动路径 | 不能证明每次启动返回值均为 `HAL_OK` |
| HAL 启动动作成立 | `Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c` 中 `HAL_TIM_PWM_Start()` 调用 `TIM_CCxChannelCmd(..., TIM_CCx_ENABLE)`，随后调用 `__HAL_TIM_ENABLE(htim)` | HAL 代码路径会使能 CCxE 和计数器 CEN | 不能证明运行时确实执行到该路径 |
| 运行时 CCR 写入路径成立 | `drv_pwmMotors.c` 中 `PWM_Motor_SetAngle()` 在关中断保护区写 `TIM3->CCR2/3/4`、`TIM3->CCR1`、`TIM2->CCR4/3`、`TIM4->CCR4`、`TIM2->CCR2`、`TIM4->CCR3`；`.list` 中可见对应 `str` 写寄存器指令 | 当前 Debug 构建存在角度到三相 CCR 写入路径 | 不能证明写入值已经在某个 PWM 周期边界前后产生预期外部波形 |
| 外部波形真实存在 | 示波器或逻辑分析仪记录目标引脚频率、占空比和相位【待验证】 | 引脚外部 PWM 行为被实测 | 当前仓库没有这类硬件实测证据 |

因此，本章能从源码和构建产物确认“配置存在、启动路径存在、CCR 写入路径存在”；但从 `CCRn` 写入到外部波形之间，还隔着通道使能位、计数器使能位、预装载/更新事件、GPIO 复用输出、外部接线和测量仪器。调试时应按上表逐层排除，避免把 `.list` 中的寄存器写指令直接等同于电机端三相波形正确。

常见异常定位：

- 没有 PWM 输出：先检查对应 `HAL_TIM_PWM_Start()` 是否调用，再检查 GPIO 复用输出配置。
- `PWM_Motor_Init()` 返回 `HAL_OK` 但某路无输出：不要只看最终返回值，逐路检查 `HAL_TIM_PWM_Start()` 返回值和通道状态。
- PWM 频率不符合预期：检查 APB1 定时器频率、Prescaler 和 Period。
- 频率算成 10kHz 或 40kHz：先确认是否把 PCLK1 36MHz 与 APB1 定时器时钟 72MHz 混淆。
- 某一路通道无输出：检查 `.ioc` 引脚、`HAL_TIM_PWM_ConfigChannel()` 和 `HAL_TIM_MspPostInit()` 三者是否一致。
- 通道已配置但仍无输出：继续检查 `HAL_TIM_PWM_Start()` 返回值、CCxE 位和 CNT 是否在运行。
- 占空比不变化：检查 `PWM_Motor_SetAngle()` 是否被调用，以及对应轴是否写到了预期的 `TIMx->CCR`。
- 只在部分电机轴上有输出：先检查 `drv_pwmMotors.c` 中的通道启动清单，再把具体轴映射留到后续三轴电机硬件映射章节分析。
- 占空比偶发突变：检查 CCR 写入值是否越界、功率百分比是否触发保护、以及写入时是否跨越 PWM 周期边界；若输入功率刚超过 60%，还要确认当前源码是否把它回退到 40%。
- 三相更新不一致：检查 `__disable_irq()` 保护区是否覆盖三相写入，同时用 CNT/波形确认是否仍存在周期内更新瞬态【待验证】。
- 想验证 DMA 或中断是否参与：搜索 `HAL_TIM_PWM_Start_DMA()`、`HAL_TIM_Base_Start_IT()`、`HAL_TIM_PWM_Start_IT()`、`HAL_DMA_Start()` 和相关 IRQ 入口。

当前仓库没有外部测量设备、驱动板接线或电机相线物理证据，因此具体引脚外部连接和波形实测结论需要标记为【待验证】。本章只确认工程内部 PWM 配置和软件写入路径。

调试记录建议：

- 记录定时器实例、PSC、ARR、CCR 初值、GPIO 复用引脚和 `HAL_TIM_PWM_Start()` 调用位置。
- 分开记录 `PWM_Motor_Init()` 的通道启动证据和 `PWM_Motor_SetAngle()` 的 CCR 写入证据。
- 外部 PWM 波形、驱动板响应、电机负载表现和相线方向属于仓库外实测证据，缺失时标记为【待验证】。
- 若只修改角度输入或功率输入，应同时记录对应轴、对应通道和 CCR 变化，避免把软件写入误判为硬件输出。

## 10. 常见问题

### 1. 为什么本项目用多个通用定时器输出 PWM？

触发条件：读者看到 TIM2、TIM3、TIM4 同时参与输出。

可能原因：一个通用定时器的通道数量有限，而三轴电机输出需要多路 PWM。项目把 TIM2、TIM3、TIM4 的通道组合起来使用。具体三轴通道映射在后续章节展开。

这里的重点不是“定时器越多越好”，而是当前项目的输出拓扑决定了需要多个 PWM 资源。
章节先讲基础波形，再讲后续的三相组合与轴映射，更符合读者的理解顺序。

### 2. Prescaler 为 0 是什么意思？

触发条件：读者以为 0 表示没有时钟。

可能原因：在 STM32 定时器中，Prescaler 寄存器值 0 表示除以 `0 + 1`，也就是不再分频。当前工程下 TIM2/TIM3/TIM4 使用 APB1 定时器频率直接计数。

这类配置很容易被初学者误读。
所以第12章必须把“寄存器值”和“实际分频结果”同时写出来，避免把 0 误解成关闭。

### 3. ARR 为什么是 3599？

触发条件：需要解释 PWM 周期。

可能原因：ARR 决定计数周期长度。按当前 APB1 定时器频率 72000000、Prescaler 0、Period 3599，PWM 频率约为 20kHz。

这说明 ARR 不是随手填的数字，而是和时钟、分频一起决定输出频率的核心参数。
本章只确认当前配置对应的频率，不替代后续电机映射章节的运行验证。

还要注意，本章计算用的是 APB1 定时器频率 72MHz，而不是 PCLK1 的 36MHz。若误用 PCLK1，会把 PWM 频率算成约 10kHz。

### 4. Pulse 500 是否就是运行时占空比？

触发条件：读者看到 `tim.c` 中所有通道初始 Pulse 为 500。

可能原因：Pulse 500 是初始化时的比较值。项目运行后会通过 `PWM_Motor_SetAngle()` 直接写 CCR，因此运行时占空比由后续控制输出决定。

也就是说，初始 Pulse 只说明上电后的默认比较值。
真正决定电机输出的，是运行时 CCR 被怎样更新。

### 4.1 CCR可以等于ARR+1吗？

触发条件：读者看到项目把 `ccr_val[]` 限幅到 `period = ARR + 1`。

可能原因：向上计数 PWM1 可以按 `CNT < CCR` 理解有效电平。CNT 在 0 到 ARR 之间运行，ARR 为 3599 时共有 3600 个计数位置。

因此，CCR 为 3600 时，整个周期内 CNT 都小于 CCR，可理解为接近 100% 占空比。

如果把 CCR 机械地理解成“不能超过 ARR”，就会误解项目里的限幅逻辑。更准确的说法是：有意义的占空比范围按 0 到 `ARR + 1` 理解；超出该范围的值会失去线性占空比意义。

### 5. 为什么 `drv_pwmMotors.c` 直接写 `TIMx->CCR`，而不是反复调用 HAL 配置函数？

触发条件：读者看到寄存器级 CCR 写入。

可能原因：初始化只需要做一次，运行时高频更新占空比只需要改变比较值。直接写 CCR 可以减少重复配置开销。第05章已经建立 CMSIS/寄存器访问前置，本章只解释它在 PWM 占空比更新中的作用。

如果每次都重新走完整 HAL 配置，开销会更大，也更容易把运行时更新和初始化配置混在一起。
所以这里的选择是“初始化用 HAL，运行时用 CCR”。

同时也要看到代价：直接写 CCR 不会自动检查通道状态、返回错误码或等待更新事件。

所以它适合已经初始化完成、路径明确、更新频率较高的控制循环；不适合用来替代初始化阶段的完整通道配置。

### 6. 为什么当前不使用 DMA 自动更新 CCR？

触发条件：读者知道定时器可以触发 DMA，疑惑为什么不用 DMA 推送 PWM 表。

可能原因：当前 CCR 值不是一张固定表，而是由 500Hz 控制循环根据姿态、PID、机械角和电角实时计算。

在这种结构下，主循环完成计算后直接写 CCR，链路更短，也更容易调试。

如果未来要输出固定测试波形、提高更新频率或降低 CPU 写寄存器开销，DMA 才值得作为重构方向。

### 7. 预装载关闭是否一定不好？

触发条件：读者看到 `AutoReloadPreload = Disable`，以为这一定是错误配置。

不能这样判断。

预装载可以让某些寄存器更新在更新事件时统一生效，适合追求严格周期边界的场景。

但当前项目需要分清 ARR 预装载和 CCR 预装载。

当前代码明确关闭的是 `AutoReloadPreload`，也就是 ARR 预装载。由于项目运行时没有动态改变 ARR，不能直接判定这会造成问题。

而 CCR 预装载由 `HAL_TIM_PWM_ConfigChannel()` 设置 `OCxPE` 打开。运行时写 `TIMx->CCRn` 后，应结合更新事件理解生效时机。

所以更准确的问题不是“预装载关闭是否不好”，而是“哪类预装载关闭、哪类预装载开启、项目是否依赖它们”。这就是本章把 ARR 和 CCR 分开讲的原因。

### 8. 为什么 PCLK1 是 36MHz，PWM 计算却用 72MHz？

触发条件：读者看到 `.ioc` 中 `RCC.APB1Freq_Value=36000000`，却在本章看到 PWM 频率按 72MHz 计算。

可能原因：STM32F1 中，当 APB 预分频器不是 1 时，该 APB 上的定时器时钟会按规则得到更高的频率。当前 `.ioc` 已经给出 `RCC.APB1TimFreq_Value=72000000`，TIM2/TIM3/TIM4 应使用这个定时器时钟计算。

因此，PWM 频率公式中的 `TimerClock` 不是普通 PCLK1，而是 APB1 timer clock。

### 9. TIM8 也配置了 PWM，为什么本章不讲？

触发条件：读者在 `tim.c` 或 `.ioc` 中看到 TIM8 PWM。

可能原因：TIM8 是高级定时器，并且项目还启用了 TIM8 更新中断。它已被安排到第13章，与 TIM6 一起用于区分辅助定时资源和主 PWM 输出路径。

这让第12章能专注于主 PWM 波形本身，而不把辅助定时器的用途、调度和中断混进来。

## 11. 实践任务

开始任务前，先回到本章第8节定位 TIM2/TIM3/TIM4 的初始化、GPIO 复用和 CCR 写入证据；第9节提供 PWM 输出调试顺序。

任务一：整理 PWM 通道和引脚。

在 `.ioc` 中列出 TIM2、TIM3、TIM4 的 PWM 通道和对应引脚。
验收依据是能得到 9 个通道的完整清单。

任务二：计算 PWM 频率和初始占空比。

在 `tim.c` 中找出 TIM2、TIM3、TIM4 的 Prescaler、Period 和 Pulse。
验收依据是参数表包含定时器、PSC、ARR、CCR 和频率计算结果。

任务二补充：区分 PCLK1 和 APB1 timer clock。

在 `.ioc` 中同时记录 `RCC.APB1Freq_Value` 和 `RCC.APB1TimFreq_Value`。
验收依据是能解释为什么当前 PWM 计算使用 72MHz，而不是 36MHz。

任务三：追踪 GPIO 复用配置。

在 `HAL_TIM_MspPostInit()` 中追踪每个 PWM 通道的 GPIO 复用配置。
验收依据是复用表包含定时器通道、GPIO 引脚、复用模式和输出路径。

任务四：确认项目启动的 PWM 通道。

在 `PWM_Motor_Init()` 中确认项目启动了哪些 PWM 通道。
验收依据是启动清单表包含通道名、`.ioc` 配置项和启动状态。

任务五：定位 CCR 写入位置。

在 `PWM_Motor_SetAngle()` 中找出 CCR 写入位置。
验收依据是写入表包含 CCR 寄存器、写入位置和占空比关系。

任务五补充：检查通道使能位。

在 `HAL_TIM_PWM_Start()` 执行后，观察对应 TIMx 的 `CCER` 寄存器。
验收依据是能说明“配置通道”和“使能通道输出”的区别。

任务五扩展：检查 CCR 预装载位。

观察 `TIMx->CCMR1/CCMR2` 中对应 `OCxPE` 位，说明 `HAL_TIM_PWM_ConfigChannel()` 是否已经开启输出比较预装载。
验收依据是能区分 ARR 预装载和 CCR 预装载。

任务六：拆分 PWM 最小教学单元。

把本章 PWM 链路拆成 APB1 定时器时钟、PSC、CNT、ARR、更新事件、CCR、输出比较、PWM 模式、GPIO 复用和运行时 CCR 更新。
验收依据是每个单元都能写出一句“它是什么”和一句“项目中在哪里出现”。

任务七：验证 DMA/NVIC 边界。

搜索 TIM2/TIM3/TIM4 是否存在业务 DMA 或中断更新 CCR 的证据。
验收依据是搜索记录包含关键词、命中文件、结论和“当前未进入主 PWM 更新路径”的边界说明。

实践边界：

当前任务优先形成表格、链路图、搜索记录和计算过程。涉及 IDE 现场、构建日志、断点数值、外部波形、主机侧结果或硬件响应时，若没有截图、日志或仓库外实测证据，结论保持【待验证】。

## 12. 思考题

1. 为什么 PWM 频率需要同时看 APB1 定时器频率、Prescaler 和 ARR？
2. 如果某一路引脚没有 PWM 波形，应先检查 `tim.c` 的通道配置，还是 `drv_pwmMotors.c` 的通道启动？为什么？
3. 初始化 Pulse 为 500，而运行时写 CCR，这两者在项目中分别承担什么角色？
4. 为什么第12章只讲 TIM2/TIM3/TIM4 的通用 PWM 基础，而不提前讲三相正弦 PWM 计算？
5. 如果多个通道分布在不同定时器上，后续为什么还需要讨论相位同步？
6. 如何区分“PWM 通道已配置”、“PWM 已启动”和“电机相线实际收到有效驱动”这三层证据？
7. 如果希望所有 CCR 更新严格在周期边界生效，预装载和更新事件可以提供什么帮助？
8. DMA 自动搬运 CCR 与主循环直接写 CCR 各适合什么场景？
9. 为什么当前项目更像“低频控制帧更新 CCR，高频定时器持续输出 PWM”，而不是“软件实时翻转 GPIO”？
10. 20kHz PWM 载波和 500Hz 控制帧之间约 40 倍的时间尺度差，对调试和性能分析有什么启发？
11. `__disable_irq()` 能解决三相 CCR 写入的哪些问题？又不能证明哪些硬件波形结论？
12. 为什么 ARR 预装载关闭和 CCR 预装载开启可以同时成立？它们分别影响什么寄存器？
13. 为什么当前 PWM 频率计算必须用 `RCC.APB1TimFreq_Value`，而不是只看 `RCC.APB1Freq_Value`？
14. `htimx.Init.Period` 写入结构体、`TIMx->ARR` 寄存器更新、`TIM_EGR_UG` 触发软件更新事件，这三层证据分别能证明什么？

## 13. 本章总结

本章建立了三轴云台项目中通用定时器 PWM 输出的证据链。

已经确认的结论是：

- `.ioc` 配置 TIM2、TIM3、TIM4 的 9 个 PWM 通道。
- TIM2、TIM3、TIM4 的 Prescaler 均为 0，Period 均为 3599，初始 Pulse 均为 500。
- 当前 PCLK1 为 36MHz，但 APB1 定时器频率为 72MHz，PWM 频率计算应使用后者。
- 按当前 APB1 定时器频率，TIM2/TIM3/TIM4 的 PWM 频率约为 20kHz。
- ARR 为 3599 时，占空比理论步进约为 0.0278%。
- `tim.c` 通过 `HAL_TIM_PWM_Init()`、`HAL_TIM_PWM_ConfigChannel()` 和 `HAL_TIM_MspPostInit()` 建立 PWM 输出基础。
- `PWM_Motor_Init()` 启动项目实际使用的 PWM 通道。
- `HAL_TIM_PWM_Start()` 会使能对应 CCxE 位，并在当前路径下启动定时器计数。
- 当前 `PWM_Motor_Init()` 没有聚合 9 次 `HAL_TIM_PWM_Start()` 的返回值，因此最终 `HAL_OK` 不能单独证明所有通道启动成功。
- `TIM_Base_SetConfig()` 会把 `htimx.Init.Period/Prescaler` 写入 `TIMx->ARR/PSC`，并通过 `TIM_EGR_UG` 触发软件更新事件。
- `HAL_TIM_PWM_ConfigChannel()` 会设置输出比较预装载位 `OCxPE`，所以 CCR 更新要结合更新事件理解。
- `PWM_Motor_SetAngle()` 通过写 `TIMx->CCR` 请求改变占空比。
- PWM1 向上计数和高有效极性下，可用 `CNT < CCR` 理解有效电平区间。
- 20kHz PWM 载波周期约 50us，500Hz 控制帧周期约 2ms，一个控制帧约包含 40 个 PWM 周期。
- 当前项目没有使用 TIM2/TIM3/TIM4 的业务 DMA 或中断来更新 CCR。
- 当前实现是“HAL 初始化 + 运行时直接 CCR 更新”的混合方案。
- 预装载、更新事件、DMA 和中断都是定时器能力的一部分，但本章只把当前仓库实际使用的路径写入主线。
- `.su/.cyclo` 能证明 TIM2/3/4 初始化、GPIO 复用、HAL PWM 配置/启动和运行时 CCR 更新函数具有静态栈与圈复杂度条目，但不能替代运行时寄存器、耗时或示波器证据。

本章边界：

- 本章确认 PWM 配置、启动入口和 CCR 更新路径，不确认电机相线实际收到有效驱动。
- 三相正弦、电角换算、多定时器相位和硬件点测都留到后续电机章节继续验证。
- 不把 `.su/.cyclo` 中的函数级静态资源条目写成 9 路 PWM 均启动成功、系统最大栈水位、真实执行耗时或 20kHz 外部波形证据。

下一章可以进入 TIM8 与 TIM6 项目配置，因为本章已经解释了主 PWM 输出基础，后续需要区分高级定时器、基本定时器和主电机输出路径之间的边界。

参考资料：

- STMicroelectronics, `RM0008 STM32F10xxx reference manual`。
- STMicroelectronics, `AN4013 Introduction to timers for STM32 MCUs`。
- STMicroelectronics, `AN4776 General-purpose timer cookbook for STM32 microcontrollers`。

---

### 章节尾部固定检查

知识链路：

知识点总表
↓
知识依赖图
↓
学习优先级
↓
教学顺序
↓
教材章节

本章对应正式知识点：

- 通用定时器PWM输出

项目证据：

- `Core/Src/tim.c`
- `Three-axis_cloud_platformV2.ioc`
- `Drivers/CustomDrivers/Src/drv_pwmMotors.c`
- `Drivers/CustomDrivers/Inc/drv_pwmMotors.h`
- `Core/Src/main.c`
- `Drivers/SRC/Src/computeMotorCommands.c`
- `Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c`
- `Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_tim.h`
- `Debug/Three-axis_cloud_platformV2.map`
- `Debug/Three-axis_cloud_platformV2.list`
- `Debug/Core/Src/tim.su`
- `Debug/Core/Src/tim.cyclo`
- `Debug/Drivers/CustomDrivers/Src/drv_pwmMotors.su`
- `Debug/Drivers/CustomDrivers/Src/drv_pwmMotors.cyclo`
- `Debug/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.su`
- `Debug/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.cyclo`

引用的函数、配置项和变量：

- `MX_TIM2_Init()`
- `MX_TIM3_Init()`
- `MX_TIM4_Init()`
- `HAL_TIM_PWM_Init()`
- `TIM_Base_SetConfig()`
- `HAL_TIM_PWM_ConfigChannel()`
- `HAL_TIM_MspPostInit()`
- `HAL_TIM_PWM_Start()`
- `HAL_TIM_CHANNEL_STATE_READY`
- `HAL_TIM_CHANNEL_STATE_BUSY`
- `TIM_CHANNEL_STATE_GET()`
- `TIM_CHANNEL_STATE_SET()`
- `TIM_CCxChannelCmd()`
- `PWM_Motor_Init()`
- `PWM_Motor_SetAngle()`
- `computeMotorCommands()`
- `htim2`
- `htim3`
- `htim4`
- `TIM2->CCR2`
- `TIM2->CCR3`
- `TIM2->CCR4`
- `TIM3->CCR1`
- `TIM3->CCR2`
- `TIM3->CCR3`
- `TIM3->CCR4`
- `TIM4->CCR3`
- `TIM4->CCR4`
- `TIMx->CNT`
- `TIMx->ARR`
- `TIMx->PSC`
- `TIMx->EGR`
- `TIMx->CR1`
- `TIMx->CCER`
- `TIMx->CCMR1`
- `TIMx->CCMR2`
- `TIMx->CR1.CEN`
- `TIM_CCMR1_OC1PE`
- `TIM_CCMR1_OC2PE`
- `TIM_CCMR2_OC3PE`
- `TIM_CCMR2_OC4PE`
- `TIM_EGR_UG`
- `TIM_CCER_CC1E`
- `TIM_CCER_CC2E`
- `TIM_CCER_CC3E`
- `TIM_CCER_CC4E`
- `TIM2.Period=3599`
- `TIM3.Period=3599`
- `TIM4.Period=3599`
- `TIM2.Prescaler=0`
- `TIM3.Prescaler=0`
- `TIM4.Prescaler=0`
- `RCC.APB1Freq_Value=36000000`
- `RCC.APB1TimFreq_Value=72000000`

质量自检：

- P0 事实错误：通过。
- P1 依赖断层：通过。
- P2 逻辑连贯：通过。
- P3 项目证据：通过。
- P4 原理展开：通过。
- P5 调试实践：通过。
- P6 表达统一：通过。

---
> 导航：上一章：[第11章_Newlib适配与UART调试输出](第11章_Newlib适配与UART调试输出.md) ｜ 下一章：[第13章_TIM8与TIM6项目配置](第13章_TIM8与TIM6项目配置.md)
