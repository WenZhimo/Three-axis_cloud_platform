# 第13章 TIM8与TIM6项目配置

## 1. 本章目标

- 区分 TIM8 高级定时器、TIM6 基本定时器和第12章 TIM2/TIM3/TIM4 主 PWM 输出路径。
- 看懂项目中 TIM8 的 PWM 通道、Break/DeadTime 配置和更新中断入口。
- 看懂项目中 TIM6 的基本计数配置和 `main.c` 中清零计数器的证据。
- 能判断“已配置”“已初始化”“已启动”“已用于业务逻辑”之间的差异。
- 为后续多定时器 PWM、三轴电机映射和硬件诊断章节建立辅助定时资源边界。

## 2. 前置知识

- 系统时钟树
- NVIC中断配置
- 通用定时器PWM输出
- CMSIS寄存器与内核访问

第06章已经说明定时器依赖系统时钟，第09章已经说明中断入口和 NVIC 配置，第12章已经说明 TIM2/TIM3/TIM4 的主 PWM 输出路径。本章在这些前置知识上解释 TIM8 和 TIM6 为什么不能直接和主 PWM 输出混为一谈。

本章不展开三相正弦 PWM 算法、不展开多定时器相位同步、不展开电机硬件诊断。TIM8 和 TIM6 在当前工程中主要作为已配置的辅助定时资源来分析。

## 3. 问题背景

第12章已经确认，项目真正启动并写 CCR 的主 PWM 输出来自 TIM2、TIM3、TIM4。可是 `tim.c` 和 `.ioc` 中还可以看到 TIM8 和 TIM6：

- TIM8 配置了 PWM CH1/CH2/CH3、PC6/PC7/PC8 引脚、Break/DeadTime 结构，并启用了 `TIM8_UP_IRQn`。
- TIM6 配置了 Prescaler 35、Period 65535，并在 `main.c` 中被清零计数器。

如果不单独解释，读者容易产生两个误解：

1. 看到 TIM8 PWM 配置，就以为它也参与当前三轴电机主输出。
2. 看到 TIM6 被清零，就以为项目周期调度来自 TIM6。

当前源码证据并不支持这两个结论。本章的任务是把“配置存在”和“业务使用”分开。

## 4. 核心概念

- 高级定时器：STM32 中功能更完整的定时器类型，通常支持高级 PWM、互补输出、Break/DeadTime 等能力。本项目中的 TIM8 属于高级定时器。
- 基本定时器：主要提供基础计数和更新事件能力，不提供普通 GPIO PWM 输出通道。本项目中的 TIM6 属于基本定时器。
- 更新中断：定时器计数周期结束后可触发的中断事件。项目启用了 TIM8 更新中断入口。
- Break/DeadTime：高级定时器面向电机控制和功率驱动场景的保护与死区相关配置。本项目 TIM8 初始化了该结构，但 DeadTime 为 0，BreakState 为 Disable。
- 已初始化：初始化函数配置了外设寄存器和句柄，例如 `MX_TIM8_Init()` 和 `MX_TIM6_Init()`。
- 已启动：运行时调用启动函数，让定时器或 PWM 通道开始工作，例如第12章中的 `HAL_TIM_PWM_Start()`。
- 业务逻辑：项目在中断、主循环或驱动函数中实际依赖该资源产生控制效果。

这些概念服务于正式知识点 `高级定时器TIM8` 和 `基本定时器TIM6`，不新增结构外知识点。

## 5. 工作原理

TIM8 和 TIM6 分别代表两类不同的定时器资源。

TIM8 是高级定时器。它可以配置 PWM 通道，也可以配置更新中断，还具备 Break/DeadTime 相关能力。当前项目中，TIM8 的 Prescaler 为 3，Period 为 999，PWM CH1/CH2/CH3 的 Pulse 为 500。按当前工程时钟配置，TIM8 属于 APB2 定时器资源；如果以 72MHz 定时器频率计算，频率关系是：

`72000000 / (3 + 1) / (999 + 1) = 18000Hz`

也就是约 18kHz。这个推导只说明当前配置下 TIM8 PWM 基础频率，不说明它已经参与电机输出。因为项目中没有发现 `HAL_TIM_PWM_Start(&htim8, ...)`。

TIM6 是基本定时器。它没有 TIM2/TIM3/TIM4 那样的输出比较通道。当前项目中，TIM6 的 Prescaler 为 35，Period 为 65535。若按 APB1 定时器频率 72MHz 计算，计数频率约为：

`72000000 / (35 + 1) = 2000000Hz`

也就是每个计数约 0.5us。但当前工程没有发现 `HAL_TIM_Base_Start(&htim6)` 或 `HAL_TIM_Base_Start_IT(&htim6)`。`main.c` 中只看到 `__HAL_TIM_SET_COUNTER(&htim6, 0)` 清零计数器。因此，本章不能把 TIM6 写成当前主时间基准。

项目当前主周期仍以第09章讲过的 SysTick 标志为准；第12章主 PWM 输出仍以 TIM2/TIM3/TIM4 为准。

## 6. STM32实现机制

### 1. TIM8 初始化机制

`MX_TIM8_Init()` 执行了以下配置：

- `htim8.Instance = TIM8`
- Prescaler = 3
- CounterMode = Up
- Period = 999
- RepetitionCounter = 0
- 内部时钟源
- PWM 初始化
- CH1/CH2/CH3 配置为 PWM1
- Pulse = 500
- Break/DeadTime 配置

`HAL_TIM_MspPostInit()` 又把 PC6、PC7、PC8 配置为 TIM8 CH1、CH2、CH3 的复用推挽输出。

`HAL_TIM_Base_MspInit()` 在 TIM8 分支中启用 TIM8 时钟，并配置 `TIM8_UP_IRQn` 的优先级为 1、0，然后使能该中断。

### 2. TIM8 中断入口

`stm32f1xx_it.c` 中存在 `TIM8_UP_IRQHandler()`，函数内部调用 `HAL_TIM_IRQHandler(&htim8)`。

这说明工程具备 TIM8 更新中断入口。但当前项目文件中没有发现用户实现的 `HAL_TIM_PeriodElapsedCallback()`，也没有在 `TIM8_UP_IRQHandler()` 用户代码区加入业务逻辑。因此，本章只能确认中断入口存在，不能把它写成已承担控制周期任务。

### 3. TIM6 初始化机制

`MX_TIM6_Init()` 执行基础计数配置：

- `htim6.Instance = TIM6`
- Prescaler = 35
- CounterMode = Up
- Period = 65535
- AutoReloadPreload = Disable
- MasterOutputTrigger = Reset
- MasterSlaveMode = Disable

`HAL_TIM_Base_MspInit()` 在 TIM6 分支中启用 TIM6 时钟。TIM6 没有 GPIO PWM 后初始化段，也没有输出通道配置。

### 4. TIM6 在 `main.c` 中的出现位置

`main()` 在外设初始化阶段调用 `MX_TIM6_Init()`。随后在系统准备就绪附近执行 `__HAL_TIM_SET_COUNTER(&htim6, 0)`。

这条证据说明项目确实访问了 TIM6 计数器，但当前源码没有显示 TIM6 被启动为运行中的业务计时器。清零计数器可以作为辅助计时准备动作，但不能单独证明 TIM6 已参与项目调度。

## 7. 项目中的应用

第13章在项目中的作用是“辅助定时资源索引”。

运行流程上，`main()` 先完成系统时钟，再初始化 TIM3、TIM2、TIM4、I2C2、TIM8、USART3、USB Device 和 TIM6。随后项目初始化 SysTick、DWT、配置对象、PWM 电机输出、滤波器、PID 和传感器。

与第12章不同的是：

- TIM2/TIM3/TIM4 在 `PWM_Motor_Init()` 中被 `HAL_TIM_PWM_Start()` 启动，并在 `PWM_Motor_SetAngle()` 中持续写 CCR。
- TIM8 虽然配置了 PWM 和更新中断，但当前源码没有发现 PWM 启动调用，也没有发现 TIM8 更新中断中的项目业务逻辑。
- TIM6 虽然初始化并被清零计数器，但当前源码没有发现启动调用，也没有发现它替代 SysTick 作为主循环节拍。

因此，TIM8 和 TIM6 的当前教材定位不是“主电机输出路径”，而是“已配置、需识别、后续调试和扩展时必须避免误判的辅助资源”。

## 8. 代码分析

### 1. `.ioc` 中的 TIM8/TIM6 证据

`.ioc` 记录 TIM8 配置了 CH1/CH2/CH3 的 PWM Generation，PC6/PC7/PC8 分别对应 TIM8_CH1、TIM8_CH2、TIM8_CH3。它还记录 `NVIC.TIM8_UP_IRQn=true...`，说明 TIM8 更新中断被启用。

`.ioc` 记录 TIM6 的虚拟时钟源为 Enable_Timer，Prescaler 为 35。TIM6 的 Period 在 `tim.c` 中为 65535。

### 2. `MX_TIM8_Init()`

`MX_TIM8_Init()` 同时初始化基础计数、PWM、主从同步和 Break/DeadTime。它的输入来自 CubeMX 配置，输出是 `htim8` 句柄和 TIM8 相关寄存器被配置。

风险点是：初始化不等于输出已经运行。项目没有发现 `HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_x)`，所以不能把 TIM8 CH1/CH2/CH3 当成当前主电机输出证据。

### 3. TIM8 MSP 与中断

TIM8 的 MSP 初始化启用 TIM8 时钟，并配置 `TIM8_UP_IRQn`。`stm32f1xx_it.c` 中的 `TIM8_UP_IRQHandler()` 调用 `HAL_TIM_IRQHandler(&htim8)`。

这条路径说明：如果 TIM8 更新事件和中断源被启动，HAL 有入口处理它。但当前用户代码没有实现定时器周期回调，也没有在中断函数用户区写业务逻辑，因此不能把 TIM8 中断写成 500Hz 控制触发源。

### 4. `MX_TIM6_Init()`

`MX_TIM6_Init()` 只做基础计数配置，不涉及 PWM 通道。它的项目意义是建立一个基本定时器资源。

当前配置下，TIM6 的计数频率可由 APB1 定时器频率和 Prescaler 推出。可是没有启动证据时，这个频率只能说明“如果启动会按该配置计数”，不能说明当前已经运行。

### 5. `main.c` 中的 TIM6 清零

`main.c` 在 `systemReady = true` 之后调用 `__HAL_TIM_SET_COUNTER(&htim6, 0)`。这会把 TIM6 的计数器值写为 0。

这条语句说明项目作者可能为后续计时或调试预留 TIM6，但当前没有看到后续读取 TIM6 计数值或启动 TIM6 的证据。因此教材必须保持克制，只写“清零计数器”这一事实。

## 9. 调试方法

调试 TIM8/TIM6 时，第一步不是看波形或猜测用途，而是区分配置状态。

TIM8 检查点：

- `.ioc` 中 TIM8 CH1/CH2/CH3 是否配置为 PWM Generation。
- `tim.c` 中 Prescaler、Period、Pulse、Break/DeadTime 是否与 `.ioc` 一致。
- `HAL_TIM_Base_MspInit()` 是否启用 `TIM8_UP_IRQn`。
- `stm32f1xx_it.c` 是否存在 `TIM8_UP_IRQHandler()`。
- 项目中是否存在 `HAL_TIM_PWM_Start(&htim8, ...)` 或业务回调。当前复查结果是未发现。

TIM6 检查点：

- `MX_TIM6_Init()` 是否配置 Prescaler 35、Period 65535。
- `main.c` 是否调用 `MX_TIM6_Init()`。
- `main.c` 是否调用 `__HAL_TIM_SET_COUNTER(&htim6, 0)`。
- 项目中是否存在 `HAL_TIM_Base_Start(&htim6)`、`HAL_TIM_Base_Start_IT(&htim6)` 或读取 TIM6 计数值。当前复查结果是未发现。

常见定位结论：

- 如果想确认项目主 PWM 输出，不应从 TIM8 开始，而应回到第12章的 TIM2/TIM3/TIM4 和 `PWM_Motor_SetAngle()`。
- 如果想确认项目主周期，不应把 TIM6 当作默认答案，而应回到第09章的 SysTick 和 `frame_500Hz`。
- 如果未来要启用 TIM8 或 TIM6，需要新增启动、回调或读取证据，并同步更新知识链。

## 10. 常见问题

### 1. TIM8 配置了 PWM，为什么不算当前主 PWM 输出？

触发条件：读者在 `.ioc` 或 `tim.c` 中看到 TIM8 CH1/CH2/CH3。

可能原因：配置存在不等于运行时启动。当前主 PWM 输出证据是 TIM2/TIM3/TIM4 被 `HAL_TIM_PWM_Start()` 启动，并被 `PWM_Motor_SetAngle()` 写 CCR；TIM8 没有同类启动证据。

### 2. TIM8 更新中断启用了，为什么不算控制周期来源？

触发条件：读者看到 `TIM8_UP_IRQHandler()`。

可能原因：中断入口存在，但当前用户代码中没有 TIM8 业务逻辑，也没有发现用户实现的 `HAL_TIM_PeriodElapsedCallback()`。第09章已经确认 500Hz 标志来自 SysTick。

### 3. TIM6 被清零，为什么不算正在计时？

触发条件：读者看到 `__HAL_TIM_SET_COUNTER(&htim6, 0)`。

可能原因：清零只是写计数器值。当前没有发现 TIM6 启动调用或读取调用，因此不能证明它正在承担业务计时。

### 4. TIM8 的 Break/DeadTime 配置有什么意义？

触发条件：读者看到 `HAL_TIMEx_ConfigBreakDeadTime()`。

可能原因：这是高级定时器配置能力的一部分。当前项目配置 DeadTime 为 0、BreakState 为 Disable，因此本章只把它作为 TIM8 高级定时器属性证据，不展开功率驱动保护策略。

### 5. 为什么第13章仍然重要？

触发条件：TIM8/TIM6 当前不是主路径，读者怀疑是否可以跳过。

可能原因：读懂“未使用或未启动的配置”是工程分析能力的一部分。移植、调试或扩展项目时，错误地把 TIM8/TIM6 当成主路径，会导致定位方向偏移。

## 11. 实践任务

1. 在 `.ioc` 中找出 TIM8 CH1/CH2/CH3、PC6/PC7/PC8 和 TIM8 更新中断配置。验收依据是能说明 TIM8 的配置项有哪些。
2. 在 `tim.c` 中找出 `MX_TIM8_Init()` 的 Prescaler、Period、Pulse 和 Break/DeadTime 设置。验收依据是能说明 TIM8 约 18kHz 的配置依据。
3. 在 `stm32f1xx_it.c` 中找到 `TIM8_UP_IRQHandler()`。验收依据是能说明它只调用 `HAL_TIM_IRQHandler(&htim8)`，当前没有用户业务逻辑。
4. 在 `tim.c` 和 `main.c` 中追踪 TIM6。验收依据是能说明 TIM6 被初始化并清零，但未发现启动和读取证据。
5. 对比第12章和第13章。验收依据是能说清 TIM2/TIM3/TIM4 与 TIM8/TIM6 在当前项目中的不同地位。

## 12. 思考题

1. 为什么“CubeMX 配置了某个外设”不等于“项目运行时正在使用它”？
2. 如果未来要让 TIM8 承担一个周期任务，至少还需要补充哪些代码证据？
3. 如果未来要用 TIM6 做辅助计时，为什么只清零计数器还不够？
4. TIM8 的高级定时器能力和第12章通用 PWM 输出有什么相同点与不同点？
5. 为什么教材要明确标出当前未发现的启动和业务回调证据？

## 13. 本章总结

本章建立了 TIM8 与 TIM6 在当前三轴云台项目中的证据边界。

已经确认的结论是：

- TIM8 配置了 PWM CH1/CH2/CH3、PC6/PC7/PC8、Prescaler 3、Period 999、Pulse 500。
- TIM8 配置了 Break/DeadTime 结构，并启用了 `TIM8_UP_IRQn`。
- `TIM8_UP_IRQHandler()` 存在，但当前用户代码未发现 TIM8 业务逻辑或用户周期回调。
- 当前工程未发现 `HAL_TIM_PWM_Start(&htim8, ...)`，因此不能把 TIM8 写成主 PWM 输出路径。
- TIM6 配置了 Prescaler 35、Period 65535，并在 `main.c` 中被清零计数器。
- 当前工程未发现 TIM6 启动或读取证据，因此不能把 TIM6 写成主时间基准。

下一章可以进入 I2C 主机通信。到这里，教材已经完成平台、时钟、中断、调试输出和定时器输出基础；后续可以开始分析 MPU6050 传感器如何通过 I2C 进入项目数据流。

---

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

- 高级定时器TIM8
- 基本定时器TIM6

项目证据：

- `Core/Src/tim.c`
- `Core/Src/stm32f1xx_it.c`
- `Core/Src/main.c`
- `Three-axis_cloud_platformV2.ioc`

引用的函数、配置项和变量：

- `MX_TIM8_Init()`
- `MX_TIM6_Init()`
- `HAL_TIM_PWM_Init()`
- `HAL_TIM_PWM_ConfigChannel()`
- `HAL_TIMEx_ConfigBreakDeadTime()`
- `HAL_TIM_Base_MspInit()`
- `HAL_TIM_MspPostInit()`
- `TIM8_UP_IRQHandler()`
- `HAL_TIM_IRQHandler()`
- `__HAL_TIM_SET_COUNTER()`
- `htim8`
- `htim6`
- `TIM8_UP_IRQn`
- `TIM8.Period=999`
- `TIM8.Prescaler=3`
- `TIM6.Prescaler=35`
- `PC6.Signal=S_TIM8_CH1`
- `PC7.Signal=S_TIM8_CH2`
- `PC8.Signal=S_TIM8_CH3`

质量自检：

- P0 事实错误：通过。
- P1 依赖断层：通过。
- P2 逻辑连贯：通过。
- P3 项目证据：通过。
- P4 原理展开：通过。
- P5 调试实践：通过。
- P6 表达统一：通过。
