# 第09章 中断系统与SysTick节拍

> 导航：上一章：[第08章_AFIO重映射与SWD调试](第08章_AFIO重映射与SWD调试.md) ｜ 下一章：[第10章_DWT计时与微秒时间基准](第10章_DWT计时与微秒时间基准.md)

## 1. 本章目标

- 理解 NVIC 在 Cortex-M3 中断入口、优先级和外设中断使能中的作用。
- 看懂本项目中 SysTick 1ms 节拍、HAL 时基和 500Hz 实时控制循环触发标志之间的关系。
- 能从启动文件、`.ioc`、HAL 源码和项目中断处理文件中追踪中断配置证据。
- 为后续 DWT 微秒计时、500Hz 实时控制循环、TIM8 配置和 USB 章节建立前置基础。

本章阅读分层：

| 阅读层次 | 建议范围 | 适合读者 |
|---|---|---|
| 【必须掌握】 | 第1节到第5节，第13节总结 | 需要理解中断入口、NVIC、SysTick 1ms节拍和 `frame_500Hz` 主线的读者 |
| 【工程深化】 | 第6节到第8.10.1节，第9节调试方法 | 需要维护优先级分组、HAL tick、SysTick配置、TIM8/USB中断边界和主循环消费逻辑的读者 |
| 【拓展阅读】 | 第6.1节，第8.9节到第8.10.1节，第10节到第12节 | 需要进一步理解优先级编码、系统异常/外设IRQ差异、布尔帧标志漏消费风险和构建证据读法的读者 |
| 【证据与验证】 | 第8节、第9节、章节尾部固定检查，以及所有 `【待验证】` 项 | 需要审查启动入口、HAL默认时基、SysTick配置、HAL tick、`frame_500Hz`、主循环消费、TIM8/USB中断边界、`.map/.list`、pending/active状态、DWT/GPIO计时、断点命中或日志证据的读者 |

如果只是沿实时节拍主线学习，可以先抓住“SysTick 1ms -> `HAL_IncTick()` -> `frameCounter` -> 每2个tick置位 `frame_500Hz` -> 主循环消费”这条链；排查抢占、漏帧或外设中断实际触发时，再回到优先级、构建证据和调试方法小节。

## 2. 前置知识

- Cortex-M3内核启动
- CMSIS寄存器与内核访问
- 系统时钟树

第03章已经说明启动文件中的中断向量表，第05章已经建立 CMSIS 和内核资源访问索引，第06章已经确认项目系统时钟来源。本章在这些基础上先讲 NVIC，再讲 SysTick 节拍，最后把 1ms 中断、500Hz 实时控制循环触发标志和 10Hz 低频任务接到项目主循环。

本章不展开 TIM8 的 PWM 配置、不展开 USB 中间件处理流程、不展开 DWT 微秒计时细节，也不展开姿态解算和电机控制算法。

## 3. 问题背景

三轴云台项目不是只按顺序执行一次初始化代码。初始化完成后，系统需要不断读取传感器、更新姿态、计算控制量并输出电机指令。这样的系统必须回答两个基础问题：

- 哪些函数会在中断发生时被硬件自动调用。
- 项目用什么时间基准把周期任务变成稳定的执行节奏。

本项目中，`HAL_Init()` 建立 HAL 默认 1ms 时基。

`SystemClock_Config()` 内部的 `HAL_RCC_ClockConfig()` 会在系统时钟切换后重新适配 HAL tick。

`main.c` 又在用户初始化段调用 `HAL_SYSTICK_Config(SystemCoreClock / 1000)` 和 `HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0)`，明确把 SysTick 配置为 1ms 节拍。

`SysTick_Handler()` 每次进入时调用 `HAL_IncTick()`。

它还会在系统就绪且 MPU6050 不处于校准状态时递增 `frameCounter`。当 `frameCounter % COUNT_500HZ == 0` 时，代码置位 `frame_500Hz`，主循环看到该标志后进入 500Hz 实时控制循环。

同时，`.ioc` 和生成代码中还存在 TIM8 更新中断、USB 低优先级中断等入口。第09章要建立的是“中断系统怎么接入项目”和“SysTick 如何成为项目节拍基础”，而不是把所有外设中断细节一次讲完。

## 4. 核心概念

- NVIC中断配置：Cortex-M3 中用于管理中断优先级、使能状态和中断入口的配置机制。
- SysTick系统节拍：由 Cortex-M3 内核 SysTick 定时器产生的周期性节拍，本项目配置为 1ms。
- 中断向量表：启动文件中保存异常和外设中断入口函数地址的表。
- 中断处理函数：中断发生后被调用的函数，例如 `SysTick_Handler()` 和 `TIM8_UP_IRQHandler()`。
- 弱默认处理函数：启动文件中用弱别名把未实现的中断入口指向 `Default_Handler`，项目同名函数会覆盖弱默认入口。
- 中断分发：外设 IRQ 入口通常先进入 HAL 分发函数，再由 HAL 根据外设标志和回调关系转到具体业务逻辑。
- HAL 时基：HAL 内部用于 `HAL_GetTick()`、`HAL_Delay()` 等时间函数的毫秒计数基础。
- 500Hz 实时控制循环触发标志：项目中由 `SysTick_Handler()` 根据 `COUNT_500HZ` 置位的 `frame_500Hz`。
- 10Hz 低频任务：主循环中用 `HAL_GetTick() - last_print_tick >= 100` 判断的低频任务入口。
- 系统异常与外设 IRQ：SysTick 属于 Cortex-M 系统异常，TIM8 和 USB 属于外设 IRQ；它们都可设置优先级，但在 CMSIS 中写入的寄存器位置不同。
- 中断源使能：外设中断不仅需要 NVIC 使能，还需要外设侧打开对应中断源，例如定时器更新中断需要 `TIM_IT_UPDATE` 路径。

这些概念服务于正式知识点 `NVIC中断配置` 和 `SysTick系统节拍`，不新增结构外知识点。

## 5. 工作原理

中断系统可以分成三层理解。

第一层是入口。启动文件中的中断向量表把中断号和处理函数名称对应起来。第03章已经讲过复位后如何进入 `main()`，本章补上另一条路径：当 SysTick、TIM8 或 USB 相关中断发生时，CPU 不通过普通函数调用进入处理函数，而是根据向量表跳到对应入口。

第二层是使能和优先级。仅有处理函数名称还不够，外设中断还需要在 NVIC 中配置优先级并使能。

本项目中，`HAL_Init()` 调用 `HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4)` 设置优先级分组。

`.ioc` 记录 SysTick、TIM8 更新中断和 USB 低优先级中断的启用状态。

`tim.c` 为 TIM8 更新中断设置优先级 1 并使能；`usbd_conf.c` 为 USB 低优先级中断设置优先级 0 并使能。

这一步只说明中断入口具备被响应的配置条件。是否真的发生中断、是否清除了外设标志、是否触发业务状态变化，
还要继续查看外设状态寄存器、HAL 分发函数、回调函数和运行时记录。
因此，教材中要区分“配置存在”和“运行发生”。

还要区分系统异常和外设 IRQ。`SysTick_IRQn` 在 CMSIS 中是负数异常号，
其优先级写入 `SCB->SHP` 系统处理优先级寄存器；`TIM8_UP_IRQn`、
`USB_LP_CAN1_RX0_IRQn` 这类外设中断号为非负值，使能位写入 `NVIC->ISER`，
优先级写入 `NVIC->IP`。所以“设置中断优先级”不是只有一种寄存器落点。

第三层是处理链路。一个外设中断从工程配置到业务效果，至少包含以下可拆单元：

1. 向量表中存在对应入口。
2. 项目源码中用同名函数覆盖弱默认处理函数。
3. NVIC 设置优先级分组、抢占优先级和使能位。
4. 外设侧打开中断源，并在硬件事件到达时置位中断标志。
5. IRQHandler 入口调用 HAL 分发函数。
6. HAL 根据外设标志进入回调或清除标志。
7. 项目业务变量、队列、状态机或输出发生变化。

缺少第4到第7步证据时，只能说“中断配置链路存在”，不能说“业务中断已经按预期运行”。

第四层是节拍行为。SysTick 与普通外设中断不同，它是 Cortex-M3 内核自带的系统节拍定时器。

项目把 SysTick 配置成 1ms 一次中断。每次进入 `SysTick_Handler()`，先调用 `HAL_IncTick()` 更新 HAL 毫秒计数。

随后项目逻辑在系统就绪时更新 `frameCounter`，并每 2 个 1ms tick 置位一次 `frame_500Hz`。

因此，本项目的主节拍链路是：

1. `SystemCoreClock` 来自系统时钟配置。
2. `HAL_RCC_ClockConfig()` 在时钟切换后重新适配 HAL tick。
3. `HAL_SYSTICK_Config(SystemCoreClock / 1000)` 在用户初始化段明确建立 1ms SysTick。
4. `SysTick_Handler()` 每 1ms 进入一次。
5. `HAL_IncTick()` 推进 HAL 毫秒计数。
6. `frameCounter` 按 1ms 累加。
7. `COUNT_500HZ` 为 2，因此每 2ms 置位 `frame_500Hz`。
8. `main()` 主循环消费 `frame_500Hz`，进入后续 500Hz 实时控制路径。

对应的数学换算为：

```text
T_systick = 1 ms = 0.001 s
COUNT_500HZ = 2
T_frame = 2 * T_systick = 2 ms = 0.002 s
f_frame = 1 / T_frame = 1 / 0.002 s = 500 Hz
```

这个结果表示调度意图和标志置位节奏，不等同于已经测得控制环稳定运行在 500Hz。
主循环耗时、中断屏蔽、同优先级中断竞争、I2C 读数耗时和调试断点都会影响实际帧间隔。

10Hz 低频任务没有单独使用一个中断，而是在主循环中用 `HAL_GetTick()` 做时间差判断。这样，项目把 500Hz 实时控制循环触发和 10Hz 低频任务都建立在同一个 1ms HAL/SysTick 时间基础上。

## 6. STM32实现机制

在 STM32F1 HAL 工程中，本章相关机制分布在以下位置：

- 启动文件提供中断向量表和弱默认处理函数。
- `HAL_Init()` 配置 NVIC 优先级分组，并通过 `HAL_InitTick()` 建立默认 SysTick 时基。
- `HAL_RCC_ClockConfig()` 在系统时钟切换后再次调用 tick 初始化入口，使 HAL 时基适配新的核心时钟。
- `HAL_SYSTICK_Config()` 最终调用 CMSIS 的 `SysTick_Config()`。
- `HAL_NVIC_SetPriority()` 和 `HAL_NVIC_EnableIRQ()` 封装 CMSIS 的 NVIC 配置入口。
- `stm32f1xx_it.c` 承载项目实际的中断处理函数。
- `main.c` 在用户初始化段中再次明确配置 SysTick 1ms 周期和 SysTick 优先级。

从寄存器层看，`HAL_SYSTICK_Config()` 最终进入 CMSIS 的 `SysTick_Config(ticks)`。

`SysTick_Config()` 做了四件关键事情：

- 先检查 `ticks - 1` 是否超过 `SysTick_LOAD_RELOAD_Msk`。`LOAD` 是 24 位重装载寄存器，上限为 `0xFFFFFF`。
- 写入 `SysTick->LOAD = ticks - 1`，使计数器从该重装载值向下计数。
- 写入 `SysTick->VAL = 0`，清当前计数值，让新的周期配置立即从干净状态开始。
- 写入 `SysTick->CTRL` 的 `CLKSOURCE`、`TICKINT`、`ENABLE` 位，选择 HCLK、打开中断并启动计数器。

在当前 `.ioc` 中，系统时钟目标值为 72MHz。若 `SystemCoreClock` 已按该时钟更新，
`HAL_SYSTICK_Config(SystemCoreClock / 1000)` 的参数就是 `72000000 / 1000 = 72000`，
于是 `SysTick->LOAD` 期望值为 `72000 - 1 = 71999`。`71999` 小于 24 位上限，
因此从寄存器约束看，该 1ms 配置是可表达的。

24 位 reload 的上限也给出了可验证边界：

```text
SysTick_LOAD_RELOAD_Msk = 0x00FF_FFFF = 16777215
最大 ticks = 16777215 + 1 = 16777216
在 72MHz HCLK 下，最大周期约为 16777216 / 72000000 = 0.233 s
项目 1ms ticks = 72000，远小于上限
```

因此，本项目的 1ms SysTick 不是受 24 位重装载寄存器限制的极限配置。

项目中 SysTick 会经历多次配置阶段：

1. `HAL_Init()` 在复位后的默认时钟状态下建立 HAL 初始 tick。
2. `HAL_RCC_ClockConfig()` 切换系统时钟后再次调用 tick 初始化，使 tick 适配新核心时钟。
3. `main.c` 用户初始化段再次调用 `HAL_SYSTICK_Config(SystemCoreClock / 1000)` 明确配置 1ms。
4. `main.c` 随后调用 `HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0)`，覆盖 CMSIS 默认最低优先级。

这种多阶段配置不是重复概念，而是 STM32 HAL 工程的时钟切换现实：
时基参数依赖 `SystemCoreClock`，而 `SystemCoreClock` 会在时钟树配置后更新。

项目中可以看到三类中断入口。

第一类是内核异常和系统异常，例如 `NMI_Handler()`、`HardFault_Handler()`、`SysTick_Handler()`。这些入口在 `stm32f1xx_it.c` 中实现，其中本章重点是 `SysTick_Handler()`。

第二类是外设中断入口，例如 `TIM8_UP_IRQHandler()`。本章只确认它由 `tim.c` 配置优先级和使能，并在处理函数中进入 HAL TIM 分发，具体 TIM8 用途留到第13章。

第三类是 USB 低优先级中断入口。本章只确认它由 `usbd_conf.c` 配置优先级和使能，并在处理函数中进入 HAL PCD 分发，USB 设备和 CDC 细节留到第15章和第16章。

当前优先级关系要按“数值越小，抢占优先级越高”理解。项目使用 `NVIC_PRIORITYGROUP_4`，
表示 4 位都用于抢占优先级，子优先级为 0 位。当前可见配置中：

- SysTick 在 `main.c` 中被设为优先级 0。
- USB 低优先级中断在 `usbd_conf.c` 中被设为优先级 0。
- TIM8 更新中断在 `tim.c` 中被设为优先级 1。

这说明 TIM8 更新中断的抢占优先级低于 SysTick 和 USB 低优先级中断。
但 SysTick 和 USB 低优先级中断同为 0 时，不能仅凭数值断言哪一路总能先完成。
实际响应还受当前执行状态、异常入口时刻、全局中断屏蔽、处理函数长度和硬件 pending 状态影响。

从 CMSIS 写寄存器方式看，STM32F103 设备头文件定义 `__NVIC_PRIO_BITS = 4`。
因此优先级值会左移 `8 - 4 = 4` 位后写入 8 bit 优先级字段：

```text
priority 0  -> 0x00
priority 1  -> 0x10
priority 15 -> 0xF0
```

`SysTick_IRQn` 作为系统异常进入 `SCB->SHP`；`TIM8_UP_IRQn` 和
`USB_LP_CAN1_RX0_IRQn` 作为外设 IRQ 进入 `NVIC->IP`，并通过
`NVIC->ISER` 使能。这个区别对调试寄存器窗口很重要。

### 6.1 NVIC优先级编码链路

还要把“调用 HAL 设置优先级”继续拆成更小的可验证单元。`HAL_NVIC_SetPriority(IRQn, PreemptPriority, SubPriority)` 本身不直接写最终寄存器值，它先读取当前 `SCB->AIRCR[10:8]` 中的优先级分组，再调用 CMSIS 的 `NVIC_EncodePriority()` 把抢占优先级和子优先级编码成一个逻辑优先级值，最后交给 `NVIC_SetPriority()` 写入系统异常或外设 IRQ 的优先级字段。

当前项目的分组来自 `HAL_Init()` 中的 `HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4)`。HAL 头文件说明该分组对应 4 位抢占优先级和 0 位子优先级；STM32F103 设备头文件又定义 `__NVIC_PRIO_BITS = 4U`。把这两个事实代入 `NVIC_EncodePriority()`：

```text
PriorityGroup = NVIC_PRIORITYGROUP_4 -> PRIGROUP = 3
PreemptPriorityBits = min(7 - 3, 4) = 4
SubPriorityBits = max(3 + 4 - 7, 0) = 0
encoded = PreemptPriority & 0x0F
register_field = encoded << (8 - __NVIC_PRIO_BITS)
```

因此，在本项目当前分组下，`HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0)` 和
`HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0, 0)` 都编码为逻辑优先级 0，写入寄存器字段时为 `0x00`；`HAL_NVIC_SetPriority(TIM8_UP_IRQn, 1, 0)` 编码为逻辑优先级 1，写入寄存器字段时为 `0x10`。这里的 `SubPriority = 0` 不是“项目暂时不用子优先级”的经验判断，而是在 `NVIC_PRIORITYGROUP_4` 下没有子优先级位可供编码。

`Debug/Three-axis_cloud_platformV2.list` 也能从构建结果侧证明这条软件路径进入了当前镜像：`HAL_NVIC_SetPriority()` 内部调用 `__NVIC_GetPriorityGrouping()`，再调用 `NVIC_EncodePriority()`，随后调用 `__NVIC_SetPriority()`。这类证据能说明“优先级值如何被编码并写入哪个寄存器族”，但不能证明运行时某个 IRQ 已经 pending、active 或完成抢占；这些状态仍需 `SCB->SHP`、`NVIC->IP/ISER/ISPR/IABR` 寄存器读回、断点命中或日志证据【待验证】。

这个拆分还能避免一个常见误解：同为优先级 0 的 SysTick 和 USB 低优先级中断，不等于二者有固定的先后执行顺序。它只能说明在当前分组下二者没有可见的抢占优先级差异；实际先后还取决于异常到达时间、当前正在执行的异常、全局中断屏蔽、外设 pending 状态、处理函数长度和 Cortex-M 异常仲裁过程。本章没有运行时轨迹，因此不能把“同优先级”写成“确定顺序”。

## 7. 项目中的应用

本章对应项目初始化和主循环之间的时间/中断基础层。

直接相关文件：

- `Core/Startup/startup_stm32f103rctx.s`
- `Core/Src/stm32f1xx_it.c`
- `Core/Src/main.c`
- `Core/Inc/main.h`
- `Core/Src/tim.c`
- `USB_DEVICE/Target/usbd_conf.c`
- `Three-axis_cloud_platformV2.ioc`
- `Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c`
- `Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c`
- `Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c`

文件之间的关系是：

- 启动文件提供中断向量表，决定中断名称如何落到处理函数。
- `.ioc` 记录 SysTick、TIM8 更新中断和 USB 低优先级中断的配置意图。
- HAL 源码说明 `HAL_Init()`、`HAL_InitTick()`、`HAL_RCC_ClockConfig()`、`HAL_SYSTICK_Config()` 和 NVIC API 如何工作。
- `stm32f1xx_it.c` 放置项目中的中断处理函数。
- `main.h` 定义 `FRAME_COUNT`、`COUNT_500HZ` 和 `frame_500Hz` 等节拍相关符号。
- `main.c` 配置 SysTick，并在主循环中消费 `frame_500Hz` 和 `HAL_GetTick()`。
- `tim.c` 与 `usbd_conf.c` 确认外设中断优先级和使能配置存在，但本章不展开外设内部逻辑。

在项目主流程中，初始化阶段先经过 `HAL_Init()`。

随后 `SystemClock_Config()` 通过 `HAL_RCC_ClockConfig()` 切换系统时钟并适配 HAL tick，再初始化外设。

用户初始化段再次配置 SysTick 1ms 周期，设置 SysTick 优先级，并在系统准备完成后把 `systemReady` 置为 `true`。

此后，`SysTick_Handler()` 才会在条件满足时推进项目帧计数并产生 `frame_500Hz`。

## 8. 代码分析

### 8.1 启动文件中的中断入口

`Core/Startup/startup_stm32f103rctx.s` 中的向量表登记了 `SysTick_Handler`、TIM8 更新中断入口和 USB 低优先级中断入口。文件后半部分还为这些处理函数提供弱别名，若项目源码中没有同名函数，就会落到默认死循环处理。

本项目在 `Core/Src/stm32f1xx_it.c` 中实现了这些同名处理函数，因此中断发生时会进入项目实现，而不是默认处理。

### 8.2 `HAL_Init()` 中的默认时基

`Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c` 中，`HAL_Init()` 会先调用 `HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4)`。

随后它调用 `HAL_InitTick(TICK_INT_PRIORITY)` 建立 HAL 默认时基。

`HAL_InitTick()` 默认使用 `HAL_SYSTICK_Config(SystemCoreClock / (1000U / uwTickFreq))`
建立 1ms 时基，并设置 SysTick 优先级。这里说明 HAL 本身依赖 SysTick 作为默认毫秒时间来源。

HAL 源码中 `uwTick` 声明为 `__IO uint32_t`，`HAL_IncTick()` 按
`uwTickFreq` 递增它，`HAL_GetTick()` 返回它。当前配置下
`uwTickFreq = HAL_TICK_FREQ_DEFAULT`，即 1kHz 毫秒时基。这个细节说明
HAL 的毫秒时间不是从 `frameCounter` 来的，而是从独立的 `uwTick` 来的。

### 8.3 `HAL_RCC_ClockConfig()` 中的 tick 适配

`Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c` 中，`HAL_RCC_ClockConfig()` 在更新 `SystemCoreClock` 后调用 `HAL_InitTick(uwTickPrio)`。

这说明系统时钟切换后，HAL 会按新的核心时钟重新适配 SysTick 时基。

本章只使用这个结论解释 SysTick 与系统时钟的关系。`HAL_RCC_ClockConfig()` 的时钟树细节已经在第06章说明。

### 8.4 `main.c` 中的 SysTick 配置

`Core/Src/main.c` 在外设初始化之后调用：

- `HAL_SYSTICK_Config(SystemCoreClock / 1000)`
- `HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0)`

这说明项目显式把 SysTick 配置为 1ms，并将 SysTick 优先级设置为 0。第06章已经说明 `SystemCoreClock` 的来源，本章只使用这个时钟结果，不重新展开时钟树。

这里的顺序有工程含义：CMSIS `SysTick_Config()` 会先把 SysTick 优先级设置为
`(1 << __NVIC_PRIO_BITS) - 1`，在本项目中就是 15，也就是最低优先级。
所以 `main.c` 紧接着调用 `HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0)`，
把 SysTick 调整回项目希望的优先级 0。

### 8.5 `SysTick_Handler()` 中的 HAL tick

`Core/Src/stm32f1xx_it.c` 的 `SysTick_Handler()` 先调用 `HAL_IncTick()`。这是 `HAL_GetTick()` 和 `HAL_Delay()` 等 HAL 时间函数能持续工作的基础。

如果这一步被删除或长时间阻塞，主循环中的 10Hz 判断 `HAL_GetTick() - last_print_tick >= 100` 就会失去可靠时间来源。

还要把 `HAL_IncTick()` 和 `HAL_SYSTICK_IRQHandler()` 分开看。HAL 源码中的
`HAL_SYSTICK_IRQHandler()` 只调用 `HAL_SYSTICK_Callback()`；而
`HAL_SYSTICK_Callback()` 是弱定义空函数，只有用户工程实现同名函数时才会产生项目行为。
当前 `Core/Src/stm32f1xx_it.c` 的 `SysTick_Handler()` 没有调用
`HAL_SYSTICK_IRQHandler()`，仓库内也没有用户实现的 `HAL_SYSTICK_Callback()`。
因此当前项目的 HAL 毫秒时基来自 `HAL_IncTick()`，不是来自
`HAL_SYSTICK_IRQHandler()`；若未来有人把周期任务放进 `HAL_SYSTICK_Callback()`，
就必须同步修改 `SysTick_Handler()` 调用链，否则该 callback 不会运行。

### 8.6 `SysTick_Handler()` 中的 500Hz 实时控制循环触发标志

`SysTick_Handler()` 在 `systemReady == true` 且 `mpu6050Calibrating == false` 时递增 `frameCounter`。

当 `frameCounter > FRAME_COUNT` 时，代码把计数器回绕到 1。`main.h` 中 `FRAME_COUNT` 为 1000，`COUNT_500HZ` 为 2。

当 `frameCounter % COUNT_500HZ == 0` 时，处理函数置位 `frame_500Hz`。由于 SysTick 是 1ms，`COUNT_500HZ` 为 2，所以该标志约每 2ms 置位一次，对应 500Hz。

项目源码中 `frame_500Hz` 附近有注释提到定时器中断，但当前实际置位位置在 `SysTick_Handler()`。教材以代码事实为准：本章阶段应把 500Hz 触发来源追踪到 SysTick，而不是 TIM8。

这个 ISR 也不是“完全只置位标志”。当前代码在条件满足时还调用 `micros()` 两次，
更新 `deltaTime1000Hz`、`previous1000HzTime` 和 `executionTime1000Hz`。
因此它比最小化节拍 ISR 稍重。源码注释中把 I2C 读取留在主循环，是为了避免在 SysTick 中执行更重的总线访问；
这种分工从工程结构上合理，但真实耗时和抖动改善仍需 DWT 记录或外部测量【待验证】。

还要注意，源码中曾经保留过 `sysTickCycleCounter` 和 `sysTickUptime` 的注释代码，
但当前有效代码没有递增 `sysTickUptime`。教材不能把注释块当成运行行为；
当前 HAL 毫秒计数以 `HAL_IncTick()` 更新的 `uwTick` 为准。

### 8.7 主循环中的 500Hz 消费

`Core/Src/main.c` 的主循环检查 `frame_500Hz`。如果标志为真，就先清除标志，再计算 `deltaTime500Hz` 和 `dt500Hz`，随后进入传感器读取、姿态更新和电机控制路径。

这些后续处理属于第26章和算法篇的内容。本章只确认：主循环不是靠延时函数固定等待 2ms，而是靠 SysTick 中断置位的标志驱动 500Hz 实时控制循环入口。

这里还要注意两个工程审查点。

第一，当前 `frame_500Hz` 在 `main.c` 和 `main.h` 中声明为普通 `bool`，未见 `volatile` 修饰。
它由 `SysTick_Handler()` 写入、由主循环读取并清除，属于中断上下文和主循环共享变量。
在优化等级、编译器行为和访问顺序变化下，普通共享标志可能带来可见性审查风险。
本章只记录该风险，不修改源码；是否需要改为 `volatile` 或引入更严格的同步策略，留给代码审查和实测验证。

同类审查也适用于 `frameCounter`、`deltaTime1000Hz`、`previous1000HzTime`
和 `executionTime1000Hz` 等由 SysTick ISR 更新的变量。它们当前用于项目内部节拍和耗时记录，
本章只提示共享上下文边界，不在文档阶段替代码下结论。

第二，`frame_500Hz` 是布尔标志，不是累计计数器。如果主循环某一轮处理时间超过 2ms，
SysTick 可能多次把它写成 `true`，但主循环下一次只会看到一个 `true`。
这会造成“多次调度请求合并成一次消费”的风险。项目用 `deltaTime500Hz = currentTime - previous500HzTime`
记录实际帧间隔，这能帮助发现滑帧或抖动，但不能阻止布尔标志合并本身。

### 8.8 主循环中的 10Hz 低频任务

主循环还使用 `HAL_GetTick() - last_print_tick >= 100` 判断 100ms 是否到达。100ms 对应 10Hz，因此这一路适合状态检查和串口打印等低频任务。

这个 10Hz 判断仍然依赖 `HAL_IncTick()` 维护的 HAL 毫秒计数。也就是说，500Hz 实时控制循环触发标志和 10Hz 判断共享同一个 1ms SysTick/HAL 时基。

### 8.9 TIM8 和 USB 中断边界

`Core/Src/tim.c` 中 TIM8 分支调用 `HAL_NVIC_SetPriority(TIM8_UP_IRQn, 1, 0)` 和 `HAL_NVIC_EnableIRQ(TIM8_UP_IRQn)`。

`stm32f1xx_it.c` 中对应处理函数调用 `HAL_TIM_IRQHandler(&htim8)`。

还要注意，`TIM8_UP_IRQn` 在 STM32F103 设备头文件中还有 `TIM8_UP_TIM13_IRQn`、`TIM13_IRQn` 别名。也就是说，IRQn 名称首先表达的是 NVIC 中断线编号和向量入口，不自动等价于唯一外设业务来源。当前项目调用的是 `TIM8_UP_IRQn`，处理函数名也是 `TIM8_UP_IRQHandler()`，所以本章只按 TIM8 更新入口记录证据；若在其它工程中启用 TIM13，则同一条 IRQ 线还需要结合外设 pending 标志和 HAL 句柄判断实际来源。

但当前仓库搜索不到 `HAL_TIM_Base_Start_IT(&htim8)`、`__HAL_TIM_ENABLE_IT(&htim8, TIM_IT_UPDATE)`
或项目自定义 `HAL_TIM_PeriodElapsedCallback()`。因此，第09章只能确认 TIM8
的 NVIC 入口和 HAL 分发入口存在，不能证明 TIM8 更新中断源已经在运行时打开，
也不能证明它已经产生业务回调。

`USB_DEVICE/Target/usbd_conf.c` 中 USB 低优先级中断分支调用
`HAL_NVIC_SetPriority(..., 0, 0)` 和 `HAL_NVIC_EnableIRQ(...)`。
`stm32f1xx_it.c` 中对应处理函数进入 HAL PCD 分发。

同理，`USB_LP_CAN1_RX0_IRQn` 在设备头文件中同时对应 USB Device Low Priority 和 CAN1 RX0，并定义了 `USB_LP_IRQn`、`CAN1_RX0_IRQn` 两个别名。当前项目的 MSP 配置和处理函数进入的是 USB PCD 分发路径，因此教材可以说项目配置了 USB 低优先级入口；但单看 IRQn 或向量名，不能推广成“这条中断线只可能来自 USB”。

这些证据说明项目确实存在外设中断配置。但第09章只建立 NVIC 配置方法和入口关系，TIM8 和 USB 的业务含义分别留到后续章节。

更严格地说，`HAL_TIM_IRQHandler(&htim8)` 和 `HAL_PCD_IRQHandler(&hpcd_USB_FS)` 都只是分发入口。
要证明它们产生了项目业务效果，还需要继续找到：

- 外设中断源是否打开。
- 对应状态标志是否在运行时置位。
- HAL 是否进入具体回调。
- 项目变量、缓冲区、通信状态或控制输出是否发生预期变化。

当前第09章不把这些分发入口误写成“TIM8 已经驱动 500Hz”或“USB 已经完成主机枚举”。
后一类结论必须等第13章、第15章、第16章结合更多证据再判断。

### 8.10 构建产物证据边界

当前 Debug 构建产物可以把中断/SysTick 链路从“源码中存在入口”推进到“入口和 HAL 分发函数进入当前镜像，并具有函数级静态资源条目”。这类证据适合确认向量入口、调用路径和静态资源规模，但仍不能证明中断真实触发频率、外设 pending 标志、主循环响应延迟或业务效果。

| 链路环节 | 函数或路径 | 静态栈估计 | 圈复杂度 | 证据文件 | 证据边界 |
|---|---|---:|---:|---|---|
| 用户SysTick配置 | `main()` 调用 `HAL_SYSTICK_Config(SystemCoreClock / 1000)` 和 `HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0)` | - | - | `Debug/Three-axis_cloud_platformV2.list` | 证明当前镜像保留了用户区 1ms SysTick 配置和优先级设置，不能证明板上节拍频率实测正确。 |
| SysTick入口 | `SysTick_Handler()` | 16 字节 | 5 | `Debug/Core/Src/stm32f1xx_it.su` / `.cyclo` | 证明 SysTick 处理函数进入构建，并包含 `HAL_IncTick()` 与项目帧标志路径。 |
| HAL毫秒时基 | `HAL_InitTick()` / `HAL_IncTick()` | 16 / 4 字节 | 3 / 1 | `Debug/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.su` / `.cyclo` | 证明 HAL 默认 tick 初始化和毫秒计数入口进入构建，不能证明 `uwTick` 在运行中没有被长时间阻塞。 |
| SysTick底层配置 | `SysTick_Config()` / `HAL_SYSTICK_Config()` | 16 / 16 字节 | 2 / 1 | `Debug/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.su` / `.cyclo` | 证明 CMSIS/HAL SysTick 配置函数进入构建，不能替代 `SysTick->LOAD/VAL/CTRL` 运行观察。 |
| NVIC配置入口 | `HAL_NVIC_SetPriorityGrouping()` / `HAL_NVIC_SetPriority()` / `HAL_NVIC_EnableIRQ()` | 16 / 32 / 16 字节 | 1 / 1 / 1 | `Debug/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.su` / `.cyclo` | 证明优先级分组、优先级和外设 IRQ 使能函数进入构建，不能证明具体 IRQ 已经 pending 或 active。 |
| USB外设IRQ入口 | `USB_LP_CAN1_RX0_IRQHandler()` -> `HAL_PCD_IRQHandler()` | 8 / 40 字节 | 1 / 12 | `Debug/Core/Src/stm32f1xx_it.su` / `.cyclo`; `Debug/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pcd.su` / `.cyclo` | 证明 USB 低优先级中断分发入口进入构建，不能证明 USB 主机枚举或端点传输成功。 |
| TIM8外设IRQ入口 | `TIM8_UP_IRQHandler()` -> `HAL_TIM_IRQHandler()` | 8 / 24 字节 | 1 / 21 | `Debug/Core/Src/stm32f1xx_it.su` / `.cyclo`; `Debug/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.su` / `.cyclo` | 证明 TIM8 更新中断分发入口进入构建，不能证明 TIM8 更新中断源已经启动或进入项目回调。 |

`.map/.list` 还能证明 `SysTick_Handler`、`USB_LP_CAN1_RX0_IRQHandler`、`TIM8_UP_IRQHandler`、`HAL_InitTick`、`HAL_IncTick`、`HAL_SYSTICK_Config`、`HAL_NVIC_SetPriority`、`HAL_NVIC_EnableIRQ`、`HAL_PCD_IRQHandler` 和 `HAL_TIM_IRQHandler` 等符号进入当前镜像。这个结论仍只属于构建证据：`.su/.cyclo` 不能简单相加为系统最大栈深，也不能换算成 ISR 真实耗时；中断实际频率、抢占延迟、pending/active 状态、`frame_500Hz` 丢触发风险和 USB/TIM8 业务效果仍需断点、寄存器窗口、DWT/GPIO 计时、日志或外部仪器证据【待验证】。

#### 8.10.1 `.map/.list` 最终地址与中断触发边界

当前 `Debug/Three-axis_cloud_platformV2.map` 中，`SysTick_Handler`、`USB_LP_CAN1_RX0_IRQHandler`、`TIM8_UP_IRQHandler`、`HAL_InitTick`、`HAL_IncTick`、`SysTick_Config`、`HAL_NVIC_SetPriority`、`HAL_NVIC_EnableIRQ`、`HAL_SYSTICK_Config`、`HAL_PCD_IRQHandler` 和 `HAL_TIM_IRQHandler` 都出现在 `.text` 最终地址区。这能证明这些入口和 HAL 分发函数进入当前 Flash 镜像，而不是只存在于源码文件或对象输入段。

`Debug/Three-axis_cloud_platformV2.list` 给出更细的调用路径：`main()` 调用 `HAL_SYSTICK_Config(SystemCoreClock / 1000)` 后又调用 `HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0)`；`SysTick_Handler()` 内部调用 `HAL_IncTick()`；`USB_LP_CAN1_RX0_IRQHandler()` 调用 `HAL_PCD_IRQHandler(&hpcd_USB_FS)`；`TIM8_UP_IRQHandler()` 调用 `HAL_TIM_IRQHandler(&htim8)`；`HAL_SYSTICK_Config()` 继续调用 CMSIS `SysTick_Config()`。这些路径能证明当前构建中断链路的静态闭合关系。

但是，最终地址和反汇编调用路径仍不能证明中断已经在板上真实触发，也不能证明 SysTick 频率就是 1kHz、TIM8 更新事件已经打开、USB 低优先级中断已经来自主机枚举，或 `frame_500Hz` 没有被主循环漏消费。运行时结论还需要 `SysTick->CTRL/LOAD/VAL`、`SCB->SHP`、`NVIC->ISER/IP`、外设 pending 标志、断点命中、DWT/GPIO 计时或日志证据；缺少这些证据时保持【待验证】。

### 8.11 本节证据边界

本节只根据当前仓库说明文件、函数、宏、变量、调用关系和 Debug 构建产物。运行时频率、外部硬件表现、主机侧现象、传感器方向、电机响应、真实栈水位、函数耗时或真实控制效果仍需调试记录、日志或仓库外实测证据；缺少证据时保持【待验证】。

## 9. 调试方法

本节按“现象 -> 可能原因 -> 定位方法 -> 验证步骤 -> 解决方案 -> 经验总结”组织。第09章调试的目标是确认中断入口、SysTick 节拍和主循环标志之间的链路闭合。

### 9.1 现象与可能原因

- 10Hz 低频任务不再触发：先检查 `HAL_IncTick()` 是否仍在 `SysTick_Handler()` 中执行，再检查 `HAL_GetTick()` 判断条件。
- 500Hz 实时控制循环不执行：先检查 `systemReady` 和 `mpu6050Calibrating`，再检查 `frameCounter` 和 `frame_500Hz`。
- 500Hz 周期明显不对：先确认 `SystemCoreClock` 和 `HAL_SYSTICK_Config(SystemCoreClock / 1000)`，再确认 `COUNT_500HZ` 是否仍为 2。
- TIM8 或 USB 中断处理异常：先确认 `.ioc` 和对应 MSP 初始化中的 NVIC 优先级/使能，再结合共享 IRQ 别名、外设 pending 标志和 HAL 句柄进入各自后续章节分析外设逻辑。
- TIM8_UP_IRQHandler 从未进入：除 NVIC 配置外，还要检查 TIM8 是否调用了启动更新中断的 API，例如 `HAL_TIM_Base_Start_IT()`，以及是否存在项目回调。
- 程序进入默认死循环：检查启动文件中向量名称和 `stm32f1xx_it.c` 中实际函数名称是否一致。
- 调试时周期看起来异常：先确认是否在 ISR 或主循环 500Hz 分支中打断点。断点和串口打印都会改变时序。

### 9.2 定位方法：链路观察对象

- `.ioc` 中 SysTick、TIM8 更新中断和 USB 低优先级中断是否被启用。
- `HAL_Init()` 是否仍在启动后执行。
- `HAL_RCC_ClockConfig()` 是否在系统时钟配置中完成并适配 HAL tick。
- `main.c` 是否调用 `HAL_SYSTICK_Config(SystemCoreClock / 1000)`。
- `main.c` 是否设置 `SysTick_IRQn` 优先级。
- `SysTick_Handler()` 是否调用 `HAL_IncTick()`。
- `SysTick_Handler()` 是否调用 `HAL_SYSTICK_IRQHandler()`；若没有，则不要假设 `HAL_SYSTICK_Callback()` 会执行。
- `SysTick_Handler()` 是否在系统就绪后递增 `frameCounter`。
- `frameCounter % COUNT_500HZ == 0` 时是否置位 `frame_500Hz`。
- 主循环是否及时清除并消费 `frame_500Hz`。
- 10Hz 低频任务是否依赖 `HAL_GetTick()` 的 100ms 差值。

### 9.3 验证步骤：寄存器与变量观察

- `SysTick->LOAD`：在 72MHz、1ms 配置下，期望看到 `71999`。
- `SysTick->VAL`：应在 0 到 `LOAD` 之间循环递减，单步调试时读数会被调试过程扰动。
- `SysTick->CTRL`：重点看 `CLKSOURCE`、`TICKINT`、`ENABLE` 是否有效。
- `SCB->SHP`：SysTick 这类系统异常的优先级落点，优先级 0 对应有效高 4 位为 0。
- `NVIC->ISER`：外设 IRQ 使能位落点，用于确认 TIM8/USB 等外设中断入口是否被 NVIC 接收。
- `NVIC->IP`：外设 IRQ 优先级落点，TIM8 priority 1 在 4 位实现中对应优先级字段 `0x10`。
- 共享 IRQ 别名：对 `USB_LP_CAN1_RX0_IRQn` 同时核对 `USB_LP_IRQn`、`CAN1_RX0_IRQn`，对 `TIM8_UP_IRQn` 同时核对 `TIM8_UP_TIM13_IRQn`、`TIM13_IRQn`，避免把同一 IRQ 线的别名误判为独立中断源。
- `uwTick`：由 `HAL_IncTick()` 推进，是 `HAL_GetTick()` 的返回基础。
- `frameCounter`：系统就绪且未校准时按 1ms 节奏递增，超过 `FRAME_COUNT` 后回绕。
- `frame_500Hz`：每两个 tick 被置为 `true`，由主循环消费并清零。
- `deltaTime500Hz`：用于观察主循环实际帧间隔，能暴露调度滑移。
- `executionTime1000Hz`：用于观察 SysTick ISR 内部记录到的执行耗时。

### 9.4 解决方案：调试记录

- 记录向量表入口、`.ioc` 中 NVIC 配置、实际中断函数名和项目标志变量，避免只凭现象判断中断是否运行。
- 对 SysTick 链路分别记录 1ms 节拍设置、`frame_500Hz` 置位位置和主循环消费位置。
- 对 TIM8 和 USB 中断只记录本章能确认的配置证据和入口证据，外设语义留到后续章节。
- 若缺少断点截图、串口日志或外部实测记录，应把“中断实际触发频率”和“任务周期稳定性”标记为【待验证】。

### 9.5 解决方案：中断入口与外设语义分开

本章只确认向量表、NVIC、IRQ 函数、SysTick 置位和主循环消费。TIM8 更新事件、USB 端点行为和主机侧表现需要在对应外设章节继续验证。

### 9.6 经验总结

中断调试不能只看向量表入口，也不能只看一次变量变化。需要同时记录入口是否存在、NVIC 是否使能、ISR 是否进入、标志是否置位、主循环是否消费，以及断点或日志对时序的影响；缺少连续记录时，周期稳定性保持【待验证】。

## 10. 常见问题

### 1. SysTick 是否就是项目的 500Hz 实时控制循环？

不是。SysTick 在本项目中是 1ms 节拍。
500Hz 实时控制入口来自 `SysTick_Handler()` 每 2 个 tick 置位一次 `frame_500Hz`，
主循环再消费该标志。
SysTick 是节拍来源，500Hz 实时控制循环是项目在主循环中基于标志组织出来的执行入口。

这里最重要的不是“500Hz 这个名字”，而是“谁在置位、谁在消费”。
当前仓库能证明 `frame_500Hz` 的置位点和消费点，但不能单靠变量名证明实时性已经通过外部测试。

### 2. 为什么 `HAL_IncTick()` 必须保留？

因为 `HAL_GetTick()` 的毫秒计数依赖它。
项目的 10Hz 低频任务使用 `HAL_GetTick() - last_print_tick >= 100` 判断时间差。
如果 `HAL_IncTick()` 不运行，低频任务和 HAL 延时都会受到影响。

这说明 SysTick 不只服务 500Hz 标志，也服务整个 HAL 毫秒时基。
因此断点、异常或重入问题一旦影响 `HAL_IncTick()`，
10Hz 输出和其他依赖 `HAL_GetTick()` 的路径都会跟着受影响。

但不能反过来把 `HAL_SYSTICK_IRQHandler()` 当成 HAL 毫秒计数的必经路径。
在当前 HAL 源码中，它只是进入 `HAL_SYSTICK_Callback()` 的分发入口。
当前项目没有调用它，也没有实现该 callback，所以教材不能写成“HAL SysTick callback 正在参与项目调度”。

### 3. 为什么中断里只置位 `frame_500Hz`，不直接读取 MPU6050？

项目源码中的注释已经说明，在中断中读 I2C 会消耗较多时间，还可能与主循环读取形成冲突。因此当前实现只在中断中设置标志，把较重的读取和控制计算留给主循环。

这是典型的“节拍中断只负责唤醒，主循环负责干活”的分工。
本章能证明这种分工存在，但不能证明中断里直接读传感器更差或更好，
需要结合运行时耗时观察和后续章节的调度链路继续看。

### 4. TIM8 已经配置了中断，为什么 500Hz 还说来自 SysTick？

因为当前代码中 `frame_500Hz` 的实际置位位置在 `SysTick_Handler()`，而 TIM8 中断处理函数只是进入 HAL TIM 分发。本章以当前源码事实为准。TIM8 的配置和用途留到第13章再分析。

换句话说，TIM8 具备中断入口，不等于当前主线用它做调度。
教材在这里刻意区分“配置存在”和“当前采用”，避免把后续章节的辅助定时器误读成实时控制主来源。

更严格地说，当前仓库还没有 `HAL_TIM_Base_Start_IT(&htim8)` 或项目级
`HAL_TIM_PeriodElapsedCallback()` 证据。因此不能把 `HAL_NVIC_EnableIRQ(TIM8_UP_IRQn)`
解释为“TIM8 更新中断已经在周期触发业务逻辑”。

### 5. SysTick 优先级 0 是否意味着一定不会被其他中断影响？

不能这样简单理解。优先级还要结合分组、具体中断优先级和当前执行状态判断。本章只确认项目中 SysTick 和 USB 低优先级中断配置为优先级 0，TIM8 更新中断配置为优先级 1；更复杂的抢占行为需要结合后续外设章节和实际调试观察。

因此，第09章能提供的是“配置结构”，不是绝对的时序保证。
如果后续调试中发现实时周期不稳，应该把 NVIC、主循环耗时和中断嵌套一起看，
而不是只改一个优先级数字。

### 6. 为什么已经有 HAL 默认 tick，`main.c` 还要再次配置 SysTick？

`HAL_Init()` 会在程序早期建立默认 HAL tick，`HAL_RCC_ClockConfig()` 又会在时钟切换后适配 tick。
这两步属于 HAL 框架层。`main.c` 用户初始化段再次调用 `HAL_SYSTICK_Config(SystemCoreClock / 1000)`，
相当于用当前 `SystemCoreClock` 明确声明项目期望的 1ms 节拍。

还要注意，CMSIS 的 `SysTick_Config()` 会先把 SysTick 优先级设到最低。
因此项目紧接着调用 `HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0)`，
把 SysTick 优先级调到当前工程设定的 0。

### 7. `frame_500Hz` 是布尔标志，会不会漏掉帧？

有这个审查风险。布尔标志只能表达“至少有一次请求发生”，不能表达“发生了几次请求”。
如果主循环长时间忙于 I2C、姿态解算、电机控制、打印或断点停顿，
多个 2ms 调度请求可能在主循环下一次读取前合并成同一个 `true`。

当前代码用 `deltaTime500Hz` 和 `dt500Hz` 记录实际帧间隔，这能让后续控制计算知道本帧距离上一帧过了多久。
但它不是调度补偿队列，也不会自动补跑漏掉的帧。是否需要计数型事件、超时降级或任务拆分，
要结合第26章的 500Hz 主循环耗时分析再判断。

## 11. 实践任务

开始任务前，先回到本章第8节定位 SysTick、`frame_500Hz`、NVIC 边界和 10Hz 低频任务入口；第9节提供节拍与中断调试顺序。

任务一至任务三属于中断入口和 HAL 时基定位；任务四至任务七属于项目节拍链路；任务八属于章节边界确认。

任务一：确认中断入口来自启动文件。

在启动文件中找到 `SysTick_Handler` 和 TIM8 更新中断入口。
验收依据是入口对照表包含向量表项、函数名、来源文件和调用边界。

任务二：确认 CubeMX 中断配置。

在 `.ioc` 中找到 NVIC 相关配置项和 SysTick 虚拟外设配置。
验收依据是配置对照表分列 `.ioc` 项、实际代码项和证据类型。

任务三：追踪 HAL Tick 初始化。

在 HAL 源码中找到 `HAL_Init()`、`HAL_InitTick()` 和 `HAL_SYSTICK_Config()`。
验收依据是时基初始化表记录调用顺序、函数名和毫秒节拍来源。

任务四：画出 500Hz 标志生成关系。

在 `stm32f1xx_it.c` 中找到 `SysTick_Handler()`，画出 `HAL_IncTick()`、`frameCounter`、`frame_500Hz` 的关系。
验收依据是关系图至少包含 `HAL_IncTick()`、计数变量、标志变量和中断职责边界。

任务五：确认 500Hz 常量定义。

在 `main.h` 中确认 `FRAME_COUNT` 和 `COUNT_500HZ` 的定义。
验收依据是常量表包含常量名、数值、单位、来源文件和周期换算结果。

任务六：确认主循环消费标志。

在 `main.c` 中找到主循环消费 `frame_500Hz` 的位置。
验收依据是主循环记录表包含轮询位置、消费条件和执行顺序。

任务七：定位 10Hz 低频任务的节拍来源。

在 `main.c` 中找到 10Hz 低频任务的 `HAL_GetTick()` 判断。
验收依据是低频任务表包含判断表达式、轮询位置和非中断结论。

任务八：说明后续中断边界。

对比 `tim.c` 和 `usbd_conf.c` 中的 NVIC 配置，说明它们为什么属于后续章节边界。
验收依据是边界表把 TIM8 和 USB 配置分到后续章节，并保留当前章节不展开的理由。

任务九：区分系统异常和外设 IRQ 的寄存器落点。

在 CMSIS 源码中找到 `__NVIC_SetPriority()` 对 `IRQn < 0` 和
`IRQn >= 0` 的分支，说明 SysTick 优先级为什么写 `SCB->SHP`，
TIM8/USB 优先级为什么写 `NVIC->IP`。
验收依据是记录表包含中断名、IRQn 类型、优先级寄存器、使能寄存器和本项目优先级数值。

实践边界：

当前任务优先形成表格、链路图、搜索记录和计算过程。涉及 IDE 现场、构建日志、断点数值、外部波形、主机侧结果或硬件响应时，若没有截图、日志或仓库外实测证据，结论保持【待验证】。

## 12. 思考题

1. 为什么第09章先讲 NVIC，再讲 SysTick？
2. 如果 `SysTick_Handler()` 中没有调用 `HAL_IncTick()`，哪些项目行为会首先受到影响？
3. `COUNT_500HZ` 为什么等于 2 时能从 1ms SysTick 得到约 500Hz 实时控制循环触发标志？
4. 为什么当前项目没有在 SysTick 中直接执行传感器读取和控制计算？
5. 如果 `systemReady` 一直为 `false`，`frame_500Hz` 会发生什么变化？
6. 为什么不能只看 `.ioc` 判断项目中断行为，还必须看 `stm32f1xx_it.c` 和外设 MSP 配置？
7. 如果一个中断在 `.ioc` 中启用，但没有业务状态变化证据，教材应如何描述它的当前边界？
8. 为什么 `HAL_NVIC_EnableIRQ(TIM8_UP_IRQn)` 不能单独证明 TIM8 更新中断源已经打开？
9. 为什么 SysTick 优先级要去看 `SCB->SHP`，而 TIM8/USB 要去看 `NVIC->IP`？

## 13. 本章总结

本章建立了三轴云台项目中 NVIC、中断入口、SysTick 1ms 节拍、500Hz 实时控制循环触发标志和 10Hz 低频任务之间的证据链。

已经确认的结论是：

- 启动文件中的中断向量表决定中断入口名称。
- `HAL_Init()` 设置 NVIC 优先级分组并建立默认 HAL 时基。
- `HAL_RCC_ClockConfig()` 在系统时钟切换后重新适配 HAL tick。
- `main.c` 明确调用 `HAL_SYSTICK_Config(SystemCoreClock / 1000)` 配置 1ms SysTick。
- `SysTick_Handler()` 调用 `HAL_IncTick()` 维护 HAL 毫秒计数。
- 当前 `SysTick_Handler()` 没有调用 `HAL_SYSTICK_IRQHandler()`，仓库内也没有 `HAL_SYSTICK_Callback()` 用户实现；因此 HAL SysTick callback 不是当前项目调度证据。
- `SysTick_Handler()` 通过 `frameCounter` 和 `COUNT_500HZ` 置位 `frame_500Hz`。
- 主循环消费 `frame_500Hz` 进入后续 500Hz 实时控制路径。
- 10Hz 低频任务使用 `HAL_GetTick()` 的 100ms 时间差判断。
- TIM8 和 USB 中断在项目中存在，但本章只建立 NVIC 入口和配置边界。
- 当前仓库没有证明 TIM8 更新中断源已经启动，也没有项目级 TIM 周期回调证据。
- SysTick 属于系统异常，优先级落在 `SCB->SHP`；TIM8/USB 属于外设 IRQ，优先级和使能分别落在 `NVIC->IP` 与 `NVIC->ISER`。
- `HAL_NVIC_SetPriority()` 需要经过优先级分组读取、`NVIC_EncodePriority()` 编码和 `__NVIC_SetPriority()` 写寄存器，不能把 HAL 调用名直接等同于最终寄存器字节。
- `.map/.list/.su/.cyclo` 的构建产物结论统一回到第8.10节到第8.10.1节判断：它们能证明 SysTick、USB、TIM8 中断入口、HAL 分发函数、静态栈和圈复杂度条目进入当前 Debug 构建，但不能替代中断真实触发频率、pending/active 状态、`frame_500Hz` 漏消费风险或 USB/TIM8 业务效果的运行时证据。

本章边界：

- 本章证明 SysTick 1ms 节拍与 `frame_500Hz` 置位关系，不展开 500Hz 分支中的传感器和控制计算。
- TIM8 与 USB 中断只记录配置和入口证据，是否承担业务逻辑需在对应章节继续判断。

下一章可以进入 DWT 计时与微秒时间基准，因为本章已经说明了 1ms SysTick 和 HAL 毫秒时间；后续需要解释项目如何用 `micros()` 和 DWT 周期计数获得更细的时间测量。

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

项目证据：

- `Core/Startup/startup_stm32f103rctx.s`
- `Core/Src/stm32f1xx_it.c`
- `Core/Src/main.c`
- `Core/Inc/main.h`
- `Core/Src/tim.c`
- `USB_DEVICE/Target/usbd_conf.c`
- `Three-axis_cloud_platformV2.ioc`
- `Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c`
- `Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c`
- `Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c`
- `Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c`
- `Drivers/CMSIS/Include/core_cm3.h`
- `Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xe.h`
- `Debug/Three-axis_cloud_platformV2.map`
- `Debug/Three-axis_cloud_platformV2.list`
- `Debug/Core/Src/stm32f1xx_it.su`
- `Debug/Core/Src/stm32f1xx_it.cyclo`
- `Debug/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.su`
- `Debug/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.cyclo`
- `Debug/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.su`
- `Debug/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.cyclo`
- `Debug/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.su`
- `Debug/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.cyclo`
- `Debug/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pcd.su`
- `Debug/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pcd.cyclo`

外部权威资料：

- ST RM0008 Reference manual: `https://www.st.com/resource/en/reference_manual/rm0008-stm32f101xx-stm32f102xx-stm32f103xx-stm32f105xx-and-stm32f107xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf`
- ST DS5792 Datasheet for STM32F103xC/xD/xE: `https://www.st.com/resource/en/datasheet/stm32f103rc.pdf`
- ST STM32F103 documentation page: `https://www.st.com/en/microcontrollers-microprocessors/stm32f103/documentation.html`

符号、函数与配置项证据：

- `HAL_Init()`
- `HAL_InitTick()`
- `HAL_RCC_ClockConfig()`
- `HAL_SYSTICK_Config()`
- `HAL_SYSTICK_IRQHandler()`
- `HAL_SYSTICK_Callback()`
- `HAL_NVIC_SetPriorityGrouping()`
- `HAL_NVIC_SetPriority()`
- `HAL_NVIC_EnableIRQ()`
- `NVIC_EncodePriority()`
- `__NVIC_GetPriorityGrouping()`
- `__NVIC_SetPriority()`
- `SysTick_Handler()`
- `HAL_IncTick()`
- `HAL_GetTick()`
- `TIM8_UP_IRQHandler()`
- `frameCounter`
- `frame_500Hz`
- `FRAME_COUNT`
- `COUNT_500HZ`
- `systemReady`
- `mpu6050Calibrating`
- `last_print_tick`
- `print_interval`
- `NVIC_PRIORITYGROUP_4`
- `SysTick_IRQn`
- `TIM8_UP_IRQn`
- `USB_LP_CAN1_RX0_IRQn`
- `SCB->SHP`
- `SCB->AIRCR`
- `NVIC->ISER`
- `NVIC->IP`
- `NVIC->ISPR`
- `NVIC->IABR`
- `SysTick->LOAD`
- `SysTick->VAL`
- `SysTick->CTRL`
- `SysTick_LOAD_RELOAD_Msk`
- `SysTick_CTRL_CLKSOURCE_Msk`
- `SysTick_CTRL_TICKINT_Msk`
- `SysTick_CTRL_ENABLE_Msk`
- `__NVIC_PRIO_BITS`
- `uwTick`
- `uwTickFreq`
- `HAL_TICK_FREQ_DEFAULT`
- `HAL_TIM_IRQHandler()`
- `HAL_TIM_Base_Start_IT()`
- `TIM_IT_UPDATE`
- `HAL_TIM_PeriodElapsedCallback()`

质量自检：

- P0 事实错误：通过
- P1 依赖断层：通过
- P2 逻辑连贯：通过
- P3 项目证据：通过
- P4 原理展开：通过
- P5 调试实践：通过
- P6 表达统一：通过

---
> 导航：上一章：[第08章_AFIO重映射与SWD调试](第08章_AFIO重映射与SWD调试.md) ｜ 下一章：[第10章_DWT计时与微秒时间基准](第10章_DWT计时与微秒时间基准.md)
