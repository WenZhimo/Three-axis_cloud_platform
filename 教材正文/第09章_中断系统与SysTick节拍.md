# 第09章 中断系统与SysTick节拍

## 1. 本章目标

- 理解 NVIC 在 Cortex-M3 中断入口、优先级和外设中断使能中的作用。
- 看懂本项目中 SysTick 1ms 节拍、HAL 时基和 500Hz 控制标志之间的关系。
- 能从启动文件、`.ioc`、HAL 源码和项目中断处理文件中追踪中断配置证据。
- 为后续 DWT 微秒计时、500Hz 实时控制循环、TIM8 配置和 USB 章节建立前置基础。

## 2. 前置知识

- Cortex-M3内核启动
- CMSIS寄存器与内核访问
- 系统时钟树

第03章已经说明启动文件中的中断向量表，第05章已经建立 CMSIS 和内核资源访问索引，第06章已经确认项目系统时钟来源。本章在这些基础上先讲 NVIC，再讲 SysTick 节拍，最后把 1ms 中断、500Hz 标志和 10Hz 低频任务接到项目主循环。

本章不展开 TIM8 的 PWM 配置、不展开 USB 中间件处理流程、不展开 DWT 微秒计时细节，也不展开姿态解算和电机控制算法。

## 3. 问题背景

三轴云台项目不是只按顺序执行一次初始化代码。初始化完成后，系统需要不断读取传感器、更新姿态、计算控制量并输出电机指令。这样的系统必须回答两个基础问题：

- 哪些函数会在中断发生时被硬件自动调用。
- 项目用什么时间基准把周期任务变成稳定的执行节奏。

本项目中，`HAL_Init()` 建立 HAL 默认 1ms 时基；`SystemClock_Config()` 内部的 `HAL_RCC_ClockConfig()` 会在系统时钟切换后重新适配 HAL tick；`main.c` 又在用户初始化段调用 `HAL_SYSTICK_Config(SystemCoreClock / 1000)` 和 `HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0)`，明确把 SysTick 配置为 1ms 节拍。`SysTick_Handler()` 每次进入时调用 `HAL_IncTick()`，并在系统就绪且 MPU6050 不处于校准状态时递增 `frameCounter`。当 `frameCounter % COUNT_500HZ == 0` 时，代码置位 `frame_500Hz`，主循环看到该标志后执行 500Hz 任务。

同时，`.ioc` 和生成代码中还存在 TIM8 更新中断、USB 低优先级中断等入口。第09章要建立的是“中断系统怎么接入项目”和“SysTick 如何成为项目节拍基础”，而不是把所有外设中断细节一次讲完。

## 4. 核心概念

- NVIC中断配置：Cortex-M3 中用于管理中断优先级、使能状态和中断入口的配置机制。
- SysTick系统节拍：由 Cortex-M3 内核 SysTick 定时器产生的周期性节拍，本项目配置为 1ms。
- 中断向量表：启动文件中保存异常和外设中断入口函数地址的表。
- 中断处理函数：中断发生后被调用的函数，例如 `SysTick_Handler()` 和 `TIM8_UP_IRQHandler()`。
- HAL 时基：HAL 内部用于 `HAL_GetTick()`、`HAL_Delay()` 等时间函数的毫秒计数基础。
- 500Hz 标志：项目中由 `SysTick_Handler()` 根据 `COUNT_500HZ` 置位的 `frame_500Hz`。
- 10Hz 低频任务：主循环中用 `HAL_GetTick() - last_print_tick >= 100` 判断的低频任务入口。

这些概念服务于正式知识点 `NVIC中断配置` 和 `SysTick系统节拍`，不新增结构外知识点。

## 5. 工作原理

中断系统可以分成三层理解。

第一层是入口。启动文件中的中断向量表把中断号和处理函数名称对应起来。第03章已经讲过复位后如何进入 `main()`，本章补上另一条路径：当 SysTick、TIM8 或 USB 相关中断发生时，CPU 不通过普通函数调用进入处理函数，而是根据向量表跳到对应入口。

第二层是使能和优先级。仅有处理函数名称还不够，外设中断还需要在 NVIC 中配置优先级并使能。本项目中，`HAL_Init()` 调用 `HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4)` 设置优先级分组；`.ioc` 记录 SysTick、TIM8 更新中断和 USB 低优先级中断的启用状态；`tim.c` 为 TIM8 更新中断设置优先级 1 并使能；`usbd_conf.c` 为 USB 低优先级中断设置优先级 0 并使能。

第三层是节拍行为。SysTick 与普通外设中断不同，它是 Cortex-M3 内核自带的系统节拍定时器。项目把 SysTick 配置成 1ms 一次中断。每次进入 `SysTick_Handler()`，先调用 `HAL_IncTick()` 更新 HAL 毫秒计数；随后项目逻辑在系统就绪时更新 `frameCounter`，并每 2 个 1ms tick 置位一次 `frame_500Hz`。

因此，本项目的主节拍链路是：

1. `SystemCoreClock` 来自系统时钟配置。
2. `HAL_RCC_ClockConfig()` 在时钟切换后重新适配 HAL tick。
3. `HAL_SYSTICK_Config(SystemCoreClock / 1000)` 在用户初始化段明确建立 1ms SysTick。
4. `SysTick_Handler()` 每 1ms 进入一次。
5. `HAL_IncTick()` 推进 HAL 毫秒计数。
6. `frameCounter` 按 1ms 累加。
7. `COUNT_500HZ` 为 2，因此每 2ms 置位 `frame_500Hz`。
8. `main()` 主循环消费 `frame_500Hz`，进入后续 500Hz 控制路径。

10Hz 低频任务没有单独使用一个中断，而是在主循环中用 `HAL_GetTick()` 做时间差判断。这样，项目把高频控制触发和低频状态任务都建立在同一个 1ms HAL/SysTick 时间基础上。

## 6. STM32实现机制

在 STM32F1 HAL 工程中，本章相关机制分布在以下位置：

- 启动文件提供中断向量表和弱默认处理函数。
- `HAL_Init()` 配置 NVIC 优先级分组，并通过 `HAL_InitTick()` 建立默认 SysTick 时基。
- `HAL_RCC_ClockConfig()` 在系统时钟切换后再次调用 tick 初始化入口，使 HAL 时基适配新的核心时钟。
- `HAL_SYSTICK_Config()` 最终调用 CMSIS 的 `SysTick_Config()`。
- `HAL_NVIC_SetPriority()` 和 `HAL_NVIC_EnableIRQ()` 封装 CMSIS 的 NVIC 配置入口。
- `stm32f1xx_it.c` 承载项目实际的中断处理函数。
- `main.c` 在用户初始化段中再次明确配置 SysTick 1ms 周期和 SysTick 优先级。

项目中可以看到三类中断入口。

第一类是内核异常和系统异常，例如 `NMI_Handler()`、`HardFault_Handler()`、`SysTick_Handler()`。这些入口在 `stm32f1xx_it.c` 中实现，其中本章重点是 `SysTick_Handler()`。

第二类是外设中断入口，例如 `TIM8_UP_IRQHandler()`。本章只确认它由 `tim.c` 配置优先级和使能，并在处理函数中进入 HAL TIM 分发，具体 TIM8 用途留到第13章。

第三类是 USB 低优先级中断入口。本章只确认它由 `usbd_conf.c` 配置优先级和使能，并在处理函数中进入 HAL PCD 分发，USB 设备和 CDC 细节留到第15章和第16章。

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
- `stm32f1xx_it.c` 放置项目真实中断处理函数。
- `main.h` 定义 `FRAME_COUNT`、`COUNT_500HZ` 和 `frame_500Hz` 等节拍相关符号。
- `main.c` 配置 SysTick，并在主循环中消费 `frame_500Hz` 和 `HAL_GetTick()`。
- `tim.c` 与 `usbd_conf.c` 证明外设中断优先级和使能配置存在，但本章不展开外设内部逻辑。

在项目主流程中，初始化阶段先经过 `HAL_Init()`，随后 `SystemClock_Config()` 通过 `HAL_RCC_ClockConfig()` 切换系统时钟并适配 HAL tick，再初始化外设。用户初始化段再次配置 SysTick 1ms 周期，设置 SysTick 优先级，并在系统准备完成后把 `systemReady` 置为 `true`。此后，`SysTick_Handler()` 才会在条件满足时推进项目帧计数并产生 `frame_500Hz`。

## 8. 代码分析

### 1. 启动文件中的中断入口

`Core/Startup/startup_stm32f103rctx.s` 中的向量表登记了 `SysTick_Handler`、TIM8 更新中断入口和 USB 低优先级中断入口。文件后半部分还为这些处理函数提供弱别名，若项目源码中没有同名函数，就会落到默认死循环处理。

本项目在 `Core/Src/stm32f1xx_it.c` 中实现了这些同名处理函数，因此中断发生时会进入项目实现，而不是默认处理。

### 2. `HAL_Init()` 中的默认时基

`Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c` 中，`HAL_Init()` 会调用 `HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4)`，然后调用 `HAL_InitTick(TICK_INT_PRIORITY)`。

`HAL_InitTick()` 默认使用 `HAL_SYSTICK_Config(SystemCoreClock / (1000U / uwTickFreq))` 建立 1ms 时基，并设置 SysTick 优先级。这里说明 HAL 本身依赖 SysTick 作为默认毫秒时间来源。

### 3. `HAL_RCC_ClockConfig()` 中的 tick 适配

`Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c` 中，`HAL_RCC_ClockConfig()` 在更新 `SystemCoreClock` 后调用 `HAL_InitTick(uwTickPrio)`。这说明系统时钟切换后，HAL 会按新的核心时钟重新适配 SysTick 时基。

本章只使用这个结论解释 SysTick 与系统时钟的关系。`HAL_RCC_ClockConfig()` 的时钟树细节已经在第06章说明。

### 4. `main.c` 中的 SysTick 配置

`Core/Src/main.c` 在外设初始化之后调用：

- `HAL_SYSTICK_Config(SystemCoreClock / 1000)`
- `HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0)`

这说明项目显式把 SysTick 配置为 1ms，并将 SysTick 优先级设置为 0。第06章已经说明 `SystemCoreClock` 的来源，本章只使用这个时钟结果，不重新展开时钟树。

### 5. `SysTick_Handler()` 中的 HAL tick

`Core/Src/stm32f1xx_it.c` 的 `SysTick_Handler()` 先调用 `HAL_IncTick()`。这是 `HAL_GetTick()` 和 `HAL_Delay()` 等 HAL 时间函数能持续工作的基础。

如果这一步被删除或长时间阻塞，主循环中的 10Hz 判断 `HAL_GetTick() - last_print_tick >= 100` 就会失去可靠时间来源。

### 6. `SysTick_Handler()` 中的 500Hz 标志

`SysTick_Handler()` 在 `systemReady == true` 且 `mpu6050Calibrating == false` 时递增 `frameCounter`。当 `frameCounter > FRAME_COUNT` 时，代码把计数器回绕到 1。`main.h` 中 `FRAME_COUNT` 为 1000，`COUNT_500HZ` 为 2。

当 `frameCounter % COUNT_500HZ == 0` 时，处理函数置位 `frame_500Hz`。由于 SysTick 是 1ms，`COUNT_500HZ` 为 2，所以该标志约每 2ms 置位一次，对应 500Hz。

项目源码中 `frame_500Hz` 附近有注释提到定时器中断，但当前实际置位位置在 `SysTick_Handler()`。教材以代码事实为准：本章阶段应把 500Hz 触发来源追踪到 SysTick，而不是 TIM8。

### 7. 主循环中的 500Hz 消费

`Core/Src/main.c` 的主循环检查 `frame_500Hz`。如果标志为真，就先清除标志，再计算 `deltaTime500Hz` 和 `dt500Hz`，随后进入传感器读取、姿态更新和电机控制路径。

这些后续处理属于第26章和算法篇的内容。本章只确认：主循环不是靠延时函数固定等待 2ms，而是靠 SysTick 中断置位的标志驱动高频任务入口。

### 8. 主循环中的 10Hz 低频任务

主循环还使用 `HAL_GetTick() - last_print_tick >= 100` 判断 100ms 是否到达。100ms 对应 10Hz，因此这一路适合状态检查和串口打印等低频任务。

这个 10Hz 判断仍然依赖 `HAL_IncTick()` 维护的 HAL 毫秒计数。也就是说，500Hz 标志和 10Hz 判断共享同一个 1ms SysTick/HAL 时基。

### 9. TIM8 和 USB 中断边界

`Core/Src/tim.c` 中 TIM8 分支调用 `HAL_NVIC_SetPriority(TIM8_UP_IRQn, 1, 0)` 和 `HAL_NVIC_EnableIRQ(TIM8_UP_IRQn)`，`stm32f1xx_it.c` 中对应处理函数调用 `HAL_TIM_IRQHandler(&htim8)`。

`USB_DEVICE/Target/usbd_conf.c` 中 USB 低优先级中断分支调用 `HAL_NVIC_SetPriority(..., 0, 0)` 和 `HAL_NVIC_EnableIRQ(...)`，`stm32f1xx_it.c` 中对应处理函数进入 HAL PCD 分发。

这些证据说明项目确实存在外设中断配置。但第09章只建立 NVIC 配置方法和入口关系，TIM8 和 USB 的业务含义分别留到后续章节。

## 9. 调试方法

本章阶段的调试目标是确认中断入口、SysTick 节拍和主循环标志之间的链路闭合。

可观察对象：

- `.ioc` 中 SysTick、TIM8 更新中断和 USB 低优先级中断是否被启用。
- `HAL_Init()` 是否仍在启动后执行。
- `HAL_RCC_ClockConfig()` 是否在系统时钟配置中完成并适配 HAL tick。
- `main.c` 是否调用 `HAL_SYSTICK_Config(SystemCoreClock / 1000)`。
- `main.c` 是否设置 `SysTick_IRQn` 优先级。
- `SysTick_Handler()` 是否调用 `HAL_IncTick()`。
- `SysTick_Handler()` 是否在系统就绪后递增 `frameCounter`。
- `frameCounter % COUNT_500HZ == 0` 时是否置位 `frame_500Hz`。
- 主循环是否及时清除并消费 `frame_500Hz`。
- 10Hz 任务是否依赖 `HAL_GetTick()` 的 100ms 差值。

常见异常定位：

- 10Hz 任务不再触发：先检查 `HAL_IncTick()` 是否仍在 `SysTick_Handler()` 中执行，再检查 `HAL_GetTick()` 判断条件。
- 500Hz 主循环不执行：先检查 `systemReady` 和 `mpu6050Calibrating`，再检查 `frameCounter` 和 `frame_500Hz`。
- 500Hz 周期明显不对：先确认 `SystemCoreClock` 和 `HAL_SYSTICK_Config(SystemCoreClock / 1000)`，再确认 `COUNT_500HZ` 是否仍为 2。
- TIM8 或 USB 中断处理异常：先确认 `.ioc` 和对应 MSP 初始化中的 NVIC 优先级/使能，再进入各自后续章节分析外设逻辑。
- 程序进入默认死循环：检查启动文件中向量名称和 `stm32f1xx_it.c` 中实际函数名称是否一致。

## 10. 常见问题

### 1. SysTick 是否就是项目的 500Hz 控制循环？

不是。SysTick 在本项目中是 1ms 节拍。500Hz 控制入口来自 `SysTick_Handler()` 每 2 个 tick 置位一次 `frame_500Hz`，主循环再消费该标志。SysTick 是节拍来源，500Hz 循环是项目在主循环中基于标志组织出来的任务入口。

### 2. 为什么 `HAL_IncTick()` 必须保留？

因为 `HAL_GetTick()` 的毫秒计数依赖它。项目的 10Hz 任务使用 `HAL_GetTick() - last_print_tick >= 100` 判断时间差；如果 `HAL_IncTick()` 不运行，低频任务和 HAL 延时都会受到影响。

### 3. 为什么中断里只置位 `frame_500Hz`，不直接读取 MPU6050？

项目源码中的注释已经说明，在中断中读 I2C 会消耗较多时间，还可能与主循环读取形成冲突。因此当前实现只在中断中设置标志，把较重的读取和控制计算留给主循环。

### 4. TIM8 已经配置了中断，为什么 500Hz 还说来自 SysTick？

因为当前代码中 `frame_500Hz` 的实际置位位置在 `SysTick_Handler()`，而 TIM8 中断处理函数只是进入 HAL TIM 分发。本章以当前源码事实为准。TIM8 的配置和用途留到第13章再分析。

### 5. SysTick 优先级 0 是否意味着一定不会被其他中断影响？

不能这样简单理解。优先级还要结合分组、具体中断优先级和当前执行状态判断。本章只确认项目中 SysTick 和 USB 低优先级中断配置为优先级 0，TIM8 更新中断配置为优先级 1；更复杂的抢占行为需要结合后续外设章节和实际调试观察。

## 11. 实践任务

- 在启动文件中找到 `SysTick_Handler` 和 TIM8 更新中断入口。
- 在 `.ioc` 中找到 NVIC 相关配置项和 SysTick 虚拟外设配置。
- 在 HAL 源码中找到 `HAL_Init()`、`HAL_InitTick()` 和 `HAL_SYSTICK_Config()`。
- 在 `stm32f1xx_it.c` 中找到 `SysTick_Handler()`，画出 `HAL_IncTick()`、`frameCounter`、`frame_500Hz` 的关系。
- 在 `main.h` 中确认 `FRAME_COUNT` 和 `COUNT_500HZ` 的定义。
- 在 `main.c` 中找到主循环消费 `frame_500Hz` 的位置。
- 在 `main.c` 中找到 10Hz 低频任务的 `HAL_GetTick()` 判断。
- 对比 `tim.c` 和 `usbd_conf.c` 中的 NVIC 配置，说明它们为什么属于后续章节边界。

## 12. 思考题

1. 为什么第09章先讲 NVIC，再讲 SysTick？
2. 如果 `SysTick_Handler()` 中没有调用 `HAL_IncTick()`，哪些项目行为会首先受到影响？
3. `COUNT_500HZ` 为什么等于 2 时能从 1ms SysTick 得到约 500Hz 标志？
4. 为什么当前项目没有在 SysTick 中直接执行传感器读取和控制计算？
5. 如果 `systemReady` 一直为 `false`，`frame_500Hz` 会发生什么变化？
6. 为什么不能只看 `.ioc` 判断项目中断行为，还必须看 `stm32f1xx_it.c` 和外设 MSP 配置？

## 13. 本章总结

本章建立了三轴云台项目中 NVIC、中断入口、SysTick 1ms 节拍、500Hz 标志和 10Hz 低频任务之间的证据链。

已经确认的结论是：

- 启动文件中的中断向量表决定中断入口名称。
- `HAL_Init()` 设置 NVIC 优先级分组并建立默认 HAL 时基。
- `HAL_RCC_ClockConfig()` 在系统时钟切换后重新适配 HAL tick。
- `main.c` 明确调用 `HAL_SYSTICK_Config(SystemCoreClock / 1000)` 配置 1ms SysTick。
- `SysTick_Handler()` 调用 `HAL_IncTick()` 维护 HAL 毫秒计数。
- `SysTick_Handler()` 通过 `frameCounter` 和 `COUNT_500HZ` 置位 `frame_500Hz`。
- 主循环消费 `frame_500Hz` 进入后续 500Hz 控制路径。
- 10Hz 低频任务使用 `HAL_GetTick()` 的 100ms 时间差判断。
- TIM8 和 USB 中断在项目中存在，但本章只建立 NVIC 入口和配置边界。

下一章可以进入 DWT 计时与微秒时间基准，因为本章已经说明了 1ms SysTick 和 HAL 毫秒时间；后续需要解释项目如何用 `micros()` 和 DWT 周期计数获得更细的时间测量。

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

符号、函数与配置项证据：

- `HAL_Init()`
- `HAL_InitTick()`
- `HAL_RCC_ClockConfig()`
- `HAL_SYSTICK_Config()`
- `HAL_NVIC_SetPriorityGrouping()`
- `HAL_NVIC_SetPriority()`
- `HAL_NVIC_EnableIRQ()`
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

质量自检：

- P0 事实错误：通过
- P1 依赖断层：通过
- P2 逻辑连贯：通过
- P3 项目证据：通过
- P4 原理展开：通过
- P5 调试实践：通过
- P6 表达统一：通过
