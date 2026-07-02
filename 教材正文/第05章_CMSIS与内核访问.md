# 第05章 CMSIS与内核访问

> 导航：上一章：[第04章_HAL工程裁剪与构建产物](第04章_HAL工程裁剪与构建产物.md) ｜ 下一章：[第06章_系统时钟树](第06章_系统时钟树.md)

## 1. 本章目标

- 建立项目中直接访问内核资源和寄存器资源的位置索引。
- 理解 CMSIS 如何把 Cortex-M3 内核寄存器和 STM32 外设寄存器暴露给 C 代码。
- 区分 HAL API 调用和直接寄存器访问的工程边界。
- 为后续 DWT、SysTick、定时器寄存器访问和中断配置章节建立前置基础。

本章阅读分层：

| 阅读层次 | 建议范围 | 适合读者 |
|---|---|---|
| 【必须掌握】 | 第1节到第5节，第13节总结 | 需要理解 CMSIS 头文件、内核/外设寄存器映射和直接访问主线的读者 |
| 【工程深化】 | 第6节到第8节，第9节调试方法 | 需要维护 DWT、SysTick、`TIMx->CCR`、`PRIMASK` 临界区和系统控制访问的读者 |
| 【拓展阅读】 | 第5.1节到第5.3节，第8.9.1节到第8.10节，第10节到第12节 | 需要进一步理解设备头文件选择、`volatile` 语义、向量表、内联函数证据读法和 HAL/CMSIS边界的读者 |
| 【证据与验证】 | 第5.1节到第5.3节、第8节、第9节、章节尾部固定检查，以及所有 `【待验证】` 项 | 需要审查 CMSIS 权威定义、设备选择、启动角色、`SystemCoreClock`、SysTick、DWT、定时器寄存器、PRIMASK临界区、系统控制访问、构建产物、寄存器读回或中断屏蔽状态的读者 |

如果只是沿后续章节前置基础学习，可以先抓住“CMSIS头文件选择 -> 固定地址寄存器映射 -> 项目直接访问点 -> HAL与寄存器访问边界”这条链；验证计时精度、临界区或外设真实行为时，再回到证据边界和调试方法小节。

本章速查：

| 查找目标 | 优先阅读 | 避免重复展开 |
|---|---|---|
| CMSIS 头文件选择、固定地址映射和直接访问主线 | 第4节到第5节、第13节 | STM32平台前提回到第01章，HAL构建边界回到第04章 |
| 内核资源、外设寄存器和使能链差异 | 第5.1节到第5.3节、第6节、第8.1节到第8.4节 | 系统时钟树继续到第06章，不把内核访问与APB门控混成一类 |
| SysTick、DWT、定时器 CCR、PRIMASK 和 SCB 访问点 | 第7节、第8.5节到第8.10节、第9节 | DWT计时回到第10章，PWM寄存器更新回到第12章 |
| 构建产物、CMSIS权威定义和运行验证边界 | 第8.11节到第8.12节、章节尾部固定检查 | `.map/.list` 不能替代寄存器读回、中断屏蔽状态或外设运行实测 |

## 2. 前置知识

- STM32F103RCTx芯片平台

这些前置主要对应第01章，并为后续第09章、第10章和第30章提供内核访问基础。

本章只要求读者已经知道项目运行在 STM32F103RCTx 平台上。HAL 构建、启动流程和链接脚本已经在前面章节建立，本章开始识别源码中更接近硬件的访问方式。

## 3. 问题背景

前几章已经说明工程如何被构建、如何启动、哪些 HAL 文件进入编译。接下来需要回答一个更细的问题：项目是不是只通过 HAL API 访问硬件？

答案是否定的。本项目中同时存在两类访问方式：

- 通过 HAL API 完成通用初始化和外设操作。
- 通过 CMSIS 名称直接访问内核寄存器或外设寄存器。

直接访问寄存器并不是“更高级”或“更随意”的写法，而是为了解决 HAL API 不够直接或不够细的问题。例如：

- 微秒计时需要读取 `SysTick->VAL` 和 `SysTick->LOAD`。
- 启用周期计数器需要写 `CoreDebug->DEMCR` 和 `DWT->CTRL`。
- 三相 PWM 快速更新需要直接写 `TIMx->CNT` 和 `TIMx->CCR`。
- 某些低功耗或系统控制行为需要访问 `SCB->SCR`。

本章先建立索引和边界，后续章节再分别展开 DWT、SysTick、定时器和中断细节。

## 4. 核心概念

- CMSIS寄存器与内核访问：通过 CMSIS 定义的结构体、宏和内联函数访问 Cortex-M 内核资源与 STM32 寄存器资源。
- CMSIS：ARM 提供的 Cortex 微控制器软件接口标准，本项目相关头文件位于 `Drivers/CMSIS`。
- CMSIS内核层：由 `core_cm3.h` 提供，描述 Cortex-M3 内核、系统控制和调试资源。
- CMSIS设备层：由 `stm32f103xe.h` 提供，描述 STM32F103xE 外设、基地址和寄存器结构。
- 设备选择宏：编译宏 `STM32F103xE` 决定 `stm32f1xx.h` 选择哪个具体设备头文件。
- CMSIS编译器层：由 `cmsis_gcc.h` 等文件提供 `__disable_irq()`、`__enable_irq()` 这类内联函数。
- 访问权限宏：`__IO`、`__IM`、`__OM`、`__IOM` 用 `volatile` 表示寄存器读写权限。
- 内存映射寄存器：硬件寄存器被映射到固定地址，C 代码通过把地址转换为结构体指针来访问。
- 基地址与偏移：外设或内核模块有固定基地址，结构体成员偏移决定具体寄存器地址。
- 位掩码：CMSIS 用 `*_Msk` 表达某一控制位或字段的位置，项目通过或运算设置控制位。
- PRIMASK：Cortex-M 的中断屏蔽寄存器，`__disable_irq()` 和 `__enable_irq()` 通过修改它影响普通可屏蔽中断。
- 内核寄存器：属于 Cortex-M3 内核或系统控制空间的寄存器，例如 `SysTick`、`DWT`、`CoreDebug`、`SCB`。
- 外设寄存器：属于 STM32 片上外设的寄存器，例如 `TIM2->CNT`、`TIM3->CCR1`。
- 直接寄存器访问：不通过 HAL API 封装，直接读写寄存器结构体成员。
- 原子性保护：在短时间关键更新前后关闭和恢复中断，减少被中断打断的风险。
- 启动向量表：复位和中断入口地址表，本项目由 `startup_stm32f103rctx.s` 中的 `g_pfnVectors` 提供。
- 系统时钟变量：`SystemCoreClock` 是 CMSIS 设备层的软件变量，用于表达当前 HCLK 估计值。

这些概念都服务于 `CMSIS寄存器与内核访问`，不新增正式知识点。

## 5. 工作原理

CMSIS 让 C 代码可以用结构体成员的形式访问固定地址上的硬件寄存器。

它的基本机制是：

1. CMSIS 头文件定义寄存器结构体类型，例如 `SysTick_Type`、`DWT_Type`、`SCB_Type`。
2. CMSIS 头文件定义外设或内核资源的基地址。
3. CMSIS 用宏把基地址转换成结构体指针，例如 `SysTick`、`DWT`、`CoreDebug`、`SCB`。
4. 项目代码通过 `SysTick->VAL`、`DWT->CYCCNT`、`TIM3->CCR1` 这样的形式读写寄存器。

这类访问绕过了 HAL 函数调用层，优点是直接、明确、开销低；风险是必须清楚寄存器含义、时序要求和并发影响。

把这件事写成最小形式，就是：

```text
寄存器地址 = 模块基地址 + 结构体成员偏移
```

例如 `stm32f103xe.h` 中 `TIM3_BASE = APB1PERIPH_BASE + 0x00000400`，而 `APB1PERIPH_BASE = 0x40000000`，所以 `TIM3_BASE = 0x40000400`。`TIM_TypeDef` 中 `CCR2` 的偏移是 `0x38`，因此 `TIM3->CCR2` 对应的寄存器地址可推为 `0x40000438`。这不是普通结构体变量寻址，而是对固定外设地址的写入。

同理，`core_cm3.h` 中 `SCS_BASE = 0xE000E000`，`SysTick_BASE = SCS_BASE + 0x0010`，所以 `SysTick` 对应 `0xE000E010`；`SCB_BASE = SCS_BASE + 0x0D00`，对应 `0xE000ED00`；`DWT_BASE = 0xE0001000`；`CoreDebug_BASE = 0xE000EDF0`。这些地址解释了为什么 CMSIS 名称能直接触达 Cortex-M3 内核资源。

这里还要再拆一层：**能直接写寄存器** 不等于 **同样的时钟前提**。`SysTick`、`DWT`、`CoreDebug` 属于 Cortex-M3 内核和系统控制空间，访问它们不依赖某个 APB 外设时钟使能位；而 `TIM2`、`TIM3`、`TIM4` 这类 STM32 外设寄存器虽然也能用 `TIMx->REG` 形式直接访问，但它们是否真正工作，还要先经过对应定时器时钟使能。

| 访问对象 | 当前证据 | 需要的前置 | 不能从哪里直接推出 |
|---|---|---|---|
| `SysTick` / `DWT` / `CoreDebug` | `core_cm3.h` 定义的内核结构体和 `main.c` 中直接访问 | 内核寄存器地址映射和 trace/系统控制权限 | 不能从 `__HAL_RCC_TIMx_CLK_ENABLE()` 推出它们需要 APB 门控 |
| `TIM2` / `TIM3` / `TIM4` | `stm32f103xe.h` 定义的外设结构体和 `drv_pwmMotors.c` / `tim.c` 中的直接访问 | 对应 TIM 外设时钟先被使能 | 不能把 `TIMx->CCR` 与内核寄存器访问混成同一类前置 |

本项目里这一差异能从源码直接看出来：`Core/Src/main.c` 先写 `CoreDebug->DEMCR`、`DWT->CYCCNT` 和 `DWT->CTRL`，而 `Core/Src/tim.c` 则在 `HAL_TIM_Base_MspInit()` 和 `HAL_TIM_PWM_MspInit()` 中分别调用 `__HAL_RCC_TIM2_CLK_ENABLE()`、`__HAL_RCC_TIM3_CLK_ENABLE()`、`__HAL_RCC_TIM4_CLK_ENABLE()` 和 `__HAL_RCC_TIM8_CLK_ENABLE()`。这说明内核调试资源和定时器外设资源虽然都用 CMSIS 风格结构体访问，但它们的使能链并不相同。读者若把两者混在一起，就会把“寄存器地址可写”误当成“硬件已经工作”。

### 5.1 CMSIS头文件选择链

本项目不是偶然能看到 `DWT`、`SysTick` 和 `TIM2` 这些名字。它们来自一条可追溯的头文件选择链：

```text
.cproject 中的 -DSTM32F103xE
-> stm32f1xx.h 根据 STM32F103xE 选择 stm32f103xe.h
-> stm32f103xe.h 引入 core_cm3.h 和 system_stm32f1xx.h
-> core_cm3.h 提供内核资源
-> stm32f103xe.h 提供 STM32F103xE 外设资源
```

从业务源码看，常见入口是：

```text
main.h
-> stm32f1xx_hal.h
-> stm32f1xx_hal_conf.h
-> stm32f1xx_hal_xxx.h
-> stm32f1xx_hal_def.h
-> stm32f1xx.h
```

因此，`DWT->CYCCNT` 和 `TIM3->CCR1` 看起来都是结构体成员访问，但来源层级不同：

- `DWT`、`SysTick`、`SCB`、`CoreDebug` 属于 Cortex-M3 内核和系统控制空间。
- `TIM2`、`TIM3`、`TIM4`、`TIM_TypeDef` 属于 ST 的 STM32F103xE 设备外设层。

如果 `STM32F103xE` 宏缺失或选错，设备头文件选择就会出错。表现可能是外设符号缺失、寄存器布局不匹配，或者编译阶段被 `stm32f1xx.h` 的设备选择检查拦住。

### 5.2 `volatile`寄存器语义

CMSIS 用 `__IO`、`__IM`、`__OM` 和 `__IOM` 修饰寄存器结构体成员。以本项目相关文件为例：

- `core_cm3.h` 中 `__IO` 和 `__IOM` 都展开为 `volatile`。
- `stm32f103xe.h` 中 `TIM_TypeDef` 的 `CNT`、`PSC`、`ARR` 和 `CCR1` 到 `CCR4` 使用 `__IO`。

`volatile` 的含义是提醒编译器：这类对象可能被硬件或中断上下文改变，不能按普通内存变量随意缓存或删除访问。

它不能保证：

- 多个寄存器写入天然不可被中断打断。
- 读改写序列天然具备原子性。
- 外设已经完成初始化或处在正确工作模式。
- 写入 `CCR` 后外部 PWM 波形一定符合预期。

所以 `drv_pwmMotors.c` 在多路 `CCR` 写入前后使用 `__disable_irq()` 和 `__enable_irq()`，它解决的是短临界区内被中断插入的问题。它不能替代定时器更新事件、预装载寄存器、硬件波形和电机响应验证。

`cmsis_gcc.h` 中 `__disable_irq()` 对应 `cpsid i`，`__enable_irq()` 对应 `cpsie i`。这两个内联函数直接改变普通可屏蔽中断的响应状态。当前项目用“关闭后再打开”的方式包围定时器寄存器写入，但没有保存进入临界区前的 `PRIMASK`。因此它适合短小、外层已知未屏蔽中断的临界区；如果未来出现嵌套临界区，更稳妥的模式是先用 `__get_PRIMASK()` 保存原状态，再用 `__set_PRIMASK()` 恢复原状态。

### 5.3 启动向量表与系统变量

CMSIS 设备层还连接了启动文件与系统时钟变量。复位后的软件链路可以概括为：

```text
g_pfnVectors
-> Reset_Handler
-> SystemInit()
-> 复制 .data
-> 清零 .bss
-> __libc_init_array
-> main()
```

`startup_stm32f103rctx.s` 中的 `g_pfnVectors` 说明复位入口和中断入口如何链接。它能证明入口地址存在，不能证明某个中断已经在运行时发生。

`system_stm32f1xx.c` 中 `SystemCoreClock` 默认值为 `8000000`。
`SystemClock_Config()` 调用 `HAL_RCC_ClockConfig()` 后，HAL 会更新这个变量。
`main.c` 后续用 `HAL_SYSTICK_Config(SystemCoreClock / 1000)` 设置 1ms SysTick。

这里有一个重要边界：`SystemCoreClock` 是软件计算值。它是否等于真实 HCLK，取决于 HSE、PLL、预分频、HAL 更新路径和板级晶振事实；没有现场测量或调试记录时，真实频率仍应保持【待验证】。

从当前 `SystemClock_Config()` 的参数看，项目意图是 `HSE = 8 MHz`、`PLL x9`，得到 `SYSCLK/HCLK = 72 MHz`；`APB1 = HCLK/2 = 36 MHz`，`APB2 = HCLK = 72 MHz`。`HAL_RCC_ClockConfig()` 中会根据当前 RCC 配置更新 `SystemCoreClock`，并重新调用 `HAL_InitTick(uwTickPrio)`。随后 `HAL_SYSTICK_Config(SystemCoreClock / 1000)` 若使用 `72 MHz`，则传入 `72000`，CMSIS `SysTick_Config()` 会写 `LOAD = ticks - 1 = 71999`。这说明 1ms tick 的计算链条是可推导的，但真实晶振和 PLL 是否稳定仍需实测【待验证】。

本项目使用直接访问主要集中在三类场景：

- 时间基准：读 `SysTick` 和 `DWT`。
- 电机输出：写定时器计数器和比较寄存器。
- 系统控制：在特定回调中写系统控制寄存器。

## 6. STM32实现机制

本项目使用的是 Cortex-M3 内核。CMSIS 证据需要分成两层看：

`Drivers/CMSIS/Include/core_cm3.h` 定义 Cortex-M3 内核资源。

`Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xe.h` 定义 STM32F103xE 设备外设资源。

`core_cm3.h` 中可以确认的核心资源包括：

- `SysTick` 映射到 SysTick 配置结构。
- `DWT` 映射到 Data Watchpoint and Trace 结构。
- `CoreDebug` 映射到核心调试寄存器结构。
- `SCB` 映射到系统控制块结构。
- `NVIC` 相关内联函数用于中断控制。

`stm32f103xe.h` 中可以确认的设备资源包括：

- `__CM3_REV` 为 `0x0200U`，用于描述 Cortex-M3 内核修订信息。
- `__NVIC_PRIO_BITS` 为 `4U`，影响 NVIC 优先级位宽解释。
- `__Vendor_SysTickConfig` 为 `0U`，表示使用 CMSIS 默认 SysTick 配置函数。
- `TIM_TypeDef` 定义定时器寄存器结构。
- `TIM2`、`TIM3` 和 `TIM4` 映射到对应定时器基地址。
- `TIMx->CNT`、`TIMx->PSC`、`TIMx->ARR` 和 `TIMx->CCR` 的访问依赖这些设备头文件定义。
- `PERIPH_BASE = 0x40000000`，`APB1PERIPH_BASE = PERIPH_BASE`，`TIM2_BASE = 0x40000000`，`TIM3_BASE = 0x40000400`，`TIM4_BASE = 0x40000800`。
- `TIM_TypeDef` 中 `CNT` 偏移为 `0x24`，`CCR1` 到 `CCR4` 偏移分别为 `0x34`、`0x38`、`0x3C`、`0x40`。

CMSIS 编译器层还提供了中断屏蔽内联函数：

- `__disable_irq()` 在 GCC 版本中对应 `cpsid i`。
- `__enable_irq()` 在 GCC 版本中对应 `cpsie i`。
- `__get_PRIMASK()` 和 `__set_PRIMASK()` 可用于保存与恢复中断屏蔽状态。

这两个函数属于 Cortex-M 处理器状态控制，不是 STM32 外设寄存器写入。它们会影响普通可屏蔽中断的响应窗口，因此应该只包住足够短、足够必要的临界区。

HAL 与 CMSIS 不是两套互斥系统。HAL 驱动内部也会使用 CMSIS，例如：

- `HAL_SYSTICK_Config()` 调用 CMSIS 的 `SysTick_Config()`。
- HAL 的 NVIC 配置函数会进一步调用 CMSIS NVIC 访问函数。
- HAL、LL 和用户代码最终都依赖 `stm32f1xx.h` 选择出的设备头文件。

所以教材中应区分“直接使用 CMSIS 名称”和“HAL 内部间接依赖 CMSIS”。本项目两种情况都存在。

项目源码中的访问证据包括：

- `Core/Src/main.c` 的 `micros()` 读取 `SysTick->VAL` 和 `SysTick->LOAD`。
- `Core/Src/main.c` 中启用 DWT 周期计数器，写入 `CoreDebug->DEMCR`、`DWT->CYCCNT` 和 `DWT->CTRL`。
- `Core/Src/main.c` 使用 `CoreDebug_DEMCR_TRCENA_Msk` 打开 trace 能力，使用 `DWT_CTRL_CYCCNTENA_Msk` 打开 cycle counter。
- `Core/Src/main.c` 中调用 `HAL_SYSTICK_Config(SystemCoreClock / 1000)` 建立 SysTick 配置入口。
- `Core/Src/stm32f1xx_it.c` 的 `SysTick_Handler()` 属于向量表可链接到的运行入口。
- `Core/Src/stm32f1xx_it.c` 的 `SysTick_Handler()` 调用 `HAL_IncTick()`，但其中记录 `DWT->CYCCNT` 和 `sysTickUptime++` 的代码处于注释块中。
- `Drivers/CustomDrivers/Src/mpu6050Calibration.c` 的 `DWT_Delay_us()` 读取 `DWT->CYCCNT` 实现微秒延时。
- `Drivers/CustomDrivers/Src/drv_pwmMotors.c` 写 `TIM2->CNT`、`TIM3->CNT`、`TIM4->CNT`，用于对齐多个定时器计数器。
- `Drivers/CustomDrivers/Src/drv_pwmMotors.c` 写 `TIMx->CCR`，用于更新 PWM 比较值。
- `USB_DEVICE/Target/usbd_conf.c` 在挂起回调中写 `SCB->SCR` 的系统控制位。
- `Core/Startup/startup_stm32f103rctx.s` 提供 `Reset_Handler` 和 `g_pfnVectors`。
- `Core/Src/system_stm32f1xx.c` 提供 `SystemInit()`、`SystemCoreClock` 和 `SystemCoreClockUpdate()`。

这些访问都能追溯到项目源码和 CMSIS 头文件。

## 7. 项目中的应用

本章对应项目里的底层访问索引层。

直接相关文件：

- `Core/Src/main.c`
- `Core/Inc/main.h`
- `Core/Inc/stm32f1xx_hal_conf.h`
- `Drivers/CustomDrivers/Src/mpu6050Calibration.c`
- `Drivers/CustomDrivers/Src/drv_pwmMotors.c`
- `USB_DEVICE/Target/usbd_conf.c`
- `Core/Src/system_stm32f1xx.c`
- `Core/Src/stm32f1xx_it.c`
- `Core/Startup/startup_stm32f103rctx.s`
- `Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_def.h`
- `Drivers/CMSIS/Include/core_cm3.h`
- `Drivers/CMSIS/Include/cmsis_gcc.h`
- `Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f1xx.h`
- `Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xe.h`
- `Debug/Three-axis_cloud_platformV2.map`
- `Debug/Three-axis_cloud_platformV2.list`

它们之间的关系是：

- `core_cm3.h` 提供内核资源名称和结构体定义。
- `stm32f1xx.h` 根据 `STM32F103xE` 选择具体设备头文件。
- `stm32f103xe.h` 提供 STM32F103xE 设备外设名称、基地址和寄存器结构定义。
- `cmsis_gcc.h` 提供 GCC 编译器下的中断控制内联函数。
- `startup_stm32f103rctx.s` 提供复位入口、中断向量表和默认处理函数。
- `system_stm32f1xx.c` 提供系统初始化函数和 `SystemCoreClock` 软件变量。
- `main.c` 使用 `SysTick` 与 `DWT` 形成时间测量入口。
- `mpu6050Calibration.c` 使用 `DWT` 做微秒级等待。
- `drv_pwmMotors.c` 使用直接定时器寄存器访问更新三相 PWM 输出。
- `usbd_conf.c` 在设备支持文件中访问系统控制块寄存器。

从项目主线看，本章是后续多个章节的公共前置：它不完整讲 DWT、SysTick、定时器和中断，而是让读者先知道项目确实存在 CMSIS 级访问，并知道这些访问分布在哪里。

## 8. 代码分析

### 8.1 `STM32F103xE` 的设备选择角色

`.cproject` 中存在 `STM32F103xE` 编译宏。`stm32f1xx.h` 在设备选择区域根据该宏包含 `stm32f103xe.h`。

这一步决定了项目看到的是 STM32F103xE 的外设集合、寄存器结构和中断枚举。它也解释了为什么 `TIM2`、`TIM3`、`TIM4` 这些宏能在用户代码中直接使用。

如果只看到 `TIM3->CCR1`，但没有追溯到 `STM32F103xE -> stm32f103xe.h -> TIM_TypeDef`，读者会误以为 `TIM3` 是普通全局变量。实际上它是把固定外设基地址转换为 `TIM_TypeDef *` 的宏。

这一点可以直接计算：

- `PERIPH_BASE = 0x40000000`
- `APB1PERIPH_BASE = PERIPH_BASE`
- `TIM2_BASE = 0x40000000`
- `TIM3_BASE = 0x40000400`
- `TIM4_BASE = 0x40000800`

`TIM_TypeDef` 中 `CNT` 偏移为 `0x24`，所以 `TIM2->CNT` 对应 `0x40000024`，`TIM3->CNT` 对应 `0x40000424`，`TIM4->CNT` 对应 `0x40000824`。`CCR1` 到 `CCR4` 偏移为 `0x34`、`0x38`、`0x3C`、`0x40`，所以 `TIM3->CCR2` 对应 `0x40000438`。这类计算能帮助读者把 C 表达式还原为真实总线地址。

### 8.2 `core_cm3.h` 的角色

`Drivers/CMSIS/Include/core_cm3.h` 定义了 Cortex-M3 内核资源。项目代码中出现的 `SysTick`、`DWT`、`CoreDebug` 和 `SCB` 都不是普通变量，而是 CMSIS 根据固定地址映射出的寄存器访问入口。

这意味着源码中的 `DWT->CYCCNT` 不是访问内存数组，而是在读写内核调试与计数资源。

内核资源地址同样可以追溯：

- `SCS_BASE = 0xE000E000`
- `SysTick_BASE = SCS_BASE + 0x0010 = 0xE000E010`
- `SCB_BASE = SCS_BASE + 0x0D00 = 0xE000ED00`
- `DWT_BASE = 0xE0001000`
- `CoreDebug_BASE = 0xE000EDF0`

所以 `SysTick->VAL`、`SCB->SCR`、`DWT->CYCCNT` 和 `CoreDebug->DEMCR` 分别属于 Cortex-M3 系统控制空间、调试/跟踪空间和系统控制块，而不是 STM32 普通外设地址段。

`Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xe.h` 定义了 STM32F103xE 设备外设资源。

项目代码中出现的 `TIM2`、`TIM3`、`TIM4` 和 `TIM_TypeDef` 由设备头文件提供。因此，定时器直接寄存器访问不能只追溯到 `core_cm3.h`。

### 8.3 `__IO` 与 `__IOM` 的角色

`core_cm3.h` 中可以看到 `__IO` 和 `__IOM` 最终都带有 `volatile` 语义。`stm32f103xe.h` 中 `TIM_TypeDef` 的 `CNT`、`PSC`、`ARR` 和 `CCR1` 到 `CCR4` 也使用这类访问权限宏。

这个设计让 C 代码可以像访问结构体字段一样访问硬件寄存器，同时避免编译器把寄存器读写当作普通变量优化掉。

需要注意的是，`volatile` 只约束编译器访问优化，不等价于硬件锁。项目中连续写三个 `CCR` 寄存器时，仍然需要考虑中断插入、定时器更新事件和外部波形验证。

### 8.4 `startup_stm32f103rctx.s` 的启动角色

启动文件声明 `.cpu cortex-m3` 和 `.thumb`，并在 `.isr_vector` 段中放置 `g_pfnVectors`。向量表第一个关键入口是 `_estack`，第二个关键入口是 `Reset_Handler`。

`Reset_Handler` 的执行顺序包括调用 `SystemInit()`、复制 `.data`、清零 `.bss`、调用 `__libc_init_array`，最后跳转到 `main()`。

这条链路解释了为什么 `system_stm32f1xx.c` 中的 CMSIS 系统函数会在应用主函数之前参与运行。它也提醒读者：向量表中有某个 `IRQHandler` 名称，只证明它可以被链接为入口，不证明该中断在硬件上已经触发。

### 8.5 `SystemCoreClock` 与 SysTick 配置

`system_stm32f1xx.c` 中 `SystemCoreClock` 的初始值是 `8000000`。`SystemClock_Config()` 配置 HSE、PLL 和总线分频后，`HAL_RCC_ClockConfig()` 会更新该变量。

`main.c` 随后调用 `HAL_SYSTICK_Config(SystemCoreClock / 1000)`。这说明 SysTick 重装载值依赖 `SystemCoreClock` 的软件值。

`SystemCoreClock` 是否与真实 HCLK 完全一致，需要检查时钟配置、HAL 更新路径、晶振实际频率和现场测量。仓库源码本身只能证明变量和调用关系，不能证明板上真实频率。

当前 `SystemClock_Config()` 的意图配置为：

```text
HSE = 8 MHz
HSEPrediv = 1
PLL = HSE * 9
SYSCLK = PLLCLK = 72 MHz
AHB = SYSCLK / 1 = 72 MHz
APB1 = HCLK / 2 = 36 MHz
APB2 = HCLK / 1 = 72 MHz
```

`HAL_RCC_ClockConfig()` 内部会更新 `SystemCoreClock`，随后重新调用 `HAL_InitTick(uwTickPrio)`。如果 `SystemCoreClock = 72000000`，则 `HAL_SYSTICK_Config(SystemCoreClock / 1000)` 传入 `72000`。CMSIS `SysTick_Config()` 会检查 `ticks - 1` 是否超过 `0xFFFFFF`，然后写入 `SysTick->LOAD = 71999`、`SysTick->VAL = 0`，并设置 `CLKSOURCE`、`TICKINT` 和 `ENABLE`。因此，1ms SysTick 的软件计算链是可复核的。

### 8.6 `main.c` 中的 SysTick 访问

`micros()` 函数读取 `HAL_GetTick()` 和 `SysTick->VAL`，并使用 `SysTick->LOAD + 1` 作为每毫秒的计数周期。它试图在毫秒 tick 之外补充一个更细的计数位置。

这里的关键点是：`HAL_GetTick()` 是 HAL 层的毫秒计数入口，`SysTick->VAL` 和 `SysTick->LOAD` 是 CMSIS 级寄存器访问。二者组合后，项目得到微秒时间戳的计算基础。

`SysTick->VAL` 是倒计数值，所以函数使用 `(tms - u0) * 1000 / tms` 把“已经走过的计数”换算为本毫秒内的微秒偏移。函数先读 `m0/u0`，再读 `m1/u1`，如果两次毫秒 tick 不同，就使用新的毫秒值和新的倒计数值，减少跨 1ms 边界时的误差。

这个算法仍有边界：它依赖 SysTick 已按 1ms 配置、`HAL_IncTick()` 正常推进、`SystemCoreClock` 与真实 HCLK 接近，并假设读取过程中的中断时序没有造成更复杂的嵌套情况。真实精度需要用 DWT 对照、逻辑分析仪或日志验证【待验证】。

### 8.7 `main.c` 中的 DWT 启用

`main.c` 在初始化阶段写入 `CoreDebug->DEMCR`，然后清零 `DWT->CYCCNT`，再通过 `DWT->CTRL` 启用周期计数。

这说明 DWT 周期计数器不是默认直接可用，项目需要先打开 trace 相关能力，再启用 cycle counter。详细计时原理留到第10章。

这三步对应的 CMSIS 位掩码是：

- `CoreDebug_DEMCR_TRCENA_Msk`：允许 trace 相关资源。
- `DWT->CYCCNT = 0`：清零周期计数器。
- `DWT_CTRL_CYCCNTENA_Msk`：启用 cycle counter。

当前项目没有显式检查 `DWT_CTRL_NOCYCCNT_Msk`，也没有在 DWT 初始化后读回确认 `CYCCNT` 是否递增。因此仓库能证明“项目尝试启用 DWT”，但硬件上 DWT 是否实际运行仍应通过断点观察或计数差验证【待验证】。

### 8.8 `mpu6050Calibration.c` 中的 DWT 延时

`DWT_Delay_us()` 读取起始 `DWT->CYCCNT`，把目标微秒数换算成周期数，然后循环等待计数差达到目标。

本章只确认它是 CMSIS 级计时访问。它为什么用于标定采样、采样间隔是否合理，留到传感器标定章节分析。

这里的无符号差值写法 `(DWT->CYCCNT - start) < ticks` 有一个工程优点：只要等待周期数小于 32 位计数器回绕周期，它可以自然跨过一次计数器溢出。若 HCLK 为 72 MHz，`2^32 / 72000000 ≈ 59.65s`，而当前标定中 `sampleRate = 1000` 微秒，对应约 `72000` 个周期，远小于回绕周期。

同时也要看到限制：`ticks = us * (SystemCoreClock / 1000000)` 可能在极大 `us` 输入下发生 32 位乘法溢出；它也依赖 `SystemCoreClock` 的软件值接近真实 HCLK。当前仓库没有运行时溢出保护或实测误差记录。

### 8.9 `drv_pwmMotors.c` 中的定时器寄存器访问

`PWM_Motor_Init()` 在启动多个 PWM 通道后，用 `__disable_irq()` 和 `__enable_irq()` 包住 `TIM2->CNT`、`TIM3->CNT`、`TIM4->CNT` 清零操作，用于减少更新过程中被中断打断的风险。

`PWM_Motor_SetAngle()` 和硬件诊断函数直接写多个 `TIMx->CCR` 寄存器，用于更新 PWM 比较值。

这些访问是项目执行器链路的重要证据，但本章只建立“直接寄存器访问”这个入口。多定时器同步、更新事件和三相 PWM 输出留到后续章节。

源码注释声称多定时器清零“只相差 1~2 个 CPU 时钟周期”。这类具体时间差需要反汇编、DWT 计数或示波器证据支撑；仓库内只能证明连续写寄存器和关闭中断的意图，实际相位误差保持【待验证】。

当前临界区使用 `__disable_irq()` 后直接 `__enable_irq()`。如果函数进入时中断本来已经被外层屏蔽，这种写法会在退出时无条件重新打开中断。当前源码中这些临界区很短，且位于电机 PWM 更新路径；本章只记录风险边界，不直接判定它已经造成问题。若后续需要提高健壮性，可用 `uint32_t primask = __get_PRIMASK(); __disable_irq(); ... __set_PRIMASK(primask);` 保留进入前状态。

#### 8.9.1 `PRIMASK` 临界区的构建产物证据边界

`Drivers/CMSIS/Include/cmsis_gcc.h` 把 `__disable_irq()` 和 `__enable_irq()` 分别定义为 `cpsid i` 与 `cpsie i`，并另外提供了 `__get_PRIMASK()` / `__set_PRIMASK()` 作为保存和恢复中断屏蔽状态的入口。当前项目里，`Drivers/CustomDrivers/Src/drv_pwmMotors.c` 的 `PWM_Motor_Init()` 和 `PWM_Motor_SetAngle()` 都直接调用了这组接口。

`Debug/Three-axis_cloud_platformV2.list` 能把这些调用展开到指令级：`PWM_Motor_Init()` 里能看到 `cpsid i`、三次 `TIMx->CNT = 0` 和 `cpsie i`，`PWM_Motor_SetAngle()` 里也能看到对应的 `cpsid i` / `cpsie i` 包围多路 `TIMx->CCR` 写入。这样可以证明“临界区写法”和“寄存器写入顺序”已经进入当前 Debug 构建，但不能证明当时的 `PRIMASK` 原值，也不能证明退出后一定恢复到进入前的中断状态【待验证】。

`Debug/Three-axis_cloud_platformV2.map` 只能进一步证明 `PWM_Motor_Init`、`PWM_Motor_SetAngle` 和相关代码路径进入了最终链接产物；它不单独给出 `cpsid i` / `cpsie i` 这种内联指令的原地上下文，也不能替代运行时读回、断点观察或中断屏蔽状态截图。

#### 8.9.2 CMSIS内联函数的证据读法

`__disable_irq()` 和 `__enable_irq()` 不是普通外部函数。`Drivers/CMSIS/Include/cmsis_gcc.h` 中的 `__STATIC_FORCEINLINE` 展开为 `__attribute__((always_inline)) static inline`，因此这类编译器层辅助函数通常会在调用点直接展开成指令，而不是在 `.map` 中留下一个可单独查找的函数符号。

所以审查 CMSIS 内联函数时，不能只按“先在 `.map` 找函数名”的顺序判断。更稳妥的证据链应按层次阅读：

| 证据层 | 当前项目证据 | 能证明什么 | 不能证明什么 |
|---|---|---|---|
| 头文件定义 | `cmsis_gcc.h` 把 `__disable_irq()` 写成 `cpsid i`，把 `__enable_irq()` 写成 `cpsie i` | CMSIS/GCC 层的语义来源 | 项目一定调用了这些内联函数 |
| 调用点源码 | `drv_pwmMotors.c` 在 CNT 清零和 CCR 更新前后调用它们 | 项目意图建立短临界区 | 编译后一定保留在这些位置 |
| `.list` 指令上下文 | `PWM_Motor_Init()` / `PWM_Motor_SetAngle()` 附近可见 `cpsid i`、寄存器写入和 `cpsie i` | 当前 Debug 构建把内联函数展开到调用点 | 运行时 `PRIMASK` 原值和恢复结果 |
| `.map` 符号表 | `.text.PWM_Motor_Init`、`.text.PWM_Motor_SetAngle` 有最终地址 | 调用者函数进入最终镜像 | 内联指令的局部上下文 |
| 运行观测 | 调试器读取进入前后 `PRIMASK`【待验证】 | 能确认一次运行中的中断屏蔽状态变化 | 不能由当前仓库静态文件直接证明 |

这条规则也适用于其它 CMSIS `static inline` 或 `always_inline` 辅助函数：`.map` 适合证明“承载它的调用者函数是否进入镜像”，`.list` 才适合证明“内联后的机器指令是否出现在具体调用点”。如果只按函数名搜索 `.map`，很容易把“已内联”误判为“未使用”。

### 8.10 `usbd_conf.c` 中的系统控制访问

`USB_DEVICE/Target/usbd_conf.c` 的挂起回调中，在特定条件下写 `SCB->SCR`。这属于系统控制块寄存器访问。

本章只把它作为 CMSIS 访问索引记录；设备中间件与通信支线的细节留到后续章节。

这次写入使用 `SCB_SCR_SLEEPDEEP_Msk` 和 `SCB_SCR_SLEEPONEXIT_Msk`。前者对应深睡眠控制位，后者对应异常返回后睡眠行为控制位。源码中这段代码位于 `hpcd->Init.low_power_enable` 条件内，而且当前 `USBD_LL_Init()` 将 `hpcd_USB_FS.Init.low_power_enable` 设置为 `DISABLE`，因此不能把它写成“USB 一定进入 STOP 模式”；只能写成“低功耗开关满足时会设置系统控制位”。

### 8.11 本节证据边界

本节只根据当前仓库说明文件、函数、宏、变量和调用关系。运行时频率、外部硬件表现、主机侧现象、传感器方向、电机响应或真实控制效果仍需调试记录、日志或仓库外实测证据；缺少证据时保持【待验证】。

### 8.12 构建产物与权威定义证据边界

CMSIS 章节要区分三类证据，避免把“头文件能定义”误写成“硬件已正确运行”。

第一类是权威定义证据。`Drivers/CMSIS/Include/core_cm3.h` 能证明 `SysTick_Type`、`DWT_Type`、`CoreDebug_Type`、`SCB_Type`、`SysTick_Config()` 和 NVIC 内联函数的接口定义存在；`Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xe.h` 能证明 `IRQn_Type`、`__NVIC_PRIO_BITS`、`__Vendor_SysTickConfig`、`TIM_TypeDef`、外设基地址和位掩码属于 STM32F103xE 设备头文件。它们提供的是编译期接口、地址映射和位定义证据，不单独证明某个寄存器在运行时已经按预期写入或产生硬件效果。

第二类是 `.map` 符号进入证据。当前 `Debug/Three-axis_cloud_platformV2.map` 中可以看到 `.text.micros`、`.text.SysTick_Handler`、`.text.DWT_Delay_us`、`.text.SysTick_Config`、`.text.__NVIC_SetPriority`、`.text.__NVIC_EnableIRQ`、`.text.HAL_NVIC_SetPriority` 和 `.text.HAL_NVIC_EnableIRQ` 等条目。这能证明这些函数或内联展开后的代码路径进入了最终链接产物，并能给出它们位于 Flash 中的符号地址。但是 `.map` 不能证明函数一定被执行到，也不能证明 SysTick 中断已经真实触发、DWT 计数器已经递增或 NVIC 配置符合现场需求。

第三类是 `.list` 指令级上下文证据。当前 `Debug/Three-axis_cloud_platformV2.list` 中可以看到 `micros()` 读取 `SysTick->VAL` 和 `SysTick->LOAD`，`main()` 写 `CoreDebug->DEMCR`、`DWT->CYCCNT` 和 `DWT->CTRL`，`SysTick_Handler()` 读取 `DWT->CYCCNT`，`DWT_Delay_us()` 用 `DWT->CYCCNT` 做差值等待，`SysTick_Config()` 写入 `SysTick->LOAD`、`SysTick->VAL` 和 `SysTick->CTRL`，以及 HAL NVIC 包装函数调用 CMSIS NVIC 内联函数。它能把 C 语句和反汇编位置对应起来，证明源码访问已被编译进当前 Debug 构建；但它仍不能替代断点读回、寄存器窗口截图、逻辑分析仪或示波器证据。

因此，本章可以写成“项目使用 CMSIS 名称访问 Cortex-M3 内核资源和 STM32F103xE 外设资源，并且这些访问已进入当前构建产物”。不能写成“微秒计时精度已经满足要求”“DWT 在所有运行条件下可靠工作”“SysTick 实际频率必定等于 1kHz”或“NVIC 优先级设计已经完成现场验证”。这些运行时结论仍保持【待验证】。

## 9. 调试方法

本节按“现象 -> 可能原因 -> 定位方法 -> 验证步骤 -> 解决方案 -> 经验总结”组织。第05章调试的目标是确认 CMSIS 访问点是否存在、是否处于正确上下文、是否有必要的保护边界。

### 9.1 现象与可能原因

- 编译阶段找不到 `TIM2` 或 `DWT`：先检查 include path、`STM32F103xE` 宏和设备头文件选择链。
- `micros()` 时间戳异常：先检查 `HAL_GetTick()` 是否推进，再检查 `SysTick->VAL` 和 `SysTick->LOAD` 的读取。
- `SysTick->LOAD` 与预期不一致：检查 `SystemCoreClock` 是否已被 `HAL_RCC_ClockConfig()` 更新，再检查 SysTick 配置调用时序。
- 72MHz 配置下 `SysTick->LOAD` 不是 `71999`：按 `HAL_SYSTICK_Config(SystemCoreClock / 1000)` 和 `SysTick_Config(ticks)` 重新推导 `ticks = 72000`、`LOAD = ticks - 1`。
- `DWT_Delay_us()` 微秒延时卡住：检查 DWT 是否已经启用，以及 `DWT->CYCCNT` 是否在运行。
- DWT 初始化后仍无计数变化：先确认 `CoreDebug_DEMCR_TRCENA_Msk` 和 `DWT_CTRL_CYCCNTENA_Msk` 写入，再用连续读数确认 `DWT->CYCCNT` 是否递增【待验证】。
- `SystemCoreClock` 停留在 `8000000`：检查 `SystemClock_Config()`、`HAL_RCC_ClockConfig()` 和错误处理路径。
- 看到向量表中有 `SysTick_Handler` 但系统节拍不推进：向量表只证明入口存在，还要检查 SysTick 配置、中断使能和运行时状态。
- PWM 相位或输出异常：先确认定时器 `CNT` 和 `CCR` 写入是否执行，再到后续定时器章节分析配置。
- 连续 `CCR` 写入仍有异常：确认 `__disable_irq()` 只能减少中断插入，不能替代定时器预装载、更新事件和外部波形验证。
- 嵌套临界区后中断状态异常：记录进入前 `PRIMASK`、退出后 `PRIMASK`，检查是否需要从直接 `__enable_irq()` 改成 `__set_PRIMASK(primask)` 恢复原状态。
- USB 挂起时没有进入低功耗路径：先确认 `hpcd->Init.low_power_enable`；当前初始化把 `hpcd_USB_FS.Init.low_power_enable` 设置为 `DISABLE`，所以 `SCB->SCR` 写入只是条件路径证据。
- 系统进入异常或停在错误处理：检查直接寄存器写入是否发生在预期上下文。
- 关键寄存器更新被打断：检查更新前后的中断保护范围。

### 9.2 定位方法：可观察对象

- `.cproject` 中是否定义 `STM32F103xE`，以及 CMSIS 设备头路径是否进入 include path。
- `stm32f1xx.h` 是否在 `STM32F103xE` 分支下包含 `stm32f103xe.h`。
- `stm32f103xe.h` 是否包含 `core_cm3.h` 和 `system_stm32f1xx.h`。
- `core_cm3.h` 中 `__IO`、`__IM`、`__OM`、`__IOM` 是否具备 `volatile` 访问语义。
- `core_cm3.h` 中是否定义 `SysTick`、`DWT`、`CoreDebug`、`SCB`。
- `stm32f103xe.h` 中是否定义 `TIM_TypeDef`、`TIM2`、`TIM3` 和 `TIM4`。
- `startup_stm32f103rctx.s` 中是否存在 `g_pfnVectors`、`Reset_Handler` 和 `.isr_vector`。
- `system_stm32f1xx.c` 中是否存在 `SystemInit()`、`SystemCoreClock` 和 `SystemCoreClockUpdate()`。
- `main.c` 中 `micros()` 是否读取 `SysTick->VAL` 和 `SysTick->LOAD`。
- `main.c` 中 `HAL_SYSTICK_Config(SystemCoreClock / 1000)` 是否发生在系统时钟配置之后。
- `main.c` 中是否在使用 DWT 前设置 `CoreDebug->DEMCR` 和 `DWT->CTRL`。
- `mpu6050Calibration.c` 中 `DWT_Delay_us()` 是否使用 `DWT->CYCCNT`。
- `drv_pwmMotors.c` 中定时器 `CNT` 和 `CCR` 写入是否位于可解释的更新区域。
- `drv_pwmMotors.c` 中关键写入前后是否使用 `__disable_irq()` 和 `__enable_irq()`。
- `usbd_conf.c` 中 `SCB->SCR` 写入是否位于挂起回调边界内。
- `usbd_conf.c` 中 `hpcd_USB_FS.Init.low_power_enable` 当前是否为 `DISABLE`，避免把条件代码写成默认运行行为。
- `stm32f103xe.h` 中 `TIM3_BASE` 和 `CCR2` 偏移是否能推出 `TIM3->CCR2 = 0x40000438`。
- `core_cm3.h` 中 `SCS_BASE`、`SysTick_BASE`、`SCB_BASE`、`DWT_BASE` 和 `CoreDebug_BASE` 是否与教材地址链一致。

### 9.3 验证步骤：调试记录

- 记录宏选择链：`STM32F103xE -> stm32f1xx.h -> stm32f103xe.h -> core_cm3.h`。
- 记录寄存器名称、访问文件、访问语句、执行上下文和观察结论。
- 记录地址推导：外设地址按 `PERIPH_BASE -> TIMx_BASE -> 成员偏移` 推，内核地址按 `SCS_BASE/DWT_BASE/CoreDebug_BASE -> CMSIS 结构体` 推。
- 记录 `SystemCoreClock` 在 `SystemClock_Config()` 前后、`HAL_SYSTICK_Config()` 前后的值。
- 记录 `SysTick->LOAD` 是否等于 `SystemCoreClock / 1000 - 1`，并记录 `SysTick->VAL` 和 `DWT->CYCCNT` 是否符合变化预期。
- 记录临界区进入前后的 `PRIMASK`，尤其是未来出现嵌套临界区或外层已关中断时。
- 对 DWT、SysTick、TIM 和 SCB 访问分别建记录，不把不同内核/外设资源混成同一类问题。
- 对直接寄存器写入导致的异常，先记录断点位置和上下文，再进入对应后续章节分析外设配置。

### 9.4 解决方案：先证明访问链

先证明宏选择、设备头、CMSIS 结构体和源码访问语句都成立，再用 `.list` 或断点确认该访问点进入当前构建和当前执行路径。只看到头文件定义，不能证明代码已经运行。

### 9.5 解决方案：把资源分开验证

DWT、SysTick、NVIC、SCB 和 TIM 属于不同资源。调试时分别记录寄存器、上下文、期望值和运行证据，避免把微秒计时、系统节拍、临界区保护、低功耗路径和 PWM 波形混成一个问题。

### 9.6 经验总结：调试边界

当前仓库能证明 CMSIS 访问点和直接寄存器写入位置。计数精度、异常触发原因、PWM 实际波形和低功耗行为需要断点、寄存器截图、示波器或日志证据；缺少证据时保持【待验证】。

## 10. 常见问题

### 1. CMSIS 和 HAL 是什么关系？

HAL 是更高层的外设抽象接口；CMSIS 提供更底层的内核与寄存器访问名称。项目可以同时使用两者，而且本项目确实同时出现 HAL 初始化调用和 CMSIS 风格的寄存器访问。

教材不能把它们写成互斥关系。更合理的理解是：HAL 负责大量标准初始化和外设操作，CMSIS 负责暴露芯片、内核和寄存器命名，项目在 DWT、SysTick、TIM 计数器和 USB 底层支持中会直接接触这些名称。

### 2. 直接写寄存器是不是一定比 HAL 好？

不是。直接访问更明确、开销更低，但也更依赖对寄存器和时序的理解。项目只在需要精细控制的位置使用这种方式。

直接访问是否合适，要看它解决的问题。
比如 `DWT->CYCCNT` 适合微秒级计数，`TIMx->CCR` 适合直接观察或写入 PWM 比较值。
但普通外设初始化仍由 HAL 生成代码承担。
教材记录的是当前项目选择，不把它拔高成通用优劣结论。

### 3. 为什么第05章不展开 DWT 计时原理？

因为 DWT 已经安排在第10章。本章只建立 CMSIS 访问前置，避免跳过教学顺序。

本章只需要读者知道 `DWT` 和 `CoreDebug` 这些名字来自 CMSIS，并能在源码中定位访问语句。至于为什么要启用、如何换算微秒、延时精度如何验证，都需要等系统时钟和 SysTick 前置讲完后再进入第10章。

### 4. 为什么第05章不展开定时器 PWM 细节？

因为通用定时器 PWM 和多定时器三相输出在后续章节。本章只说明项目中存在 `TIMx->CNT` 和 `TIMx->CCR` 直接访问。

`TIMx->CNT` 和 `TIMx->CCR` 在第05章只是 CMSIS 直接访问的例子。它们的频率来源、通道含义、PWM 模式、CCR 到电机相位的关系，都需要第12章、第29章和第30章逐步建立。

### 5. 为什么有些 CMSIS 访问出现在中间件支持文件里？

因为中间件底层也可能需要访问系统控制资源。本章只记录访问位置，不把对应中间件功能提前写成主线。

例如 USB 支持文件中出现系统控制或内存相关访问，只能说明中间件底层依赖 MCU 资源。是否成功枚举、是否出现虚拟串口、是否承载业务协议，要等 USB 章节结合初始化链路和主机侧证据再判断。

### 6. `TIM2` 是变量吗？

不是普通变量。`TIM2` 是设备头文件把 `TIM2_BASE` 转换为 `TIM_TypeDef *` 的宏。`TIM2->CNT` 表示按定时器寄存器结构访问固定外设地址。

这也是为什么教材必须同时讲 `core_cm3.h` 和 `stm32f103xe.h`：内核资源来自前者，STM32 外设资源来自后者。

### 7. `__IO` 是否保证原子性？

不保证。`__IO` 和 `__IOM` 的关键作用是 `volatile` 访问语义，避免编译器误优化寄存器读写。

它不能保证多个寄存器写入不可分割，也不能保证外设状态正确。项目在多路 `CCR` 写入前后关闭中断，是额外的临界区保护，不是 `__IO` 自带的效果。

### 8. `SystemCoreClock` 是否一定等于真实 HCLK？

不一定。它是软件变量，默认值为 `8000000`，后续可由 `SystemCoreClockUpdate()` 或 HAL 时钟配置路径更新。

源码能证明项目使用它配置 SysTick 和 DWT 延时换算，但真实 HCLK 还依赖晶振、PLL 配置、HAL 更新路径和硬件实际状态。没有测量证据时保持【待验证】。

### 9. 向量表中有中断入口，是否证明中断已经发生？

不能。向量表证明的是链接入口和默认处理关系。是否发生中断，要结合外设配置、中断使能、NVIC 状态、断点或日志观察。

例如 `g_pfnVectors` 中出现 `SysTick_Handler`，只能说明 SysTick 中断有入口地址；节拍是否真实推进，还要看 `SysTick->CTRL`、`HAL_IncTick()` 调用和运行时观察。

### 10. `TIM3->CCR2` 为什么能推出固定地址？

因为 `TIM3` 不是普通变量，而是设备头文件把 `TIM3_BASE` 转换成 `TIM_TypeDef *` 的宏。`TIM3_BASE = 0x40000400`，`CCR2` 在 `TIM_TypeDef` 中的偏移是 `0x38`，所以 `TIM3->CCR2` 对应 `0x40000438`。

这个推导只能证明 C 表达式对应的总线地址，不能证明 PWM 输出正确。输出波形、相位关系和电机响应仍要到定时器和执行器章节结合实测证据分析。

### 11. 为什么 72MHz 下 `SysTick->LOAD` 是 `71999`？

`main.c` 调用 `HAL_SYSTICK_Config(SystemCoreClock / 1000)`。如果 `SystemCoreClock = 72000000`，传入 CMSIS `SysTick_Config()` 的 `ticks` 就是 `72000`。

`SysTick_Config()` 写入的是 `SysTick->LOAD = ticks - 1`，所以 reload 值是 `71999`。这个结论是软件配置链推导；真实 HCLK 是否稳定在 72MHz 仍需硬件或调试证据【待验证】。

### 12. `__disable_irq()` 后直接 `__enable_irq()` 有什么风险？

风险在于它不保存进入临界区前的中断屏蔽状态。如果外层已经关闭中断，内层函数退出时直接调用 `__enable_irq()`，可能把外层希望保持关闭的中断重新打开。

当前项目里的相关临界区很短，本章只记录风险边界，不判定已经发生故障。更稳妥的通用写法是保存 `PRIMASK`，临界区结束后用 `__set_PRIMASK(primask)` 恢复原状态。

### 13. `SCB->SCR` 出现在 USB 文件中，是否说明默认会进入低功耗？

不能。`SCB->SCR` 写入位于 `hpcd->Init.low_power_enable` 条件内，而当前 `USBD_LL_Init()` 把 `hpcd_USB_FS.Init.low_power_enable` 设置为 `DISABLE`。

所以这里能证明 USB 支持文件中存在一条条件性的 CMSIS 系统控制访问路径，不能证明默认运行时一定写入 `SCB->SCR`，也不能证明设备一定进入 STOP 或低功耗状态。

## 11. 实践任务

开始任务前，先回到本章第8节定位 CMSIS 内核结构体、设备寄存器映射和项目中的直接寄存器访问；第9节提供寄存器访问调试顺序。

任务一至任务四属于基础定位；任务五至任务九属于启动、时钟和项目调用证据。
任务十至任务十五属于寄存器地址、临界区和章节边界确认。

任务一：确认设备选择宏链路。

在 `.cproject` 中找到 `STM32F103xE`，再到 `stm32f1xx.h` 中找到包含 `stm32f103xe.h` 的分支。
验收依据是宏选择表包含编译宏、选择文件、设备头文件和缺失宏时的可能后果。

任务二：确认内核外设结构体定义。

在 `core_cm3.h` 中找到 `SysTick`、`DWT`、`CoreDebug` 和 `SCB` 的定义位置。
验收依据是内核定义表包含名称、定义类型、来源文件和用途提示。

任务三：确认芯片外设寄存器定义。

在 `stm32f103xe.h` 中找到 `TIM_TypeDef`、`TIM2`、`TIM3` 和 `TIM4` 的定义位置。
验收依据是设备定义表包含名称、定义类型、来源文件和对应外设。

任务四：确认 `volatile` 访问语义。

在 `core_cm3.h` 中找到 `__IO`、`__IOM` 的定义，在 `stm32f103xe.h` 中找到 `TIM_TypeDef` 对 `CNT` 和 `CCR` 的使用。
验收依据是权限语义表区分“防编译器优化”“原子性保护”和“硬件正确性验证”。

任务五：定位启动向量表与复位入口。

在 `startup_stm32f103rctx.s` 中找到 `g_pfnVectors`、`Reset_Handler`、`.data` 初始化、`.bss` 清零和跳转 `main()` 的位置。
验收依据是启动链路图说明向量表能证明入口存在，但不能证明中断运行。

任务六：定位 `SystemCoreClock` 更新链路。

在 `system_stm32f1xx.c` 中找到 `SystemCoreClock`、`SystemInit()` 和 `SystemCoreClockUpdate()`。
再到 `main.c` 中找到 `HAL_SYSTICK_Config(SystemCoreClock / 1000)`。
验收依据是时钟变量表区分默认值、更新路径、SysTick 依赖和真实 HCLK【待验证】边界。

任务七：定位 `micros()` 的 SysTick 读取。

在 `main.c` 中找出 `micros()` 对 SysTick 寄存器的读取。
验收依据是时间戳记录表包含读取表达式、变量用途、计算位置和输出含义。

任务八：定位 DWT 计数器启用。

在 `main.c` 中找出启用 DWT 周期计数器的三条语句。
验收依据是 DWT 启用顺序表按执行顺序列出三个寄存器写入和各自动作。

任务九：定位 DWT 延时判断。

在 `mpu6050Calibration.c` 中找出 `DWT_Delay_us()` 的计数差判断。
验收依据是延时依据表记录起始计数、当前计数、差值条件和非毫秒节拍结论。

任务十：定位 TIM 直接寄存器写入。

在 `drv_pwmMotors.c` 中找出 `CNT` 清零和 `CCR` 写入位置。
验收依据是寄存器写入表包含寄存器、写入位置、所属定时器和 HAL 边界结论。

任务十一：确认 USB 低功耗寄存器边界。

在 `usbd_conf.c` 中找出 `SCB->SCR` 写入位置，并说明本章为什么不展开设备中间件细节。
同时找到 `hpcd_USB_FS.Init.low_power_enable = DISABLE`，说明当前默认配置下该写入不应被直接写成必然执行。
验收依据是章节边界表包含 CMSIS 访问证据、条件开关、USB 中间件保留项和后续章节位置。

任务十二：计算 `TIM3->CCR2` 的总线地址。

从 `PERIPH_BASE`、`APB1PERIPH_BASE`、`TIM3_BASE` 和 `TIM_TypeDef.CCR2` 偏移出发，手算 `TIM3->CCR2` 对应地址。
验收依据是地址推导过程写出 `0x40000000 + 0x400 + 0x38 = 0x40000438`，并说明这只证明地址映射，不证明 PWM 波形。

任务十三：复核 SysTick reload 计算。

在 `main.c` 中定位 `HAL_SYSTICK_Config(SystemCoreClock / 1000)`，在 `core_cm3.h` 中定位 `SysTick_Config()`。
验收依据是计算表说明 `SystemCoreClock = 72000000` 时 `ticks = 72000`、`SysTick->LOAD = 71999`，并标注真实 HCLK【待验证】。

任务十四：比较两种临界区恢复方式。

在 `drv_pwmMotors.c` 中记录当前 `__disable_irq()` / `__enable_irq()` 包围范围，再写出 PRIMASK 保存恢复伪代码。
验收依据是对比表说明当前写法的前提、嵌套临界区风险，以及 `__get_PRIMASK()` / `__set_PRIMASK()` 的恢复语义。

任务十五：验证 DWT 计数器启用证据边界。

在 `main.c` 中定位 `CoreDebug_DEMCR_TRCENA_Msk`、`DWT->CYCCNT = 0` 和 `DWT_CTRL_CYCCNTENA_Msk` 三步，再说明为什么仓库源码只能证明“尝试启用”。
验收依据是 DWT 证据表区分源码写入、寄存器读回、计数递增和硬件实测【待验证】。

实践边界：

当前任务优先形成表格、链路图、搜索记录和计算过程。涉及 IDE 现场、构建日志、断点数值、外部波形、主机侧结果或硬件响应时，若没有截图、日志或仓库外实测证据，结论保持【待验证】。

## 12. 思考题

1. 为什么 `SysTick->VAL` 和 `HAL_GetTick()` 可以在同一个时间函数中同时出现？
2. 如果没有先启用 DWT 周期计数器，`DWT_Delay_us()` 可能出现什么问题？
3. 为什么多个 `TIMx->CCR` 写入前后要考虑中断打断风险？
4. 直接寄存器访问和 HAL API 的边界应该如何根据源码调用位置、执行上下文和后续章节证据判断？
5. 为什么本章只建立访问索引，而不展开每个寄存器的完整功能？
6. 如果教材看到一次直接寄存器写入，应如何判断它是当前主线逻辑、初始化辅助，还是调试工具？
7. 如果没有断点、寄存器截图或外部波形证据，哪些 CMSIS 访问结论仍应保持【待验证】？
8. 为什么保存并恢复 `PRIMASK` 比简单地先关中断、后开中断更适合嵌套临界区？
9. 为什么 `SCB->SCR` 代码存在，不等于当前默认运行一定进入低功耗？

## 13. 本章总结

本章建立了三轴云台项目中的 CMSIS 与直接寄存器访问索引。

已经确认的结论是：

- `core_cm3.h` 提供 Cortex-M3 内核资源访问入口。
- `cmsis_gcc.h` 提供 `__disable_irq()`、`__enable_irq()`、`__get_PRIMASK()` 和 `__set_PRIMASK()` 的编译器内联入口。
- `.cproject` 中的 `STM32F103xE` 决定 `stm32f1xx.h` 选择 `stm32f103xe.h`。
- `stm32f103xe.h` 提供 STM32F103xE 设备外设寄存器访问入口。
- `TIM3->CCR2` 可由 `TIM3_BASE` 和 `CCR2` 偏移推导到 `0x40000438`。
- `startup_stm32f103rctx.s` 提供 `g_pfnVectors`、`Reset_Handler` 和默认中断入口关系。
- `system_stm32f1xx.c` 提供 `SystemInit()`、`SystemCoreClock` 和 `SystemCoreClockUpdate()`。
- `main.c` 直接访问 `SysTick`、`CoreDebug` 和 `DWT`。
- `main.c` 使用 `SystemCoreClock / 1000` 配置 SysTick；72MHz 意图配置下可推导 `SysTick->LOAD = 71999`。
- `mpu6050Calibration.c` 使用 `DWT->CYCCNT` 实现微秒级等待。
- `drv_pwmMotors.c` 直接写定时器 `CNT` 和 `CCR`。
- `usbd_conf.c` 在特定回调中保留条件性的 `SCB->SCR` 写入路径，但当前 `low_power_enable` 初始化为 `DISABLE`。
- 本章只建立访问边界，DWT、SysTick、定时器和中间件细节分别留到后续章节。

本章待验证分类：

| 类别 | 已由本章证明 | 仍保持【待验证】 |
|---|---|---|
| 访问索引边界 | 本章定位了项目直接访问内核寄存器、外设寄存器和 CMSIS 内联函数的位置 | 不逐项推导这些寄存器的完整行为，DWT、SysTick、定时器和 USB 中间件细节留给后续章节 |
| 调用路径边界 | 源码能证明某处存在寄存器读写或 CMSIS 函数调用 | 该写入属于主线、初始化辅助还是诊断工具，仍需结合后续调用路径、执行上下文和构建产物判断 |
| `volatile` 与原子性边界 | `__IO` 和 `__IOM` 证明寄存器访问具备 `volatile` 语义 | 不能据此证明多寄存器更新原子、PWM 波形正确、外设状态正确或运行时访问顺序满足业务需求 |
| 临界区边界 | `__disable_irq()` / `__enable_irq()` 能建立短临界区，当前项目可定位其包围范围 | 该写法不保存进入前中断状态；嵌套场景仍应优先考虑 `PRIMASK` 保存恢复并通过现场路径验证 |
| 软件结构与真实硬件边界 | 向量表、`SystemCoreClock`、`SysTick_Config()` 和 CMSIS 地址映射能证明软件结构与配置意图 | 不能证明中断已经发生、真实 HCLK 已经测得、DWT 已实际递增或目标板寄存器状态与软件推导完全一致 |

下一章可以进入系统时钟树，因为内核访问和时间基准入口已经建立，后续才能解释这些计数值和时钟配置之间的关系。

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

- `Core/Src/main.c`
- `Core/Inc/main.h`
- `Core/Inc/stm32f1xx_hal_conf.h`
- `Core/Src/system_stm32f1xx.c`
- `Core/Src/stm32f1xx_it.c`
- `Core/Startup/startup_stm32f103rctx.s`
- `Drivers/CustomDrivers/Src/mpu6050Calibration.c`
- `Drivers/CustomDrivers/Src/drv_pwmMotors.c`
- `USB_DEVICE/Target/usbd_conf.c`
- `.cproject`
- `Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_def.h`
- `Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c`
- `Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c`
- `Drivers/CMSIS/Include/core_cm3.h`
- `Drivers/CMSIS/Include/cmsis_gcc.h`
- `Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f1xx.h`
- `Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xe.h`

符号与函数证据：

- `micros()`
- `DWT_Delay_us()`
- `PWM_Motor_Init()`
- `PWM_Motor_SetAngle()`
- `HAL_PCD_SuspendCallback()`
- `SysTick_Handler()`
- `HAL_IncTick()`
- `STM32F103xE`
- `__CM3_REV`
- `__NVIC_PRIO_BITS`
- `__Vendor_SysTickConfig`
- `__IO`
- `__IM`
- `__OM`
- `__IOM`
- `SysTick->VAL`
- `SysTick->LOAD`
- `SysTick_Config()`
- `SysTick_LOAD_RELOAD_Msk`
- `CoreDebug->DEMCR`
- `CoreDebug_DEMCR_TRCENA_Msk`
- `DWT->CYCCNT`
- `DWT->CTRL`
- `DWT_CTRL_CYCCNTENA_Msk`
- `TIMx->CNT`
- `TIMx->CCR`
- `SCB->SCR`
- `SCB_SCR_SLEEPDEEP_Msk`
- `SCB_SCR_SLEEPONEXIT_Msk`
- `TIM_TypeDef`
- `PERIPH_BASE`
- `APB1PERIPH_BASE`
- `TIM2_BASE`
- `TIM3_BASE`
- `TIM4_BASE`
- `SCS_BASE`
- `SysTick_BASE`
- `SCB_BASE`
- `DWT_BASE`
- `CoreDebug_BASE`
- `TIM2`
- `TIM3`
- `TIM4`
- `__disable_irq()`
- `__enable_irq()`
- `PRIMASK`
- `__get_PRIMASK()`
- `__set_PRIMASK()`
- `g_pfnVectors`
- `Reset_Handler`
- `SystemInit()`
- `SystemCoreClock`
- `SystemCoreClockUpdate()`
- `HAL_RCC_ClockConfig()`
- `HAL_SYSTICK_Config(SystemCoreClock / 1000)`
- `hpcd_USB_FS.Init.low_power_enable`
- `.text.micros`
- `.text.SysTick_Handler`
- `.text.DWT_Delay_us`
- `.text.SysTick_Config`
- `.text.__NVIC_SetPriority`
- `.text.__NVIC_EnableIRQ`

质量自检：

- P0 事实错误：通过
- P1 依赖断层：通过
- P2 逻辑连贯：通过
- P3 项目证据：通过
- P4 原理展开：通过
- P5 调试实践：通过
- P6 表达统一：通过

---
> 导航：上一章：[第04章_HAL工程裁剪与构建产物](第04章_HAL工程裁剪与构建产物.md) ｜ 下一章：[第06章_系统时钟树](第06章_系统时钟树.md)
