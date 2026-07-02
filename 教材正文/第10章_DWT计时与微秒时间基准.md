# 第10章 DWT计时与微秒时间基准

> 导航：上一章：[第09章_中断系统与SysTick节拍](第09章_中断系统与SysTick节拍.md) ｜ 下一章：[第11章_Newlib适配与UART调试输出](第11章_Newlib适配与UART调试输出.md)

## 1. 本章目标

- 理解项目为什么同时需要毫秒节拍、微秒时间戳和微秒级等待。
- 区分 `micros()` 的 SysTick/HAL tick 时间戳路径与 `DWT_Delay_us()` 的 DWT 周期计数路径。
- 看懂项目中 `CoreDebug->DEMCR`、`DWT->CYCCNT` 和 `DWT->CTRL` 的启用关系。
- 能从 `main.c`、`mpu6050Calibration.c`、`mpu6050.c` 和 CMSIS 头文件追踪微秒计时证据。

本章阅读分层：

| 阅读层次 | 建议范围 | 适合读者 |
|---|---|---|
| 【必须掌握】 | 第1节到第5节，第13节总结 | 需要理解 `micros()` 时间戳路径、`DWT_Delay_us()` 忙等路径和两者边界的读者 |
| 【工程深化】 | 第6节到第8.7.1节，第9节调试方法 | 需要维护 DWT启用、`CYCCNT` 读写、微秒等待、MPU6050采样等待和计时证据链的读者 |
| 【拓展阅读】 | 第5节中的回绕与误差说明，第8.6节到第8.7.1节，第10节到第12节 | 需要进一步理解计数回绕、整数换算误差、CMSIS头文件证据和 TIM6/DWT取舍的读者 |
| 【证据与验证】 | 第8节、第9节、章节尾部固定检查，以及所有 `【待验证】` 项 | 需要审查 `main.c` 中 DWT 启用、`micros()` 时间戳、`DWT_Delay_us()` 忙等、温度补偿和静态零偏采样等待路径、CMSIS 定义、`.map/.list` 构建产物、`CYCCNT` 运行读回、GPIO 翻转计时、示波器或逻辑分析仪证据的读者 |

如果只是沿微秒时间基础学习，可以先抓住“SysTick细分得到时间戳 -> DWT周期计数实现忙等 -> 两者都依赖时钟前提但用途不同”这条链；验证延时精度、采样周期或控制循环抖动时，再回到证据边界和调试方法小节。

## 2. 前置知识

- CMSIS寄存器与内核访问
- 系统时钟树
- SysTick系统节拍

第05章已经说明 CMSIS 如何暴露 `SysTick`、`DWT` 和 `CoreDebug` 等内核资源。

第06章已经确认 `SystemCoreClock` 的来源，第09章已经建立 1ms SysTick 和 HAL 毫秒时基。本章在这些前提上解释项目如何获得比 1ms 更细的时间信息。

本章不展开 MPU6050 传感器标定算法、不展开 500Hz 实时控制循环的姿态与电机计算，也不展开后续 PID 和电机硬件诊断。

## 3. 问题背景

第09章已经说明，项目通过 SysTick 建立 1ms 系统节拍，并由 `frame_500Hz` 驱动主循环中的 500Hz 实时控制循环入口。可是三轴云台项目还需要更细的时间观察能力：

- 500Hz 实时控制循环的理想周期是 2ms，计算 `dt500Hz` 时不能只依赖整数毫秒的粗略差值。
- 运行时间分析需要知道某段逻辑消耗了多少微秒。
- 传感器标定和静态零偏采样中，需要在连续采样之间插入约 1000us 的等待。

因此，项目中出现了两条微秒相关路径。

第一条是 `micros()`：它读取 `HAL_GetTick()`、`SysTick->VAL` 和 `SysTick->LOAD`，把 HAL 毫秒计数和当前 SysTick 倒计数位置组合成微秒时间戳。

第二条是 `DWT_Delay_us()`：它读取 `DWT->CYCCNT`，把目标微秒数换算成 CPU 周期数，并等待周期差达到目标值。

两者都服务于“微秒级时间”，但来源不同、用途不同。第10章要先把这条边界讲清楚。

## 4. 核心概念

- DWT周期计数器：Cortex-M3 内核调试组件中的周期计数资源，本项目通过 `DWT->CYCCNT` 使用它。
- `CYCCNT`：DWT 中的 CPU 周期计数寄存器。
- `CoreDebug->DEMCR`：CoreDebug 调试异常与监控控制寄存器，本项目通过设置跟踪使能位，为 DWT 周期计数打开前置条件。
- `DWT_CTRL_CYCCNTENA_Msk`：CMSIS 中用于启用 `CYCCNT` 的位掩码。
- `DWT_CTRL_NOCYCCNT_Msk`：CMSIS 中表示该实现不支持周期计数器的能力标志，可作为健壮性检查点。
- 微秒时间戳：用于记录某个时刻的微秒级数值，本项目由 `micros()` 返回。
- 微秒级等待：让代码等待指定微秒数，本项目由 `DWT_Delay_us()` 实现。
- 忙等：CPU 在循环中反复读取条件，直到条件满足才继续执行；`DWT_Delay_us()` 属于这类等待。
- 计数回绕：固定宽度计数器超过最大值后从 0 重新开始，DWT 的 `CYCCNT` 是 32 位计数器。
- `SystemCoreClock`：核心时钟频率，`DWT_Delay_us()` 用它把微秒换算成周期数。
- 时间戳回绕：`micros()` 返回 `uint32_t` 微秒时间戳，超过 32 位范围后会回绕。
- 整数换算误差：`DWT_Delay_us()` 使用 `SystemCoreClock / 1000000` 的整数结果，若核心频率不是 1MHz 的整数倍，会产生截断误差。

这些概念服务于正式知识点 `DWT周期计数器`，不新增结构外知识点。

## 5. 工作原理

本项目中的微秒时间可以按两条路径理解。

第一条路径是 SysTick 细分时间戳。

`HAL_GetTick()` 提供当前毫秒数，`SysTick->VAL` 提供当前 1ms 周期内已经倒计到哪里。

`SysTick->LOAD + 1` 表示 1ms 内总计数长度。

因此，`micros()` 的核心换算可以写成：

```text
tms = SysTick->LOAD + 1
elapsed_us_in_current_ms = ((tms - SysTick->VAL) * 1000) / tms
timestamp_us = HAL_GetTick() * 1000 + elapsed_us_in_current_ms
```

在项目 72MHz、1ms SysTick 配置下，`tms` 期望为 `72000`。
1us 对应约 `72000000 / 1000000 = 72` 个核心时钟计数。
由于函数返回 `uint32_t` 微秒整数，除法结果会截断小于 1us 的分数部分；
它适合项目当前的微秒级时间差观察，但不是纳秒级或周期级测量函数。

`micros()` 先读取一次毫秒数和 SysTick 倒计数，再读取第二次，用两次毫秒数判断是否刚好跨过 tick 边界。

如果两次毫秒数不同，说明读取过程中发生了 SysTick 更新，函数使用新的毫秒数和新的倒计数值。如果两次毫秒数相同，说明仍在同一个毫秒周期内，函数使用第一次读到的倒计数值。这样可以降低跨边界读取造成的时间戳跳变风险。

把源码分支继续展开，可以得到两个可教学单元：

```text
m1 == m0:
  micros = m0 * 1000 + ((tms - u0) * 1000) / tms

m1 != m0:
  micros = m1 * 1000 + ((tms - u1) * 1000) / tms
```

这里的双采样保护依据是 `HAL_GetTick()` 的毫秒值是否变化。它没有关闭中断形成原子快照，
也没有读取 `SysTick->CTRL.COUNTFLAG` 判断是否已有未处理的 SysTick 计数到零事件。
因此它适合在当前主循环和 SysTick ISR 的短时间差场景中降低边界读数风险，
但不能被写成“任意上下文都严格单调、无抖动、原子一致”的硬实时微秒时钟。

由于返回类型是 `uint32_t`，`micros()` 的微秒时间戳会按 32 位无符号数回绕：

```text
T_micros_wrap = 2^32 us = 4294967296 us
              ≈ 4294.967 s
              ≈ 71.58 min
```

这不等于程序只能运行 71.58 分钟。项目计算 `deltaTime500Hz = currentTime - previous500HzTime`
时使用无符号差值，只要两次采样间隔远小于回绕周期，就可以跨过一次回绕仍得到正确短时间差。
但教材不能把 `micros()` 返回值写成无限单调增长的绝对时间。

第二条路径是 DWT 周期等待。

`DWT_Delay_us()` 先记录当前 `DWT->CYCCNT`，再根据 `SystemCoreClock / 1000000` 把目标微秒数换算成 CPU 周期数。随后它循环读取当前 `DWT->CYCCNT`，直到当前计数与起始计数之差达到目标周期数。

在当前 72MHz 核心时钟下：

```text
cycles_per_us = SystemCoreClock / 1000000 = 72
DWT_Delay_us(1000) -> ticks = 1000 * 72 = 72000 cycles
```

这里 `SystemCoreClock / 1000000` 是整数除法。当前 72MHz 正好能整除 1MHz，
所以 `cycles_per_us = 72` 没有除法截断。若后续项目改为不能整除 1MHz
的核心频率，这个写法会把每微秒周期数向下取整，等待时间将出现系统性偏短风险。

`DWT->CYCCNT` 是 32 位计数器。若按 72MHz 连续递增，理论回绕时间约为：

```text
T_wrap = 2^32 / 72000000 ≈ 59.65 s
```

项目当前调用的是 1000us 级短等待，远小于回绕周期。代码使用无符号差值
`DWT->CYCCNT - start`，对短等待跨过一次计数边界的情况也更稳健。
但它没有超时退出，也没有检查乘法溢出，因此不适合直接扩展为长时间等待接口。

乘法溢出边界也可以估算：

```text
ticks = us * 72
uint32_t 最大值 = 4294967295
us_max_no_overflow = floor(4294967295 / 72) = 59652323 us
                   ≈ 59.65 s
```

这只是算术不溢出的上限，不是推荐等待这么久。忙等 59s 会让 CPU 长时间不能执行主循环普通任务，
对实时系统几乎不可接受。

这条路径的前提是 DWT 周期计数器必须已经运行。项目在 `main.c` 用户初始化段中执行：

1. 设置 `CoreDebug->DEMCR` 的跟踪使能位。
2. 清零 `DWT->CYCCNT`。
3. 设置 `DWT->CTRL` 的周期计数使能位。

因此，本章的核心关系是：`micros()` 解决“现在是多少微秒”，`DWT_Delay_us()` 解决“等待多少微秒”。

## 6. STM32实现机制

在 STM32F103RCTx 的 Cortex-M3 内核中，DWT 和 CoreDebug 属于 CMSIS 暴露的内核调试资源。项目不通过普通外设初始化函数启用它们，而是在 `main.c` 中直接访问 CMSIS 寄存器。

本项目的 DWT 启用顺序出现在 `main.c` 用户初始化段，位于 SysTick 配置之后、项目业务初始化之前：

- `CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk`
- `DWT->CYCCNT = 0`
- `DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk`

对应到 CMSIS 位级定义：

| 寄存器/位 | 位号 | 项目动作 | 含义 |
| --- | --- | --- | --- |
| `CoreDebug->DEMCR.TRCENA` | 24 | 置 1 | 打开跟踪资源访问前提 |
| `DWT->CYCCNT` | 32 位计数寄存器 | 写 0 | 清空周期计数观察起点 |
| `DWT->CTRL.CYCCNTENA` | 0 | 置 1 | 允许 `CYCCNT` 随核心周期递增 |
| `DWT->CTRL.NOCYCCNT` | 25 | 当前未检查 | 能力标志，1 表示无周期计数器 |

`Drivers/CMSIS/Include/core_cm3.h` 提供了这些符号的定义：

- `DWT_Type` 中包含 `CTRL` 和 `CYCCNT`。
- `CoreDebug_Type` 中包含 `DEMCR`。
- `DWT_CTRL_CYCCNTENA_Msk` 表示周期计数使能位。
- `DWT_CTRL_NOCYCCNT_Msk` 表示没有周期计数器能力的标志位。
- `CoreDebug_DEMCR_TRCENA_Msk` 表示跟踪资源使能位。

`micros()` 虽然也返回微秒级时间戳，但它的直接寄存器来源是 `SysTick->VAL` 和 `SysTick->LOAD`，不是 `DWT->CYCCNT`。这一点必须和 `DWT_Delay_us()` 区分。

更完整的工程化 DWT 初始化通常还会检查 `DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk`，
以判断当前内核实现是否真的提供周期计数器。当前项目代码没有做这一步，
所以本章只能确认“项目按 Cortex-M3 DWT 路径启用了 `CYCCNT`”，不能把它写成已经经过运行时能力检查。

项目还初始化了 TIM6：`.ioc` 中 TIM6 预分频值为 35，`tim.c` 中 `Period` 为 65535。
在 APB1 定时器时钟 72MHz 条件下，若 TIM6 被启动，它的计数频率将是：

```text
f_tim6_cnt = 72000000 / (35 + 1) = 2000000 Hz
T_tim6_cnt = 0.5 us
T_tim6_overflow = 65536 / 2000000 ≈ 32.768 ms
```

但当前源码检索到的是 `MX_TIM6_Init()` 和 `__HAL_TIM_SET_COUNTER(&htim6, 0)`，
没有看到项目业务代码调用 `HAL_TIM_Base_Start(&htim6)` 或读取 `__HAL_TIM_GET_COUNTER(&htim6)`。
因此，第10章不能把 TIM6 作为当前 `micros()` 或 `DWT_Delay_us()` 的实际时间来源。
TIM6 的配置意义留到第13章继续判断。

## 7. 项目中的应用

本章对应项目中的微秒时间基础层。

直接相关文件：

- `Core/Src/main.c`
- `Drivers/CustomDrivers/Src/mpu6050Calibration.c`
- `Drivers/CustomDrivers/Inc/mpu6050Calibration.h`
- `Drivers/CustomDrivers/Src/mpu6050.c`
- `Drivers/CMSIS/Include/core_cm3.h`

文件之间的关系是：

- `core_cm3.h` 提供 DWT、CoreDebug 和 SysTick 的 CMSIS 定义。
- `main.c` 启用 DWT 周期计数器，实现 `micros()`，并初始化但未作为本章时间源使用 TIM6。
- `mpu6050Calibration.c` 实现 `DWT_Delay_us()`，并在温度补偿采样中调用它。
- `mpu6050Calibration.h` 声明 `DWT_Delay_us()`，使其他文件可以调用。
- `mpu6050.c` 在静态零偏采样循环中调用 `DWT_Delay_us(1000)`。
- `tim.c` 和 `.ioc` 提供 TIM6 配置证据，但当前不构成 `micros()` 或 `DWT_Delay_us()` 的来源证据。

运行流程上，项目先完成 HAL、系统时钟、SysTick 和外设初始化，再启用 DWT 周期计数器。

这里还要拆开一个容易被跳过的前置：`SystemCoreClock` 不是编译期永远等于 72MHz 的常量。
`Core/Src/system_stm32f1xx.c` 中它的初始定义是 `8000000`，表示复位后默认 HSI 语义；
`SystemClock_Config()` 内部调用 `HAL_RCC_ClockConfig()` 后，HAL 在
`stm32f1xx_hal_rcc.c` 中用 `HAL_RCC_GetSysClockFreq()` 和 AHB 分频表更新
`SystemCoreClock`。因此，当前 72MHz 结论依赖系统时钟配置已经执行并成功返回。

这条边界会同时影响两条微秒路径：`main()` 在 `SystemClock_Config()` 之后又调用
`HAL_SYSTICK_Config(SystemCoreClock / 1000)`，所以 `micros()` 假设的 1ms
SysTick 重装载值依赖更新后的 `SystemCoreClock`；`DWT_Delay_us()` 的
`SystemCoreClock / 1000000` 也依赖同一个运行时变量。`.map` 只能证明
`SystemCoreClock` 位于 `.data`，当前地址为 `0x20000000`；`.list` 能证明
`HAL_RCC_ClockConfig()` 中存在更新写回以及 `main()` 后续读取该变量配置 SysTick。
它们仍不能替代运行时读取 `SystemCoreClock`、`SysTick->LOAD` 和
`DWT->CYCCNT` 的调试证据【待验证】。

随后，`micros()` 可以计算 500Hz 周期时间差和执行耗时。

`DWT_Delay_us()` 可以提供采样间隔等待。

注意，项目中还存在 `sysTickUptime`、`sysTickCycleCounter`、`usTicks` 等时间相关变量。
当前仓库能看到 `sysTickCycleCounter = DWT->CYCCNT` 处在 `stm32f1xx_it.c` 的块注释中，
`usTicks` 也未构成本章两条微秒路径的有效来源。
教材应以实际参与编译和调用的函数为准，而不是以变量名推断当前行为。

## 8. 代码分析

### 8.1 `main.c` 中的 DWT 启用

`main.c` 在用户初始化段中先配置 SysTick，再启用 DWT 周期计数器。它写入 `CoreDebug->DEMCR`，清零 `DWT->CYCCNT`，最后设置 `DWT->CTRL`。

这段代码的项目含义是：后续任何基于 `DWT->CYCCNT` 的等待或测量，都依赖这里已经执行。若这段初始化未执行，`DWT_Delay_us()` 可能无法按预期结束等待。

### 8.2 `micros()` 的时间戳路径

`micros()` 定义在 `main.c` 中。它读取：

- `HAL_GetTick()`
- `SysTick->VAL`
- `SysTick->LOAD`

函数的核心目标是把毫秒 tick 与当前毫秒内的 SysTick 倒计数位置合成微秒时间戳。它不是 DWT 周期计数函数。

`stm32f1xx_it.c` 中还能看到一段已经被块注释包住的旧逻辑，
其中提到读取 `DWT->CYCCNT` 给 `micros()` 使用。
该段当前不参与编译，不能作为当前 `micros()` 实现的有效证据；
本章以 `main.c` 中实际编译的 `micros()` 函数为准。

`micros()` 的精度边界也要讲清楚。它的最小输出单位是 1us，但内部使用整数除法，
所以小于 1us 的周期余量会被截断。它还依赖 `HAL_GetTick()` 正常推进和 SysTick 配置保持 1ms。
如果 SysTick 被暂停、`HAL_IncTick()` 不运行，或者 `SysTick->LOAD` 与实际核心时钟不匹配，
`micros()` 的返回值就不能再被当作可靠的微秒时间戳。

当前实现的跨 tick 保护只比较 `m1` 与 `m0`。若在禁止中断、暂停 HAL tick
或调试停顿等场景下调用，SysTick 当前计数、HAL 毫秒值和真实墙钟之间可能不再保持本章假设的同步关系。
所以本章把 `micros()` 定位为项目内时间差观测工具，而不是替代硬件捕获、逻辑分析仪或严格实时调度器的计量标准。

`micros()` 内部表达式 `HAL_GetTick() * 1000` 也在 `uint32_t` 范围内运行。
因此它的返回值本质上是“32 位微秒计数模 2^32”的时间戳。适合计算相邻帧
和短代码段耗时，不适合直接当作长时间运行的绝对时间记录。

项目在两个位置使用这类时间戳：

- `SysTick_Handler()` 中计算 `deltaTime1000Hz` 和 `executionTime1000Hz`。
- 主循环消费 `frame_500Hz` 时计算 `deltaTime500Hz`、`dt500Hz` 和 `executionTime500Hz`。

这些使用位置说明 `micros()` 服务于时间差测量和运行耗时观察。

### 8.3 `DWT_Delay_us()` 的等待路径

`DWT_Delay_us()` 定义在 `mpu6050Calibration.c` 中。它读取起始 `DWT->CYCCNT`，计算目标周期数 `us * (SystemCoreClock / 1000000)`，然后等待计数差达到目标值。

这段逻辑的输入是目标微秒数，输出不是一个返回值，而是“经过约定的等待时间后继续执行”。它的风险点是：如果 `DWT->CYCCNT` 没有运行，循环条件就可能一直不满足。

它还有五个工程限制：

1. 忙等期间 CPU 被占用，不能执行主循环中的其他普通任务。
2. 中断仍可能打断忙等，因此实际墙钟时间可能大于目标等待时间。
3. `ticks = us * (SystemCoreClock / 1000000)` 对大 `us` 存在乘法溢出风险。
4. `SystemCoreClock / 1000000` 使用整数除法，核心频率不是 1MHz 整数倍时会向下取整。
5. 没有检查 `DWT_CTRL_NOCYCCNT_Msk`，也没有超时逃生路径，初始化失败时可能卡在循环中。

所以本项目把它用于 1000us 级采样间隔是相对合理的短等待用法；
若要等待毫秒到秒级时间，应优先考虑 HAL tick、定时器事件或状态机，而不是长时间 DWT 忙等。

### 8.4 温度补偿采样中的微秒等待

`mpu6050Calibration.c` 中设置 `sampleRate = 1000`，并在冷机采样和热机采样循环中调用 `DWT_Delay_us(sampleRate)`。

这里的项目含义是：温度补偿采样希望在连续读取之间插入约 1000us 的间隔。温度补偿算法本身属于后续传感器标定章节，本章只说明它使用 DWT 微秒等待。

需要注意，`sampleRate = 1000` 在这里是等待参数，不是已经实测的采样频率。
每次循环还包含 `MPU6050_Read_And_Process()`、数据累加和循环控制开销，
所以 2000 次采样总时长必然大于纯等待时间 `2000 * 1000us = 2s`。
实际采样周期仍需用日志、DWT 记录或外部测量验证【待验证】。

同一个标定函数中还存在中间升温等待：串口文本写着“2分钟”，源码注释写着“120秒”，
但真正参与编译的调用是 `HAL_Delay(10000)`。按 HAL 毫秒时基解释，它表示约 10000ms，
也就是约 10s。第10章只把这一点作为时间基准证据边界：采样间隔来自
`DWT_Delay_us(sampleRate)`，中间等待来自 `HAL_Delay()`，两者不能混成同一条 DWT 微秒路径。

### 8.5 静态零偏采样中的微秒等待

`mpu6050.c` 的静态零偏采样循环中调用 `DWT_Delay_us(1000)`。该循环进行 5000 次采样，并在每次采样后等待约 1000us。

本章不分析静态零偏如何计算，只确认：`DWT_Delay_us()` 已经从校准文件扩展为 MPU6050 采样流程中的公共微秒等待工具。

同样，5000 次循环的最小等待时间约为 `5000 * 1000us = 5s`，
但总耗时还要加上传感器读取、温度补偿计算和循环开销。
因此本章只能说明“代码插入了 1000us 等待”，不能证明静态零偏采样总时长精确等于 5s。

### 8.6 CMSIS 头文件证据

`core_cm3.h` 中可以追踪到 `DWT_Type`、`CoreDebug_Type` 和相关位掩码。第05章已经说明 CMSIS 访问边界，第10章进一步说明这些定义如何被本项目用于 DWT 周期计数。

如果只看 `main.c` 中的 `DWT->CYCCNT`，读者会知道项目读写了某个寄存器；如果再追踪到 `core_cm3.h`，读者才能确认它属于 Cortex-M3 内核 DWT 资源。

### 8.7 构建产物证据边界

当前 Debug 构建产物还能把“源码中写了这些逻辑”和“这些逻辑进入了本次镜像”连接起来。

`Debug/Three-axis_cloud_platformV2.map` 中可以看到 `DWT_Delay_us` 被分配到 `.text`，也能看到
`deltaTime500Hz`、`executionTime500Hz` 和 `frame_500Hz` 被分配到 `.bss`。这说明这些函数和全局状态已经进入当前 Debug ELF 的代码段或未初始化数据段，而不是只停留在源码文本中。

`Debug/Three-axis_cloud_platformV2.list` 中可以继续追踪到本次编译后的指令与源码对应关系：

- `main()` 中写入 `CoreDebug->DEMCR`、清零 `DWT->CYCCNT`，并设置 `DWT->CTRL`。
- `main()` 中使用 `micros()` 初始化 `previous500HzTime`，在 500Hz 分支里计算 `deltaTime500Hz`、`dt500Hz` 和 `executionTime500Hz`。
- `mpu6050.c` 中静态零偏采样调用 `DWT_Delay_us(1000)`。
- `DWT_Delay_us()` 的反汇编段读取 `DWT->CYCCNT`，并通过循环等待计数差达到目标值。

`Debug/Core/Src/main.su` 和 `Debug/Core/Src/main.cyclo` 还能给出函数级静态资源条目：

- `micros` 的静态栈使用量为 32 字节，圈复杂度为 2。
- `main` 的静态栈使用量为 56 字节，圈复杂度为 7。

`Debug/Drivers/CustomDrivers/Src/mpu6050Calibration.su` 和对应 `.cyclo` 文件中，`DWT_Delay_us` 的静态栈使用量为 24 字节，圈复杂度为 2。

`Debug/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.su` 和对应 `.cyclo` 文件中，`HAL_GetTick` 的静态栈使用量为 4 字节，圈复杂度为 1。
这条证据补上了 `micros()` 依赖 HAL 毫秒 tick 的函数级资源边界。

这些构建产物证据能证明当前 Debug 镜像包含 DWT 启用、微秒时间戳消费、500Hz 时间差记录和 DWT 忙等函数调用路径，也能证明上述函数在当前编译选项下生成了静态栈与圈复杂度记录。
但它们不能证明板上运行的一定是同一个镜像，也不能证明 `CYCCNT` 在运行时确实递增、`DWT_Delay_us(1000)` 的墙钟等待精确等于 1000us、500Hz 周期没有抖动、采样频率严格达到 1000Hz，或这些函数在中断嵌套、库函数调用和调试器停顿下的完整最坏栈深度。上述运行时结论仍需要调试日志、寄存器观察、示波器或逻辑分析仪等实测证据；缺少证据时保持【待验证】。

#### 8.7.1 `.map/.list` 最终地址与 DWT 运行边界

更细地拆开看，`Debug/Three-axis_cloud_platformV2.map` 给出的不是“源码里存在这些函数”，而是当前 Debug 链接结果中的最终放置位置：`micros` 位于 `.text.micros`，入口地址为 `0x08001458`；`DWT_Delay_us` 位于 `.text.DWT_Delay_us`，入口地址为 `0x080036f0`；`HAL_GetTick` 位于 `.text.HAL_GetTick`，入口地址为 `0x080058f4`。这能证明三条相关函数路径已经进入当前 ELF 的 Flash 代码段。

`Debug/Three-axis_cloud_platformV2.list` 进一步把这些最终地址和源代码行、反汇编指令对应起来：`micros()` 调用 `HAL_GetTick()`，并读取 `SysTick->VAL` 与 `SysTick->LOAD`；`main()` 写入 `CoreDebug->DEMCR`、`DWT->CYCCNT` 和 `DWT->CTRL`，形成 DWT 周期计数启用序列；`main()` 还调用 `micros()` 初始化 `previous500HzTime`，并在 500Hz 分支中计算 `currentTime`、`deltaTime500Hz` 与 `executionTime500Hz`；`mpu6050.c` 的静态零偏采样调用 `DWT_Delay_us(1000)`；`DWT_Delay_us()` 自身读取 `DWT->CYCCNT`，计算 `us * (SystemCoreClock / 1000000)`，并用无符号差值循环等待目标周期数。

因此，`.map/.list` 可以证明当前 Debug 构建包含“SysTick/HAL tick 时间戳路径”“DWT 启用写寄存器路径”“DWT 忙等路径”和“MPU6050 采样等待调用点”。它们不能证明烧录到板上的一定是同一个 ELF，不能证明 `DWT->CYCCNT` 在运行时真实递增，不能证明 `DWT_Delay_us(1000)` 的墙钟等待精确等于 1000us，也不能证明 500Hz 控制分支没有抖动或采样频率严格达到 1000Hz。这些结论需要调试器寄存器回读、连续日志、GPIO 翻转计时、示波器或逻辑分析仪证据；缺少这些证据时继续标为【待验证】。

### 本节证据边界

本节只根据当前仓库说明文件、函数、宏、变量和调用关系。运行时频率、外部硬件表现、主机侧现象、传感器方向、电机响应或真实控制效果仍需调试记录、日志或仓库外实测证据；缺少证据时保持【待验证】。

## 9. 调试方法

本节按“现象 -> 可能原因 -> 定位方法 -> 验证步骤 -> 解决方案 -> 经验总结”组织。第10章调试的目标是确认微秒时间戳和微秒等待的来源没有混淆。

### 9.1 现象与可能原因

- `DWT_Delay_us()` 卡住：先检查 DWT 启用代码是否执行，再检查 `DWT->CYCCNT` 是否在增长。
- 微秒延时明显不准：先检查 `SystemCoreClock` 是否已经由系统时钟配置更新，再检查 `us * (SystemCoreClock / 1000000)` 的换算。
- 更换系统时钟后延时偏短：检查 `SystemCoreClock / 1000000` 是否出现整数截断，必要时应改用更精确的换算方式。
- `micros()` 时间戳异常：先检查 `HAL_GetTick()` 是否推进，再检查 SysTick 1ms 配置和 `SysTick->VAL/LOAD` 读取。
- `micros()` 在调试或临界段中表现异常：检查是否长时间关中断、暂停 HAL tick，或在断点停顿后继续用时间戳推断真实连续运行时间。
- 长时间运行后时间戳变小：检查是否遇到 `uint32_t` 微秒回绕，优先用无符号差值分析短时间间隔。
- 500Hz `dt500Hz` 抖动：先确认 `frame_500Hz` 由第09章的 SysTick 标志触发，再检查 `micros()` 时间差计算；控制算法本身留到后续章节。
- 标定采样间隔异常：先确认 `DWT_Delay_us()` 的调用位置，再进入后续标定章节分析采样逻辑。
- TIM6 相关误判：若只看到 `MX_TIM6_Init()` 或清零 `CNT`，不能直接得出 TIM6 正在提供微秒时基。
- 调试器断点影响：在忙等循环、SysTick 或 500Hz 分支中断住程序，会改变观察到的时间差和采样间隔。

### 9.2 定位方法：可观察对象

- `main.c` 中 DWT 启用代码是否发生在调用 `DWT_Delay_us()` 之前。
- `CoreDebug->DEMCR` 是否设置跟踪使能位。
- `DWT->CYCCNT` 是否在启用前被清零。
- `DWT->CTRL` 是否设置周期计数使能位。
- `DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk` 是否为 0，用于确认周期计数能力。
- `DWT->CYCCNT` 是否在运行状态下持续递增，且暂停断点时是否被调试器停止或扰动。
- `DWT_Delay_us()` 中 `ticks` 是否由 `SystemCoreClock / 1000000` 换算得到。
- `SystemCoreClock` 是否为 72MHz，确认 `cycles_per_us = 72` 是当前项目成立的换算。
- `micros()` 是否读取 `HAL_GetTick()`、`SysTick->VAL` 和 `SysTick->LOAD`。
- `SysTick->LOAD + 1` 是否与当前 `SystemCoreClock / 1000` 匹配。
- `mpu6050Calibration.c` 中两段采样循环是否调用 `DWT_Delay_us(sampleRate)`。
- `mpu6050Calibration.c` 中间升温等待是否实际调用 `HAL_Delay(10000)`，不要只按串口文本或源码注释推断等待时长。
- `mpu6050.c` 中静态零偏采样是否调用 `DWT_Delay_us(1000)`。
- TIM6 是否只是初始化和清零，还是存在启动、读取或中断消费证据。

### 9.3 验证步骤：调试记录

- 记录 DWT 启用语句、`DWT->CYCCNT` 清零与递增现象、`SystemCoreClock` 数值和微秒换算结果。
- 分开记录 `micros()` 的时间戳路径和 `DWT_Delay_us()` 的忙等路径，避免把两类时间基准混为一个结论。
- 对 TIM6 只记录初始化、计数器清零、启动函数、读取函数和中断入口是否存在，不用配置意图替代运行证据。
- 对 500Hz 采样间隔，只记录本章能证明的时间基准证据；控制算法影响留到后续控制循环章节。
- 若没有示波器、逻辑分析仪或连续日志，微秒延时精度和周期抖动只能标记为【待验证】。

### 9.4 解决方案：分清时间戳与忙等延时

`micros()` 的时间戳路径依赖 `HAL_GetTick()`、`SysTick->VAL` 和 `SysTick->LOAD`；`DWT_Delay_us()` 的忙等路径依赖 DWT 启用、`DWT->CYCCNT` 递增和 `SystemCoreClock` 换算。两条路径要分开记录。

### 9.5 解决方案：排除 TIM6 误判

TIM6 初始化或清零 `CNT` 只能证明源码存在相关配置。除非能看到启动、读取、中断消费或调用链证据，否则不能把 TIM6 写成当前微秒时间基准。

### 9.6 经验总结

微秒时间基准调试要同时保留源码路径、寄存器观察和连续运行证据。缺少示波器、逻辑分析仪或连续日志时，微秒延时精度、周期抖动和断点后的真实连续时间都保持【待验证】。

## 10. 常见问题

### 1. `micros()` 是否使用 DWT？

当前项目中不是。`micros()` 使用 `HAL_GetTick()`、`SysTick->VAL` 和 `SysTick->LOAD` 计算微秒时间戳。DWT 主要服务于 `DWT_Delay_us()` 的周期等待。

如果读到 `stm32f1xx_it.c` 中关于 `sysTickCycleCounter = DWT->CYCCNT` 的说明，要先确认它处在块注释内。教材只把实际参与编译的代码作为当前项目行为依据。

这也是为什么“代码里提到 DWT”不等于“当前 `micros()` 在用 DWT”。
当前仓库能证明的是：微秒时间戳和微秒等待被拆成了两条路径，教材要分别观察。

### 2. 为什么已经有 SysTick，还要启用 DWT？

SysTick 已经提供 1ms 系统节拍，并能通过 `SysTick->VAL` 细分出时间戳。
DWT 的优势在于直接按 CPU 周期计数。
当前项目实际把它用于 `DWT_Delay_us()` 的短时间等待，
而执行耗时观察主要由 `micros()` 生成的时间戳完成。
两者各自承担不同任务。

可以把它理解成“时间戳”和“等待器”。
SysTick 提供连续时基，DWT 负责短时间等待。
第09章和第10章合起来，才构成项目里较完整的时间观察链路。

### 3. `DWT_Delay_us()` 为什么要用 `SystemCoreClock`？

因为 `DWT->CYCCNT` 记录的是 CPU 周期数，不是微秒数。要等待指定微秒数，必须用核心时钟频率把微秒换算成周期数。

如果系统时钟变化而 `SystemCoreClock` 没同步，等待时间就会偏差。
所以第10章必须依赖第06章的时钟前提，而不是单独讨论 DWT。

### 4. 为什么 `DWT->CYCCNT` 要先清零？

清零不是绝对必要条件，但它让后续调试观察更直观，也避免启用时携带未知计数状态。`DWT_Delay_us()` 本身使用差值判断，因此计数器从非零开始也能工作，但前提是计数器持续增长。

换句话说，清零更多是调试便利，不是算法必须。
这类细节在教材里要说清楚，避免读者把“便于观察”误读成“功能必须”。

### 5. 第10章是否已经讲完传感器标定？

没有。本章只说明温度补偿采样和静态零偏采样中使用了 DWT 微秒等待。标定数学、偏置估计和参数意义留到后续传感器标定章节。

本章只建立时间工具，不处理标定公式本身。
如果把 DWT 的用法和标定算法混在一起，读者很难区分“等待间隔”与“数据计算”两条链路。

### 6. TIM6 已经配置了，为什么本章不把它当作微秒时间基准？

因为当前证据不足。`.ioc` 和 `tim.c` 能证明 TIM6 被配置，`main.c` 也清零了 `htim6` 计数器。
但当前源码没有看到业务代码启动 TIM6、读取 TIM6 计数器或用 TIM6 中断更新微秒变量。

因此第10章只能把 TIM6 作为“存在配置但未成为当前微秒路径”的边界证据。
如果后续版本启用了 `HAL_TIM_Base_Start(&htim6)` 并用 `__HAL_TIM_GET_COUNTER(&htim6)` 生成时间戳，
教材再把 TIM6 纳入微秒时间基准才有充分依据。

### 7. `DWT_Delay_us(1000)` 是否意味着采样频率一定是 1000Hz？

不能这样说。`DWT_Delay_us(1000)` 只保证循环中插入了目标 1000us 的等待。
一次采样循环还包括 I2C 读取、数据处理、累加和循环控制开销。

所以实际采样频率一定低于或等于由纯等待推导出的理想频率，
具体数值需要连续时间戳、DWT 记录、逻辑分析仪或其他实测证据【待验证】。

### 8. `DWT_Delay_us()` 可以替代所有延时吗？

不适合。它是忙等，会占用 CPU；目标时间越长，浪费的执行时间越多。
它适合短时间、初始化或采样间隔这类简单等待。

对于毫秒级以上任务、可并行等待、低频状态机或需要响应其他事件的场景，
应优先考虑 HAL tick、定时器中断、状态机或调度器。

### 9. `micros()` 回绕后，500Hz 时间差会不会立刻错误？

不一定。当前代码用 `uint32_t` 做 `currentTime - previous500HzTime`。
在 C 语言无符号整数规则下，这个差值按模 2^32 计算。只要两次时间戳之间的真实间隔远小于
`2^32 us`，跨过一次回绕仍能得到正确的短间隔。

风险在于把 `micros()` 当作绝对时间排序，或把两个相隔很久的时间戳直接比较大小。
对本项目 500Hz 邻近帧间隔而言，回绕不是主要风险；主循环超时、I2C 阻塞和中断延迟更需要后续实测。

### 10. 为什么 `DWT_Delay_us()` 的 1000us 不是严格采样周期？

因为它只覆盖循环中的等待段。一次采样还包括 MPU6050 I2C 读取、温度补偿、累加和循环控制。
中断也可能在忙等期间插入执行。因此 `DWT_Delay_us(1000)` 更准确的说法是
“每次循环末尾至少插入一个目标 1000us 的忙等窗口”，不是“整个采样循环严格 1000us”。

## 11. 实践任务

开始任务前，先回到本章第8节定位 DWT 启用、`micros()` 和 `DWT_Delay_us()` 证据；第9节提供微秒时间基准调试顺序。

任务一至任务二属于 DWT 基础定位；任务三至任务六属于项目调用证据；
任务七属于 TIM6 边界核查；任务八属于双路径综合整理。

任务一：确认 DWT 与 CoreDebug 类型定义。

在 `core_cm3.h` 中找到 `DWT_Type`、`CYCCNT` 和 `CoreDebug_Type`。
验收依据是定义表包含名称、类型、来源文件和用途结论。

任务二：确认 DWT 启用顺序。

在 `main.c` 中找到 DWT 启用顺序：`CoreDebug->DEMCR`、`DWT->CYCCNT`、`DWT->CTRL`。
验收依据是启用顺序表按执行顺序列出三条寄存器写入。

任务三：追踪 `micros()` 时间戳来源。

在 `main.c` 中找出 `micros()` 使用的 `HAL_GetTick()`、`SysTick->VAL` 和 `SysTick->LOAD`。
验收依据是时间戳表包含读取项、换算项和输出单位。

任务四：追踪 DWT 微秒等待实现。

在 `mpu6050Calibration.c` 中找出 `DWT_Delay_us()` 的周期差判断。
验收依据是等待表包含起始计数、当前计数、差值条件、结束条件、回绕风险和忙等影响。

任务五：确认 DWT 延时声明。

在 `mpu6050Calibration.h` 中确认 `DWT_Delay_us()` 的函数声明。
验收依据是声明表包含函数名、头文件、引用模块和调用边界。

任务六：确认 DWT 延时调用点。

在 `mpu6050Calibration.c` 和 `mpu6050.c` 中分别找到 `DWT_Delay_us()` 的调用点。
验收依据是调用表按文件分列采样循环、等待参数、循环次数、理论最小等待时间和证据边界。

任务七：核查 TIM6 是否参与当前微秒路径。

在 `.ioc`、`tim.c` 和 `main.c` 中记录 TIM6 的配置、清零、启动和读取证据。
验收依据是边界表明确区分“TIM6 已配置”和“TIM6 当前未作为 `micros()` 或 `DWT_Delay_us()` 来源”。

任务八：画出两条微秒证据链。

画出“微秒时间戳路径”和“DWT 微秒等待路径”两条证据链。
验收依据是双链图分列时间戳路径、等待路径和各自证据位置。

任务九：计算回绕和溢出边界。

计算 `micros()` 的 32 位微秒回绕时间、`DWT->CYCCNT` 在 72MHz 下的回绕时间，
以及 `DWT_Delay_us()` 在 `ticks = us * 72` 下的最大不溢出 `us`。
验收依据是计算表包含公式、数值、单位和工程结论。

任务十：核查整数除法误差。

说明为什么 72MHz 下 `SystemCoreClock / 1000000 = 72` 没有截断误差，
并讨论如果核心时钟改成不能被 1MHz 整除的频率，`DWT_Delay_us()` 会怎样偏差。
验收依据是能区分“当前项目成立”和“移植后需要重新审查”。

实践边界：

当前任务优先形成表格、链路图、搜索记录和计算过程。涉及 IDE 现场、构建日志、断点数值、外部波形、主机侧结果或硬件响应时，若没有截图、日志或仓库外实测证据，结论保持【待验证】。

## 12. 思考题

1. 为什么 `micros()` 和 `DWT_Delay_us()` 都和微秒有关，却不能混为一谈？
2. 如果 `DWT->CYCCNT` 没有增长，`DWT_Delay_us()` 会出现什么现象？
3. `SystemCoreClock` 变化后，为什么会影响 `DWT_Delay_us()` 的微秒换算？
4. 为什么 `micros()` 需要连续读取两次 `HAL_GetTick()` 和 `SysTick->VAL`？
5. 为什么第10章只讲采样等待，不展开 MPU6050 标定计算？
6. `micros()` 返回值看起来连续时，还需要哪些证据才能判断它足够支撑 500Hz 控制周期分析？
7. 为什么 `micros()` 的返回值可以回绕，但相邻帧差值仍可能是正确的？
8. 为什么 `SystemCoreClock / 1000000` 是一个移植风险点？
9. 如果 `DWT_CTRL_NOCYCCNT_Msk` 为 1，`DWT_Delay_us()` 的当前实现会有什么风险？

## 13. 本章总结

本章建立了三轴云台项目中 DWT 周期计数器、微秒时间戳和微秒等待之间的证据链。

已经确认的结论是：

- `core_cm3.h` 提供 `DWT`、`CoreDebug` 和相关位掩码定义。
- `main.c` 启用 DWT 周期计数器。
- `micros()` 使用 HAL tick 和 SysTick 当前计数位置生成微秒时间戳。
- `DWT_Delay_us()` 使用 `DWT->CYCCNT` 和 `SystemCoreClock` 实现微秒级等待。
- `SystemCoreClock` 初始定义为 8MHz，当前 72MHz 换算依赖 `SystemClock_Config()` 中 `HAL_RCC_ClockConfig()` 对它的运行时更新。
- 72MHz 下 `DWT_Delay_us(1000)` 的目标等待量为 72000 个 CPU 周期。
- 32 位 `DWT->CYCCNT` 在 72MHz 下理论回绕时间约为 59.65s。
- `micros()` 的 32 位微秒时间戳约每 71.58min 回绕一次，短间隔应使用无符号差值理解。
- `micros()` 通过两次 `HAL_GetTick()` 与两次 `SysTick->VAL` 降低跨 tick 读数风险，但它不是关中断后的原子快照。
- `DWT_Delay_us()` 当前使用整数除法和 32 位乘法，1000us 调用安全，但长等待和移植时钟需要重新审查。
- `.su/.cyclo` 的静态资源结论统一回到第8.7节判断：当前 Debug 构建中 `micros`、`DWT_Delay_us` 与 `HAL_GetTick` 都生成了静态栈和圈复杂度条目，但这些条目只能说明本次编译的静态估算，不能替代运行时最坏栈深度、500Hz 抖动或采样周期实测证据。
- `mpu6050Calibration.c` 在温度补偿采样中使用 `DWT_Delay_us(sampleRate)`。
- 同一标定函数中的中间升温等待使用 `HAL_Delay(10000)`，实际时间基准是 HAL 毫秒 tick，而不是 DWT 微秒忙等。
- `mpu6050.c` 在静态零偏采样中使用 `DWT_Delay_us(1000)`。
- TIM6 在当前仓库中有初始化和清零证据，但没有成为本章两条微秒路径的有效来源。
- 第10章只建立微秒时间基础，传感器标定、500Hz 实时控制循环和控制算法留到后续章节。

本章边界：

- `micros()` 与 `DWT_Delay_us()` 都属于时间工具，但分别服务时间戳和等待。
- 本章不证明 500Hz 周期没有抖动或丢帧，只提供后续观测 `dt500Hz` 和执行时间的基础。
- 本章不证明 1000us 等待对应精确 1000Hz 采样频率；采样总周期仍需实测【待验证】。
- 本章不把 TIM6 配置意图等同于当前微秒时基使用证据。

下一章可以进入 Newlib 适配与 UART 调试输出，因为当前已经具备毫秒节拍、微秒时间戳和基本调试时间观察能力，后续需要解释项目如何把 `printf` 调试信息输出到串口。

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
- `Drivers/CustomDrivers/Src/mpu6050Calibration.c`
- `Drivers/CustomDrivers/Inc/mpu6050Calibration.h`
- `Drivers/CustomDrivers/Src/mpu6050.c`
- `Core/Src/tim.c`
- `Three-axis_cloud_platformV2.ioc`
- `Drivers/CMSIS/Include/core_cm3.h`
- `Debug/Three-axis_cloud_platformV2.map`
- `Debug/Three-axis_cloud_platformV2.list`
- `Debug/Core/Src/main.su`
- `Debug/Core/Src/main.cyclo`
- `Debug/Drivers/CustomDrivers/Src/mpu6050Calibration.su`
- `Debug/Drivers/CustomDrivers/Src/mpu6050Calibration.cyclo`
- `Debug/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.su`
- `Debug/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.cyclo`

外部权威资料：

- ST RM0008 Reference manual: `https://www.st.com/resource/en/reference_manual/rm0008-stm32f101xx-stm32f102xx-stm32f103xx-stm32f105xx-and-stm32f107xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf`
- ST DS5792 Datasheet for STM32F103xC/xD/xE: `https://www.st.com/resource/en/datasheet/stm32f103rc.pdf`
- ST STM32F103 documentation page: `https://www.st.com/en/microcontrollers-microprocessors/stm32f103/documentation.html`
- Arm CMSIS-Core `DWT_Type`: `https://arm-software.github.io/CMSIS_5/Core/html/structDWT__Type.html`
- Arm CMSIS-Core `SysTick_Type`: `https://arm-software.github.io/CMSIS_5/Core/html/structSysTick__Type.html`
- Arm CMSIS-Core `CoreDebug_Type`: `https://arm-software.github.io/CMSIS_5/Core/html/structCoreDebug__Type.html`

符号、函数与配置项证据：

- `CoreDebug->DEMCR`
- `CoreDebug_DEMCR_TRCENA_Pos`
- `CoreDebug_DEMCR_TRCENA_Msk`
- `DWT->CYCCNT`
- `DWT->CTRL`
- `DWT_CTRL_CYCCNTENA_Pos`
- `DWT_CTRL_CYCCNTENA_Msk`
- `DWT_CTRL_NOCYCCNT_Pos`
- `DWT_CTRL_NOCYCCNT_Msk`
- `DWT_BASE`
- `CoreDebug_BASE`
- `SystemCoreClock`
- `micros()`
- `HAL_GetTick()`
- `SysTick->VAL`
- `SysTick->LOAD`
- `SysTick_CTRL_COUNTFLAG_Msk`
- `DWT_Delay_us()`
- `sampleRate`
- `MX_TIM6_Init()`
- `htim6`
- `__HAL_TIM_SET_COUNTER()`
- `deltaTime500Hz`
- `executionTime500Hz`

质量自检：

- P0 事实错误：通过
- P1 依赖断层：通过
- P2 逻辑连贯：通过
- P3 项目证据：通过
- P4 原理展开：通过
- P5 调试实践：通过
- P6 表达统一：通过

---
> 导航：上一章：[第09章_中断系统与SysTick节拍](第09章_中断系统与SysTick节拍.md) ｜ 下一章：[第11章_Newlib适配与UART调试输出](第11章_Newlib适配与UART调试输出.md)
