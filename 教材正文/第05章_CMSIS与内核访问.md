# 第05章 CMSIS与内核访问

## 1. 本章目标

- 建立项目中直接访问内核资源和寄存器资源的位置索引。
- 理解 CMSIS 如何把 Cortex-M3 内核寄存器和 STM32 外设寄存器暴露给 C 代码。
- 区分 HAL API 调用和直接寄存器访问的工程边界。
- 为后续 DWT、SysTick、定时器寄存器访问和中断配置章节建立前置基础。

## 2. 前置知识

- STM32F103RCTx芯片平台

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
- 内核寄存器：属于 Cortex-M3 内核或系统控制空间的寄存器，例如 `SysTick`、`DWT`、`CoreDebug`、`SCB`。
- 外设寄存器：属于 STM32 片上外设的寄存器，例如 `TIM2->CNT`、`TIM3->CCR1`。
- 直接寄存器访问：不通过 HAL API 封装，直接读写寄存器结构体成员。
- 原子性保护：在短时间关键更新前后关闭和恢复中断，减少被中断打断的风险。

这些概念都服务于 `CMSIS寄存器与内核访问`，不新增正式知识点。

## 5. 工作原理

CMSIS 让 C 代码可以用结构体成员的形式访问固定地址上的硬件寄存器。

它的基本机制是：

1. CMSIS 头文件定义寄存器结构体类型，例如 `SysTick_Type`、`DWT_Type`、`SCB_Type`。
2. CMSIS 头文件定义外设或内核资源的基地址。
3. CMSIS 用宏把基地址转换成结构体指针，例如 `SysTick`、`DWT`、`CoreDebug`、`SCB`。
4. 项目代码通过 `SysTick->VAL`、`DWT->CYCCNT`、`TIM3->CCR1` 这样的形式读写寄存器。

这类访问绕过了 HAL 函数调用层，优点是直接、明确、开销低；风险是必须清楚寄存器含义、时序要求和并发影响。

本项目使用直接访问主要集中在三类场景：

- 时间基准：读 `SysTick` 和 `DWT`。
- 电机输出：写定时器计数器和比较寄存器。
- 系统控制：在特定回调中写系统控制寄存器。

## 6. STM32实现机制

本项目使用的是 Cortex-M3 内核。CMSIS 证据需要分成两层看：`Drivers/CMSIS/Include/core_cm3.h` 定义 Cortex-M3 内核资源，`Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xe.h` 定义 STM32F103xE 设备外设资源。

`core_cm3.h` 中可以确认的核心资源包括：

- `SysTick` 映射到 SysTick 配置结构。
- `DWT` 映射到 Data Watchpoint and Trace 结构。
- `CoreDebug` 映射到核心调试寄存器结构。
- `SCB` 映射到系统控制块结构。
- `NVIC` 相关内联函数用于中断控制。

`stm32f103xe.h` 中可以确认的设备资源包括：

- `TIM_TypeDef` 定义定时器寄存器结构。
- `TIM2`、`TIM3` 和 `TIM4` 映射到对应定时器基地址。
- `TIMx->CNT` 和 `TIMx->CCR` 的访问依赖这些设备头文件定义。

项目源码中的访问证据包括：

- `Core/Src/main.c` 的 `micros()` 读取 `SysTick->VAL` 和 `SysTick->LOAD`。
- `Core/Src/main.c` 中启用 DWT 周期计数器，写入 `CoreDebug->DEMCR`、`DWT->CYCCNT` 和 `DWT->CTRL`。
- `Drivers/CustomDrivers/Src/mpu6050Calibration.c` 的 `DWT_Delay_us()` 读取 `DWT->CYCCNT` 实现微秒延时。
- `Drivers/CustomDrivers/Src/drv_pwmMotors.c` 写 `TIM2->CNT`、`TIM3->CNT`、`TIM4->CNT`，用于对齐多个定时器计数器。
- `Drivers/CustomDrivers/Src/drv_pwmMotors.c` 写 `TIMx->CCR`，用于更新 PWM 比较值。
- `USB_DEVICE/Target/usbd_conf.c` 在挂起回调中写 `SCB->SCR` 的系统控制位。

这些访问都能追溯到项目源码和 CMSIS 头文件。

## 7. 项目中的应用

本章对应项目里的底层访问索引层。

直接相关文件：

- `Core/Src/main.c`
- `Drivers/CustomDrivers/Src/mpu6050Calibration.c`
- `Drivers/CustomDrivers/Src/drv_pwmMotors.c`
- `USB_DEVICE/Target/usbd_conf.c`
- `Drivers/CMSIS/Include/core_cm3.h`
- `Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xe.h`

它们之间的关系是：

- `core_cm3.h` 提供内核资源名称和结构体定义。
- `stm32f103xe.h` 提供 STM32F103xE 设备外设名称、基地址和寄存器结构定义。
- `main.c` 使用 `SysTick` 与 `DWT` 形成时间测量入口。
- `mpu6050Calibration.c` 使用 `DWT` 做微秒级等待。
- `drv_pwmMotors.c` 使用直接定时器寄存器访问更新三相 PWM 输出。
- `usbd_conf.c` 在设备支持文件中访问系统控制块寄存器。

从项目主线看，本章是后续多个章节的公共前置：它不完整讲 DWT、SysTick、定时器和中断，而是让读者先知道项目确实存在 CMSIS 级访问，并知道这些访问分布在哪里。

## 8. 代码分析

### 1. `core_cm3.h` 的角色

`Drivers/CMSIS/Include/core_cm3.h` 定义了 Cortex-M3 内核资源。项目代码中出现的 `SysTick`、`DWT`、`CoreDebug` 和 `SCB` 都不是普通变量，而是 CMSIS 根据固定地址映射出的寄存器访问入口。

这意味着源码中的 `DWT->CYCCNT` 不是访问内存数组，而是在读写内核调试与计数资源。

`Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xe.h` 定义了 STM32F103xE 设备外设资源。项目代码中出现的 `TIM2`、`TIM3`、`TIM4` 和 `TIM_TypeDef` 由设备头文件提供，因此定时器直接寄存器访问不能只追溯到 `core_cm3.h`。

### 2. `main.c` 中的 SysTick 访问

`micros()` 函数读取 `HAL_GetTick()` 和 `SysTick->VAL`，并使用 `SysTick->LOAD + 1` 作为每毫秒的计数周期。它试图在毫秒 tick 之外补充一个更细的计数位置。

这里的关键点是：`HAL_GetTick()` 是 HAL 层的毫秒计数入口，`SysTick->VAL` 和 `SysTick->LOAD` 是 CMSIS 级寄存器访问。二者组合后，项目得到微秒时间戳的计算基础。

### 3. `main.c` 中的 DWT 启用

`main.c` 在初始化阶段写入 `CoreDebug->DEMCR`，然后清零 `DWT->CYCCNT`，再通过 `DWT->CTRL` 启用周期计数。

这说明 DWT 周期计数器不是默认直接可用，项目需要先打开 trace 相关能力，再启用 cycle counter。详细计时原理留到第10章。

### 4. `mpu6050Calibration.c` 中的 DWT 延时

`DWT_Delay_us()` 读取起始 `DWT->CYCCNT`，把目标微秒数换算成周期数，然后循环等待计数差达到目标。

本章只确认它是 CMSIS 级计时访问。它为什么用于标定采样、采样间隔是否合理，留到传感器标定章节分析。

### 5. `drv_pwmMotors.c` 中的定时器寄存器访问

`PWM_Motor_Init()` 在启动多个 PWM 通道后，用 `__disable_irq()` 和 `__enable_irq()` 包住 `TIM2->CNT`、`TIM3->CNT`、`TIM4->CNT` 清零操作，用于减少更新过程中被中断打断的风险。

`PWM_Motor_SetAngle()` 和硬件诊断函数直接写多个 `TIMx->CCR` 寄存器，用于更新 PWM 比较值。

这些访问是项目执行器链路的重要证据，但本章只建立“直接寄存器访问”这个入口。多定时器同步和三相 PWM 输出留到后续章节。

### 6. `usbd_conf.c` 中的系统控制访问

`USB_DEVICE/Target/usbd_conf.c` 的挂起回调中，在特定条件下写 `SCB->SCR`。这属于系统控制块寄存器访问。

本章只把它作为 CMSIS 访问索引记录；设备中间件与通信支线的细节留到后续章节。

## 9. 调试方法

本章阶段的调试目标是确认直接寄存器访问点是否存在、是否处于正确上下文、是否有保护边界。

可观察对象：

- `core_cm3.h` 中是否定义 `SysTick`、`DWT`、`CoreDebug`、`SCB`。
- `stm32f103xe.h` 中是否定义 `TIM_TypeDef`、`TIM2`、`TIM3` 和 `TIM4`。
- `main.c` 中 `micros()` 是否读取 `SysTick->VAL` 和 `SysTick->LOAD`。
- `main.c` 中是否在使用 DWT 前设置 `CoreDebug->DEMCR` 和 `DWT->CTRL`。
- `mpu6050Calibration.c` 中 `DWT_Delay_us()` 是否使用 `DWT->CYCCNT`。
- `drv_pwmMotors.c` 中定时器 `CNT` 和 `CCR` 写入是否位于可解释的更新区域。
- `drv_pwmMotors.c` 中关键写入前后是否使用 `__disable_irq()` 和 `__enable_irq()`。
- `usbd_conf.c` 中 `SCB->SCR` 写入是否位于挂起回调边界内。

常见异常定位：

- `micros()` 时间戳异常：先检查 `HAL_GetTick()` 是否推进，再检查 `SysTick->VAL` 和 `SysTick->LOAD` 的读取。
- `DWT_Delay_us()` 微秒延时卡住：检查 DWT 是否已经启用，以及 `DWT->CYCCNT` 是否在运行。
- PWM 相位或输出异常：先确认定时器 `CNT` 和 `CCR` 写入是否执行，再到后续定时器章节分析配置。
- 系统进入异常或停在错误处理：检查直接寄存器写入是否发生在预期上下文。
- 关键寄存器更新被打断：检查更新前后的中断保护范围。

## 10. 常见问题

### 1. CMSIS 和 HAL 是什么关系？

HAL 是更高层的外设抽象接口；CMSIS 提供更底层的内核与寄存器访问名称。项目可以同时使用两者。

### 2. 直接写寄存器是不是一定比 HAL 好？

不是。直接访问更明确、开销更低，但也更依赖对寄存器和时序的理解。项目只在需要精细控制的位置使用这种方式。

### 3. 为什么第05章不展开 DWT 计时原理？

因为 DWT 已经安排在第10章。本章只建立 CMSIS 访问前置，避免跳过教学顺序。

### 4. 为什么第05章不展开定时器 PWM 细节？

因为通用定时器 PWM 和多定时器三相输出在后续章节。本章只说明项目中存在 `TIMx->CNT` 和 `TIMx->CCR` 直接访问。

### 5. 为什么有些 CMSIS 访问出现在中间件支持文件里？

因为中间件底层也可能需要访问系统控制资源。本章只记录访问位置，不把对应中间件功能提前写成主线。

## 11. 实践任务

- 在 `core_cm3.h` 中找到 `SysTick`、`DWT`、`CoreDebug` 和 `SCB` 的定义位置。
- 在 `stm32f103xe.h` 中找到 `TIM_TypeDef`、`TIM2`、`TIM3` 和 `TIM4` 的定义位置。
- 在 `main.c` 中找出 `micros()` 对 SysTick 寄存器的读取。
- 在 `main.c` 中找出启用 DWT 周期计数器的三条语句。
- 在 `mpu6050Calibration.c` 中找出 `DWT_Delay_us()` 的计数差判断。
- 在 `drv_pwmMotors.c` 中找出 `CNT` 清零和 `CCR` 写入位置。
- 在 `usbd_conf.c` 中找出 `SCB->SCR` 写入位置，并说明本章为什么不展开设备中间件细节。

## 12. 思考题

1. 为什么 `SysTick->VAL` 和 `HAL_GetTick()` 可以在同一个时间函数中同时出现？
2. 如果没有先启用 DWT 周期计数器，`DWT_Delay_us()` 可能出现什么问题？
3. 为什么多个 `TIMx->CCR` 写入前后要考虑中断打断风险？
4. 直接寄存器访问和 HAL API 的边界应该如何判断？
5. 为什么本章只建立访问索引，而不展开每个寄存器的完整功能？

## 13. 本章总结

本章建立了三轴云台项目中的 CMSIS 与直接寄存器访问索引。

已经确认的结论是：

- `core_cm3.h` 提供 Cortex-M3 内核资源访问入口。
- `stm32f103xe.h` 提供 STM32F103xE 设备外设寄存器访问入口。
- `main.c` 直接访问 `SysTick`、`CoreDebug` 和 `DWT`。
- `mpu6050Calibration.c` 使用 `DWT->CYCCNT` 实现微秒级等待。
- `drv_pwmMotors.c` 直接写定时器 `CNT` 和 `CCR`。
- `usbd_conf.c` 在特定回调中写 `SCB->SCR`。
- 本章只建立访问边界，DWT、SysTick、定时器和中间件细节分别留到后续章节。

下一章可以进入系统时钟树，因为内核访问和时间基准入口已经建立，后续才能解释这些计数值和时钟配置之间的关系。

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
- `Drivers/CustomDrivers/Src/drv_pwmMotors.c`
- `USB_DEVICE/Target/usbd_conf.c`
- `Drivers/CMSIS/Include/core_cm3.h`
- `Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xe.h`

符号与函数证据：

- `micros()`
- `DWT_Delay_us()`
- `PWM_Motor_Init()`
- `PWM_Motor_SetAngle()`
- `HAL_PCD_SuspendCallback()`
- `SysTick->VAL`
- `SysTick->LOAD`
- `CoreDebug->DEMCR`
- `DWT->CYCCNT`
- `DWT->CTRL`
- `TIMx->CNT`
- `TIMx->CCR`
- `SCB->SCR`
- `TIM_TypeDef`
- `TIM2`
- `TIM3`
- `TIM4`
- `__disable_irq()`
- `__enable_irq()`

质量自检：

- P0 事实错误：通过
- P1 依赖断层：通过
- P2 逻辑连贯：通过
- P3 项目证据：通过
- P4 原理展开：通过
- P5 调试实践：通过
- P6 表达统一：通过
