# 第10章 DWT计时与微秒时间基准

## 1. 本章目标

- 理解项目为什么同时需要毫秒节拍、微秒时间戳和微秒级等待。
- 区分 `micros()` 的 SysTick/HAL tick 时间戳路径与 `DWT_Delay_us()` 的 DWT 周期计数路径。
- 看懂项目中 `CoreDebug->DEMCR`、`DWT->CYCCNT` 和 `DWT->CTRL` 的启用关系。
- 能从 `main.c`、`mpu6050Calibration.c`、`mpu6050.c` 和 CMSIS 头文件追踪微秒计时证据。

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
- 微秒时间戳：用于记录某个时刻的微秒级数值，本项目由 `micros()` 返回。
- 微秒级等待：让代码等待指定微秒数，本项目由 `DWT_Delay_us()` 实现。
- `SystemCoreClock`：核心时钟频率，`DWT_Delay_us()` 用它把微秒换算成周期数。

这些概念服务于正式知识点 `DWT周期计数器`，不新增结构外知识点。

## 5. 工作原理

本项目中的微秒时间可以按两条路径理解。

第一条路径是 SysTick 细分时间戳。

`HAL_GetTick()` 提供当前毫秒数，`SysTick->VAL` 提供当前 1ms 周期内已经倒计到哪里。

`SysTick->LOAD + 1` 表示 1ms 内总计数长度。

`micros()` 先读取一次毫秒数和 SysTick 倒计数，再读取第二次，用两次毫秒数判断是否刚好跨过 tick 边界。

如果两次毫秒数不同，说明读取过程中发生了 SysTick 更新，函数使用新的毫秒数和新的倒计数值。如果两次毫秒数相同，说明仍在同一个毫秒周期内，函数使用第一次读到的倒计数值。这样可以降低跨边界读取造成的时间戳跳变风险。

第二条路径是 DWT 周期等待。

`DWT_Delay_us()` 先记录当前 `DWT->CYCCNT`，再根据 `SystemCoreClock / 1000000` 把目标微秒数换算成 CPU 周期数。随后它循环读取当前 `DWT->CYCCNT`，直到当前计数与起始计数之差达到目标周期数。

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

`Drivers/CMSIS/Include/core_cm3.h` 提供了这些符号的定义：

- `DWT_Type` 中包含 `CTRL` 和 `CYCCNT`。
- `CoreDebug_Type` 中包含 `DEMCR`。
- `DWT_CTRL_CYCCNTENA_Msk` 表示周期计数使能位。
- `CoreDebug_DEMCR_TRCENA_Msk` 表示跟踪资源使能位。

`micros()` 虽然也返回微秒级时间戳，但它的直接寄存器来源是 `SysTick->VAL` 和 `SysTick->LOAD`，不是 `DWT->CYCCNT`。这一点必须和 `DWT_Delay_us()` 区分。

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
- `main.c` 启用 DWT 周期计数器，并实现 `micros()`。
- `mpu6050Calibration.c` 实现 `DWT_Delay_us()`，并在温度补偿采样中调用它。
- `mpu6050Calibration.h` 声明 `DWT_Delay_us()`，使其他文件可以调用。
- `mpu6050.c` 在静态零偏采样循环中调用 `DWT_Delay_us(1000)`。

运行流程上，项目先完成 HAL、系统时钟、SysTick 和外设初始化，再启用 DWT 周期计数器。随后，`micros()` 可以用于计算 500Hz 周期时间差和执行耗时，`DWT_Delay_us()` 可以用于采样间隔等待。

## 8. 代码分析

### 1. `main.c` 中的 DWT 启用

`main.c` 在用户初始化段中先配置 SysTick，再启用 DWT 周期计数器。它写入 `CoreDebug->DEMCR`，清零 `DWT->CYCCNT`，最后设置 `DWT->CTRL`。

这段代码的项目含义是：后续任何基于 `DWT->CYCCNT` 的等待或测量，都依赖这里已经执行。若这段初始化未执行，`DWT_Delay_us()` 可能无法按预期结束等待。

### 2. `micros()` 的时间戳路径

`micros()` 定义在 `main.c` 中。它读取：

- `HAL_GetTick()`
- `SysTick->VAL`
- `SysTick->LOAD`

函数的核心目标是把毫秒 tick 与当前毫秒内的 SysTick 倒计数位置合成微秒时间戳。它不是 DWT 周期计数函数。

`stm32f1xx_it.c` 中还能看到一段已经被块注释包住的旧逻辑，其中提到读取 `DWT->CYCCNT` 给 `micros()` 使用。该段当前不参与编译，不能作为当前 `micros()` 实现的有效证据；本章以 `main.c` 中实际编译的 `micros()` 函数为准。

项目在两个位置使用这类时间戳：

- `SysTick_Handler()` 中计算 `deltaTime1000Hz` 和 `executionTime1000Hz`。
- 主循环消费 `frame_500Hz` 时计算 `deltaTime500Hz`、`dt500Hz` 和 `executionTime500Hz`。

这些使用位置说明 `micros()` 服务于时间差测量和运行耗时观察。

### 3. `DWT_Delay_us()` 的等待路径

`DWT_Delay_us()` 定义在 `mpu6050Calibration.c` 中。它读取起始 `DWT->CYCCNT`，计算目标周期数 `us * (SystemCoreClock / 1000000)`，然后等待计数差达到目标值。

这段逻辑的输入是目标微秒数，输出不是一个返回值，而是“经过约定的等待时间后继续执行”。它的风险点是：如果 `DWT->CYCCNT` 没有运行，循环条件就可能一直不满足。

### 4. 温度补偿采样中的微秒等待

`mpu6050Calibration.c` 中设置 `sampleRate = 1000`，并在冷机采样和热机采样循环中调用 `DWT_Delay_us(sampleRate)`。

这里的项目含义是：温度补偿采样希望在连续读取之间插入约 1000us 的间隔。温度补偿算法本身属于后续传感器标定章节，本章只说明它使用 DWT 微秒等待。

### 5. 静态零偏采样中的微秒等待

`mpu6050.c` 的静态零偏采样循环中调用 `DWT_Delay_us(1000)`。该循环进行 5000 次采样，并在每次采样后等待约 1000us。

本章不分析静态零偏如何计算，只确认：`DWT_Delay_us()` 已经从校准文件扩展为 MPU6050 采样流程中的公共微秒等待工具。

### 6. CMSIS 头文件证据

`core_cm3.h` 中可以追踪到 `DWT_Type`、`CoreDebug_Type` 和相关位掩码。第05章已经说明 CMSIS 访问边界，第10章进一步说明这些定义如何被本项目用于 DWT 周期计数。

如果只看 `main.c` 中的 `DWT->CYCCNT`，读者会知道项目读写了某个寄存器；如果再追踪到 `core_cm3.h`，读者才能确认它属于 Cortex-M3 内核 DWT 资源。

### 本节证据边界

本节只根据当前仓库说明文件、函数、宏、变量和调用关系。运行时频率、外部硬件表现、主机侧现象、传感器方向、电机响应或真实控制效果仍需调试记录、日志或仓库外实测证据；缺少证据时保持【待验证】。

## 9. 调试方法

本章阶段的调试目标是确认微秒时间戳和微秒等待的来源没有混淆。

可观察对象：

- `main.c` 中 DWT 启用代码是否发生在调用 `DWT_Delay_us()` 之前。
- `CoreDebug->DEMCR` 是否设置跟踪使能位。
- `DWT->CYCCNT` 是否在启用前被清零。
- `DWT->CTRL` 是否设置周期计数使能位。
- `DWT_Delay_us()` 中 `ticks` 是否由 `SystemCoreClock / 1000000` 换算得到。
- `micros()` 是否读取 `HAL_GetTick()`、`SysTick->VAL` 和 `SysTick->LOAD`。
- `mpu6050Calibration.c` 中两段采样循环是否调用 `DWT_Delay_us(sampleRate)`。
- `mpu6050.c` 中静态零偏采样是否调用 `DWT_Delay_us(1000)`。

常见异常定位：

- `DWT_Delay_us()` 卡住：先检查 DWT 启用代码是否执行，再检查 `DWT->CYCCNT` 是否在增长。
- 微秒延时明显不准：先检查 `SystemCoreClock` 是否已经由系统时钟配置更新，再检查 `us * (SystemCoreClock / 1000000)` 的换算。
- `micros()` 时间戳异常：先检查 `HAL_GetTick()` 是否推进，再检查 SysTick 1ms 配置和 `SysTick->VAL/LOAD` 读取。
- 500Hz `dt500Hz` 抖动：先确认 `frame_500Hz` 由第09章的 SysTick 标志触发，再检查 `micros()` 时间差计算；控制算法本身留到后续章节。
- 标定采样间隔异常：先确认 `DWT_Delay_us()` 的调用位置，再进入后续标定章节分析采样逻辑。

调试记录建议：

- 记录 DWT 启用语句、`DWT->CYCCNT` 清零与递增现象、`SystemCoreClock` 数值和微秒换算结果。
- 分开记录 `micros()` 的时间戳路径和 `DWT_Delay_us()` 的忙等路径，避免把两类时间基准混为一个结论。
- 对 500Hz 采样间隔，只记录本章能证明的时间基准证据；控制算法影响留到后续控制循环章节。
- 若没有示波器、逻辑分析仪或连续日志，微秒延时精度和周期抖动只能标记为【待验证】。

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

## 11. 实践任务

开始任务前，先回到本章第8节定位 DWT 启用、`micros()` 和 `DWT_Delay_us()` 证据；第9节提供微秒时间基准调试顺序。

任务一至任务二属于 DWT 基础定位；任务三至任务六属于项目调用证据；任务七属于双路径综合整理。

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
验收依据是等待表包含起始计数、当前计数、差值条件和结束条件。

任务五：确认 DWT 延时声明。

在 `mpu6050Calibration.h` 中确认 `DWT_Delay_us()` 的函数声明。
验收依据是声明表包含函数名、头文件、引用模块和调用边界。

任务六：确认 DWT 延时调用点。

在 `mpu6050Calibration.c` 和 `mpu6050.c` 中分别找到 `DWT_Delay_us()` 的调用点。
验收依据是调用表按文件分列标定延时、初始化延时和操作延时位置。

任务七：画出两条微秒证据链。

画出“微秒时间戳路径”和“DWT 微秒等待路径”两条证据链。
验收依据是双链图分列时间戳路径、等待路径和各自证据位置。

实践边界：

当前任务优先形成表格、链路图、搜索记录和计算过程。涉及 IDE 现场、构建日志、断点数值、外部波形、主机侧结果或硬件响应时，若没有截图、日志或仓库外实测证据，结论保持【待验证】。

## 12. 思考题

1. 为什么 `micros()` 和 `DWT_Delay_us()` 都和微秒有关，却不能混为一谈？
2. 如果 `DWT->CYCCNT` 没有增长，`DWT_Delay_us()` 会出现什么现象？
3. `SystemCoreClock` 变化后，为什么会影响 `DWT_Delay_us()` 的微秒换算？
4. 为什么 `micros()` 需要连续读取两次 `HAL_GetTick()` 和 `SysTick->VAL`？
5. 为什么第10章只讲采样等待，不展开 MPU6050 标定计算？
6. `micros()` 返回值看起来连续时，还需要哪些证据才能判断它足够支撑 500Hz 控制周期分析？

## 13. 本章总结

本章建立了三轴云台项目中 DWT 周期计数器、微秒时间戳和微秒等待之间的证据链。

已经确认的结论是：

- `core_cm3.h` 提供 `DWT`、`CoreDebug` 和相关位掩码定义。
- `main.c` 启用 DWT 周期计数器。
- `micros()` 使用 HAL tick 和 SysTick 当前计数位置生成微秒时间戳。
- `DWT_Delay_us()` 使用 `DWT->CYCCNT` 和 `SystemCoreClock` 实现微秒级等待。
- `mpu6050Calibration.c` 在温度补偿采样中使用 `DWT_Delay_us(sampleRate)`。
- `mpu6050.c` 在静态零偏采样中使用 `DWT_Delay_us(1000)`。
- 第10章只建立微秒时间基础，传感器标定、500Hz 实时控制循环和控制算法留到后续章节。

本章边界：

- `micros()` 与 `DWT_Delay_us()` 都属于时间工具，但分别服务时间戳和等待。
- 本章不证明 500Hz 周期没有抖动或丢帧，只提供后续观测 `dt500Hz` 和执行时间的基础。

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
- `Drivers/CMSIS/Include/core_cm3.h`

符号、函数与配置项证据：

- `CoreDebug->DEMCR`
- `CoreDebug_DEMCR_TRCENA_Msk`
- `DWT->CYCCNT`
- `DWT->CTRL`
- `DWT_CTRL_CYCCNTENA_Msk`
- `SystemCoreClock`
- `micros()`
- `HAL_GetTick()`
- `SysTick->VAL`
- `SysTick->LOAD`
- `DWT_Delay_us()`
- `sampleRate`
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
