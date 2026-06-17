# 第08章 AFIO重映射与SWD调试

## 1. 本章目标

- 理解 AFIO 重映射在 STM32F1 引脚复用中的作用。
- 看懂本项目为什么同时保留串行调试接口并配置 USART3 部分重映射。
- 能从 `HAL_MspInit()`、`HAL_UART_MspInit()` 和 `.ioc` 中追踪调试接口与串口引脚的配置证据。
- 为后续 UART 串口调试输出和 ST-LINK 调试配置建立前置基础。

## 2. 前置知识

- HAL MSP初始化机制
- GPIO输出与复用

第07章已经说明 MSP 函数和 GPIO 复用配置分布在哪里。本章在此基础上只解释 AFIO 重映射与串行调试接口保留，不展开 UART 数据收发、printf 重定向或 ST-LINK 下载流程。

## 3. 问题背景

STM32F1 的一些外设功能并不是永远固定在唯一一组引脚上。为了让同一个芯片封装适配不同硬件连接，部分外设可以通过 AFIO 进行重映射。

三轴云台项目中，第07章已经确认 USART3 使用 PC10/PC11，`.ioc` 也把 PC10/PC11 标为 USART3_TX 和 USART3_RX。与此同时，项目还要保留 PA13/PA14 作为串行调试接口，方便后续下载、调试和在线查看程序状态。

如果不理解这一层，读者容易混淆三件事：

- PC10/PC11 为什么能作为 USART3 引脚。
- PA13/PA14 为什么不能随意当普通 GPIO 使用。
- `HAL_MspInit()` 中的调试接口配置和 `usart.c` 中的 USART3 重映射为什么分布在两个位置。

本章要解决的就是这条引脚复用与调试保留的关系。

## 4. 核心概念

- AFIO重映射与SWD调试：通过 AFIO 配置改变部分外设引脚映射，同时保留串行调试接口的项目配置关系。
- AFIO：STM32F1 中负责部分引脚复用、重映射和调试接口配置的功能模块。
- 重映射：把某个外设功能从默认引脚组切换到另一组允许的引脚。
- 串行调试接口：本项目 `.ioc` 中 PA13/PA14 对应的调试接口信号。
- JTAG关闭保留SWD：项目中通过 `__HAL_AFIO_REMAP_SWJ_NOJTAG()` 释放部分调试相关资源，同时保留串行调试能力。
- USART3部分重映射：项目中通过 `__HAL_AFIO_REMAP_USART3_PARTIAL()` 配合 PC10/PC11 的 USART3 配置。

这些概念都服务于正式知识点 `AFIO重映射与SWD调试`，不新增结构外知识点。

## 5. 工作原理

AFIO 重映射的核心作用是处理“同一外设功能如何落到具体引脚”的问题。

本项目中存在两条相关链路。

第一条是调试接口链路：

1. `.ioc` 将 PA13 配置为串行调试数据相关信号。
2. `.ioc` 将 PA14 配置为串行调试时钟相关信号。
3. `HAL_MspInit()` 启用 AFIO 时钟。
4. `HAL_MspInit()` 在同一全局初始化段中也启用 PWR 时钟。
5. `HAL_MspInit()` 调用 `__HAL_AFIO_REMAP_SWJ_NOJTAG()`。

这条链路说明项目没有把 PA13/PA14 当成普通业务引脚，而是保留给串行调试接口。

第二条是 USART3 引脚链路：

1. `.ioc` 将 PC10 配置为 USART3_TX。
2. `.ioc` 将 PC11 配置为 USART3_RX。
3. `HAL_UART_MspInit()` 在 USART3 分支中配置 PC10/PC11 的 GPIO 模式。
4. `HAL_UART_MspInit()` 调用 `__HAL_AFIO_REMAP_USART3_PARTIAL()`。

这条链路说明 PC10/PC11 的串口功能不是孤立的 GPIO 配置，而是和 AFIO 重映射共同成立。

## 6. STM32实现机制

在 STM32F1 HAL 工程中，AFIO 相关动作通常出现在 MSP 初始化层，因为它们属于外设映射和底层资源配置，而不是业务逻辑。

本项目中，AFIO 相关实现分布在两个位置：

- `Core/Src/stm32f1xx_hal_msp.c` 的 `HAL_MspInit()`。
- `Core/Src/usart.c` 的 `HAL_UART_MspInit()`。

`HAL_MspInit()` 中的关键动作是：

- 启用 AFIO 时钟。
- 调用 `__HAL_AFIO_REMAP_SWJ_NOJTAG()`。

同一函数中还启用了 PWR 时钟，这是 CubeMX 生成的全局 MSP 初始化动作；本章只把它作为相邻源码证据记录，不把它解释为 USART3 或 SWD 重映射的必要条件。

`HAL_UART_MspInit()` 中的关键动作是：

- 在 USART3 分支中启用 USART3 和 GPIOC 时钟。
- 配置 PC10 为复用推挽输出。
- 配置 PC11 为输入。
- 调用 `__HAL_AFIO_REMAP_USART3_PARTIAL()`。

本章只解释这些动作的位置、作用和证据关系。USART3 的波特率、发送接收、`printf` 重定向和调试输出留到后续 UART 章节。

## 7. 项目中的应用

本章对应项目初始化流程中的引脚重映射与调试保留层。

直接相关文件：

- `Core/Src/stm32f1xx_hal_msp.c`
- `Core/Src/usart.c`
- `Three-axis_cloud_platformV2.ioc`

文件之间的关系是：

- `.ioc` 记录 PA13/PA14 的串行调试配置。
- `.ioc` 记录 PC10/PC11 的 USART3 配置。
- `stm32f1xx_hal_msp.c` 承载全局 AFIO 与调试接口重映射动作。
- `usart.c` 承载 USART3 引脚配置和 USART3 部分重映射动作。

在项目主流程中，`HAL_Init()` 会触发全局 MSP 初始化；后续 `MX_USART3_UART_Init()` 会进入 USART3 初始化链路，并触发对应 UART MSP 初始化。两者分别处理“调试接口保留”和“USART3 引脚重映射”，共同形成本章的项目主线。

## 8. 代码分析

### 1. `.ioc` 中的串行调试接口

`Three-axis_cloud_platformV2.ioc` 中 PA13 被配置为串行调试数据相关信号，PA14 被配置为串行调试时钟相关信号。

这说明项目在 CubeMX 配置层已经为调试接口保留了 PA13/PA14。教材不能把这两个引脚当成普通可自由分配 GPIO。

### 2. `HAL_MspInit()` 中的全局重映射

`Core/Src/stm32f1xx_hal_msp.c` 中 `HAL_MspInit()` 启用 AFIO 时钟，随后调用 `__HAL_AFIO_REMAP_SWJ_NOJTAG()`；同一初始化段中还启用了 PWR 时钟，但它不是本章 AFIO 重映射机制的解释重点。

该调用的项目含义是：调整调试接口相关映射，关闭 JTAG 相关占用并保留串行调试能力。它属于全局 MSP 初始化，不属于某个业务模块的运行逻辑。

### 3. `.ioc` 中的 USART3 引脚

`.ioc` 中 PC10 被配置为 USART3_TX，PC11 被配置为 USART3_RX。

这与第07章中 `usart.c` 的 GPIO 配置一致：PC10 是复用推挽输出，PC11 是输入模式。第08章进一步说明它们还需要 USART3 部分重映射支撑。

### 4. `HAL_UART_MspInit()` 中的 USART3 部分重映射

`Core/Src/usart.c` 中 `HAL_UART_MspInit()` 在 `USART3` 分支内调用 `__HAL_AFIO_REMAP_USART3_PARTIAL()`。

这说明 USART3 选择 PC10/PC11 不是单纯配置两个 GPIO 模式，还需要 AFIO 重映射参与。否则代码和 `.ioc` 中的引脚选择就无法形成完整解释链。

### 5. 和后续调试章节的边界

本章只确认串行调试接口保留与 AFIO 重映射。ST-LINK 启动配置、连接策略、GDB 停在 `main` 等内容属于后续 `ST-LINK与SWD调试配置`，不在本章展开。

### 本节证据边界

本节只根据当前仓库说明文件、函数、宏、变量和调用关系。运行时频率、外部硬件表现、主机侧现象、传感器方向、电机响应或真实控制效果仍需调试记录、日志或仓库外实测证据；缺少证据时保持【待验证】。

## 9. 调试方法

本章阶段的调试目标是确认 AFIO 重映射和调试接口保留没有互相冲突。

可观察对象：

- `.ioc` 中 PA13/PA14 是否保留为串行调试接口。
- `HAL_MspInit()` 中是否启用 AFIO 时钟。
- `HAL_MspInit()` 中是否调用 `__HAL_AFIO_REMAP_SWJ_NOJTAG()`。
- `.ioc` 中 PC10/PC11 是否配置为 USART3_TX/RX。
- `HAL_UART_MspInit()` 中是否调用 `__HAL_AFIO_REMAP_USART3_PARTIAL()`。
- `usart.c` 中 PC10/PC11 的 GPIO 模式是否与第07章一致。

常见异常定位：

- 无法使用串行调试接口：先检查 `.ioc` 中 PA13/PA14 是否仍为调试接口，再检查全局 MSP 中的调试接口重映射。
- USART3 引脚与预期不一致：先检查 `.ioc` 中 PC10/PC11，再检查 `HAL_UART_MspInit()` 是否存在部分重映射调用。
- 串口配置看似正确但引脚无输出：先确认 PC10/PC11 和 AFIO 重映射，再到 UART 章节检查波特率和发送逻辑。
- 修改引脚后工程行为异常：同时检查 `.ioc`、GPIO 配置和 AFIO 重映射，不能只改其中一个位置。

本章的证据边界是：`.ioc` 和生成代码可以证明 PA13/PA14 被保留为串行调试接口。
USART3 被部分重映射到 PC10/PC11。
但 ST-LINK 实际连线、目标板供电、外部串口转接和 PC 端工具配置仍属于仓库外实测证据，缺少实测时保持【待验证】。

调试记录建议：

- 记录 PA13/PA14 调试保留链和 PC10/PC11 USART3 重映射链。
- 对无法下载、串口无输出或引脚冲突，分别记录 `.ioc`、AFIO 宏、GPIO 配置和仓库外实测项。
- 对 ST-LINK 连线、目标板供电、串口转接器和 PC 端工具配置，缺少实测时保持【待验证】。

## 10. 常见问题

### 1. 为什么要单独讲 AFIO？

因为第07章只能说明 GPIO 模式和 MSP 位置，不能解释 USART3 为什么映射到 PC10/PC11，也不能解释调试接口为什么要保留。AFIO 正好连接这两类问题。

AFIO 是 STM32F1 系列里理解引脚复用和调试口占用的重要前置。没有这一章，读者会误以为只要配置 GPIO 模式就足够，忽略外设功能映射和调试接口保留之间的关系。

### 2. 关闭 JTAG 是否等于关闭调试？

不是。本项目的配置语义是关闭 JTAG 相关占用并保留串行调试接口。`.ioc` 中 PA13/PA14 仍保留为串行调试信号。

这类结论必须按配置语义说清楚：仓库内能证明的是固件配置保留串行调试引脚，不能证明某一次 ST-LINK 连接一定成功。实际连线、供电、驱动和主机工具状态都属于仓库外实测证据。

### 3. USART3 已经配置了 PC10/PC11，为什么还需要重映射？

因为 GPIO 模式配置只说明 PC10/PC11 怎么工作，AFIO 重映射说明 USART3 功能为什么能落在这组引脚上。二者缺一不可。

如果只有 GPIO 复用模式，没有 USART3 部分重映射证据，教材无法解释为什么 USART3 选择这组引脚。如果只有重映射，没有 GPIO 模式配置，也无法证明引脚已经按串口功能输出或输入。

### 4. 本章是否已经说明串口如何输出调试信息？

没有。本章只说明 USART3 的引脚映射前提。串口参数、`printf` 重定向和调试输出留到 UART 章节。

这也是章节边界的一部分：第08章建立“USART3 能映射到哪里”的前提，第11章才分析“字符如何从 `printf()` 走到 USART3”。不能把引脚映射直接写成串口输出已经验证。

### 5. 本章是否已经说明 ST-LINK 下载配置？

没有。本章只说明固件内部和 CubeMX 层面的串行调试接口保留。调试启动配置文件留到后续调试章节。

下载调试还需要 `.launch` 配置、调试器连接、目标板供电和下载校验等证据。本章只负责解释为什么 PA13/PA14 不应随意改作普通 GPIO；实际下载链路是否成功，留到第33章验证。

## 11. 实践任务

开始任务前，先回到本章第8节定位 AFIO 重映射、USART3 重映射和 SWD/JTAG 配置证据；第9节提供调试口与复用冲突排查顺序。

任务一：整理调试与重映射配置。

在 `.ioc` 中找到 PA13、PA14、PC10 和 PC11 的配置项。
验收依据是配置表包含引脚、功能、复用类型和证据结论四列。

任务二：追踪生成代码证据。

在 `stm32f1xx_hal_msp.c` 中找到 AFIO 时钟使能和 `__HAL_AFIO_REMAP_SWJ_NOJTAG()`。
在 `usart.c` 中找到 PC10/PC11 的 GPIO 配置和 `__HAL_AFIO_REMAP_USART3_PARTIAL()`。
验收依据是证据链表分列调试保留链、USART3 重映射链、代码位置和边界结论。

任务三：标注 SWD 与 USART3 的仓库外验证项。

画出“PA13/PA14 调试保留”和“PC10/PC11 USART3 部分重映射”两条证据链。
标出 ST-LINK 连线、目标板供电、串口转接器和 PC 端工具配置。
验收依据是记录表把仓库内证据和仓库外实测【待验证】项分成不同栏位。

## 12. 思考题

1. 为什么 PA13/PA14 不应在本项目中被随意改成普通 GPIO？
2. `HAL_MspInit()` 和 `HAL_UART_MspInit()` 中的 AFIO 动作分别解决什么问题？
3. 如果只看 `usart.c` 的 GPIO 配置而不看 AFIO 重映射，会漏掉什么解释？
4. 如果只看 `.ioc` 而不看生成代码，会有哪些风险？
5. 为什么 ST-LINK 调试配置要放到后续章节，而不是在本章展开？
6. 如何区分“保留 SWD 引脚配置”和“当前已经通过 ST-LINK 成功连接目标板”这两类证据？

## 13. 本章总结

本章建立了三轴云台项目中 AFIO 重映射与串行调试接口保留的证据链。

已经确认的结论是：

- `.ioc` 将 PA13/PA14 保留为串行调试接口。
- `HAL_MspInit()` 启用 AFIO 时钟，并调用 `__HAL_AFIO_REMAP_SWJ_NOJTAG()`；同一函数中的 PWR 时钟使能仅作为相邻全局初始化证据记录。
- `.ioc` 将 PC10/PC11 配置为 USART3_TX/RX。
- `HAL_UART_MspInit()` 调用 `__HAL_AFIO_REMAP_USART3_PARTIAL()`。
- 本章只建立引脚重映射和调试保留前提，UART 输出和 ST-LINK 调试配置留到后续章节。

本章边界：

- 保留 SWD 引脚和关闭 JTAG 是仓库内配置证据，不等于已经连接到真实目标板。
- USART3 重映射只证明引脚路径前提，实际 `printf` 输出留到第11章验证。

下一章可以进入中断系统与 SysTick 节拍，因为平台、时钟、MSP、GPIO 复用和调试接口保留已经建立，后续可以分析周期性调度和中断入口。

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

- `Core/Src/stm32f1xx_hal_msp.c`
- `Core/Src/usart.c`
- `Three-axis_cloud_platformV2.ioc`

符号、函数与配置项证据：

- `HAL_MspInit()`
- `HAL_UART_MspInit()`
- `__HAL_AFIO_REMAP_SWJ_NOJTAG()`
- `__HAL_AFIO_REMAP_USART3_PARTIAL()`
- `PA13`
- `PA14`
- `PC10`
- `PC11`
- `USART3_TX`
- `USART3_RX`
- `SYS_JTMS-SWDIO`
- `SYS_JTCK-SWCLK`

质量自检：

- P0 事实错误：通过
- P1 依赖断层：通过
- P2 逻辑连贯：通过
- P3 项目证据：通过
- P4 原理展开：通过
- P5 调试实践：通过
- P6 表达统一：通过
