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
- `AFIO->MAPR`：STM32F1 中承载多数组复用重映射和 SWJ 配置的 AFIO 映射寄存器。
- `USART3_REMAP[1:0]`：`AFIO->MAPR` 中决定 USART3 默认、部分或完全重映射的位域。
- `SWJ_CFG[2:0]`：`AFIO->MAPR` 中决定 JTAG-DP 与 SW-DP 调试接口组合状态的位域。
- 默认映射：USART3 不改 AFIO 映射时使用 PB10/PB11/PB12/PB13/PB14 这一组引脚。
- 部分重映射：USART3 迁移到 PC10/PC11/PC12/PB13/PB14，本项目采用这一组。
- 完全重映射：USART3 迁移到 PD8/PD9/PD10/PD11/PD12，本项目没有采用这一组。

这些概念都服务于正式知识点 `AFIO重映射与SWD调试`，不新增结构外知识点。

## 5. 工作原理

AFIO 重映射的核心作用是处理“同一外设功能如何落到具体引脚”的问题。

第07章已经说明 GPIO 模式决定“这个引脚怎样驱动或采样”。本章补上另一个问题：

```text
外设功能是否被映射到这组引脚
```

对 STM32F1 来说，这个问题经常落在 `AFIO->MAPR` 寄存器上。`GPIO_MODE_AF_PP`
只能说明 PC10 是复用推挽输出；`USART3_REMAP[1:0]` 才说明 USART3_TX 为什么可以选择 PC10。

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

USART3 的映射选择可以按项目证据拆成三种路线：

| 映射状态 | `USART3_REMAP[1:0]` | TX/RX 路线 | 项目结论 |
| --- | --- | --- | --- |
| 不重映射 | `00b` / `0x00000000` | PB10/PB11 | 与本项目 I2C2_SCL/SDA 冲突，不采用 |
| 部分重映射 | `01b` / `0x00000010` | PC10/PC11 | `.ioc` 和 `usart.c` 均采用 |
| 完全重映射 | `11b` / `0x00000030` | PD8/PD9 | 当前目标封装 LQFP64 不走这条路线 |

因此，USART3 partial remap 是在“避开 I2C2 默认占用”和“使用当前封装可见引脚”之间形成的工程选择。
`.ioc` 明确给出 `Mcu.CPN=STM32F103RCT6`、`Mcu.Package=LQFP64` 和
`Mcu.UserName=STM32F103RCTx`。对当前封装而言，不能把 PD8/PD9 full remap
当成等价备选路线写入项目结论。

## 6. STM32实现机制

在 STM32F1 HAL 工程中，AFIO 相关动作通常出现在 MSP 初始化层，因为它们属于外设映射和底层资源配置，而不是业务逻辑。

本项目中，AFIO 相关实现分布在两个位置：

- `Core/Src/stm32f1xx_hal_msp.c` 的 `HAL_MspInit()`。
- `Core/Src/usart.c` 的 `HAL_UART_MspInit()`。

`HAL_MspInit()` 中的关键动作是：

- 启用 AFIO 时钟。
- 调用 `__HAL_AFIO_REMAP_SWJ_NOJTAG()`。

同一函数中还启用了 PWR 时钟，这是 CubeMX 生成的全局 MSP 初始化动作；本章只把它作为相邻源码证据记录，不把它解释为 USART3 或 SWD 重映射的必要条件。

AFIO 时钟使能也可以继续向下拆到 RCC 位级证据：
`__HAL_RCC_AFIO_CLK_ENABLE()` 通过 HAL RCC 宏设置
`RCC->APB2ENR` 中的 `RCC_APB2ENR_AFIOEN` 位。没有这个时钟门控，
教材不能假设后续对 `AFIO->MAPR` 的配置路径已经具备访问前提。

`HAL_UART_MspInit()` 中的关键动作是：

- 在 USART3 分支中启用 USART3 和 GPIOC 时钟。
- 配置 PC10 为复用推挽输出。
- 配置 PC11 为输入。
- 调用 `__HAL_AFIO_REMAP_USART3_PARTIAL()`。

本章只解释这些动作的位置、作用和证据关系。USART3 的波特率、发送接收、`printf` 重定向和调试输出留到后续 UART 章节。

从 HAL 与 CMSIS 头文件继续往下看，可以得到寄存器级证据：

- `AFIO_TypeDef` 中包含 `MAPR` 和 `MAPR2` 成员。
- `AFIO` 宏指向 AFIO 外设基地址。
- `AFIO_MAPR_USART3_REMAP_Msk` 对应 `USART3_REMAP[1:0]` 位域。
- `AFIO_MAPR_USART3_REMAP_PARTIALREMAP` 对应 USART3 部分重映射。
- `AFIO_MAPR_SWJ_CFG_Msk` 对应 `SWJ_CFG[2:0]` 位域。
- `AFIO_MAPR_SWJ_CFG_JTAGDISABLE` 的注释语义是 JTAG-DP 关闭、SW-DP 保留。

位域关系可以继续拆成：

| 位域 | 位置 | 项目相关取值 | 含义 |
| --- | --- | --- | --- |
| `USART3_REMAP[1:0]` | `AFIO->MAPR[5:4]` | `0x00000010` | USART3 partial remap，TX/PC10、RX/PC11 |
| `SWJ_CFG[2:0]` | `AFIO->MAPR[26:24]` | `0x02000000` | JTAG-DP Disabled，SW-DP Enabled |
| `AFIOEN` | `RCC->APB2ENR[0]` | `1` | AFIO 外设时钟打开 |

这三个字段解决的问题不同：`AFIOEN` 解决寄存器访问前提，`SWJ_CFG`
解决调试接口组合，`USART3_REMAP` 解决 USART3 引脚路线。把它们混成一个“串口配置”
会造成讲解跳步。

HAL 宏不是普通函数调用的“标签”。`__HAL_AFIO_REMAP_USART3_PARTIAL()` 最终通过
`AFIO_REMAP_PARTIAL(...)` 对 `AFIO->MAPR` 做读改写；`__HAL_AFIO_REMAP_SWJ_NOJTAG()`
最终通过 `AFIO_DBGAFR_CONFIG(...)` 修改 `SWJ_CFG` 位域。

把项目使用的两个宏展开到位级，可以看到它们的关注点不同：

```text
__HAL_AFIO_REMAP_SWJ_NOJTAG()
-> AFIO_DBGAFR_CONFIG(AFIO_MAPR_SWJ_CFG_JTAGDISABLE)
-> tmpreg = AFIO->MAPR
-> tmpreg = (tmpreg & ~0x07000000) | 0x02000000
-> AFIO->MAPR = tmpreg

__HAL_AFIO_REMAP_USART3_PARTIAL()
-> AFIO_REMAP_PARTIAL(AFIO_MAPR_USART3_REMAP_PARTIALREMAP,
                      AFIO_MAPR_USART3_REMAP_FULLREMAP)
-> tmpreg = AFIO->MAPR
-> tmpreg = tmpreg & ~0x00000030
-> tmpreg = tmpreg | 0x07000000  ; HAL 宏携带 SWJ_CFG 掩码
-> tmpreg = tmpreg | 0x00000010
-> AFIO->MAPR = tmpreg
```

第一个宏只针对 `SWJ_CFG[2:0]` 做掩码更新；第二个宏清除 `USART3_REMAP[1:0]`
后写入 partial remap，同时在写回值中包含 `AFIO_MAPR_SWJ_CFG` 掩码。这里的
`0x07000000` 是 `SWJ_CFG[2:0]` 的掩码值，不是本项目想把调试接口配置成
`111b` 的目标状态。本项目的明确调试目标仍来自前面的
`__HAL_AFIO_REMAP_SWJ_NOJTAG()`，即 JTAG-DP Disabled、SW-DP Enabled。

教学上要强调：USART3 partial remap 不是只影响 `AFIO->MAPR[5:4]` 的孤立动作，而是一次对
`AFIO->MAPR` 的完整读改写。读者在手算或调试时必须区分“目标位域值”和“宏实现中的掩码常量”。
因此调试时应分别处理两类证据：`USART3_REMAP[1:0]` 可以作为 USART3 引脚路线的寄存器观察对象；
`SWJ_CFG[2:0]` 则要结合 `__HAL_AFIO_REMAP_SWJ_NOJTAG()` 的源码链路、调试器连接状态和 ST-LINK 日志理解，
不能只凭某一个宏曾经被调用、也不能只凭中间步骤出现 `0x07000000`，就推断最终调试接口已经关闭或已经保持。

更严格地说，RM0008 对 `SWJ_CFG` 这类调试接口配置位有特殊读写边界，调试时不应把
`AFIO->MAPR[26:24]` 的单次读回值当作唯一证据。仓库内能证明的是项目调用了
`__HAL_AFIO_REMAP_SWJ_NOJTAG()`，仓库外需要 ST-LINK 连接、下载日志或断点命中来证明 SW-DP 实际可用。

这也是为什么不建议在业务代码中手写一个完整的 `AFIO->MAPR = ...` 字面值。完整覆盖容易误伤其它重映射位或 SWJ 配置位。尤其是 `USART3_REMAP`
和 `SWJ_CFG` 共用同一个 `MAPR` 寄存器，修改串口映射后仍要确认调试接口配置没有被破坏。教材只要求读者看懂生成代码和宏链路，不要求在本项目中手写 AFIO 寄存器配置。

## 7. 项目中的应用

本章对应项目初始化流程中的引脚重映射与调试保留层。

直接相关文件：

- `Core/Src/stm32f1xx_hal_msp.c`
- `Core/Src/usart.c`
- `Three-axis_cloud_platformV2.ioc`
- `Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_gpio_ex.h`
- `Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc.h`
- `Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xe.h`

文件之间的关系是：

- `.ioc` 记录 PA13/PA14 的串行调试配置。
- `.ioc` 记录 PC10/PC11 的 USART3 配置。
- `stm32f1xx_hal_msp.c` 承载全局 AFIO 与调试接口重映射动作。
- `usart.c` 承载 USART3 引脚配置和 USART3 部分重映射动作。
- HAL GPIOEx 头文件定义项目使用的 AFIO 重映射宏。
- HAL RCC 头文件定义 `__HAL_RCC_AFIO_CLK_ENABLE()` 如何设置 AFIO 时钟门控。
- CMSIS 设备头文件定义 `AFIO->MAPR`、`USART3_REMAP` 与 `SWJ_CFG` 位域。

在项目主流程中，`HAL_Init()` 会触发全局 MSP 初始化；后续 `MX_USART3_UART_Init()` 会进入 USART3 初始化链路，并触发对应 UART MSP 初始化。两者分别处理“调试接口保留”和“USART3 引脚重映射”，共同形成本章的项目主线。

## 8. 代码分析

### 1. `.ioc` 中的串行调试接口

`Three-axis_cloud_platformV2.ioc` 中 PA13 被配置为串行调试数据相关信号，PA14 被配置为串行调试时钟相关信号。

这说明项目在 CubeMX 配置层已经为调试接口保留了 PA13/PA14。教材不能把这两个引脚当成普通可自由分配 GPIO。

更精确地说，`.ioc` 证据为：

- `PA13.Mode=Serial_Wire`
- `PA13.Signal=SYS_JTMS-SWDIO`
- `PA14.Mode=Serial_Wire`
- `PA14.Signal=SYS_JTCK-SWCLK`

这些配置只证明固件工程的配置意图。真实 ST-LINK 是否连上，还要看调试器、接线、供电、复位策略和主机工具状态。

### 2. `HAL_MspInit()` 中的全局重映射

`Core/Src/stm32f1xx_hal_msp.c` 中 `HAL_MspInit()` 启用 AFIO 时钟，随后调用 `__HAL_AFIO_REMAP_SWJ_NOJTAG()`；同一初始化段中还启用了 PWR 时钟，但它不是本章 AFIO 重映射机制的解释重点。

该调用的项目含义是：调整调试接口相关映射，关闭 JTAG 相关占用并保留串行调试能力。它属于全局 MSP 初始化，不属于某个业务模块的运行逻辑。

在 HAL GPIOEx 头文件中，相关 SWJ 选项可以拆成：

| 宏 | 配置语义 |
| --- | --- |
| `__HAL_AFIO_REMAP_SWJ_ENABLE()` | Full SWJ，JTAG-DP + SW-DP |
| `__HAL_AFIO_REMAP_SWJ_NONJTRST()` | Full SWJ，但不使用 NJTRST |
| `__HAL_AFIO_REMAP_SWJ_NOJTAG()` | JTAG-DP Disabled，SW-DP Enabled |
| `__HAL_AFIO_REMAP_SWJ_DISABLE()` | JTAG-DP Disabled，SW-DP Disabled |

本项目选择第三种。它比完全关闭 SWJ 更适合调试阶段，因为 SW-DP 仍保留；它比 Full SWJ 占用更少调试相关资源。当前仓库没有证明项目实际复用了 JTAG 被释放的其它引脚，因此不能把“释放 JTAG 资源”写成已被硬件使用的事实。

### 3. `.ioc` 中的 USART3 引脚

`.ioc` 中 PC10 被配置为 USART3_TX，PC11 被配置为 USART3_RX。

这与第07章中 `usart.c` 的 GPIO 配置一致：PC10 是复用推挽输出，PC11 是输入模式。第08章进一步说明它们还需要 USART3 部分重映射支撑。

对照 CMSIS 头文件，USART3 映射有三个取值：

- `AFIO_MAPR_USART3_REMAP_NOREMAP = 0x00000000`：TX/PB10，RX/PB11。
- `AFIO_MAPR_USART3_REMAP_PARTIALREMAP = 0x00000010`：TX/PC10，RX/PC11。
- `AFIO_MAPR_USART3_REMAP_FULLREMAP = 0x00000030`：TX/PD8，RX/PD9。

项目中 PB10/PB11 已经在 `.ioc` 中配置为 I2C2_SCL/SDA。因此 USART3 如果不重映射，会和 I2C2 的项目引脚路线冲突。项目选择 PC10/PC11 是有明确资源避让理由的。

再叠加第01章平台证据，项目目标是 `STM32F103RCTx`、封装是 `LQFP64`。ST
STM32F103xC/xD/xE 数据手册的封装引脚表中，PD8/PD9 不属于 LQFP64
引出的普通引脚路线。因此本项目不能把 USART3 full remap 当成与 PC10/PC11
同等可用的备选方案。

### 4. `HAL_UART_MspInit()` 中的 USART3 部分重映射

`Core/Src/usart.c` 中 `HAL_UART_MspInit()` 在 `USART3` 分支内调用 `__HAL_AFIO_REMAP_USART3_PARTIAL()`。

这说明 USART3 选择 PC10/PC11 不是单纯配置两个 GPIO 模式，还需要 AFIO 重映射参与。否则代码和 `.ioc` 中的引脚选择就无法形成完整解释链。

这条链路可以写成：

```text
MX_USART3_UART_Init()
-> HAL_UART_Init(&huart3)
-> HAL_UART_MspInit(&huart3)
-> __HAL_RCC_USART3_CLK_ENABLE()
-> __HAL_RCC_GPIOC_CLK_ENABLE()
-> PC10/PC11 GPIO mode
-> __HAL_AFIO_REMAP_USART3_PARTIAL()
-> AFIO->MAPR USART3_REMAP[1:0] = 01b
```

链路中的每一层解决的问题不同：USART3 时钟让外设寄存器工作，GPIOC 时钟让 PC10/PC11 配置可写，GPIO 模式让引脚进入输入或复用输出状态，AFIO 重映射让 USART3 功能选择 PC10/PC11 这组路线。

这条链路仍不能证明串口已经发送成功。发送函数、波特率、PCLK1 频率、外部转接器、主机串口工具和线序都属于第11章或仓库外实测范围。

### 5. `.launch` 中的 SWD 调试配置边界

项目根目录存在 `Three-axis_cloud_platformV2 Debug.launch`。该文件中可以看到 `swd_mode=true`、`connect_under_reset`、`stopAt=main` 等 IDE 调试配置。

这些配置与本章的 SWD 保留方向一致，但它们仍然只是工程配置证据。实际下载和调试是否成功，还需要 ST-LINK 硬件、目标板供电、复位线、驱动、GDB server 日志或现场调试记录证明。

### 6. 和后续调试章节的边界

本章只确认串行调试接口保留与 AFIO 重映射。ST-LINK 启动配置、连接策略、GDB 停在 `main` 等内容属于后续 `ST-LINK与SWD调试配置`，不在本章展开。

### 7. 构建产物证据边界

当前 Debug 构建产物可以把 AFIO/SWD 证据从“源码中写了宏”推进到“宏所在函数进入最终镜像，并在 `.list` 中展开为对 `AFIO->MAPR` 的读改写路径”。这类证据适合确认入口、链接和函数级资源，但仍不能证明调试器已经连上目标板，也不能证明 USART3 已经实际输出字符。

| 链路环节 | 函数或路径 | 静态栈估计 | 圈复杂度 | 证据文件 | 证据边界 |
|---|---|---:|---:|---|---|
| 全局MSP入口 | `HAL_Init()` 调用 `HAL_MspInit()` | - | - | `Debug/Three-axis_cloud_platformV2.list` | 证明 HAL 初始化主线会进入全局 MSP，不能证明 ST-LINK 已经连接成功。 |
| SWD保留宏所在函数 | `HAL_MspInit()` | 24 字节 | 1 | `Debug/Core/Src/stm32f1xx_hal_msp.su` / `.cyclo` | 证明全局 MSP 函数进入构建，并包含 AFIO/PWR 时钟与 NOJTAG 配置入口。 |
| NOJTAG宏展开 | `__HAL_AFIO_REMAP_SWJ_NOJTAG()` | - | - | `Debug/Three-axis_cloud_platformV2.list` | 证明当前反汇编旁注记录了 NOJTAG 宏路径，不能替代 ST-LINK 下载日志或断点证据。 |
| USART3 MSP入口 | `HAL_UART_MspInit()` | 48 字节 | 2 | `Debug/Core/Src/usart.su` / `.cyclo` | 证明 USART3 MSP 函数进入构建，不能证明串口发送或主机接收成功。 |
| USART3部分重映射宏展开 | `__HAL_AFIO_REMAP_USART3_PARTIAL()` | - | - | `Debug/Three-axis_cloud_platformV2.list` | 证明 `.list` 中保留了 partial remap 源码旁注和写 `AFIO->MAPR` 的路径，不能证明 PC10/PC11 外部连线正确。 |
| 符号进入镜像 | `HAL_MspInit()` / `HAL_UART_MspInit()` | - | - | `Debug/Three-axis_cloud_platformV2.map` | 证明两个 MSP 符号被链接进当前镜像，不能证明每次上电都越过所有初始化错误路径。 |
| 宏语义来源 | `AFIO_MAPR_USART3_REMAP_*` / `AFIO_MAPR_SWJ_CFG_*` | - | - | `stm32f1xx_hal_gpio_ex.h` / `stm32f103xe.h` | 证明 partial remap 和 JTAG disable 的位域常量来源，不能替代运行时寄存器读数。 |

### 7.1 `AFIO->MAPR` 最终地址与宏展开边界

当前 `Debug/Three-axis_cloud_platformV2.map` 中，`HAL_MspInit()` 位于 `0x08001a58`，`HAL_UART_MspInit()` 位于 `0x080024bc`。这说明本章讨论的两个关键入口已经进入当前最终 Flash 镜像，而不是只停留在源码文本或输入对象文件里。

`Debug/Three-axis_cloud_platformV2.list` 还能把这两个入口展开到指令级：`HAL_MspInit()` 中能看到 `__HAL_AFIO_REMAP_SWJ_NOJTAG()`，其反汇编旁注指向对 `AFIO->MAPR` 的读改写；`HAL_UART_MspInit()` 中能看到 `__HAL_AFIO_REMAP_USART3_PARTIAL()`，其反汇编同样指向 `AFIO->MAPR` 的读改写流程。对照头文件，`AFIO_MAPR_USART3_REMAP_PARTIALREMAP` 的值是 `0x00000010`，`AFIO_MAPR_SWJ_CFG_JTAGDISABLE` 的值是 `0x02000000`，而 `AFIO_MAPR_SWJ_CFG_Msk` 的掩码是 `0x07000000`。

这说明本章可以精确写成“当前 Debug 构建中，AFIO 重映射和 SWD 保留的宏调用已经进入最终镜像，并在 `.list` 中展开为对 `AFIO->MAPR` 的读改写路径”。但仍不能把这等同为“调试器已经连上目标板”或“USART3 一定已经在 PC10/PC11 上正常收发”；前者需要 ST-LINK/GDB 或下载日志，后者还需要外部串口日志和引脚电平证据【待验证】。同样，`AFIO_REMAP_PARTIAL()` 的写回语义是对 `MAPR` 做掩码读改写，不是把某个单一常量整寄存器覆盖到 `AFIO->MAPR`。

因此，第08章引用 `.map/.list/.su/.cyclo` 时只能证明“当前 Debug 构建包含 AFIO/SWD 与 USART3 remap 相关入口和宏展开痕迹”。它们不能简单相加成最大栈深，不能换算成真实执行时间，也不能把 `SWJ_CFG` 或 `USART3_REMAP` 写成已经通过外部工具验证；这些仍需 `RCC->APB2ENR`、`AFIO->MAPR` 运行观察、ST-LINK/GDB 日志、串口主机日志或示波器/逻辑分析仪证据【待验证】。

### 本节证据边界

本节只根据当前仓库说明文件、函数、宏、变量、调用关系和 Debug 构建产物。运行时频率、外部硬件表现、主机侧现象、传感器方向、电机响应、真实栈水位、函数耗时或真实控制效果仍需调试记录、日志或仓库外实测证据；缺少证据时保持【待验证】。

## 9. 调试方法

本章阶段的调试目标是确认 AFIO 重映射和调试接口保留没有互相冲突。

可观察对象：

- `.ioc` 中 PA13/PA14 是否保留为串行调试接口。
- `HAL_MspInit()` 中是否启用 AFIO 时钟。
- `RCC->APB2ENR.AFIOEN` 是否为 1。
- `HAL_MspInit()` 中是否调用 `__HAL_AFIO_REMAP_SWJ_NOJTAG()`。
- 若读取 `AFIO->MAPR[26:24]`，必须标注 `SWJ_CFG` 读回证据边界，不能把它作为唯一调试口证据。
- `.ioc` 中 PC10/PC11 是否配置为 USART3_TX/RX。
- `HAL_UART_MspInit()` 中是否调用 `__HAL_AFIO_REMAP_USART3_PARTIAL()`。
- `AFIO->MAPR[5:4]` 是否对应 USART3 partial remap。
- 所有 AFIO 宏执行后，`AFIO->MAPR[5:4]`、`__HAL_AFIO_REMAP_SWJ_NOJTAG()` 源码链和 ST-LINK 实测证据是否相互一致。
- `usart.c` 中 PC10/PC11 的 GPIO 模式是否与第07章一致。

常见异常定位：

- 无法使用串行调试接口：先检查 `.ioc` 中 PA13/PA14 是否仍为调试接口，再检查全局 MSP 中的调试接口重映射。
- USART3 引脚与预期不一致：先检查 `.ioc` 中 PC10/PC11，再检查 `HAL_UART_MspInit()` 是否存在部分重映射调用。
- 串口配置看似正确但引脚无输出：先确认 PC10/PC11 和 AFIO 重映射，再到 UART 章节检查波特率和发送逻辑。
- 修改引脚后工程行为异常：同时检查 `.ioc`、GPIO 配置和 AFIO 重映射，不能只改其中一个位置。
- 如果误把 USART3 改回默认映射：检查 PB10/PB11 是否已经被 I2C2 占用。
- 如果误把 SWJ 完全关闭：后续调试可能需要 connect under reset 或重新下载恢复，不能把它当作普通 GPIO 改动。

建议按下面顺序排查：

```text
1. .ioc 是否保留 PA13/PA14 Serial_Wire
2. HAL_MspInit() 是否先启用 AFIO 时钟
3. RCC->APB2ENR.AFIOEN 是否已经置位
4. 源码链是否调用 NOJTAG 而不是 DISABLE，并记录 SWJ_CFG 读回边界
5. .ioc 中 USART3 是否选择 PC10/PC11
6. usart.c 是否配置 PC10/PC11 GPIO 模式
7. usart.c 是否调用 USART3 partial remap
8. USART3_REMAP[1:0] 是否为 partial remap
9. USART3 remap 之后 ST-LINK 连接、下载或断点证据是否仍符合调试口预期【待验证】
10. PB10/PB11 是否同时承担 I2C2，避免 USART3 默认映射冲突
11. ST-LINK/串口工具/外部线序是否有实测证据【待验证】
```

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

从项目资源分配看，默认 USART3_TX/RX 对应 PB10/PB11，而这两个引脚已经是 I2C2_SCL/SDA。部分重映射把 USART3_TX/RX 放到 PC10/PC11，避免了这组冲突。

### 4. 本章是否已经说明串口如何输出调试信息？

没有。本章只说明 USART3 的引脚映射前提。串口参数、`printf` 重定向和调试输出留到 UART 章节。

这也是章节边界的一部分：第08章建立“USART3 能映射到哪里”的前提，第11章才分析“字符如何从 `printf()` 走到 USART3”。不能把引脚映射直接写成串口输出已经验证。

### 5. 本章是否已经说明 ST-LINK 下载配置？

没有。本章只说明固件内部和 CubeMX 层面的串行调试接口保留。调试启动配置文件留到后续调试章节。

下载调试还需要 `.launch` 配置、调试器连接、目标板供电和下载校验等证据。本章只负责解释为什么 PA13/PA14 不应随意改作普通 GPIO；实际下载链路是否成功，留到第33章验证。

### 6. 为什么不直接完全关闭 SWJ，把 PA13/PA14 也释放出来？

因为项目仍需要调试和下载通路。`__HAL_AFIO_REMAP_SWJ_DISABLE()` 的语义是 JTAG-DP 和 SW-DP 都关闭；这会让运行后的调试访问变得更困难，通常只应在有明确量产或引脚复用理由、并有恢复手段时考虑。

本项目选择 `NOJTAG`，保留 SW-DP，对教材和调试阶段更稳妥。仓库当前没有硬件证据说明 PA13/PA14 必须释放给其它功能。

### 7. 为什么不使用 USART3 完全重映射？

完全重映射把 USART3_TX/RX 放到 PD8/PD9。当前 `.ioc` 目标为
`STM32F103RCTx` 和 `LQFP64`，而 ST 数据手册封装引脚表没有把 PD8/PD9
作为 LQFP64 的可用普通引脚列出。当前 `.ioc` 也只使用 PD0/PD1 作为 HSE，
没有 PD8/PD9 这条 USART3 路线。

因此，从当前工程证据和封装边界看，PC10/PC11 的部分重映射比 PD8/PD9
完全重映射更符合项目引脚分配。

### 8. 为什么不能用 `AFIO->MAPR = 某个常量` 快速配置？

因为 `AFIO->MAPR` 同时承载 USART、TIM、CAN、PD0/PD1、SWJ 等多类映射位。
本章只关心 USART3 和 SWJ，但项目中其它章节还会依赖定时器、USB、时钟和调试资源。

如果直接整寄存器赋值，可能在配置 USART3 的同时改掉 `SWJ_CFG` 或其它外设映射。
HAL 的重映射宏采用掩码读改写，就是为了只修改目标位域。教材允许读者理解这些位域，
但不建议把演示代码改成手写 `AFIO->MAPR` 常量。

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

任务四：计算 AFIO 位域目标值。

从 CMSIS 头文件中找出 `AFIO_MAPR_USART3_REMAP_Pos`、
`AFIO_MAPR_USART3_REMAP_PARTIALREMAP`、`AFIO_MAPR_SWJ_CFG_JTAGDISABLE`
和 `RCC_APB2ENR_AFIOEN`。
验收依据是能写出 USART3 partial remap 对应 `AFIO->MAPR[5:4] = 01b`，
NOJTAG 对应 `AFIO->MAPR[26:24]` 中的 JTAG disable 语义，
AFIO 时钟对应 `RCC->APB2ENR[0] = 1`，并能解释
`AFIO_REMAP_PARTIAL()` 为什么必须看作 `MAPR` 读改写，而不是单个位写入。
同时必须说明宏中的 `AFIO_MAPR_SWJ_CFG = 0x07000000` 是位域掩码，
不是本项目 `SWJ_CFG` 的目标取值；项目目标取值仍由
`AFIO_MAPR_SWJ_CFG_JTAGDISABLE = 0x02000000` 说明。若把调试器中的
`AFIO->MAPR[26:24]` 作为观察项，必须标注它弱于源码宏链和 ST-LINK 实测证据。

任务五：核对封装边界。

在 `.ioc` 中确认 `Mcu.CPN=STM32F103RCT6`、`Mcu.Package=LQFP64`，
再结合 ST 数据手册封装引脚表说明为什么 PD8/PD9 full remap
不能作为当前项目的等价路线。
验收依据是说明中同时引用工程配置证据和数据手册封装证据。

## 12. 思考题

1. 为什么 PA13/PA14 不应在本项目中被随意改成普通 GPIO？
2. `HAL_MspInit()` 和 `HAL_UART_MspInit()` 中的 AFIO 动作分别解决什么问题？
3. 如果只看 `usart.c` 的 GPIO 配置而不看 AFIO 重映射，会漏掉什么解释？
4. 如果只看 `.ioc` 而不看生成代码，会有哪些风险？
5. 为什么 ST-LINK 调试配置要放到后续章节，而不是在本章展开？
6. 如何区分“保留 SWD 引脚配置”和“当前已经通过 ST-LINK 成功连接目标板”这两类证据？
7. 如果 USART3 不做 partial remap，会和项目中的哪个外设引脚分配产生冲突？
8. 为什么直接覆盖写 `AFIO->MAPR` 比调用 HAL 生成宏更容易引入调试口或其它重映射问题？
9. 为什么 `NOJTAG` 比 `DISABLE` 更适合保留开发阶段的调试能力？
10. 为什么 `STM32F103RCTx + LQFP64` 会影响 USART3 full remap 是否能作为备选方案？

## 13. 本章总结

本章建立了三轴云台项目中 AFIO 重映射与串行调试接口保留的证据链。

已经确认的结论是：

- `.ioc` 将 PA13/PA14 保留为串行调试接口。
- `HAL_MspInit()` 启用 AFIO 时钟，并调用 `__HAL_AFIO_REMAP_SWJ_NOJTAG()`；同一函数中的 PWR 时钟使能仅作为相邻全局初始化证据记录。
- `.ioc` 将 PC10/PC11 配置为 USART3_TX/RX。
- `HAL_UART_MspInit()` 调用 `__HAL_AFIO_REMAP_USART3_PARTIAL()`。
- USART3 默认映射会占用 PB10/PB11，与项目 I2C2 路线冲突。
- USART3 部分重映射对应 PC10/PC11，是当前工程采用的路线。
- USART3 full remap 对应 PD8/PD9，但当前目标封装 LQFP64 不把它作为等价可用路线。
- `NOJTAG` 表示 JTAG-DP Disabled、SW-DP Enabled，不等于完全关闭调试接口。
- AFIO 时钟来自 `RCC->APB2ENR.AFIOEN`，`USART3_REMAP` 和 `SWJ_CFG`
  都落在 `AFIO->MAPR`，不能用整寄存器常量随意覆盖。
- `__HAL_AFIO_REMAP_USART3_PARTIAL()` 会通过 `AFIO_REMAP_PARTIAL()` 对
  `AFIO->MAPR` 做读改写；USART3 路线可看 `USART3_REMAP[1:0]`，
  调试口保留则应结合 NOJTAG 源码链和 ST-LINK 实测证据。
- `AFIO_REMAP_PARTIAL()` 内部出现的 `AFIO_MAPR_SWJ_CFG = 0x07000000`
  是掩码常量，不是项目最终调试接口目标值；不能把它误读成关闭 SW-DP。
- `.map/.list` 能证明 `HAL_MspInit()`、`HAL_UART_MspInit()` 进入当前镜像并保留 NOJTAG/USART3 partial remap 的源码旁注，`.su/.cyclo` 能证明两个 MSP 函数的静态栈和圈复杂度条目。
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
- `Three-axis_cloud_platformV2 Debug.launch`
- `Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_gpio_ex.h`
- `Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc.h`
- `Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xe.h`
- `Debug/Three-axis_cloud_platformV2.map`
- `Debug/Three-axis_cloud_platformV2.list`
- `Debug/Core/Src/stm32f1xx_hal_msp.su`
- `Debug/Core/Src/stm32f1xx_hal_msp.cyclo`
- `Debug/Core/Src/usart.su`
- `Debug/Core/Src/usart.cyclo`

外部权威资料：

- ST RM0008 Reference manual: `https://www.st.com/resource/en/reference_manual/rm0008-stm32f101xx-stm32f102xx-stm32f103xx-stm32f105xx-and-stm32f107xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf`
- ST DS5792 Datasheet for STM32F103xC/xD/xE: `https://www.st.com/resource/en/datasheet/stm32f103rc.pdf`
- ST STM32F103 documentation page: `https://www.st.com/en/microcontrollers-microprocessors/stm32f103/documentation.html`

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
- `AFIO`
- `AFIO->MAPR`
- `AFIO_TypeDef`
- `MAPR`
- `MAPR2`
- `AFIO_REMAP_PARTIAL`
- `AFIO_DBGAFR_CONFIG`
- `AFIO_MAPR_SWJ_CFG`
- `AFIO_MAPR_USART3_REMAP_Msk`
- `AFIO_MAPR_USART3_REMAP_Pos`
- `AFIO_MAPR_USART3_REMAP_NOREMAP`
- `AFIO_MAPR_USART3_REMAP_PARTIALREMAP`
- `AFIO_MAPR_USART3_REMAP_FULLREMAP`
- `AFIO_MAPR_SWJ_CFG_Msk`
- `AFIO_MAPR_SWJ_CFG_Pos`
- `AFIO_MAPR_SWJ_CFG_RESET`
- `AFIO_MAPR_SWJ_CFG_JTAGDISABLE`
- `AFIO_MAPR_SWJ_CFG_DISABLE`
- `RCC->APB2ENR`
- `RCC_APB2ENR_AFIOEN`
- `Mcu.CPN=STM32F103RCT6`
- `Mcu.Package=LQFP64`
- `PB10`
- `PB11`
- `PD8`
- `PD9`

质量自检：

- P0 事实错误：通过
- P1 依赖断层：通过
- P2 逻辑连贯：通过
- P3 项目证据：通过
- P4 原理展开：通过
- P5 调试实践：通过
- P6 表达统一：通过
