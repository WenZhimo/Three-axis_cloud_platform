# 第01章 MCU平台与项目边界

> 导航：上一章：无 ｜ 下一章：[第02章_STM32CubeIDE工程结构](第02章_STM32CubeIDE工程结构.md)

## 1. 本章目标

- 识别本项目的目标芯片、封装和工程边界。
- 看懂为什么后续所有启动、时钟、外设和构建分析都必须先从平台开始。
- 为后续章节建立统一的“项目到底跑在哪颗芯片上”这一前提。

本章阅读分层：

| 阅读层次 | 建议范围 | 适合读者 |
|---|---|---|
| 【必须掌握】 | 第3节到第7节，先确认目标芯片、封装、工程边界和平台锁定链路 | 第一次阅读本教程，需要建立全书共同平台前提的读者 |
| 【工程深化】 | 第8.1节到第8.5节，重点核对 `.ioc`、`.cproject`、CMSIS宏、启动文件、链接脚本和构建产物证据 | 需要把平台结论落实到工程文件、编译配置和 Debug 构建结果中的读者 |
| 【拓展阅读】 | 第8.5.1节、第8.5.2节和第10节，理解 `.list`、`.map` 与官方资料各自能证明到哪一层 | 已经能读懂基础平台链路，想进一步区分证据强度和结论边界的读者 |
| 【证据与验证】 | 第9节、第11节和“本章边界”，只把仓库内证据写成已验证事实，实物板状态继续保留【待验证】 | 准备复查教材结论、维护证据表或后续接入下载调试记录的读者 |

如果只是沿三轴云台主线学习，可以先抓住“STM32F103RCTx 平台 -> 编译宏 -> 启动文件 -> 链接脚本 -> Debug 构建产物”这条链；等需要判断某个结论能否写成已验证事实时，再回到第8节和第9节核对证据边界。

## 2. 前置知识

- 无

## 3. 问题背景

三轴云台项目不是抽象的 STM32 练习题，而是一个绑定了具体 MCU、封装、时钟和外设资源的工程。  
如果平台没认清，后面的时钟树、定时器、I2C、USB、启动文件和链接脚本都会失去解释基础。

这个项目的起点不是“先学某个外设”，而是先确认：

- 用的是什么芯片。
- 选了什么封装。
- 工程是怎么被 CubeMX 和 CubeIDE 组织起来的。
- 哪些资源是项目真实使用的，哪些只是库里存在但项目没用。

## 4. 核心概念

本章只建立平台层概念，不展开外设细节。

- MCU：项目运行的目标芯片。
- 封装：芯片可用引脚和硬件连接边界。
- 工程边界：CubeMX 配置、编译器宏、链接脚本和工程元信息共同定义的项目范围。
- 平台一致性：`.ioc`、`.cproject`、`.mxproject` 和链接脚本必须指向同一个目标芯片。

### 4.1 平台命名层级

同一个 STM32 项目里经常会同时出现多个相似名字。它们不是随意混用，而是来自不同工具和不同抽象层：

- `STM32F103RCT6`：`.ioc` 中的 `Mcu.CPN`，更接近具体订货型号。
- `STM32F103RCTx`：`.ioc` 的 `Mcu.UserName` 和 `.cproject` 的目标 MCU，表示当前工程面向的芯片平台。
- `STM32F103R(C-D-E)Tx`：`.ioc` 的 `Mcu.Name`，表示 CubeMX 数据库中的系列/容量族表达。
- `STM32F103xE`：编译器宏，决定 `stm32f1xx.h` 最终包含 `stm32f103xe.h`。
- `startup_stm32f103rctx.s`：启动文件名，说明当前构建使用 RCTx 对应启动向量表。
- `STM32F103RCTX_FLASH.ld`：链接脚本名，说明 FLASH/RAM 布局按该平台生成。

这些名字必须被放在同一条证据链里理解。`STM32F103xE` 不是封装名，`LQFP64` 也不是容量宏；如果把它们混成一个概念，后面分析寄存器头文件、启动向量表或内存布局时就会跳步。

### 4.2 平台证据层级

本章把平台证据分成六层：

```text
CubeMX配置证据 -> IDE构建证据 -> CMSIS宏证据
-> 启动文件证据 -> 链接脚本证据 -> 构建产物证据
```

其中前五层说明工程“按哪个 MCU 平台组织和编译”。构建产物证据说明某次 Debug 构建实际把哪些对象和链接脚本送进了 ELF。

但这些仍然不是板卡实物证据。芯片丝印、焊接封装、电源、外部晶振、调试线和外设连线仍属于仓库外证据；缺少实测或照片时保持【待验证】。

## 5. 工作原理

从教材视角看，一个 STM32 项目首先要完成三件事：

1. 让工程知道目标芯片是谁。
2. 让编译器和链接器知道代码、堆栈和内存该放在哪里。
3. 让 CubeMX 生成的外设配置和源码结构围绕同一平台展开。

本项目的核心平台信息已经在多个文件中互相印证：

- `.ioc` 明确写出了 `Mcu.CPN=STM32F103RCT6`、`Mcu.Family=STM32F1`、`Mcu.Package=LQFP64`、`Mcu.UserName=STM32F103RCTx`。
- `.cproject` 的目标 MCU 是 `STM32F103RCTx`，并启用了 `USE_HAL_DRIVER` 和 `STM32F103xE`。
- 链接脚本 `STM32F103RCTX_FLASH.ld` 定义了 RAM 和 FLASH 大小，为后续启动流程与内存布局分析提供平台边界。

这意味着本项目不是“泛用 STM32 工程”，而是一个已经被固定到 STM32F103RCTx 平台上的三轴云台工程。

### 5.1 平台锁定链路

更细地看，当前平台不是由某一个文件单独决定，而是由一串互相约束的文件共同锁定：

```text
Three-axis_cloud_platformV2.ioc
-> .cproject
-> STM32F103xE宏
-> stm32f1xx.h
-> stm32f103xe.h
-> startup_stm32f103rctx.s
-> STM32F103RCTX_FLASH.ld
-> Debug/makefile与objects.list
```

这条链路的含义是：

- `.ioc` 定义 CubeMX 视角下的芯片、封装、引脚和外设选择。
- `.cproject` 把目标 MCU、头文件路径、编译宏和链接脚本交给构建系统。
- `STM32F103xE` 让 `stm32f1xx.h` 选择 `stm32f103xe.h`，从而得到寄存器结构体、中断号和外设基地址。
- `startup_stm32f103rctx.s` 提供复位入口、向量表、`.data` 搬运和 `.bss` 清零逻辑。
- `STM32F103RCTX_FLASH.ld` 定义 FLASH、RAM、栈顶、堆栈保留区和各段放置位置。
- `Debug/makefile` 和 `Debug/objects.list` 证明 Debug 构建实际使用了启动对象和链接脚本。

如果这条链路中任何一处偏离，后续章节就不能继续假设平台一致。例如：`.ioc` 仍是 RCTx，但 `.cproject` 宏改成 `STM32F103xB`，编译器会进入另一个设备头文件分支；链接脚本若仍保留 256K FLASH/48K RAM，则内存边界和设备头文件的容量语义就会失配。

### 5.2 为什么不能只看芯片名字

芯片名字只能说明平台意图，不能自动证明所有底层配置正确。工程上至少还要追问：

- 容量宏是否与设备头文件一致。
- 启动文件是否与容量族一致。
- 链接脚本是否与目标 FLASH/RAM 一致。
- 构建产物是否真的使用这些对象和脚本。
- 外设引脚是否仍落在当前封装可用引脚上。

因此第01章不是简单记录“用 STM32F103”，而是建立一种证据读法：平台结论必须由配置、编译、链接、启动和构建产物共同支撑。

## 6. STM32实现机制

在 STM32 工程里，平台首先体现在三层机制上：

- 目标芯片选择：决定可用外设、寄存器地址和中断资源。
- 编译与链接配置：决定预处理宏、头文件路径、链接脚本和输出格式。
- 生成元信息：记录 CubeMX/CubeIDE 生成文件、库文件和工程路径。

本项目已经落实到以下配置：

- `Three-axis_cloud_platformV2.ioc` 绑定 STM32F103RCTx。
- `.cproject` 配置 ARM GCC、目标 MCU、宏定义和链接脚本。
- `.mxproject` 记录 CubeMX/CubeIDE 生成文件、历史库文件、HeaderPath 和 CDefines。
- `STM32F103RCTX_FLASH.ld` 绑定 256K FLASH 和 48K RAM。

这也是为什么本书必须先讲平台，再讲外设：平台错了，后面的所有结论都会漂。

### 6.1 CMSIS宏分派机制

`.cproject` 中定义 `STM32F103xE` 后，CMSIS 设备总头文件 `stm32f1xx.h` 会进入对应条件分支：

```text
#elif defined(STM32F103xE)
  #include "stm32f103xe.h"
```

这个分派决定了后续 C 代码看到哪些寄存器结构体、IRQn 枚举、外设基地址和位定义。换句话说，`STM32F103xE` 不只是一个给 IDE 看看的标签，它会影响实际编译时的头文件内容。

这也是为什么第05章讲 CMSIS 之前，第01章必须先把平台宏讲清楚。否则读者看到 `TIM2`、`RCC`、`USB_LP_CAN1_RX0_IRQn` 或 `FLASH_BASE` 时，只知道它们“来自头文件”，却不知道为什么是这一个设备头文件。

### 6.2 启动文件与链接脚本耦合

`startup_stm32f103rctx.s` 的 `Reset_Handler` 会引用 `_sidata`、`_sdata`、`_edata`、`_sbss` 和 `_ebss` 等符号，用于把初始化数据从 FLASH 搬到 RAM，并清零 `.bss`。

这些符号不是启动文件自己定义的，而是由 `STM32F103RCTX_FLASH.ld` 提供。链接脚本中：

```text
RAM   ORIGIN = 0x20000000, LENGTH = 48K
FLASH ORIGIN = 0x08000000, LENGTH = 256K
_estack = ORIGIN(RAM) + LENGTH(RAM)
```

所以启动文件和链接脚本必须一致。启动文件负责“怎么启动”，链接脚本负责“从哪里搬、搬到哪里、栈顶在哪里”。如果只看启动文件，不看链接脚本，就无法判断 RAM/FLASH 边界是否满足当前工程。

### 6.3 封装与引脚资源边界

`.ioc` 中 `Mcu.Package=LQFP64`，并记录 `Mcu.PinsNb=29`。这说明 CubeMX 当前按 LQFP64 封装分配引脚，例如 PA13/PA14 用于 SWD，PB10/PB11 用于 I2C2，PA11/PA12 用于 USB，PC10/PC11 用于 USART3。

封装边界会影响后续所有引脚章节：如果换成同系列不同封装，即使内核和寄存器头文件仍然相似，也可能没有相同引脚可用，或同一外设通道需要重新映射。

但这里仍然只能证明工程配置。实物板上是否焊接了 LQFP64 的 STM32F103RCT6、外部连线是否接到这些引脚、晶振和电源是否符合设计，都不能由 `.ioc` 单独证明。

## 7. 项目中的应用

本章对应项目里的最上层边界：

- 项目名称：`Three-axis_cloud_platformV2`
- 目标芯片：`STM32F103RCTx`
- 目标封装：`LQFP64`
- 工程组织：`CubeMX + STM32CubeIDE + GCC`

项目中与本章直接相关的文件：

- `Three-axis_cloud_platformV2.ioc`
- `.cproject`
- `.mxproject`
- `STM32F103RCTX_FLASH.ld`

本章的作用不是去解释某个外设怎么用，而是先确认项目“落地在哪个硬件平台上”。

## 8. 代码分析

本章不进入业务代码细节，但要说明几个关键位置的意义。第01章的代码分析不是追踪函数逻辑，而是追踪“平台事实从哪里进入工程”。

- `.ioc` 负责定义芯片和外设资源边界，是 CubeMX 生成配置的源头。
- `.cproject` 负责把 MCU、宏、头文件路径和链接脚本送进编译器/链接器。
- `.mxproject` 记录 CubeMX/CubeIDE 生成和使用过的库文件、路径和构建信息，是平台证据的一部分。
- `STM32F103RCTX_FLASH.ld` 决定程序、数据和栈在 FLASH/RAM 中的布局。

这一章的“代码分析”重点不是逻辑，而是边界确认：这些文件共同证明本项目确实运行在 STM32F103RCTx 这条硬件线上。

### 8.1 `.ioc` 平台字段

当前 `.ioc` 中的关键字段包括：

```text
Mcu.CPN=STM32F103RCT6
Mcu.Family=STM32F1
Mcu.Name=STM32F103R(C-D-E)Tx
Mcu.Package=LQFP64
Mcu.UserName=STM32F103RCTx
ProjectManager.DeviceId=STM32F103RCTx
```

这些字段说明 CubeMX 配置源把项目放在 STM32F103RCTx 平台和 LQFP64 封装上。

同时，`.ioc` 还列出当前工程选择的 IP：

```text
I2C2, NVIC, RCC, SYS, TIM2, TIM3, TIM4, TIM6, TIM8, USART3, USB, USB_DEVICE
```

这些 IP 字段说明工程配置中有哪些外设入口，但不能单独证明外设已经在业务路径中运行。后续章节必须继续检查生成源码、初始化调用、启动函数、主循环调用和外部实测。

### 8.2 `.cproject` 编译字段

`.cproject` 中 Debug 配置至少提供四类平台证据：

```text
target_mcu = STM32F103RCTx
target_board = genericBoard
Define symbols = USE_HAL_DRIVER, STM32F103xE
Linker Script = STM32F103RCTX_FLASH.ld
```

其中 `genericBoard` 只说明工程没有绑定某个 ST 官方评估板模板，不能证明项目实物板的 PCB、供电或外设连接。真正能继续支撑代码编译的是目标 MCU、宏定义、头文件路径和链接脚本。

### 8.3 CMSIS设备头文件

`stm32f1xx.h` 根据宏选择具体设备头文件。当前宏为 `STM32F103xE`，所以会包含：

```text
stm32f103xe.h
```

`stm32f103xe.h` 中定义了核心内存基地址，例如：

```text
FLASH_BASE = 0x08000000
SRAM_BASE  = 0x20000000
PERIPH_BASE = 0x40000000
```

这些定义为第03章启动布局、第05章 CMSIS、第06章时钟、第09章中断、第12章定时器和后续寄存器访问提供共同语言。

这里还有一个容量边界：`stm32f103xe.h` 是 `STM32F103xE` 容量族头文件，不是本项目链接边界的唯一来源。当前工程最终允许程序放入多大 FLASH、RAM 起止在哪里，仍要看 `STM32F103RCTX_FLASH.ld` 中的 `MEMORY` 区域。教材不能因为设备头文件中存在某个更宽的族级地址定义，就把本项目写成拥有更大可用 FLASH。

### 8.4 启动文件与链接脚本证据

启动文件 `Core/Startup/startup_stm32f103rctx.s` 明确使用 Cortex-M3 和 softvfp：

```text
.cpu cortex-m3
.fpu softvfp
```

它的 `Reset_Handler` 会先调用 `SystemInit`，再搬运 `.data`、清零 `.bss`，最后进入 C 库初始化和 `main()` 路径。它引用的 `_sidata`、`_sdata`、`_edata`、`_sbss`、`_ebss` 等符号来自链接脚本。

`STM32F103RCTX_FLASH.ld` 则定义：

```text
ENTRY(Reset_Handler)
RAM   = 0x20000000, 48K
FLASH = 0x08000000, 256K
```

因此，本项目的平台边界不是只靠 `.ioc` 声明，还被启动文件和链接脚本落到了复位入口、段初始化和内存地址上。

### 8.5 Debug构建产物证据

`Debug/objects.list` 中包含：

```text
./Core/Startup/startup_stm32f103rctx.o
```

`Debug/makefile` 的链接命令使用：

```text
-T STM32F103RCTX_FLASH.ld
```

这说明 Debug 构建产物层面确实把当前启动对象和链接脚本纳入链接。它能证明构建链路的静态事实，但仍不能证明目标板已经下载并运行这个 ELF；下载和运行证据留到第33章。

这里还要单独拆开一个文件名边界。`Core/Startup/startup_stm32f103rctx.s` 是源码路径证据，说明仓库中存在 RCTx 对应启动文件；`Debug/objects.list` 中的 `./Core/Startup/startup_stm32f103rctx.o` 才是对象进入当前 Debug 链接输入的证据；`Debug/makefile` 中的 `-T STM32F103RCTX_FLASH.ld` 则证明本次链接命令使用了对应链接脚本。如果将来出现拼写相近但不完全一致的启动文件名，或者 `objects.list` 中没有对应 `.o`，教材应把它视为证据链断点，而不是把名称差异当成无关变体。

### 8.5.1 `.list`中的启动链接证据边界

`Debug/Three-axis_cloud_platformV2.list` 还能把“启动文件引用链接脚本符号”推进到反汇编和源码混排层。当前构建中可以看到启动代码围绕 `.data` 搬运、`.bss` 清零和栈边界检查展开；同一个 `.list` 中还出现：

```text
extern uint8_t _estack;
extern uint32_t _Min_Stack_Size;
const uint32_t stack_limit = (uint32_t)&_estack - (uint32_t)&_Min_Stack_Size;
```

这些符号来自 `STM32F103RCTX_FLASH.ld`，而不是普通 C 变量。它们把第03章要讲的启动流程提前给出一个平台层证据：当前 Debug ELF 的启动代码确实依赖链接脚本提供的 RAM 顶、栈保留大小、初始化数据边界和 `.bss` 边界。

这层证据的教学价值在于区分四个结论强度：

| 证据 | 可以证明 | 不能证明 |
|---|---|---|
| `startup_stm32f103rctx.s` | 仓库中存在 RCTx 启动文件源码 | 该启动文件已经进入本次链接 |
| `Debug/objects.list` | RCTx 启动对象进入 Debug 链接输入 | 启动代码中的每个符号都来自当前链接脚本 |
| `Debug/makefile` 的 `-T STM32F103RCTX_FLASH.ld` | 本次链接命令使用 RCTx 链接脚本 | 目标板已经运行该 ELF |
| `.list` 中 `_estack`、`_Min_Stack_Size`、`.data/.bss` 启动路径 | 启动代码与链接脚本符号在当前构建产物中形成闭环 | 实物 RAM 边界、下载镜像和运行栈水位已经现场验证 |

因此，第01章可以把 `.list` 作为“当前构建确实把平台、启动和链接脚本连接起来”的强证据；但不能把它写成“当前板卡已经按该栈边界安全运行”。后者需要下载记录、调试器寄存器/内存窗口、栈水位检查或运行日志支撑，缺少时保持【待验证】。

### 8.5.2 MCU容量与链接边界证据

平台容量也要分清“官方器件能力”“工程选择”和“当前构建结果”三层。

ST 官方 `STM32F103xC/D/E` 数据手册说明该高密度系列覆盖 256KB 到 512KB Flash、最高 64KB SRAM，并包含 LQFP64 封装选项。这个资料可以支撑“STM32F103RC/RD/RE 所在系列具备对应容量和封装组合”的器件层背景，但不能单独证明本仓库工程实际使用了哪一个链接边界。

本项目仓库内的证据更具体：

| 证据位置 | 能证明的内容 | 不能证明的内容 |
|---|---|---|
| `Three-axis_cloud_platformV2.ioc` 中 `Mcu.CPN=STM32F103RCT6`、`Mcu.Package=LQFP64`、`Mcu.UserName=STM32F103RCTx` | CubeMX 配置源把项目放在 STM32F103RCTx / LQFP64 平台上 | 板上芯片丝印、焊接封装和外部连线一定与配置一致 |
| `.cproject` 中 `target_mcu=STM32F103RCTx`、`STM32F103xE`、`STM32F103RCTX_FLASH.ld` | IDE 构建配置选择了目标 MCU、容量族宏和链接脚本 | 当前目标板已经下载并运行此 Debug ELF |
| `STM32F103RCTX_FLASH.ld` 中 `FLASH=0x08000000, 256K`、`RAM=0x20000000, 48K` | 链接器按 256KB Flash、48KB RAM 放置段和栈顶 | 实物芯片的 Flash/RAM 读数已经现场验证 |
| `Debug/Three-axis_cloud_platformV2.map` 中 `FLASH 0x08000000 0x00040000`、`RAM 0x20000000 0x0000c000`、`_estack=0x2000c000` | 当前 Debug 构建快照确实采用了上述链接内存区域 | `.map` 一定由最新源码重新生成，或目标板当前正在运行该镜像 |

因此，第01章可以把“本仓库当前 Debug 构建按 256KB Flash / 48KB RAM 的 STM32F103RCTx 链接边界生成”写成构建事实；但不能把它扩展成“实物板已经确认焊接 STM32F103RCT6 且容量读回正确”。后者需要第33章下载调试、芯片丝印照片、读保护/设备 ID 读取或调试器连接记录支撑，缺少证据时保持【待验证】。

如果后续要把“链接边界”推进到“实物容量读回”，证据入口也要分层记录。`stm32f103xe.h` 定义 `FLASHSIZE_BASE = 0x1FFFF7E0UL`，`stm32f1xx_ll_utils.h` 又把它封装为 `FLASHSIZE_BASE_ADDRESS`，并提供 `LL_GetFlashSize()` 读取 Flash size data register，返回值单位为 Kbytes。这能说明当前库和设备头文件提供了容量读数入口，但本仓库没有看到主线代码调用 `LL_GetFlashSize()`，也没有调试器读回记录。因此本章只能把它写成“后续验证方法”，不能写成实物容量已经验证通过【待验证】。

### 本节证据边界

本节只根据当前仓库说明文件、函数、宏、变量和调用关系。运行时频率、外部硬件表现、主机侧现象、传感器方向、电机响应或真实控制效果仍需调试记录、日志或仓库外实测证据；缺少证据时保持【待验证】。

## 9. 调试方法

本章阶段的调试，不是调功能，而是调平台一致性。

可观察的对象：

- `.ioc` 里芯片型号、封装和外设列表是否与工程配置一致；实际硬件是否完全一致需要硬件资料或仓库外实测另行证明【待验证】。
- `.cproject` 中的目标 MCU、宏定义和链接脚本是否一致。
- `STM32F103RCTX_FLASH.ld` 中的 RAM/FLASH 是否满足工程需要。
- `Debug/objects.list`、`Debug/sources.mk` 是否仍然对应当前工程结构。

常见异常：

- 芯片型号和封装不一致。
- 编译器宏与芯片头文件不一致。
- 链接脚本和实际 RAM/FLASH 不一致。
- CubeMX 配置和源码目录不一致。

调试记录建议：

- 记录 `.ioc`、`.cproject`、链接脚本和构建清单中的平台字段。
- 记录 `STM32F103xE -> stm32f103xe.h` 的宏分派路径。
- 记录启动文件对象是否进入 `Debug/objects.list`。
- 记录 `Debug/makefile` 链接命令是否仍使用 `STM32F103RCTX_FLASH.ld`。
- 若要验证实物 Flash 容量，记录调试器从 `FLASHSIZE_BASE` 或 `LL_GetFlashSize()` 读回的 Kbytes 值；没有读回记录时，只能证明工程链接边界，不能证明目标板实际容量。
- 每条记录都写明来源文件、字段值、结论和证据边界。
- 对板卡实物、供电、外部连线和器件丝印等仓库外信息，缺少实测时保持【待验证】。

建议按以下顺序排查平台不一致：

```text
.ioc平台字段
-> .cproject目标MCU和宏
-> stm32f1xx.h设备头文件分派
-> startup文件名和CPU声明
-> linker MEMORY区域
-> Debug构建清单和链接命令
-> 目标板实物与下载运行记录【待验证】
```

如果前面任意一层已经不一致，不要继续用后续章节的外设结论解释现象；先把平台证据链修正。

## 10. 常见问题

### 1. 为什么先讲平台，而不是先讲 GPIO 或 TIM？

因为 GPIO、TIM、I2C、USB 都依赖同一个目标芯片、封装、时钟和工程宏定义。平台没定，后续外设章节就没有共同前提，读者也无法判断某个引脚、寄存器名或链接脚本是否属于当前项目。

本章先把 `.ioc`、`.cproject`、链接脚本和设备头文件指向的 MCU 平台对齐。这样后续讲定时器、通信、USB 或调试接口时，都能回到同一条证据链，而不是凭经验猜测项目运行在哪个芯片上。

### 2. 为什么项目里同时出现 `.ioc`、`.cproject` 和 `.mxproject`？

因为它们分别承担不同职责：

- `.ioc` 定义配置源。
- `.cproject` 定义编译与链接目标。
- `.mxproject` 记录生成和工程元信息。

这三类文件都属于仓库内工程证据，但证据层级不同。
`.ioc` 不能替代实际源码，`.cproject` 不能替代 CubeMX 配置源。
`.mxproject` 也不能单独证明某个外设已经成为当前主线。
教材需要把它们交叉核对，而不是只挑一个文件下结论。

### 3. 为什么同一项目里会出现 `STM32F103RCT6`、`STM32F103RCTx`、`STM32F103R(C-D-E)Tx` 这些写法？

它们对应的是同一平台族内的不同层级表述：芯片型号、项目用户名和家族型名称。  
教材中正式名称统一使用 `STM32F103RCTx芯片平台`，避免混用。

这里的统一命名不是为了抹掉差异，而是为了让读者知道这些写法如何互相对应。涉及具体宏定义、设备头文件或链接脚本时，仍应回到文件中的原始写法；涉及教材知识点和章节标题时，统一使用正式知识点名称。

### 4. 为什么不能把 HAL 库存在当成项目一定使用了某个能力？

因为库存在只代表“仓库中有这部分文件”，不代表它被宏启用、参与编译、进入链接，更不代表业务代码调用了它。正式教材只能把项目中真实启用并有调用路径的能力写成主线。

如果只有库目录存在，没有配置、构建产物和源码调用证据，就只能写成“仓库存在”或“可作为后续扩展材料”。外部硬件、供电和连线也不能由库文件推出，缺少实测时保持【待验证】。

### 5. `STM32F103xE` 是否等于 `STM32F103RCTx`？

不等于。`STM32F103RCTx` 是当前工程面向的芯片平台名；`STM32F103xE` 是编译器使用的容量族宏。它们在当前工程中应该互相匹配，但不能混写成同一个概念。

教材写作时可以说：当前 `.cproject` 的目标 MCU 为 `STM32F103RCTx`，同时定义 `STM32F103xE` 宏，使 CMSIS 选择 `stm32f103xe.h`。这样既保留平台名，也解释了编译器实际看到的头文件路径。

### 6. `genericBoard` 是否说明硬件板卡不重要？

不是。`genericBoard` 只说明 CubeIDE 工程目标没有绑定某个特定评估板模板。它不能证明项目没有自定义 PCB，也不能证明外设连线、供电或晶振状态。

从教材证据角度看，`genericBoard` 反而提醒读者：工程配置能证明目标 MCU 和软件资源，真实板卡形态仍需仓库外硬件资料或实测记录支撑。

### 7. 为什么第01章要提启动文件和链接脚本？

因为平台不只影响外设列表，也影响程序从哪里启动、栈顶在哪里、`.data` 从哪里加载到哪里、`.bss` 如何清零。启动文件和链接脚本把“目标芯片”落到了复位入口和内存布局上。

如果只看 `.ioc`，读者会以为平台只是 CubeMX 界面中的选择项。加入启动文件和链接脚本后，读者才能看到平台如何进入真正的 ELF。

### 8. 如果换成同系列不同容量芯片，哪些结论最先失效？

最先需要复查的是 `STM32F103xE` 宏、设备头文件、启动文件、链接脚本 MEMORY 区域、`.cproject` 目标 MCU 和 `.ioc` 平台字段。外设源码看起来可能还能编译，但 RAM/FLASH 边界、向量表、可用引脚和部分外设资源都可能已经不再成立。

### 9. 设备头文件里的地址范围能否直接当作本项目可用容量？

不能直接这样用。设备头文件给出的是容量族和寄存器层面的定义，链接脚本才是当前 ELF 放置代码、数据、堆和栈时使用的容量边界。

在本项目中，`stm32f103xe.h` 说明编译进入了 `STM32F103xE` 设备分支；`STM32F103RCTX_FLASH.ld` 则明确把 FLASH 写成 256K、RAM 写成 48K。教材要以二者分工解释：头文件负责寄存器和地址符号，链接脚本负责当前工程的链接布局。

## 11. 实践任务

开始任务前，先回到本章第8节定位 `.ioc`、`.cproject`、链接脚本和构建产物证据；第9节提供平台边界核对顺序。

任务一：确认 CubeMX 平台字段。

在 `Three-axis_cloud_platformV2.ioc` 中找到 `Mcu.CPN`、`Mcu.Package`、`Mcu.UserName` 和 `Mcu.Family`。
验收依据是平台字段表至少包含字段名、字段值、来源文件和平台结论四列。

任务二：确认工程目标字段。

在 `.cproject` 中找到目标 MCU、`USE_HAL_DRIVER`、`STM32F103xE` 和 `STM32F103RCTX_FLASH.ld`。
验收依据是工程目标表至少包含宏定义、目标芯片、链接脚本和对应关系四项。

任务三：确认存储器边界。

在 `STM32F103RCTX_FLASH.ld` 中找到 FLASH/RAM 的起始地址和容量。
验收依据是存储器边界表包含区域名、起始地址、容量、来源文件和证据边界。

任务四：区分构建目录和链接对象。

在 `Debug/sources.mk` 和 `Debug/objects.list` 中各举一个“目录进入构建”和“对象进入链接”的证据。
验收依据是证据对照表分别列出 `sources.mk` 目录项、`objects.list` 对象项和二者结论。

任务五：整理平台证据边界。

说明上述证据为什么共同指向 `STM32F103RCTx芯片平台`，并记录哪些仓库外硬件实测信息仍然需要标记为【待验证】。
验收依据是结论只使用仓库内证据证明项目目标平台，不把板卡实物、供电或外设连线写成已验证事实。

任务六：追踪 CMSIS 宏分派。

从 `.cproject` 中的 `STM32F103xE` 出发，进入 `stm32f1xx.h`，找到包含 `stm32f103xe.h` 的条件分支。
验收依据是表格包含宏定义、分派文件、最终设备头文件和该头文件提供的至少三个平台定义，例如 `FLASH_BASE`、`SRAM_BASE`、`PERIPH_BASE`。

任务七：核对启动文件与链接脚本耦合。

在 `startup_stm32f103rctx.s` 中找到 `_sidata`、`_sdata`、`_edata`、`_sbss` 和 `_ebss` 的引用，再在 `STM32F103RCTX_FLASH.ld` 中找到对应符号或段边界。
验收依据是说明启动文件负责执行搬运/清零动作，链接脚本负责提供地址和边界；不能只写“启动文件会初始化内存”。

任务八：确认构建产物层平台证据。

在 `Debug/objects.list` 中确认 `startup_stm32f103rctx.o`，在 `Debug/makefile` 中确认 `-T STM32F103RCTX_FLASH.ld`。
验收依据是能区分“构建产物使用了启动对象和链接脚本”与“目标板已经运行该 ELF”这两层证据；后者仍需第33章下载调试记录。

## 12. 思考题

1. 如果把目标芯片换成同系列但不同 RAM/FLASH 的型号，最先要改哪些文件？
2. 如果链接脚本和 `.cproject` 中的目标配置不一致，会出现什么问题？
3. 当前仓库中哪些证据能证明目标 MCU，哪些证据只能说明工程配置意图？
4. 为什么教材不把“库文件存在”直接等同于“项目功能已启用”？
5. 为什么 `STM32F103xE` 宏会影响设备头文件，而不仅是一个注释性标签？
6. 为什么 LQFP64 封装信息会影响后续 GPIO、I2C、USB 和 PWM 章节？
7. 为什么 `genericBoard` 不能替代真实板卡资料？
8. 如果 `.ioc` 平台字段正确，但 `Debug/makefile` 链接了另一个 `.ld` 文件，应优先相信哪一层证据，为什么？
9. 为什么启动文件和链接脚本必须一起阅读？
10. 为什么设备头文件和链接脚本对内存边界的证明力不同？
11. 如果只知道仓库面向 STM32F103RCTx，哪些硬件事实仍必须保持【待验证】？

## 13. 本章总结

本章只做一件事：把三轴云台项目的目标平台钉死。  
本章已经确认的结论是：

- 项目目标芯片是 STM32F103RCTx。
- 工程使用 CubeMX + STM32CubeIDE + GCC。
- 平台边界由 `.ioc`、`.cproject`、`.mxproject` 和链接脚本共同定义。
- `STM32F103xE` 宏使 CMSIS 选择 `stm32f103xe.h`，从而决定寄存器、IRQn 和外设基地址定义。
- `startup_stm32f103rctx.s` 与 `STM32F103RCTX_FLASH.ld` 共同决定复位入口、段初始化和 RAM/FLASH 边界。
- `Debug/objects.list` 和 `Debug/makefile` 可以证明 Debug 构建使用了对应启动对象和链接脚本，但不能证明目标板已经运行该 ELF。
- `LQFP64` 和 `Mcu.PinsNb=29` 是工程配置中的封装与引脚分配证据，不是板卡实物连线证据。

本章边界：

- 本章只确认目标平台和工程边界，不判断任何外设功能是否已经进入业务主线。
- 仓库内证据可以证明工程面向 STM32F103RCTx；目标板实物状态仍需后续下载和调试章节验证。
- 芯片丝印、实际 FLASH/RAM 读数、外部晶振、供电质量、调试连线和外设接线仍属于仓库外证据。

下一章开始，才能正式进入工程结构与构建配置。

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

- `Three-axis_cloud_platformV2.ioc`
- `.cproject`
- `.mxproject`
- `STM32F103RCTX_FLASH.ld`
- `Core/Startup/startup_stm32f103rctx.s`
- `Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f1xx.h`
- `Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xe.h`
- `Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_ll_utils.h`
- `Debug/objects.list`
- `Debug/makefile`
- `Debug/Three-axis_cloud_platformV2.map`

官方资料：

- STMicroelectronics, `STM32F103xC, STM32F103xD, STM32F103xE datasheet`, DS5792。

质量自检：

- P0 事实错误：通过
- P1 依赖断层：通过
- P2 逻辑连贯：通过
- P3 项目证据：通过
- P4 原理展开：通过
- P5 调试实践：通过
- P6 表达统一：通过

---
> 导航：上一章：无 ｜ 下一章：[第02章_STM32CubeIDE工程结构](第02章_STM32CubeIDE工程结构.md)
