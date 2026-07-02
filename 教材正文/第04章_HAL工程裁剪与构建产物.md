# 第04章 HAL工程裁剪与构建产物

> 导航：上一章：[第03章_启动流程与内存布局](第03章_启动流程与内存布局.md) ｜ 下一章：[第05章_CMSIS与内核访问](第05章_CMSIS与内核访问.md)

## 1. 本章目标

- 明确本项目启用的 HAL 模块、参与构建的 HAL 源文件和最终对象文件之间的关系。
- 区分“库文件存在”“宏启用”和“实际参与构建”三类证据。
- 看懂 `Debug/sources.mk`、`Debug/makefile` 和 `Debug/objects.list` 在构建链路中的位置。
- 为后续外设初始化、中间件和调试章节建立工程依据。

本章阅读分层：

| 阅读层次 | 建议范围 | 适合读者 |
|---|---|---|
| 【必须掌握】 | 第1节到第5节，第13节总结 | 需要理解 HAL 模块宏、构建规则、对象文件和后续章节证据来源的读者 |
| 【工程深化】 | 第6节到第8节，第9节调试方法 | 需要维护 `stm32f1xx_hal_conf.h`、`sources.mk`、`subdir.mk`、`objects.list`、`makefile` 和 HAL 弱符号覆盖关系的读者 |
| 【拓展阅读】 | 第8.5节到第8.7节，第10节常见问题 | 需要进一步理解 `--gc-sections`、保留根、浮点格式化库、增量构建和产物新鲜度的读者 |
| 【证据与验证】 | 第8节、第9节、章节尾部固定检查，以及所有 `【待验证】` 项 | 需要审查 `stm32f1xx_hal_conf.h`、`sources.mk`、HAL目录 `subdir.mk`、`objects.list`、`makefile`、`.map/.list/.su/.cyclo`、构建日志、下载日志、栈水位和运行现场证据的读者 |

如果只是沿工程证据主线学习，可以先抓住“HAL 模块宏决定声明可见 -> `subdir.mk` 证明源文件被编译 -> `objects.list` 证明对象进入链接 -> `.map/.list` 证明 section 或调用路径进入 ELF -> 运行事实仍需现场证据”这条链；分析栈风险、链接裁剪或旧产物问题时，再回到资源分析和产物新鲜度小节。

本章速查：

| 查找目标 | 优先阅读 | 避免重复展开 |
|---|---|---|
| HAL 模块宏、构建规则和对象文件主证据链 | 第4节到第5节、第13节 | CubeIDE 工程结构回到第02章，链接脚本回到第03章 |
| `sources.mk`、`subdir.mk`、`objects.list` 和 `makefile` 分工 | 第6节到第8.5节、第9节 | 不把“库文件存在”或“宏启用”直接写成“最终进入 ELF” |
| `.map/.list/.su/.cyclo`、section 保留和栈风险边界 | 第8.6节到第8.6.2节、第10节 | CMSIS 访问继续到第05章，具体外设证据回到对应章节 |
| 增量构建、产物新鲜度和运行现场验证边界 | 第8.7节到第8.8节、章节尾部固定检查 | 构建产物不能替代下载日志、断点、栈水位或硬件运行记录 |

## 2. 前置知识

- STM32F103RCTx芯片平台
- STM32CubeIDE工程配置
- 链接脚本与内存布局

本章使用第02章的工程配置结论和第03章的链接脚本结论：CubeIDE 工程通过 ARM GCC、Makefile 片段、对象文件列表和链接脚本把源码组织成可运行镜像。

## 3. 问题背景

STM32CubeIDE 工程里通常会有很多驱动文件和头文件。对教材来说，不能看到某个库文件存在，就把它写成项目主线。真正可靠的判断需要至少回答三件事：

- 配置头文件是否启用了相关 HAL 模块。
- 构建规则是否把对应源文件加入编译。
- 对象文件列表是否显示对应 `.o` 参与最终链接。

三轴云台项目中，HAL 驱动承担外设初始化和底层访问的基础角色，但第04章不展开每个外设怎么工作。这里先建立工程层判断方法：哪些 HAL 源文件被纳入构建，构建产物在哪里，后续章节引用 HAL 行为时应回到哪些证据。

## 4. 核心概念

- HAL驱动模块裁剪：通过配置宏控制 HAL 头文件包含和驱动能力范围。
- STM32 HAL驱动库：ST 提供的外设抽象层源码和头文件，本项目位于 `Drivers/STM32F1xx_HAL_Driver`。
- Makefile构建产物：CubeIDE 生成的构建规则、对象文件列表、依赖文件、分析文件和输出镜像。
- 源文件：参与编译的 `.c` 文件。
- 对象文件：源文件编译后形成的 `.o` 文件，会被链接器组合成最终 ELF。
- 构建索引：`sources.mk`、各目录 `subdir.mk`、`objects.list` 共同形成的工程构建证据链。
- 节级裁剪：编译器把每个函数和数据放进独立 section，链接器再删除未引用 section。
- 弱符号与用户实现：HAL 库里的回调常以弱定义提供，项目源码中的同名强定义会在链接时替代它。
- 函数保留证据：对象文件进入链接后，还要通过 `map` 或 `list` 判断具体函数是否最终保留在 ELF 中。
- 调试产物：`map`、`list`、`.su`、`.cyclo` 等文件用于观察链接、反汇编、栈使用和复杂度。
- 单函数栈估算：`.su` 文件按函数记录编译器估算的栈使用字节数和限定标签。
- 调用链栈上界：把某条真实调用路径上的函数栈估算、异常入口现场和可能的中断嵌套分层组合后，才能讨论的系统级栈风险。
- 构建新鲜度：产物时间戳、构建日志和下载日志共同证明当前源码是否进入最新 ELF。

这些概念只服务第04章的三个正式知识点，不新增结构外知识点。

## 5. 工作原理

本项目的 HAL 工程裁剪和构建产物可以按五层理解。

第一层是配置层：

- `Core/Inc/stm32f1xx_hal_conf.h` 中的 `HAL_*_MODULE_ENABLED` 宏决定哪些 HAL 模块头文件会被条件包含。
- `USE_FULL_ASSERT` 当前保持注释状态，因此 HAL 参数检查宏不会展开为 `assert_failed()` 调用。
- 相关调度适配开关设置为 `0U`，说明 HAL 配置层未启用该适配路径。

第二层是构建规则层：

- `Debug/sources.mk` 列出所有包含源码的子目录。
- `Debug/makefile` 包含 `sources.mk`、各子目录 `subdir.mk` 和 `objects.mk`。
- `Debug/Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk` 明确列出参与编译的 HAL 源文件。

第三层是链接输入层：

- `Debug/objects.list` 列出最终链接使用的对象文件。
- `Debug/makefile` 调用 `arm-none-eabi-gcc`，通过 `@"objects.list"` 把对象文件交给链接器。
- `Debug/makefile` 同时生成 ELF、map 和反汇编列表等输出。

第四层是链接裁剪和产物观察层：

- 编译命令使用 `-ffunction-sections` 和 `-fdata-sections`。
- 链接命令使用 `-Wl,--gc-sections`。
- `Debug/Three-axis_cloud_platformV2.map` 可以观察被丢弃的输入 section 和最终内存映射。
- `Debug/Three-axis_cloud_platformV2.list` 可以观察段表和反汇编。
- `.su` 文件可以观察编译器估算的函数栈使用。

第五层是符号解析和用户回调层：

- HAL 库中的 `HAL_MspInit()` 是弱定义。
- 项目在 `Core/Src/stm32f1xx_hal_msp.c` 中提供了同名强定义。
- `HAL_Init()` 调用 `HAL_MspInit()` 时，最终进入项目实现，而不是 HAL 库里的空弱实现。
- `map` 中 HAL 库的弱 `HAL_MspInit()` 和项目实现的 `HAL_MspInit()` 不是同一个证据层。

因此，第04章判断项目“实际构建了什么”，不能只看 `Drivers/` 目录，
也不能只看 `stm32f1xx_hal_conf.h`，必须把配置、构建规则、对象文件列表、
链接选项、符号解析和产物观察合起来看。

还要区分两句话：

```text
对象文件进入 objects.list
函数或数据最终保留在 ELF
```

前者说明对象文件是链接输入；后者还要受 section 粒度、引用关系和
`--gc-sections` 影响。也就是说，某个 HAL `.o` 进入链接，不等于其中每个函数
都留在最终镜像，更不等于每个函数都在运行时被调用。

还要区分另外两句话：

```text
头文件声明对 C 编译可见
函数符号被最终调用
```

`main.h` 包含 `stm32f1xx_hal.h`，`stm32f1xx_hal.h` 再包含 `stm32f1xx_hal_conf.h`。
这条链路说明 HAL 声明如何进入编译单元，但函数是否进入 ELF 要看链接结果，
函数是否运行过还要看调用链、断点、日志或外设现象。

## 6. STM32实现机制

STM32 HAL 的工程实现由头文件配置和源文件编译共同完成。

本项目中可以确认的启用宏包括：

- `HAL_MODULE_ENABLED`
- `HAL_GPIO_MODULE_ENABLED`
- `HAL_I2C_MODULE_ENABLED`
- `HAL_PCD_MODULE_ENABLED`
- `HAL_TIM_MODULE_ENABLED`
- `HAL_UART_MODULE_ENABLED`
- `HAL_CORTEX_MODULE_ENABLED`
- `HAL_DMA_MODULE_ENABLED`
- `HAL_FLASH_MODULE_ENABLED`
- `HAL_EXTI_MODULE_ENABLED`
- `HAL_PWR_MODULE_ENABLED`
- `HAL_RCC_MODULE_ENABLED`

这些宏会影响 `stm32f1xx_hal_conf.h` 后续条件包含哪些 HAL 头文件。

这里要注意两个边界：

- `.cproject`、`.mxproject` 和 Debug 编译命令中的 `USE_HAL_DRIVER` 是工程级编译宏，用来启用 HAL 驱动路径；`stm32f1xx_hal_conf.h` 中的 `HAL_*_MODULE_ENABLED` 是模块级裁剪宏，用来决定具体 HAL 模块头文件是否被包含。
- 当前 `stm32f1xx_hal_conf.h` 中 `HAL_GPIO_MODULE_ENABLED` 出现两次，属于配置文本重复；它不能被解释为两个 GPIO 模块，也不能单独证明 GPIO 函数运行过。

同时，`Debug/Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk` 和 `Debug/objects.list` 确认以下 HAL 源文件已经形成对象文件并参与链接：

- `stm32f1xx_hal.c`
- `stm32f1xx_hal_cortex.c`
- `stm32f1xx_hal_dma.c`
- `stm32f1xx_hal_exti.c`
- `stm32f1xx_hal_flash.c`
- `stm32f1xx_hal_flash_ex.c`
- `stm32f1xx_hal_gpio.c`
- `stm32f1xx_hal_gpio_ex.c`
- `stm32f1xx_hal_i2c.c`
- `stm32f1xx_hal_pcd.c`
- `stm32f1xx_hal_pcd_ex.c`
- `stm32f1xx_hal_pwr.c`
- `stm32f1xx_hal_rcc.c`
- `stm32f1xx_hal_rcc_ex.c`
- `stm32f1xx_hal_tim.c`
- `stm32f1xx_hal_tim_ex.c`
- `stm32f1xx_hal_uart.c`
- `stm32f1xx_ll_usb.c`

当前 `Drivers/STM32F1xx_HAL_Driver/Src` 目录下共有 18 个 `.c` 文件，`objects.list` 中对应 18 个 HAL/LL 对象文件。本章可以据此说“当前 Debug 构建把该目录下的 HAL/LL 源文件全部列入对象清单”。但这仍然只是对象层结论；具体函数是否保留，要继续看 `map/list`。

这些文件属于 HAL/LL 驱动层。其中 `stm32f1xx_ll_usb.c` 是 USB 底层驱动文件，和 PCD/USB Device 路径相关；它的存在不表示本项目整体改用 LL 风格开发。教材后续讨论 GPIO、TIM、I2C、UART 时，仍要分别看对应源码调用的是 HAL API、LL API 还是直接寄存器宏。

构建选项还体现了 Cortex-M3 工程约束。当前 Debug 构建使用：

- `-mcpu=cortex-m3`
- `-mthumb`
- `-mfloat-abi=soft`
- `-O0`
- `-g3`
- `--specs=nano.specs`
- `--specs=nosys.specs`
- `-u _printf_float`
- `-u _scanf_float`

这些选项说明当前工程以 Cortex-M3 Thumb 指令集、软浮点 ABI 和 Debug 可观察性为主。
`nano.specs` 和 `nosys.specs` 影响 C 运行库与系统调用适配，
`-u _printf_float`、`-u _scanf_float` 会强制拉入浮点格式化相关符号。
它们可能影响代码体积和链接结果；具体影响必须通过 map、size 或 list 产物验证，
不能只凭选项名称定量判断。

从 `HAL_Init()` 的实际代码看，HAL 初始化还会执行一条基础运行链：

```text
HAL_Init()
-> 根据 PREFETCH_ENABLE 启用 Flash prefetch
-> HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4)
-> HAL_InitTick(TICK_INT_PRIORITY)
-> HAL_MspInit()
```

这条链路说明 HAL 工程裁剪不是“把库文件放进工程”这么简单。配置宏、C 源文件、弱符号覆盖、SysTick 时基和 MSP 全局初始化共同决定 HAL 进入应用层之前的最低运行环境。

## 7. 项目中的应用

本章对应项目中的工程构建层。

直接相关文件：

- `Core/Inc/stm32f1xx_hal_conf.h`
- `.mxproject`
- `Drivers/STM32F1xx_HAL_Driver`
- `Debug/sources.mk`
- `Debug/makefile`
- `Debug/Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk`
- `Debug/objects.list`
- `Debug/Three-axis_cloud_platformV2.map`
- `Debug/Three-axis_cloud_platformV2.list`

它们之间的关系是：

- `stm32f1xx_hal_conf.h` 说明 HAL 模块选择。
- `.mxproject` 记录 CubeMX/CubeIDE 生成和曾使用的 HAL、CMSIS、中间件相关文件。
- `Drivers/STM32F1xx_HAL_Driver` 提供 HAL/LL 源码和头文件。
- `sources.mk` 说明哪些目录进入构建扫描。
- HAL 目录下的 `subdir.mk` 说明哪些 HAL 源文件被编译。
- `objects.list` 说明哪些对象文件进入最终链接。
- `makefile` 把对象文件、链接脚本和库选项组合成 ELF、map 和 list 输出。
- `map` 进一步显示哪些 section 被保留或丢弃。
- `list` 进一步显示最终段表和反汇编。
- `.su` 文件进一步显示每个编译单元的栈使用估算。

在项目主线中，第04章仍不是控制算法或外设应用章节，而是后续外设章节的工程证据底座。

例如 `main.c` 调用 `HAL_Init()`，`HAL_Init()` 再调用项目实现的 `HAL_MspInit()`；`main.c` 后续还调用 `HAL_SYSTICK_Config()` 和 `HAL_NVIC_SetPriority()` 重新配置 SysTick。这些调用说明 HAL 已经进入项目启动路径，但第04章仍只记录工程链路和入口关系，不提前分析 SysTick、NVIC 或具体外设行为。

## 8. 代码分析

### 8.1 `stm32f1xx_hal_conf.h`

该文件的核心作用是 HAL 模块选择。

被定义的 `HAL_*_MODULE_ENABLED` 宏代表对应 HAL 模块在配置层被启用；
被注释的模块不能仅凭库文件存在写成项目主线。
文件后半部分通过 `#ifdef HAL_*_MODULE_ENABLED` 条件包含对应头文件，
这说明“宏启用”会影响编译期可见的 HAL 声明。

完整头文件入口链路是：

```text
Core/Inc/main.h
-> Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal.h
-> Core/Inc/stm32f1xx_hal_conf.h
-> #ifdef HAL_*_MODULE_ENABLED
-> include 对应 stm32f1xx_hal_xxx.h
```

这条链路只能证明声明可见性。若某个模块宏被启用但对应源文件没有进入 `subdir.mk`，编译可能能看到声明，却在链接阶段缺少实现；若源文件进入对象列表但函数没有引用，`--gc-sections` 仍可能把具体函数丢弃。

本项目还设置：

- `HSE_VALUE = 8000000U`
- `TICK_INT_PRIORITY = 0U`
- 调度适配开关为 `0U`
- `PREFETCH_ENABLE = 1U`

这些配置为后续时钟、节拍和系统配置章节提供证据，但本章只记录它们的工程位置。

当前 `USE_FULL_ASSERT` 保持注释状态，所以 `assert_param(expr)` 展开为空操作。这会降低运行期参数检查开销，但也意味着参数错误不会通过 `assert_failed()` 暴露。是否适合在调试阶段打开，需要结合调试便利性、代码体积和实时性继续权衡。

### 8.2 `Debug/sources.mk`

`sources.mk` 是自动生成文件，文件头明确提示不要手工编辑。它列出的 `SUBDIRS` 包含：

- `Core/Src`
- `Core/Startup`
- `Drivers/CustomDrivers/Src`
- `Drivers/SRC/Src`
- `Drivers/STM32F1xx_HAL_Driver/Src`
- 中间件与 USB Device 目录

这说明构建系统把 HAL 驱动目录和项目业务目录一起纳入构建扫描。

### 8.3 HAL 目录 `subdir.mk`

HAL 目录下的 `subdir.mk` 明确列出 `C_SRCS`、`OBJS` 和 `C_DEPS`。这比单纯看 `Drivers/STM32F1xx_HAL_Driver/Src` 目录更强，因为它证明这些源文件已经进入当前 Debug 构建规则。

当前 HAL 驱动目录下 18 个 `.c` 与 `subdir.mk`、`objects.list` 中的 18 个对象文件一一对应。这个计数结论只适用于当前 Debug 构建；如果切换 Release 配置、重新生成工程或手工改动构建文件，需要重新统计。

同一文件中的编译命令包含：

- `-mcpu=cortex-m3`
- `-DDEBUG`
- `-DUSE_HAL_DRIVER`
- `-DSTM32F103xE`
- 多个 `-I` 头文件路径
- `-ffunction-sections`
- `-fdata-sections`
- `-fstack-usage`
- `-fcyclomatic-complexity`

这些选项说明构建产物不仅包含对象文件，还会生成依赖、栈使用和复杂度分析相关文件。

### 8.4 `Debug/objects.list`

`objects.list` 是最终链接输入的证据。它包含 Core、启动文件、自定义驱动、项目算法、HAL、中间件和 USB Device 对象文件。

对第04章来说，最关键的是它列出了 HAL 驱动对象文件。只有出现在这里的对象文件，才可以说已经参与当前 Debug 链接。

同时要把 `Core/Src/stm32f1xx_hal_msp.o` 单独看待。它不是 HAL 库目录里的对象，而是用户工程对 MSP 初始化回调的实现。它进入 `objects.list` 后，会和 HAL 库中的弱 `HAL_MspInit()` 发生符号解析关系，最终保留项目实现。

### 8.5 `Debug/makefile`

`Debug/makefile` 通过 `-include sources.mk` 和各目录 `subdir.mk` 汇总构建规则。它的链接命令使用 `@"objects.list"`，并指定第03章分析过的链接脚本。

它还声明构建输出包括：

- `Three-axis_cloud_platformV2.elf`
- `Three-axis_cloud_platformV2.map`
- `Three-axis_cloud_platformV2.list`

这说明第04章的构建产物不仅能证明“编了哪些文件”，也能为后续大小、符号和反汇编检查提供入口。

链接命令中还有几组重要选项：

- `-T"...STM32F103RCTX_FLASH.ld"`：指定链接脚本。
- `-Wl,-Map="Three-axis_cloud_platformV2.map"`：生成 map 文件。
- `-Wl,--gc-sections`：丢弃未引用 section。
- `--specs=nosys.specs`、`--specs=nano.specs`：选择嵌入式 C 库适配。
- `-u _printf_float`、`-u _scanf_float`：强制保留浮点 printf/scanf 支持入口。

这些选项决定“对象文件进入链接以后，哪些符号和 section 最终进入 ELF”。
如果只看 `objects.list`，会漏掉链接器后续裁剪和库符号拉入过程。

这里要再拆出“保留根”的概念。`--gc-sections` 不是随意删除函数，而是从一组根 section / 根符号出发，沿重定位引用保留可达 section。当前项目至少有三类保留根：第一，链接脚本中的 `KEEP(*(.isr_vector))`、`KEEP(*(.init))` 和 `KEEP(*(.init_array*))` 这类规则会要求关键启动段或运行时初始化段保留；第二，链接命令中的 `-u _printf_float`、`-u _scanf_float` 会把浮点格式化相关符号当成未解析入口，促使 C 库拉入对应实现；第三，普通源码调用关系会让 `main -> HAL_Init -> HAL_MspInit`、`main -> computeMotorCommands` 等路径保留到最终 ELF。

| 保留来源 | 当前证据 | 能证明什么 | 不能证明什么 |
|---|---|---|---|
| 链接脚本 `KEEP` | `STM32F103RCTX_FLASH.ld` 保留 `.isr_vector`、`.init`、`.fini`、`.preinit_array`、`.init_array`、`.fini_array` | 这些段即使不表现为普通 C 调用，也应进入链接结果 | 目标板已经按这些入口运行【待验证】 |
| 链接命令 `-u` | `Debug/makefile` 中 `-u _printf_float`、`-u _scanf_float`，`.map/.list` 中存在对应符号 | 浮点 printf/scanf 支持被强制纳入当前 Debug 构建 | 每个格式化路径都在运行时执行过 |
| 调用引用 | `.list` 中可见 `main()` 对 HAL、配置和控制入口的分支调用 | 被调用路径相关 section 可由引用关系保留 | 调用频率、耗时和硬件效果；仍需运行证据【待验证】 |

### 8.6 `map`、`list` 和分析产物

当前 `Debug/Three-axis_cloud_platformV2.map` 中可以看到 `Discarded input sections`。
这说明链接器确实记录了被丢弃的输入 section，适合用于分析“对象文件进入链接，
但其中某些函数或数据未进入最终镜像”的情况。

当前 map 给出了很好的对照例子：

- `stm32f1xx_hal.o` 进入 `objects.list`。
- `HAL_Init` 保留在最终 `.text` 中，地址为 `0x08005844`。
- `HAL_Delay` 也保留在最终 `.text` 中，地址为 `0x08005908`，因为项目和库代码存在调用路径。
- HAL 库里的弱 `HAL_MspInit` 位于 `Discarded input sections`。
- 项目 `Core/Src/stm32f1xx_hal_msp.c` 中的强 `HAL_MspInit` 保留在最终 `.text` 中，地址为 `0x08001a58`。

这组证据展示了三件事：对象文件进入链接、函数被保留、弱符号被项目实现替换，是三层不同结论。

当前 `Debug/Three-axis_cloud_platformV2.list` 开头包含 section 表，
例如 `.isr_vector`、`.text` 和 `.rodata`，后续还包含 `main` 的反汇编。
这使它适合回答两个问题：

- 某个段是否落在预期地址区间。
- 某个函数是否在最终 ELF 中有反汇编证据。

例如 list 中 `main` 调用 `HAL_Init` 的反汇编为 `bl 8005844 <HAL_Init>`；`HAL_Init` 内部又调用 `HAL_NVIC_SetPriorityGrouping`、`HAL_InitTick` 和 `HAL_MspInit`。这比单纯在 C 源码中搜索函数名更强，因为它证明当前 ELF 中存在对应调用指令。

编译命令还包含 `-fstack-usage` 和 `-fcyclomatic-complexity`。
当前 Debug 目录下能看到 46 个 `.su` 文件和 46 个 `.cyclo` 文件，说明栈使用估算和圈复杂度分析文件已经生成。
这些文件可以作为后续性能和可靠性分析入口，但它们不是运行时最大栈深度证据；
真实栈余量仍需要运行时水位或调试记录验证【待验证】。

`.su` 文件还需要继续拆读。

GNU 的[静态栈使用分析](https://gcc.gnu.org/onlinedocs/gnat_ugn/Static-Stack-Usage-Analysis.html)说明，`.su` 行组织为三类信息：

- 源位置或函数名。
- 字节数。
- 限定标签。

本项目的 `.su` 行也符合这个结构。例如：

- `main` 为 `56 static`。
- `SystemClock_Config` 为 `96 static`。
- `computeMotorCommands` 为 `120 static`。
- `SysTick_Handler` 为 `16 static`。

当前 46 个 `.su` 文件中，单函数估算较高的条目包括：

- `USB_EPStartXfer`：`272` 字节。
- `HAL_UART_IRQHandler`：`240` 字节。
- `computeMotorCommands`：`120` 字节。

它们回答的是“单个函数帧估算有多大”，不是“系统任意时刻最大栈深度是多少”。

把 `.su` 变成工程判断时，至少要再加三层证据：

- 调用链：如果 A 调用 B，再调用 C，不能只取 B 或 C 的单项最大值，而要沿实际调用路径分析当前仍在栈上的帧。
- 异常和中断：主循环执行到某处时可能被 SysTick、TIM 或 USB 中断打断，ISR 栈帧会叠加在被打断上下文之上。
- 构建配置：当前 `.su` 来自 `-O0`、`-g3`、`-fstack-usage` 的 Debug 构建，切换优化等级、内联策略或库选项后，估算值可能变化。

因此第04章只能把 `.su` 定位为静态入口证据，不能把最大 `.su` 数字直接写成 MSP 最大占用。

这也解释了第03章留下的运行期边界：链接脚本和 `_sbrk()` 能约束内存布局与堆增长，`.su` 能提示单函数栈压力，但真正要证明“剩余栈足够”，仍需要把静态估算、调用路径、中断嵌套和运行水位放在同一张证据表中。

#### 8.6.1 资源分析产物的证据边界

`.su`、`.cyclo`、`.map` 和 `.list` 都属于构建产物，但它们回答的问题不同，不能互相替代。

`.su` 回答“单个函数在当前编译配置下的栈帧估算是多少”。当前 Debug 构建中，`main` 为 `56 static`，`SystemClock_Config` 为 `96 static`，`updatePID` 为 `64 static`，`computeMotorCommands` 为 `120 static`。这些数字能提示哪些函数值得进入调用链栈分析，但不能证明 500Hz 主循环运行时的 MSP 峰值，也不能证明中断嵌套后的总栈深度安全。

`.cyclo` 回答“单个函数的圈复杂度是多少”。当前构建中，`main` 为 `7`，`updatePID` 为 `34`，`computeMotorCommands` 为 `48`。这能提示哪些函数分支较多、测试路径较复杂，适合在后续控制算法章节重点拆解；但它不是执行时间、CPU 周期、实时性或可维护性的直接数值。一个复杂度较低的函数也可能因为外设等待或库函数调用变慢，一个复杂度较高的函数也可能在当前参数下只走少量分支。

`.map` 回答“哪些符号和 section 进入或离开最终链接结果”。例如它能证明 `computeMotorCommands`、`updatePID`、HAL 强弱符号解析和 `.bss` 对象布局是否进入当前 ELF。它不能证明这些函数在运行期被调用了多少次，也不能证明分支覆盖率。

`.list` 回答“当前 ELF 中是否存在某段反汇编和源码对应关系”。例如它能证明 `main` 中存在 `bl 80043b4 <computeMotorCommands>`，也能证明 `HAL_Init()` 相关调用路径出现在当前 Debug ELF 中。它不能证明该路径在目标板上按预期频率执行，也不能证明循环耗时满足 500Hz 控制周期。

因此，本章可以把资源分析产物写成“静态风险筛查入口”：`.su` 指向栈风险，`.cyclo` 指向分支复杂度，`.map` 指向链接保留，`.list` 指向指令级路径。运行时性能结论仍需要 DWT 计数、GPIO 翻转测量、日志时间戳或调试器采样；缺少这些现场证据时保持【待验证】。

#### 8.6.2 构建证据断点

第04章最容易被误读的地方，是把一层证据直接外推到下一层。当前项目可以把 HAL 构建证据拆成五个断点：

| 证据层 | 当前项目例子 | 能证明什么 | 不能证明什么 |
| --- | --- | --- | --- |
| 宏和头文件可见 | `stm32f1xx_hal_conf.h` 中启用 `HAL_GPIO_MODULE_ENABLED`、`HAL_I2C_MODULE_ENABLED`、`HAL_TIM_MODULE_ENABLED`、`HAL_UART_MODULE_ENABLED`、`HAL_CORTEX_MODULE_ENABLED`、`HAL_RCC_MODULE_ENABLED` | 对应 HAL 头文件声明在编译期可见。 | 对应 `.c` 一定参与构建，或函数一定进入 ELF。 |
| 编译规则 | HAL 目录 `subdir.mk` 列出 `stm32f1xx_hal_gpio.o`、`stm32f1xx_hal_i2c.o`、`stm32f1xx_hal_tim.o`、`stm32f1xx_hal_uart.o`、`stm32f1xx_hal_rcc.o`、`stm32f1xx_hal_cortex.o` | 对应源文件被当前 Debug 规则编译为对象。 | 对象内所有函数都会被最终保留。 |
| 链接输入 | `Debug/objects.list` 包含上述 HAL 对象，`Debug/makefile` 通过 `@"objects.list"` 交给链接器。 | 对象文件进入链接输入集合。 | `--gc-sections` 后每个 section 都留在 ELF。 |
| 链接与指令结果 | `.map` 给出 `HAL_MspInit`、`HAL_InitTick`、`HAL_NVIC_SetPriorityGrouping`、`HAL_SYSTICK_Config` 等最终地址；`.list` 中能看到 `HAL_Init()` 调用这些函数的 `bl` 指令。 | 这些函数或调用路径进入当前 Debug ELF。 | 目标板已经下载这份 ELF，或这些路径按预期频率运行。 |
| 运行现场 | 需要下载日志、断点命中、DWT 计数、GPIO 翻转或串口时间戳。 | 才能证明真实执行、耗时、频率和外设响应。 | 仅凭仓库构建产物无法替代这一层。 |

因此，教材写 HAL 裁剪时应按“宏可见 -> 源文件编译 -> 对象链接 -> section/指令保留 -> 运行现场”逐层推进。任何一层缺证，都只能停在上一层结论；例如 `objects.list` 中有 `stm32f1xx_hal_tim.o`，可以证明 TIM HAL 对象进入链接输入，但要证明某个 TIM 函数最终保留，应继续查 `.map/.list`；要证明它驱动了真实定时器，还必须回到寄存器、调试器或硬件测量证据【待验证】。

### 8.7 增量构建和产物新鲜度

`Debug` 目录是构建产物目录，不是源码真相本身。当前仓库可以证明：

- `makefile` 如何组织构建。
- `objects.list` 当前列出哪些对象。
- `elf/map/list` 当前存在。

但这三点不能单独证明“刚修改的源码已经重新编进 ELF”。要证明产物新鲜，
至少需要以下证据：

```text
源码修改时间
-> 对应 .o 时间更新
-> ELF 时间更新
-> 构建日志成功
-> 下载日志或调试会话使用该 ELF
```

因此，第04章与第33章要保持同一边界：构建产物存在不等于板上运行最新源码。
缺少构建日志、ELF 时间戳和下载调试记录时，当前运行固件的新鲜度保持【待验证】。

当前本地证据显示 `Three-axis_cloud_platformV2.elf`、`.map`、`.list`、`objects.list` 和 `makefile` 时间戳一致，均为同一次 Debug 构建产物；`stm32f1xx_hal_conf.h` 的时间戳早于这些产物。这可以支持“该配置文件在这次构建前已经存在”的弱结论，但仍不能证明目标板正在运行这份 ELF。

### 8.8 本节证据边界

本节只根据当前仓库说明文件、函数、宏、变量和调用关系。运行时频率、外部硬件表现、主机侧现象、传感器方向、电机响应或真实控制效果仍需调试记录、日志或仓库外实测证据；缺少证据时保持【待验证】。

## 9. 调试方法

本节按“现象 -> 可能原因 -> 定位方法 -> 验证步骤 -> 解决方案 -> 经验总结”组织。第04章调试的目标是确认 HAL 裁剪配置、构建清单、链接产物和静态栈估算之间是否一致。

### 9.1 现象与可能原因

- 源码里能看到 HAL 文件，但链接时没有对应对象文件：检查 HAL 目录 `subdir.mk` 和 `objects.list`。
- 宏已经启用但编译找不到声明：检查 `stm32f1xx_hal_conf.h` 的条件包含和 `.cproject` Include Path。
- 某个驱动源文件存在但没有参与链接：以 `objects.list` 为准，不以库目录存在为准。
- 改动配置后构建产物未变化：检查是否重新生成或重新构建了 Debug 目录。
- 链接阶段找不到符号：检查对象文件是否进入 `objects.list`，再检查对应源文件是否进入 `subdir.mk`。
- 对象文件进入链接但函数不在 map/list 中：检查 `-ffunction-sections`、`-fdata-sections` 和 `--gc-sections`。
- ELF 变大但业务代码未明显增加：检查 C 库选项、浮点 printf/scanf、map 中拉入的库符号。
- `.su` 显示某函数栈使用偏大：先记录编译器估算，再结合运行时栈水位验证【待验证】。
- `.su` 最大单项看起来不大但系统仍疑似栈溢出：检查调用链叠加、中断嵌套、异常现场和库函数路径，不要只按最大单函数数值判断。
- HAL 全局初始化没有生效：检查 `HAL_Init()` 是否被 `main()` 调用、`HAL_MspInit()` 是否被项目强定义替代、`HAL_MspInit()` 中 AFIO/PWR 时钟和重映射代码是否保留。

### 9.2 定位方法：可观察对象

- `stm32f1xx_hal_conf.h` 中相关 `HAL_*_MODULE_ENABLED` 是否按项目需要启用。
- `Debug/sources.mk` 是否包含 `Drivers/STM32F1xx_HAL_Driver/Src`。
- `Debug/Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk` 是否列出对应 `C_SRCS` 和 `OBJS`。
- `Debug/objects.list` 是否包含对应 HAL 对象文件。
- `Debug/makefile` 的链接命令是否使用 `objects.list` 和 `STM32F103RCTX_FLASH.ld`。
- `Debug/Three-axis_cloud_platformV2.map` 和 `Debug/Three-axis_cloud_platformV2.list` 是否存在。
- `map` 中是否出现 `Discarded input sections`，用于判断链接裁剪证据。
- `list` 中是否出现 section 表和目标函数反汇编，用于判断最终 ELF 观察证据。
- `.su` 文件是否生成，用于判断栈使用分析产物是否存在。
- `.su` 行中是否能记录函数名、估算字节数和限定标签；较大条目是否被放回实际调用链中解释。
- HAL 库弱 `HAL_MspInit` 是否被丢弃，项目 `HAL_MspInit` 是否保留在最终 `.text`。
- `main` 是否在 list 反汇编中调用 `HAL_Init`，`HAL_Init` 是否继续调用 `HAL_MspInit`。

### 9.3 验证步骤：调试记录

- 记录宏启用、源文件进入 `subdir.mk`、对象进入 `objects.list` 三层证据。
- 对链接问题，先记录缺失符号、对象文件、源文件和构建清单位置。
- 对库目录中存在但未链接的文件，结论栏写成“仓库存在”，不要写成“当前固件使用”。
- 对产物新鲜度，记录源码修改时间、对象文件时间、ELF 时间、构建日志和下载日志。
- 对链接裁剪，记录对象文件是否进入 `objects.list`，以及函数是否出现在 `map/list`。
- 对弱符号覆盖，记录 HAL 库弱定义、项目强定义、map 中被丢弃的弱 section 和最终保留的强符号地址。
- 对 `.su` 分析，记录函数名、估算字节数、限定标签、所属构建配置、可能调用者和是否可能处于 ISR 路径；系统级栈结论保持到运行水位验证之后。

### 9.4 解决方案：按证据层级排查

先看配置源头，再看 `subdir.mk/objects.list`，最后看 `.map/.list/.su`。库目录中存在某个文件，只能证明仓库包含它；对象进入链接和函数保留到最终镜像，需要由构建清单和链接产物继续证明。

### 9.5 解决方案：区分静态估算与运行结果

`.su` 是编译期静态栈估算入口，适合发现需要重点关注的函数；系统最大栈深度、异常现场和中断嵌套仍要靠运行水位、断点或日志验证，缺少证据时保持【待验证】。

### 9.6 经验总结：调试边界

当前仓库能证明 HAL 裁剪宏、构建清单、对象文件和 `.su` 静态栈估算之间的关系。某次本机构建是否成功、是否使用旧产物或是否发生增量构建缓存问题，需要构建日志证明；运行期最大栈深度还需要水位或调试记录证明。缺少证据时保持【待验证】。

## 10. 常见问题

### 1. 为什么不能只看 `Drivers/STM32F1xx_HAL_Driver` 目录？

因为目录存在只表示工程中有这些库文件。是否参与当前构建，要看 `subdir.mk`、编译规则和 `objects.list`。

教材判断“当前固件使用了什么”时，必须至少区分三层：仓库存在、配置启用、对象进入链接。只看目录会把备用文件、未启用模块和真实主线混在一起。

### 2. 为什么宏启用还不等于对象文件已经链接？

宏启用主要影响头文件包含和条件编译路径；对象文件是否参与链接，需要构建规则和对象列表证明。

反过来也要注意：对象文件进入链接，只能说明它是固件的一部分，不等于每个函数都在业务路径中运行。后续章节还要继续追踪初始化调用、回调入口和主循环调用，才能判断它是否成为项目主线。

### 3. 为什么第04章不展开每个 HAL 模块内部 API？

因为本章的任务是建立工程证据。每个外设的初始化、MSP、时钟和引脚关系会在后续对应章节展开。

如果在第04章提前讲 API 细节，读者会在没有时钟、GPIO、NVIC 和外设上下文的情况下接收大量函数名，反而不利于理解。第04章只回答“哪些 HAL 模块进入了工程证据链”。

### 4. `objects.list` 和 `sources.mk` 哪个更接近最终链接事实？

`objects.list` 更接近最终链接事实，因为 `makefile` 链接命令直接通过它传入对象文件。`sources.mk` 更适合确认源码目录是否被构建系统扫描。

二者最好一起看。`sources.mk` 帮助定位源码目录进入构建系统，`objects.list` 帮助确认最终链接对象。若二者与源码目录不一致，应优先记录差异，再结合 make 输出判断当前构建到底采用了哪些文件。

### 5. map 和 list 文件有什么作用？

map 文件用于观察链接后的符号、段和内存分布；list 文件用于观察反汇编和源码混合输出。本章只确认它们由构建系统生成，具体分析留到后续调试和优化场景。

它们属于构建后的观察证据，不是源码配置源。用它们可以验证符号是否存在、函数是否被链接、段是否落在预期区域；但若要解释某个业务行为是否发生，仍需要回到源码调用链和运行调试记录。

### 6. 为什么对象文件进入 `objects.list`，还不能证明其中所有函数都在 ELF 中？

因为当前构建命令使用 `-ffunction-sections`、`-fdata-sections` 和 `-Wl,--gc-sections`。
对象文件是链接输入，函数和数据 section 才是链接器裁剪的细粒度对象。

因此，某个 `.o` 进入 `objects.list` 只能证明该编译单元参与链接。
若要证明某个函数最终保留，应继续查看 `map` 或 `list`。
若要证明函数运行过，还要看断点、调用链、变量或日志。

### 7. 为什么启用浮点 printf/scanf 要进入工程证据链？

当前 `.cproject` 中浮点 printf/scanf 选项为 true，`Debug/makefile` 链接命令中也出现
`-u _printf_float` 和 `-u _scanf_float`。
这会影响 C 库符号保留和最终 ELF 内容。

教材不能只写“项目使用 newlib-nano”就结束，而要提醒读者：
库规格、系统调用适配和浮点格式化支持都会改变链接结果。
实际体积影响应通过 `map`、`size` 或 ELF 产物继续验证。

### 8. Debug 目录里的 ELF 存在，是否证明最新源码已经构建成功？

不能单独证明。ELF 存在只能说明某次构建产生过产物。
要证明它来自最新源码，需要构建日志、对象文件更新时间、ELF 更新时间和必要时的 clean build 记录。
要证明它已经进入目标板，还需要第33章的下载和调试证据。

### 9. `.su` 文件里的最大值是否等于系统最大栈深度？

不等于。`.su` 文件中的一个条目对应一个函数的栈使用估算，例如当前构建中 `USB_EPStartXfer` 为 `272` 字节，`HAL_UART_IRQHandler` 为 `240` 字节，`computeMotorCommands` 为 `120` 字节。这个最大值只能说明“单函数条目里谁最大”，不能说明系统运行时只会占用这么多栈。

系统最大栈深度要看实际调用路径上仍未返回的函数帧，还要叠加异常入口、ISR 嵌套和库函数路径。第04章可以把 `.su` 作为静态筛查入口，不能把它当作运行期水位证据。

### 10. `HAL_GPIO_MODULE_ENABLED` 出现两次，是否表示 GPIO 被启用了两遍？

不是。相同宏重复定义只能说明配置文本里有重复项，不能推导出两个 GPIO 模块、两份对象文件或两次运行初始化。

工程判断仍要回到证据层：头文件条件包含说明声明可见，`subdir.mk` 说明 `stm32f1xx_hal_gpio.c` 进入编译规则，`objects.list` 说明 `stm32f1xx_hal_gpio.o` 进入最终链接，`map/list` 才能继续判断具体 GPIO 函数是否保留。

## 11. 实践任务

开始任务前，先回到本章第8节定位 HAL 裁剪宏、构建清单和链接对象证据；第9节提供“源码存在”和“参与构建”的核对顺序。

任务一：确认 HAL 模块裁剪宏。

在 `stm32f1xx_hal_conf.h` 中找出当前启用的 HAL 模块宏。
验收依据是 HAL 裁剪表包含模块宏、启用状态、来源文件和可编译结论。

任务二：确认 HAL 目录进入构建列表。

在 `Debug/sources.mk` 中确认 HAL 驱动目录是否进入构建目录列表。
验收依据是构建目录表包含目录项、对应清单文件和“目录不等于最终链接”的边界结论。

任务三：确认 HAL 源文件生成对象。

在 HAL 目录 `subdir.mk` 中找出 HAL 源文件、对象文件和依赖文件列表。
验收依据是生成物对照表包含一个 HAL 源文件、对象文件、依赖文件和清单位置。

任务四：确认对象进入最终链接。

在 `Debug/objects.list` 中确认 HAL 对象文件是否进入最终链接。
验收依据是最终链接记录包含至少一个 HAL 对象文件、所在行和链接结论。

任务五：区分库存在和参与链接。

对比 `Drivers/STM32F1xx_HAL_Driver/Src` 与 `objects.list`，说明“库存在”和“参与链接”的区别。
验收依据是边界表分列仓库中存在的 HAL 文件、进入链接的 HAL 对象和不可推出结论。

任务六：检查链接裁剪证据。

在 `Debug/makefile` 中找到 `-ffunction-sections`、`-fdata-sections` 和
`-Wl,--gc-sections`。再到 `Debug/Three-axis_cloud_platformV2.map` 中找到
`Discarded input sections`。
验收依据是能解释“对象文件进入链接”和“函数最终保留在 ELF”不是同一层证据。

任务七：检查 list 反汇编证据。

在 `Debug/Three-axis_cloud_platformV2.list` 中找到 section 表和 `main` 反汇编。
验收依据是能说明 list 文件适合验证段布局和函数是否进入最终 ELF，
但不能单独证明函数在运行时已经执行。

任务八：检查构建产物新鲜度。

记录一个源文件、对应 `.o`、最终 ELF、map 和 list 的时间戳。
验收依据是表格能区分“产物存在”“产物更新”和“产物已下载运行”三类结论；
缺少构建日志或下载日志时，最新固件运行状态保持【待验证】。

任务九：验证 HAL 弱符号覆盖。

在 `stm32f1xx_hal.c` 中找到弱定义 `HAL_MspInit()`，在 `Core/Src/stm32f1xx_hal_msp.c` 中找到项目实现。
再到 `map/list` 中确认 HAL 库弱实现被丢弃、项目实现被保留，并记录最终地址。
验收依据是能解释“HAL 回调入口存在”和“项目实现被链接采用”之间的区别。

任务十：验证 `HAL_Init()` 调用链。

在 `main.c`、`stm32f1xx_hal.c` 和 `Debug/Three-axis_cloud_platformV2.list` 中追踪 `main -> HAL_Init -> HAL_InitTick/HAL_MspInit`。
验收依据是同时记录 C 源码调用和当前 ELF 反汇编调用，且不把这条链路扩展解释为所有外设已经初始化成功。

任务十一：建立 `.su` 栈估算证据表。

在 `Debug/**/*.su` 中找出若干高栈使用函数、ISR 入口函数和项目控制函数，记录函数名、估算字节数、限定标签和所属文件。
验收依据是能解释“最大单函数估算”“某条调用链估算”和“运行期栈水位证明”不是同一层结论。

实践边界：

当前任务优先形成表格、链路图、搜索记录和计算过程。涉及 IDE 现场、构建日志、断点数值、外部波形、主机侧结果或硬件响应时，若没有截图、日志或仓库外实测证据，结论保持【待验证】。

## 12. 思考题

1. 如果某个 HAL 源文件存在于库目录，但没有出现在 `objects.list`，教材能否把它写成项目构建主线？为什么？
2. 如果 `stm32f1xx_hal_conf.h` 启用了某个模块，但 `subdir.mk` 没有对应源文件，可能会出现什么问题？
3. 为什么 `Debug/makefile` 使用 `@"objects.list"` 对理解构建链路很重要？
4. `-ffunction-sections` 和 `-fdata-sections` 出现在编译命令中，对后续链接分析有什么提示？
5. 为什么第04章必须放在具体外设章节之前？
6. 如何区分“头文件启用模块”、“源文件参与构建”和“业务代码实际调用”这三层证据？
7. 为什么 `--gc-sections` 会让“对象文件进入链接”和“函数进入 ELF”变成两层证据？
8. 为什么 `map/list` 能帮助判断链接事实，却不能替代运行时断点证据？
9. 为什么 Debug ELF 存在，不等于板上正在运行最新源码？
10. 为什么 HAL 库弱 `HAL_MspInit()` 被丢弃，反而可以证明项目强定义生效？
11. 为什么 `USE_HAL_DRIVER` 和 `HAL_*_MODULE_ENABLED` 不是同一层配置？
12. 为什么 `stm32f1xx_ll_usb.c` 进入构建，不能推导出整个项目采用 LL 风格？
13. 为什么 `.su` 文件中最大的单函数栈估算值不能直接当作系统最大栈深度？

## 13. 本章总结

本章建立了 HAL 工程裁剪与构建产物的证据链。

已经确认的结论是：

- `stm32f1xx_hal_conf.h` 是 HAL 模块选择的配置入口。
- `Drivers/STM32F1xx_HAL_Driver` 是 HAL/LL 驱动源码和头文件所在位置。
- `Debug/sources.mk` 说明 HAL 驱动目录被纳入构建扫描。
- HAL 目录 `subdir.mk` 说明具体 HAL 源文件被编译为对象文件。
- `Debug/objects.list` 说明哪些对象文件进入最终链接。
- `Debug/makefile` 通过 `objects.list`、链接脚本和库选项生成 ELF、map 和 list 输出。
- 当前构建使用 `-ffunction-sections`、`-fdata-sections` 和 `--gc-sections`，
  因此对象进入链接不等于其中所有函数都进入最终 ELF。
- `map`、`list` 和 `.su` 可以继续提供链接裁剪、反汇编和栈使用估算证据。
- `.su` 的最大单函数估算值不等于系统最大栈深度，调用链、中断嵌套和运行水位必须分层验证。
- `nano.specs`、`nosys.specs`、浮点 printf/scanf 选项会影响 C 库符号和链接结果。
- `USE_HAL_DRIVER` 属于工程级编译宏，`HAL_*_MODULE_ENABLED` 属于 HAL 模块级配置宏。
- HAL 库弱 `HAL_MspInit()` 被丢弃，项目强 `HAL_MspInit()` 被保留，说明 MSP 全局初始化采用项目实现。
- 当前 Debug 构建中 HAL 驱动目录 18 个 `.c` 与 `objects.list` 中 18 个 HAL/LL 对象一一对应。
- 当前 Debug 目录中存在 46 个 `.su` 和 46 个 `.cyclo` 分析产物，可作为后续栈使用和复杂度分析入口。

本章待验证分类：

| 类别 | 已由本章证明 | 仍保持【待验证】 |
|---|---|---|
| 构建层级边界 | 头文件启用、源文件参与构建、对象文件进入链接和业务代码调用可以作为四层不同证据拆开判断 | 不能把任一单层证据直接写成项目功能已经运行 |
| 宏与符号边界 | 工程级编译宏、HAL 模块宏、弱符号覆盖和链接符号能说明当前构建选择与可替换入口 | 不能替代函数运行事实、业务调用路径或运行时分支命中证据 |
| 模块可用性边界 | 本章确认 HAL 构建链路、模块可用性和部分构建产物存在 | 不把库文件存在解释为外设功能已经运行或业务主线已经采用 |
| 产物新鲜度边界 | Debug 目录中的构建产物能作为当前仓库内静态分析入口 | 构建产物存在不等于产物来自最新源码，也不等于已经下载到目标板运行 |
| 静态栈边界 | `.su` 静态栈估算可以提示风险入口 | 不能替代运行期栈水位、异常现场、真实调用深度或硬件运行证据 |

下一章可以进入 CMSIS 与内核访问，因为当前已经确认：平台、工程配置、启动路径和 HAL 构建证据四层前置已经建立。

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

- `Core/Inc/stm32f1xx_hal_conf.h`
- `.mxproject`
- `Drivers/STM32F1xx_HAL_Driver`
- `Debug/sources.mk`
- `Debug/makefile`
- `Debug/Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk`
- `Debug/objects.list`
- `Debug/Three-axis_cloud_platformV2.map`
- `Debug/Three-axis_cloud_platformV2.list`
- `Debug/**/*.su`
- `Debug/**/*.cyclo`

配置与构建证据：

- `HAL_MODULE_ENABLED`
- `USE_HAL_DRIVER`
- `HAL_GPIO_MODULE_ENABLED`
- `HAL_I2C_MODULE_ENABLED`
- `HAL_PCD_MODULE_ENABLED`
- `HAL_TIM_MODULE_ENABLED`
- `HAL_UART_MODULE_ENABLED`
- `HAL_CORTEX_MODULE_ENABLED`
- `HAL_DMA_MODULE_ENABLED`
- `HAL_FLASH_MODULE_ENABLED`
- `HAL_EXTI_MODULE_ENABLED`
- `HAL_PWR_MODULE_ENABLED`
- `HAL_RCC_MODULE_ENABLED`
- 调度适配开关为 `0U`
- `USE_FULL_ASSERT` 未启用
- `objects.list`
- `Three-axis_cloud_platformV2.elf`
- `Three-axis_cloud_platformV2.map`
- `Three-axis_cloud_platformV2.list`
- `-ffunction-sections`
- `-fdata-sections`
- `-fstack-usage`
- `-fcyclomatic-complexity`
- `-Wl,--gc-sections`
- `--specs=nano.specs`
- `--specs=nosys.specs`
- `-u _printf_float`
- `-u _scanf_float`
- `-mfloat-abi=soft`
- `HAL_Init`
- `HAL_MspInit`
- `HAL_InitTick`
- `HAL_SYSTICK_Config`
- `HAL_NVIC_SetPriorityGrouping`
- `main.su`
- `main.cyclo`
- `pid.su`
- `pid.cyclo`
- `computeMotorCommands.su`
- `computeMotorCommands.cyclo`
- `updatePID`
- `computeMotorCommands`

质量自检：

- P0 事实错误：通过
- P1 依赖断层：通过
- P2 逻辑连贯：通过
- P3 项目证据：通过
- P4 原理展开：通过
- P5 调试实践：通过
- P6 表达统一：通过

---
> 导航：上一章：[第03章_启动流程与内存布局](第03章_启动流程与内存布局.md) ｜ 下一章：[第05章_CMSIS与内核访问](第05章_CMSIS与内核访问.md)
