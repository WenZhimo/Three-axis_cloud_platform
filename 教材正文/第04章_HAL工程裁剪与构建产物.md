# 第04章 HAL工程裁剪与构建产物

## 1. 本章目标

- 明确本项目启用的 HAL 模块、参与构建的 HAL 源文件和最终对象文件之间的关系。
- 区分“库文件存在”“宏启用”和“实际参与构建”三类证据。
- 看懂 `Debug/sources.mk`、`Debug/makefile` 和 `Debug/objects.list` 在构建链路中的位置。
- 为后续外设初始化、中间件和调试章节建立工程依据。

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

这些概念只服务第04章的三个正式知识点，不新增结构外知识点。

## 5. 工作原理

本项目的 HAL 工程裁剪和构建产物可以按三层理解。

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

因此，第04章判断项目“实际构建了什么”，不能只看 `Drivers/` 目录，也不能只看 `stm32f1xx_hal_conf.h`，必须把配置、构建规则和对象文件列表合起来看。

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

同时，`Debug/Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk` 和 `Debug/objects.list` 证明以下 HAL 源文件已经形成对象文件并参与链接：

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

这些文件属于 HAL/LL 驱动层。本章只确认它们进入工程构建，不展开各模块内部实现。

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

在项目主线中，第04章仍不是控制算法或外设应用章节，而是后续外设章节的工程证据底座。

## 8. 代码分析

### 1. `stm32f1xx_hal_conf.h`

该文件的核心作用是 HAL 模块选择。

被定义的 `HAL_*_MODULE_ENABLED` 宏代表对应 HAL 模块在配置层被启用；被注释的模块不能仅凭库文件存在写成项目主线。文件后半部分通过 `#ifdef HAL_*_MODULE_ENABLED` 条件包含对应头文件，这说明“宏启用”会影响编译期可见的 HAL 声明。

本项目还设置：

- `HSE_VALUE = 8000000U`
- `TICK_INT_PRIORITY = 0U`
- 调度适配开关为 `0U`
- `PREFETCH_ENABLE = 1U`

这些配置为后续时钟、节拍和系统配置章节提供证据，但本章只记录它们的工程位置。

### 2. `Debug/sources.mk`

`sources.mk` 是自动生成文件，文件头明确提示不要手工编辑。它列出的 `SUBDIRS` 包含：

- `Core/Src`
- `Core/Startup`
- `Drivers/CustomDrivers/Src`
- `Drivers/SRC/Src`
- `Drivers/STM32F1xx_HAL_Driver/Src`
- 中间件与 USB Device 目录

这说明构建系统把 HAL 驱动目录和项目业务目录一起纳入构建扫描。

### 3. HAL 目录 `subdir.mk`

HAL 目录下的 `subdir.mk` 明确列出 `C_SRCS`、`OBJS` 和 `C_DEPS`。这比单纯看 `Drivers/STM32F1xx_HAL_Driver/Src` 目录更强，因为它证明这些源文件已经进入当前 Debug 构建规则。

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

### 4. `Debug/objects.list`

`objects.list` 是最终链接输入的证据。它包含 Core、启动文件、自定义驱动、项目算法、HAL、中间件和 USB Device 对象文件。

对第04章来说，最关键的是它列出了 HAL 驱动对象文件。只有出现在这里的对象文件，才可以说已经参与当前 Debug 链接。

### 5. `Debug/makefile`

`Debug/makefile` 通过 `-include sources.mk` 和各目录 `subdir.mk` 汇总构建规则。它的链接命令使用 `@"objects.list"`，并指定第03章分析过的链接脚本。

它还声明构建输出包括：

- `Three-axis_cloud_platformV2.elf`
- `Three-axis_cloud_platformV2.map`
- `Three-axis_cloud_platformV2.list`

这说明第04章的构建产物不仅能证明“编了哪些文件”，也能为后续大小、符号和反汇编检查提供入口。

## 9. 调试方法

本章阶段的调试目标是确认 HAL 配置和构建产物一致。

可观察对象：

- `stm32f1xx_hal_conf.h` 中相关 `HAL_*_MODULE_ENABLED` 是否按项目需要启用。
- `Debug/sources.mk` 是否包含 `Drivers/STM32F1xx_HAL_Driver/Src`。
- `Debug/Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk` 是否列出对应 `C_SRCS` 和 `OBJS`。
- `Debug/objects.list` 是否包含对应 HAL 对象文件。
- `Debug/makefile` 的链接命令是否使用 `objects.list` 和 `STM32F103RCTX_FLASH.ld`。
- `Debug/Three-axis_cloud_platformV2.map` 和 `Debug/Three-axis_cloud_platformV2.list` 是否存在。

常见异常定位：

- 源码里能看到 HAL 文件，但链接时没有对应对象文件：检查 HAL 目录 `subdir.mk` 和 `objects.list`。
- 宏已经启用但编译找不到声明：检查 `stm32f1xx_hal_conf.h` 的条件包含和 `.cproject` Include Path。
- 某个驱动源文件存在但没有参与链接：以 `objects.list` 为准，不以库目录存在为准。
- 改动配置后构建产物未变化：检查是否重新生成或重新构建了 Debug 目录。
- 链接阶段找不到符号：检查对象文件是否进入 `objects.list`，再检查对应源文件是否进入 `subdir.mk`。

## 10. 常见问题

### 1. 为什么不能只看 `Drivers/STM32F1xx_HAL_Driver` 目录？

因为目录存在只表示工程中有这些库文件。是否参与当前构建，要看 `subdir.mk` 和 `objects.list`。

### 2. 为什么宏启用还不等于对象文件已经链接？

宏启用主要影响头文件包含和条件编译路径；对象文件是否参与链接，需要构建规则和对象列表证明。

### 3. 为什么第04章不展开每个 HAL 模块内部 API？

因为本章的任务是建立工程证据。每个外设的初始化、MSP、时钟和引脚关系会在后续对应章节展开。

### 4. `objects.list` 和 `sources.mk` 哪个更接近最终链接事实？

`objects.list` 更接近最终链接事实，因为 `makefile` 链接命令直接通过它传入对象文件。`sources.mk` 更适合确认源码目录是否被构建系统扫描。

### 5. map 和 list 文件有什么作用？

map 文件用于观察链接后的符号、段和内存分布；list 文件用于观察反汇编和源码混合输出。本章只确认它们由构建系统生成，具体分析留到后续调试和优化场景。

## 11. 实践任务

- 在 `stm32f1xx_hal_conf.h` 中找出当前启用的 HAL 模块宏。
- 在 `Debug/sources.mk` 中确认 HAL 驱动目录是否进入构建目录列表。
- 在 HAL 目录 `subdir.mk` 中找出 HAL 源文件、对象文件和依赖文件列表。
- 在 `Debug/objects.list` 中确认 HAL 对象文件是否进入最终链接。
- 对比 `Drivers/STM32F1xx_HAL_Driver/Src` 与 `objects.list`，说明“库存在”和“参与链接”的区别。

## 12. 思考题

1. 如果某个 HAL 源文件存在于库目录，但没有出现在 `objects.list`，教材能否把它写成项目构建主线？为什么？
2. 如果 `stm32f1xx_hal_conf.h` 启用了某个模块，但 `subdir.mk` 没有对应源文件，可能会出现什么问题？
3. 为什么 `Debug/makefile` 使用 `@"objects.list"` 对理解构建链路很重要？
4. `-ffunction-sections` 和 `-fdata-sections` 出现在编译命令中，对后续链接分析有什么提示？
5. 为什么第04章必须放在具体外设章节之前？

## 13. 本章总结

本章建立了 HAL 工程裁剪与构建产物的证据链。

已经确认的结论是：

- `stm32f1xx_hal_conf.h` 是 HAL 模块选择的配置入口。
- `Drivers/STM32F1xx_HAL_Driver` 是 HAL/LL 驱动源码和头文件所在位置。
- `Debug/sources.mk` 说明 HAL 驱动目录被纳入构建扫描。
- HAL 目录 `subdir.mk` 说明具体 HAL 源文件被编译为对象文件。
- `Debug/objects.list` 说明哪些对象文件进入最终链接。
- `Debug/makefile` 通过 `objects.list`、链接脚本和库选项生成 ELF、map 和 list 输出。

下一章可以进入 CMSIS 与内核访问，因为当前已经确认：平台、工程配置、启动路径和 HAL 构建证据四层前置已经建立。

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

配置与构建证据：

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
- 调度适配开关为 `0U`
- `USE_FULL_ASSERT` 未启用
- `objects.list`
- `Three-axis_cloud_platformV2.elf`
- `Three-axis_cloud_platformV2.map`
- `Three-axis_cloud_platformV2.list`

质量自检：

- P0 事实错误：通过
- P1 依赖断层：通过
- P2 逻辑连贯：通过
- P3 项目证据：通过
- P4 原理展开：通过
- P5 调试实践：通过
- P6 表达统一：通过
