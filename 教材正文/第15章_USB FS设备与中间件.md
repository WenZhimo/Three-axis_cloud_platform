# 第15章 USB FS设备与中间件

## 1. 本章目标

- 理解项目为什么需要先建立 USB FS 设备和 USB Device 中间件基础。
- 看懂 `.ioc` 中 USB 48MHz 时钟、PA11/PA12 和 USB 中断配置的证据。
- 能追踪 `main.c`、`usb_device.c`、`usbd_conf.c` 和 USB Device Library 之间的初始化链路。
- 区分 USB FS 设备底层、PCD 驱动、USB Device 中间件和 CDC 类之间的层次关系。
- 明确本章只建立 USB 设备与中间件框架，不提前展开 CDC 收发接口、描述符和静态内存分配细节。

## 2. 前置知识

- 系统时钟树
- GPIO输出与复用
- NVIC中断配置
- Makefile构建产物

第06章已经说明项目把 USB 外设时钟配置为 48MHz，第07章已经说明外设引脚和 MSP 初始化不只是协议参数，第09章已经说明中断入口和 HAL IRQ 分发，第04章已经说明如何用构建产物确认源文件是否真正参与编译。

本章在这些基础上分析 USB FS 设备和 USB Device 中间件。USB CDC 虚拟串口、USB 描述符、唯一序列号和静态内存分配安排在第16章，本章只为它们建立前置框架。

## 3. 问题背景

项目中已经存在 USART3 调试输出，也存在 USB Device 相关目录。读者很容易产生两个误判：

1. 看到 USB Device 初始化，就以为项目的调试输出主线已经切到 USB。
2. 看到 CDC 类库参与构建，就直接跳到 CDC 收发函数和描述符细节。

第11章已经确认，当前 `printf()` 调试输出主线是 USART3。本章需要解决另一个问题：USB 作为一个设备功能是如何在工程中被初始化、被中间件接管、并进入可被主机枚举的准备状态。

项目中的 USB 证据链可以分成四段：

1. `.ioc` 配置 USB Device CDC_FS、PA11/PA12、USB 48MHz 时钟和 USB 中断。
2. `main.c` 在外设初始化阶段调用 `MX_USB_DEVICE_Init()`。
3. `USB_DEVICE/App/usb_device.c` 调用 `USBD_Init()`、`USBD_RegisterClass()`、`USBD_CDC_RegisterInterface()` 和 `USBD_Start()`。
4. `USB_DEVICE/Target/usbd_conf.c` 把 USB Device Library 连接到底层 PCD 驱动，并配置 USB FS 和 PMA 端点。

本章的重点不是“USB CDC 如何收发数据”，而是“USB 设备栈如何被搭起来”。

## 4. 核心概念

- USB FS设备：USB Full Speed 设备模式。本项目使用 MCU 内置 USB 外设作为设备端，而不是主机端。
- USB时钟：USB FS 需要稳定的 48MHz 外设时钟。项目在 `.ioc` 和 `SystemClock_Config()` 中把 USB 时钟来源配置为 PLL 分频。
- D+/D-：USB 差分信号线。本项目 `.ioc` 将 PA12 配置为 USB_DP，将 PA11 配置为 USB_DM。
- PCD：Peripheral Controller Driver，HAL 中面向 USB 设备控制器的底层驱动层。本项目使用 `PCD_HandleTypeDef hpcd_USB_FS`。
- USB Device中间件：ST 提供的设备协议栈，位于 `Middlewares/ST/STM32_USB_Device_Library`。
- USBD：USB Device Library 的核心对象和函数前缀，例如 `USBD_Init()`、`USBD_Start()`。
- 端点：USB 设备和主机交换数据的逻辑通道。项目在 `usbd_conf.c` 中为控制端点和 CDC 相关端点配置 PMA 缓冲区。
- PMA：Packet Memory Area，STM32 USB FS 外设用于端点数据收发的专用缓冲区区域。

这些概念服务于正式知识点 `USB FS设备` 和 `USB Device中间件`，不新增结构外知识点。

## 5. 工作原理

USB Device 初始化可以理解为两层配合。

第一层是 MCU USB FS 设备控制器。它需要 USB 时钟、D+/D- 引脚、USB 中断和 PCD 句柄。`usbd_conf.c` 中的 `USBD_LL_Init()` 会建立 `USBD_HandleTypeDef` 和 `PCD_HandleTypeDef` 的相互关联，并把 `hpcd_USB_FS.Instance` 设置为 `USB`，速度设置为 `PCD_SPEED_FULL`。

第二层是 USB Device 中间件。`usb_device.c` 中的 `MX_USB_DEVICE_Init()` 先创建设备栈上下文，再注册类，再注册接口，最后启动设备栈。它不是直接操作 USB 寄存器，而是通过 USBD 核心和底层 LL 接口间接调用 PCD。

当前项目中，初始化顺序可以概括为：

1. `SystemClock_Config()` 配置系统时钟和 USB 外设时钟。
2. `main()` 调用 `MX_USB_DEVICE_Init()`。
3. `MX_USB_DEVICE_Init()` 调用 `USBD_Init()`。
4. `USBD_Init()` 进入 USB Device Library，并调用底层 `USBD_LL_Init()`。
5. `USBD_LL_Init()` 初始化 PCD、配置 PMA 端点，并建立中间件与 PCD 的指针关联。
6. `MX_USB_DEVICE_Init()` 注册 CDC 类和接口，然后调用 `USBD_Start()`。
7. USB 中断到来时，项目中断入口调用 `HAL_PCD_IRQHandler(&hpcd_USB_FS)`，再由 HAL PCD 回调把事件送回 USB Device Library。

这条链路说明：USB 功能不是一个单独函数完成的，而是 CubeMX 生成代码、HAL PCD、USBD 中间件、类驱动和中断分发共同组成的分层结构。

## 6. STM32实现机制

### 1. USB 时钟和引脚

`.ioc` 中记录：

- `RCC.USBFreq_Value=48000000`
- `RCC.USBPrescaler=RCC_USBCLKSOURCE_PLL_DIV1_5`
- `PA11.Signal=USB_DM`
- `PA12.Signal=USB_DP`
- USB Device 虚拟模式为 CDC_FS

`main.c` 的 `SystemClock_Config()` 中也能看到 USB 外设时钟选择为 `RCC_USBCLKSOURCE_PLL_DIV1_5`。这说明 USB 48MHz 不是附带结果，而是被项目显式配置的外设时钟。

PA11/PA12 没有像普通 GPIO 那样在 `gpio.c` 中手写成输出或输入。它们作为 USB 专用功能由 USB 外设接管，配置来源主要体现在 `.ioc` 和 USB PCD 初始化路径。

### 2. PCD MSP 与中断

`USB_DEVICE/Target/usbd_conf.c` 中的 `HAL_PCD_MspInit()` 在 USB 分支中完成两个关键动作：

- 调用 `__HAL_RCC_USB_CLK_ENABLE()` 启用 USB 外设时钟。
- 配置并使能 USB 低优先级中断。

中断入口位于 `Core/Src/stm32f1xx_it.c`，函数体中调用 `HAL_PCD_IRQHandler(&hpcd_USB_FS)`。这和第09章讲过的 HAL IRQ 分发一致：中断入口不直接处理完整协议，而是把硬件中断交给 HAL PCD，再由回调连接到 USB Device Library。

### 3. `USBD_LL_Init()` 与 PCD

`USBD_LL_Init()` 是 USB Device Library 到 HAL PCD 的连接层。当前项目中它完成：

- `hpcd_USB_FS.pData = pdev`
- `pdev->pData = &hpcd_USB_FS`
- `hpcd_USB_FS.Instance = USB`
- `hpcd_USB_FS.Init.dev_endpoints = 8`
- `hpcd_USB_FS.Init.speed = PCD_SPEED_FULL`
- 低功耗、LPM 和电池充电检测均为 Disable
- 调用 `HAL_PCD_Init(&hpcd_USB_FS)`
- 调用多次 `HAL_PCDEx_PMAConfig()` 配置端点 PMA

这里的关键是指针互联：USBD 层通过 `pdev->pData` 找到底层 PCD，PCD 回调又通过 `hpcd->pData` 找回 USBD 设备对象。没有这组关联，中间件和底层驱动就无法互相传递事件。

### 4. USB Device Library

`Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c` 提供 `USBD_Init()`、`USBD_RegisterClass()` 和 `USBD_Start()` 等核心函数。`Class/CDC/Src/usbd_cdc.c` 提供 CDC 类驱动对象和类回调。

`Debug/objects.list` 显示以下对象参与链接：

- `usbd_core.o`
- `usbd_ctlreq.o`
- `usbd_ioreq.o`
- `usbd_cdc.o`
- `usb_device.o`
- `usbd_cdc_if.o`
- `usbd_desc.o`
- `usbd_conf.o`

这说明 USB 中间件不是“目录存在但没编译”，而是确实进入当前 Debug 构建产物。第04章的 Makefile 构建产物知识在这里发挥作用。

## 7. 项目中的应用

`main()` 的初始化顺序中，`MX_USB_DEVICE_Init()` 位于 `MX_USART3_UART_Init()` 之后、`MX_TIM6_Init()` 之前。也就是说，项目启动阶段确实初始化了 USB Device 栈。

从运行边界看，当前项目可确认的是：

- USB FS 设备功能被配置并启动。
- USB Device Library 和 CDC 类对象参与构建。
- USB 中断入口会把事件交给 HAL PCD。
- `usbd_conf.c` 配置了 FS PCD 和 PMA 端点。

但当前第15章不能得出以下结论：

- 不能把 USB 写成 `printf()` 主输出路径。第11章已经确认主输出是 USART3。
- 不能展开 CDC 收发缓冲区和 `CDC_Transmit_FS()` 的业务使用，因为这属于第16章边界。
- 不能把 USB 描述符和唯一序列号提前作为本章主线。
- 不能声称当前项目存在上位机 USB 协议解析；知识库中只确认 USB CDC 接口初始化，未发现业务协议解析主线。

因此，本章在项目中的定位是：建立 USB 设备栈框架，帮助读者理解后续 CDC 接口与描述符为什么需要这些前置条件。

## 8. 代码分析

### 1. `main.c` 中的 `MX_USB_DEVICE_Init()`

`main()` 在系统时钟、GPIO、定时器、I2C、TIM8 和 USART3 初始化后调用 `MX_USB_DEVICE_Init()`。

这个位置说明 USB Device 初始化属于系统外设初始化阶段，而不是 500Hz 控制循环的一部分。后续控制主循环不依赖 USB 才能执行；USB 当前更像一个已初始化的通信支线。

### 2. `MX_USB_DEVICE_Init()`

`USB_DEVICE/App/usb_device.c` 中的 `MX_USB_DEVICE_Init()` 依次调用：

- `USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS)`
- `USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC)`
- `USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS)`
- `USBD_Start(&hUsbDeviceFS)`

这四步分别对应设备栈创建、类注册、接口注册和启动。第15章只分析这条框架链路；`FS_Desc` 和 `USBD_Interface_fops_FS` 的具体内容留到第16章。

### 3. `USBD_Init()`

`USBD_Init()` 来自 `usbd_core.c`。它属于 USB Device 中间件核心函数，负责初始化 USBD 设备对象，并在内部进入底层初始化流程。

项目中 `USBD_Init()` 的参数包含：

- `hUsbDeviceFS`：项目的 USB Device 句柄。
- `FS_Desc`：全速设备描述符集合，具体描述符内容第16章展开。
- `DEVICE_FS`：设备速度和实例选择。

这一层说明 CubeMX 生成的 `usb_device.c` 不是直接初始化 PCD，而是先进入中间件核心。

### 4. `USBD_LL_Init()`

`USBD_LL_Init()` 位于 `usbd_conf.c`，是项目为 USB Device Library 提供的底层接口。它初始化 `hpcd_USB_FS`，调用 `HAL_PCD_Init()`，并配置 PMA。

这一函数是第15章最重要的桥：上面连着中间件，下面连着 HAL PCD 和 USB FS 外设。

### 5. `HAL_PCD_MspInit()`

`HAL_PCD_Init()` 内部会触发 MSP 初始化。项目在 `HAL_PCD_MspInit()` 中启用 USB 外设时钟并打开 USB 中断。

如果只看 `MX_USB_DEVICE_Init()`，读者会以为 USB 初始化只有 USBD 函数；追踪到 MSP 后才能看到硬件资源的准备过程。

### 6. `HAL_PCDEx_PMAConfig()`

`USBD_LL_Init()` 中多次调用 `HAL_PCDEx_PMAConfig()`，为不同端点地址配置 PMA 偏移。

本章只说明 PMA 是 USB FS 端点数据缓冲区，确认项目确实做了端点缓冲配置。每个端点与 CDC 类请求、发送和接收之间的详细关系留到第16章。

### 7. `Debug/objects.list`

`Debug/objects.list` 是构建证据。它证明 USB Device 中间件核心、CDC 类、应用层 USB 文件和目标适配文件都参与链接。

这能避免一种常见误判：源码目录存在并不等于功能参与构建。当前项目通过 objects list 可以确认 USB 栈相关对象进入了 Debug 构建。

## 9. 调试方法

USB 调试先检查工程配置，再检查初始化链路，最后检查主机侧现象。

工程配置检查：

- `.ioc` 中 USB 时钟是否为 48MHz。
- `.ioc` 中 PA11/PA12 是否分别为 USB_DM/USB_DP。
- `.ioc` 中 USB Device 是否配置为 CDC_FS。
- `main.c` 中是否调用 `MX_USB_DEVICE_Init()`。

初始化链路检查：

- `usb_device.c` 中 `USBD_Init()`、`USBD_RegisterClass()`、`USBD_CDC_RegisterInterface()` 和 `USBD_Start()` 是否依次存在。
- `usbd_conf.c` 中 `USBD_LL_Init()` 是否设置 `PCD_SPEED_FULL` 并调用 `HAL_PCD_Init()`。
- `HAL_PCD_MspInit()` 是否启用 USB 时钟和中断。
- `stm32f1xx_it.c` 中 USB 中断入口是否调用 `HAL_PCD_IRQHandler(&hpcd_USB_FS)`。
- `Debug/objects.list` 中是否包含 USBD core、USBD CDC、USB_DEVICE App 和 Target 相关对象。

主机侧检查：

- 插入 USB 后，主机是否能识别出一个 USB 设备。
- 若不能识别，先回到 USB 时钟、D+/D- 配置、中断和 PCD 初始化链路排查。
- 若能识别但不能作为虚拟串口正常使用，再进入第16章检查 CDC 接口、描述符和缓冲区。

当前工作树没有 USB 线缆连接、主机设备管理器截图、枚举日志或抓包证据，因此主机侧枚举结果只能标记为【待验证】。本章只确认工程内部 USB FS 设备和中间件初始化证据。

## 10. 常见问题

### 1. 为什么 USB 需要 48MHz 时钟？

触发条件：读者看到 `.ioc` 中 `RCC.USBFreq_Value=48000000`。

可能原因：USB FS 外设需要符合 USB 全速设备时序的外设时钟。项目通过 PLL 分频为 USB 提供 48MHz。若这个时钟配置错误，USB 设备可能无法稳定枚举。

### 2. 为什么 PA11/PA12 不像普通 GPIO 那样在 `gpio.c` 中配置？

触发条件：读者在 `gpio.c` 中找不到 USB_DM/USB_DP 的普通 GPIO 初始化。

可能原因：PA11/PA12 是 USB 外设专用信号，由 USB 外设和 PCD 初始化路径接管。它们的配置证据主要来自 `.ioc` 和 USB PCD 初始化，而不是普通 GPIO 输出配置。

### 3. `USBD` 和 `PCD` 有什么区别？

触发条件：读者同时看到 `USBD_Init()` 和 `HAL_PCD_Init()`。

可能原因：`USBD` 是 USB Device 中间件层，负责设备栈、类和请求处理；`PCD` 是 HAL 底层驱动层，负责 MCU USB 设备控制器。项目通过 `USBD_LL_Init()` 把两层连接起来。

### 4. 为什么 objects list 很重要？

触发条件：读者看到 `Middlewares` 目录后就认为 USB 中间件一定被使用。

可能原因：目录存在只能说明源码在工作树中。`Debug/objects.list` 显示对象文件参与链接，才能证明这些模块进入当前构建。

### 5. 既然注册了 CDC 类，为什么本章不讲 CDC 收发？

触发条件：`MX_USB_DEVICE_Init()` 中出现 `USBD_CDC_RegisterInterface()`。

可能原因：第15章的正式知识点是 `USB FS设备` 和 `USB Device中间件`。CDC 接口、缓冲区、描述符和虚拟串口行为属于第16章。提前展开会破坏教学顺序。

### 6. USB 初始化成功是否等于项目已经通过 USB 和上位机通信？

触发条件：读者看到 `USBD_Start()`。

可能原因：`USBD_Start()` 只能说明设备栈被启动。是否成功枚举、是否出现虚拟串口、是否有项目级协议解析，需要主机侧证据和第16章接口分析。当前工作树未提供主机枚举日志，标记为【待验证】。

## 11. 实践任务

1. 在 `.ioc` 中找到 USB 48MHz、PA11/PA12 和 CDC_FS 配置。验收依据是能说明 USB FS 的时钟和引脚来源。
2. 在 `main.c` 中找到 `MX_USB_DEVICE_Init()` 的调用位置。验收依据是能说明它属于启动阶段初始化，而不是控制循环任务。
3. 在 `usb_device.c` 中追踪 `USBD_Init()`、`USBD_RegisterClass()`、`USBD_CDC_RegisterInterface()` 和 `USBD_Start()`。验收依据是能画出 USB Device 初始化顺序。
4. 在 `usbd_conf.c` 中找到 `USBD_LL_Init()`、`HAL_PCD_Init()` 和 `HAL_PCDEx_PMAConfig()`。验收依据是能解释中间件如何连接到底层 PCD。
5. 在 `stm32f1xx_it.c` 中找到 USB 中断入口对 `HAL_PCD_IRQHandler(&hpcd_USB_FS)` 的调用。验收依据是能说明 USB 事件如何进入 HAL PCD。
6. 在 `Debug/objects.list` 中列出 USB Device Library 和 USB_DEVICE 相关对象。验收依据是能区分“源码存在”和“参与构建”。

## 12. 思考题

1. 为什么 USB FS 设备章节必须依赖系统时钟树？
2. 如果 USB 时钟不是 48MHz，设备枚举可能出现什么现象？
3. 为什么 `USBD_LL_Init()` 要同时设置 `hpcd_USB_FS.pData` 和 `pdev->pData`？
4. 为什么不能只凭 `USBD_Start()` 就断定 USB 虚拟串口业务已经可用？
5. 如果主机不能识别 USB 设备，应该优先检查第15章中的哪些证据？
6. 为什么第15章要用 `Debug/objects.list` 证明中间件参与构建？

## 13. 本章总结

本章建立了三轴云台项目中 USB FS 设备和 USB Device 中间件的证据链。

已经确认的结论是：

- `.ioc` 配置 USB Device CDC_FS、PA11/PA12、USB 48MHz 时钟和 USB 中断。
- `main.c` 在启动初始化阶段调用 `MX_USB_DEVICE_Init()`。
- `usb_device.c` 通过 `USBD_Init()`、`USBD_RegisterClass()`、`USBD_CDC_RegisterInterface()` 和 `USBD_Start()` 搭建设备栈。
- `usbd_conf.c` 通过 `USBD_LL_Init()` 连接 USB Device Library 和 HAL PCD。
- `usbd_conf.c` 将 `hpcd_USB_FS` 配置为 USB FS、全速模式，并配置端点 PMA。
- USB 中断入口把事件交给 `HAL_PCD_IRQHandler(&hpcd_USB_FS)`。
- `Debug/objects.list` 证明 USB Device Library 核心、CDC 类和 USB_DEVICE 适配文件参与当前 Debug 构建。

下一章可以进入 USB CDC 接口与描述符。第15章已经建立了设备栈框架，第16章才能在此基础上分析 CDC 接口、描述符、唯一序列号和静态内存分配。

---

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

本章对应正式知识点：

- USB FS设备
- USB Device中间件

项目证据：

- `Three-axis_cloud_platformV2.ioc`
- `Core/Src/main.c`
- `Core/Src/stm32f1xx_it.c`
- `USB_DEVICE/App/usb_device.c`
- `USB_DEVICE/Target/usbd_conf.c`
- `Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c`
- `Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c`
- `Debug/objects.list`
- `Debug/sources.mk`
- `Debug/makefile`

引用的函数、配置项和变量：

- `MX_USB_DEVICE_Init()`
- `USBD_Init()`
- `USBD_RegisterClass()`
- `USBD_CDC_RegisterInterface()`
- `USBD_Start()`
- `USBD_LL_Init()`
- `HAL_PCD_Init()`
- `HAL_PCD_MspInit()`
- `HAL_PCDEx_PMAConfig()`
- `HAL_PCD_IRQHandler()`
- `hUsbDeviceFS`
- `hpcd_USB_FS`
- `FS_Desc`
- `USBD_CDC`
- `USBD_Interface_fops_FS`
- `RCC.USBFreq_Value=48000000`
- `RCC.USBPrescaler=RCC_USBCLKSOURCE_PLL_DIV1_5`
- `PA11.Signal=USB_DM`
- `PA12.Signal=USB_DP`
- `USB_DEVICE.VirtualModeFS=Cdc_FS`

质量自检：

- P0 事实错误：通过。
- P1 依赖断层：通过。
- P2 逻辑连贯：通过。
- P3 项目证据：通过。
- P4 原理展开：通过。
- P5 调试实践：通过。
- P6 表达统一：通过。
