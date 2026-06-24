# 第15章 USB FS设备与中间件

> 导航：上一章：[第14章_I2C主机通信](第14章_I2C主机通信.md) ｜ 下一章：[第16章_USB CDC接口与描述符](第16章_USB CDC接口与描述符.md)

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
- 设备连接：设备端通过全速连接条件让主机检测到设备存在；当前仓库没有线缆、上拉和主机检测记录。
- PCD：Peripheral Controller Driver，HAL 中面向 USB 设备控制器的底层驱动层。本项目使用 `PCD_HandleTypeDef hpcd_USB_FS`。
- USB Device中间件：ST 提供的设备协议栈，位于 `Middlewares/ST/STM32_USB_Device_Library`。
- USBD：USB Device Library 的核心对象和函数前缀，例如 `USBD_Init()`、`USBD_Start()`。
- 端点：USB 设备和主机交换数据的逻辑通道。项目在 `usbd_conf.c` 中为控制端点和 CDC 相关端点配置 PMA 缓冲区。
- 端点地址方向位：端点地址最高位为 1 表示 IN，例如 `0x80`、`0x81`、`0x82`；最高位为 0 表示 OUT。
- 控制端点 EP0：USB 设备必须具备的默认控制端点，用于接收主机的标准请求、地址设置和配置设置。
- Setup 阶段：主机向设备发送控制请求的起点，ST 中间件通过 `USBD_LL_SetupStage()` 解析请求。
- 标准请求：USB 枚举阶段常见的设备请求，例如 `USB_REQ_SET_ADDRESS` 和 `USB_REQ_SET_CONFIGURATION`。
- 设备状态：ST 中间件用 `USBD_STATE_DEFAULT`、`USBD_STATE_ADDRESSED` 和 `USBD_STATE_CONFIGURED` 描述枚举进度。
- PMA：Packet Memory Area，STM32 USB FS 外设用于端点数据收发的专用缓冲区区域。
- BTABLE：USB FS 外设的端点缓冲描述表基址，HAL 底层把 PMA 地址和端点计数写入相关表项。
- 最大包长：当前中间件定义 `USB_FS_MAX_PACKET_SIZE=64U`、`USB_MAX_EP0_SIZE=64U`，说明全速端点和 EP0 的协议尺度。
- SOF：Start Of Frame，全速 USB 主机按 1ms 帧节奏产生帧起点；本章只把它作为 USB 事件，不把它写成项目业务调度源。
- 自供电声明：`USBD_SELF_POWERED=1` 会影响标准请求返回的设备状态位，但它只是当前中间件配置声明，不等于已经证明目标板供电拓扑。
- 静态分配入口：`USBD_malloc` 被映射到 `USBD_static_malloc()`，说明 USB Device 栈使用工程提供的静态分配入口；具体 CDC 类对象和应用缓冲细节留到第16章。
- USB低功耗开关：`hpcd_USB_FS.Init.low_power_enable = DISABLE`，所以 suspend 回调中写 `SCB->SCR` 的低功耗分支不是当前默认启用路径。

这些概念服务于正式知识点 `USB FS设备` 和 `USB Device中间件`，不新增结构外知识点。

## 5. 工作原理

USB Device 初始化可以理解为两层配合。

第一层是 MCU USB FS 设备控制器。它需要 USB 时钟、D+/D- 引脚、USB 中断和 PCD 句柄。

`usbd_conf.c` 中的 `USBD_LL_Init()` 会建立 `USBD_HandleTypeDef` 和 `PCD_HandleTypeDef` 的相互关联。

它把 `hpcd_USB_FS.Instance` 设置为 `USB`，速度设置为 `PCD_SPEED_FULL`。
随后通过 `HAL_PCDEx_PMAConfig()` 配置 EP0 和 CDC 相关端点的 PMA 偏移。

从端点地址看，项目至少配置了：

- `0x00`：EP0 OUT，PMA 偏移 `0x18`。
- `0x80`：EP0 IN，PMA 偏移 `0x58`。
- `0x81`：CDC 数据 IN，PMA 偏移 `0xC0`。
- `0x01`：CDC 数据 OUT，PMA 偏移 `0x110`。
- `0x82`：CDC 命令 IN，PMA 偏移 `0x100`。

这张表说明 PMA 是 USB 外设内部的端点缓冲布局，
不是普通 C 数组，也不是 `UserRxBufferFS` / `UserTxBufferFS` 这类应用层缓冲。
CDC 端点的类语义留到第16章，本章只确认 USB FS 设备栈已经为这些端点准备底层缓冲。

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

但是初始化链路还不是完整枚举链路。`USBD_Start()` 会进一步进入 `USBD_LL_Start()` 和 `HAL_PCD_Start()`，HAL PCD 会启用 USB 设备控制器并执行 `USB_DevConnect()`。

这只能说明设备端已经开始连接和响应的准备动作。主机随后还需要发起 USB reset、通过 EP0 发送 setup 请求、分配地址、选择配置。
在当前 STM32F1 `USB_TypeDef` 路径下，`USB_DevConnect()` 函数体本身只做兼容性占位并返回 `HAL_OK`。
`HAL_PCDEx_SetConnectionState()` 的用户代码区也没有实现板级连接控制。
所以不能把这个函数名理解成“软件已经证明主机检测到设备”。
主机检测还依赖线缆、供电、D+/D- 连接和板级上拉等仓库外条件【待验证】。

在 ST 中间件中，这个过程可以观察为：

1. `USBD_Init()` 或 USB reset 后，`pdev->dev_state` 处于 `USBD_STATE_DEFAULT`。
2. 主机发送 `USB_REQ_SET_ADDRESS` 后，`USBD_SetAddress()` 将非零地址写入设备并进入 `USBD_STATE_ADDRESSED`。
3. 主机发送 `USB_REQ_SET_CONFIGURATION` 后，`USBD_SetConfig()` 才进入 `USBD_STATE_CONFIGURED` 并调用类配置。

因此，本章必须把“工程代码已经启动 USB 设备栈”和“主机已经完成枚举”分开。
当前仓库没有主机侧枚举日志，只能确认前者；后者应标记为【待验证】。

从时间尺度看，USB FS 名义信号速率是 12 Mbit/s，全速主机以 1ms 帧节奏调度 SOF。
这些数值只说明 USB 协议层节奏，不等于 CDC 应用层吞吐量，也不等于项目控制循环调度周期。
当前仓库没有 USB 传输吞吐、延迟或主机驱动日志，性能结论仍需【待验证】。

## 6. STM32实现机制

### 1. USB 时钟和引脚

`.ioc` 中记录：

- `RCC.USBFreq_Value=48000000`
- `RCC.USBPrescaler=RCC_USBCLKSOURCE_PLL_DIV1_5`
- `PA11.Signal=USB_DM`
- `PA12.Signal=USB_DP`
- USB Device 虚拟模式为 CDC_FS

`main.c` 的 `SystemClock_Config()` 中也能看到 USB 外设时钟选择为 `RCC_USBCLKSOURCE_PLL_DIV1_5`。这说明 USB 48MHz 不是附带结果，而是被项目显式配置的外设时钟。

结合项目系统时钟配置，PLLCLK 为 72MHz，USB 分频选择 `PLL_DIV1_5`，
因此 USB 外设时钟为：

`72MHz / 1.5 = 48MHz`

这一步是 USB FS 能否进入稳定枚举的时钟前提。
不过它只证明 MCU 侧时钟配置，不能证明主机侧枚举已经发生。

PA11/PA12 没有像普通 GPIO 那样在 `gpio.c` 中手写成输出或输入。它们作为 USB 专用功能由 USB 外设接管，配置来源主要体现在 `.ioc` 和 USB PCD 初始化路径。

### 2. PCD MSP 与中断

`USB_DEVICE/Target/usbd_conf.c` 中的 `HAL_PCD_MspInit()` 在 USB 分支中完成两个关键动作：

- 调用 `__HAL_RCC_USB_CLK_ENABLE()` 启用 USB 外设时钟。
- 配置并使能 USB 低优先级中断。

中断入口位于 `Core/Src/stm32f1xx_it.c`，函数体中调用 `HAL_PCD_IRQHandler(&hpcd_USB_FS)`。

这和第09章讲过的 HAL IRQ 分发一致：中断入口不直接处理完整协议，而是把硬件中断交给 HAL PCD，再由回调连接到 USB Device Library。

这里还要拆开一个命名陷阱：STM32F103 设备头文件把 `USB_LP_IRQHandler` 和 `CAN1_RX0_IRQHandler` 都定义为 `USB_LP_CAN1_RX0_IRQHandler`。也就是说，这个名字表达的是 USB 低优先级和 CAN1 RX0 共用的 NVIC 向量入口，不自动证明项目启用了 CAN 接收业务。

当前项目的证据指向 USB 路径：`.ioc` 启用 `NVIC.USB_LP_CAN1_RX0_IRQn`，`HAL_PCD_MspInit()` 配置并使能 `USB_LP_CAN1_RX0_IRQn`，`USB_LP_CAN1_RX0_IRQHandler()` 内部调用 `HAL_PCD_IRQHandler(&hpcd_USB_FS)`。同时 `stm32f1xx_hal_conf.h` 中 CAN HAL 模块保持注释状态，当前仓库没有发现 `MX_CAN_Init()` 或 CAN 业务处理路径。因此第15章只能把该入口写成当前 USB PCD 分发入口，不能因为名称中含有 `CAN1_RX0` 就推断项目使用 CAN。

在 STM32F1 USB FS 路径下，`HAL_PCD_IRQHandler()` 会读取 `USB_ISTR` 相关中断状态，
并按事件分派到不同处理：

- `CTR`：端点正确传输，进入 `PCD_EP_ISR_Handler()`。
- `RESET`：调用 reset 回调，并把 USB 地址复位为 0。
- `WKUP`：恢复回调。
- `SUSP`：挂起回调。
- `SOF`：SOF 回调。
- `PMAOVR`、`ERR`、`ESOF`：清标志后返回。

这条分派链说明 USB 中断不是单一事件。
调试时如果只看 IRQ 是否进入，仍然不足以判断枚举推进到哪一步。

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

PMA 配置还体现了 USB FS 外设的一个特殊性：端点数据不是直接从普通 SRAM 地址收发，
而是先经过 USB 外设的 Packet Memory Area。
HAL 底层使用 `BTABLE`、端点 TX/RX 地址和计数字段来描述这些缓冲。
因此 `HAL_PCDEx_PMAConfig()` 的偏移值是外设缓冲布局的一部分，
不是可随意替换的应用层数组下标。

### 4. USB Device Library

`Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c` 提供 `USBD_Init()`、`USBD_RegisterClass()` 和 `USBD_Start()` 等核心函数。

`Class/CDC/Src/usbd_cdc.c` 提供 CDC 类驱动对象和类回调。

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

### 5. EP0、标准请求与设备状态

`usbd_core.c` 中的 `USBD_Init()` 会把 `pdev->dev_state` 置为 `USBD_STATE_DEFAULT`。

USB reset 回调最终进入 `USBD_LL_Reset()`，中间件会打开 EP0 OUT 和 EP0 IN，端点类型为控制端点，最大包长使用 `USB_MAX_EP0_SIZE`。

主机发来的 setup 包进入 `USBD_LL_SetupStage()`。该函数解析 `pdev->request`，然后根据请求接收者把标准设备请求交给 `USBD_StdDevReq()`。

`usbd_ctlreq.c` 中可以看到两个状态推进点：

- `USB_REQ_SET_ADDRESS` 进入 `USBD_SetAddress()`，非零地址会使设备进入 `USBD_STATE_ADDRESSED`。
- `USB_REQ_SET_CONFIGURATION` 进入 `USBD_SetConfig()`，配置成功后设备进入 `USBD_STATE_CONFIGURED`。

这个状态机是本章的关键拆分点。`USBD_Start()` 只是让设备控制器开始工作；只有主机完成地址和配置阶段，设备状态才会进入 configured。

项目仓库没有主机侧枚举记录，所以不能把 `USBD_Start()` 的返回值等同于 `USBD_STATE_CONFIGURED`。

还要注意，`usbd_core.c` 中的类数据收发回调通常要求 `pdev->dev_state == USBD_STATE_CONFIGURED`。
也就是说，CDC 类已经注册并不等于 CDC 数据端点已经可以收发。
只有主机配置完成后，类回调才有进入正常数据阶段的前提。

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

这个位置说明 USB Device 初始化属于系统外设初始化阶段，而不是 500Hz 实时控制循环的一部分。后续控制主循环不依赖 USB 才能执行；USB 当前更像一个已初始化的通信支线。

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

从状态角度看，`USBD_Init()` 还会把设备状态初始化为 `USBD_STATE_DEFAULT`。
这意味着它建立的是设备栈初始上下文，不代表主机已经给设备分配地址或选择配置。

### 4. `USBD_LL_Init()`

`USBD_LL_Init()` 位于 `usbd_conf.c`，是项目为 USB Device Library 提供的底层接口。它初始化 `hpcd_USB_FS`，调用 `HAL_PCD_Init()`，并配置 PMA。

这一函数是第15章最重要的桥：上面连着中间件，下面连着 HAL PCD 和 USB FS 外设。
同一处初始化还把 `hpcd_USB_FS.Init.low_power_enable` 设置为 `DISABLE`，并关闭 LPM 和 battery charging。
因此，虽然 `HAL_PCD_SuspendCallback()` 中存在低功耗条件分支，但当前仓库配置不能直接推出 USB suspend 会让 MCU 进入深睡眠。

### 5. `USBD_Start()`

`USBD_Start()` 来自 `usbd_core.c`，底层会进入 `USBD_LL_Start()`。

项目提供的 `USBD_LL_Start()` 调用 `HAL_PCD_Start(pdev->pData)`，HAL PCD 再启用 USB 设备控制器并执行 `USB_DevConnect()`。

这一步是“设备端开始连接”的代码证据。它不能替代主机侧枚举证据，也不能证明 CDC 虚拟串口已经可用。

在当前 F1 USB FS 底层实现中，`USB_DevConnect()` 对 `USB_TypeDef` 路径只是兼容性函数。
函数体没有切换项目 GPIO，也没有在用户代码区控制外部上拉。
因此如果主机完全没有检测到设备，不能只盯着 `USBD_Start()`，
还必须检查板级连接、PA11/PA12、USB 线缆、供电和主机端口【待验证】。

### 6. `HAL_PCD_MspInit()`

`HAL_PCD_Init()` 内部会触发 MSP 初始化。项目在 `HAL_PCD_MspInit()` 中启用 USB 外设时钟并打开 USB 中断。

如果只看 `MX_USB_DEVICE_Init()`，读者会以为 USB 初始化只有 USBD 函数；追踪到 MSP 后才能看到硬件资源的准备过程。

### 7. `HAL_PCDEx_PMAConfig()`

`USBD_LL_Init()` 中多次调用 `HAL_PCDEx_PMAConfig()`，为不同端点地址配置 PMA 偏移。

本章只说明 PMA 是 USB FS 端点数据缓冲区，确认项目确实做了端点缓冲配置。每个端点与 CDC 类请求、发送和接收之间的详细关系留到第16章。

端点地址中的 `0x80` 位表示 IN 方向。
因此 `0x00` 和 `0x80` 是 EP0 的 OUT/IN 两个方向，
`0x01` 和 `0x81` 是 1 号端点的 OUT/IN 两个方向。
这能帮助读者理解为什么 PMA 配置看起来像“五个端点地址”，
实际上包含控制端点两个方向和 CDC 类相关端点。

还要把 PMA 和中间件内存分配分开看。
`usbd_conf.h` 把 `USBD_malloc` 映射到 `USBD_static_malloc()`，`usbd_conf.c` 中该函数返回一个静态数组。
这说明 USB Device 栈不会在这里走普通堆分配，但它也不是 PMA 端点缓冲。
静态分配对象服务于中间件和类驱动上下文，应用层 CDC RX/TX 缓冲仍留到第16章分析。

### 8. PCD 回调桥

`USB_DEVICE/Target/usbd_conf.c` 中的 PCD 回调把硬件事件送回 USB Device Library：

- setup 阶段回调调用 `USBD_LL_SetupStage()`。
- data out 阶段回调调用 `USBD_LL_DataOutStage()`。
- data in 阶段回调调用 `USBD_LL_DataInStage()`。
- reset 回调调用 `USBD_LL_SetSpeed()` 和 `USBD_LL_Reset()`。
- SOF、suspend、resume、connect 和 disconnect 回调也分别转交给 USBD LL 函数。

这说明 USB 中断并不是在 `stm32f1xx_it.c` 中直接完成协议处理。
真正的事件路径是 IRQ 入口到 HAL PCD，再到 PCD 回调，最后进入 USB Device Library。

从枚举角度看，最值得打断点的回调顺序通常是：

`HAL_PCD_ResetCallback()` -> `USBD_LL_Reset()` -> `HAL_PCD_SetupStageCallback()` -> `USBD_LL_SetupStage()`

随后主机的标准请求会进入 `USBD_StdDevReq()`。
如果 reset 回调不出现，问题还在连接、时钟、中断或主机检测之前。
如果 reset 出现但 setup 不出现，问题可能在 EP0 或主机后续请求阶段【待验证】。

### 9. `Debug/objects.list`

`Debug/objects.list` 是构建证据。它证明 USB Device 中间件核心、CDC 类、应用层 USB 文件和目标适配文件都参与链接。

这能避免一种常见误判：源码目录存在并不等于功能参与构建。当前项目通过 objects list 可以确认 USB 栈相关对象进入了 Debug 构建。

### 10. `.map` 与 `.list` 的证据边界

`Debug/objects.list` 只能证明目标文件参与构建，不能证明每个函数都进入最终镜像。第15章判断 USB Device 栈是否真正接入时，还要继续看 `Debug/Three-axis_cloud_platformV2.map` 和 `Debug/Three-axis_cloud_platformV2.list`。

`.map` 的最终内存映射区能看到 `MX_USB_DEVICE_Init()`、`USBD_Init()`、`USBD_RegisterClass()`、`USBD_CDC_RegisterInterface()`、`USBD_Start()`、`USBD_LL_Init()`、PCD 回调桥函数、`CDC_Init_FS()` 和 `CDC_Receive_FS()` 等符号具有最终地址。这说明设备栈初始化、CDC 类注册、底层 PCD 连接和 CDC 接收相关回调函数进入了最终 ELF。

`.list` 提供更强的调用证据：`main()` 中存在到 `MX_USB_DEVICE_Init()` 的分支调用；`MX_USB_DEVICE_Init()` 内部依次调用 `USBD_Init()`、`USBD_RegisterClass()`、`USBD_CDC_RegisterInterface()` 和 `USBD_Start()`；`USB_LP_CAN1_RX0_IRQHandler()` 会把中断交给 `HAL_PCD_IRQHandler(&hpcd_USB_FS)`；PCD 处理路径中可以看到 reset、setup、data in/out、SOF 等回调桥的调用点。

`Debug/USB_DEVICE/App/usb_device.su` 和对应 `.cyclo` 文件还能看到 `MX_USB_DEVICE_Init` 的函数级静态资源记录：静态栈使用量为 8 字节，圈复杂度为 5。

`Debug/USB_DEVICE/Target/usbd_conf.su` 和对应 `.cyclo` 文件能覆盖项目提供的 PCD 到 USBD 连接层：

- `HAL_PCD_MspInit` 的静态栈使用量为 24 字节，圈复杂度为 2。
- `USBD_LL_Init` 的静态栈使用量为 16 字节，圈复杂度为 2。
- `USBD_LL_Start` 的静态栈使用量为 24 字节，圈复杂度为 1。
- `HAL_PCD_SetupStageCallback`、`HAL_PCD_DataOutStageCallback` 和 `HAL_PCD_DataInStageCallback` 的静态栈使用量均为 16 字节，圈复杂度均为 1。
- `HAL_PCD_ResetCallback` 的静态栈使用量为 24 字节，圈复杂度为 2。

`Debug/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.su` 和对应 `.cyclo` 文件中，`USBD_Init` 为 24 字节静态栈、圈复杂度 4，`USBD_RegisterClass` 为 24 字节静态栈、圈复杂度 2，`USBD_Start` 为 16 字节静态栈、圈复杂度 1。
`Debug/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.su` 还记录 `USBD_CDC_RegisterInterface` 为 24 字节静态栈、圈复杂度 2。

HAL PCD 层也有对应记录：`Debug/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pcd.su` 中 `HAL_PCD_Init` 为 24 字节静态栈、圈复杂度 8，`HAL_PCD_Start` 为 16 字节静态栈、圈复杂度 2，`HAL_PCD_IRQHandler` 为 40 字节静态栈、圈复杂度 12。

这些 `.su/.cyclo` 条目只能证明当前编译选项下的函数级静态栈和圈复杂度记录。它们不能证明主机已经完成枚举、设备进入 `USBD_STATE_CONFIGURED`、CDC 虚拟串口已在主机侧出现，也不能替代中断嵌套后的完整最坏栈深度分析。若 `.su/.cyclo` 中出现某个函数名，还必须继续结合 `.map` 是否有最终地址和 `.list` 是否有调用点判断其是否形成当前可执行路径。

但同一个 `.map` 也显示 `CDC_Transmit_FS()` 和 `USBD_CDC_TransmitPacket()` 位于地址为 `0x00000000` 的输入段附近，属于当前构建中被丢弃的发送相关函数段。它们只能说明源码和头文件提供了发送接口，不能证明当前项目已经存在 USB CDC 业务发送路径。

CDC 接收路径还要区分“函数表注册”和“直接分支调用”：`USBD_Interface_fops_FS` 中保存了 `CDC_Receive_FS`，`USBD_CDC_RegisterInterface()` 把该函数表写入 `pdev->pUserData`，中间件 DataOut 路径再通过 `Receive` 函数指针回调应用层。`CDC_Receive_FS()` 函数体内则能在 `.list` 中看到重新设置接收缓冲并调用 `USBD_CDC_ReceivePacket()` 的分支调用。

因此，本章的工程结论要分层写：USB Device/CDC 类初始化链路已经进入最终镜像；CDC 接收回调函数表和重新投递接收包的函数体具有构建证据；CDC 主动发送接口在当前构建中没有最终地址，发送业务应留到第16章继续核对。

### 本节证据边界

本节只根据当前仓库说明文件、函数、宏、变量和调用关系。运行时频率、外部硬件表现、主机侧现象、传感器方向、电机响应或真实控制效果仍需调试记录、日志或仓库外实测证据；缺少证据时保持【待验证】。

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
- `usbd_conf.c` 中 `low_power_enable` 是否为 `DISABLE`，避免把 suspend 回调中的低功耗代码误判为默认运行路径。
- `usbd_conf.h` 中 `USBD_malloc` 是否映射到 `USBD_static_malloc()`，并确认它属于中间件内存入口，不属于 PMA。
- `USBD_LL_Start()` 是否调用 `HAL_PCD_Start(pdev->pData)`。
- `HAL_PCD_MspInit()` 是否启用 USB 时钟和中断。
- `stm32f1xx_it.c` 中 USB 中断入口是否调用 `HAL_PCD_IRQHandler(&hpcd_USB_FS)`。
- `USB_LP_CAN1_RX0_IRQHandler()` 的名称是否只是共享向量入口；当前项目是否存在 CAN HAL 模块启用、`MX_CAN_Init()` 或 CAN 业务分发证据。
- `Debug/objects.list` 中是否包含 USBD core、USBD CDC、USB_DEVICE App 和 Target 相关对象。

状态机检查：

- 在 `hUsbDeviceFS.dev_state` 上观察 `USBD_STATE_DEFAULT`、`USBD_STATE_ADDRESSED` 和 `USBD_STATE_CONFIGURED`。
- 在 `HAL_PCD_IRQHandler()` 中观察 `USB_ISTR_RESET`、`USB_ISTR_CTR`、`USB_ISTR_SOF`、`USB_ISTR_ERR` 和 `USB_ISTR_PMAOVR`。
- 在 `USBD_LL_Reset()` 处打断点，确认主机是否触发 USB reset。
- 在 `USBD_LL_SetupStage()` 处打断点，确认是否收到 EP0 setup 请求。
- 在 `USBD_SetAddress()` 和 `USBD_SetConfig()` 处打断点，区分地址阶段和配置阶段。
- 在 `USBD_LL_SOF()` 处观察 SOF 是否出现，但不要把 SOF 出现等同于 CDC 已可用。
- 若 `USB_LP_CAN1_RX0_IRQHandler()` 不触发，应先检查 USB 中断、D+/D- 连接、供电和主机连接状态。

主机侧检查：

- 插入 USB 后，主机是否能识别出一个 USB 设备。
- 设备状态是否真的进入 `USBD_STATE_CONFIGURED`。
- 若主机读取设备状态，应区分 `USBD_SELF_POWERED=1` 的配置返回值和真实板级供电方式。
- 若不能识别，先回到 USB 时钟、D+/D- 配置、中断和 PCD 初始化链路排查。
- 若能识别但不能作为虚拟串口正常使用，再进入第16章检查 CDC 接口、描述符和缓冲区。

当前仓库没有 USB 线缆连接、主机设备管理器截图、枚举日志或抓包证据，因此主机侧枚举结果只能标记为【待验证】。本章只确认工程内部 USB FS 设备和中间件初始化证据。

调试记录建议：

- 记录 USB 48MHz 时钟、PA11/PA12、PCD 初始化、中间件初始化和已链接对象清单。
- 记录 `USBD_Start()`、`HAL_PCD_Start()`、`USB_DevConnect()` 和 `hUsbDeviceFS.dev_state` 的先后关系。
- 主机枚举应单独记录线缆、电源状态、系统日志、设备管理器截图或抓包结果，不能由源码配置直接推出。
- 缺少主机侧记录时，只能证明当前仓库存在 USB Device 初始化链路，不能证明主机已经成功枚举。
- 若枚举失败，应把仓库内配置检查和仓库外连接/主机检查分开记录，避免证据边界混杂。

PMA 与端点检查：

- 记录 `0x00`、`0x80`、`0x81`、`0x01`、`0x82` 的 PMA 偏移。
- 检查 `USB_ISTR_PMAOVR` 是否出现；若出现，说明 USB PMA 或传输处理存在异常风险。
- PMA 偏移和应用 RX/TX 缓冲区分属不同层级，不应混为同一个容量概念。

性能边界检查：

- USB FS 的名义信号速率为 12 Mbit/s，属于协议层指标，不能直接等同于应用层 CDC 吞吐量。
- 当前中间件宏定义给出 `USB_FS_MAX_PACKET_SIZE=64U` 和 `USB_MAX_EP0_SIZE=64U`。
- `usbd_conf.c` 存在 SOF 回调桥，但本章不声称项目业务以 1ms USB 帧为调度主线。
- 真实吞吐量、丢包、主机驱动延迟和 CDC 发送阻塞情况应放到第16章或实测记录中验证。

## 10. 常见问题

### 1. 为什么 USB 需要 48MHz 时钟？

触发条件：读者看到 `.ioc` 中 `RCC.USBFreq_Value=48000000`。

可能原因：USB FS 外设需要符合 USB 全速设备时序的外设时钟。项目通过 PLL 分频为 USB 提供 48MHz。若这个时钟配置错误，USB 设备可能无法稳定枚举。

这说明 USB 章节的前提不只是“有 USB 代码”，还包括时钟树和外设时序。
没有 48MHz 证据，后续枚举和中间件分析就失去基础。

### 2. 为什么 PA11/PA12 不像普通 GPIO 那样在 `gpio.c` 中配置？

触发条件：读者在 `gpio.c` 中找不到 USB_DM/USB_DP 的普通 GPIO 初始化。

可能原因：PA11/PA12 是 USB 外设专用信号，由 USB 外设和 PCD 初始化路径接管。它们的配置证据主要来自 `.ioc` 和 USB PCD 初始化，而不是普通 GPIO 输出配置。

这也是第07章和第15章的分工：第07章讲引脚和 MSP 入口，第15章讲 USB 外设初始化链路。
不能把普通 GPIO 的理解直接套到 USB 专用引脚上。

### 3. `USBD` 和 `PCD` 有什么区别？

触发条件：读者同时看到 `USBD_Init()` 和 `HAL_PCD_Init()`。

可能原因：`USBD` 是 USB Device 中间件层，负责设备栈、类和请求处理；`PCD` 是 HAL 底层驱动层，负责 MCU USB 设备控制器。项目通过 `USBD_LL_Init()` 把两层连接起来。

这层分工决定了本章只能先讲“设备栈和底层驱动如何接上”，不能直接讲主机侧已经看到了什么。
主机枚举结果要到后面的证据里才算完整。

### 4. 为什么 objects list 很重要？

触发条件：读者看到 `Middlewares` 目录后就认为 USB 中间件一定被使用。

可能原因：目录存在只能说明源码在工作树中。`Debug/objects.list` 显示对象文件参与链接，才能证明这些模块进入当前构建。

所以第15章强调的是“构建事实”，不是“文件存在事实”。
这个区别和前面章节的工程构建逻辑完全一致。

### 5. 既然注册了 CDC 类，为什么本章不讲 CDC 收发？

触发条件：`MX_USB_DEVICE_Init()` 中出现 `USBD_CDC_RegisterInterface()`。

可能原因：第15章的正式知识点是 `USB FS设备` 和 `USB Device中间件`。CDC 接口、缓冲区、描述符和虚拟串口行为属于第16章。提前展开会破坏教学顺序。

换句话说，第15章负责“把 USB 设备栈搭起来”，第16章负责“让它具备 CDC 虚拟串口接口”。

### 6. USB 初始化成功是否等于项目已经通过 USB 和上位机通信？

触发条件：读者看到 `USBD_Start()`。

可能原因：`USBD_Start()` 只能说明设备栈被启动。是否成功枚举、是否出现虚拟串口、是否有项目级协议解析，需要主机侧证据和第16章接口分析。当前仓库未提供主机枚举日志，标记为【待验证】。

也就是说，USB 设备启动只是“开始工作”，不是“工作结果已验证”。
主机设备管理器、系统日志和抓包记录才是后面那一步的证据。

### 7. 为什么 `hUsbDeviceFS.dev_state` 很适合作为调试观察点？

触发条件：读者需要区分设备栈初始化、主机地址分配和主机配置完成。

可能原因：`USBD_Init()` 和 reset 阶段会进入 `USBD_STATE_DEFAULT`。
`SET_ADDRESS` 后进入 `USBD_STATE_ADDRESSED`，`SET_CONFIGURATION` 后才进入 `USBD_STATE_CONFIGURED`。

如果设备一直停在 default，通常说明主机 reset 或 setup 阶段没有继续推进。
如果停在 addressed，说明地址阶段已发生，但配置阶段仍未完成。
这些判断仍然需要结合主机侧日志，不能仅凭源码静态推断。

### 8. 为什么 `USB_DevConnect()` 返回成功仍不能证明主机识别了设备？

触发条件：读者看到 `HAL_PCD_Start()` 内部调用 `USB_DevConnect()`。

可能原因：当前 STM32F1 USB FS 底层的 `USB_DevConnect()` 对 `USB_TypeDef` 路径只是兼容性占位函数。
它没有在项目中切换一个可观察 GPIO，也没有提供主机侧日志。

因此 `USBD_Start()`、`HAL_PCD_Start()` 和 `USB_DevConnect()` 只能证明软件启动链路走到设备端连接语义。
主机是否检测到 D+ 全速连接、是否发出 reset、是否进入 setup 阶段，仍需中断断点、主机日志或抓包证明。

### 9. 为什么 PMA 偏移不能和 CDC 的 1024 字节缓冲区混为一谈？

触发条件：读者同时看到 `HAL_PCDEx_PMAConfig()` 和 `UserRxBufferFS` / `UserTxBufferFS`。

可能原因：PMA 是 USB FS 外设内部端点缓冲布局，应用 RX/TX 缓冲区是 C 代码中的普通内存对象。
两者通过 PCD 和 USBD 类驱动转接，但不是同一个存储区域。

所以 `0x18`、`0x58`、`0xC0` 这类 PMA 偏移不表示应用缓冲区大小。
应用缓冲区、端点最大包长和 PMA 布局需要分层理解。

### 10. 为什么 SOF 出现不等于 CDC 数据已经可收发？

触发条件：读者在 `USBD_LL_SOF()` 或 `HAL_PCD_SOFCallback()` 上看到中断活动。

可能原因：SOF 是 USB 全速帧节奏事件，说明主机帧调度已经到达设备侧。
但 CDC 数据回调通常还要求设备进入 `USBD_STATE_CONFIGURED`，并且类端点已经完成配置。

因此 SOF 是有用的调试信号，却不是 CDC 虚拟串口可用性的充分证据。

### 11. 为什么 `USBD_SELF_POWERED=1` 不能直接证明目标板是自供电？

触发条件：读者在 `usbd_conf.h` 中看到 `USBD_SELF_POWERED`。

可能原因：USB Device Library 会在 `GET_STATUS` 请求处理中根据这个宏设置 `USB_CONFIG_SELF_POWERED` 状态位。
这能证明固件配置会向主机声明自供电属性，但不能证明目标板实际供电路径、VBUS 检测和线缆供电关系。

因此它是一个需要硬件原理图和实测配合解释的配置项。
在没有板级证据时，只能写成“当前仓库配置声明为自供电”，不能写成“硬件已经验证为自供电”。

## 11. 实践任务

开始任务前，先回到本章第8节定位 USB Device 初始化、中间件到 PCD 的连接和构建对象证据；第9节提供仓库内证据与主机侧实测的分层验证顺序。

任务一至任务九属于仓库内 USB Device 证据；任务十属于仓库外主机侧验证设计。

任务一：确认 USB FS 基础配置。

在 `.ioc` 中找到 USB 48MHz、PA11/PA12 和 CDC_FS 配置。
验收依据是记录 USB 时钟、D-/D+ 引脚和 CDC_FS 配置三项来源。

任务二：定位 USB 初始化入口。

在 `main.c` 中找到 `MX_USB_DEVICE_Init()` 的调用位置。
验收依据是记录它在 `main()` 初始化序列中的位置，并标明它不在 500Hz 控制循环内。

任务三：画出 USB Device 初始化顺序。

在 `usb_device.c` 中追踪 `USBD_Init()`、`USBD_RegisterClass()`、`USBD_CDC_RegisterInterface()` 和 `USBD_Start()`。
验收依据是初始化顺序图包含这四个函数，并按源码调用顺序排列。

任务四：追踪中间件到底层 PCD。

在 `usbd_conf.c` 中找到 `USBD_LL_Init()`、`HAL_PCD_Init()` 和 `HAL_PCDEx_PMAConfig()`。
验收依据是表格能把 `USBD_LL_Init()`、`HAL_PCD_Init()`、PMA 配置分别对应到中间件、PCD 和缓冲区配置层级。

任务五：确认 USB 中断入口。

在 `stm32f1xx_it.c` 中找到 USB 中断入口对 `HAL_PCD_IRQHandler(&hpcd_USB_FS)` 的调用。
验收依据是记录中断入口名称、调用的 HAL 处理函数和 `hpcd_USB_FS` 三项证据。

任务六：确认 USB 对象参与构建。

在 `Debug/objects.list` 中列出 USB Device Library 和 USB_DEVICE 相关对象。
验收依据是至少列出一个 Middlewares 对象和一个 USB_DEVICE 对象，并标明它们来自最终链接清单。

任务七：确认 USB 枚举状态机代码路径。

在 `usbd_core.c` 和 `usbd_ctlreq.c` 中找到默认状态、地址状态和配置状态的设置位置。
验收依据是记录 `USBD_STATE_DEFAULT`、`USBD_STATE_ADDRESSED` 和 `USBD_STATE_CONFIGURED` 分别由哪些函数推进。

任务八：确认 PCD 中断事件分派。

在 `stm32f1xx_hal_pcd.c` 中找到 `HAL_PCD_IRQHandler()` 对 `CTR`、`RESET`、`WKUP`、`SUSP`、`SOF`、`PMAOVR` 和 `ERR` 的处理。
验收依据是能说明 reset、setup/data 传输和 SOF 分别从哪些中断分支进入回调。

任务九：整理 PMA 端点偏移表。

在 `usbd_conf.c` 中列出 `HAL_PCDEx_PMAConfig()` 的端点地址、缓冲模式和 PMA 偏移。
验收依据是能解释 `0x80` 方向位，以及 PMA 偏移和应用层 CDC 缓冲区的区别。

任务十：整理主机侧枚举验证项。

整理 USB 线缆、目标板供电、主机设备管理器或系统日志、USB 抓包等仓库外实测证据。
验收依据是主机侧验证表至少包含线缆/供电、设备管理器或系统日志、抓包记录三类证据；未实测项标记为【待验证】。

## 12. 思考题

1. 为什么 USB FS 设备章节必须依赖系统时钟树？
2. 如果 USB 时钟不是 48MHz，设备枚举可能出现什么现象？
3. 为什么 `USBD_LL_Init()` 要同时设置 `hpcd_USB_FS.pData` 和 `pdev->pData`？
4. 为什么不能只凭 `USBD_Start()` 就断定 USB 虚拟串口业务已经可用？
5. 如果主机不能识别 USB 设备，应该优先检查第15章中的哪些证据？
6. 为什么第15章要用 `Debug/objects.list` 证明中间件参与构建？
7. 如何区分“USB 中间件参与构建”、“设备栈启动”和“主机侧枚举成功”这三层证据？
8. `USBD_STATE_ADDRESSED` 和 `USBD_STATE_CONFIGURED` 在调试含义上有什么区别？
9. 为什么 USB FS 名义速率不能直接写成项目 CDC 应用层吞吐量？
10. 为什么 `USB_DevConnect()` 在当前 F1 FS 路径下不能作为主机已检测到设备的证据？
11. `USB_ISTR_CTR`、`USB_ISTR_RESET` 和 `USB_ISTR_SOF` 分别说明 USB 事件链推进到哪一层？
12. 为什么 PMA 端点缓冲和 CDC 应用缓冲需要分层记录？
13. 为什么 `USBD_SELF_POWERED=1` 只能作为固件配置声明，不能替代硬件供电证据？
14. 为什么 `low_power_enable = DISABLE` 时，不能把 suspend 回调中的深睡眠代码写成默认行为？

## 13. 本章总结

本章建立了三轴云台项目中 USB FS 设备和 USB Device 中间件的证据链。

已经确认的结论是：

- `.ioc` 配置 USB Device CDC_FS、PA11/PA12、USB 48MHz 时钟和 USB 中断。
- `main.c` 在启动初始化阶段调用 `MX_USB_DEVICE_Init()`。
- `usb_device.c` 通过 `USBD_Init()`、`USBD_RegisterClass()`、`USBD_CDC_RegisterInterface()` 和 `USBD_Start()` 搭建设备栈。
- `usbd_conf.c` 通过 `USBD_LL_Init()` 连接 USB Device Library 和 HAL PCD。
- `usbd_conf.c` 将 `hpcd_USB_FS` 配置为 USB FS、全速模式，并配置端点 PMA。
- `USBD_Start()` 经过 `USBD_LL_Start()` 和 `HAL_PCD_Start()` 启动底层设备控制器。
- `USB_DevConnect()` 在当前 F1 FS 底层路径下不能单独证明主机检测到设备。
- EP0 setup、`SET_ADDRESS` 和 `SET_CONFIGURATION` 共同决定设备是否进入 configured 状态。
- USB 中断入口把事件交给 `HAL_PCD_IRQHandler(&hpcd_USB_FS)`。
- `USB_LP_CAN1_RX0_IRQHandler()` 是 USB LP 与 CAN1 RX0 共享向量名；当前项目证据只支持 USB PCD 分发，不支持 CAN 接收业务已经接入。
- `HAL_PCD_IRQHandler()` 会分派 `CTR`、`RESET`、`WKUP`、`SUSP`、`SOF`、`PMAOVR` 和 `ERR` 等事件。
- PCD 回调桥把 reset、setup、data in/out 和 SOF 等事件转交给 USB Device Library。
- `HAL_PCDEx_PMAConfig()` 为 EP0 和 CDC 相关端点分配 PMA 偏移，但 PMA 不是应用层 CDC 缓冲区。
- `USBD_malloc` 映射到 `USBD_static_malloc()`，这是 USB Device 中间件内存入口，不是 PMA 端点缓冲区。
- `USBD_SELF_POWERED=1` 只证明当前固件配置会声明自供电状态，不证明板级供电方式已经验证。
- `low_power_enable = DISABLE` 说明 suspend 回调中的深睡眠分支不是当前默认启用路径。
- `Debug/objects.list` 证明 USB Device Library 核心、CDC 类和 USB_DEVICE 适配文件参与当前 Debug 构建。
- `Debug/Three-axis_cloud_platformV2.map` 与 `.list` 进一步证明 USB Device 初始化、CDC 类注册、PCD 回调桥和 CDC 接收相关函数进入最终镜像；同时 `CDC_Transmit_FS()` 与 `USBD_CDC_TransmitPacket()` 当前处于 `0x00000000` 输入段，不能写成已经存在 CDC 主动发送业务路径。
- 当前 Debug `.su/.cyclo` 文件显示：`MX_USB_DEVICE_Init` 为 8 字节静态栈、圈复杂度 5；`USBD_LL_Init` 为 16 字节静态栈、圈复杂度 2；`HAL_PCD_IRQHandler` 为 40 字节静态栈、圈复杂度 12。它们属于静态构建资源记录，不证明 USB 主机侧枚举成功。

本章边界：

- 本章证明 USB Device 栈配置、初始化和中间件构建证据，不证明主机侧枚举已经成功。
- `USBD_STATE_CONFIGURED` 需要主机侧枚举过程推进，当前仓库缺少运行证据，标记为【待验证】。
- 线缆、供电、D+/D- 板级连接、主机检测、SOF 连续性、PMA 异常、真实吞吐量和自供电声明是否符合板级硬件均需实测。
- CDC 描述符、主动发送接口和业务协议解析需要在第16章继续区分；第15章只确认 CDC 类初始化、接收函数表和接收函数体进入当前镜像。

下一章可以进入 USB CDC 接口与描述符。第15章已经建立了设备栈框架，第16章才能在此基础上分析 CDC 接口、描述符、唯一序列号和静态内存分配。

---

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
- `Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c`
- `Middlewares/ST/STM32_USB_Device_Library/Core/Inc/usbd_def.h`
- `Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c`
- `Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pcd.c`
- `Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_usb.c`
- `Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_pcd.h`
- `Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_ll_usb.h`
- `Debug/objects.list`
- `Debug/Three-axis_cloud_platformV2.map`
- `Debug/Three-axis_cloud_platformV2.list`
- `Debug/USB_DEVICE/App/usb_device.su`
- `Debug/USB_DEVICE/App/usb_device.cyclo`
- `Debug/USB_DEVICE/Target/usbd_conf.su`
- `Debug/USB_DEVICE/Target/usbd_conf.cyclo`
- `Debug/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.su`
- `Debug/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.cyclo`
- `Debug/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.su`
- `Debug/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.cyclo`
- `Debug/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pcd.su`
- `Debug/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pcd.cyclo`
- `Debug/sources.mk`
- `Debug/makefile`

权威参考资料：

- ST RM0008，STM32F101xx/102xx/103xx/105xx/107xx Reference Manual。
- ST UM1734，STM32Cube USB Device Library User Manual。
- USB-IF Universal Serial Bus 2.0 Specification。

引用的函数、配置项和变量：

- `MX_USB_DEVICE_Init()`
- `USBD_Init()`
- `USBD_RegisterClass()`
- `USBD_CDC_RegisterInterface()`
- `USBD_Start()`
- `USBD_LL_Init()`
- `USBD_LL_Start()`
- `USBD_LL_Reset()`
- `USBD_LL_SetupStage()`
- `USBD_CDC_SetTxBuffer()`
- `USBD_CDC_SetRxBuffer()`
- `USBD_CDC_ReceivePacket()`
- `USBD_CDC_TransmitPacket()`
- `CDC_Init_FS()`
- `CDC_Receive_FS()`
- `CDC_Transmit_FS()`
- `HAL_PCD_Init()`
- `HAL_PCD_Start()`
- `HAL_PCD_MspInit()`
- `HAL_PCDEx_PMAConfig()`
- `HAL_PCD_IRQHandler()`
- `HAL_PCD_SetupStageCallback()`
- `HAL_PCD_DataOutStageCallback()`
- `HAL_PCD_DataInStageCallback()`
- `HAL_PCD_SOFCallback()`
- `HAL_PCD_ResetCallback()`
- `HAL_PCD_SuspendCallback()`
- `HAL_PCD_ResumeCallback()`
- `USB_DevConnect()`
- `USB_DevDisconnect()`
- `USB_ReadInterrupts()`
- `USBD_SetAddress()`
- `USBD_SetConfig()`
- `USBD_StdDevReq()`
- `USBD_CtlError()`
- `USBD_static_malloc()`
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
- `USB_FS_MAX_PACKET_SIZE`
- `USB_MAX_EP0_SIZE`
- `USB_ISTR_CTR`
- `USB_ISTR_RESET`
- `USB_ISTR_SOF`
- `USB_ISTR_ERR`
- `USB_ISTR_PMAOVR`
- `USB_REQ_SET_ADDRESS`
- `USB_REQ_SET_CONFIGURATION`
- `USBD_STATE_DEFAULT`
- `USBD_STATE_ADDRESSED`
- `USBD_STATE_CONFIGURED`
- `USBD_SELF_POWERED`
- `USBD_malloc`
- `USBD_free`
- `hpcd_USB_FS.Init.low_power_enable`
- `BTABLE_ADDRESS`
- `PMA_ACCESS`
- `PCD_SNG_BUF`

质量自检：

- P0 事实错误：通过。
- P1 依赖断层：通过。
- P2 逻辑连贯：通过。
- P3 项目证据：通过。
- P4 原理展开：通过。
- P5 调试实践：通过。
- P6 表达统一：通过。

---
> 导航：上一章：[第14章_I2C主机通信](第14章_I2C主机通信.md) ｜ 下一章：[第16章_USB CDC接口与描述符](第16章_USB CDC接口与描述符.md)
