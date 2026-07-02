# 第16章 USB CDC接口与描述符

> 导航：上一章：[第15章_USB FS设备与中间件](第15章_USB FS设备与中间件.md) ｜ 下一章：[第17章_项目数据结构与配置对象](第17章_项目数据结构与配置对象.md)

## 1. 本章目标

- 理解 USB CDC 虚拟串口在项目 USB Device 栈中的位置。
- 看懂 `usb_device.c` 如何注册 CDC 类和 `USBD_Interface_fops_FS`。
- 能追踪 `usbd_cdc_if.c` 中 CDC 初始化、接收、发送和缓冲区配置。
- 能理解 `usbd_conf.c/h` 中 USB 静态内存分配的项目作用。
- 能定位 `usbd_desc.c` 中 VID、PID、字符串描述符和唯一序列号生成路径。
- 明确当前项目 USB CDC 属于已初始化通信接口，未发现业务协议解析主线。

本章阅读分层：

| 阅读层次 | 建议范围 | 适合读者 |
|---|---|---|
| 【必须掌握】 | 第1节到第5节，第13节总结 | 需要理解 USB CDC 接口框架、静态内存、收发入口和描述符主线的读者 |
| 【工程深化】 | 第6节到第8.13节，第9节调试方法 | 需要维护 CDC 回调、端点、line coding、发送状态和唯一序列号路径的读者 |
| 【拓展阅读】 | 第6.7节到第6.8节，第10节到第12节 | 需要进一步理解 VID/PID、self-powered 声明、包长、`USBD_BUSY` 和主机枚举差异的读者 |
| 【证据与验证】 | 第8节、第9节、章节尾部固定检查，以及所有 `【待验证】` 项 | 需要审查静态内存钩子、CDC类注册、收发回调、函数表、描述符路径、构建产物、主机侧枚举证据、USB 抓包或业务协议边界的读者 |

如果只是沿 USB 通信接口主线学习，可以先抓住“静态内存钩子 -> CDC 类注册 -> 收发回调 -> 描述符与序列号”这条链；判断是否真的完成主机通信或业务协议时，再回到构建证据、调试方法和待验证项。

本章速查：

| 查找目标 | 优先阅读 | 避免重复展开 |
|---|---|---|
| USB CDC接口框架、静态内存和类注册主线 | 第1节到第5节、第6.1节到第6.3节、第13节 | USB FS设备和中间件框架回到第15章 |
| CDC接收、发送、line coding和状态边界 | 第6.4节到第6.6节、第8.4节到第8.10节、第9节 | USART `printf()` 主线回到第11章，不把 CDC 写成当前主调试输出 |
| 描述符、VID/PID、self-powered和唯一序列号 | 第6.7节到第6.8节、第8.11节到第8.12节、第10节 | CMSIS唯一ID地址回到第05章，硬件供电方式仍按【待验证】处理 |
| 构建产物、函数表和业务协议证据边界 | 第8.13节到第8.13.2节、章节尾部固定检查 | 不把“收发回调存在”误读为“已实现上位机业务协议” |

## 2. 前置知识

- 链接脚本与内存布局
- CMSIS寄存器与内核访问
- USB FS设备
- USB Device中间件

第03章已经说明 MCU 的 RAM 和运行时内存边界，第05章已经说明 CMSIS 如何暴露芯片唯一 ID 地址，第15章已经建立 USB FS 设备、PCD 和 USB Device 中间件框架。本章在这些基础上进入 USB CDC 接口、静态内存和描述符。

本章不把 USB CDC 写成当前 `printf()` 主输出路径。第11章已经确认 `printf()` 主线是 USART3。本章也不引入上位机协议解析，因为当前仓库没有发现 USB 接收数据被解析成项目参数或控制命令的证据。

## 3. 问题背景

第15章已经说明项目启动了 USB Device 栈，但“USB Device 已启动”不等于“项目已经通过 USB 完成业务通信”。USB Device 栈还需要三个关键部分才能被主机识别并提供 CDC 虚拟串口接口：

1. USB Device 中间件运行时对象需要内存。
2. CDC 类需要接口回调、收发缓冲区和收发函数。
3. 主机枚举时需要设备描述符、字符串描述符和序列号。

项目中的证据对应三组文件：

- `USB_DEVICE/Target/usbd_conf.c` 和 `USB_DEVICE/Target/usbd_conf.h`：提供 USB 中间件静态内存分配钩子。
- `USB_DEVICE/App/usb_device.c` 与 `USB_DEVICE/App/usbd_cdc_if.c`：注册 CDC 接口并提供收发入口。
- `USB_DEVICE/App/usbd_desc.c` 与 `USB_DEVICE/App/usbd_desc.h`：提供设备描述符、字符串描述符和基于唯一 ID 的序列号。

本章要解决的问题是：读者如何从项目源码判断 USB CDC 已经具备接口框架，同时又不把它误判为已经承载项目业务协议。

## 4. 核心概念

- USB静态内存分配：USB Device 中间件需要类运行时对象，本项目用静态数组返回对象内存，而不是调用通用堆分配。
- `USBD_malloc`：USB Device Library 的分配宏，本项目映射到 `USBD_static_malloc()`。
- CDC：Communication Device Class，USB 通信设备类。本项目使用它提供虚拟串口接口。
- CDC ACM：Abstract Control Model，是常见虚拟串口模型。项目 CDC 配置描述符中通信接口类为 `0x02`，子类为 `0x02`。
- CDC接口回调：CDC 类驱动调用的应用侧函数集合，本项目对象为 `USBD_Interface_fops_FS`。
- `pUserData`：USB Device 句柄中保存 CDC 应用回调表的位置，由 `USBD_CDC_RegisterInterface()` 写入。
- `pClassData`：USB Device 句柄中保存 CDC 类运行时对象的位置，由 `USBD_CDC_Init()` 分配并使用。
- 接收缓冲区：本项目 `UserRxBufferFS`，大小由 `APP_RX_DATA_SIZE` 定义为 1024。
- 发送缓冲区：本项目 `UserTxBufferFS`，大小由 `APP_TX_DATA_SIZE` 定义为 1024。
- CDC数据端点：项目 CDC 类使用 `CDC_OUT_EP=0x01` 和 `CDC_IN_EP=0x81` 传输数据，端点类型为 bulk。
- CDC命令端点：项目 CDC 类使用 `CDC_CMD_EP=0x82` 传输类通知，端点类型为 interrupt，包长为 8。
- CDC功能描述符：CDC ACM 配置描述符中的类专用描述符，例如 Header、Call Management、ACM 和 Union，用来说明通信接口与数据接口之间的关系。
- 类控制请求：主机通过 EP0 发送给 CDC 类的请求，例如 line coding 和 control line state；它不同于 bulk 数据端点承载的业务字节流。
- `TxState`：CDC 类运行时对象中的发送状态。非零表示上一笔发送尚未完成。
- `RxState`：CDC 类运行时对象中的接收状态字段，当前 ST CDC 类结构体保留该字段，但项目接收路径主要通过 OUT 端点回调和重新准备接收推进。
- OUT回调占用窗口：`CDC_Receive_FS()` 运行期间，后续 OUT 包会受 USB CDC 回调路径影响；当前模板注释明确提醒该函数退出前会让后续 OUT 包处于 NAK 等待状态。
- USB描述符：主机枚举 USB 设备时读取的一组结构化信息，包括设备、配置、接口和字符串描述符。
- VID/PID：USB 设备的厂商 ID 和产品 ID，本项目在 `usbd_desc.c` 中定义。
- 模板设备身份：当前 `USBD_VID=1155` 即 `0x0483`，`USBD_PID_FS=22336` 即 `0x5740`，属于 ST 生成模板常见虚拟串口身份；它不能证明项目已经拥有独立产品 USB 身份。
- self-powered描述符位：配置描述符中的 `bmAttributes=0xC0` 声明设备自供电；这只是描述符声明，真实供电方式仍需硬件原理图或实测记录【待验证】。
- MaxPower字段：USB 配置描述符中的最大总线取电字段以 2mA 为单位；当前 `0x32` 对应 100mA 的描述符值，不等于实测电流。
- 唯一序列号：项目通过 STM32 唯一 ID 地址 `DEVICE_ID1/2/3` 生成 USB 字符串序列号。

这些概念服务于正式知识点 `USB静态内存分配`、`USB CDC虚拟串口` 和 `USB描述符与唯一序列号`，不新增结构外知识点。

## 5. 工作原理

第16章内部有明确的顺序。

第一步是静态内存分配。USB Device 中间件在运行 CDC 类时需要保存类状态。

项目在 `usbd_conf.h` 中把 `USBD_malloc` 映射到 `USBD_static_malloc()`。

`usbd_conf.c` 用静态数组 `mem` 返回一块 `USBD_CDC_HandleTypeDef` 大小的内存。

`USBD_static_free()` 是空函数，说明这条路径不是通用动态内存管理，而是为 USB Device 类对象准备的固定分配方式。
更准确地说，它不是“静态内存池”，而是“单次固定对象分配入口”。
当前配置只有一个 CDC 类对象，所以这个实现可以覆盖项目现状；如果未来叠加多个 USB 类或多个类运行时对象，就不能直接假设这个函数仍然足够。

第二步是 CDC 接口注册。

`usb_device.c` 中 `MX_USB_DEVICE_Init()` 先注册 `USBD_CDC` 类。

随后它调用 `USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS)`。

`USBD_Interface_fops_FS` 定义在 `usbd_cdc_if.c`，包含四个回调入口：

- `CDC_Init_FS()`
- `CDC_DeInit_FS()`
- `CDC_Control_FS()`
- `CDC_Receive_FS()`

这一步不是打开端点，而是把应用侧回调表保存到 `pdev->pUserData`。
CDC 类真正初始化时，会通过这个指针回调 `CDC_Init_FS()`、`CDC_Control_FS()` 和 `CDC_Receive_FS()`。

第三步是 CDC 收发。

`CDC_Init_FS()` 设置默认发送和接收缓冲区。

`CDC_Receive_FS()` 在收到数据后重新设置接收缓冲，并调用 `USBD_CDC_ReceivePacket()` 继续接收。

`CDC_Transmit_FS()` 检查 `hcdc->TxState`，若正在发送则返回 `USBD_BUSY`；否则设置发送缓冲并调用 `USBD_CDC_TransmitPacket()`。
这一句描述的是源码中提供的发送包装逻辑，不等于该函数已经进入当前 Debug 最终镜像；是否保留和是否被业务调用，要以后文构建产物和调用点证据为准。

接收路径还有一个容易被忽略的实时性边界。
`usbd_cdc_if.c` 的模板注释说明，`CDC_Receive_FS()` 退出前，后续 OUT 包会被 NAK 等待。
因此未来若在这个回调里直接做命令解析、浮点计算或阻塞输出，会把 USB 接收窗口拉长。
更稳妥的扩展方式是先复制 `Buf/Len` 到项目命令缓冲，再由主循环或低优先级任务解析。

从 CDC 类驱动内部看，收发还要经过端点层：

1. 配置成功后，`USBD_CDC_Init()` 打开 `CDC_IN_EP`、`CDC_OUT_EP` 和 `CDC_CMD_EP`。
2. `USBD_CDC_Init()` 用 `USBD_malloc(sizeof(USBD_CDC_HandleTypeDef))` 分配类运行时对象。
3. `USBD_CDC_DataOut()` 取得 OUT 端点收到的数据长度，然后调用应用层 `Receive()` 回调。
4. `USBD_CDC_TransmitPacket()` 在 `TxState == 0` 时置位 `TxState` 并发起 IN 端点传输。
5. `USBD_CDC_DataIn()` 在发送完成后清除 `TxState`，下一次发送才不再返回 busy。

第四步是描述符。

`usb_device.c` 中 `USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS)` 把描述符集合交给 USB Device 栈。

`FS_Desc` 定义在 `usbd_desc.c`，它包含设备描述符、语言 ID 字符串、厂商字符串、产品字符串、序列号字符串、配置字符串和接口字符串的函数指针。

CDC 的 FS 配置描述符本身也可以继续拆开。

`USB_CDC_CONFIG_DESC_SIZ=67U` 不是一个随意数字，而是下列描述符长度之和：

`9 + 9 + 5 + 5 + 4 + 5 + 7 + 9 + 7 + 7 = 67`

对应关系是：

- 配置描述符 9 字节。
- 通信接口描述符 9 字节。
- Header 功能描述符 5 字节。
- Call Management 功能描述符 5 字节。
- ACM 功能描述符 4 字节。
- Union 功能描述符 5 字节。
- 命令 interrupt IN 端点描述符 7 字节。
- 数据接口描述符 9 字节。
- 数据 OUT bulk 端点描述符 7 字节。
- 数据 IN bulk 端点描述符 7 字节。

这能帮助读者看懂“CDC 虚拟串口”不是单个端点或单个字符串，
而是一组配置、接口、类专用功能描述符和端点描述符共同组成的 USB 功能。

这些机制共同说明：USB CDC 接口已经具备枚举、接口回调、收发入口和运行时对象分配基础。

不过，描述符只能让主机知道“这个设备声明自己是什么”。主机是否绑定 CDC/ACM 驱动、是否出现串口节点、串口工具是否能打开，都需要主机侧证据。

但当前源码没有发现 `Core` 或 `Drivers` 中调用 `CDC_Transmit_FS()`。

也没有发现 `CDC_Receive_FS()` 对收到的数据做业务解析。

因此，本章只能确认接口框架存在，不能写成项目已经通过 USB CDC 进行控制参数通信。

## 6. STM32实现机制

### 1. 静态内存钩子

`USB_DEVICE/Target/usbd_conf.h` 中定义：

- `USBD_MAX_NUM_INTERFACES = 1`
- `USBD_MAX_NUM_CONFIGURATION = 1`
- `USBD_MAX_STR_DESC_SIZ = 512`
- `USBD_malloc` 映射到 `USBD_static_malloc`
- `USBD_free` 映射到 `USBD_static_free`
- `USBD_Delay` 映射到 `HAL_Delay`

`USB_DEVICE/Target/usbd_conf.c` 中 `USBD_static_malloc(uint32_t size)` 没有使用 `size` 做通用分配，而是返回一个静态数组：

- 类型为 `uint32_t`
- 大小按 `sizeof(USBD_CDC_HandleTypeDef)` 换算
- 注释说明按 32 位边界对齐

这说明项目采用固定内存策略，避免 USB Device 中间件运行时依赖堆分配。

### 2. CDC 接口对象

`USB_DEVICE/App/usbd_cdc_if.c` 中定义：

- `UserRxBufferFS[APP_RX_DATA_SIZE]`
- `UserTxBufferFS[APP_TX_DATA_SIZE]`
- `USBD_Interface_fops_FS`

`APP_RX_DATA_SIZE` 和 `APP_TX_DATA_SIZE` 都是 1024。`USBD_Interface_fops_FS` 把 CDC 类驱动需要的应用回调指向当前文件中的四个静态函数。

这组对象不是主循环业务对象，而是 USB CDC 类驱动和应用层之间的适配层。

注册动作发生在 `USBD_CDC_RegisterInterface()` 中。该函数并不复制回调函数本身，而是把 `USBD_Interface_fops_FS` 的地址保存到 `pdev->pUserData`。

因此，`pUserData` 可以理解为“CDC 类驱动找到项目应用回调表的指针”。

### 3. CDC 类端点

`Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc/usbd_cdc.h` 中定义：

- `CDC_IN_EP = 0x81`
- `CDC_OUT_EP = 0x01`
- `CDC_CMD_EP = 0x82`
- `CDC_DATA_FS_MAX_PACKET_SIZE = 64U`
- `CDC_CMD_PACKET_SIZE = 8U`

`usbd_cdc.c` 的 FS 配置描述符显示，CDC 由两个接口组成：

- 通信接口：接口类 `0x02`，子类 `0x02`，使用一个 interrupt IN 命令端点。
- 数据接口：接口类 `0x0A`，使用一个 bulk OUT 端点和一个 bulk IN 端点。

通信接口下面还带有四个 CDC 类专用功能描述符：

- Header Functional Descriptor：声明 CDC 规范版本。
- Call Management Functional Descriptor：说明呼叫管理能力和关联的数据接口。
- ACM Functional Descriptor：说明 ACM 能力位。
- Union Functional Descriptor：把通信接口 `0` 和数据接口 `1` 组织成同一个 CDC 功能。

这解释了为什么第15章看到 PMA 配置里有 `0x81`、`0x01` 和 `0x82` 三个 CDC 相关端点。
CDC 虚拟串口不是只靠一个函数完成，而是由控制接口、数据接口和三个非 EP0 端点共同支撑。

从控制流看，CDC 类请求和数据流也要分开。

`USBD_CDC_Setup()` 处理 EP0 上的类请求。
若请求方向是设备到主机，它会调用 `Control()` 填充 `hcdc->data`，
再通过 `USBD_CtlSendData()` 回传。
若请求方向是主机到设备，它会先记录 `CmdOpCode` 和 `CmdLength`，
再通过 `USBD_CtlPrepareRx()` 等待 EP0 数据阶段，
随后由 `USBD_CDC_EP0_RxReady()` 调用 `Control()`。

`USBD_CDC_DataOut()` 处理 bulk OUT 数据端点。它读取 OUT 端点接收长度，再调用 `Receive()`。

因此，line coding 这类控制请求走 EP0 控制传输，用户输入的数据字节走 `CDC_OUT_EP` bulk OUT 传输。
二者都可能最终回到 `usbd_cdc_if.c`，但语义完全不同。

### 4. CDC 接收路径

`CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)` 的当前实现很克制：

- 调用 `USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0])`
- 调用 `USBD_CDC_ReceivePacket(&hUsbDeviceFS)`
- 返回 `USBD_OK`

它没有解析 `Buf` 内容，没有使用 `Len` 做协议分发，也没有修改项目配置对象。因此当前项目不能被写成“USB 接收上位机指令并控制云台”。如果未来要扩展 USB 协议，`CDC_Receive_FS()` 才是自然入口之一。

从 CDC 类驱动看，OUT 端点数据先进入 `USBD_CDC_DataOut()`。
该函数用 `USBD_LL_GetRxDataSize()` 得到 `RxLength`，再调用 `pdev->pUserData` 中的 `Receive()`。

所以 `CDC_Receive_FS()` 不是硬件中断入口，而是 CDC 类把 OUT 端点数据上交给应用层后的回调入口。

还要继续拆一层返回值边界。当前 ST CDC 类的 `USBD_CDC_DataOut()` 调用应用层 `Receive()` 后直接返回 `USBD_OK`，没有把 `Receive()` 的返回值向外传播；而当前 `CDC_Receive_FS()` 调用 `USBD_CDC_ReceivePacket()` 后也没有检查该函数返回值。因此“进入了 `CDC_Receive_FS()`”只能证明 OUT 数据到达应用回调，不能单独证明下一次 OUT 接收已经可靠重新准备好。

### 5. CDC 发送路径

`CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)` 是项目暴露的 USB CDC 发送函数。

它先取得 `hUsbDeviceFS.pClassData` 并转换为 `USBD_CDC_HandleTypeDef*`，然后检查 `TxState`：

- 若 `TxState != 0`，返回 `USBD_BUSY`。
- 否则调用 `USBD_CDC_SetTxBuffer()` 设置待发送数据。
- 再调用 `USBD_CDC_TransmitPacket()` 发起发送。

这说明发送函数在源码层具备非阻塞忙状态判断。可是当前 `Core` 和 `Drivers` 目录没有发现业务代码调用 `CDC_Transmit_FS()`，后文构建产物还显示该函数没有最终链接地址，所以不能把它写成实际运行中的状态输出通道。

还要注意一个工程前置条件：当前 `CDC_Transmit_FS()` 包装函数在读取 `hcdc->TxState` 之前，没有先判断 `hUsbDeviceFS.pClassData` 是否为空。
`pClassData` 由 `USBD_CDC_Init()` 在主机设置配置后分配。
如果未来业务代码在 USB 设备尚未进入 configured 状态、CDC 类尚未初始化时直接调用 `CDC_Transmit_FS()`，它不一定只是返回 `USBD_FAIL`，还可能因为空指针访问导致异常。
因此教材不能把 `CDC_Transmit_FS()` 写成“任意时刻可调用的安全打印函数”。
更稳妥的调用前提是：设备已进入 `USBD_STATE_CONFIGURED`、`hUsbDeviceFS.pClassData` 非空、`TxState==0`。

`USBD_CDC_TransmitPacket()` 内部会在 `TxState == 0` 时把 `TxState` 置为 1，然后调用 `USBD_LL_Transmit()`。

发送完成后，`USBD_CDC_DataIn()` 会把 `TxState` 清回 0。若发送长度正好是最大包长的整数倍，类驱动还可能发送 ZLP。

因此，`USBD_BUSY` 不是普通错误码，而是“上一笔 IN 传输尚未完成”的状态反馈。
若未来把 USB CDC 用作高频日志通道，需要在业务层设计队列、重试或丢弃策略。

### 6. Line coding 与真实 UART 的边界

`CDC_Control_FS()` 中保留了 `CDC_SET_LINE_CODING`、`CDC_GET_LINE_CODING`、`CDC_SET_CONTROL_LINE_STATE` 等分支。

当前项目这些分支基本为空，且没有把 line coding 写入 USART3、控制参数或配置对象。

因此，主机串口工具中设置的波特率、停止位、校验位和数据位，只能被理解为 CDC 控制请求的一部分。
它们不能被写成项目真实 UART 或控制循环参数，除非源码中出现明确映射。

还要注意 `CDC_GET_LINE_CODING` 分支当前没有填充 `pbuf`。
虽然 `USBD_CDC_Setup()` 会在设备到主机方向调用 `Control()` 后再通过 EP0 回传 `hcdc->data`，但项目侧没有写入一组明确的 115200-8N1 或其它默认串口参数。
所以本章不能声称固件已经向主机实现了有意义的 line coding 协商；它只能说明控制请求入口存在。

### 7. 描述符集合

`USB_DEVICE/App/usbd_desc.c` 中定义 `FS_Desc`，它把多个描述符函数注册给 USB Device Library：

- `USBD_FS_DeviceDescriptor`
- `USBD_FS_LangIDStrDescriptor`
- `USBD_FS_ManufacturerStrDescriptor`
- `USBD_FS_ProductStrDescriptor`
- `USBD_FS_SerialStrDescriptor`
- `USBD_FS_ConfigStrDescriptor`
- `USBD_FS_InterfaceStrDescriptor`

设备描述符中包含 VID、PID、设备类、端点 0 最大包大小、厂商字符串索引、产品字符串索引和序列号字符串索引。

字符串配置中，当前项目定义：

- `USBD_MANUFACTURER_STRING = "STMicroelectronics"`
- `USBD_PRODUCT_STRING_FS = "STM32 Virtual ComPort"`
- `USBD_CONFIGURATION_STRING_FS = "CDC Config"`
- `USBD_INTERFACE_STRING_FS = "CDC Interface"`

这些字符串是主机侧看到设备身份信息的来源之一。

VID/PID 也要用数字值和工程含义分开看。
当前 `USBD_VID` 为十进制 `1155`，换算为十六进制是 `0x0483`；`USBD_PID_FS` 为十进制 `22336`，换算为 `0x5740`。
这能说明当前工程沿用 ST 虚拟串口模板身份，但不能证明它已经完成面向产品发布的 USB 身份规划。

`USBD_FS_DeviceDesc` 中 `bDeviceClass=0x02`、`bDeviceSubClass=0x02`，并包含 `USB_MAX_EP0_SIZE`、`USBD_VID`、`USBD_PID_FS` 和字符串索引。

`USBD_CDC_CfgFSDesc` 中包含两个接口、一个 interrupt 命令端点、一个 bulk OUT 数据端点和一个 bulk IN 数据端点。

这说明主机枚举时读取到的不只是产品字符串，还包括 CDC 类、接口数量、端点方向、端点类型和最大包长。

配置描述符中还包含供电相关声明。
当前 `usbd_cdc.c` 的 FS 配置描述符写入 `bmAttributes=0xC0`，含义是配置有效并声明 self-powered。
`usbd_conf.h` 同时定义 `USBD_SELF_POWERED=1`，`usbd_ctlreq.c` 在处理 GET_STATUS 时会据此返回 `USB_CONFIG_SELF_POWERED`。

但是这仍然只是 USB 描述符和中间件状态位。
真实电源来源、是否从 USB VBUS 取电、最大电流是否满足描述符声明，需要看硬件原理图或实测记录；当前仓库不能证明，保持【待验证】。
另外，FS 配置描述符中 `MaxPower=0x32`，按 USB 2.0 配置描述符字段单位计算为 `0x32 * 2mA = 100mA`。
源码注释中的“MaxPower 0 mA”不能覆盖字段本身的规范含义。

### 8. 唯一序列号

`usbd_desc.h` 定义：

- `DEVICE_ID1 = UID_BASE`
- `DEVICE_ID2 = UID_BASE + 0x4`
- `DEVICE_ID3 = UID_BASE + 0x8`
- `USB_SIZ_STRING_SERIAL = 0x1A`

`Get_SerialNum()` 从这三个地址读取 32 位值，将 `deviceserial0` 与 `deviceserial2` 相加。

随后它把 `deviceserial0` 和 `deviceserial1` 通过 `IntToUnicode()` 写入 `USBD_StringSerial`。

`IntToUnicode()` 每次取 4 位十六进制数，转换为 ASCII 字符，并在后一个字节写 0，生成 USB 字符串描述符所需的 Unicode 形式。

这条路径说明序列号来自 MCU 唯一 ID，而不是手写固定字符串。

`Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xe.h` 中定义 `UID_BASE = 0x1FFFF7E8UL`。
这为 `DEVICE_ID1/2/3` 的地址来源提供了本地 CMSIS 证据。

## 7. 项目中的应用

在项目启动阶段，`main()` 调用 `MX_USB_DEVICE_Init()`。第15章已经分析它启动 USB Device 栈；本章进一步看它在 CDC 和描述符层做了什么：

1. `USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS)` 使用 `FS_Desc` 提供描述符集合。
2. `USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC)` 注册 CDC 类。
3. `USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS)` 注册 CDC 应用接口。
4. CDC 类运行时对象通过 `USBD_static_malloc()` 分配。
5. CDC 接口初始化时挂接 `UserTxBufferFS` 和 `UserRxBufferFS`。

当前项目中，USB CDC 的项目定位是“已初始化、可作为扩展通信接口的支线”。它不同于 USART3 调试输出，也不同于 500Hz 实时控制循环。当前仓库没有发现：

- `printf()` 重定向到 USB CDC。
- 主循环调用 `CDC_Transmit_FS()` 输出状态。
- `CDC_Receive_FS()` 解析上位机命令。
- USB 接收数据写入 PID、姿态目标或运行状态。

因此，本章只能确认 USB CDC 接口框架存在，并指出它为后续扩展提供入口。

从工程取舍看，当前项目保留 USB CDC 框架有三个好处：

- 不改变现有 USART3 调试主线。
- 为后续上位机调参或日志输出预留接口。
- 把 USB 相关风险限制在通信支线内，不侵入 500Hz 控制循环。

它的代价是：教材和调试时必须反复区分“接口已经生成”和“业务已经使用”。
如果没有调用点、解析逻辑和主机侧验证，USB CDC 只能算可扩展能力，不能算已生效业务链路。

## 8. 代码分析

第8.1至8.13节把 USB CDC 接口拆成静态内存、接口注册、收发回调、类驱动状态、描述符、序列号和构建产物证据。阅读时可先用下面的速查表区分“接口存在、回调可达、数据收发、业务协议”四类边界，再进入对应小节细看：

| USB CDC 接口边界 | 优先查看小节 |
|---|---|
| 静态内存钩子和 CDC 接口注册 | 8.1 至 8.4 |
| 控制请求、接收回调和发送入口 | 8.5 至 8.7 |
| CDC 类驱动打开端点和 DataOut/DataIn 中间层 | 8.8 至 8.10 |
| 描述符对象、唯一序列号和主机身份信息 | 8.11、8.12 |
| 构建产物、收发路径和业务协议证据边界 | 8.13 |

### 8.1 `USBD_static_malloc()`

该函数位于 `usbd_conf.c`。它返回静态数组 `mem`，数组大小覆盖 `USBD_CDC_HandleTypeDef`。

它的输入参数名是 `size`，但当前实现不根据 `size` 分配不同大小。这是 CubeMX 生成 USB Device 工程中常见的固定类对象分配方式。本章不把它扩展成通用内存管理器。

### 8.2 `USBD_static_free()`

该函数为空实现。由于分配的是静态数组，不需要像堆内存那样释放。

调试时如果看到 `USBD_free`，要记住它在当前项目中只是映射到这个空释放函数。

### 8.3 `USBD_CDC_RegisterInterface()`

该调用位于 `usb_device.c`。它把 `USBD_Interface_fops_FS` 交给 CDC 类驱动，使 CDC 类在初始化、控制请求和接收数据时能回调到应用侧函数。

这一句是 `usb_device.c` 和 `usbd_cdc_if.c` 的连接点。

### 8.4 `CDC_Init_FS()`

该函数设置 CDC 发送和接收缓冲区：

- 发送缓冲区为 `UserTxBufferFS`
- 接收缓冲区为 `UserRxBufferFS`

这一步说明 USB CDC 接口启动时已经具备基本缓冲区。缓冲区大小来自 `APP_TX_DATA_SIZE` 和 `APP_RX_DATA_SIZE`。

### 8.5 `CDC_Control_FS()`

该函数包含 CDC 类控制命令的 switch 分支，例如 `CDC_SET_LINE_CODING`、`CDC_GET_LINE_CODING`、`CDC_SET_CONTROL_LINE_STATE` 等。

当前各分支基本为空，仅返回 `USBD_OK`。因此本项目没有使用这些控制请求来保存波特率、停止位、校验位或数据位配置。对于 USB CDC 虚拟串口来说，主机侧看到的串口参数不等于 MCU 内部真实 USART 参数；当前项目没有把这些参数映射到任何业务外设。

### 8.6 `CDC_Receive_FS()`

该函数收到数据后立即重新挂接接收缓冲并继续接收。它没有复制数据到项目命令缓冲区，也没有解析协议。

当前实现也没有保存 `USBD_CDC_ReceivePacket()` 的返回值。若未来调试连续接收丢包或 OUT 包停止进入回调，应在这里临时记录该返回值、`hUsbDeviceFS.pClassData`、`hUsbDeviceFS.dev_state` 和 `Len`，而不是只看 `CDC_Receive_FS()` 最终返回 `USBD_OK`。

这条事实很关键：有接收回调不等于有上位机命令协议。

### 8.7 `CDC_Transmit_FS()`

该函数检查 `TxState`，避免上一包未发完时继续发新包。返回值可能是 `USBD_OK`、`USBD_FAIL` 或 `USBD_BUSY`。

它是项目未来通过 USB CDC 发数据的入口，但当前源码未发现业务调用。

### 8.8 `USBD_CDC_Init()`

该函数位于 `usbd_cdc.c`。配置完成后，CDC 类通过它打开数据 IN、数据 OUT 和命令 IN 端点。

随后它调用 `USBD_malloc(sizeof(USBD_CDC_HandleTypeDef))` 分配 `pClassData`，再通过 `pdev->pUserData` 调用项目侧 `Init()`。

这说明 `USBD_static_malloc()` 的实际服务对象是 CDC 类运行时状态，而不是任意业务对象。

### 8.9 `USBD_CDC_DataOut()`

该函数是 CDC OUT 数据进入应用回调的中间层。

它先把 OUT 端点收到的字节数写入 `hcdc->RxLength`，然后调用 `USBD_Interface_fops_FS` 中的 `Receive()`，最终进入 `CDC_Receive_FS()`。

这条链路说明：如果未来要加入上位机命令解析，不能只看 `CDC_Receive_FS()`，还要理解 `RxLength` 和 OUT 端点数据到应用回调的传递路径。

### 8.10 `USBD_CDC_DataIn()`

该函数在 IN 端点发送完成后运行。普通情况下它会把 `hcdc->TxState` 清为 0。

这解释了 `CDC_Transmit_FS()` 为什么要先检查 `TxState`：没有发送完成事件之前，再次发送会返回 `USBD_BUSY`。

### 8.11 `FS_Desc`

`FS_Desc` 是 `USBD_DescriptorsTypeDef` 类型对象。`USBD_Init()` 使用它把描述符函数交给 USB Device Library。

主机枚举时，USB Device Library 会通过这些函数取得设备描述符、字符串描述符和序列号描述符。

### 8.12 `Get_SerialNum()`

该函数读取 STM32 唯一 ID，并生成序列号字符串。它不是随机数，也不是固定常量。不同 MCU 理论上应生成不同的序列号字符串，但当前仓库没有主机枚举日志或实际设备验证记录，主机侧显示结果仍标记为【待验证】。

### 8.13 构建产物证据边界

第16章不能只依据源码目录和头文件判断 CDC 接口是否参与最终固件，还要分开看 `.map` 与 `.list`。

`Debug/Three-axis_cloud_platformV2.map` 显示 `USBD_CDC_Init()`、`USBD_CDC_Setup()`、`USBD_CDC_DataIn()`、`USBD_CDC_DataOut()`、`USBD_CDC_RegisterInterface()`、`USBD_CDC_SetTxBuffer()`、`USBD_CDC_SetRxBuffer()`、`USBD_CDC_ReceivePacket()`、`CDC_Init_FS()`、`CDC_Control_FS()` 和 `CDC_Receive_FS()` 都有最终地址。这说明 CDC 类初始化、控制请求入口、收发缓冲挂接和接收回调相关函数进入了最终 ELF。

同一个 `.map` 还显示 `USBD_CDC_CfgFSDesc`、`USBD_CDC`、`USBD_Interface_fops_FS`、`FS_Desc`、`USBD_FS_DeviceDesc`、`USBD_StringSerial`、`UserRxBufferFS`、`UserTxBufferFS` 和 `USBD_StrDesc` 进入数据段或 BSS 区。这说明 CDC 配置描述符、设备描述符、回调表、序列号字符串缓冲和应用层 RX/TX 缓冲都进入当前镜像。

但是 `CDC_Transmit_FS()` 与 `USBD_CDC_TransmitPacket()` 在 `.map` 中位于地址为 `0x00000000` 的输入段附近，当前 `.list` 也没有最终地址处的函数体或业务调用点。因此，本章不能只写“发送函数未被业务调用”，还应更准确地写成：当前 Debug 构建中 CDC 主动发送包装函数和底层发送包函数没有进入最终镜像，USB CDC 发送路径尚未形成可执行业务证据。

`.list` 对接收和控制请求提供了更细的证据：`CDC_Init_FS()` 里能看到对 `USBD_CDC_SetTxBuffer()` 和 `USBD_CDC_SetRxBuffer()` 的分支调用；`CDC_Receive_FS()` 里能看到对 `USBD_CDC_SetRxBuffer()` 和 `USBD_CDC_ReceivePacket()` 的分支调用；`USBD_CDC_Setup()` 中能看到类请求经 `pUserData->Control()` 回到 `CDC_Control_FS()`，并通过 `USBD_CtlSendData()` 或 `USBD_CtlPrepareRx()` 处理 EP0 数据阶段。由于 `CDC_Control_FS()` 的 line coding 分支没有写入明确配置值，这些调用证据仍不能推出固件保存了主机串口参数。

#### 8.13.1 接收、发送与业务协议证据边界

USB CDC 最容易被误读成“能枚举就等于能通信，能通信就等于有业务协议”。本项目必须按路径拆开：

| 路径 | 当前构建证据 | 可以确认 | 仍不能确认 |
|---|---|---|---|
| EP0 控制请求 | `USBD_CDC_Setup()`、`USBD_CDC_EP0_RxReady()`、`CDC_Control_FS()` 有最终地址和 `.list` 分支证据 | CDC 类控制请求入口存在，line coding 等请求能进入项目回调 | 当前分支未保存或填充明确 line coding，不能证明固件维护了串口参数状态 |
| Bulk OUT 接收 | `USBD_CDC_DataOut()`、`CDC_Receive_FS()`、`USBD_CDC_ReceivePacket()` 有最终地址或调用边证据 | OUT 数据可以进入应用层接收回调，并重新准备下一次接收 | `Buf/Len` 未被解析或复制到业务队列，不能证明上位机命令协议已经存在 |
| Bulk IN 主动发送 | `CDC_Transmit_FS()` 与 `USBD_CDC_TransmitPacket()` 只出现在 `0x00000000` 输入段，未见业务调用点 | 源码层保留了未来发送入口和 `TxState` 忙判断设计 | 当前 Debug ELF 没有可执行主动发送主线，不能证明 USB CDC 已承担日志或状态输出 |
| 项目业务协议 | 搜索 `Core`、`Drivers` 和 `USB_DEVICE` 只发现模板收发入口 | 可以确认 USB CDC 框架存在、业务扩展入口明确 | 没有 PID、目标角、配置对象或状态输出与 CDC 数据流相连的证据 |

因此，本章对 USB CDC 的结论必须使用四个不同层级的措辞：描述符可枚举、控制请求可进入、OUT 数据可回调、业务协议已生效。当前仓库只能证明前三者中的一部分静态路径；最后一层仍缺少解析逻辑、调用点、主机日志和运行记录【待验证】。

#### 8.13.2 函数表证据断点

`USBD_Interface_fops_FS` 是一个很容易被过度解释的证据点。它在 `usbd_cdc_if.c` 中保存 `CDC_Init_FS`、`CDC_DeInit_FS`、`CDC_Control_FS` 和 `CDC_Receive_FS` 四个函数入口；`USBD_CDC_RegisterInterface()` 再把这个函数表地址写入 `pdev->pUserData`。这能证明应用层 CDC 回调已经注册到 USB Device 栈，但不能证明每个回调都已经被主机触发。

更细地说，`.list` 中能看到 `USBD_CDC_Setup()` 和 `USBD_CDC_EP0_RxReady()` 通过 `pUserData->Control()` 回到 `CDC_Control_FS()`，也能看到 `USBD_CDC_DataOut()` 通过 `pUserData->Receive()` 回到 `CDC_Receive_FS()`。这些属于“类驱动到应用回调的可执行边”证据。它们仍然不能替代三类更高层证据：主机是否真的发起对应请求、回调内部是否解析业务数据、解析结果是否写入 PID/目标角/配置对象。

所以，调试 USB CDC 时应把函数表分成三层记录：函数表对象是否进入 `.map`，类驱动是否在 `.list` 中通过 `pUserData` 调用它，应用回调是否真正处理了项目业务。当前仓库只支持前两层；第三层仍保持【待验证】。

`Debug/USB_DEVICE/App/usbd_cdc_if.su` 和对应 `.cyclo` 文件还能给出应用侧 CDC 回调的静态资源记录：

- `CDC_Init_FS` 的静态栈使用量为 8 字节，圈复杂度为 1。
- `CDC_DeInit_FS` 的静态栈使用量为 4 字节，圈复杂度为 1。
- `CDC_Control_FS` 的静态栈使用量为 16 字节，圈复杂度为 1。
- `CDC_Receive_FS` 的静态栈使用量为 16 字节，圈复杂度为 1。
- `CDC_Transmit_FS` 的静态栈使用量为 24 字节，圈复杂度为 2。

其中 `CDC_Transmit_FS` 要特别分层解释：`.su/.cyclo` 记录说明该目标文件中生成过函数级静态资源信息，但当前 `.map/.list` 没有给出它进入最终镜像并被业务调用的证据。因此它仍只能作为源码层发送入口和构建中间证据，不能写成当前 USB CDC 主动发送路径已经生效。

`Debug/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.su` 和对应 `.cyclo` 文件覆盖 CDC 类驱动层：

- `USBD_CDC_Init` 的静态栈使用量为 24 字节，圈复杂度为 4。
- `USBD_CDC_Setup` 的静态栈使用量为 32 字节，圈复杂度为 12。
- `USBD_CDC_DataIn` 的静态栈使用量为 24 字节，圈复杂度为 4。
- `USBD_CDC_DataOut` 的静态栈使用量为 24 字节，圈复杂度为 2。
- `USBD_CDC_EP0_RxReady` 的静态栈使用量为 24 字节，圈复杂度为 3。
- `USBD_CDC_RegisterInterface` 的静态栈使用量为 24 字节，圈复杂度为 2。

`Debug/USB_DEVICE/App/usbd_desc.su` 和对应 `.cyclo` 文件覆盖描述符与序列号生成层：

- `USBD_FS_DeviceDescriptor` 的静态栈使用量为 16 字节，圈复杂度为 1。
- `USBD_FS_ProductStrDescriptor` 的静态栈使用量为 16 字节，圈复杂度为 2。
- `USBD_FS_SerialStrDescriptor` 的静态栈使用量为 16 字节，圈复杂度为 1。
- `Get_SerialNum` 的静态栈使用量为 24 字节，圈复杂度为 2。
- `IntToUnicode` 的静态栈使用量为 32 字节，圈复杂度为 3。

`Debug/USB_DEVICE/Target/usbd_conf.su` 还记录 `USBD_static_malloc` 和 `USBD_static_free` 的静态栈使用量均为 16 字节，圈复杂度均为 1。

这些 `.su/.cyclo` 条目只能证明当前 Debug 编译过程中生成了函数级静态栈和圈复杂度记录。它们不能证明主机已经绑定 CDC/ACM 驱动、虚拟串口已经出现、USB OUT 回调能连续稳定触发、USB IN 发送路径已经进入最终镜像，或业务协议已经生效。运行时和主机侧结论仍需枚举日志、串口工具记录、USB 抓包或断点记录；缺少证据时保持【待验证】。

## 9. 调试方法

USB CDC 调试应先区分“设备能枚举”“虚拟串口能出现”“项目业务能通信”三个层级。

本节按统一调试结构组织：现象 -> 可能原因 -> 定位方法 -> 验证步骤 -> 解决方案 -> 经验总结。对第16章而言，最重要的是不要把“USB CDC框架存在”直接写成“主机侧通信已经可用”。

### 9.1 现象与可能原因

常见现象包括：主机没有枚举设备、主机有设备但没有虚拟串口、`pClassData` 为空、`CDC_Transmit_FS()` 返回 `USBD_BUSY`、`CDC_Receive_FS()` 没有触发、接收回调触发但业务参数没有变化。可能原因包括描述符或端点配置不一致、CDC 回调表没有注册、设备尚未进入 configured 状态、发送忙状态没有处理、接收回调只重挂缓冲而没有解析业务协议，或缺少主机侧枚举/抓包证据。

### 9.2 定位方法：静态检查

- `usbd_conf.h` 中 `USBD_malloc` 是否映射到 `USBD_static_malloc`。
- `usbd_conf.c` 中 `USBD_static_malloc()` 是否返回 `USBD_CDC_HandleTypeDef` 大小的静态数组。
- `usb_device.c` 中是否调用 `USBD_CDC_RegisterInterface()`。
- `usbd_cdc_if.c` 中是否定义 `USBD_Interface_fops_FS`。
- `usbd_cdc_if.c` 中 `APP_RX_DATA_SIZE` 和 `APP_TX_DATA_SIZE` 是否为 1024。
- `CDC_Receive_FS()` 是否只做缓冲重挂和继续接收，是否存在耗时解析或阻塞操作。
- `usbd_cdc.h` 中 `CDC_IN_EP`、`CDC_OUT_EP`、`CDC_CMD_EP` 和 FS 最大包长是否与描述符一致。
- `usbd_desc.c` 中产品字符串是否为 `STM32 Virtual ComPort`。
- `usbd_desc.c` 中 `USBD_VID=1155` 和 `USBD_PID_FS=22336` 是否仍是模板身份值。
- `usbd_desc.c` 中序列号是否由 `DEVICE_ID1/2/3` 生成。

### 9.3 验证步骤：运行检查

- 主机能否枚举出 USB 设备。
- 主机是否出现 CDC 虚拟串口。
- `hUsbDeviceFS.dev_state` 是否进入 `USBD_STATE_CONFIGURED`。
- `USBD_CDC_Init()` 是否执行，`hUsbDeviceFS.pClassData` 是否非空。
- 调用 `CDC_Transmit_FS()` 时是否返回 `USBD_BUSY`。
- `USBD_CDC_DataIn()` 是否在发送完成后清除 `TxState`。
- `CDC_Receive_FS()` 是否被触发。
- `CDC_Receive_FS()` 中 `Len` 是否等于预期接收字节数。
- 调用 `CDC_Transmit_FS()` 前，`hUsbDeviceFS.pClassData` 是否已经非空。
- CDC 控制请求是否进入 `USBD_CDC_Setup()` / `USBD_CDC_EP0_RxReady()`，数据请求是否进入 `USBD_CDC_DataOut()`。
- `CDC_GET_LINE_CODING` 是否填充返回数据；当前源码未填充时，不要把主机串口参数写成固件已保存配置。
- 配置描述符中的 `bmAttributes=0xC0`、`MaxPower=0x32` 是否与硬件供电设计一致【待验证】。

### 9.4 验证步骤：项目业务检查

- 当前 `Core` 和 `Drivers` 目录是否存在 `CDC_Transmit_FS()` 调用。
- `CDC_Receive_FS()` 是否解析 `Buf` 和 `Len`。
- USB 接收数据是否进入 PID、目标角、运行状态或配置对象。

当前仓库没有主机枚举日志、虚拟串口截图、USB 抓包或上位机通信记录，因此主机侧表现只能标记为【待验证】。源码层面只能确认 USB CDC 接口框架存在，未确认业务协议存在。

### 9.5 解决方案：调试记录

- 记录静态内存分配、CDC 回调表、RX/TX 缓冲区、描述符字符串和序列号来源。
- 记录 `pUserData` 指向回调表、`pClassData` 指向 CDC 类对象这两个不同指针。
- 记录 CDC 三个非 EP0 端点：`0x81`、`0x01`、`0x82`。
- 主机 VCOM 结论应单独记录枚举截图、串口节点、抓包或主机日志，缺失时标记为【待验证】。
- 项目业务通信应记录 `CDC_Transmit_FS()` 调用点、`CDC_Receive_FS()` 解析逻辑和控制参数写入路径。
- 如果搜索不到业务调用或解析路径，应写成“框架存在、业务协议未确认”，不能写成“USB 通信已完成”。
- 若准备在业务代码中调用 `CDC_Transmit_FS()`，先记录 configured 状态、`pClassData` 非空和 busy 处理策略。
- 若验证主机串口参数，应区分 line coding 控制请求是否到达，以及项目是否真正使用这些参数。

### 9.6 经验总结：性能记录

- `APP_RX_DATA_SIZE` 和 `APP_TX_DATA_SIZE` 是应用侧缓冲区大小，不是 USB 单包大小。
- `CDC_DATA_FS_MAX_PACKET_SIZE=64U` 是 FS 数据端点最大包长。
- `CDC_Transmit_FS()` 没有队列；`USBD_BUSY` 出现时，调用方必须决定重试、排队还是丢弃。
- `CDC_Receive_FS()` 运行在 USB CDC 回调路径上。若未来加入复杂解析，宜先复制数据，再在主循环或低优先级任务中处理。
- `CDC_Receive_FS()` 退出前后续 OUT 包可能处于 NAK 等待状态；解析越重，越容易增加接收停顿风险。
- 当前模板没有检查 `USBD_CDC_ReceivePacket()` 的返回值，连续接收问题需要额外记录重新准备接收是否成功。
- `USB_CDC_CONFIG_DESC_SIZ=67U` 属于枚举描述符长度，不是运行时数据缓冲容量。
- `MaxPower=0x32` 是描述符声明值，不能代替实际电流测量。

## 10. 常见问题

### 1. 为什么 USB Device 中间件不用普通 `malloc()`？

触发条件：读者看到 `USBD_malloc` 被映射到 `USBD_static_malloc`。

可能原因：嵌入式工程常避免在运行时依赖堆分配。本项目只需要 CDC 类对象的固定内存，因此使用静态数组即可满足当前 USB Device 中间件需求。

这不是说堆分配一定不行，而是当前项目没有必要把 USB 中间件对象放到不确定的运行时堆里。
教材要让读者看见“工程选择”，不是背一条绝对规则。

### 2. `MAX_STATIC_ALLOC_SIZE` 是不是实际分配大小？

触发条件：读者在 `usbd_conf.h` 中看到 `MAX_STATIC_ALLOC_SIZE = 512`。

可能原因：当前 `USBD_static_malloc()` 的实际返回数组按 `sizeof(USBD_CDC_HandleTypeDef)` 定义，而不是按 `MAX_STATIC_ALLOC_SIZE` 创建。教材以当前函数实现为准。

这说明宏定义和真正分配尺寸并不总是一回事。
写教材时必须跟着实际代码走，而不能只看宏名猜用途。

### 3. 为什么 `CDC_Control_FS()` 里很多分支是空的？

触发条件：读者看到 line coding 相关命令但没有处理逻辑。

可能原因：当前项目没有把 USB CDC 的串口参数同步到真实 UART 或业务配置。对于 USB 虚拟串口接口，主机控制请求可以存在，但项目不一定使用这些参数。

也就是说，接口协议和业务协议是两层东西。
接口收到了控制请求，不代表项目已经拿它做了配置联动。

### 4. 有 `CDC_Transmit_FS()`，为什么说 USB 不是当前调试输出主线？

触发条件：读者把存在发送函数等同于正在使用。

可能原因：函数存在只是可调用入口。当前 `Core` 和 `Drivers` 中没有发现业务调用 `CDC_Transmit_FS()`；第11章已经确认 `printf()` 经 USART3 输出。

所以本章只能把 USB CDC 定位为“可扩展接口”，不能写成“当前状态输出通道”。
这也是章节边界的一部分。

### 5. 有 `CDC_Receive_FS()`，为什么不能说项目支持上位机控制？

触发条件：读者看到 USB 接收回调。

可能原因：当前 `CDC_Receive_FS()` 只重新挂接接收缓冲并继续接收，没有解析 `Buf`，也没有写入项目控制参数。上位机控制需要额外协议解析证据。

换句话说，接收回调存在，只能说明 USB 通道可接收数据。
是否真的把数据变成控制命令，还要看后面有没有解析和写参。

### 6. 产品字符串 `STM32 Virtual ComPort` 代表什么？

触发条件：主机枚举时显示类似虚拟串口名称。

可能原因：该字符串来自 `USBD_PRODUCT_STRING_FS`。它说明设备描述符把产品标识为虚拟串口，但不证明项目业务层已经使用 USB 通信。

主机看到什么名字，取决于描述符。
项目有没有用 USB 做业务通信，还要看发送、接收和协议处理路径。

### 7. 序列号为什么要读 `DEVICE_ID1/2/3`？

触发条件：读者看到 `Get_SerialNum()` 直接读取地址。

可能原因：这些宏基于 STM32 唯一 ID 地址。项目用唯一 ID 生成 USB 序列号，使不同芯片可以在主机侧具有不同设备实例标识。实际主机侧显示结果需要硬件枚举证据，当前标记为【待验证】。

这能帮助主机区分不同板子，但前提仍是先成功枚举。
没有主机侧证据时，只能写“序列号生成机制存在”，不能写“已经在主机侧确认唯一标识正确”。

### 8. 为什么 CDC 有 1024 字节缓冲区，仍然要关心 64 字节包长？

触发条件：读者看到 `APP_RX_DATA_SIZE=1024`、`APP_TX_DATA_SIZE=1024` 和 `CDC_DATA_FS_MAX_PACKET_SIZE=64U`。

可能原因：1024 字节是项目应用侧缓冲区规模，64 字节是 FS bulk 数据端点单包最大长度。
两者不在同一层级。应用可以准备较大的缓冲区，但 USB 总线仍按端点包长、主机调度和类驱动状态分批传输。

所以不能用 1024 字节缓冲区直接推导单次 USB 包长，也不能用 64 字节包长直接推导项目吞吐量。

### 9. 为什么 `USBD_BUSY` 需要业务层处理策略？

触发条件：调用 `CDC_Transmit_FS()` 时返回 `USBD_BUSY`。

可能原因：`USBD_CDC_TransmitPacket()` 发起 IN 传输后会置位 `TxState`，`USBD_CDC_DataIn()` 完成回调后才清零。

如果业务层在上一包完成前继续发送，就会遇到 busy。当前项目没有 USB 发送队列，因此不能假定所有日志或数据都会自动排队。
未来扩展时要明确选择重试、缓存、限频或丢弃策略。

### 10. 为什么 `CDC_Transmit_FS()` 不能被当成任意时刻安全可用的 `printf()` 后端？

触发条件：读者准备把 `printf()` 或高频日志直接改到 USB CDC。

可能原因：当前包装函数先把 `hUsbDeviceFS.pClassData` 转成 `USBD_CDC_HandleTypeDef*`，随后直接读取 `TxState`。
而 `pClassData` 由 `USBD_CDC_Init()` 在配置阶段分配。
如果主机尚未完成配置，`pClassData` 可能为空。

所以把它作为日志后端前，至少要设计三个保护：确认设备 configured、确认 `pClassData` 非空、确认 `TxState==0` 或具备队列/限频策略。
当前仓库没有这些业务调用和保护代码，因此只能把 USB CDC 写成扩展入口。

### 11. 为什么描述符里写了 self-powered 仍要标记硬件供电为【待验证】？

触发条件：读者看到 `bmAttributes=0xC0` 或 `USBD_SELF_POWERED=1`。

可能原因：这些值说明设备在 USB 协议层声明 self-powered，并在 GET_STATUS 中返回对应状态位。
它们不是硬件原理图，也不是电流测量记录。

因此本章可以确认“描述符和中间件声明了 self-powered”，不能确认目标板实际供电方式和取电能力。
`MaxPower=0x32` 也只是描述符字段，按 2mA 单位表示 100mA，不代表实测电流。

### 12. 为什么 `CDC_GET_LINE_CODING` 分支为空会影响结论？

触发条件：读者看到主机串口工具可以设置波特率、停止位、校验位和数据位。

可能原因：这些设置通过 CDC 类控制请求进入 `CDC_Control_FS()`。
当前项目的 `CDC_GET_LINE_CODING` 和 `CDC_SET_LINE_CODING` 分支没有保存或填充 line coding 数据。

因此不能把主机串口工具中的参数写成固件已经接收、保存或应用。
当前仓库只能证明 CDC 控制请求入口存在，不能证明 line coding 协商有项目级含义。

### 13. 为什么 `CDC_Receive_FS()` 中不适合直接放重解析？

触发条件：未来想把 USB CDC 扩展为上位机命令接口。

可能原因：当前模板注释说明，`CDC_Receive_FS()` 退出前后续 OUT 包会被 NAK 等待。
如果在这个回调中直接进行耗时解析、打印或阻塞等待，会延长 USB OUT 接收路径占用时间。

所以更好的工程分层是：在回调里复制 `Buf/Len` 并快速重新准备接收；在主循环、低频任务或专门通信任务中解析命令。
当前项目尚未实现这层命令缓冲和解析，因此不能写成已具备上位机协议。

### 14. 为什么当前 VID/PID 不能直接写成项目产品身份？

触发条件：读者看到 `USBD_VID`、`USBD_PID_FS` 和产品字符串。

可能原因：当前值换算为 `VID=0x0483`、`PID=0x5740`，同时 Manufacturer/Product 字符串仍是 ST 虚拟串口模板风格。
这能证明固件会以这组描述符向主机声明身份，但不能证明项目已经完成独立 USB 产品身份分配或发布侧 USB 身份配置。

因此教材只能把它写成当前工程描述符配置，不能把它写成面向量产的 USB 身份设计。

## 11. 实践任务

开始任务前，先回到本章第8节定位静态内存、CDC 回调表、描述符和序列号生成证据；第9节提供 USB CDC 分层验证顺序。

任务一至任务九属于仓库内 CDC 与描述符证据；任务十属于仓库外主机侧验证分层。

任务一：确认 USB Device 静态限制。

在 `usbd_conf.h` 中找到 `USBD_malloc`、`USBD_free`、`USBD_MAX_NUM_INTERFACES` 和 `USBD_MAX_NUM_CONFIGURATION`。
验收依据是记录内存分配宏、接口数量上限和配置数量上限三类限制。

任务二：分析静态分配函数。

在 `usbd_conf.c` 中找到 `USBD_static_malloc()` 和 `USBD_static_free()`。
验收依据是记录静态缓冲区名称、分配函数返回值和释放函数行为，结论标明它不是通用堆分配。

任务三：追踪 CDC 接口注册。

在 `usb_device.c` 中找到 `USBD_CDC_RegisterInterface()`。
验收依据是记录 `USBD_CDC_RegisterInterface()` 的实参，并对应到 `USBD_Interface_fops_FS`。

任务四：画出 CDC 回调表。

在 `usbd_cdc_if.c` 中找到 `UserRxBufferFS`、`UserTxBufferFS` 和 `USBD_Interface_fops_FS`。
验收依据是回调表至少列出 Init、DeInit、Control、Receive 四个入口及其对应函数。

任务五：比较 CDC 接收和发送边界。

在 `usbd_cdc_if.c` 中比较 `CDC_Receive_FS()` 和 `CDC_Transmit_FS()`。
验收依据是记录接收函数当前处理动作、发送函数调用搜索结果，并标明当前未形成业务协议证据。
附加要求：记录 `CDC_Receive_FS()` 模板注释中的 NAK 提示，并说明为什么未来协议解析应快速复制后延后处理。

任务六：拆分 CDC 端点和接口。

在 `usbd_cdc.h` 和 `usbd_cdc.c` 中找到 `CDC_IN_EP`、`CDC_OUT_EP`、`CDC_CMD_EP` 和 FS 配置描述符。
验收依据是表格列出通信接口、数据接口、命令端点、数据 OUT 端点、数据 IN 端点和各自类型。

附加要求：把 `USB_CDC_CONFIG_DESC_SIZ=67U` 拆成 `9+9+5+5+4+5+7+9+7+7`，并说明每一段对应的描述符类型。

任务七：追踪 `TxState` 忙状态。

在 `usbd_cdc_if.c` 和 `usbd_cdc.c` 中追踪 `CDC_Transmit_FS()`、`USBD_CDC_TransmitPacket()` 和 `USBD_CDC_DataIn()`。
验收依据是画出 `TxState` 从 0 到 1 再回到 0 的路径，并说明 `USBD_BUSY` 的含义。
同时记录 `CDC_Transmit_FS()` 调用前的 `pClassData` 非空前提。

任务八：确认描述符字符串来源。

在 `usbd_desc.c` 中找到 VID、PID、Manufacturer、Product、Configuration 和 Interface 字符串。
验收依据是描述符表包含 VID、PID、Manufacturer、Product、Configuration、Interface 六项来源。
附加要求：把十进制 `1155/22336` 换算为十六进制 `0x0483/0x5740`，并标明它们是当前工程描述符值，不是独立产品身份验证。

附加要求：在 `usbd_cdc.c` 和 `usbd_conf.h` 中记录 `bmAttributes=0xC0`、`MaxPower=0x32` 和 `USBD_SELF_POWERED=1`，并把硬件供电真实性标记为【待验证】。

任务九：追踪唯一序列号生成。

在 `usbd_desc.c/h` 中追踪 `DEVICE_ID1/2/3`、`Get_SerialNum()` 和 `IntToUnicode()`。
验收依据是序列号链路图包含 `DEVICE_ID1/2/3`、`Get_SerialNum()` 和 `IntToUnicode()` 三个节点。

任务十：整理 USB CDC 主机侧验证表。

整理“设备枚举”“虚拟串口出现”“CDC 收发回调触发”“项目业务协议生效”四个层级。
验收依据是验证表分成四行：设备枚举、虚拟串口出现、CDC 收发回调触发、项目业务协议生效；缺少主机截图、串口工具记录或抓包时，对应结论保持【待验证】。

## 12. 思考题

1. 为什么 USB 静态内存分配要排在 CDC 接口分析之前？
2. 如果 `CDC_Transmit_FS()` 返回 `USBD_BUSY`，说明发送路径处于什么状态？
3. 为什么 `CDC_Receive_FS()` 被调用并不等于项目已经有 USB 上位机协议？
4. USB CDC 的 line coding 参数为什么不能直接理解为 USART3 参数？
5. 为什么描述符能影响主机识别结果，却不等于项目业务通信已经完成？
6. 如果要把 USB CDC 扩展成调参接口，应该优先在哪个文件增加解析入口？
7. 如果仓库中存在 `CDC_Transmit_FS()`，还需要哪些调用证据才能说明 USB CDC 已经承担当前调试输出主线？
8. `pUserData` 和 `pClassData` 分别保存什么？为什么不能混淆？
9. CDC 的通信接口、数据接口和三个端点分别承担什么职责？
10. 若将 USB CDC 用作高频日志输出，为什么必须设计 busy 处理策略？
11. `USB_CDC_CONFIG_DESC_SIZ=67U` 可以拆成哪些描述符长度？这种拆分能防止什么误解？
12. 为什么 `MaxPower=0x32` 应解释为描述符字段，而不能直接写成实测电流？
13. 为什么 line coding 控制请求到达不等于项目使用了真实 UART 参数？
14. 为什么 `CDC_GET_LINE_CODING` 分支为空时，不能声称固件已经返回明确串口参数？
15. 为什么 `CDC_Receive_FS()` 回调里应避免耗时解析？
16. 为什么当前 `VID=0x0483`、`PID=0x5740` 只能作为模板描述符配置，而不是项目产品身份结论？

## 13. 本章总结

本章建立了三轴云台项目中 USB 静态内存、CDC 接口和 USB 描述符之间的证据链。

已经确认的结论是：

- `usbd_conf.h` 将 `USBD_malloc` 和 `USBD_free` 映射到静态分配/释放函数。
- `usbd_conf.c` 使用静态数组为 `USBD_CDC_HandleTypeDef` 提供内存。
- `usb_device.c` 注册 `USBD_CDC` 类，并把 `USBD_Interface_fops_FS` 注册为 CDC 应用接口。
- `USBD_CDC_RegisterInterface()` 将 CDC 应用回调表保存到 `pUserData`。
- `USBD_CDC_Init()` 打开 CDC 数据 IN、数据 OUT 和命令 IN 端点，并分配 `pClassData`。
- `usbd_cdc_if.c` 提供 1024 字节接收缓冲和 1024 字节发送缓冲。
- `CDC_Init_FS()` 挂接默认收发缓冲。
- `CDC_Receive_FS()` 当前只继续接收，不解析业务协议。
- `CDC_Receive_FS()` 回调退出前后续 OUT 包可能被 NAK 等待，未来扩展协议时应快速复制后延后解析。
- 当前 `CDC_Receive_FS()` 没有检查重新准备 OUT 接收的返回值，因此一次回调命中不能单独证明后续连续接收链路可靠。
- `CDC_Transmit_FS()` 具备发送入口和忙状态判断，但当前未发现业务调用；构建产物还显示它和 `USBD_CDC_TransmitPacket()` 未进入最终镜像。
- `USBD_CDC_DataIn()` 完成发送后清除 `TxState`，`USBD_BUSY` 代表上一笔发送尚未完成。
- `usbd_desc.c` 定义 VID、PID 和多个字符串描述符。
- 当前 VID/PID 为 `0x0483/0x5740`，字符串仍是 ST 虚拟串口模板风格，不能直接写成项目独立产品身份。
- CDC FS 配置描述符包含通信接口、数据接口、interrupt 命令端点、bulk OUT 端点和 bulk IN 端点。
- `USB_CDC_CONFIG_DESC_SIZ=67U` 可拆为配置、接口、CDC 功能描述符和端点描述符的长度之和。
- CDC 类控制请求走 EP0 控制传输，CDC 数据字节流走 bulk OUT/IN 数据端点。
- `Get_SerialNum()` 通过 STM32 唯一 ID 生成 USB 序列号字符串。
- `bmAttributes=0xC0`、`USBD_SELF_POWERED=1` 和 `MaxPower=0x32` 只是描述符/协议层声明，硬件供电真实性仍需验证。
- `.map/.list/.su/.cyclo` 的构建产物结论统一回到第13节判断：它们能证明 CDC 类函数、描述符对象、回调表、RX/TX 缓冲、字符串缓冲、接收回调、EP0 控制请求路径、静态栈和圈复杂度条目进入某次 Debug 构建；但 `CDC_Transmit_FS()` 与 `USBD_CDC_TransmitPacket()` 当前缺少最终镜像和业务调用证据，也不能替代主机侧 CDC 通信成功记录。

本章待验证分类：

| 类别 | 已由本章证明 | 仍保持【待验证】 |
|---|---|---|
| 构建验证 | `.map/.list/.su/.cyclo` 能证明 CDC 类函数、描述符对象、回调表、RX/TX 缓冲、字符串缓冲、接收回调、EP0 控制请求路径、静态栈和圈复杂度条目进入某次 Debug 构建。 | `CDC_Transmit_FS()` 与 `USBD_CDC_TransmitPacket()` 当前缺少最终镜像和业务调用证据，不能替代主机侧 CDC 通信成功记录。 |
| 软件验证 | 本章能证明 USB CDC 接口、描述符、静态内存路径、`CDC_Receive_FS()`、`CDC_Transmit_FS()` 和 `CDC_Control_FS()` 的源码入口存在。 | USB CDC 未被证明承担当前调试输出主线；`CDC_Receive_FS()` 当前没有业务协议解析证据。 |
| 参数验证 | VID/PID、CDC FS 配置描述符长度、接口/端点组合、`bmAttributes=0xC0`、`USBD_SELF_POWERED=1`、`MaxPower=0x32` 和序列号生成路径已经按描述符口径列清。 | 描述符声明不能证明项目独立产品身份、硬件供电真实性、主机串口参数或最终通信策略。 |
| 硬件验证 | 仓库内 USB CDC 代码能证明设备侧描述符和接口路径。 | 主机是否成功绑定 CDC/ACM 驱动、出现虚拟串口、完成收发、供电声明真实有效和硬件链路稳定仍需主机侧与板上证据。 |
| 官方资料待确认 | CDC 类控制请求、EP0 控制传输、bulk IN/OUT 数据端点和 line coding 响应入口已按 USB CDC 结构区分。 | 当前没有保存或返回明确 line coding 数据；主机串口参数、busy 策略和日志后端安全性仍需结合 USB CDC/HAL 资料确认。 |
| 实验待完成 | 本章已经把描述符、控制请求、收发接口、日志后端和主机侧验证拆成可检查对象。 | 后续需记录 configured、`pClassData`、busy 状态、主机枚举、虚拟串口出现、收发日志、line coding 行为和业务调用路径。 |

下一章可以进入项目数据结构与配置对象。到这里，USB 支线已经完成：教材确认了 USB Device 初始化、CDC 接口和描述符，但也明确当前项目主调试输出仍是 USART3，USB CDC 未被证明承担业务协议解析。

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

- USB静态内存分配
- USB CDC虚拟串口
- USB描述符与唯一序列号

项目证据：

以下清单用于追溯 USB Device 入口、CDC 回调、描述符、静态内存、类驱动、唯一 ID 地址和构建产物证据来源，建议按“应用层回调 -> 描述符 -> USB 目标适配 -> 中间件类驱动 -> CMSIS 地址 -> 构建产物”的顺序分组查阅，不必线性阅读全部条目。

- `USB_DEVICE/App/usb_device.c`
- `USB_DEVICE/App/usbd_cdc_if.c`
- `USB_DEVICE/App/usbd_cdc_if.h`
- `USB_DEVICE/App/usbd_desc.c`
- `USB_DEVICE/App/usbd_desc.h`
- `USB_DEVICE/Target/usbd_conf.c`
- `USB_DEVICE/Target/usbd_conf.h`
- `Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc/usbd_cdc.h`
- `Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c`
- `Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xe.h`
- `Debug/Three-axis_cloud_platformV2.map`
- `Debug/Three-axis_cloud_platformV2.list`
- `Debug/USB_DEVICE/App/usbd_cdc_if.su`
- `Debug/USB_DEVICE/App/usbd_cdc_if.cyclo`
- `Debug/USB_DEVICE/App/usbd_desc.su`
- `Debug/USB_DEVICE/App/usbd_desc.cyclo`
- `Debug/USB_DEVICE/Target/usbd_conf.su`
- `Debug/USB_DEVICE/Target/usbd_conf.cyclo`
- `Debug/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.su`
- `Debug/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.cyclo`

权威参考资料：

- ST UM1734，STM32Cube USB Device Library User Manual。
- USB-IF USB 2.0 Specification。
- USB-IF Communications Device Class Abstract Control Model specification。
- USB-IF Communications Device Class specification。

引用的函数、配置项和变量：

- `USBD_static_malloc()`
- `USBD_static_free()`
- `USBD_malloc`
- `USBD_free`
- `USBD_CDC_RegisterInterface()`
- `USBD_Interface_fops_FS`
- `CDC_Init_FS()`
- `CDC_DeInit_FS()`
- `CDC_Control_FS()`
- `CDC_Receive_FS()`
- `CDC_Transmit_FS()`
- `USBD_CDC_SetTxBuffer()`
- `USBD_CDC_SetRxBuffer()`
- `USBD_CDC_ReceivePacket()`
- `USBD_CDC_TransmitPacket()`
- `USBD_CDC_Init()`
- `USBD_CDC_DataOut()`
- `USBD_CDC_DataIn()`
- `USBD_CDC_Setup()`
- `USBD_CDC_EP0_RxReady()`
- `USBD_CtlSendData()`
- `USBD_CtlPrepareRx()`
- `USBD_CDC_CfgFSDesc`
- `USBD_FS_DeviceDesc`
- `USBD_StringSerial`
- `USBD_StrDesc`
- `CDC_GET_LINE_CODING`
- `CDC_SET_LINE_CODING`
- `UserRxBufferFS`
- `UserTxBufferFS`
- `APP_RX_DATA_SIZE`
- `APP_TX_DATA_SIZE`
- `CDC_IN_EP`
- `CDC_OUT_EP`
- `CDC_CMD_EP`
- `CDC_DATA_FS_MAX_PACKET_SIZE`
- `CDC_CMD_PACKET_SIZE`
- `USB_CDC_CONFIG_DESC_SIZ`
- `TxState`
- `RxState`
- `RxLength`
- `CmdOpCode`
- `CmdLength`
- `pUserData`
- `pClassData`
- `USBD_SELF_POWERED`
- `USB_CONFIG_SELF_POWERED`
- `FS_Desc`
- `USBD_FS_DeviceDescriptor()`
- `USBD_FS_ProductStrDescriptor()`
- `USBD_FS_SerialStrDescriptor()`
- `Get_SerialNum()`
- `IntToUnicode()`
- `DEVICE_ID1`
- `DEVICE_ID2`
- `DEVICE_ID3`
- `USBD_VID`
- `USBD_PID_FS`
- `VID=0x0483`
- `PID=0x5740`
- `USBD_PRODUCT_STRING_FS`
- `UID_BASE`

质量自检：

- P0 事实错误：通过。
- P1 依赖断层：通过。
- P2 逻辑连贯：通过。
- P3 项目证据：通过。
- P4 原理展开：通过。
- P5 调试实践：通过。
- P6 表达统一：通过。

---
> 导航：上一章：[第15章_USB FS设备与中间件](第15章_USB FS设备与中间件.md) ｜ 下一章：[第17章_项目数据结构与配置对象](第17章_项目数据结构与配置对象.md)
