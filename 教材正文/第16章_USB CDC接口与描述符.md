# 第16章 USB CDC接口与描述符

## 1. 本章目标

- 理解 USB CDC 虚拟串口在项目 USB Device 栈中的位置。
- 看懂 `usb_device.c` 如何注册 CDC 类和 `USBD_Interface_fops_FS`。
- 能追踪 `usbd_cdc_if.c` 中 CDC 初始化、接收、发送和缓冲区配置。
- 能理解 `usbd_conf.c/h` 中 USB 静态内存分配的项目作用。
- 能定位 `usbd_desc.c` 中 VID、PID、字符串描述符和唯一序列号生成路径。
- 明确当前项目 USB CDC 属于已初始化通信接口，未发现业务协议解析主线。

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
- CDC接口回调：CDC 类驱动调用的应用侧函数集合，本项目对象为 `USBD_Interface_fops_FS`。
- 接收缓冲区：本项目 `UserRxBufferFS`，大小由 `APP_RX_DATA_SIZE` 定义为 1024。
- 发送缓冲区：本项目 `UserTxBufferFS`，大小由 `APP_TX_DATA_SIZE` 定义为 1024。
- USB描述符：主机枚举 USB 设备时读取的一组结构化信息，包括设备、配置、接口和字符串描述符。
- VID/PID：USB 设备的厂商 ID 和产品 ID，本项目在 `usbd_desc.c` 中定义。
- 唯一序列号：项目通过 STM32 唯一 ID 地址 `DEVICE_ID1/2/3` 生成 USB 字符串序列号。

这些概念服务于正式知识点 `USB静态内存分配`、`USB CDC虚拟串口` 和 `USB描述符与唯一序列号`，不新增结构外知识点。

## 5. 工作原理

第16章内部有明确的顺序。

第一步是静态内存分配。USB Device 中间件在运行 CDC 类时需要保存类状态。

项目在 `usbd_conf.h` 中把 `USBD_malloc` 映射到 `USBD_static_malloc()`。

`usbd_conf.c` 用静态数组 `mem` 返回一块 `USBD_CDC_HandleTypeDef` 大小的内存。

`USBD_static_free()` 是空函数，说明这条路径不是通用动态内存管理，而是为 USB Device 类对象准备的固定分配方式。

第二步是 CDC 接口注册。

`usb_device.c` 中 `MX_USB_DEVICE_Init()` 先注册 `USBD_CDC` 类。

随后它调用 `USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS)`。

`USBD_Interface_fops_FS` 定义在 `usbd_cdc_if.c`，包含四个回调入口：

- `CDC_Init_FS()`
- `CDC_DeInit_FS()`
- `CDC_Control_FS()`
- `CDC_Receive_FS()`

第三步是 CDC 收发。

`CDC_Init_FS()` 设置默认发送和接收缓冲区。

`CDC_Receive_FS()` 在收到数据后重新设置接收缓冲，并调用 `USBD_CDC_ReceivePacket()` 继续接收。

`CDC_Transmit_FS()` 检查 `hcdc->TxState`，若正在发送则返回 `USBD_BUSY`；否则设置发送缓冲并调用 `USBD_CDC_TransmitPacket()`。

第四步是描述符。

`usb_device.c` 中 `USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS)` 把描述符集合交给 USB Device 栈。

`FS_Desc` 定义在 `usbd_desc.c`，它包含设备描述符、语言 ID 字符串、厂商字符串、产品字符串、序列号字符串、配置字符串和接口字符串的函数指针。

这些机制共同说明：USB CDC 接口已经具备枚举、接口回调、收发入口和运行时对象分配基础。

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

### 3. CDC 接收路径

`CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)` 的当前实现很克制：

- 调用 `USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0])`
- 调用 `USBD_CDC_ReceivePacket(&hUsbDeviceFS)`
- 返回 `USBD_OK`

它没有解析 `Buf` 内容，没有使用 `Len` 做协议分发，也没有修改项目配置对象。因此当前项目不能被写成“USB 接收上位机指令并控制云台”。如果未来要扩展 USB 协议，`CDC_Receive_FS()` 才是自然入口之一。

### 4. CDC 发送路径

`CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)` 是项目暴露的 USB CDC 发送函数。它先取得 `hUsbDeviceFS.pClassData` 并转换为 `USBD_CDC_HandleTypeDef*`，然后检查 `TxState`：

- 若 `TxState != 0`，返回 `USBD_BUSY`。
- 否则调用 `USBD_CDC_SetTxBuffer()` 设置待发送数据。
- 再调用 `USBD_CDC_TransmitPacket()` 发起发送。

这说明发送函数具备非阻塞忙状态判断。可是当前 `Core` 和 `Drivers` 目录没有发现业务代码调用 `CDC_Transmit_FS()`，所以不能把它写成实际运行中的状态输出通道。

### 5. 描述符集合

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

### 6. 唯一序列号

`usbd_desc.h` 定义：

- `DEVICE_ID1 = UID_BASE`
- `DEVICE_ID2 = UID_BASE + 0x4`
- `DEVICE_ID3 = UID_BASE + 0x8`
- `USB_SIZ_STRING_SERIAL = 0x1A`

`Get_SerialNum()` 从这三个地址读取 32 位值，将 `deviceserial0` 与 `deviceserial2` 相加，再把 `deviceserial0` 和 `deviceserial1` 通过 `IntToUnicode()` 写入 `USBD_StringSerial`。

`IntToUnicode()` 每次取 4 位十六进制数，转换为 ASCII 字符，并在后一个字节写 0，生成 USB 字符串描述符所需的 Unicode 形式。

这条路径说明序列号来自 MCU 唯一 ID，而不是手写固定字符串。

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

## 8. 代码分析

### 1. `USBD_static_malloc()`

该函数位于 `usbd_conf.c`。它返回静态数组 `mem`，数组大小覆盖 `USBD_CDC_HandleTypeDef`。

它的输入参数名是 `size`，但当前实现不根据 `size` 分配不同大小。这是 CubeMX 生成 USB Device 工程中常见的固定类对象分配方式。本章不把它扩展成通用内存管理器。

### 2. `USBD_static_free()`

该函数为空实现。由于分配的是静态数组，不需要像堆内存那样释放。

调试时如果看到 `USBD_free`，要记住它在当前项目中只是映射到这个空释放函数。

### 3. `USBD_CDC_RegisterInterface()`

该调用位于 `usb_device.c`。它把 `USBD_Interface_fops_FS` 交给 CDC 类驱动，使 CDC 类在初始化、控制请求和接收数据时能回调到应用侧函数。

这一句是 `usb_device.c` 和 `usbd_cdc_if.c` 的连接点。

### 4. `CDC_Init_FS()`

该函数设置 CDC 发送和接收缓冲区：

- 发送缓冲区为 `UserTxBufferFS`
- 接收缓冲区为 `UserRxBufferFS`

这一步说明 USB CDC 接口启动时已经具备基本缓冲区。缓冲区大小来自 `APP_TX_DATA_SIZE` 和 `APP_RX_DATA_SIZE`。

### 5. `CDC_Control_FS()`

该函数包含 CDC 类控制命令的 switch 分支，例如 `CDC_SET_LINE_CODING`、`CDC_GET_LINE_CODING`、`CDC_SET_CONTROL_LINE_STATE` 等。

当前各分支基本为空，仅返回 `USBD_OK`。因此本项目没有使用这些控制请求来保存波特率、停止位、校验位或数据位配置。对于 USB CDC 虚拟串口来说，主机侧看到的串口参数不等于 MCU 内部真实 USART 参数；当前项目没有把这些参数映射到任何业务外设。

### 6. `CDC_Receive_FS()`

该函数收到数据后立即重新挂接接收缓冲并继续接收。它没有复制数据到项目命令缓冲区，也没有解析协议。

这条事实很关键：有接收回调不等于有上位机命令协议。

### 7. `CDC_Transmit_FS()`

该函数检查 `TxState`，避免上一包未发完时继续发新包。返回值可能是 `USBD_OK`、`USBD_FAIL` 或 `USBD_BUSY`。

它是项目未来通过 USB CDC 发数据的入口，但当前源码未发现业务调用。

### 8. `FS_Desc`

`FS_Desc` 是 `USBD_DescriptorsTypeDef` 类型对象。`USBD_Init()` 使用它把描述符函数交给 USB Device Library。

主机枚举时，USB Device Library 会通过这些函数取得设备描述符、字符串描述符和序列号描述符。

### 9. `Get_SerialNum()`

该函数读取 STM32 唯一 ID，并生成序列号字符串。它不是随机数，也不是固定常量。不同 MCU 理论上应生成不同的序列号字符串，但当前仓库没有主机枚举日志或实际设备验证记录，主机侧显示结果仍标记为【待验证】。

## 9. 调试方法

USB CDC 调试应先区分“设备能枚举”“虚拟串口能出现”“项目业务能通信”三个层级。

静态检查：

- `usbd_conf.h` 中 `USBD_malloc` 是否映射到 `USBD_static_malloc`。
- `usbd_conf.c` 中 `USBD_static_malloc()` 是否返回 `USBD_CDC_HandleTypeDef` 大小的静态数组。
- `usb_device.c` 中是否调用 `USBD_CDC_RegisterInterface()`。
- `usbd_cdc_if.c` 中是否定义 `USBD_Interface_fops_FS`。
- `usbd_cdc_if.c` 中 `APP_RX_DATA_SIZE` 和 `APP_TX_DATA_SIZE` 是否为 1024。
- `usbd_desc.c` 中产品字符串是否为 `STM32 Virtual ComPort`。
- `usbd_desc.c` 中序列号是否由 `DEVICE_ID1/2/3` 生成。

运行检查：

- 主机能否枚举出 USB 设备。
- 主机是否出现 CDC 虚拟串口。
- 调用 `CDC_Transmit_FS()` 时是否返回 `USBD_BUSY`。
- `CDC_Receive_FS()` 是否被触发。

项目业务检查：

- 当前 `Core` 和 `Drivers` 目录是否存在 `CDC_Transmit_FS()` 调用。
- `CDC_Receive_FS()` 是否解析 `Buf` 和 `Len`。
- USB 接收数据是否进入 PID、目标角、运行状态或配置对象。

当前仓库没有主机枚举日志、虚拟串口截图、USB 抓包或上位机通信记录，因此主机侧表现只能标记为【待验证】。源码层面只能确认 USB CDC 接口框架存在，未确认业务协议存在。

调试记录建议：

- 记录静态内存分配、CDC 回调表、RX/TX 缓冲区、描述符字符串和序列号来源。
- 主机 VCOM 结论应单独记录枚举截图、串口节点、抓包或主机日志，缺失时标记为【待验证】。
- 项目业务通信应记录 `CDC_Transmit_FS()` 调用点、`CDC_Receive_FS()` 解析逻辑和控制参数写入路径。
- 如果搜索不到业务调用或解析路径，应写成“框架存在、业务协议未确认”，不能写成“USB 通信已完成”。

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

## 11. 实践任务

开始任务前，先回到本章第8节定位静态内存、CDC 回调表、描述符和序列号生成证据；第9节提供 USB CDC 分层验证顺序。

任务一至任务七属于仓库内 CDC 与描述符证据；任务八属于仓库外主机侧验证分层。

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

任务六：确认描述符字符串来源。

在 `usbd_desc.c` 中找到 VID、PID、Manufacturer、Product、Configuration 和 Interface 字符串。
验收依据是描述符表包含 VID、PID、Manufacturer、Product、Configuration、Interface 六项来源。

任务七：追踪唯一序列号生成。

在 `usbd_desc.c/h` 中追踪 `DEVICE_ID1/2/3`、`Get_SerialNum()` 和 `IntToUnicode()`。
验收依据是序列号链路图包含 `DEVICE_ID1/2/3`、`Get_SerialNum()` 和 `IntToUnicode()` 三个节点。

任务八：整理 USB CDC 主机侧验证表。

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

## 13. 本章总结

本章建立了三轴云台项目中 USB 静态内存、CDC 接口和 USB 描述符之间的证据链。

已经确认的结论是：

- `usbd_conf.h` 将 `USBD_malloc` 和 `USBD_free` 映射到静态分配/释放函数。
- `usbd_conf.c` 使用静态数组为 `USBD_CDC_HandleTypeDef` 提供内存。
- `usb_device.c` 注册 `USBD_CDC` 类，并把 `USBD_Interface_fops_FS` 注册为 CDC 应用接口。
- `usbd_cdc_if.c` 提供 1024 字节接收缓冲和 1024 字节发送缓冲。
- `CDC_Init_FS()` 挂接默认收发缓冲。
- `CDC_Receive_FS()` 当前只继续接收，不解析业务协议。
- `CDC_Transmit_FS()` 具备发送入口和忙状态判断，但当前未发现业务调用。
- `usbd_desc.c` 定义 VID、PID 和多个字符串描述符。
- `Get_SerialNum()` 通过 STM32 唯一 ID 生成 USB 序列号字符串。

本章边界：

- 本章证明 USB CDC 接口、描述符和静态内存路径存在，不证明 USB CDC 已承担当前调试输出主线。
- `CDC_Receive_FS()` 当前没有业务协议解析证据，`CDC_Transmit_FS()` 当前也没有业务调用证据。

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

- `USB_DEVICE/App/usb_device.c`
- `USB_DEVICE/App/usbd_cdc_if.c`
- `USB_DEVICE/App/usbd_cdc_if.h`
- `USB_DEVICE/App/usbd_desc.c`
- `USB_DEVICE/App/usbd_desc.h`
- `USB_DEVICE/Target/usbd_conf.c`
- `USB_DEVICE/Target/usbd_conf.h`
- `Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c`

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
- `UserRxBufferFS`
- `UserTxBufferFS`
- `APP_RX_DATA_SIZE`
- `APP_TX_DATA_SIZE`
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
- `USBD_PRODUCT_STRING_FS`

质量自检：

- P0 事实错误：通过。
- P1 依赖断层：通过。
- P2 逻辑连贯：通过。
- P3 项目证据：通过。
- P4 原理展开：通过。
- P5 调试实践：通过。
- P6 表达统一：通过。
