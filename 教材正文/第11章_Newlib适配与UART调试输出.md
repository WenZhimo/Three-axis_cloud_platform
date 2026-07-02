# 第11章 Newlib适配与UART调试输出

> 导航：上一章：[第10章_DWT计时与微秒时间基准](第10章_DWT计时与微秒时间基准.md) ｜ 下一章：[第12章_通用定时器PWM输出](第12章_通用定时器PWM输出.md)

## 1. 本章目标

- 理解裸机 STM32 工程为什么需要为 C 标准库提供 Newlib 系统调用适配。
- 看懂项目中 `syscalls.c`、`sysmem.c` 和 `usart.c` 如何共同支撑 `printf()` 输出。
- 区分 `syscalls.c` 中的弱 `_write()` 路径、`usart.c` 中的强 `_write()` 路径和 `fputc()` 兼容路径。
- 能追踪 USART3 从 `.ioc` 引脚配置、HAL 初始化到 `HAL_UART_Transmit()` 阻塞发送的完整证据链。
- 能定位项目中初始化提示、传感器调试信息和 10Hz 状态输出对应的串口打印入口。

本章阅读分层：

| 阅读层次 | 建议范围 | 适合读者 |
|---|---|---|
| 【必须掌握】 | 第1节到第5节，第13节总结 | 需要理解 `printf()`、Newlib底层输出入口、项目 `_write()` 和 USART3 调试输出主线的读者 |
| 【工程深化】 | 第6节到第8节，第9节调试方法 | 需要维护 `syscalls.c`、`sysmem.c`、`usart.c`、阻塞发送、浮点打印和初始化顺序的读者 |
| 【拓展阅读】 | 第6.3节到第6.6节，第8.2节到第8.7节，第10节常见问题 | 需要进一步理解 BRR 量化、逐字节发送代价、`_scanf_float`、堆路径和 USB CDC 边界的读者 |
| 【证据与验证】 | 第8节、第9节、第10.7节到第10.9节、章节尾部固定检查，以及所有 `【待验证】` 项 | 需要审查 `_write()`重定向、`fputc()`兼容路径、`_sbrk()`堆边界、初始化打印、10Hz状态输出、`.map/.list/.su/.cyclo`、断点、PC10波形和终端日志的读者 |

如果只是沿串口调试主线学习，可以先抓住“`printf()` 格式化 -> Newlib `_write()` 入口 -> `usart.c` 强符号覆盖 -> `HAL_UART_Transmit()` 阻塞发送 -> USART3_TX”这条链；排查浮点输出、输入能力、堆路径或实时性影响时，再回到构建产物和调试方法小节。

本章速查：

| 查找目标 | 优先阅读 | 避免重复展开 |
|---|---|---|
| `printf()`、Newlib `_write()` 和 USART3 调试输出主线 | 第4节到第5节、第13节 | 系统时钟回到第06章，GPIO复用和USART3引脚回到第07章、第08章 |
| 弱 `_write()`、强 `_write()`、`fputc()` 和阻塞发送边界 | 第6节到第8.3节、第9节 | 不把 USB CDC 写成当前 `printf()` 主输出路径 |
| 浮点打印、`_sbrk()` 堆路径和输入能力边界 | 第8.4节到第8.7节、第10节 | USB CDC 接口继续到第16章，MPU6050 调试日志回到第18章 |
| 构建产物、PC10波形、终端日志和实时性验证边界 | 第10.7节到第10.9节、章节尾部固定检查 | `.map/.list/.su/.cyclo` 不能替代断点、波形或终端实测 |

## 2. 前置知识

- 链接脚本与内存布局
- 系统时钟树
- GPIO输出与复用
- AFIO重映射与SWD调试
- SysTick系统节拍

本章内部先讲 `Newlib系统调用适配`，再讲 `UART串口调试输出`。原因是 `printf()` 属于 C 标准库接口，它要先能落到低层输出函数，之后才能通过 USART3 发出字节。

第03章已经说明链接脚本和 `_end`、`_estack` 等内存边界，第06章已经说明 UART 依赖系统时钟。

第07章和第08章已经说明 PC10/PC11 的 GPIO 复用与 USART3 部分重映射，第09章已经说明 `HAL_GetTick()` 的毫秒时基。本章只把这些前置知识接到串口调试输出路径上。

本章不展开 USB CDC 虚拟串口、不展开上位机协议、不展开 MPU6050 初始化和温度标定算法。`mpu6050.c` 与 `mpu6050Calibration.c` 只作为 `printf()` 调用证据出现。

## 3. 问题背景

三轴云台项目运行在 STM32F103RCTx 裸机环境中，没有操作系统提供文件、终端、堆内存和标准输入输出设备。可是项目仍然使用了 C 标准库风格的调试输出：

- `main.c` 在 MPU6050 初始化成功或失败时打印提示。
- `main.c` 在 AHRS 收敛完成时打印状态提示。
- `main.c` 在 10Hz 低频任务中输出 Roll 姿态角和 `return_state_count_roll`。
- `mpu6050.c` 在器件 ID 读取、重试和错误诊断中打印信息。
- `mpu6050Calibration.c` 在温度漂移校准过程中打印阶段和结果。

这些调用表面上都是 `printf()`，但真正落到硬件时必须回答三个问题：

1. `printf()` 在没有操作系统的情况下如何找到输出函数？
2. 输出字节最终从哪个串口发出？
3. 浮点格式打印为什么能在当前构建配置中工作？

本章解决的就是这条从 C 标准库到 USART3 的工程链路。

## 4. 核心概念

- Newlib：ARM GCC 裸机工具链常用的 C 标准库实现。项目使用 newlib-nano 链接选项以减小运行库体积。
- 系统调用适配：为 C 标准库补齐底层入口，例如 `_write()`、`_read()`、`_sbrk()`、`_fstat()` 和 `_isatty()`。
- 弱符号：可以被同名强符号覆盖的函数定义。本项目 `syscalls.c` 中 `_write()` 和 `_read()` 是弱定义。
- `_write()`：标准库输出路径的底层写入口。当前项目在 `usart.c` 中提供 GCC 条件下的强 `_write()`，逐字节调用 `HAL_UART_Transmit()`。
- `fputc()`：部分 `printf()` 实现可能经过的字符输出路径。本项目也将它重定向到 `huart3`。
- `_sbrk()`：Newlib 堆分配入口，`malloc()` 等函数会通过它向裸机工程申请堆空间。
- 文件描述符：`_write(int file, ...)` 中的 `file` 用于区分 stdout、stderr 等输出目标；当前项目忽略该参数，因此不同输出流不会分流到不同设备。
- 可重入系统调用封装：Newlib 内部可能通过 `_write_r()`、`_sbrk_r()` 等可重入包装进入底层系统调用；项目真正需要提供的是能落到硬件或内存边界的底层入口。
- USART3：本项目用于 UART 调试输出的串口实例，配置为 115200、8N1、TX/RX、无硬件流控。
- 阻塞发送：`HAL_UART_Transmit()` 在超时时间内等待发送完成。本项目 `_write()` 和 `fputc()` 都使用这种发送方式。
- 串口帧时间：115200、8N1 表示每个字符约 10bit，纯线路发送时间约为 86.8us/字符。
- 浮点打印支持：`.cproject` 和 `Debug/makefile` 中启用 `-u _printf_float`，使 newlib-nano 支持 `%f` 等浮点输出格式。
- map 文件：链接器生成的符号落点记录，可用于确认最终 `_write` 来自 `usart.o` 还是 `syscalls.o`。
- 符号拉入：链接器为了满足未解析符号或库内依赖，把 archive 中的目标文件纳入最终 ELF；它证明能力和代码存在，不等于每次运行都会执行其中所有分支。
- 运行路径证据：断点、日志、水位、返回值或现场记录，用于证明某次具体输出是否真的进入了某个库函数、堆分配路径或异常分支。

这些概念对应正式知识点 `Newlib系统调用适配` 和 `UART串口调试输出`，不新增结构外知识点。

## 5. 工作原理

从 `printf()` 到串口输出，可以分成四层。

第一层是标准库格式化层。

`printf()` 负责把格式字符串和变量转换成字符序列。

例如项目中的 `printf("%.6f,%d\r\n", sensors.margAttitude500Hz[ROLL], return_state_count_roll)` 会生成一串 ASCII 字符。

这一层关心格式化，不关心字符从哪个硬件引脚出去。

第二层是低层输出入口。

Newlib 需要一个 `_write()` 这样的底层函数，把格式化后的字符写到目标设备。在桌面系统里，这个目标可能是终端文件描述符；在本项目中，目标被项目代码改造成 USART3。

这里还隐藏着一层可重入封装。Newlib 为许多系统调用准备了带 `_r` 后缀的入口，例如 `_write_r()` 和 `_sbrk_r()`，它们携带 reentrancy 结构以保存 `errno` 等运行库状态。裸机项目通常不直接实现所有 `_r` 函数，而是提供 `_write()`、`_sbrk()` 等底层入口，让运行库包装函数最终落到这些入口上。

因此，本章不能只说“有 `printf()` 就能打印”。更准确的链路是：

```text
printf()
  -> Newlib 格式化与 stdio 缓冲
  -> _write_r() 或相关包装路径
  -> 项目提供的 _write()
  -> HAL_UART_Transmit()
  -> USART3_TX 引脚
```

第三层是 HAL UART 发送。

`usart.c` 中的强 `_write()` 遍历 `ptr` 指向的字符缓冲区，对每个字节调用 `HAL_UART_Transmit(&huart3, ...)`。这一步把标准库字符流转换为 USART3 外设发送动作。

注意这里的“逐字节”不是小细节。Newlib 传入 `_write()` 的可能是一整段格式化后的字符串，但当前实现没有把整段缓冲区一次交给 HAL，而是每个字符调用一次 `HAL_UART_Transmit()`。

因此，输出成本包含三部分：浮点/整数格式化成本、每字节 HAL 调用和状态检查成本、串口线路本身的发送时间。

继续把这个知识点拆细，还要区分“UART 硬件逐位发送”和“软件逐字节调用 HAL”。`HAL_UART_Transmit(&huart3, ..., Size)` 内部会在每个数据写入前等待 `UART_FLAG_TXE`，但只在该次函数调用的末尾等待一次 `UART_FLAG_TC`。如果把 `len` 字节一次交给 HAL，TC 完成等待发生在整段缓冲区末尾；当前 `_write()` 把 `Size` 固定为 1，循环调用 `len` 次，所以每个字符都会经历一次 `READY -> BUSY_TX -> READY` 状态切换和一次 TC 完成等待。

第四层是引脚与重映射。

`.ioc` 将 PC10 标为 USART3_TX、PC11 标为 USART3_RX，并配置 USART3 异步模式。

`usart.c` 的 MSP 初始化启用 USART3 和 GPIOC 时钟，配置 PC10 为复用推挽输出、PC11 为输入。

随后代码调用 `__HAL_AFIO_REMAP_USART3_PARTIAL()`。

这条链路解释了为什么 `HAL_UART_Transmit(&huart3, ...)` 的工程配置出口对应 PC10。

同一章还要解释 `_sbrk()`。它不直接参与串口发送，但属于 Newlib 运行时适配的一部分。

`sysmem.c` 让 Newlib 堆从链接脚本符号 `_end` 之后开始增长。

它用 `_estack - _Min_Stack_Size` 作为最大边界，避免堆侵入预留 MSP 栈空间。

这也解释了为什么 `-u _scanf_float` 不能简单理解成“串口输入已经可用”。它只说明链接时拉入浮点扫描格式支持；若 `_read()` 和 `__io_getchar()` 没有被有效接到 UART RX，`scanf()` 仍然没有可靠的项目级输入通道。

## 6. STM32实现机制

本项目的 STM32 实现机制由生成代码、用户代码和构建配置共同组成。

### 1. 系统调用文件

`Core/Src/syscalls.c` 是 STM32CubeIDE 生成的最小系统调用文件。它提供：

- `_getpid()`、`_kill()`、`_exit()` 等基础占位函数。
- 弱 `_read()`，逐字节调用弱 `__io_getchar()`。
- 弱 `_write()`，逐字节调用弱 `__io_putchar()`。
- `_fstat()` 和 `_isatty()`，把输出目标按字符设备处理。
- `_open()`、`_wait()`、`_unlink()` 等不支持或占位的入口。

这里的关键点是“弱”。

如果工程中没有更具体的 `_write()`，标准库可能落到 `syscalls.c` 的弱 `_write()`，再依赖 `__io_putchar()`。

当前项目在 `usart.c` 中提供了同名非弱 `_write()`，因此 GCC 构建下实际串口输出应以 `usart.c` 的实现为主。

`syscalls.c` 和 `usart.c` 的 `_write()` 都忽略 `file` 参数。因此在当前实现中，stdout、stderr 或其它文件描述符不会被分流到不同设备，而是被统一折叠到同一条 UART 输出路径。

从源码层面可以判断强符号覆盖弱符号；当前 `Debug/Three-axis_cloud_platformV2.map` 还给出了链接层证据：

- `./Core/Src/usart.o` 提供最终地址上的 `.text._write`，符号地址为 `0x0800257c`。
- `./Core/Src/syscalls.o` 中也出现 `.text._write`，但地址记录为 `0x00000000`，不是最终运行地址上的输出入口。

因此，在当前 Debug 链接产物中，`printf()` 的底层写路径应以 `usart.c` 的 `_write()` 为准。若切换构建配置或清理重建，应重新检查对应配置的 `.map` 文件。

#### 8.2.1 `_write()`重定向的构建证据边界

`_write()` 重定向不是一句“printf 走串口”就能完整说明。当前仓库可以把证据拆成四层：

| 层级 | 当前证据 | 能证明什么 | 不能证明什么 |
|---|---|---|---|
| 源码入口 | `syscalls.c` 弱 `_write()`；`usart.c` 强 `_write()` 和 `fputc()` | 工程提供了 Newlib 输出适配入口，且项目输出目标写成 `huart3` | 标准库每次运行一定选择哪条内部缓冲路径 |
| 链接结果 | `.map` 中最终 `_write` 位于 `./Core/Src/usart.o`，`syscalls.o` 的弱 `_write` 未占最终运行地址 | 当前 Debug ELF 的底层写入口由 USART3 实现覆盖 | Release 或重新生成后的链接结果仍保持不变 |
| 反汇编调用边 | `.list` 中 `_write_r` 跳转到 `0x0800257c <_write>`，项目 `_write()` 循环调用 `HAL_UART_Transmit()` | 当前构建存在 Newlib 可重入包装到项目 `_write()`、再到 HAL UART 的静态调用边 | 某次上电已经执行到该路径，或每个字符都被主机收到 |
| 运行与外部观察 | 断点观察 `_write()`、`HAL_UART_Transmit()` 返回值、`huart3.gState`，以及 PC10 波形或终端日志【待验证】 | 可以验证现场是否真正发送、是否超时或忙碌、主机是否接收 | 当前仓库没有这些现场记录，不能宣称物理串口输出已验证 |

还要注意 `file` 参数的边界。`printf()`、`fprintf(stderr, ...)` 或其它标准流最终可能都通过 `_write(int file, ...)` 传入不同文件描述符，但当前 `usart.c` 实现没有检查 `file`，也没有按 stdout/stderr 分流。因此本章只能说“当前项目把底层写入统一折叠到 USART3 发送路径”，不能写成“已经实现独立标准输出、标准错误和串口日志等级”。

### 2. 堆内存文件

`Core/Src/sysmem.c` 实现 `_sbrk()`。它读取链接脚本中的 `_end`、`_estack` 和 `_Min_Stack_Size`，把 newlib 堆限制在 `.bss` 之后和预留 MSP 栈之前。

这与第03章的链接脚本前置知识直接相连：如果链接脚本边界理解错误，`malloc()`、浮点格式化等潜在内存使用就可能更难定位。当前章节只说明 `_sbrk()` 的边界机制，不展开动态内存管理策略。

### 3. USART3 初始化

`Core/Src/usart.c` 中 `MX_USART3_UART_Init()` 设置：

- `huart3.Instance = USART3`
- 波特率 115200
- 8 位数据位
- 1 位停止位
- 无校验
- TX/RX 模式
- 无硬件流控
- 16 倍过采样

`HAL_UART_MspInit()` 在 USART3 分支中启用 USART3 和 GPIOC 时钟，配置 PC10/PC11，并执行 USART3 部分重映射。它承接第07章和第08章的 GPIO/AFIO 前置知识。

USART3 属于 APB1 侧 USART。HAL 的 `UART_SetConfig()` 对非 USART1 外设调用 `HAL_RCC_GetPCLK1Freq()`，再用 `UART_BRR_SAMPLING16(pclk, BaudRate)` 写入 BRR。

项目使用 115200 和 16 倍过采样；若 APB1 时钟为 36MHz，则 BRR 原始量和 USARTDIV 理想值约为：

```text
BRR_raw_ideal = 36,000,000 / 115,200 = 312.5
USARTDIV_ideal = BRR_raw_ideal / 16 = 19.53125
```

实际 BRR 的尾数和小数编码由 HAL 宏完成。本章只要求读者理解“波特率来自 APB1 时钟和 BRR 分频”，不要求手写寄存器编码。

如果继续把该知识点拆细，可以把 BRR 量化过程写成当前项目的数值边界。`Three-axis_cloud_platformV2.ioc` 记录 APB1 频率为 36MHz，`usart.c` 配置 USART3 为 115200、16 倍过采样。理想的 BRR 原始量约为：

```text
BRR_raw_ideal = PCLK1 / BaudRate
              = 36,000,000 / 115,200
              = 312.5
```

HAL 的 `UART_BRR_SAMPLING16()` 会把该值量化成寄存器尾数和小数位。按当前 HAL 宏计算，尾数约为 19，小数位约为 8，BRR 约为 `0x0138`。对应实际波特率近似为：

```text
USARTDIV_actual = 19 + 8/16 = 19.5
Baud_actual = 36,000,000 / (16 * 19.5)
            ≈ 115,384.6 baud
误差 ≈ +0.16%
```

这个误差通常在 UART 容差范围内，但它说明波特率不是“写 115200 就完全无误差”，而是由时钟、分频寄存器位宽和取整策略共同决定。

### 4. 输出重定向

`usart.c` 用户代码区提供两条输出路径：

- `fputc(int ch, FILE *f)`：把单个字符通过 `HAL_UART_Transmit(&huart3, ...)` 发出。
- GCC 条件下的 `_write(int file, char *ptr, int len)`：遍历缓冲区，逐字节通过 `HAL_UART_Transmit(&huart3, ...)` 发出。

这两条路径都指向 `huart3`，因此本项目的 `printf()` 调试输出主线是 USART3，而不是 USB CDC。

当前实现还有一个调试边界：`_write()` 不检查 `HAL_UART_Transmit()` 返回值，始终返回 `len`。

`fputc()` 也不检查 HAL 返回状态。这意味着即使 HAL 返回 `HAL_TIMEOUT` 或 `HAL_BUSY`，上层 `printf()` 也不会从当前代码中得到失败信号。

这里还要区分两个返回值语义：

- C 标准库关心 `_write()` 返回“实际写入了多少字节”或失败。
- 当前项目 `_write()` 无论 HAL 是否成功都返回 `len`。

所以当前实现更适合“调试观察”，不适合作为可靠日志传输协议。若要把它升级为可靠输出通道，应至少累积 HAL 返回状态、在失败时返回已发送字节数或错误，并定义上层如何处理丢字节、超时和重试。

### 5. HAL阻塞发送内部过程

HAL 源码中 `HAL_UART_Transmit()` 先要求 `huart->gState == HAL_UART_STATE_READY`。

进入发送后，它把 `gState` 改为 `HAL_UART_STATE_BUSY_TX`，记录 `tickstart = HAL_GetTick()`，再等待发送缓冲区空标志 `UART_FLAG_TXE`。

每写入一个字节后，HAL 更新 `TxXferCount`。全部字节写完后，它继续等待发送完成标志 `UART_FLAG_TC`。完成后，`gState` 回到 `HAL_UART_STATE_READY` 并返回 `HAL_OK`。

因此，对一次长度为 `N` 的阻塞发送调用，可以把 HAL 内部等待点写成：

```text
HAL_UART_Transmit(buf, N):
  TXE wait: N times
  TC wait:  1 time
  gState:   READY -> BUSY_TX -> READY, 1 round trip
```

但当前项目的 `_write()` 不是上述形式，而是：

```text
for each character:
  HAL_UART_Transmit(one_char, 1)
```

所以对同样的 `N` 个字符，当前路径更接近：

```text
current _write(buf, N):
  TXE wait: N times
  TC wait:  N times
  gState:   READY -> BUSY_TX -> READY, N round trips
```

这不是功能错误，而是调试输出实现的工程取舍：代码简单、行为直观，但在长日志或高频路径中会放大阻塞成本。

如果等待超时，HAL 会把状态恢复为 READY 并返回 `HAL_TIMEOUT`。

如果进入函数时 UART 已经在发送，则返回 `HAL_BUSY`。本项目传入的超时参数是 `0xFFFF`，单位由 HAL 毫秒 tick 管理；在 1ms tick 下约等于 65.535s。

这个数值只是超时上限，不是实际每个字符都会等待这么久。

### 6. 构建选项

`.cproject` 中 Debug 配置启用了 newlib-nano 的浮点 `printf` 和 `scanf` 选项。

`Debug/makefile` 的链接命令也包含 `--specs=nano.specs`、`--specs=nosys.specs`、`-u _printf_float` 和 `-u _scanf_float`。

这解释了为什么项目中存在 `%f` 输出，例如 10Hz 状态输出和 `MPU6050_Send_VOFA_Plus_Float()`。如果缺少 `-u _printf_float`，浮点格式输出可能出现异常或不完整。

当前 `.map` 文件进一步证明了浮点格式化代码被拉入链接结果：其中可以看到 `_printf_float` 和 `_scanf_float` 来自 `libc_nano.a`。这比只看 IDE 选项更接近最终固件事实。

继续沿 map 文件向下读，还能看到浮点格式化不是一个孤立符号。

当前构建中的链接链路可以分成两类：

- 浮点格式化入口：`libc_a-nano-vfprintf_float.o` 因 `_printf_float` 被纳入，`libc_a-nano-vfscanf_float.o` 因 `_scanf_float` 被纳入。
- 后续库成员：map 又显示 `libc_a-dtoa.o` 提供 `_dtoa_r`，`libc_a-malloc.o` 提供 `malloc`，`libc_a-mallocr.o` 提供 `_malloc_r`，`libc_a-sbrkr.o` 提供 `_sbrk_r`。

这条链路说明：浮点格式化能力可能把双精度转换、堆分配包装和 `_sbrk()` 底层入口一起带入 ELF。

但是这里必须区分“符号被拉入”和“某次运行已经执行”：

- map 能证明这些库成员进入当前 Debug ELF，也能说明 Flash 体积和潜在 RAM/堆路径会受到影响。
- map 不能单独证明每一次 `printf("%.6f", value)` 都触发了 `malloc()`、`_sbrk_r()` 或最坏栈峰值。
- 要证明实际运行路径，需要在 `_printf_float`、`_dtoa_r`、`_malloc_r`、`_sbrk()` 等位置断点，或记录 `_sbrk()` 返回值、堆顶变化、栈水位和现场日志。

但 `-u _scanf_float` 只代表浮点扫描格式支持被链接，不代表当前项目已经实现了可用的 UART 标准输入。`syscalls.c` 的弱 `_read()` 依赖 `__io_getchar()`，而本章证据链没有发现项目把 `__io_getchar()` 强实现为 USART3 接收。因此本章只能确认浮点输出链路，不能把 `scanf("%f")` 当作已验证功能。

代价是浮点格式化会拉入更多库代码，并可能增加 Flash、栈和临时内存压力。`_sbrk()` 能约束堆增长边界，但不能证明 `printf("%f")` 的栈峰值已经安全；真实余量仍需 map、栈水位或运行记录【待验证】。

## 7. 项目中的应用

第11章在项目主线中的位置是“调试观察通道”。

运行流程可以这样追踪：

1. `main()` 调用 `HAL_Init()` 和 `SystemClock_Config()`，建立 HAL 与系统时钟前提。
2. `main()` 按 CubeMX 生成顺序调用 GPIO、TIM、I2C2、TIM8、USART3、USB Device 和 TIM6 初始化入口。
3. `MX_USART3_UART_Init()` 配置 `huart3`，并通过 MSP 初始化 PC10/PC11 与 USART3 部分重映射。
4. 用户初始化阶段开始后，项目调用 `MPU6050_Init()`，并根据结果打印成功、跳过校准或失败提示。
5. 主循环中，500Hz 实时控制循环负责实时采样、姿态和控制计算；10Hz 低频任务用 `HAL_GetTick() - last_print_tick >= 100` 触发状态处理和串口输出。
6. 每次 `printf()` 输出时，标准库格式化字符，最终由 `usart.c` 中 `_write()` 或 `fputc()` 走 `HAL_UART_Transmit(&huart3, ...)`。

在“采集—处理—控制—输出”链路中，UART 调试输出不是控制执行器，也不是传感器通信。它的作用是让开发者观察项目状态：初始化是否成功、AHRS 是否收敛、姿态角和状态计数是否按预期变化。

## 8. 代码分析

### 8.1 `syscalls.c` 中的弱 `_write()`

入口是 Newlib 的底层写调用。输入是 `file`、字符指针 `ptr` 和长度 `len`。函数忽略 `file`，按长度逐字节调用 `__io_putchar()`。

风险点在于：`__io_putchar()` 也是弱外部符号。如果工程没有提供有效实现，而标准库最终落到这条路径，输出可能没有硬件目的地。当前项目用 `usart.c` 中的强 `_write()` 避开这个不确定性。

### 8.2 `usart.c` 中的强 `_write()`

入口同样是 Newlib 的底层写调用。输入是格式化后的字符缓冲区。函数对 `DataIdx` 从 0 到 `len - 1` 遍历，每次发送一个字节。

输出是 USART3 发送动作，状态变化发生在 `huart3` 对应的 UART 外设和发送状态机中。因为使用 `HAL_UART_Transmit()` 且超时参数为 `0xFFFF`，这条路径属于阻塞发送。调试输出过多时，它会占用 CPU 时间。

115200、8N1 的一帧通常包含 1bit 起始位、8bit 数据位和 1bit 停止位，即 10bit/字符。纯线路时间为：

```text
t_char = 10 / 115200 s ≈ 86.8us
t_50_char ≈ 4.34ms
t_100_char ≈ 8.68ms
```

这还没有计算 `printf()` 格式化、逐字节函数调用、HAL 等待标志和中断插入造成的额外开销。因此，10Hz 打印可以作为观察通道；若把长字符串直接放进 500Hz 控制循环，2ms 帧预算很容易被串口输出吞掉。

把同样的 100 字符输出放在 10Hz 低频任务中，纯线路时间约占 100ms 周期的 8.68%；放在 500Hz 控制帧中，则纯线路时间已经超过 2ms 周期的 4 倍。这个对比解释了为什么项目把连续状态输出放在低频任务中，而不是放进 500Hz 实时控制主路径。

还要注意，8.68ms 只是 100 字符在 115200 8N1 下的线路发送下限。当前逐字节 HAL 调用会额外叠加 `N` 次函数调用、`N` 次 `gState` 往返和 `N` 次 TC 轮询路径。若未来要把调试输出做成更可靠的日志通道，第一层可选优化是把 `_write()` 改为一次性调用 `HAL_UART_Transmit(&huart3, (uint8_t *)ptr, len, timeout)`；更进一步才是环形缓冲区配合 UART IT/DMA。前者降低软件调用和 TC 重复等待，后者降低实时路径阻塞，但都会引入返回值、缓冲区满、并发访问和发送完成回调等新设计问题。

当前 `_write()` 忽略 HAL 返回值并直接返回 `len`，所以“打印函数返回成功”不能等价于“所有字符都已被主机端看到”。主机端终端、接线、USB 转串口和接收日志仍是【待验证】。

还有一个并发边界：`HAL_UART_Transmit()` 使用 `huart3.gState` 判断发送状态。如果在一次阻塞发送尚未结束时，另一路径再次调用 `printf()`，HAL 可能返回 `HAL_BUSY`。当前 `_write()` 忽略该返回值，所以上层不容易发现这类丢输出或输出不完整问题。当前仓库没有在中断服务函数中调用 `printf()` 的证据；教材仍应提醒读者不要轻易把阻塞 `printf()` 放入 ISR 或高频实时路径。

#### 8.2.2 `_write()`返回值与错误传播边界

`_write(int file, char *ptr, int len)` 的返回值本来是标准库判断“底层写出了多少字节”的入口。
但当前项目实现把这条反馈链路截断了：

```text
for DataIdx in 0..len-1:
    HAL_UART_Transmit(...)
return len
```

因此，本项目的 UART 调试输出要拆成五层看：

| 层级 | 当前源码行为 | 能证明什么 | 不能证明什么 |
|---|---|---|---|
| 输出流选择 | `file` 参数未被使用 | stdout、stderr 等标准流都会折叠到同一条 USART3 路径 | 不能证明工程已有日志级别、错误流或多设备分流 |
| 单字节发送 | 每个字符都调用一次 `HAL_UART_Transmit(&huart3, ..., 1, 0xFFFF)` | 软件层尝试逐字节送入 USART3 | 不能证明每次 HAL 调用都返回 `HAL_OK` |
| HAL状态 | `HAL_UART_Transmit()` 可能返回 `HAL_OK`、`HAL_BUSY` 或 `HAL_TIMEOUT` | 可通过断点观察 `huart3.gState`、`ErrorCode` 和返回值【待验证】 | 当前 `_write()` 没有保存或上报这些状态 |
| Newlib感知 | `_write()` 无条件返回 `len` | 上层格式化函数会看到“请求长度已写完”的软件返回值 | 不能把 `printf()` 返回成功写成真实串口完整发送成功 |
| 外部接收 | 需要终端日志、逻辑分析仪或串口抓包证明【待验证】 | 可以证明主机端实际收到哪些字节【待验证】 | 仓库源码和 `.list` 不能替代主机接收证据 |

这个边界对调试很重要。若终端偶发缺字、输出卡顿或日志顺序异常，
仅看 `printf()` 返回值可能找不到原因；应在 `_write()` 内临时记录
`HAL_UART_Transmit()` 返回状态，或在调试器中观察 `huart3.gState`、`ErrorCode`
和 PC10 波形【待验证】。这些记录属于现场调试证据，不能由当前源码静态推出。

### 8.3 `fputc()` 兼容路径

`fputc()` 的输入是单个字符。它同样调用 `HAL_UART_Transmit(&huart3, ...)`，并返回传入字符。

这条路径用于兼容可能走字符输出接口的库实现。当前项目同时提供 `_write()` 和 `fputc()`，目的是让不同工具链或库路径都能落到 USART3。

### 8.4 `sysmem.c` 中的 `_sbrk()`

入口是 Newlib 堆申请请求，输入是增长量 `incr`。首次调用时，`__sbrk_heap_end` 从链接符号 `_end` 开始。每次申请前，函数用 `_estack - _Min_Stack_Size` 计算最大堆边界。

如果新堆顶超过边界，函数设置 `errno = ENOMEM` 并返回 `(void *)-1`。这说明项目没有无限堆空间，标准库功能虽然可用，但仍受 STM32 RAM 和链接脚本约束。

### 8.5 `main.c` 中的初始化打印

`MX_USART3_UART_Init()` 在 `main()` 的外设初始化阶段执行，早于用户初始化区中的 MPU6050 初始化提示。因此这些初始化提示具备 UART 已初始化的前提。

成功路径会打印 MPU6050 初始化成功和跳过温度校准提示；失败路径会打印初始化失败。这里的项目意义是：串口输出可以帮助判断程序是否已经越过传感器初始化节点。

### 8.6 `main.c` 中的 10Hz 状态输出

10Hz 低频任务用 `HAL_GetTick()` 与 `last_print_tick` 比较，约每 100ms 进入一次。它在状态处理之后打印 Roll 姿态角和 `return_state_count_roll`。

本章只分析这条输出路径：`HAL_GetTick()` 提供低频触发依据，`printf()` 提供格式化，USART3 负责发送。10Hz 低频任务的完整系统作用会在第32章展开。

还要补一条容易漏看的证据：`main.c` 在 500Hz 分支里还有一次性的 `printf(">>> AHRS 收敛完成，俯仰轴控制已使能。\\r\\n");`。
它不是周期性高频日志，而是状态切换时的单次提示，但它仍然运行在 500Hz 控制路径里，属于“偶发调试打印”而不是“完全无害的空操作”。
教材应把这种一次性标记和 10Hz 周期打印分开：前者只在收敛完成那一帧触发，后者才是常规观察通道。

### 8.7 MPU6050 文件中的打印证据

`mpu6050.c` 在器件 ID 检查、重试、关键错误和 VOFA 风格浮点输出中调用 `printf()`。`mpu6050Calibration.c` 在温度漂移校准阶段输出状态和结果。

这些文件说明 `printf()` 不是只在 `main.c` 中使用，而是已成为项目调试观察的公共输出入口。本章不分析其中的 I2C、标定和数据处理算法。

### 本节证据边界

本节只根据当前仓库说明文件、函数、宏、变量和调用关系。运行时频率、外部硬件表现、主机侧现象、传感器方向、电机响应或真实控制效果仍需调试记录、日志或仓库外实测证据；缺少证据时保持【待验证】。

## 9. 调试方法

本章调试围绕“字符是否能从 `printf()` 走到 USART3”展开。

本节按统一调试结构组织：现象 -> 可能原因 -> 定位方法 -> 验证步骤 -> 解决方案 -> 经验总结。对第11章而言，核心是把“Newlib 格式化路径”“USART3 发送路径”和“主机侧可见输出”分开验证。

### 9.1 现象与可能原因

可观察现象包括：

- `main.c` 中 MPU6050 初始化成功、跳过校准或失败提示。
- `mpu6050.c` 中器件 ID 检查和错误提示。
- `main.c` 中约 100ms 输出一次的 Roll 姿态角和 `return_state_count_roll`。
- `Debug/makefile` 中是否存在 `-u _printf_float`。
- `Debug/Three-axis_cloud_platformV2.map` 中是否出现 `_printf_float`、`_dtoa_r`、`malloc`、`_malloc_r` 和 `_sbrk_r`。
- `.ioc` 中 PC10/PC11 是否对应 USART3_TX/RX。

常见可能原因包括：`printf()` 早于 `MX_USART3_UART_Init()`、强 `_write()` 没有覆盖弱实现、USART3 GPIO 或重映射未生效、`HAL_UART_Transmit()` 返回忙或超时、浮点格式化链接选项缺失、SysTick 节拍异常，或主机侧接线和终端参数缺少实测证据。

### 9.2 定位方法

1. 如果完全没有输出，先确认 `MX_USART3_UART_Init()` 位于第一次 `printf()` 之前。
2. 再确认 `usart.c` 中是否存在 GCC 条件下的强 `_write()`，以及它是否调用 `HAL_UART_Transmit(&huart3, ...)`。
3. 再确认 `HAL_UART_MspInit()` 是否启用 USART3、GPIOC、PC10/PC11 和 USART3 部分重映射。
4. 在 `HAL_UART_Transmit()` 入口断点观察 `huart3.gState`、`ErrorCode` 和返回值，确认是否出现 `HAL_BUSY` 或 `HAL_TIMEOUT`。

### 9.3 验证步骤

5. 如果整数输出正常但浮点输出异常，检查 `.cproject` 和 `Debug/makefile` 中的 `-u _printf_float`。
6. 如果输出间隔异常，先回到第09章检查 `HAL_GetTick()` 和 SysTick 时基，再检查 `last_print_tick` 的更新逻辑。
7. 如果 MCU 内部路径成立但主机无输出，再检查 PC10/PC11、TX/RX 交叉、共地、115200 8N1 和终端接收记录【待验证】。

当前仓库没有串口接线、外部 USB 转串口设备或终端工具配置证据，因此具体物理接线和上位机串口工具参数只能标记为【待验证】。教材在本章只确认工程内部的 USART3 输出路径。

### 9.4 解决方案：调试记录

- 记录 `printf()`、`_write()`、`fputc()`、`HAL_UART_Transmit()` 和 `huart3` 之间的调用链证据。
- 对 `_write()`、`fputc()` 和 `HAL_UART_Transmit()` 分别设断点，可区分 Newlib 实际走缓冲写路径还是字符输出兼容路径。
- 记录 `.cproject`、`Debug/makefile` 与 `-u _printf_float`，把浮点输出能力和普通字符输出分开验证。
- 若要验证波特率，可以在 PC10 上用逻辑分析仪测单字节帧宽；没有波形或终端日志时，波特率输出效果保持【待验证】。
- 若要验证最终链接路径，检查 `Debug/Three-axis_cloud_platformV2.map` 中 `_write`、`_printf_float` 和 `_sbrk` 的符号来源；切换 Release 或重新生成工程后重新记录。
- 若要验证浮点格式化是否实际触发堆路径，在 `_printf_float`、`_dtoa_r`、`_malloc_r`、`_sbrk()` 处设断点，并记录 `_sbrk()` 的 `incr`、返回值、`errno` 和当前堆顶；只看 map 不能证明运行路径。
- 若要验证 `scanf()` 或串口输入，不要只看 `-u _scanf_float`，还要确认 `_read()` 或 `__io_getchar()` 是否已经接到 USART3 RX。
- 串口线、USB 转串口模块、波特率、终端截图和接收日志属于仓库外实测证据，缺失时标记为【待验证】。
- 若日志内容来自运行现场，应同时记录固件版本、编译配置和触发路径，避免把旧日志当作当前仓库证据。

### 9.5 经验总结

本章的调试结论应按证据层级收敛：`.map/.list` 能证明当前 Debug 构建中的符号落点和调用链，断点能证明某次运行进入发送路径，终端日志或波形才能证明主机侧真实可见。缺少最后一层证据时，不把仓库内路径写成硬件输出已验证。

## 10. 常见问题

### 1. 为什么 `printf()` 在裸机工程里需要 `_write()`？

触发条件：项目调用 `printf()`，但 MCU 没有操作系统终端。

可能原因：标准库只能完成格式化，底层输出设备需要工程提供。定位入口是 `syscalls.c` 的弱 `_write()` 和 `usart.c` 的强 `_write()`。

本章能证明的是“格式化结果最终走到哪条发送路径”。
它不能证明串口线已接好、主机终端已打开、或者当前调试会话一定看得见输出。

### 2. `syscalls.c` 已经有 `_write()`，为什么 `usart.c` 还要再写一个？

触发条件：读者看到两个同名 `_write()`。

可能原因：`syscalls.c` 的 `_write()` 是弱定义，默认只调用弱 `__io_putchar()`。
`usart.c` 的 `_write()` 是项目实际指定的 UART 输出路径。
定位入口是函数属性和 `#ifdef __GNUC__`。

也就是说，这不是重复定义，而是在当前构建条件下用更明确的实现覆盖默认弱实现。
教材记录的是“链接后实际生效的路径”，不是“文件里谁先出现”。

### 3. 为什么还保留 `fputc()`？

触发条件：项目同时存在 `_write()` 和 `fputc()`。

可能原因：不同标准库或配置可能通过不同字符输出入口实现 `printf()`。保留 `fputc()` 能增强兼容性。当前项目两条路径都指向 `huart3`。

这说明本章要看的不是某个函数名单独是否存在，而是它们是否最终汇到同一 UART 发送端。
如果后续移植时改变库配置，这种兼容路径就有可能变成必要证据。

### 4. 为什么浮点输出要看构建选项？

触发条件：`%f` 输出不正常，或输出为空、格式异常。

可能原因：newlib-nano 默认可能不拉入浮点格式化支持。当前项目通过 `.cproject` 和 `Debug/makefile` 中的 `-u _printf_float` 启用该能力。

如果不看构建选项，读者会以为浮点输出失败是串口或代码逻辑问题。
实际上，这类问题经常先落在链接选项上。

### 5. 10Hz 输出是否来自 UART 中断？

触发条件：读者把周期输出和中断发送混淆。

可能原因：10Hz 低频任务的触发依据是 `HAL_GetTick()` 与 `last_print_tick` 的差值；
输出动作是阻塞式 `HAL_UART_Transmit()`。
当前项目没有把本章这条 `printf()` 输出写成 UART 中断发送。

这意味着 10Hz 输出更适合当作调试观察入口，而不是高实时通信通道。
如果后续想把它改成非阻塞方式，需要重新评估调度和串口发送策略。

可选方案包括降低打印频率、缩短行长度、用环形缓冲区解耦生产者和发送者，或改成 UART IT/DMA 发送。它们能降低实时循环被阻塞的概率，但会引入缓冲区溢出、并发状态和发送完成回调等新问题。

### 6. 为什么本章不讲 USB CDC？

触发条件：`.ioc` 中也能看到 USB CDC 配置。

可能原因：USB CDC 属于后续 USB 章节。当前第11章的正式主线是 Newlib 和 USART3 调试输出，不能把 USB 通信支线提前展开。

这也是章节衔接的一部分：第11章说明“字符怎样从 `printf()` 出来”，
USB FS 设备与 USB CDC 的完整讲解位置是第15章和第16章，本章只说明当前 `printf()` 主线没有切到 USB。

从当前源码看，USB CDC 侧存在 `CDC_Transmit_FS()`，内部调用 `USBD_CDC_SetTxBuffer()` 和 `USBD_CDC_TransmitPacket()`。

但 `_write()` 和 `fputc()` 没有调用它，因此不能把 USB 虚拟串口当作当前 `printf()` 后端。
同样，`USB_DEVICE/Target/usbd_conf.h` 里的 `USBD_UsrLog()`、`USBD_ErrLog()` 和 `USBD_DbgLog()` 虽然内部也调用 `printf()`，
但它们调用到的仍是本章前面那条 Newlib -> `_write()` -> USART3 链路；这些宏只能证明 USB 中间件在借用现有调试通道，不证明 USB CDC 已成为输出后端。

### 7. `.map` / `.list` / `.su` / `.cyclo` 文件能证明什么？

触发条件：源码里同时存在 `syscalls.c` 的弱 `_write()` 和 `usart.c` 的强 `_write()`，读者仍不确定最终链接结果。

可能原因：源码阅读只能说明“强符号应该覆盖弱符号”，但最终固件由链接器决定。`.map` 文件能把符号落点展开为对象文件、地址和大小。

当前 Debug map 中，最终地址上的 `_write` 来自 `./Core/Src/usart.o`，并且 `_printf_float` 来自 `libc_nano.a`。这能证明当前 Debug 固件确实拉入了 USART3 输出重定向和浮点格式化支持。

`.list` 能继续把这条链路拆到反汇编层：`main()` 中能看到对 `MX_USART3_UART_Init()` 的调用，`_write_r` 中能看到跳转到项目 `_write()`，`Core/Src/usart.c` 的 `_write()` 中能看到循环调用 `HAL_UART_Transmit()`，`_sbrk_r` 中也能看到跳转到项目 `_sbrk()`。这比 `.map` 更接近“调用边是否存在”，但它仍然是构建后的静态证据，不是某次运行的串口日志。

`.su` 和 `.cyclo` 则提供另一类边界证据：当前构建显示 `usart.c` 的 `_write()` 静态栈估算为 32 字节、圈复杂度为 2，`fputc()` 静态栈估算为 16 字节、圈复杂度为 1，HAL 的 `HAL_UART_Transmit()` 静态栈估算为 48 字节、圈复杂度为 10。这些数字能帮助读者理解阻塞发送路径的静态复杂度和栈预算入口，但不能替代运行时最大栈水位，也不能证明实际输出没有超时或丢失。

边界是：这些构建产物只对应某一次构建配置。Release、不同工具链版本或清理后重新生成的产物，都需要重新检查对应 `.map` / `.list` / `.su` / `.cyclo`。

### 8. 启用了 `_scanf_float` 是否等于 UART 输入已经可用？

不等于。

`_scanf_float` 只控制 newlib-nano 是否拉入浮点扫描格式代码。真正从串口读字符还需要 `_read()` 或 `__io_getchar()` 最终落到 USART3 RX。

当前项目能看到 USART3 RX 引脚 PC11 被配置，也能看到 `syscalls.c` 中存在弱 `_read()`；但没有看到项目把 `__io_getchar()` 强实现为 UART 接收。因此本章只能说“构建配置启用了浮点 scanf 库能力”，不能说“标准输入已经通过 UART 工作”。

#### 10.8.1 `scanf()` 输入路径的构建证据边界

把当前证据拆开看，会发现它们证明的是不同层面的事实：

- `.cproject` 和 `Debug/makefile` 中的 `-u _scanf_float`：证明链接器被要求保留浮点扫描格式支持。
- `.map` 中的 `_scanf_float`：证明 `libc_nano.a` 的浮点扫描代码进入了当前 Debug ELF。
- `.map` 中的 `_read_r` 和 `_read`：证明 newlib 的重入包装入口会落到项目提供的 `_read()` 符号。
- `.list` 中 `_read_r` 对 `_read` 的跳转，以及 `_read()` 中逐字节写入 `*ptr++ = __io_getchar()` 的反汇编：证明当前输入底层仍停在弱 `_read()` / 弱 `__io_getchar()` 这一层。
- `usart.c` 中 PC11 配置为 `USART3_RX`：证明硬件引脚和外设方向具备 RX 配置前提。

这些证据合在一起，能说明“浮点 `scanf` 库代码存在，标准输入包装路径存在，USART3 RX 引脚也被配置”。但它们还不能证明“`scanf("%f", ...)` 已经能从 PC11 收到终端输入”，原因是当前源码没有看到强 `__io_getchar()`，也没有看到强 `_read()` 调用 `HAL_UART_Receive(&huart3, ...)` 或其它接收路径。

`Debug/Three-axis_cloud_platformV2.map` 中虽然还能看到 `HAL_UART_Receive`、`HAL_UART_Receive_IT` 和 `HAL_UART_Receive_DMA` 相关条目，但这类条目只能说明 HAL 库提供或参与了这些接收 API 的链接/输入集合，不能自动推出 newlib 标准输入已经调用它们。判断 `scanf()` 是否真正可用，需要在 `_read_r`、`_read`、`__io_getchar` 和 `HAL_UART_Receive` 等位置设置断点，并配合串口终端向 PC11 输入字符的硬件记录【待验证】。

### 9. map 中出现 `malloc` 和 `_sbrk_r`，是否说明每次 `printf()` 都会申请堆？

不说明。当前 map 可以证明浮点格式化相关库成员已经进入 ELF，例如 `_dtoa_r`、`malloc`、`_malloc_r` 和 `_sbrk_r`。这说明当前固件具备这些库路径，也说明 Flash 体积和潜在堆路径需要被纳入分析。

但 map 是链接结果，不是运行轨迹。某一次 `printf()` 是否真的进入堆分配路径，要看格式字符串、参数、库内部执行分支和现场状态。工程结论应写成“浮点格式化引入了可能触发堆路径的库代码”，不要直接写成“每次浮点打印都会 malloc”。要下运行结论，需要断点、日志、`_sbrk()` 调用记录或堆水位证据。

## 11. 实践任务

开始任务前，先回到本章第8节定位 `_write()`、`fputc()`、`_sbrk()` 和浮点打印链接证据；第9节提供 UART 输出断点观察顺序。

任务一：追踪 `printf()` 输出入口。

在 `Core/Src/usart.c` 中追踪 `printf()` 可能经过的两个输出入口：`_write()` 和 `fputc()`。
验收依据是输出路径表包含入口函数、最终发送函数和证据位置。

任务二：区分弱 `_write()` 与项目实现。

在 `Core/Src/syscalls.c` 中找出弱 `_write()`，并解释它为什么不是当前项目最具体的 UART 输出实现。
验收依据是符号关系表包含弱符号、强符号、覆盖关系和最终生效项。

任务三：追踪 `_sbrk()` 内存边界。

在 `Core/Src/sysmem.c` 中追踪 `_sbrk()` 的三个链接符号：`_end`、`_estack` 和 `_Min_Stack_Size`。
验收依据是边界表包含堆起点、栈顶、最小栈和边界结论。

任务四：确认浮点打印支持。

在 `.cproject` 和 `Debug/makefile` 中确认浮点打印支持。
验收依据是构建表记录 `-u _printf_float` 的出现位置和构建文件来源。

任务五：确认首次打印前的初始化顺序。

在 `main.c` 中找出第一次有效 `printf()` 之前的初始化顺序。
验收依据是初始化顺序表包含 UART 初始化和首次打印位置。

任务六：计算串口输出时间预算。

按 115200、8N1 计算 1 个字符、50 个字符和 100 个字符的理论线路发送时间。
验收依据是时间表能解释为什么调试打印应放在低频任务或被抽样输出。

任务七：用 map 文件确认链接落点。

在 `Debug/Three-axis_cloud_platformV2.map` 中查找 `_write`、`_printf_float`、`_scanf_float` 和 `_sbrk`。
验收依据是符号表能区分“源码中存在的弱定义”和“最终链接地址上的生效实现”。

任务八：区分输出能力和输入能力。

分别列出 `printf("%f")` 输出成立所需证据，以及 `scanf("%f")` 从 UART 输入成立所需证据。
验收依据是表格能说明为什么 `-u _scanf_float` 不是 UART 标准输入已经可用的充分条件。

任务九：区分浮点格式化符号拉入和运行执行。

在 `Debug/Three-axis_cloud_platformV2.map` 中追踪 `_printf_float`、`_dtoa_r`、`malloc`、`_malloc_r`、`_sbrk_r` 和项目 `_sbrk()`。
验收依据是证据表能分清“库成员进入 ELF”“可能影响 Flash/RAM/堆”和“某次运行实际进入堆路径”三类结论。

实践边界：

当前任务优先形成表格、链路图、搜索记录和计算过程。涉及 IDE 现场、构建日志、断点数值、外部波形、主机侧结果或硬件响应时，若没有截图、日志或仓库外实测证据，结论保持【待验证】。

## 12. 思考题

1. 如果删除 `usart.c` 中的强 `_write()`，`printf()` 可能会落到哪条路径？这条路径还缺少什么项目级输出实现？
2. 为什么 `_sbrk()` 属于 Newlib 适配的一部分，即使本章主要分析 UART 输出？
3. `HAL_UART_Transmit()` 阻塞发送对 500Hz 实时控制循环可能有什么影响？为什么项目把连续状态输出放在 10Hz 低频任务中更合理？
4. 如果 `%d` 输出正常而 `%f` 输出异常，应该优先检查源码、串口引脚还是构建选项？为什么？
5. 为什么 `mpu6050.c` 中的 `printf()` 能作为串口输出证据，却不能让本章提前展开 MPU6050 寄存器配置？
6. 如果串口终端没有输出，如何按“构建选项、重定向函数、USART 初始化、引脚映射、外部接线”分层排查？
7. 当前 `_write()` 忽略 `file` 参数和 HAL 返回值，这对错误诊断和 stdout/stderr 分流分别有什么影响？
8. 为什么 map 文件能增强“强 `_write()` 生效”的证据强度？它又为什么不能替代现场串口接收验证？
9. 为什么启用了 `-u _scanf_float`，仍不能把 `scanf("%f")` 当作已经接入 USART3 RX 的功能？
10. 如果要把当前阻塞式调试输出升级为可靠日志通道，返回值、缓冲区、重试和超时策略分别需要补哪些设计？
11. 为什么 map 中出现 `malloc` 和 `_sbrk_r`，仍不能证明每一次 `printf("%f")` 都发生了堆分配？

## 13. 本章总结

本章建立了 `printf()`、Newlib 系统调用适配、USART3 初始化和项目调试输出之间的完整证据链。

已经确认的结论是：

- `syscalls.c` 提供弱 `_write()`、弱 `_read()` 和多个最小系统调用入口。
- `sysmem.c` 用 `_sbrk()` 把 Newlib 堆限制在链接脚本定义的 RAM 边界内。
- `usart.c` 配置 USART3 为 115200、8N1、TX/RX，并通过 PC10/PC11 与 USART3 部分重映射完成硬件输出前提。
- 当前 Debug map 证明最终 `_write` 符号来自 `./Core/Src/usart.o`，浮点格式化符号来自 `libc_nano.a`。
- `usart.c` 中的强 `_write()` 和 `fputc()` 都通过 `HAL_UART_Transmit(&huart3, ...)` 输出字符。
- `HAL_UART_Transmit()` 是阻塞发送路径，内部等待 TXE 和 TC 标志，并通过 `gState` 管理 READY/BUSY_TX 状态。
- 当前 `_write()` 逐字节调用 `HAL_UART_Transmit(..., Size=1)`，会让 TC 等待和 `gState` 状态切换按字符重复出现。
- 115200、8N1 下单字符纯线路时间约 86.8us，长日志可能明显占用控制循环时间预算。
- 36MHz APB1 与 115200 baud 会经过 BRR 量化，当前计算误差约为 +0.16%，需要理解为时钟和寄存器分辨率共同决定。
- `.cproject` 和 `Debug/makefile` 启用了 newlib-nano 浮点格式化支持。
- 当前 Debug map 显示浮点格式化相关库成员会拉入 `_dtoa_r`、`malloc`、`_malloc_r` 和 `_sbrk_r` 等符号。
- `-u _scanf_float` 只说明浮点扫描库能力被链接，不证明 UART 标准输入已经实现。
- `main.c`、`mpu6050.c` 和 `mpu6050Calibration.c` 中的 `printf()` 是项目调试观察的直接证据。
- `.map/.list/.su/.cyclo` 的构建产物结论统一回到第8节、第10.7节到第10.9节判断：它们能证明 `_write` 重定向、浮点格式化库成员、`_write_r -> _write -> HAL_UART_Transmit()`、`_sbrk_r -> _sbrk()`、静态栈和圈复杂度条目进入某次 Debug 构建，但不能替代某次运行的串口输出、堆峰值、栈水位、终端日志或 PC10 波形证据。

本章待验证分类：

| 类别 | 已由本章证明 | 仍保持【待验证】 |
|---|---|---|
| 构建验证 | `.map/.list/.su/.cyclo` 能证明 `_write` 重定向、浮点格式化库成员、`_write_r -> _write -> HAL_UART_Transmit()`、`_sbrk_r -> _sbrk()`、静态栈和圈复杂度条目进入某次 Debug 构建。 | 构建产物不能替代某次运行的串口输出、堆峰值、栈水位、终端日志或 PC10 波形证据。 |
| 软件验证 | 源码能证明当前 `printf()` 通过强 `_write()` 重定向到 USART3，并调用阻塞式 `HAL_UART_Transmit()`。 | 不能证明 USB CDC 已承担调试输出主线，也不能把 UART 软件路径写成主机端已收到完整日志。 |
| 参数验证 | 115200、8N1、36MHz APB1 与 BRR 量化误差约 +0.16% 的计算口径已经列清。 | 终端参数、实际波特率、长日志占用控制循环时间和阻塞发送耗时仍需运行记录或外部测量。 |
| 硬件验证 | `.ioc`、`usart.c` 和 HAL 初始化能证明 USART3、PC10/PC11 和阻塞发送路径配置。 | 串口输出通路成立仍依赖目标板接线、USB 转串口、PC10 波形、终端接收和完整日志记录。 |
| 官方资料待确认 | `.cproject` 和 `Debug/makefile` 启用 newlib-nano 浮点格式化支持，`.map` 显示 `_dtoa_r`、`malloc`、`_malloc_r`、`_sbrk_r` 等库成员进入链接。 | `-u _scanf_float` 只说明浮点扫描库能力被链接，不证明 UART 标准输入已经实现；newlib 堆行为仍需结合运行证据确认。 |
| 实验待完成 | 本章已经把输出重定向、UART 后端、浮点格式化、堆入口和外部接收拆成可观察对象。 | 后续需记录断点、堆顶变化、`_sbrk()` 返回值、运行水位、堆峰值、栈水位、终端日志、PC10 波形和阻塞耗时。 |

下一章可以进入通用定时器 PWM 输出。到这里，读者已经理解了项目如何观察内部状态；后续需要理解项目如何把控制结果输出到电机驱动相关的定时器通道。

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

- Newlib系统调用适配
- UART串口调试输出

项目证据：

- `Core/Src/syscalls.c`
- `Core/Src/sysmem.c`
- `Core/Src/usart.c`
- `Core/Src/main.c`
- `Three-axis_cloud_platformV2.ioc`
- `.cproject`
- `Debug/makefile`
- `Debug/Three-axis_cloud_platformV2.map`
- `Debug/Three-axis_cloud_platformV2.list`
- `Debug/Core/Src/usart.su`
- `Debug/Core/Src/usart.cyclo`
- `Debug/Core/Src/syscalls.su`
- `Debug/Core/Src/syscalls.cyclo`
- `Debug/Core/Src/sysmem.su`
- `Debug/Core/Src/sysmem.cyclo`
- `Debug/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.su`
- `Debug/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.cyclo`
- `Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c`
- `Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_uart.h`
- `Drivers/CustomDrivers/Src/mpu6050.c`
- `Drivers/CustomDrivers/Src/mpu6050Calibration.c`

外部权威资料：

- Sourceware Newlib C Library manual: `https://sourceware.org/newlib/libc.html`
- ST RM0008 Reference manual: `https://www.st.com/resource/en/reference_manual/rm0008-stm32f101xx-stm32f102xx-stm32f103xx-stm32f105xx-and-stm32f107xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf`
- ST STM32F103 documentation page: `https://www.st.com/en/microcontrollers-microprocessors/stm32f103/documentation.html`

引用的函数、配置项和变量：

- `_write()`
- `_write_r()`
- `_read()`
- `_sbrk()`
- `_sbrk_r()`
- `_printf_float`
- `_scanf_float`
- `_dtoa_r`
- `malloc`
- `_malloc_r`
- `_fstat()`
- `_isatty()`
- `fputc()`
- `printf()`
- `MX_USART3_UART_Init()`
- `HAL_UART_Init()`
- `HAL_UART_MspInit()`
- `HAL_UART_Transmit()`
- `UART_FLAG_TXE`
- `UART_FLAG_TC`
- `HAL_UART_STATE_READY`
- `HAL_UART_STATE_BUSY_TX`
- `UART_BRR_SAMPLING16`
- `HAL_RCC_GetPCLK1Freq()`
- `__HAL_AFIO_REMAP_USART3_PARTIAL()`
- `HAL_GetTick()`
- `Debug/Three-axis_cloud_platformV2.map`
- `huart3`
- `huart3.gState`
- `HAL_BUSY`
- `HAL_TIMEOUT`
- `last_print_tick`
- `return_state_count_roll`
- `sensors.margAttitude500Hz`
- `PC10.Signal=USART3_TX`
- `PC11.Signal=USART3_RX`
- `USART3.VirtualMode=VM_ASYNC`
- `-u _printf_float`
- `-u _scanf_float`

质量自检：

- P0 事实错误：通过。
- P1 依赖断层：通过。
- P2 逻辑连贯：通过。
- P3 项目证据：通过。
- P4 原理展开：通过。
- P5 调试实践：通过。
- P6 表达统一：通过。

---
> 导航：上一章：[第10章_DWT计时与微秒时间基准](第10章_DWT计时与微秒时间基准.md) ｜ 下一章：[第12章_通用定时器PWM输出](第12章_通用定时器PWM输出.md)
