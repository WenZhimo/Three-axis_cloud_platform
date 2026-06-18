# 第07章 MSP初始化与GPIO复用

## 1. 本章目标

- 理解 HAL MSP 初始化在外设初始化链路中的位置。
- 区分普通 GPIO 输出、复用推挽输出、复用开漏输出和输入配置。
- 能从 `.ioc`、`MX_GPIO_Init()`、各外设 MSP 函数和 `HAL_TIM_MspPostInit()` 追踪项目引脚配置。
- 为后续 AFIO、定时器 PWM、I2C、UART 和 USB 章节建立引脚与底层初始化前提。

## 2. 前置知识

- HAL驱动模块裁剪
- STM32 HAL驱动库
- 系统时钟树

这些前置主要对应第04章和第06章。

本章内部顺序为：先讲 `HAL MSP初始化机制`，再讲 `GPIO输出与复用`。

因为 GPIO 复用配置通常由具体外设的 MSP 或 PostInit 函数承载。读者必须先知道 MSP 是什么，再判断某个引脚为什么出现在 `gpio.c`、`tim.c`、`i2c.c` 或 `usart.c` 中。

## 3. 问题背景

第06章已经确认系统时钟。接下来要回答的问题是：外设初始化时，谁负责把外设时钟、GPIO 端口时钟、引脚模式和必要的底层资源准备好？

在三轴云台项目中，不能只看 `MX_TIMx_Init()`、`MX_I2C2_Init()` 或 `MX_USART3_UART_Init()` 的外设参数。外设要真正接到芯片引脚，还需要底层初始化：

- GPIO 端口时钟必须先打开。
- 引脚必须配置成输出、复用输出、复用开漏或输入。
- 某些全局底层资源必须在 `HAL_MspInit()` 中提前准备。
- 定时器 PWM 引脚配置分布在 `HAL_TIM_MspPostInit()` 中。
- USB 的 PCD 底层初始化位于 USB Device 支持文件中。

因此第07章的任务不是讲每个外设协议，而是建立“HAL 初始化如何落到引脚和底层资源”的项目索引。

## 4. 核心概念

- HAL MSP初始化机制：HAL 外设初始化前后调用的底层支持函数集合，用于配置外设时钟、GPIO、中断等硬件支撑资源。
- MSP：MCU Support Package，可理解为“让 HAL 外设对象落到具体 MCU 硬件资源上的支持层”。
- GPIO输出与复用：普通 GPIO 输出和外设复用功能引脚配置的统称。
- 普通 GPIO 输出：由软件直接控制电平的输出模式，本项目 PB12 属于该类。
- 复用推挽输出：由片上外设驱动引脚输出，本项目 TIM PWM 引脚和 USART3_TX 属于该类。
- 复用开漏输出：适合总线线与结构的复用输出，本项目 I2C2_SCL 和 I2C2_SDA 属于该类。
- 输入配置：引脚作为输入采样，本项目 USART3_RX 使用输入模式。
- PostInit：部分外设初始化完成后，再补充 GPIO 复用配置的阶段，本项目定时器 PWM 引脚通过 `HAL_TIM_MspPostInit()` 配置。
- GPIO 端口时钟：让 GPIOA/GPIOB/GPIOC/GPIOD 的配置寄存器可以被访问的时钟门控。
- 外设时钟：让 TIM、I2C、USART、USB 等外设模块本身可以工作的时钟门控。
- AFIO 时钟：让复用重映射、调试接口配置和 EXTI 等 AFIO 相关寄存器可以被访问的时钟门控。
- NVIC 使能：让某个中断线具备进入 Cortex-M 中断控制器的路径，不等于该中断已经发生。
- GPIO 输出速度：HAL 对 F1 GPIO 输出驱动能力/最大输出翻转能力的配置，不等于 PWM 或串口的实际频率。

这些概念服务于 `HAL MSP初始化机制` 和 `GPIO输出与复用`，不新增结构外知识点。

## 5. 工作原理

HAL 外设初始化通常不是一个单层函数。以项目中多个外设为例，初始化可以拆成三层：

1. CubeMX 生成的 `MX_*_Init()` 配置外设句柄和外设参数。
2. HAL 初始化函数内部或相关阶段调用 MSP 函数。
3. MSP 函数启用底层时钟、配置 GPIO 引脚、配置必要的中断或其他底层资源。

这三层要分清“谁描述参数”和“谁打开硬件通路”：

```text
MX_*_Init()
-> HAL_*_Init()
-> HAL_*_MspInit()
-> RCC clock / GPIO mode / AFIO remap / NVIC route
```

`MX_*_Init()` 中的句柄参数说明外设希望怎样工作。MSP 函数说明这些外设怎样接入当前 MCU 的端口、时钟门和中断入口。两者相互补充，但不能互相替代。

GPIO 配置也不是都在 `gpio.c` 里完成。项目中存在两类入口：

- 普通 GPIO 输出由 `MX_GPIO_Init()` 配置，例如 PB12。
- 外设复用引脚由对应外设文件中的 MSP 或 PostInit 配置，例如 TIM、I2C、UART。

这就是为什么第07章必须同时看多个文件。只看 `gpio.c`，会漏掉 PWM、I2C 和 UART 的引脚；只看 `tim.c`，又会漏掉普通 GPIO 输出和 I2C、UART、USB 的底层入口。

从项目证据看，底层初始化链路可以概括为：

- `stm32f1xx_hal_msp.c` 提供全局 MSP 初始化。
- `gpio.c` 配置普通 GPIO 输出。
- `tim.c` 在定时器 PostInit 中配置 PWM 复用引脚。
- `i2c.c` 在 I2C MSP 中配置 I2C2 引脚和外设时钟。
- `usart.c` 在 UART MSP 中配置 USART3 引脚和外设时钟。
- `usbd_conf.c` 在 PCD MSP 中配置 USB 底层时钟和中断入口。
- `.ioc` 提供 CubeMX 引脚分配源头，用于核对生成代码与配置来源是否一致。

本章复查时还要建立证据层级：

```text
.ioc 配置意图
-> 生成代码中的 MSP/GPIO 调用
-> HAL 驱动内部对 CRL/CRH、RCC、AFIO、NVIC 的写入
-> 运行时寄存器观察或外部测量【待验证】
```

前三层属于仓库内证据。第四层需要调试器、逻辑分析仪、示波器、主机枚举记录或硬件连线资料，仓库当前没有这些实测证据。

判断一个引脚“准备好”时，不能只看某一行 `HAL_GPIO_Init()`。本章使用下面的最小核对清单：

- GPIO 端口时钟是否打开，例如 GPIOA/GPIOB/GPIOC。
- GPIO 模式是否匹配用途，例如普通输出、复用推挽、复用开漏或输入。
- 如果涉及重映射，AFIO 时钟和 `AFIO->MAPR` 位是否有代码证据。
- 外设自身时钟是否打开，例如 TIM、I2C、USART、USB。
- 外设是否已经启动输出或数据通路，例如 PWM Start、UART Transmit、USB Start。
- 外部电气条件是否有证据，例如上拉电阻、负载、线序、ACK、波形或主机枚举。

前五项可以主要从仓库代码追踪，第六项通常必须靠硬件资料或实测记录补齐。

## 6. STM32实现机制

在 STM32 HAL 中，MSP 函数通常采用弱定义可覆盖机制。用户工程提供同名函数后，HAL 初始化流程会调用用户工程中的实现。

本项目中可以确认的 MSP 入口包括：

- `HAL_MspInit()`：全局 MSP 初始化。
- `HAL_TIM_Base_MspInit()`：定时器基础 MSP 初始化。
- `HAL_TIM_PWM_MspInit()`：定时器 PWM MSP 初始化。
- `HAL_TIM_MspPostInit()`：定时器 GPIO 复用后置初始化。
- `HAL_I2C_MspInit()`：I2C MSP 初始化。
- `HAL_UART_MspInit()`：UART MSP 初始化。
- `HAL_PCD_MspInit()`：USB PCD MSP 初始化。

HAL 驱动源码中可以看到更底层的触发关系：

```text
HAL_Init()
-> HAL_InitTick()
-> HAL_MspInit()

HAL_I2C_Init()
-> HAL_I2C_MspInit()

HAL_UART_Init()
-> HAL_UART_MspInit()

HAL_PCD_Init()
-> HAL_PCD_MspInit()
```

定时器稍微特殊。本项目 TIM2、TIM4、TIM8 先调用 `HAL_TIM_Base_Init()`，
因此它们的时钟门控在 `HAL_TIM_Base_MspInit()` 中打开；TIM3 直接走
`HAL_TIM_PWM_Init()`，因此它的时钟门控在 `HAL_TIM_PWM_MspInit()` 中打开。
所有 PWM 输出引脚最终仍通过 `HAL_TIM_MspPostInit()` 配置到 GPIO 复用输出。

`HAL_MspInit()` 启用 AFIO 和 PWR 时钟，并进行调试接口相关配置。AFIO 重映射和 SWD 保留会在第08章展开，本章只确认它属于全局 MSP 初始化的一部分。

GPIO 配置通过 `GPIO_InitTypeDef` 承载，关键字段包括：

- `Pin`：选择具体引脚。
- `Mode`：选择普通输出、复用推挽、复用开漏或输入等模式。
- `Pull`：选择上拉、下拉或无上下拉。
- `Speed`：选择输出速度等级。

在 STM32F1 HAL GPIO 实现中，`HAL_GPIO_Init()` 会把这些字段转换为 GPIO `CRL`
或 `CRH` 寄存器中的 `MODEy[1:0]` 与 `CNFy[1:0]` 配置。低 8 个引脚进入
`CRL`，高 8 个引脚进入 `CRH`。因此 `GPIO_MODE_AF_PP`、`GPIO_MODE_AF_OD`
等宏不是抽象标签，最终会落到 F1 的端口配置寄存器。

进一步拆开看，F1 的每个 GPIO 引脚在 `CRL/CRH` 中占 4 bit：

```text
MODEy[1:0]：输出速度或输入模式
CNFy[1:0]：通用输出、复用输出、开漏、浮空输入、上下拉输入等配置
```

HAL 源码按引脚号选择目标寄存器：0 到 7 号脚写 `CRL`，8 到 15 号脚写 `CRH`。
偏移量按每脚 4 bit 计算，例如 PB12 写入 `GPIOB->CRH` 的
`(12 - 8) * 4 = 16` 位偏移，PC10 写入 `GPIOC->CRH` 的
`(10 - 8) * 4 = 8` 位偏移。这个位级关系能帮助读者把
`GPIO_InitStruct.Mode` 和寄存器窗口中的实际值对应起来。

`HAL_GPIO_WritePin()` 也不是直接读改写 `ODR`。HAL GPIO 源码说明它通过
`GPIOx->BSRR` 完成置位或复位：低 16 bit 用于置位，高 16 bit 用于复位。
工程意义是单次写寄存器即可改变输出锁存状态，避免在读改写过程中被中断打断。
但这只能证明 MCU 输出锁存被请求改变，不能证明外部引脚电压、LED 亮灭或负载状态一定正确。

本章只解释这些字段如何在项目中承载引脚配置，不展开各外设协议或 PWM 参数。

## 7. 项目中的应用

本章对应项目初始化流程中的底层硬件连接层。

直接相关文件：

- `Core/Src/stm32f1xx_hal_msp.c`
- `Core/Src/gpio.c`
- `Core/Src/tim.c`
- `Core/Src/i2c.c`
- `Core/Src/usart.c`
- `USB_DEVICE/Target/usbd_conf.c`
- `Three-axis_cloud_platformV2.ioc`

文件之间的关系是：

- `.ioc` 记录引脚分配源头。
- `gpio.c` 配置 PB12 普通输出。
- `tim.c` 配置 TIM2、TIM3、TIM4、TIM8 的 PWM 复用引脚。
- `i2c.c` 配置 PB10/PB11 为 I2C2 复用开漏引脚。
- `usart.c` 配置 PC10/PC11 为 USART3 发送和接收引脚。
- `usbd_conf.c` 配置 USB PCD 底层时钟和中断支持。
- `stm32f1xx_hal_msp.c` 提供全局 MSP 初始化入口。

在项目主流程中，`main()` 先完成系统时钟配置，再调用一组外设初始化函数：

```text
MX_GPIO_Init()
MX_TIM3_Init()
MX_TIM2_Init()
MX_TIM4_Init()
MX_I2C2_Init()
MX_TIM8_Init()
MX_USART3_UART_Init()
MX_USB_DEVICE_Init()
MX_TIM6_Init()
```

这些外设初始化会进一步触发或依赖对应 MSP 和 GPIO 配置。

## 8. 代码分析

### 1. 全局 MSP 初始化

`Core/Src/stm32f1xx_hal_msp.c` 中的 `HAL_MspInit()` 是全局底层初始化入口。

调用链来自 `HAL_Init()`：项目 `main()` 先调用 `HAL_Init()`，HAL 驱动随后调用用户工程覆盖的 `HAL_MspInit()`。因此全局 MSP 发生在 `SystemClock_Config()` 和各 `MX_*_Init()` 之前。

它启用 AFIO 和 PWR 时钟，并执行调试接口相关配置。本章只确认这些动作存在。具体为什么关闭 JTAG 并保留 SWD、USART3 为什么需要部分重映射，留到第08章分析。

工程含义是：AFIO/PWR 这类全局底层资源不是某一个外设独有的配置。若 AFIO 时钟没有打开，后续涉及 AFIO 的重映射或调试接口配置就缺少寄存器访问前提。

### 2. 普通 GPIO 输出

`Core/Src/gpio.c` 中的 `MX_GPIO_Init()` 先启用 GPIOD、GPIOA、GPIOB、GPIOC 端口时钟。

随后它先将 PB12 输出复位，再把 PB12 配置为：

- `GPIO_MODE_OUTPUT_PP`
- `GPIO_PULLUP`
- `GPIO_SPEED_FREQ_LOW`

`Core/Inc/gpio.h` 将 `LED1_GPIO` 定义为 `GPIOB`，将 `LED1_PIN` 定义为
`GPIO_PIN_12`。`main.c` 在 MPU6050 初始化成功分支中调用
`HAL_GPIO_WritePin(LED1_GPIO, LED1_PIN, GPIO_PIN_SET)`。

因此仓库内可以确认：PB12 是被软件直接控制的普通推挽输出，并被项目代码作为 `LED1` 输出使用。板上是否真的连接 LED、电平是否有效、亮灭逻辑是否与硬件一致，仍属于【待验证】。

从寄存器落点看，PB12 属于 8 到 15 号脚，因此 `HAL_GPIO_Init(GPIOB, ...)`
最终配置的是 `GPIOB->CRH`，不是 `GPIOB->CRL`。初始化前的
`HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)` 会通过 `BSRR`
复位半区预先清除 PB12 的输出锁存位，随后再把 PB12 配置成推挽输出。
这种顺序有助于减少引脚从复位态切到输出态时出现非预期高电平的风险。

还要注意 `Core/Inc/gpio.h` 中保留了 `LED2_PIN GPIO_PIN_3` 和
`LED2_GPIO GPIOA`，而 `.ioc` 与 `tim.c` 当前把 PA3 用作 TIM2_CH4。
所以教材不能只凭旧宏名称判断 PA3 是 LED；当前行为证据应以 `.ioc`
和生成代码中的 TIM2 复用配置为准。

### 3. 定时器 PWM 复用引脚

`Core/Src/tim.c` 中的 `HAL_TIM_MspPostInit()` 按定时器实例分支配置 PWM 复用引脚。

这里有一个容易漏掉的细节：定时器“外设时钟使能”和“PWM GPIO 复用配置”分属不同入口。

- TIM2、TIM4、TIM8 的外设时钟在 `HAL_TIM_Base_MspInit()` 中使能。
- TIM3 的外设时钟在 `HAL_TIM_PWM_MspInit()` 中使能。
- TIM2、TIM3、TIM4、TIM8 的 PWM 引脚统一在 `HAL_TIM_MspPostInit()` 中配置。

项目中可以确认的映射包括：

- TIM2：PA1、PA2、PA3。
- TIM3：PA6、PA7、PB0、PB1。
- TIM4：PB8、PB9。
- TIM8：PC6、PC7、PC8。

这些引脚均配置为 `GPIO_MODE_AF_PP` 和 `GPIO_SPEED_FREQ_LOW`。这说明引脚输出控制权交给定时器复用功能，但还不能证明 PWM 已经启动。

PWM 是否真正输出，还需要后续确认 `HAL_TIM_PWM_Start()`、CCR 更新、MOE/高级定时器输出使能、负载连接和实测波形。本章只确认定时器通道如何连接到 GPIO 复用输出，不展开 PWM 频率、占空比、相位或电机控制。

复用推挽输出还有一个调试边界：当引脚被配置为 `GPIO_MODE_AF_PP` 后，
引脚电平主要由定时器输出比较单元驱动，而不是由普通 `GPIOx->ODR`
路径直接决定。除非先停用外设并重新配置为普通 GPIO，否则不应把
`HAL_GPIO_WritePin()` 当作验证 PWM 引脚输出的主要方法。

TIM8 还在 `HAL_TIM_Base_MspInit()` 中配置 `TIM8_UP_IRQn` 优先级并使能中断。这个动作只证明 NVIC 路由前提存在，不证明 TIM8 更新中断已经产生，也不证明 500Hz 控制循环由 TIM8 驱动。

### 4. I2C2 复用开漏引脚

`Core/Src/i2c.c` 中的 `HAL_I2C_MspInit()` 在实例为 I2C2 时启用 GPIOB 时钟，并配置：

- PB10：I2C2_SCL。
- PB11：I2C2_SDA。

这两个引脚使用 `GPIO_MODE_AF_OD` 和高速输出设置。复用开漏模式与 I2C 总线电气行为有关，但 I2C 协议、时序和 MPU6050 通信留到后续章节。

从 HAL 驱动注释和本项目生成代码看，I2C MSP 的最小链路是：

```text
MX_I2C2_Init()
-> HAL_I2C_Init()
-> HAL_I2C_MspInit()
-> GPIOB clock + PB10/PB11 AF open-drain + I2C2 clock
```

`GPIO_SPEED_FREQ_HIGH` 不是 100 kHz I2C 速率本身。I2C 速率来自
`hi2c2.Init.ClockSpeed = 100000` 以及 PCLK1 条件；GPIO speed 只是端口输出配置的一部分。
外部上拉电阻、总线电容、器件地址和 ACK 仍需第14章与实测证据验证。

本项目的 I2C2 MSP 中没有显式设置 `GPIO_InitStruct.Pull`。由于
`GPIO_InitTypeDef GPIO_InitStruct = {0}`，而 HAL 中 `GPIO_NOPULL` 的值为
`0x00000000u`，仓库内证据只能说明 PB10/PB11 使用复用开漏且未启用内部上下拉。
I2C 是否有合适上拉，必须来自原理图、板卡资料或示波器/逻辑分析仪测量，不能由当前 GPIO 初始化代码直接推出。

### 5. USART3 引脚

`Core/Src/usart.c` 中的 `HAL_UART_MspInit()` 在实例为 USART3 时启用 USART3 和 GPIOC 时钟。

项目配置为：

- PC10：USART3_TX，复用推挽输出。
- PC11：USART3_RX，输入模式，无上下拉。

同一函数中还存在 USART3 相关重映射调用。该调用属于第08章 AFIO 重映射与 SWD 调试的主线，本章只记录它位于 UART MSP 初始化中。

项目里 USART3 的链路为：

```text
MX_USART3_UART_Init()
-> HAL_UART_Init()
-> HAL_UART_MspInit()
-> USART3 clock + GPIOC clock + PC10/PC11 + USART3 partial remap
```

`GPIO_SPEED_FREQ_HIGH` 用在 PC10 发送脚，说明 TX 是高速复用推挽输出配置；
PC11 接收脚使用 `GPIO_MODE_INPUT` 和 `GPIO_NOPULL`。这只能证明 MCU 侧引脚模式，
不能证明主机或 USB 转串口工具已经收到 115200 baud 数据。

PC11 的 `GPIO_NOPULL` 还意味着仓库内没有内部上拉维持 RX 空闲电平的证据。
USART 空闲高电平是否稳定，取决于外部连接的串口转换器、线缆、对端驱动和地线参考。
因此串口章节需要继续验证波特率、重映射、对端连接和实际接收日志。

### 6. USB PCD MSP

`USB_DEVICE/Target/usbd_conf.c` 中的 `HAL_PCD_MspInit()` 在实例为 USB 时启用 USB 外设时钟，并配置 USB 相关中断入口。

USB DM/DP 引脚在 `.ioc` 中记录为 PA11/PA12。PCD MSP 中还可以确认：

- `__HAL_RCC_USB_CLK_ENABLE()` 打开 USB 外设时钟门。
- `HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0, 0)` 配置 USB 低优先级中断优先级。
- `HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn)` 使能 USB 中断入口。

这些动作只建立 USB 设备栈的底层入口。USB Device 中间件、端点、描述符、CDC 数据流、主机枚举和 Windows/Linux 设备管理器现象不在第07章展开。

当前 `HAL_PCD_MspInit()` 没有像 TIM/I2C/USART 那样调用
`HAL_GPIO_Init()` 配置 PA11/PA12 为普通复用输出。对第07章而言，USB
引脚证据主要来自 `.ioc` 的固定 USB_DM/USB_DP 分配，以及 PCD MSP
对 USB 外设时钟和中断入口的配置。不要把 USB PA11/PA12 误写成项目中显式的 GPIO AF_PP 初始化。

### 7. `.ioc` 与生成代码的关系

`.ioc` 中记录了 PA1、PA2、PA3、PA6、PA7、PB0、PB1、PB8、PB9、PC6、PC7、PC8、PB10、PB11、PB12、PC10、PC11、PA11、PA12 等引脚功能。

生成代码中的 `HAL_GPIO_Init()` 调用应与 `.ioc` 中的引脚分配相互印证。教材分析时要同时看配置源头和生成代码，避免只凭函数名判断引脚用途。

本章复查采用三列证据法：

| 层级 | 作用 | 项目例子 |
| --- | --- | --- |
| `.ioc` | CubeMX 配置意图 | `PC10.Signal=USART3_TX` |
| 生成代码 | 实际编译进入工程的初始化 | `HAL_GPIO_Init(GPIOC, &GPIO_InitStruct)` |
| HAL/CMSIS | 宏和寄存器语义 | `AFIO_MAPR_USART3_REMAP_PARTIALREMAP` |

如果三列不一致，优先以实际编译代码为当前行为证据，再回到 `.ioc` 判断是否需要重新生成或手工修正。

### 本节证据边界

本节只根据当前仓库说明文件、函数、宏、变量和调用关系。运行时频率、外部硬件表现、主机侧现象、传感器方向、电机响应或真实控制效果仍需调试记录、日志或仓库外实测证据；缺少证据时保持【待验证】。

## 9. 调试方法

本章阶段的调试目标是确认底层初始化入口和引脚模式是否与项目配置一致。

可观察对象：

- `.ioc` 中目标引脚是否分配到预期外设或 GPIO 输出。
- `MX_GPIO_Init()` 是否启用对应 GPIO 端口时钟。
- `MX_GPIO_Init()` 是否配置 PB12 的输出模式、上拉和速度。
- `GPIOx->CRL/CRH` 中目标引脚 4 bit 字段是否与 `MODE/CNF` 预期一致。
- `GPIOx->ODR/IDR` 是否与软件请求和外部电平观察一致。
- `RCC->APB2ENR` 和 `RCC->APB1ENR` 中相关 GPIO、AFIO、TIM、I2C、USART、USB 时钟门控是否符合预期。
- `AFIO->MAPR` 是否包含 USART3 部分重映射和 SWJ 配置的实际位值。
- `HAL_TIM_MspPostInit()` 是否配置 TIM2/TIM3/TIM4/TIM8 对应 PWM 复用引脚。
- `HAL_I2C_MspInit()` 是否配置 PB10/PB11 为复用开漏。
- `HAL_UART_MspInit()` 是否配置 PC10/PC11。
- `HAL_PCD_MspInit()` 是否存在 USB 底层时钟和中断支持。
- `HAL_TIM_Base_MspInit()` 是否为 TIM8 配置 `TIM8_UP_IRQn`，并注意这只是中断入口前提。

常见异常定位：

- 引脚没有输出：先检查 GPIO 端口时钟是否启用，再检查 `HAL_GPIO_Init()` 是否配置对应引脚。
- PWM 引脚无波形：先检查 `HAL_TIM_MspPostInit()` 是否配置对应复用引脚，再进入定时器章节检查 PWM 参数。
- I2C 通信异常：先检查 PB10/PB11 是否为复用开漏，再进入 I2C 章节分析总线和设备通信。
- 串口无输出：先检查 PC10/PC11 配置，再到后续 UART 章节分析波特率和重映射。
- USB 不枚举：先确认 `.ioc` 中 PA11/PA12 为 USB 引脚，并确认 PCD MSP 入口存在，再进入 USB 章节分析中间件。

建议按下面顺序排查，而不是直接怀疑业务算法：

```text
1. .ioc 引脚分配是否符合预期
2. 生成代码中端口时钟是否使能
3. GPIO Mode / Pull / Speed 是否符合外设类型
4. 外设时钟是否在 MSP 中使能
5. AFIO remap 或 SWJ 配置是否影响目标引脚
6. NVIC 是否只建立入口，还是已有中断源触发证据
7. 运行时寄存器、波形、主机枚举或设备 ACK 是否有实测证据【待验证】
```

本章能够证明的是工程配置和生成代码中的引脚模式。外部电平、连接线、上拉电阻、目标设备响应和实际波形都需要仓库外实测或硬件资料支撑；缺少这些证据时，外部电气结论保持【待验证】。

调试记录建议：

- 记录引脚名、`.ioc` 功能、生成代码入口、GPIO 模式和证据边界。
- 对外部电平、波形、连线和设备响应，单独放入仓库外实测栏。
- 对跨章节问题，只在本章确认 MSP/GPIO 入口，参数和协议细节转入后续章节。

## 10. 常见问题

### 1. 为什么 GPIO 配置不都在 `gpio.c` 里？

因为普通 GPIO 输出通常在 `MX_GPIO_Init()` 中配置，而外设复用引脚往往由外设自己的 MSP 或 PostInit 函数配置。项目中的 PWM、I2C 和 UART 引脚都不只依赖 `gpio.c`。

如果只看 `gpio.c`，会漏掉由 `HAL_TIM_MspPostInit()`、`HAL_I2C_MspInit()`、
`HAL_UART_MspInit()` 等函数配置的复用引脚。
教材要求读者同时看配置源、生成代码和 MSP 入口，
就是为了避免漏掉外设和引脚之间的连接证据。

### 2. MSP 和 `MX_*_Init()` 是什么关系？

`MX_*_Init()` 配置外设句柄和外设参数；MSP 负责把外设连接到 MCU 底层资源，例如 GPIO、外设时钟和中断。二者合起来才构成完整初始化。

可以把 `MX_*_Init()` 看作外设“工作参数”的配置，把 MSP 看作外设“接入芯片资源”的配置。缺少任一侧，教材都不能说该外设初始化链路完整。

### 3. 为什么定时器 PWM 引脚在 `HAL_TIM_MspPostInit()` 里？

因为定时器 PWM 的 GPIO 复用配置在定时器基本参数配置之后完成。本章只确认 PostInit 位置，PWM 模式和通道参数留到定时器章节。

这意味着调试 PWM 时不能只看 `MX_TIMx_Init()` 里的 PSC、ARR、通道配置，还要看 PostInit 是否把对应引脚配置为复用推挽输出。第07章建立这个入口，第12章再把它接到 PWM 输出链路。

### 4. 为什么 I2C 引脚使用复用开漏？

项目代码明确把 PB10/PB11 配置为 `GPIO_MODE_AF_OD`。这与 I2C 总线行为有关，但本章不展开 I2C 电气和协议细节。

本章能证明的是仓库内 GPIO 模式配置。外部上拉、电源、线序和设备响应不能由这一行代码直接推出，需要第14章和仓库外实测证据继续验证。

### 5. USART3 的重映射为什么不在本章展开？

因为 AFIO 重映射与 SWD 调试已经安排在第08章。本章只确认它出现在 UART MSP 初始化中，避免提前跨章节讲解。

第07章回答“USART3 的 GPIO 模式在哪里配置”，第08章回答“USART3 为什么能映射到 PC10/PC11”。两个问题相邻但不相同，分开讲可以让读者更容易追踪证据。

### 6. USB 的引脚为什么主要从 `.ioc` 看？

项目的 USB PCD MSP 配置底层时钟和中断，PA11/PA12 的设备功能则在 `.ioc` 中记录。USB 中间件和引脚固定功能细节留到后续 USB 章节。

因此，第07章不能把 USB 主机识别、虚拟串口或通信结果写成已验证事实。本章只保留引脚与底层初始化证据，真正的 USB 初始化链路和主机侧现象要到第15章、第16章处理。

### 7. 为什么 TIM3 的 MSP 入口和 TIM2/TIM4/TIM8 不同？

因为生成代码的初始化顺序不同。TIM2、TIM4、TIM8 先调用 `HAL_TIM_Base_Init()`，
所以时钟门控落在 `HAL_TIM_Base_MspInit()`；TIM3 直接调用 `HAL_TIM_PWM_Init()`，
所以时钟门控落在 `HAL_TIM_PWM_MspInit()`。

这不是功能高低之分，而是 HAL 状态机和 CubeMX 生成路径造成的入口差异。读代码时要跟着实际调用链走，不能只按“PWM 都在 PWM MSP”这种口头规则判断。

### 8. GPIO speed 设置是不是 PWM 或 I2C 的实际频率？

不是。`GPIO_SPEED_FREQ_LOW`、`GPIO_SPEED_FREQ_HIGH` 是 GPIO 输出驱动速度等级，影响端口输出能力和边沿特性；PWM 频率由定时器时钟、PSC 和 ARR 决定，I2C 速率由 I2C 时钟配置和总线条件决定。

因此本章只记录 speed 配置，不用它推导 PWM 频率、I2C 波形质量或串口波特率准确性。

### 9. 为什么看到 `HAL_GPIO_Init()` 还不能说外部电气已经正确？

因为 `HAL_GPIO_Init()` 只配置 MCU 片内寄存器。它不能证明外部连接存在、上拉电阻合适、负载未短路、线缆未接反、对端设备已经上电，也不能证明波形边沿满足协议要求。

所以本章把寄存器配置和外部验证分开写：前者来自仓库代码，后者需要原理图、万用表、示波器、逻辑分析仪或主机侧记录。

### 10. 为什么 AF_PP 引脚不适合用普通 GPIO 写法验证？

复用推挽输出的控制主体是外设输出单元。TIM PWM 引脚配置为 `GPIO_MODE_AF_PP`
后，真正决定波形的是定时器通道、CCR、输出使能和高级定时器 MOE 等后续条件。
此时用 `HAL_GPIO_WritePin()` 只能修改普通 GPIO 输出锁存，不等于能驱动复用输出波形。

正确做法是先确认 `CRL/CRH` 的复用配置，再到定时器章节检查 PWM 启动和波形实测。

### 11. PB12 的 `BSRR` 写入证明了什么，又不能证明什么？

`HAL_GPIO_WritePin(LED1_GPIO, LED1_PIN, GPIO_PIN_SET/RESET)` 通过 `GPIOB->BSRR`
请求置位或复位 PB12 输出锁存。它能证明软件走了原子写 GPIO 的路径，也能在调试器中结合 `ODR/IDR` 辅助判断 MCU 侧状态。

但它不能单独证明 LED 物理存在、LED 极性正确、电阻参数正确或板上电平确实达到预期。这些仍是硬件验证问题。

## 11. 实践任务

开始任务前，先回到本章第8节定位 MSP 初始化、GPIO 复用和 `HAL_TIM_MspPostInit()` 的代码证据；第9节提供引脚配置核对顺序。

任务一：整理引脚配置清单。

在 `.ioc` 中找出 PB12、PB10、PB11、PC10、PC11、PA11、PA12 和所有 TIM PWM 相关引脚。
验收依据是引脚分类表至少包含引脚名、功能类型、来源文件和证据边界四列。

任务二：核对 GPIO、TIM、I2C、USART 与 USB 的生成代码。

分别在 `gpio.c`、`tim.c`、`i2c.c`、`usart.c` 和 `usbd_conf.c` 中确认对应 MSP 或 PostInit 入口。
验收依据是引脚追踪表至少包含引脚名、生成文件、入口函数和用途结论四列。

任务三：画出初始化关系表。

画出 `MX_*_Init()` 与对应 MSP/PostInit 函数之间的关系表。
验收依据是关系表分列初始化函数、关联 MSP/PostInit、仓库内证据和【待验证】项。

任务四：计算 GPIO 寄存器落点。

分别计算 PB12、PB10、PB11、PC10、PC11、PA1、PA2、PA3、PB8、PB9 位于
`CRL` 还是 `CRH`，以及 4 bit 字段偏移量。
验收依据是能解释为什么 PB12 写 `GPIOB->CRH` 偏移 16，
为什么 PA1/PA2/PA3 写 `GPIOA->CRL`。

任务五：区分片内配置证据和外部验证证据。

任选 I2C2、USART3 或 USB，列出 `.ioc` 信号、GPIO 模式、外设时钟、重映射/中断入口和外部实测项。
验收依据是表格中明确区分“仓库已证明”和“仍需硬件或运行时验证”。

## 12. 思考题

1. 如果只看 `gpio.c`，会漏掉哪些项目引脚？为什么？
2. 为什么系统时钟树必须在 MSP 和 GPIO 复用之前讲？
3. 普通 GPIO 输出和复用推挽输出的控制主体有什么不同？
4. 为什么 I2C2 的 PB10/PB11 不应按普通推挽输出理解？
5. 如果 PWM 引脚配置正确但仍没有输出，下一章之后应继续检查哪些层级？
6. 为什么只看到 `HAL_GPIO_Init()` 还不能证明外部引脚电气条件已经满足？
7. 第07章确认的是 GPIO/MSP 前提，为什么还必须回到第11、14、15章判断串口、I2C 和 USB 的真实行为？
8. 为什么 NVIC 使能中断入口不等于该中断已经发生？
9. 为什么 `GPIO_SPEED_FREQ_HIGH` 不能直接解释为 I2C 的 100 kHz 或 USART3 的 115200 baud？

## 13. 本章总结

本章建立了三轴云台项目中 HAL MSP 初始化和 GPIO 复用配置的项目索引。

已经确认的结论是：

- 全局 MSP 初始化位于 `stm32f1xx_hal_msp.c`。
- PB12 普通 GPIO 输出位于 `gpio.c`。
- TIM2、TIM3、TIM4、TIM8 的 PWM 复用引脚位于 `tim.c` 的 `HAL_TIM_MspPostInit()`。
- I2C2 的 PB10/PB11 复用开漏配置位于 `i2c.c`。
- USART3 的 PC10/PC11 配置位于 `usart.c`。
- USB PCD MSP 位于 `USB_DEVICE/Target/usbd_conf.c`。
- `.ioc` 是引脚分配的配置源头，生成代码是执行层证据。

本章边界：

- 本章证明 MCU 侧 GPIO 与复用初始化路径，不证明外部接线、电平条件或负载状态正确。
- 外设是否产生有效通信或输出，还要结合 I2C、UART、USB、PWM 和后续点测章节继续验证。

下一章可以进入 AFIO 重映射与 SWD 调试，因为第07章已经说明 MSP 和 GPIO 复用在哪里，第08章需要解释项目中调试接口保留和 USART3 重映射为什么出现在这些初始化入口中。

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
- `Core/Src/gpio.c`
- `Core/Src/tim.c`
- `Core/Src/i2c.c`
- `Core/Src/usart.c`
- `USB_DEVICE/Target/usbd_conf.c`
- `Three-axis_cloud_platformV2.ioc`
- `Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c`
- `Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c`
- `Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c`
- `Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_i2c.c`
- `Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c`
- `Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pcd.c`
- `Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xe.h`

外部权威资料：

- ST RM0008 Reference manual: `https://www.st.com/resource/en/reference_manual/rm0008-stm32f101xx-stm32f102xx-stm32f103xx-stm32f105xx-and-stm32f107xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf`
- ST DS5792 Datasheet for STM32F103xC/xD/xE: `https://www.st.com/resource/en/datasheet/stm32f103rc.pdf`
- ST STM32F103 documentation page: `https://www.st.com/en/microcontrollers-microprocessors/stm32f103/documentation.html`

符号、函数与配置项证据：

- `HAL_MspInit()`
- `MX_GPIO_Init()`
- `HAL_TIM_Base_MspInit()`
- `HAL_TIM_PWM_MspInit()`
- `HAL_TIM_MspPostInit()`
- `HAL_I2C_MspInit()`
- `HAL_UART_MspInit()`
- `HAL_PCD_MspInit()`
- `GPIO_InitTypeDef`
- `HAL_GPIO_Init()`
- `HAL_GPIO_WritePin()`
- `GPIO_MODE_OUTPUT_PP`
- `GPIO_MODE_AF_PP`
- `GPIO_MODE_AF_OD`
- `GPIO_MODE_AF_INPUT`
- `GPIO_MODE_INPUT`
- `GPIO_PULLUP`
- `GPIO_NOPULL`
- `GPIO_SPEED_FREQ_LOW`
- `GPIO_SPEED_FREQ_HIGH`
- `GPIO_CR_MODE_INPUT`
- `GPIO_CR_CNF_GP_OUTPUT_PP`
- `GPIO_CR_CNF_AF_OUTPUT_PP`
- `GPIO_CR_CNF_AF_OUTPUT_OD`
- `GPIO_CRL_MODE0`
- `GPIO_CRL_CNF0`
- `GPIOx->CRL`
- `GPIOx->CRH`
- `GPIOx->BSRR`
- `GPIOx->ODR`
- `GPIOx->IDR`
- `RCC->APB2ENR`
- `RCC->APB1ENR`
- `AFIO->MAPR`
- `__HAL_RCC_AFIO_CLK_ENABLE()`
- `__HAL_RCC_PWR_CLK_ENABLE()`
- `__HAL_RCC_USB_CLK_ENABLE()`
- `__HAL_AFIO_REMAP_USART3_PARTIAL()`
- `AFIO_MAPR_USART3_REMAP_PARTIALREMAP`
- `HAL_NVIC_SetPriority()`
- `HAL_NVIC_EnableIRQ()`
- `TIM8_UP_IRQn`
- `USB_LP_CAN1_RX0_IRQn`
- `PB12`
- `PB10`
- `PB11`
- `PC10`
- `PC11`
- `PA11`
- `PA12`

质量自检：

- P0 事实错误：通过
- P1 依赖断层：通过
- P2 逻辑连贯：通过
- P3 项目证据：通过
- P4 原理展开：通过
- P5 调试实践：通过
- P6 表达统一：通过
