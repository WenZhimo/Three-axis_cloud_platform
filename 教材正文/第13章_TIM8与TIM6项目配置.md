# 第13章 TIM8与TIM6项目配置

> 导航：上一章：[第12章_通用定时器PWM输出](第12章_通用定时器PWM输出.md) ｜ 下一章：[第14章_I2C主机通信](第14章_I2C主机通信.md)

## 1. 本章目标

- 区分 TIM8 高级定时器、TIM6 基本定时器和第12章 TIM2/TIM3/TIM4 主 PWM 输出路径。
- 看懂项目中 TIM8 的 PWM 通道、Break/DeadTime 配置和更新中断入口。
- 看懂项目中 TIM6 的基本计数配置和 `main.c` 中清零计数器的证据。
- 能判断“已配置”“已初始化”“已启动”“已用于业务逻辑”之间的差异。
- 为后续多定时器 PWM、三轴电机映射和硬件诊断章节建立辅助定时资源边界。

本章阅读分层：

| 阅读层次 | 建议范围 | 适合读者 |
|---|---|---|
| 【必须掌握】 | 第1节到第5节，第13节总结 | 需要区分 TIM8、TIM6 与第12章 TIM2/TIM3/TIM4 主 PWM 输出路径的读者 |
| 【工程深化】 | 第6节到第8节，第9节调试方法 | 需要维护 `MX_TIM8_Init()`、`MX_TIM6_Init()`、TIM8 更新中断入口、TIM6 清零证据和主线判定的读者 |
| 【拓展阅读】 | 第5节 APB 定时器时钟推导，第7节工程权衡，第8.6节构建产物证据边界，第10节常见问题 | 需要进一步理解高级定时器输出链路、TIM6 计数粒度、NVIC 与中断源差异的读者 |
| 【证据与验证】 | 第8节、第9节、章节尾部固定检查，以及所有 `【待验证】` 项 | 需要审查 `.ioc/tim.c/main.c`、TIM8/TIM6初始化、TIM8 MSP与中断、TIM6清零路径、`.map/.list/.su/.cyclo`、`CR1.CEN`、`DIER.UIE`、`BDTR.MOE`、断点、中断命中和波形证据的读者 |

如果只是沿定时器主线学习，可以先抓住“第12章 TIM2/TIM3/TIM4 才是当前主 PWM 输出 -> 第13章 TIM8/TIM6 是已配置辅助资源 -> 初始化证据不能替代启动和业务调用证据”这条链；调试扩展定时器、更新中断或辅助计时时，再回到构建产物和运行状态验证小节。

本章速查：

| 查找目标 | 优先阅读 | 避免重复展开 |
|---|---|---|
| TIM8/TIM6 与 TIM2/TIM3/TIM4 主 PWM 输出路径的区别 | 第4节到第5节、第13节 | PSC/ARR 和主 PWM 基础回到第06章、第12章 |
| TIM8 PWM、Break/DeadTime 和更新中断入口 | 第6.1节到第6.2节、第8.1节到第8.3节 | 不把初始化函数或中断向量存在写成已启动、已输出或控制周期来源 |
| TIM6 基本计数配置和 `main.c` 清零证据 | 第6.3节到第6.4节、第8.4节到第8.5节 | 不把一次清零语句写成调度定时器已经运行 |
| `.ioc/tim.c/main.c`、构建产物和运行验证边界 | 第8.6节到第9.6节、章节尾部固定检查 | `.map/.list/.su/.cyclo` 不能替代寄存器、断点、中断命中或波形证据 |

## 2. 前置知识

- 系统时钟树
- NVIC中断配置
- 通用定时器PWM输出
- CMSIS寄存器与内核访问

第06章已经说明定时器依赖系统时钟，第09章已经说明中断入口和 NVIC 配置，第12章已经说明 TIM2/TIM3/TIM4 的主 PWM 输出路径。本章在这些前置知识上解释 TIM8 和 TIM6 为什么不能直接和主 PWM 输出混为一谈。

本章不展开三相正弦 PWM 算法、不展开多定时器相位同步、不展开电机硬件诊断。TIM8 和 TIM6 在当前工程中主要作为已配置的辅助定时资源来分析。

## 3. 问题背景

第12章已经确认，项目真正启动并写 CCR 的主 PWM 输出来自 TIM2、TIM3、TIM4。可是 `tim.c` 和 `.ioc` 中还可以看到 TIM8 和 TIM6：

- TIM8 配置了 PWM CH1/CH2/CH3、PC6/PC7/PC8 引脚、Break/DeadTime 结构，并启用了 `TIM8_UP_IRQn`。
- TIM6 配置了 Prescaler 35、Period 65535，并在 `main.c` 中被清零计数器。

如果不单独解释，读者容易产生两个误解：

1. 看到 TIM8 PWM 配置，就以为它也参与当前三轴电机主输出。
2. 看到 TIM6 被清零，就以为项目周期调度来自 TIM6。

当前源码证据并不支持这两个结论。本章的任务是把“配置存在”和“业务使用”分开。

## 4. 核心概念

- 高级定时器：STM32 中功能更完整的定时器类型，通常支持高级 PWM、互补输出、Break/DeadTime 等能力。本项目中的 TIM8 属于高级定时器。
- 基本定时器：主要提供基础计数和更新事件能力，不提供普通 GPIO PWM 输出通道。本项目中的 TIM6 属于基本定时器。
- `CR1.CEN`：定时器计数器使能位。初始化配置寄存器不等于计数器已经开始计数。
- `DIER.UIE`：更新中断使能位。NVIC 允许接收中断号，不等于定时器已经打开更新中断源。
- `SR.UIF`：更新中断标志位。计数器产生更新事件后会置位该标志，HAL 中断分发会检查它。
- 更新事件：计数器到达自动重装载边界后产生的定时器内部事件，可用于刷新影子寄存器、置位更新标志或触发请求。
- 更新中断：更新事件置位更新标志后，若更新中断源也被使能，才会进入对应 NVIC 中断服务函数。
- `BDTR.MOE`：高级定时器的主输出使能位。高级定时器即使配置了 PWM 通道，也还需要输出使能链路完整，外部引脚才可能输出波形。
- Break/DeadTime：高级定时器面向电机控制和功率驱动场景的保护与死区相关配置。本项目 TIM8 初始化了该结构，但 DeadTime 为 0，BreakState 为 Disable。
- RepetitionCounter：高级定时器的重复计数器，用来控制多少个计数周期后才产生一次更新事件。本项目 TIM8 配置为 0。
- TRGO：定时器触发输出，可把更新事件、使能信号或比较事件送给其它外设。本项目 TIM8/TIM6 均配置为 Reset。
- 已初始化：初始化函数配置了外设寄存器和句柄，例如 `MX_TIM8_Init()` 和 `MX_TIM6_Init()`。
- 已启动：运行时调用启动函数，让定时器或 PWM 通道开始工作，例如第12章中的 `HAL_TIM_PWM_Start()`。
- 业务逻辑：项目在中断、主循环或驱动函数中实际依赖该资源产生控制效果。

这些概念服务于正式知识点 `高级定时器TIM8` 和 `基本定时器TIM6`，不新增结构外知识点。

## 5. 工作原理

TIM8 和 TIM6 分别代表两类不同的定时器资源。

TIM8 是高级定时器。它可以配置 PWM 通道，也可以配置更新中断，还具备 Break/DeadTime 相关能力。

当前项目中，TIM8 的 Prescaler 为 3，Period 为 999，PWM CH1/CH2/CH3 的 Pulse 为 500。

按当前工程时钟配置，TIM8 属于 APB2 定时器资源；如果以 72MHz 定时器频率计算，频率关系是：

`72000000 / (3 + 1) / (999 + 1) = 18000Hz`

也就是约 18kHz。这个推导只说明当前配置下 TIM8 PWM 基础频率，不说明它已经参与电机输出。因为项目中没有发现 `HAL_TIM_PWM_Start(&htim8, ...)`。

这里要拆开一个容易跳步的前置知识：`PCLKx` 不一定等于定时器输入时钟 `TIMxCLK`。当前 `SystemClock_Config()` 使用 HSE 经过 PLL 9 倍得到 72MHz 系统时钟，随后设置：

```text
AHBCLKDivider  = RCC_SYSCLK_DIV1
APB1CLKDivider = RCC_HCLK_DIV2
APB2CLKDivider = RCC_HCLK_DIV1
```

因此：

```text
HCLK  = 72MHz
PCLK1 = 36MHz
PCLK2 = 72MHz
```

STM32F1 定时器还有一条关键规则：当 APB 预分频为 1 时，该 APB 上的定时器时钟等于 PCLK；当 APB 预分频不为 1 时，该 APB 上的定时器时钟等于 2 * PCLK。

所以在当前工程中：

```text
TIM8 属于 APB2，APB2 分频为 1，因此 TIM8CLK = PCLK2 = 72MHz
TIM6 属于 APB1，APB1 分频为 2，因此 TIM6CLK = 2 * PCLK1 = 72MHz
```

这就是本章用 72MHz 计算 TIM8 和 TIM6 的原因。若读者把 TIM6 直接按 PCLK1=36MHz 计算，就会把 0.5us 计数粒度误算成 1us，把 32.768ms 溢出周期误算成 65.536ms。

如果 TIM8 被启动并且更新中断源被打开，`RepetitionCounter = 0` 表示每个计数周期都可以产生一次更新事件。

若重复计数器为 N，则高级定时器会在约 N+1 个计数周期后才产生一次更新事件。
当前项目没有发现 TIM8 启动证据，因此这里仍只是配置层面的理论推导。

如果 TIM8 真的以 18kHz 更新中断运行，每次更新间隔约为：

```text
T_tim8 = 1 / 18000 ≈ 55.6us
```

这个时间预算远小于 500Hz 控制帧的 2ms。若未来把重计算放入 TIM8 更新中断，必须重新评估中断执行时间、嵌套优先级和对 SysTick、I2C、USB 的影响。

当前仓库没有把 TIM8 中断用于这类任务。

TIM6 是基本定时器。它没有 TIM2/TIM3/TIM4 那样的输出比较通道。

当前项目中，TIM6 的 Prescaler 为 35，Period 为 65535。

若按 APB1 定时器频率 72MHz 计算，计数频率约为：

`72000000 / (35 + 1) = 2000000Hz`

也就是每个计数约 0.5us。

如果 TIM6 被启动，16 位计数器从 0 计到 65535 后溢出的理论周期为：

`(65535 + 1) / 2000000 = 0.032768s`

也就是约 32.768ms。这个周期只说明 TIM6 当前配置适合形成毫秒级溢出或微秒级粗计数基础，不说明它已经取代 SysTick。

它也不适合直接解释当前 500Hz 主循环节拍。500Hz 对应 2ms，而 TIM6 若按当前 ARR 溢出，周期约 32.768ms。

除非改 ARR、轮询 CNT 或使用中断策略，否则不能把这个配置直接写成 2ms 调度来源。

但当前工程没有发现 `HAL_TIM_Base_Start(&htim6)` 或 `HAL_TIM_Base_Start_IT(&htim6)`。

`main.c` 中只看到 `__HAL_TIM_SET_COUNTER(&htim6, 0)` 清零计数器。因此，本章不能把 TIM6 写成当前主时间基准。

项目当前主周期仍以第09章讲过的 SysTick 标志为准；第12章主 PWM 输出仍以 TIM2/TIM3/TIM4 为准。

## 6. STM32实现机制

定时器从配置到真正产生业务行为，至少要经过一条完整链路：

`APB定时器时钟 -> PSC -> CNT -> ARR -> 更新事件 -> SR.UIF -> DIER.UIE -> NVIC -> HAL_TIM_IRQHandler() -> 用户回调或中断用户区`

这条链路中的任何一段缺失，都只能说明资源已经配置或入口已经存在，不能说明项目已经把它用作周期任务。

第一段“APB定时器时钟”本身也要拆开。`SystemClock_Config()` 先决定 HCLK、PCLK1 和 PCLK2，再由 STM32 定时器时钟规则决定各定时器的 `TIMxCLK`。本项目中，TIM6 在 APB1，TIM8 在 APB2，但二者当前都可以按 72MHz 定时器输入时钟参与 PSC/ARR 计算。

因此定时器频率推导应写成两层，而不是直接从系统时钟跳到 PWM 频率：

```text
总线层：SYSCLK/HCLK -> PCLK1或PCLK2 -> TIMxCLK
计数层：TIMxCLK -> PSC -> CNT计数频率 -> ARR溢出或PWM周期
```

如果漏掉总线层，就会在 APB1 分频不为 1 时产生 2 倍误差；如果漏掉计数层，就会把“时钟已提供”误读成“定时器已启动并产生业务行为”。

这里最容易混淆的是 NVIC 和定时器自身中断源。

NVIC 的职责是让 Cortex-M3 接受某个外设中断号。定时器自身还必须在 `DIER` 中打开相应中断源，例如 `TIM_IT_UPDATE` 对应 `TIM_DIER_UIE`。

HAL 的 `HAL_TIM_Base_Start_IT()` 会同时做两件关键动作：调用 `__HAL_TIM_ENABLE_IT(htim, TIM_IT_UPDATE)` 打开更新中断源，并调用 `__HAL_TIM_ENABLE(htim)` 让定时器计数。

当前 TIM8 只有 NVIC 入口和 IRQ 处理函数证据，没有发现这类启动调用。

### 1. TIM8 初始化机制

`MX_TIM8_Init()` 执行了以下配置：

- `htim8.Instance = TIM8`
- Prescaler = 3
- CounterMode = Up
- Period = 999
- RepetitionCounter = 0
- 内部时钟源
- PWM 初始化
- CH1/CH2/CH3 配置为 PWM1
- Pulse = 500
- Break/DeadTime 配置

`HAL_TIM_MspPostInit()` 又把 PC6、PC7、PC8 配置为 TIM8 CH1、CH2、CH3 的复用推挽输出。

`HAL_TIM_Base_MspInit()` 在 TIM8 分支中启用 TIM8 时钟，并配置 `TIM8_UP_IRQn` 的优先级为 1、0，然后使能该中断。

对高级定时器 PWM 来说，还要注意输出启动链路。HAL 的 `HAL_TIM_PWM_Start()` 不只是“开始 PWM”这一句抽象动作；在当前 HAL 源码中，它至少会先调用 `TIM_CCxChannelCmd(..., TIM_CCx_ENABLE)` 打开对应通道输出，再在 `IS_TIM_BREAK_INSTANCE(htim->Instance)` 成立时调用 `__HAL_TIM_MOE_ENABLE(htim)` 置位高级定时器主输出使能，最后在非触发从模式下调用 `__HAL_TIM_ENABLE(htim)` 打开计数器。

因此，`MX_TIM8_Init()` 中的 `HAL_TIM_PWM_ConfigChannel()`、`HAL_TIMEx_ConfigBreakDeadTime()` 和 `HAL_TIM_MspPostInit()` 只能证明 PWM 模式、BDTR 参数和 PC6/PC7/PC8 复用输出脚已经被初始化；它们不能替代 `CCER.CCxE`、`BDTR.MOE`、`CR1.CEN` 这些启动后状态证据。

当前没有发现 `HAL_TIM_PWM_Start(&htim8, ...)`，因此不能仅凭 PC6/PC7/PC8 复用和 PWM 通道配置，推断 TIM8 引脚已经输出 PWM。

### 2. TIM8 中断入口

`stm32f1xx_it.c` 中存在 `TIM8_UP_IRQHandler()`，函数内部调用 `HAL_TIM_IRQHandler(&htim8)`。

这说明工程具备 TIM8 更新中断入口。

HAL 源码中的 `HAL_TIM_IRQHandler()` 会检查 `TIM_FLAG_UPDATE` 和 `TIM_IT_UPDATE`，清除更新标志后调用 `HAL_TIM_PeriodElapsedCallback()`。

但当前项目文件中没有发现用户实现的 `HAL_TIM_PeriodElapsedCallback()`，也没有在 `TIM8_UP_IRQHandler()` 用户代码区加入业务逻辑。

这里还要继续拆一层 HAL 回调机制。`Core/Inc/stm32f1xx_hal_conf.h` 中 `USE_HAL_TIM_REGISTER_CALLBACKS` 为 0，因此当前工程走的是“弱函数被用户强函数覆盖”的传统回调路径，而不是运行时注册 `htim->PeriodElapsedCallback` 函数指针的路径。HAL 库自带的 `HAL_TIM_PeriodElapsedCallback()` 是 `__weak` 空实现；如果项目没有提供同名强实现，即使中断分发进入回调分支，也不会自动产生业务状态变化。

同时，项目虽然启用了 `TIM8_UP_IRQn`，但没有发现 `HAL_TIM_Base_Start_IT(&htim8)` 这类启动并打开更新中断源的调用。

因此，本章只能确认中断入口存在，不能把它写成已承担控制周期任务。

完整的 TIM8 周期任务证据至少应包含：

```text
HAL_TIM_Base_Start_IT(&htim8)
-> CR1.CEN = 1
-> DIER.UIE = 1
-> 周期到达后 SR.UIF = 1
-> TIM8_UP_IRQHandler()
-> HAL_TIM_IRQHandler(&htim8)
-> USE_HAL_TIM_REGISTER_CALLBACKS=0 时调用 HAL_TIM_PeriodElapsedCallback()
-> 项目提供同名强回调，或在 IRQ 用户代码区写入业务处理
-> 项目状态发生可解释变化
```

当前仓库只稳定证明了初始化、NVIC 入口和 HAL 分发入口，缺少启动、用户处理和状态变化证据。

### 3. TIM6 初始化机制

`MX_TIM6_Init()` 执行基础计数配置：

- `htim6.Instance = TIM6`
- Prescaler = 35
- CounterMode = Up
- Period = 65535
- AutoReloadPreload = Disable
- MasterOutputTrigger = Reset
- MasterSlaveMode = Disable

`HAL_TIM_Base_MspInit()` 在 TIM6 分支中启用 TIM6 时钟。TIM6 没有 GPIO PWM 后初始化段，也没有输出通道配置。

这与 CMSIS 的 `TIM_TypeDef` 也能互相印证：定时器寄存器结构中有 `CR1`、`DIER`、`SR`、`CNT`、`PSC`、`ARR` 等字段；高级定时器还会用到 `RCR` 和 `BDTR`。

TIM6 当前章节只用到基础计数相关字段，不能把它写成 GPIO PWM 输出资源。

还要补一条容易误判的边界：启动文件中有 `TIM6_IRQHandler` 向量项，CMSIS 设备头文件中也定义了 `TIM6_IRQn = 54`，并把 `TIM6_DAC_IRQn`、`TIM6_DAC_IRQHandler` 映射为 TIM6 的同一中断线/处理函数名。这说明芯片和启动文件具备 TIM6 中断入口框架，不说明当前项目已经把 TIM6 更新中断接入业务代码。

当前工程没有在 `Core/Src/stm32f1xx_it.c` 中提供强定义的 `TIM6_IRQHandler()`，也没有在 `.ioc` 中看到 TIM6 NVIC 使能项。启动文件里的弱符号默认指向 `Default_Handler`，它更像“没有用户处理函数时的兜底入口”，不能当作 TIM6 已经参与调度的证据。

### 4. TIM6 在 `main.c` 中的出现位置

`main()` 在外设初始化阶段调用 `MX_TIM6_Init()`。随后在系统准备就绪附近执行 `__HAL_TIM_SET_COUNTER(&htim6, 0)`。

这条证据说明项目确实访问了 TIM6 计数器。

但当前源码没有显示 TIM6 被启动为运行中的业务计时器。

清零计数器可以作为辅助计时准备动作，但不能单独确认 TIM6 已参与项目调度。

如果运行中 `TIM6->CR1.CEN` 为 0，则 `CNT` 不会因定时器时钟自动递增。若要证明 TIM6 正在计时，需要断点、寄存器观察或代码调用证据【待验证】。

## 7. 项目中的应用

第13章在项目中的作用是“辅助定时资源索引”。

运行流程上，`main()` 先完成系统时钟，再初始化 TIM3、TIM2、TIM4、I2C2、TIM8、USART3、USB Device 和 TIM6。随后项目初始化 SysTick、DWT、配置对象、PWM 电机输出、滤波器、PID 和传感器。

与第12章不同的是：

- TIM2/TIM3/TIM4 在 `PWM_Motor_Init()` 中被 `HAL_TIM_PWM_Start()` 启动，并在 `PWM_Motor_SetAngle()` 中持续写 CCR。
- TIM8 虽然配置了 PWM 和更新中断，但当前源码没有发现 PWM 启动调用，也没有发现 TIM8 更新中断中的项目业务逻辑。
- TIM6 虽然初始化并被清零计数器，但当前源码没有发现启动调用，也没有发现它替代 SysTick 作为主循环节拍。

因此，TIM8 和 TIM6 的当前教材定位不是“主电机输出路径”，而是“已配置、需识别、后续调试和扩展时必须避免误判的辅助资源”。

这里体现了一个工程权衡：保留 TIM8/TIM6 配置有利于后续扩展和现场调试，
但教材不能因为配置存在就抬高它们在当前主线中的地位。

当前项目已经有 TIM2/TIM3/TIM4 作为实际 PWM 输出链路，
也已经有 SysTick 作为 500Hz 主循环触发链路。
在缺少启动、回调和读写路径证据时，切换到 TIM8 或 TIM6 反而会增加解释成本和调试歧义。

从性能角度看，TIM8 配置推导出的 18kHz 更新频率更适合轻量、严格受控的硬件节拍，不适合未经评估就塞入传感器读取、姿态解算或串口打印。

TIM6 当前配置推导出的 32.768ms 溢出周期又慢于 500Hz 主控制节拍。它可以作为粗粒度辅助计时或调试资源的候选，但当前仓库没有这种接入证据。

## 8. 代码分析

### 8.1 `.ioc` 中的 TIM8/TIM6 证据

`.ioc` 记录 TIM8 配置了 CH1/CH2/CH3 的 PWM Generation。
PC6/PC7/PC8 分别对应 TIM8_CH1、TIM8_CH2、TIM8_CH3。
它还记录 `NVIC.TIM8_UP_IRQn=true...`，说明 TIM8 更新中断入口被启用。

`.ioc` 记录 TIM6 的虚拟时钟源为 Enable_Timer，Prescaler 为 35。TIM6 的 Period 在 `tim.c` 中为 65535。

### 8.2 `MX_TIM8_Init()`

`MX_TIM8_Init()` 同时初始化基础计数、PWM、主从同步和 Break/DeadTime。它的输入来自 CubeMX 配置，输出是 `htim8` 句柄和 TIM8 相关寄存器被配置。

风险点是：初始化不等于输出已经运行。项目没有发现 `HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_x)`，所以不能把 TIM8 CH1/CH2/CH3 当成当前主电机输出证据。

### 8.3 TIM8 MSP 与中断

TIM8 的 MSP 初始化启用 TIM8 时钟，并配置 `TIM8_UP_IRQn`。`stm32f1xx_it.c` 中的 `TIM8_UP_IRQHandler()` 调用 `HAL_TIM_IRQHandler(&htim8)`。

这条路径说明：如果 TIM8 更新事件和中断源被启动，HAL 有入口处理它。但当前用户代码没有实现定时器周期回调，也没有在中断函数用户区写业务逻辑，因此不能把 TIM8 中断写成 500Hz 实时控制触发源。

更细地看，NVIC 只解决“CPU 是否接收这个中断号”的问题，`TIM_IT_UPDATE` 才解决“定时器是否向中断控制器提出更新中断请求”的问题。当前仓库能证明前者，不能证明后者已经在运行时被业务代码打开。

HAL 分发逻辑还会同时检查 `TIM_FLAG_UPDATE` 和 `TIM_IT_UPDATE`。也就是说，只有 `SR.UIF` 标志置位且 `DIER.UIE` 中断源打开，才会进入更新回调分支。

当前仓库没有看到用户侧开启 TIM8 `DIER.UIE` 的调用，也没有看到用户侧覆盖弱 `HAL_TIM_PeriodElapsedCallback()`。所以即使 `TIM8_UP_IRQHandler()` 函数存在，也不能把它写成正在执行项目周期逻辑。

如果把这一点写成可验证路径，当前工程需要满足的是：

```text
USE_HAL_TIM_REGISTER_CALLBACKS == 0
-> HAL_TIM_IRQHandler() 选择 HAL_TIM_PeriodElapsedCallback(htim)
-> 项目中存在非弱同名函数实现
-> 回调内修改项目变量、调度任务或记录可观察事件
```

本轮复查没有发现这样的项目侧强回调。因此，第13章应把 TIM8 更新中断定位为“入口和 HAL 分发条件已存在”，而不是“项目周期任务已经接入”。

### 8.4 `MX_TIM6_Init()`

`MX_TIM6_Init()` 只做基础计数配置，不涉及 PWM 通道。它的项目意义是建立一个基本定时器资源。

当前配置下，TIM6 的计数频率可由 APB1 定时器频率和 Prescaler 推出。可是没有启动证据时，这个频率只能说明“如果启动会按该配置计数”，不能说明当前已经运行。

TIM6 也没有 `HAL_TIM_MspPostInit()` 引脚配置，因为基本定时器不承担普通外部 PWM 通道输出。本章要避免把“定时器”三个字自动等同于“PWM 输出”。

从中断证据看，`HAL_TIM_Base_MspInit()` 的 TIM6 分支只启用了 TIM6 外设时钟，没有像 TIM8 分支那样调用 `HAL_NVIC_SetPriority()` 和 `HAL_NVIC_EnableIRQ()`。因此 TIM6 当前更接近“已初始化的基础计数资源”，而不是“已接入 NVIC 的周期中断资源”。

### 8.5 `main.c` 中的 TIM6 清零

`main.c` 在 `systemReady = true` 之后调用 `__HAL_TIM_SET_COUNTER(&htim6, 0)`。这会把 TIM6 的计数器值写为 0。

这条语句说明项目作者可能为后续计时或调试预留 TIM6，但当前没有看到后续读取 TIM6 计数值或启动 TIM6 的证据。因此教材必须保持克制，只写“清零计数器”这一事实。

### 8.6 构建产物证据边界

`Debug/Three-axis_cloud_platformV2.map` 要先区分两个区域：`Discarded input sections` 和真正的 `Linker script and memory map`。当前 `HAL_TIM_Base_Start`、`HAL_TIM_Base_Start_IT` 出现在丢弃输入段附近，地址为 `0x00000000`；而 `TIM8_UP_IRQHandler`、`HAL_TIM_PeriodElapsedCallback`、`MX_TIM8_Init()` 和 `MX_TIM6_Init()` 在内存映射区有最终地址。

这类证据要分层解释：丢弃输入段只能说明目标文件里曾有该函数段，但链接器没有把它放进最终镜像；最终地址只能说明符号进入 ELF，仍不能单独证明项目业务已经调用它。尤其是 HAL 驱动函数和弱回调函数，即使在 `.map` 中有最终地址，也不等于它们已经以 `htim8` 或 `htim6` 为对象被用户路径启动。

更强的构建证据来自 `Debug/Three-axis_cloud_platformV2.list` 中的调用点。当前 `.list` 能看到 `main()` 调用 `MX_TIM8_Init()` 和 `MX_TIM6_Init()`，能看到 `TIM8_UP_IRQHandler()` 调用 `HAL_TIM_IRQHandler(&htim8)`，也能看到 `main.c` 对 `__HAL_TIM_SET_COUNTER(&htim6, 0)` 的编译结果。

把这些证据拆到地址层，可以得到一张更清楚的边界表：

| 证据项 | `.map/.list` 中的表现 | 能证明什么 | 不能证明什么 |
|---|---|---|---|
| `MX_TIM8_Init()` | `.map` 最终地址 `0x0800209c`，`.list` 中 `main()` 有 `bl 800209c <MX_TIM8_Init>` | TIM8 初始化函数进入 Debug 镜像，并被 `main()` 调用 | 不能证明 TIM8 PWM 已被 `HAL_TIM_PWM_Start(&htim8, ...)` 启动 |
| `MX_TIM6_Init()` | `.map` 最终地址 `0x08002030`，`.list` 中 `main()` 有 `bl 8002030 <MX_TIM6_Init>` | TIM6 初始化函数进入 Debug 镜像，并被 `main()` 调用 | 不能证明 TIM6 计数器已经 `CR1.CEN=1` |
| `TIM8_UP_IRQHandler()` | `.map` 最终地址 `0x08001bb0`，`.list` 中入口调用 `HAL_TIM_IRQHandler(&htim8)` | TIM8 更新中断入口和 HAL 分发入口进入最终镜像 | 不能证明 `DIER.UIE` 已打开，也不能证明用户周期回调存在 |
| `HAL_TIM_Base_Start` / `HAL_TIM_Base_Start_IT` | `.map` 中位于 `0x00000000` 丢弃输入段 | 当前目标文件曾包含这些 HAL 函数段 | 不能证明它们进入最终镜像，更不能证明 TIM6/TIM8 已启动 |
| `HAL_TIM_PeriodElapsedCallback()` | `.map` 最终地址 `0x08009b7e`，`.list` 标注为 `__weak` | HAL 默认弱回调被链接以满足分发路径 | 不能证明项目提供了同名强回调或在回调中执行了业务逻辑 |
| `TIM6_IRQHandler` | `.map` 中与多个弱中断同在 `0x08002604` | 启动文件/CMSIS 提供了 TIM6 中断别名或默认入口 | 不能证明项目侧实现了 TIM6 强中断函数，也不能证明 TIM6 NVIC 已接入 |

这张表的教学价值在于，它把“函数存在”“函数进入镜像”“主线调用初始化”“启动外设”和“运行时命中”拆成不同层级。TIM8/TIM6 这类辅助资源尤其容易被误读：看到初始化函数和中断入口，不等于已经看到输出波形、更新中断或周期调度。

`Debug/Core/Src/tim.su` 和 `Debug/Core/Src/tim.cyclo` 还能补上函数级静态资源边界：

- `MX_TIM6_Init` 的静态栈使用量为 16 字节，圈复杂度为 3。
- `MX_TIM8_Init` 的静态栈使用量为 96 字节，圈复杂度为 9。
- `HAL_TIM_Base_MspInit` 的静态栈使用量为 32 字节，圈复杂度为 5。
- `HAL_TIM_PWM_MspInit` 的静态栈使用量为 24 字节，圈复杂度为 2。
- `HAL_TIM_MspPostInit` 的静态栈使用量为 56 字节，圈复杂度为 5。

这些 `.su/.cyclo` 条目只能说明当前编译选项下这些函数生成了静态栈和圈复杂度记录。
它们不能证明 TIM8 PWM 已输出、TIM8 更新中断已命中、TIM6 计数器已运行，也不能替代中断嵌套后的完整栈深度分析。

但当前源码搜索和 `.list` 调用点没有证明以下路径存在：

- `HAL_TIM_PWM_Start(&htim8, ...)`
- `HAL_TIM_Base_Start_IT(&htim8)`
- `HAL_TIM_Base_Start(&htim6)` 或 `HAL_TIM_Base_Start_IT(&htim6)`
- 项目侧强定义的 `HAL_TIM_PeriodElapsedCallback()`
- 项目侧强定义的 `TIM6_IRQHandler()`

因此，本章判断 TIM8/TIM6 未进入当前主线时，不能只说“没搜到源码调用”，还应说明 `.map` 中的符号需要先判断是否被丢弃、是否有最终地址，再判断是否有业务调用；需要把目标文件函数段、最终 ELF 符号、反汇编调用点和运行时寄存器状态分开记录。

### 8.7 本节证据边界

本节只根据当前仓库说明文件、函数、宏、变量和调用关系。运行时频率、外部硬件表现、主机侧现象、传感器方向、电机响应或真实控制效果仍需调试记录、日志或仓库外实测证据；缺少证据时保持【待验证】。

## 9. 调试方法

调试 TIM8/TIM6 时，第一步不是看波形或猜测用途，而是区分配置状态。

本节按统一调试结构组织：现象 -> 可能原因 -> 定位方法 -> 验证步骤 -> 解决方案 -> 经验总结。对第13章而言，关键是把“定时器已配置”“函数进入构建”“定时器已启动”“中断或业务回调已接入”分成不同证据层。

### 9.1 现象与可能原因

常见现象：

- 看到 TIM8 PWM 配置，就误以为它是主 PWM 输出。
- 看到 TIM8/TIM6 中断入口，就误以为它们提供控制周期。
- 看到 `__HAL_TIM_SET_COUNTER(&htim6, 0)`，就误以为 TIM6 正在计时。

可能原因：

- 把 CubeMX 配置和初始化函数混为一层证据。
- 把 NVIC 入口和定时器已经启动混为一层证据。
- 把启动调用、用户回调和运行寄存器状态混在一起判断。

### 9.2 定位方法：TIM8 检查点

- 在 `SystemClock_Config()` 中确认 `APB2CLKDivider = RCC_HCLK_DIV1`，再确认 TIM8 属于 APB2，推得 `TIM8CLK = 72MHz`。
- `.ioc` 中 TIM8 CH1/CH2/CH3 是否配置为 PWM Generation。
- `tim.c` 中 Prescaler、Period、Pulse、Break/DeadTime 是否与 `.ioc` 一致。
- `HAL_TIM_Base_MspInit()` 是否启用 `TIM8_UP_IRQn`。
- `stm32f1xx_it.c` 是否存在 `TIM8_UP_IRQHandler()`。
- 运行中若断点可用，再检查 `TIM8->CR1.CEN`、`TIM8->DIER.UIE` 和 `TIM8->SR.UIF` 是否符合预期。
- 若调试器支持，检查 TIM8 的 `BDTR.MOE` 是否因 PWM 启动而置位；当前源码未发现 TIM8 PWM 启动调用。
- 项目中是否存在 `HAL_TIM_PWM_Start(&htim8, ...)` 或业务回调。当前复查结果是未发现。

### 9.3 定位方法：TIM6 检查点

- 在 `SystemClock_Config()` 中确认 `APB1CLKDivider = RCC_HCLK_DIV2`，再确认 TIM6 属于 APB1；不要直接把 PCLK1=36MHz 当作 TIM6 计数输入，而应按 APB 定时器规则得到 `TIM6CLK = 72MHz`。
- `MX_TIM6_Init()` 是否配置 Prescaler 35、Period 65535。
- `main.c` 是否调用 `MX_TIM6_Init()`。
- `main.c` 是否调用 `__HAL_TIM_SET_COUNTER(&htim6, 0)`。
- 启动文件和 CMSIS 是否只是提供了 TIM6 中断向量/别名，而项目侧是否真的提供 `TIM6_IRQHandler()` 强实现并启用 TIM6 NVIC。当前复查结果是未发现项目侧 TIM6 IRQ 接入证据。
- 运行中若断点可用，再检查 `TIM6->CR1.CEN`、`TIM6->DIER.UIE` 和 `TIM6->CNT` 是否真的随时间变化。
- 项目中是否存在 `HAL_TIM_Base_Start(&htim6)`、`HAL_TIM_Base_Start_IT(&htim6)` 或读取 TIM6 计数值。当前复查结果是未发现。

### 9.4 验证步骤：常见定位结论

- 如果想确认项目主 PWM 输出，不应从 TIM8 开始，而应回到第12章的 TIM2/TIM3/TIM4 和 `PWM_Motor_SetAngle()`。
- 如果想确认项目主周期，不应把 TIM6 当作默认答案，而应回到第09章的 SysTick 和 `frame_500Hz`。
- 如果未来要启用 TIM8 或 TIM6，需要新增启动、回调或读取证据，并同步更新知识链。
- 如果 TIM8 中断真的被启用为 18kHz 周期任务，先用 DWT 或 GPIO 翻转测量 ISR 耗时，再决定能否放入业务逻辑【待验证】。

### 9.5 解决方案：调试记录

- 记录 TIM8/TIM6 的 `.ioc` 配置、`tim.c` 初始化、NVIC 入口和项目调用搜索结果。
- 若引用 `.map` 或 `.list`，应区分函数段是否处在 `Discarded input sections`、符号是否进入最终 ELF、以及项目路径是否真的存在 `bl` 调用点。
- 记录 `SystemClock_Config()` 中的 AHB/APB 分频，再单独记录由 APB 定时器规则得到的 `TIMxCLK`；不要把 PCLK 和 TIMxCLK 混写在同一列。
- 对“已配置但未接入主线”的判断，应同时保留配置证据和缺少启动/读取/回调调用的证据。
- 未来若启用 TIM8 或 TIM6，应把新增调用点、运行目的和章节映射同步记录为新的接入条件。
- 没有外部波形、断点命中或运行日志时，不应宣称 TIM8/TIM6 已经产生有效项目行为。

### 9.6 经验总结：调试边界

当前仓库能证明 TIM8/TIM6 的初始化配置和调用缺口。实际计数、PWM 波形、中断命中和业务效果需要运行日志、断点记录或外部测量；缺少证据时保持【待验证】。

## 10. 常见问题

### 1. TIM8 配置了 PWM，为什么不算当前主 PWM 输出？

触发条件：读者在 `.ioc` 或 `tim.c` 中看到 TIM8 CH1/CH2/CH3。

可能原因：配置存在不等于运行时启动。当前主 PWM 输出证据是 TIM2/TIM3/TIM4 被 `HAL_TIM_PWM_Start()` 启动，并被 `PWM_Motor_SetAngle()` 写 CCR；TIM8 没有同类启动证据。

因此，第13章要教读者把“配置存在”与“主线运行”分开。
只有看见启动、回调或业务调用，才可以把 TIM8 写成项目行为的一部分。

### 2. TIM8 更新中断启用了，为什么不算控制周期来源？

触发条件：读者看到 `TIM8_UP_IRQHandler()`。

可能原因：中断入口存在，但当前用户代码中没有 TIM8 业务逻辑，也没有发现用户实现的 `HAL_TIM_PeriodElapsedCallback()`。第09章已经确认 500Hz 实时控制循环触发标志来自 SysTick。

也就是说，TIM8 现在更像一个“可用资源”而不是“已接入调度器”的资源。
教材在这里刻意避免把配置和实际用途混为一谈。

完整证据至少应同时出现：定时器启动、更新中断源使能、NVIC 入口、HAL 分发路径和用户业务处理。当前仓库只稳定证明其中一部分。

更具体地说，`HAL_TIM_Base_Start_IT()` 会打开 `DIER.UIE` 并使能计数器。单独的 `HAL_NVIC_EnableIRQ(TIM8_UP_IRQn)` 不会替代这一步。

还要继续区分“回调机制存在”和“项目回调存在”。当前配置 `USE_HAL_TIM_REGISTER_CALLBACKS=0`，HAL 会调用弱符号 `HAL_TIM_PeriodElapsedCallback()`；只有项目提供同名强函数时，弱回调才会被覆盖。当前仓库没有发现这个强函数，因此不能把 HAL 的默认空回调写成项目业务处理。

### 3. TIM6 被清零，为什么不算正在计时？

触发条件：读者看到 `__HAL_TIM_SET_COUNTER(&htim6, 0)`。

可能原因：清零只是写计数器值。当前没有发现 TIM6 启动调用或读取调用，因此不能证明它正在承担业务计时。

这一点和第04章的“对象进入链接不等于业务调用”很像。
计数器存在、被清零和正在承担调度，是三件不同的事。

同理，启动文件中存在 `TIM6_IRQHandler` 弱符号，也不等于项目写了 TIM6 中断业务。弱符号默认连到 `Default_Handler`，只有项目提供同名强函数、启用 NVIC、打开 `DIER.UIE` 并启动计数器后，TIM6 更新中断链路才具备可分析的业务意义。

### 4. TIM8 的 Break/DeadTime 配置有什么意义？

触发条件：读者看到 `HAL_TIMEx_ConfigBreakDeadTime()`。

可能原因：这是高级定时器配置能力的一部分。当前项目配置 DeadTime 为 0、BreakState 为 Disable，因此本章只把它作为 TIM8 高级定时器属性证据，不展开功率驱动保护策略。

这也说明 TIM8 目前只是“具备高级特性”，不代表这些特性已经被项目用在实际电机保护链路中。

Break/DeadTime 常见于半桥、全桥或互补 PWM 功率级，目标是避免上下桥臂同时导通或在故障输入时关断输出。当前项目的 TIM8 DeadTime 为 0、BreakState 为 Disable，因此不能把它写成已经实现硬件保护。

另外，高级定时器还有主输出使能链路。当前 TIM8 配置了 BDTR 结构，但未发现 TIM8 PWM 启动调用，因此也不能把 PC6/PC7/PC8 写成已经对外输出高级 PWM。

### 5. 为什么第13章仍然重要？

触发条件：TIM8/TIM6 当前没有进入主线，读者怀疑是否可以跳过。

可能原因：读懂“未使用或未启动的配置”是工程分析能力的一部分。移植、调试或扩展项目时，错误地把 TIM8/TIM6 当成当前主线，会导致定位方向偏移。

换句话说，本章是为了防止读者把“看见配置”误当成“看见功能”。

### 6. 为什么 TIM6 在 APB1 上，却仍按 72MHz 计算？

触发条件：读者看到 `APB1CLKDivider = RCC_HCLK_DIV2`，以为 TIM6 只能按 36MHz 计算。

可能原因：STM32F1 的 APB 定时器时钟有倍频规则。当前 HCLK 为 72MHz，APB1 分频为 2，所以 PCLK1 为 36MHz；但当 APB 预分频不为 1 时，该 APB 上的定时器时钟为 2 * PCLK。因此 TIM6 的输入时钟仍可按 72MHz 推导。

这个区别会直接影响 PSC/ARR 计算。若误用 36MHz，TIM6 的计数粒度会从 0.5us 被误写成 1us，溢出周期也会从约 32.768ms 被误写成约 65.536ms。

## 11. 实践任务

开始任务前，先回到本章第8节定位 TIM8、TIM6 的初始化和中断边界；第9节提供 TIM8/TIM6 与主线 PWM 的对比调试顺序。

任务一：整理 TIM8 配置项。

在 `.ioc` 中找出 TIM8 CH1/CH2/CH3、PC6/PC7/PC8 和 TIM8 更新中断配置。
验收依据是 TIM8 配置表包含通道、引脚、中断和来源文件。

任务二：计算 TIM8 PWM 频率。

在 `tim.c` 中找出 `MX_TIM8_Init()` 的 Prescaler、Period、Pulse 和 Break/DeadTime 设置。
验收依据是频率表包含 PSC、ARR、Pulse、Break/DeadTime 和频率结果。

任务三：确认 TIM8 中断边界。

在 `stm32f1xx_it.c` 中找到 `TIM8_UP_IRQHandler()`。
验收依据是中断边界表包含入口函数、回调函数和用户逻辑结论。

任务三补充：拆分更新中断完整链路。

按 `CR1.CEN`、`SR.UIF`、`DIER.UIE`、NVIC、`HAL_TIM_IRQHandler()`、`HAL_TIM_PeriodElapsedCallback()` 的顺序整理 TIM8 更新中断证据。
验收依据是每一层都标出“已证明”“未发现”或“需实测【待验证】”。

任务四：追踪 TIM6 当前用途。

在 `tim.c` 和 `main.c` 中追踪 TIM6。
验收依据是 TIM6 追踪表包含初始化位置、清零位置和未启动结论。

任务五：对比定时器主线地位。

对比第12章和第13章。
验收依据是对比表分列主线 PWM、辅助定时器、证据来源和当前地位。

任务六：拆分 APB 总线时钟和定时器输入时钟。

在 `Core/Src/main.c` 中找到 `SystemClock_Config()`，记录 HCLK、APB1、APB2 的分频；再结合 TIM6 属于 APB1、TIM8 属于 APB2，计算 `TIM6CLK` 和 `TIM8CLK`。
验收依据是表格至少包含 `SYSCLK/HCLK`、`PCLK1`、`PCLK2`、`TIM6CLK`、`TIM8CLK`、PSC、ARR 和最终周期。若把 `PCLK1` 与 `TIM6CLK` 写成同一个数，应标为计算错误。

实践边界：

当前任务优先形成表格、链路图、搜索记录和计算过程。涉及 IDE 现场、构建日志、断点数值、外部波形、主机侧结果或硬件响应时，若没有截图、日志或仓库外实测证据，结论保持【待验证】。

## 12. 思考题

1. 为什么“CubeMX 配置了某个外设”不等于“项目运行时正在使用它”？
2. 如果未来要让 TIM8 承担一个周期任务，至少还需要补充哪些代码证据？
3. 如果未来要用 TIM6 做辅助计时，为什么只清零计数器还不够？
4. TIM8 的高级定时器能力和第12章通用 PWM 输出有什么相同点与不同点？
5. 为什么教材要明确标出当前未发现的启动和业务回调证据？
6. 如果 `.ioc` 中启用了 TIM8 中断，但业务变量没有随之变化，教材应如何描述 TIM8 的当前状态？
7. 第13章判断 TIM8/TIM6 未进入主线时，需要同时引用哪些仓库内证据和缺失证据？
8. 为什么只启用 NVIC 还不足以证明定时器更新中断已经在运行时触发？
9. 如果 TIM8 真的以 18kHz 进入更新中断，为什么不适合直接放入耗时较长的传感器读取或串口打印？
10. TIM6 当前 32.768ms 的理论溢出周期和 500Hz 控制帧的 2ms 周期有什么差异？
11. 高级定时器的 BDTR/MOE 链路为什么会影响“配置了 PWM”和“引脚真的输出 PWM”的判断？
12. 为什么 APB1 的 PCLK1 为 36MHz 时，TIM6 的定时器输入时钟仍可能是 72MHz？
13. 如果把 TIM6CLK 误写成 PCLK1，会怎样影响计数粒度、溢出周期和调试结论？

## 13. 本章总结

本章建立了 TIM8 与 TIM6 在当前三轴云台项目中的证据边界。

已经确认的结论是：

- TIM8 配置了 PWM CH1/CH2/CH3、PC6/PC7/PC8、Prescaler 3、Period 999、Pulse 500。
- TIM8 配置了 Break/DeadTime 结构，并启用了 `TIM8_UP_IRQn`。
- `TIM8_UP_IRQHandler()` 存在，但当前用户代码未发现 TIM8 业务逻辑或用户周期回调。
- 当前 `USE_HAL_TIM_REGISTER_CALLBACKS=0`，TIM 更新回调依赖用户覆盖弱 `HAL_TIM_PeriodElapsedCallback()`，但仓库未发现同名强实现。
- 当前仓库未发现 TIM8 `DIER.UIE` 由用户启动路径打开的证据，也未发现 TIM8 `CR1.CEN` 运行证据。
- 当前工程未发现 `HAL_TIM_PWM_Start(&htim8, ...)`，因此不能把 TIM8 写成主 PWM 输出路径。
- TIM6 配置了 Prescaler 35、Period 65535，并在 `main.c` 中被清零计数器。
- 启动文件和 CMSIS 提供 TIM6 中断向量及 `TIM6_DAC` 别名，但当前项目未发现 `TIM6_IRQHandler()` 强实现、TIM6 NVIC 使能、`DIER.UIE` 启动路径或 TIM6 中断业务代码。
- 当前工程未发现 TIM6 启动或读取证据，因此不能把 TIM6 写成主时间基准。
- 当前 `SystemClock_Config()` 设置 APB1 分频为 2、APB2 分频为 1；结合 STM32F1 APB 定时器时钟规则，TIM6 和 TIM8 在本工程中都按 72MHz 定时器输入时钟推导。
- `.map/.list/.su/.cyclo` 的构建产物结论统一回到第6节判断：它们能证明 TIM8/TIM6 初始化函数、TIM8 更新中断入口、HAL 默认弱回调、静态栈和圈复杂度条目进入某次 Debug 构建，也能暴露 HAL 启动函数停在丢弃段；但是否存在项目调用、外设启动和运行态命中，还要结合源码搜索、`.list` 调用点、寄存器状态或波形实测判断。

本章待验证分类：

| 类别 | 已由本章证明 | 仍保持【待验证】 |
|---|---|---|
| 构建验证 | `.map/.list/.su/.cyclo` 能证明 TIM8/TIM6 初始化函数、TIM8 更新中断入口、HAL 默认弱回调、静态栈和圈复杂度条目进入某次 Debug 构建。 | 构建产物不能替代项目调用、外设启动、运行态命中、计数器状态、PWM 波形或中断触发证据。 |
| 软件验证 | `.ioc`、`tim.c`、启动文件、CMSIS、源码搜索和 `.list` 能证明 TIM8/TIM6 存在配置、初始化函数、中断入口，并能说明当前未发现 `HAL_TIM_PWM_Start(&htim8, ...)`、TIM6 强中断处理函数、TIM6 启动或读取路径。 | 不证明 TIM8/TIM6 已经承担当前主控制周期、主 PWM 输出或主时间基准；外设是否进入业务主线仍需继续查启动调用、回调函数和状态变量变化。 |
| 参数验证 | TIM8 的 18kHz、TIM6 的 0.5us 计数粒度和 32.768ms 溢出周期可由当前配置推导。 | 这些推导不属于实测运行结论，不能替代定时器启动状态、PWM 波形、计数器运行或中断命中证据。 |
| 硬件验证 | 本章能说明 TIM8/TIM6 的配置意图、引脚/中断入口前提和当前仓库内未进入主业务路径的证据边界。 | 真实 PWM 输出、计数器运行、TIM6/TIM8 中断触发、示波器波形和目标板寄存器状态仍需现场证据。 |
| 官方资料待确认 | 本章频率计算依赖 `SystemClock_Config()`、STM32F1 APB 定时器时钟规则、TIMxCLK 与 PCLK 的区别，以及 TIM6/TIM8 外设行为。 | 若时钟树、APB 分频、定时器时钟规则或 TIM6/TIM8 用途改变，频率和周期结论必须结合参考手册与当前配置重新确认。 |
| 实验待完成 | 本章已经把配置存在、构建进入、业务主线缺口和参数推导拆成可检查对象。 | 后续需记录定时器启动调用、回调命中、寄存器状态、计数器变化、PWM 波形和中断触发日志。 |

下一章可以进入 I2C 主机通信。到这里，教材已经完成平台、时钟、中断、调试输出和定时器输出基础；后续可以开始分析 MPU6050 传感器如何通过 I2C 进入项目数据流。

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

- 高级定时器TIM8
- 基本定时器TIM6

项目证据：

- `Core/Src/tim.c`
- `Core/Src/stm32f1xx_it.c`
- `Core/Src/main.c`
- `Three-axis_cloud_platformV2.ioc`
- `Debug/Three-axis_cloud_platformV2.map`
- `Debug/Three-axis_cloud_platformV2.list`
- `Debug/Core/Src/tim.su`
- `Debug/Core/Src/tim.cyclo`
- `Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c`
- `Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_tim.h`
- `Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xe.h`

参考资料：

- STMicroelectronics, `RM0008 STM32F10xxx reference manual`。
- STMicroelectronics, `AN4013 Introduction to timers for STM32 MCUs`。
- STMicroelectronics, `AN4776 General-purpose timer cookbook for STM32 microcontrollers`。
- 项目内 HAL 源码 `Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c`。

引用的函数、配置项和变量：

- `MX_TIM8_Init()`
- `MX_TIM6_Init()`
- `HAL_TIM_PWM_Init()`
- `HAL_TIM_PWM_ConfigChannel()`
- `HAL_TIMEx_ConfigBreakDeadTime()`
- `HAL_TIM_Base_MspInit()`
- `HAL_TIM_MspPostInit()`
- `TIM8_UP_IRQHandler()`
- `HAL_TIM_IRQHandler()`
- `HAL_TIM_PeriodElapsedCallback()`
- `HAL_TIM_Base_Start()`
- `HAL_TIM_Base_Start_IT()`
- `HAL_TIM_PWM_Start()`
- `__HAL_TIM_SET_COUNTER()`
- `__HAL_TIM_ENABLE_IT()`
- `USE_HAL_TIM_REGISTER_CALLBACKS`
- `htim8`
- `htim6`
- `TIM8_UP_IRQn`
- `TIM_IT_UPDATE`
- `TIM_FLAG_UPDATE`
- `TIM_DIER_UIE`
- `TIM_SR_UIF`
- `TIM_CR1_CEN`
- `TIM_BDTR_MOE`
- `TIM8.Period=999`
- `TIM8.Prescaler=3`
- `TIM6.Prescaler=35`
- `PC6.Signal=S_TIM8_CH1`
- `PC7.Signal=S_TIM8_CH2`
- `PC8.Signal=S_TIM8_CH3`

质量自检：

- P0 事实错误：通过。
- P1 依赖断层：通过。
- P2 逻辑连贯：通过。
- P3 项目证据：通过。
- P4 原理展开：通过。
- P5 调试实践：通过。
- P6 表达统一：通过。

---
> 导航：上一章：[第12章_通用定时器PWM输出](第12章_通用定时器PWM输出.md) ｜ 下一章：[第14章_I2C主机通信](第14章_I2C主机通信.md)
