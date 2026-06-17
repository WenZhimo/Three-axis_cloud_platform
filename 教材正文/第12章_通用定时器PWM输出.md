# 第12章 通用定时器PWM输出

## 1. 本章目标

- 理解通用定时器如何通过计数周期和比较值产生 PWM 输出。
- 看懂项目中 TIM2、TIM3、TIM4 的 PSC、ARR、PWM 模式和通道配置。
- 能从 `.ioc`、`tim.c` 和 `drv_pwmMotors.c` 追踪 PWM 从配置、GPIO 复用、通道启动到 CCR 更新的路径。
- 明确通用定时器 PWM 输出在三轴电机驱动中的基础作用。
- 区分本章的 PWM 基础输出与后续 TIM8/TIM6、三相正弦驱动、多定时器相位同步章节边界。

## 2. 前置知识

- 系统时钟树
- HAL MSP初始化机制
- GPIO输出与复用
- CMSIS寄存器与内核访问

第06章已经说明 APB1 定时器时钟来源，第07章已经说明 GPIO 复用和 MSP 初始化，第05章已经说明项目中存在寄存器级访问。本章在这些前置基础上解释 TIM2、TIM3、TIM4 如何产生 PWM 波形，并说明 `drv_pwmMotors.c` 如何启动和更新这些通道。

本章不展开 TIM8 高级定时器配置、不展开 TIM6 基本定时器用途、不展开三相正弦 PWM 电机驱动算法、不展开多定时器相位同步细节。它们已经安排在后续章节。

## 3. 问题背景

三轴云台最终要驱动电机。对 MCU 来说，电机驱动并不是直接输出一个“角度”或“力矩”变量，而是需要把控制计算转换成定时器通道上的 PWM 占空比。

项目中这条输出基础链路可以从三个位置看到：

- `.ioc` 把 TIM2、TIM3、TIM4 的多个通道配置为 PWM Generation。
- `Core/Src/tim.c` 生成 TIM2、TIM3、TIM4 的 PWM 初始化和 GPIO 复用配置。
- `Drivers/CustomDrivers/Src/drv_pwmMotors.c` 启动这些 PWM 通道，并在运行时写入 `TIMx->CCR` 改变占空比。

如果读者只看控制算法，就会看到 `PWM_Motor_SetAngle()` 这样的抽象接口，却不知道电机输出实际落在哪些定时器通道上。如果只看 `tim.c`，又只能看到通道被配置，却不知道项目什么时候启动它们、怎样更新比较值。本章把这两部分连接起来。

## 4. 核心概念

- 通用定时器：STM32 中可用于计数、比较和 PWM 输出的定时器资源。本项目主 PWM 输出使用 TIM2、TIM3、TIM4。
- PSC：预分频器，决定定时器计数频率。本项目 TIM2、TIM3、TIM4 的 Prescaler 均为 0。
- ARR：自动重装载值，决定一个 PWM 周期包含多少个计数。本项目 TIM2、TIM3、TIM4 的 Period 均为 3599。
- CCR：捕获比较寄存器，PWM 输出中通常决定翻转或有效电平保持到哪个计数位置。本项目运行时直接写 `TIMx->CCR` 改变占空比。
- PWM 模式 1：计数器从 0 向上计数时，比较值之前和之后输出不同电平，从而形成占空比。
- 占空比：一个 PWM 周期内有效电平所占比例。简化理解可用 `CCR / (ARR + 1)` 表示。
- 定时器通道：一个定时器可以有多个输出通道，例如 TIM3 CH1、CH2、CH3、CH4。
- 复用推挽输出：GPIO 引脚由定时器外设驱动输出波形，本项目 PWM 引脚都配置为 `GPIO_MODE_AF_PP`。

这些概念服务于正式知识点 `通用定时器PWM输出`，不新增结构外知识点。

## 5. 工作原理

PWM 的基本思想是：用固定频率的周期波形表示一个连续强度。周期由定时器计数决定，占空比由比较值决定。

对向上计数 PWM 来说，定时器从 0 开始计数，计到 ARR 后回到 0。每一轮计数就是一个 PWM 周期。CCR 是周期内的比较点。CCR 越大，有效电平持续时间越长；CCR 越小，有效电平持续时间越短。

本项目 TIM2、TIM3、TIM4 的配置很统一：

- Prescaler = 0
- Period = 3599
- PWM 模式 = `TIM_OCMODE_PWM1`
- 初始 Pulse = 500
- 输出极性 = 高有效
- 自动重装载预装载 = Disable

`.ioc` 中 APB1 定时器频率为 72000000，TIM2/TIM3/TIM4 又都配置为 Prescaler 0、Period 3599。因此按当前工程配置，PWM 频率可按：

`72000000 / (0 + 1) / (3599 + 1) = 20000Hz`

也就是约 20kHz。初始 Pulse 500 对应的初始占空比约为：

`500 / (3599 + 1) = 13.9%`

运行时，项目不会一直保持这个初始 Pulse。`PWM_Motor_SetAngle()` 会根据上层传入的角度和功率计算三个 CCR 值，再写入对应定时器通道。也就是说，`tim.c` 给出 PWM 的基本载波和通道，`drv_pwmMotors.c` 在运行时改变每个通道的占空比。

## 6. STM32实现机制

STM32 HAL 中，PWM 输出至少需要四类配置。

第一类是定时器基础参数。TIM2、TIM3、TIM4 都设置向上计数、Prescaler 0、Period 3599。TIM2 和 TIM4 还显式配置内部时钟源；TIM3 通过 PWM 初始化路径生成。

第二类是输出比较参数。`TIM_OC_InitTypeDef` 中配置 PWM 模式、Pulse、输出极性和快速模式。本项目使用 `TIM_OCMODE_PWM1`、Pulse 500、`TIM_OCPOLARITY_HIGH` 和 `TIM_OCFAST_DISABLE`。

第三类是通道配置。TIM2 启用 CH2/CH3/CH4，TIM3 启用 CH1/CH2/CH3/CH4，TIM4 启用 CH3/CH4。`HAL_TIM_PWM_ConfigChannel()` 把输出比较参数绑定到具体通道。

第四类是 MSP 后初始化。`HAL_TIM_MspPostInit()` 把定时器通道连接到 GPIO 引脚：

- TIM2 CH2/CH3/CH4：PA1、PA2、PA3。
- TIM3 CH1/CH2/CH3/CH4：PA6、PA7、PB0、PB1。
- TIM4 CH3/CH4：PB8、PB9。

这些引脚都配置为复用推挽输出，速度为低速。第07章已经说明 GPIO 复用的含义，本章把它接到 PWM 波形输出。

项目还配置了 TIM8 和 TIM6，但它们不属于本章主线。TIM8 的高级定时器属性和中断配置放到第13章；TIM6 的基本定时器用途也放到第13章。

## 7. 项目中的应用

本章在项目运行流程中的位置如下：

1. `main()` 完成系统时钟配置后，依次调用 `MX_TIM3_Init()`、`MX_TIM2_Init()`、`MX_TIM4_Init()`。
2. 这些初始化函数在 `tim.c` 中配置 PWM 周期、通道和 GPIO 复用输出。
3. 用户初始化阶段调用 `PWM_Motor_Init()`。
4. `PWM_Motor_Init()` 调用 `HAL_TIM_PWM_Start()` 启动 TIM2、TIM3、TIM4 上用于电机输出的 9 个 PWM 通道。
5. 500Hz 控制循环中，`computeMotorCommands(dt500Hz)` 会在满足运行条件时调用 `PWM_Motor_SetAngle()`。
6. `PWM_Motor_SetAngle()` 根据目标轴、角度和功率计算三个比较值，并写入对应的 `TIMx->CCR`。

在项目“采集—处理—控制—输出”链路中，本章处于“输出”基础层。传感器采样和姿态计算决定控制输入，PID 和角度换算决定输出命令，而 TIM2/TIM3/TIM4 的 PWM 通道把命令转换成可送往电机驱动硬件的波形。

## 8. 代码分析

### 1. `.ioc` 中的 PWM 通道证据

`.ioc` 明确记录了 TIM2、TIM3、TIM4 的 PWM 通道和引脚：

- PA1、PA2、PA3 对应 TIM2 CH2、CH3、CH4。
- PA6、PA7、PB0、PB1 对应 TIM3 CH1、CH2、CH3、CH4。
- PB8、PB9 对应 TIM4 CH3、CH4。

`.ioc` 还记录 TIM2、TIM3、TIM4 的 Prescaler 为 0，Period 为 3599，初始 Pulse 为 500。这是 `tim.c` 生成配置的上游证据。

### 2. `MX_TIM2_Init()`

TIM2 配置了内部时钟源、PWM 初始化、主从同步关闭和 CH2/CH3/CH4 三个 PWM 通道。它的项目意义是提供三路 PWM 输出，其中 `drv_pwmMotors.c` 后续会使用 TIM2 CH2、CH3、CH4 分别参与不同电机轴的输出。

### 3. `MX_TIM3_Init()`

TIM3 配置了 CH1/CH2/CH3/CH4 四个 PWM 通道。它是项目中使用通道最多的通用定时器。`PWM_Motor_Init()` 会启动 TIM3 的四个通道；`PWM_Motor_SetAngle()` 也会直接写 TIM3 的 CCR 寄存器。

### 4. `MX_TIM4_Init()`

TIM4 配置了 CH3/CH4 两个 PWM 通道。项目后续将它们用于某个电机轴的输出组合。本章只确认 TIM4 提供这两路 PWM 基础波形，具体三轴电机映射在后续章节展开。

### 5. `HAL_TIM_MspPostInit()`

`HAL_TIM_MspPostInit()` 是 PWM 引脚真正连接到 GPIO 的位置。它启用 GPIOA 或 GPIOB 时钟，把对应引脚配置为 `GPIO_MODE_AF_PP`。如果只执行 `HAL_TIM_PWM_ConfigChannel()`，但没有完成 GPIO 复用输出配置，波形就无法从对应引脚输出。

### 6. `PWM_Motor_Init()`

`PWM_Motor_Init()` 是项目业务代码启动 PWM 输出的入口。它先生成正弦查表，然后对 TIM3、TIM2、TIM4 的相关通道调用 `HAL_TIM_PWM_Start()`。

本章只分析启动通道这个事实。函数中还清零 TIM2/TIM3/TIM4 的计数器，这涉及多定时器相位同步，属于后续章节，本章只记录它位于 PWM 通道启动之后。

### 7. `PWM_Motor_SetAngle()`

`PWM_Motor_SetAngle()` 的输入是电机轴、角度和功率百分比。函数根据角度得到三个相位的输出值，再把结果写入对应 `TIMx->CCR`。

从本章角度看，关键不是正弦计算本身，而是最后的寄存器写入：CCR 值改变后，下一轮 PWM 周期的占空比也随之改变。这说明项目并不是反复重新初始化定时器，而是在运行时更新比较寄存器。

## 9. 调试方法

调试 PWM 输出时，应先确认配置链路，再确认运行链路。

配置链路观察点：

- `.ioc` 中 TIM2、TIM3、TIM4 是否配置了预期 PWM 通道。
- `tim.c` 中 Prescaler、Period、Pulse 是否与 `.ioc` 一致。
- `HAL_TIM_MspPostInit()` 是否把对应引脚配置为复用推挽输出。
- `MX_TIM3_Init()`、`MX_TIM2_Init()`、`MX_TIM4_Init()` 是否在 `main()` 中先于 `PWM_Motor_Init()` 调用。

运行链路观察点：

- `PWM_Motor_Init()` 是否已经执行。
- `HAL_TIM_PWM_Start()` 是否覆盖项目需要的 9 个通道。
- `PWM_Motor_SetAngle()` 是否在电机使能后被调用。
- `TIMx->CCR` 是否会随输入角度或功率变化。

常见异常定位：

- 没有 PWM 输出：先检查对应 `HAL_TIM_PWM_Start()` 是否调用，再检查 GPIO 复用输出配置。
- PWM 频率不符合预期：检查 APB1 定时器频率、Prescaler 和 Period。
- 某一路通道无输出：检查 `.ioc` 引脚、`HAL_TIM_PWM_ConfigChannel()` 和 `HAL_TIM_MspPostInit()` 三者是否一致。
- 占空比不变化：检查 `PWM_Motor_SetAngle()` 是否被调用，以及对应轴是否写到了预期的 `TIMx->CCR`。
- 只在部分电机轴上有输出：先检查 `drv_pwmMotors.c` 中的通道启动清单，再把具体轴映射留到后续三轴电机硬件映射章节分析。

当前工作树没有外部测量设备、驱动板接线或电机相线物理证据，因此具体引脚外部连接和波形实测结论需要标记为【待验证】。本章只确认工程内部 PWM 配置和软件写入路径。

## 10. 常见问题

### 1. 为什么本项目用多个通用定时器输出 PWM？

触发条件：读者看到 TIM2、TIM3、TIM4 同时参与输出。

可能原因：一个通用定时器的通道数量有限，而三轴电机输出需要多路 PWM。项目把 TIM2、TIM3、TIM4 的通道组合起来使用。具体三轴通道映射在后续章节展开。

### 2. Prescaler 为 0 是什么意思？

触发条件：读者以为 0 表示没有时钟。

可能原因：在 STM32 定时器中，Prescaler 寄存器值 0 表示除以 `0 + 1`，也就是不再分频。当前工程下 TIM2/TIM3/TIM4 使用 APB1 定时器频率直接计数。

### 3. ARR 为什么是 3599？

触发条件：需要解释 PWM 周期。

可能原因：ARR 决定计数周期长度。按当前 APB1 定时器频率 72000000、Prescaler 0、Period 3599，PWM 频率约为 20kHz。

### 4. Pulse 500 是否就是运行时占空比？

触发条件：读者看到 `tim.c` 中所有通道初始 Pulse 为 500。

可能原因：Pulse 500 是初始化时的比较值。项目运行后会通过 `PWM_Motor_SetAngle()` 直接写 CCR，因此运行时占空比由后续控制输出决定。

### 5. 为什么 `drv_pwmMotors.c` 直接写 `TIMx->CCR`，而不是反复调用 HAL 配置函数？

触发条件：读者看到寄存器级 CCR 写入。

可能原因：初始化只需要做一次，运行时高频更新占空比只需要改变比较值。直接写 CCR 可以减少重复配置开销。第05章已经建立 CMSIS/寄存器访问前置，本章只解释它在 PWM 占空比更新中的作用。

### 6. TIM8 也配置了 PWM，为什么本章不讲？

触发条件：读者在 `tim.c` 或 `.ioc` 中看到 TIM8 PWM。

可能原因：TIM8 是高级定时器，并且项目还启用了 TIM8 更新中断。它已被安排到第13章，与 TIM6 一起用于区分辅助定时资源和主 PWM 输出路径。

## 11. 实践任务

1. 在 `.ioc` 中列出 TIM2、TIM3、TIM4 的 PWM 通道和对应引脚。验收依据是能得到 9 个通道的完整清单。
2. 在 `tim.c` 中找出 TIM2、TIM3、TIM4 的 Prescaler、Period 和 Pulse。验收依据是能算出约 20kHz 的 PWM 频率和初始占空比。
3. 在 `HAL_TIM_MspPostInit()` 中追踪每个 PWM 通道的 GPIO 复用配置。验收依据是能说明为什么定时器输出能到达 PA/PB 引脚。
4. 在 `PWM_Motor_Init()` 中确认项目启动了哪些 PWM 通道。验收依据是能把启动通道与 `.ioc` 通道清单对应起来。
5. 在 `PWM_Motor_SetAngle()` 中找出 CCR 写入位置。验收依据是能说明 CCR 改变会改变 PWM 占空比。

## 12. 思考题

1. 为什么 PWM 频率需要同时看 APB1 定时器频率、Prescaler 和 ARR？
2. 如果某一路引脚没有 PWM 波形，应先检查 `tim.c` 的通道配置，还是 `drv_pwmMotors.c` 的通道启动？为什么？
3. 初始化 Pulse 为 500，而运行时写 CCR，这两者在项目中分别承担什么角色？
4. 为什么第12章只讲 TIM2/TIM3/TIM4 的通用 PWM 基础，而不提前讲三相正弦 PWM 计算？
5. 如果多个通道分布在不同定时器上，后续为什么还需要讨论相位同步？

## 13. 本章总结

本章建立了三轴云台项目中通用定时器 PWM 输出的证据链。

已经确认的结论是：

- `.ioc` 配置 TIM2、TIM3、TIM4 的 9 个 PWM 通道。
- TIM2、TIM3、TIM4 的 Prescaler 均为 0，Period 均为 3599，初始 Pulse 均为 500。
- 按当前 APB1 定时器频率，TIM2/TIM3/TIM4 的 PWM 频率约为 20kHz。
- `tim.c` 通过 `HAL_TIM_PWM_Init()`、`HAL_TIM_PWM_ConfigChannel()` 和 `HAL_TIM_MspPostInit()` 建立 PWM 输出基础。
- `PWM_Motor_Init()` 启动项目实际使用的 PWM 通道。
- `PWM_Motor_SetAngle()` 通过写 `TIMx->CCR` 改变占空比。

下一章可以进入 TIM8 与 TIM6 项目配置，因为本章已经解释了主 PWM 输出基础，后续需要区分高级定时器、基本定时器和主电机输出路径之间的边界。

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

- 通用定时器PWM输出

项目证据：

- `Core/Src/tim.c`
- `Three-axis_cloud_platformV2.ioc`
- `Drivers/CustomDrivers/Src/drv_pwmMotors.c`
- `Drivers/CustomDrivers/Inc/drv_pwmMotors.h`
- `Core/Src/main.c`
- `Drivers/SRC/Src/computeMotorCommands.c`

引用的函数、配置项和变量：

- `MX_TIM2_Init()`
- `MX_TIM3_Init()`
- `MX_TIM4_Init()`
- `HAL_TIM_PWM_Init()`
- `HAL_TIM_PWM_ConfigChannel()`
- `HAL_TIM_MspPostInit()`
- `HAL_TIM_PWM_Start()`
- `PWM_Motor_Init()`
- `PWM_Motor_SetAngle()`
- `computeMotorCommands()`
- `htim2`
- `htim3`
- `htim4`
- `TIM2->CCR2`
- `TIM2->CCR3`
- `TIM2->CCR4`
- `TIM3->CCR1`
- `TIM3->CCR2`
- `TIM3->CCR3`
- `TIM3->CCR4`
- `TIM4->CCR3`
- `TIM4->CCR4`
- `TIM2.Period=3599`
- `TIM3.Period=3599`
- `TIM4.Period=3599`
- `TIM2.Prescaler=0`
- `TIM3.Prescaler=0`
- `TIM4.Prescaler=0`
- `RCC.APB1TimFreq_Value=72000000`

质量自检：

- P0 事实错误：通过。
- P1 依赖断层：通过。
- P2 逻辑连贯：通过。
- P3 项目证据：通过。
- P4 原理展开：通过。
- P5 调试实践：通过。
- P6 表达统一：通过。
