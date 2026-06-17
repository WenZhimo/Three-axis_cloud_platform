# 第20章 IMU安装方向与坐标变换

## 1. 本章目标

第19章已经说明，`MPU6050_Read_And_Process()` 不只是从 MPU6050 连续读取 14 字节，还会调用方向矩阵处理加速度和陀螺仪原始值。本章专门解释这一步。

本章目标是：

- 理解 `三维矩阵乘法` 在项目中的最小实现：3x3 方向矩阵乘 3x1 传感器向量。
- 理解 `IMU安装方向矩阵` 解决的问题：传感器芯片坐标和云台控制坐标不一定一致。
- 找到 `orientationMatrix[9]` 的定义、`orientIMU()` 的赋值入口和 `matrixMultiply()` 的调用位置。
- 明确当前项目实际使用的安装方向：`orientIMU()` 内部强制把 `eepromConfig.imuOrientation` 设置为 10。
- 建立第21章标定、第23章姿态解算之前的坐标一致性基础。

本章知识链为：

`知识点总表` 中的 `三维矩阵乘法`、`IMU安装方向矩阵`
-> `知识依赖图` 中 `IMU安装方向矩阵` 依赖 `MPU6050连续数据读取` 和 `三维矩阵乘法`
-> `学习优先级` 中 `三维矩阵乘法` 属于 P1，`IMU安装方向矩阵` 属于 P2
-> `教学顺序` 第32、33项
-> `教材章节` 第20章。

## 2. 前置知识

本章正式前置知识包括：

- `MPU6050连续数据读取`
- `三维矩阵乘法`

其中 `三维矩阵乘法` 虽然是第20章内正式知识点，但它在本章内部先讲，再服务于 `IMU安装方向矩阵`。这和第17章中先讲联合体再讲配置结构的处理类似：同章内部允许存在先后关系，但必须在正文顺序上讲清楚。

读者不需要先掌握四元数姿态解算。本章只处理“原始三轴向量如何根据安装方向重新排列和变号”，不进入姿态算法。

## 3. 问题背景

MPU6050 输出的是芯片自身坐标系下的 X、Y、Z 三轴数据。云台控制代码使用的却是控制坐标，例如 Roll、Pitch、Yaw 或项目中约定的 X/Y/Z 输入方向。

如果传感器安装方向和代码假设不一致，就会出现这些问题：

- 云台向前倾，代码却认为是侧倾。
- 加速度重力方向符号反了。
- 陀螺仪角速度方向和控制轴方向相反。
- 后续姿态解算和 PID 控制得到错误输入。

项目用 `orientationMatrix` 解决这个问题。它不是滤波器，也不是姿态解算算法，而是一个安装方向修正层：先把传感器原始向量变换到项目希望使用的坐标，再交给后续标定、缩放和姿态解算。

## 4. 核心概念

### 4.1 三维向量

加速度和角速度都可以看作三维向量：

```text
[x, y, z]^T
```

在项目中，读取后的原始加速度会进入 `straightAccelData[3]`，原始陀螺仪会进入 `straightGyroData[3]`。这里的 `straight` 可以理解为“刚从当前原始轴整理出来、还没有应用安装方向矩阵”的向量。

### 4.2 3x3方向矩阵

项目用一维数组保存 3x3 矩阵：

```c
int16_t orientationMatrix[9];
```

按 `matrixMultiply()` 的索引方式，`matrixA[i * aCols_bRows + j]` 表示第 `i` 行第 `j` 列。因此 `orientationMatrix[0..8]` 可以理解为：

```text
[ m00 m01 m02
  m10 m11 m12
  m20 m21 m22 ]
```

当前项目中的方向矩阵元素多为 `-1`、`0`、`1`，用于表达轴交换和符号反转。

### 4.3 矩阵乘向量

`matrixMultiply(3, 3, 1, rotatedAccelData, orientationMatrix, straightAccelData)` 表示：

```text
rotated = orientationMatrix * straight
```

如果矩阵是单位矩阵，输出等于输入。如果矩阵某一行把 `y` 放到 `z`，就表示坐标轴交换。如果某一项是 `-1`，就表示对应轴取反。

## 5. 工作原理

第20章的工作路径在源码中非常集中：

1. `Core/Src/main.c` 调用 `orientIMU()`。
2. `orientIMU()` 设置 `eepromConfig.imuOrientation` 并填充 `orientationMatrix[9]`。
3. 第19章的 `MPU6050_Read_And_Process()` 读取并装载原始三轴数据。
4. 读取函数把 `rawAccel/rawGyro` 的值复制到 `straightAccelData/straightGyroData`。
5. `matrixMultiply()` 用 `orientationMatrix` 分别旋转加速度向量和陀螺仪向量。
6. 旋转后的结果写回 `rawAccel/rawGyro`。

这意味着后续 `Core/Src/main.c` 用 `rawAccel/rawGyro` 做物理量缩放时，拿到的已经是经过安装方向矩阵处理后的值。

## 6. STM32实现机制

本章没有使用新的 STM32 外设。它完全发生在 CPU 软件计算中。

实现机制主要包括：

- `int16_t orientationMatrix[9]` 保存 3x3 整数矩阵。
- `matrixMultiply()` 使用三重循环完成矩阵乘法。
- `MPU6050_Read_And_Process()` 在每次 500Hz 读取后调用矩阵乘法。

需要注意，`matrixMultiply()` 的数组参数类型都是 `int16_t`，而函数内部把 `matrixC[i]` 清零时写的是 `0.0`。这会被转换为整数 0。当前矩阵元素和原始数据都是整数，项目并没有在这一层做浮点旋转。

本章不讨论 FPU、DSP 指令或优化库。STM32F103RCTx 是 Cortex-M3 内核，没有硬件 FPU；当前源码选择了简单整数矩阵乘法，符合项目现状。

## 7. 项目中的应用

本章涉及的项目证据文件包括：

- `Drivers/CustomDrivers/Src/mpu6050.c`
- `Drivers/CustomDrivers/Inc/mpu6050.h`
- `Drivers/SRC/Src/config.c`
- `Core/Src/main.c`

`Drivers/CustomDrivers/Src/mpu6050.c` 定义 `orientationMatrix[9]`、`matrixMultiply()` 和 `orientIMU()`，并在 `MPU6050_Read_And_Process()` 中调用矩阵乘法。

`Drivers/CustomDrivers/Inc/mpu6050.h` 声明 `orientationMatrix[9]`、`matrixMultiply()` 和 `orientIMU()`，让其他文件可以调用或观察这些对象。

`Drivers/SRC/Src/config.c` 中默认设置：

```c
eepromConfig.imuOrientation = 4;
```

但 `orientIMU()` 开头又执行：

```c
eepromConfig.imuOrientation = 10;
```

因此，当前运行主线中实际使用的是 `case 10` 对应矩阵，而不是 `config.c` 默认值 4。教材必须以 `orientIMU()` 的运行时覆盖为准。

## 8. 代码分析

### 8.1 orientationMatrix定义

`Drivers/CustomDrivers/Src/mpu6050.c` 中定义：

```c
int16_t orientationMatrix[9];
```

这是一个全局数组。头文件 `Drivers/CustomDrivers/Inc/mpu6050.h` 中通过：

```c
extern int16_t orientationMatrix[9];
```

暴露声明。

由于它是全局对象，如果没有调用 `orientIMU()`，矩阵内容虽然会按 C 全局变量规则初始化为 0，但那不是有效方向矩阵。因此调试时必须确认 `Core/Src/main.c` 中 `orientIMU()` 已在读取前执行。

### 8.2 matrixMultiply实现

`matrixMultiply()` 的核心索引是：

```c
matrixC[i * bCols + k] +=
    matrixA[i * aCols_bRows + j] * matrixB[j * bCols + k];
```

在本项目调用中，参数固定为 `3, 3, 1`。因此它实际完成的是：

```text
3x3 matrixA * 3x1 matrixB -> 3x1 matrixC
```

例如第 0 个输出元素等于：

```text
C0 = A00 * B0 + A01 * B1 + A02 * B2
```

这就是三轴坐标变换的最小数学结构。

### 8.3 orientIMU的当前实际分支

`orientIMU()` 开头写死：

```c
eepromConfig.imuOrientation = 10;
```

随后进入 `switch (eepromConfig.imuOrientation)`。所以当前实际分支是 `case 10`：

```text
[  1  0  0
   0  0  1
   0 -1  0 ]
```

这表示：

- 输出 X 来自输入 X。
- 输出 Y 来自输入 Z。
- 输出 Z 来自输入 Y 的相反数。

换成向量关系就是：

```text
x' = x
y' = z
z' = -y
```

这和第19章读到的“读取后还会方向处理”连接起来：后续 `rawAccel[YAXIS]` 不再只是原始 Y 轴，而是矩阵处理后的 Y 轴。

### 8.4 config.c默认值被覆盖

`Drivers/SRC/Src/config.c` 中 `init_eepromConfig(true)` 设置 `eepromConfig.imuOrientation = 4`。如果只看配置默认值，会以为项目使用 `case 4`。

但 `Core/Src/main.c` 调用 `orientIMU()`，而 `orientIMU()` 内部立刻把值改成 10。因此当前实际运行路径是：

```text
config.c 默认值 4 -> orientIMU() 覆盖为 10 -> switch 进入 case 10
```

这是一处项目一致性重点。教材不能只引用配置默认值，也不能只列出所有 case，而要明确当前实际路径。

### 8.5 default分支不是单位矩阵

`orientIMU()` 的 `default` 注释写着 `Dot Front/Left/Top`，但实际矩阵是：

```text
[ 1 0 0
  0 1 0
  0 1 0 ]
```

它不是标准单位矩阵，因为最后一行不是 `[0 0 1]`。这可能是源码中的默认分支错误或未清理状态。本章不修改源码，但要提醒读者：不能把 `default` 当作“无旋转默认值”。当前实际运行使用 `case 10`，所以 `default` 分支暂不影响主线，但后续维护时需要核对。

### 8.6 矩阵结果写回rawAccel和rawGyro

`MPU6050_Read_And_Process()` 调用矩阵乘法后执行：

```c
rawAccel[axis].value = (int16_t)rotatedAccelData[axis];
rawGyro[axis].value  = (int16_t)rotatedGyroData[axis];
```

这说明方向处理发生在 `rawAccel/rawGyro` 层，而不是 `sensors.accel500Hz/sensors.gyro500Hz` 层。第21章做物理量缩放时，输入已经是方向处理后的原始值。

## 9. 调试方法

调试 IMU 安装方向时，建议按顺序检查。

第一步，确认 `orientIMU()` 是否执行。

- 在 `Core/Src/main.c` 的 `orientIMU()` 调用处设置断点。
- 单步进入 `Drivers/CustomDrivers/Src/mpu6050.c`。
- 确认 `eepromConfig.imuOrientation` 被设置为 10。

第二步，检查 `orientationMatrix`。

- 调用 `orientIMU()` 后观察 `orientationMatrix[0..8]`。
- 当前应为 `[1,0,0, 0,0,1, 0,-1,0]`。
- 如果不是，说明代码路径或内存被改写。

第三步，检查矩阵乘法输入输出。

- 在 `MPU6050_Read_And_Process()` 中观察 `straightAccelData` 和 `rotatedAccelData`。
- 对照 `case 10` 的关系，确认 `x'=x`、`y'=z`、`z'=-y`。
- 对 `straightGyroData` 和 `rotatedGyroData` 做同样检查。

第四步，检查写回结果。

- 确认 `rawAccel[axis].value` 等于对应 `rotatedAccelData[axis]`。
- 确认 `rawGyro[axis].value` 等于对应 `rotatedGyroData[axis]`。
- 再进入第21章的缩放验证。

第五步，记录待验证硬件事实。

- 当前教材只能证明代码使用 `case 10`。
- `case 10` 是否与实际 IMU 贴片方向、云台机械坐标完全一致，需要结合实物安装或运动测试验证，标记为【待验证】。

## 10. 常见问题

问题一：为什么需要安装方向矩阵。

因为传感器芯片坐标不一定等于云台控制坐标。矩阵可以把芯片输出的三轴数据重新排列、取反，使后续算法看到统一坐标。

问题二：当前项目是否使用 `config.c` 中的 `imuOrientation = 4`。

当前主线不是。`config.c` 默认值为 4，但 `orientIMU()` 开头强制改为 10，所以实际使用 `case 10`。

问题三：`orientationMatrix` 是浮点矩阵吗。

不是。源码中它是 `int16_t orientationMatrix[9]`，当前用于 `-1`、`0`、`1` 的轴交换和符号反转。

问题四：`default` 分支是否表示单位矩阵。

不能这样认为。当前 `default` 分支最后一行是 `[0,1,0]`，不是 `[0,0,1]`。由于主线使用 `case 10`，它暂不影响当前路径，但属于需要后续核对的源码风险。

问题五：方向矩阵处理后，`rawAccel/rawGyro` 还是原始寄存器值吗。

不是严格意义上的寄存器直出值。它们仍是未缩放的整数原始量，但已经经过安装方向矩阵处理。更准确地说，它们是“方向处理后的原始整数值”。

## 11. 实践任务

任务一：手算 `case 10` 的输出。

假设输入向量为 `[100, 200, 300]^T`，使用 `case 10` 矩阵计算输出向量，并和调试器中的 `matrixMultiply()` 输出比较。

任务二：验证 `config.c` 默认值被覆盖。

在 `init_eepromConfig(true)` 后观察 `eepromConfig.imuOrientation`，再在 `orientIMU()` 后观察一次，记录从 4 到 10 的变化。

任务三：检查 default 分支。

把 `default` 分支矩阵写成 3x3 形式，判断它是否为单位矩阵，并记录你的结论。

任务四：定位方向处理发生层级。

在 `MPU6050_Read_And_Process()` 中找到 `rawAccel/rawGyro` 写回位置，再到 `Core/Src/main.c` 中找到物理量缩放位置，说明两者顺序。

任务五：设计实物验证动作。

设计三个简单动作：绕 X、Y、Z 方向缓慢转动或倾斜云台，记录 `rawAccel/rawGyro` 哪个轴变化，用于验证 `case 10` 是否符合实物方向。

## 12. 思考题

1. 为什么安装方向矩阵应该在物理量缩放之前执行。
2. 如果把 `orientationMatrix` 设置为全 0，会对后续姿态解算产生什么影响。
3. `case 10` 的 `z'=-y` 对重力方向判断意味着什么。
4. 为什么只凭源码不能证明 `case 10` 一定符合真实硬件安装。
5. 如果希望通过 `eepromConfig.imuOrientation` 动态选择方向，应如何修改当前 `orientIMU()` 的覆盖行为。
6. 方向矩阵和后续四元数姿态解算之间是什么关系。

## 13. 本章总结

本章解释了第19章留下的方向矩阵边界。

项目在 `Drivers/CustomDrivers/Src/mpu6050.c` 中定义 `orientationMatrix[9]`，用 `matrixMultiply()` 完成 3x3 矩阵乘 3x1 向量，并在 `MPU6050_Read_And_Process()` 中对加速度和陀螺仪数据分别执行方向处理。

本章已经证明：

- 当前 `orientIMU()` 会把 `eepromConfig.imuOrientation` 强制设置为 10。
- 当前实际矩阵为 `[1,0,0, 0,0,1, 0,-1,0]`。
- 当前 `config.c` 中的默认值 4 会被 `orientIMU()` 覆盖。
- `default` 分支不是单位矩阵，不能作为无旋转默认配置理解。
- `rawAccel/rawGyro` 在进入物理量缩放前已经经过方向矩阵处理。

本章保留两个边界：

- 不证明 `case 10` 与实物安装必然一致，该项需要【待验证】。
- 不展开四元数姿态解算，只说明方向矩阵为后续姿态输入提供坐标一致性。

下一章将进入 `传感器标定、零偏与物理量缩放`，在已经完成连续读取和方向处理的基础上，分析项目如何处理温度漂移、静态零偏和物理单位转换。
