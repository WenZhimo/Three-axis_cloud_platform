#include "drv_pwmMotors.h"

// 私有变量
static int16_t sinTable[SIN_TABLE_SIZE];
static uint8_t isInitialized = 0;

// --- 新增：在这里定义目标角度和运动参数 ---
const float POLE_PAIRS = 7.0f;         // 你的电机极对数
float target_deg_yaw = 90.0f;          // 你想要转到的目标机械角度 (比如 90 度)



/**
 * @brief 内部辅助函数：生成正弦查表
 */
static void GenerateSinTable(void)
{
    for (int i = 0; i < SIN_TABLE_SIZE; i++)
    {
        float rad = (float)i * 2.0f * M_PI / (float)SIN_TABLE_SIZE;
        sinTable[i] = (int16_t)(sinf(rad) * (float)SIN_SCALE);
    }
}

/**
 * @brief 初始化所有电机的 PWM 输出
 */
HAL_StatusTypeDef PWM_Motor_Init(void)
{
    if (isInitialized) return HAL_OK;

    GenerateSinTable();

    // --- 启动 Mot0 (Pitch) 相关的定时器通道 ---
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    // --- 启动 Mot1 (Roll) 相关的定时器通道 ---
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

    // --- 启动 Mot2 (Yaw) 相关的定时器通道 ---
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

    // === 新增：解决跨定时器相位错位问题的核心代码 ===
	// 在所有 PWM 通道都开启后，关闭全局中断
	__disable_irq();

	// 暴力将三个独立定时器的计数器同时清零！
	// 连续执行这三句代码只相差 1~2 个 CPU 时钟周期（可忽略不计），
	// 从而完美保证 TIM2, TIM3, TIM4 的 PWM 波形相位完全对齐。
	TIM2->CNT = 0;
	TIM3->CNT = 0;
	TIM4->CNT = 0;

	// 恢复全局中断
	__enable_irq();

    isInitialized = 1;
    return HAL_OK;
}

/**
 * @brief 设置指定电机的角度和输出功率
 * @param axis: 目标电机 (MOTOR_ROLL, MOTOR_PITCH, MOTOR_YAW)
 * @param angle_rad: 电气角度 (弧度)，支持正负值，负值即反转
 * @param power_percent: 功率百分比 (0.0 ~ 100.0)
 */
void PWM_Motor_SetAngle(MotorAxis_t axis, float angle_rad, float power_percent)
{
    if (!isInitialized) return;

    // 安全限制：开环功率限制在 40% 以内，防止电机极速发烫
    if (power_percent > 60.0f) power_percent = 40.0f;
    if (power_percent < 0.0f) power_percent = 0.0f;

    // 1. 处理正负角度，归一化到 0 ~ 2π 之间
    float normalized_angle = fmodf(angle_rad, 2.0f * M_PI);
    if (normalized_angle < 0.0f)
    {
        normalized_angle += 2.0f * M_PI;
    }

    // 2. 映射到正弦表索引 (0 ~ 359)
    int index = (int)(normalized_angle * (float)SIN_TABLE_SIZE / (2.0f * M_PI));
    if (index >= SIN_TABLE_SIZE) index = 0; // 防越界保护

    // 3. 计算幅值基准 (假设三个定时器的 ARR 设置完全一致)
    uint32_t period = htim3.Init.Period + 1;
    uint32_t half_period = period / 2;
    int32_t current_amp = (int32_t)((float)half_period * (power_percent / 100.0f));

    // 4. 计算三相 CCR 比较值 (相位相差 120 度)
    int32_t ccr_val[3];
    int32_t sin_a = sinTable[index];
    int32_t sin_b = sinTable[(index + SIN_TABLE_SIZE / 3) % SIN_TABLE_SIZE];
    int32_t sin_c = sinTable[(index + 2 * SIN_TABLE_SIZE / 3) % SIN_TABLE_SIZE];

    ccr_val[0] = half_period + ((sin_a * current_amp) / SIN_SCALE);
    ccr_val[1] = half_period + ((sin_b * current_amp) / SIN_SCALE);
    ccr_val[2] = half_period + ((sin_c * current_amp) / SIN_SCALE);

    // 溢出保护
    for(int i = 0; i < 3; i++) {
        if(ccr_val[i] > (int32_t)period) ccr_val[i] = period;
        if(ccr_val[i] < 0) ccr_val[i] = 0;
    }

    // 5. 根据硬件映射将 CCR 写入相应的寄存器
        __disable_irq(); // 保证三相更新的原子性

        switch (axis)
        {
            case MOTOR_PITCH: // P轴 -> 物理接口 MOT0 (全部在 TIM3)
                TIM3->CCR2 = (uint32_t)ccr_val[0];
                TIM3->CCR3 = (uint32_t)ccr_val[1];
                TIM3->CCR4 = (uint32_t)ccr_val[2];
                break;

            case MOTOR_ROLL:  // R轴 -> 物理接口 MOT1 (跨 TIM3 和 TIM2)
                TIM3->CCR1 = (uint32_t)ccr_val[0];
                TIM2->CCR4 = (uint32_t)ccr_val[1];
                TIM2->CCR3 = (uint32_t)ccr_val[2];
                break;

            case MOTOR_YAW:   // Y轴 -> 物理接口 MOT2 (跨 TIM4 和 TIM2)
                TIM4->CCR4 = (uint32_t)ccr_val[0];
                TIM2->CCR2 = (uint32_t)ccr_val[1];
                TIM4->CCR3 = (uint32_t)ccr_val[2];


                break;
        }

        __enable_irq();
}


/**
 * @brief 综合测试函数：让三个电机以设定的角度来回摆动
 */
void PWM_Motor_TestAllAngles(void)
{

}

/**
 * @brief 终极硬件点测函数：绕过所有算法，直接验证底层引脚和驱动管
 */
void PWM_Motor_Hardware_Diagnosis(void)
{
    if (!isInitialized) return;

    static uint8_t step = 0;
    static uint32_t last_tick = 0;
    uint32_t current_tick = HAL_GetTick();

    // 每 1000 毫秒（1秒）切换一次相位，给你充足的时间用眼睛看、用手摸
    if (current_tick - last_tick >= 1000)
    {
        last_tick = current_tick;

        // 设置一个固定的测试占空比 (假设你的 ARR 是 3599，1000 大约是 27% 功率)
        // 如果你的 ARR 是 999，请把 duty 改成 250！
        uint32_t duty = 1000;
        uint32_t off = 0;

        __disable_irq();

        // 步骤：先把所有通道彻底关断
        TIM3->CCR2 = off; TIM3->CCR3 = off; TIM3->CCR4 = off; // P轴
        TIM3->CCR1 = off; TIM2->CCR4 = off; TIM2->CCR3 = off; // R轴
        TIM4->CCR4 = off; TIM2->CCR2 = off; TIM4->CCR3 = off; // Y轴

        // 然后根据当前 step，只给三个电机的同一相通电
        switch (step)
        {
            case 0: // 给所有电机的 A 相通电
                //TIM3->CCR2 = duty; // P轴 A相
                TIM3->CCR1 = duty; // R轴 A相
                //TIM4->CCR4 = duty; // Y轴 A相
                break;

            case 1: // 给所有电机的 B 相通电
                //TIM3->CCR3 = duty; // P轴 B相
                TIM2->CCR4 = duty; // R轴 B相
                //TIM2->CCR2 = duty; // Y轴 B相
                break;

            case 2: // 给所有电机的 C 相通电
                //TIM3->CCR4 = duty; // P轴 C相
                TIM2->CCR3 = duty; // R轴 C相
                //TIM4->CCR3 = duty; // Y轴 C相
                break;
        }

        step++;
        if (step > 2) step = 0;

        __enable_irq();
    }
}
