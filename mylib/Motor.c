#include "ti_msp_dl_config.h"
#include "mylib/Motor.h"

// ==================== 内部变量 ====================
// 当前速度值（-100 ~ 100）
static int16_t g_left_speed = 0;
static int16_t g_right_speed = 0;

// 目标速度（用于速度曲线）
static int16_t g_target_left_speed = 0;
static int16_t g_target_right_speed = 0;

// 加速度值（每周期增加/减少的速度量）
static uint16_t g_acceleration = 3;

// PWM周期值（对应syscfg中的timerCount=3，实际PWM周期+1）
#define PWM_PERIOD  100

/* 调试用：右电机GPIO中断触发次数计数器
 * 注意: 该变量在中断处理函数中递增，用于诊断右编码器是否正常工作 */
static volatile uint32_t g_capture_interrupt_count = 0;

// ==================== 内部函数声明 ====================
static void Motor_SetLeftDirection(uint8_t direction);
static void Motor_SetRightDirection(uint8_t direction);
static void Motor_SetPWM(uint8_t channel, int16_t speed);

// ==================== 基础控制 ====================
// 简介：使能电机（STBY = 1）
// 参数：无
void Motor_Enable(void)
{
    DL_GPIO_setPins(Motor_STBY_PORT, Motor_STBY_PIN);
}

// 简介：禁用电机（STBY = 0）
// 参数：无
void Motor_Disable(void)
{
    DL_GPIO_clearPins(Motor_STBY_PORT, Motor_STBY_PIN);
    // 禁用时停止PWM输出
    Motor_SetPWM(0, 0);
    Motor_SetPWM(1, 0);
}

// 简介：快速停止（切断供电，滑行停止）
// 参数：无
void Motor_Stop(void)
{
    // AIN1=0, AIN2=0 → 电机停止（高阻态）
    DL_GPIO_clearPins(Motor_AIN1_PORT, Motor_AIN1_PIN);
    DL_GPIO_clearPins(Motor_AIN2_PORT, Motor_AIN2_PIN);
    DL_GPIO_clearPins(Motor_BIN1_PORT, Motor_BIN1_PIN);
    DL_GPIO_clearPins(Motor_BIN2_PORT, Motor_BIN2_PIN);

    // 关闭PWM
    Motor_SetPWM(0, 0);
    Motor_SetPWM(1, 0);

    // 速度清零
    g_left_speed = 0;
    g_right_speed = 0;
    g_target_left_speed = 0;
    g_target_right_speed = 0;
}

// 简介：刹车减速（PWM逐渐降为0，缓慢停止）
// 参数：无
void Motor_Brake(void)
{
    // 设置PWM占空比为0实现刹车
    Motor_SetPWM(0, 0);
    Motor_SetPWM(1, 0);
    g_left_speed = 0;
    g_right_speed = 0;
}

// ==================== 基础运动（固定速度） ====================
// 简介：电机向前动（双轮同速前进）
// 参数：无
void Motor_Forward(void)
{
    Motor_Enable();
    Motor_SetSpeed(MOTOR_SPEED_DEFAULT, MOTOR_SPEED_DEFAULT);
}

// 简介：电机向后动（双轮同速后退）
// 参数：无
void Motor_Backward(void)
{
    Motor_Enable();
    Motor_SetSpeed(-MOTOR_SPEED_DEFAULT, -MOTOR_SPEED_DEFAULT);
}

// ==================== 速度控制（核心PWM调速） ====================
// 简介：设置双轮独立速度
// 参数：left_speed - 左轮速度(-100~100), right_speed - 右轮速度(-100~100)
void Motor_SetSpeed(int16_t left_speed, int16_t right_speed)
{
    // 限幅
    if (left_speed > MOTOR_SPEED_MAX)  left_speed = MOTOR_SPEED_MAX;
    if (left_speed < MOTOR_SPEED_MIN) left_speed = MOTOR_SPEED_MIN;
    if (right_speed > MOTOR_SPEED_MAX)  right_speed = MOTOR_SPEED_MAX;
    if (right_speed < MOTOR_SPEED_MIN) right_speed = MOTOR_SPEED_MIN;

    // 保存目标速度
    g_target_left_speed = left_speed;
    g_target_right_speed = right_speed;

    // 立即更新速度
    g_left_speed = left_speed;
    g_right_speed = right_speed;

    // 根据速度正负设置方向
    if (left_speed < 0) {
        Motor_SetLeftDirection(0);  // 左轮正转
    } else {
        Motor_SetLeftDirection(1);   // 左轮反转
    }

    if (right_speed < 0) {
        Motor_SetRightDirection(0);  // 右轮正转
    } else {
        Motor_SetRightDirection(1);  // 右轮反转
    }

    // 设置PWM
    //printf("SetSpeed: %d, %d\r\n", left_speed, right_speed);
    Motor_SetPWM(0, left_speed);
    Motor_SetPWM(1, right_speed);
}

// 简介：设置左轮速度
// 参数：speed - 左轮速度(-100~100)
void Motor_SetLeftSpeed(int16_t speed)
{
    Motor_SetSpeed(speed, g_right_speed);
}

// 简介：设置右轮速度
// 参数：speed - 右轮速度(-100~100)
void Motor_SetRightSpeed(int16_t speed)
{
    Motor_SetSpeed(g_left_speed, speed);
}

// 简介：获取左轮当前速度
// 返回：左轮速度值(-100~100)
int16_t Motor_GetLeftSpeed(void)
{
    return g_left_speed;
}

// 简介：获取右轮当前速度
// 返回：右轮速度值(-100~100)
int16_t Motor_GetRightSpeed(void)
{
    return g_right_speed;
}

float Motor_CmdToSpeedPps(int16_t cmd)
{
    if (cmd > MOTOR_SPEED_MAX) {
        cmd = MOTOR_SPEED_MAX;
    } else if (cmd < MOTOR_SPEED_MIN) {
        cmd = MOTOR_SPEED_MIN;
    }

    return ((float)cmd * MOTOR_MAX_SPEED_PPS) / (float)MOTOR_SPEED_MAX;
}

// ==================== 转向控制（差速控制 - 循迹核心） ====================
// 简介：左转（降低左轮速度，右轮正常）
// 参数：speed - 基础速度(-100~100)
void Motor_TurnLeft(int16_t speed)
{
    int16_t left_speed = (int16_t)(speed * 0.6f);   // 左轮减速到30%
    int16_t right_speed = speed;                    // 右轮全速

    Motor_SetSpeed(left_speed, right_speed);
}

// 简介：右转（降低右轮速度，左轮正常）
// 参数：speed - 基础速度(-100~100)
void Motor_TurnRight(int16_t speed)
{
    int16_t left_speed = speed;                     // 左轮全速
    int16_t right_speed = (int16_t)(speed * 0.6f);  // 右轮减速到30%

    Motor_SetSpeed(left_speed, right_speed);
}

// 简介：原地旋转
// 参数：direction - (-1=原地左转, 1=原地右转)
void Motor_Spin(int16_t direction)
{
    Motor_Enable();

    if (direction < 0) {
        // 原地左转：左轮反转，右轮正转
        Motor_SetSpeed(-MOTOR_SPEED_DEFAULT, MOTOR_SPEED_DEFAULT);
    } else {
        // 原地右转：左轮正转，右轮反转
        Motor_SetSpeed(MOTOR_SPEED_DEFAULT, -MOTOR_SPEED_DEFAULT);
    }
}

// 简介：小幅度转向修正（循迹中使用）
// 参数：error - 偏差值，负数=左偏，正数=右偏
void Motor_TurnSmall(int16_t error)
{
    int16_t base_speed = MOTOR_SPEED_DEFAULT;
    float turn_factor = 0.5f;  // 转向系数，可调整

    // 根据偏差调整左右轮速度差
    int16_t left_speed = base_speed - (int16_t)(error * turn_factor);
    int16_t right_speed = base_speed + (int16_t)(error * turn_factor);

    Motor_SetSpeed(left_speed, right_speed);
}

// ==================== 速度曲线（加减速） ====================
// 简介：设置加速度值
// 参数：accel - 加速度(值越大加速越快，建议1~20)
void Motor_SetAcceleration(uint16_t accel)
{
    if (accel > 100) accel = 100;
    g_acceleration = accel;
}

// 简介：速度斜坡更新（需在循环中调用，实现平滑加减速）
// 说明：调用此函数后，需要配合 Motor_SetSpeed 使用目标速度
void Motor_SpeedRamp(void)
{
    // 左轮速度渐变
    if (g_left_speed < g_target_left_speed) {
        g_left_speed += g_acceleration;
        if (g_left_speed > g_target_left_speed) {
            g_left_speed = g_target_left_speed;
        }
    } else if (g_left_speed > g_target_left_speed) {
        g_left_speed -= g_acceleration;
        if (g_left_speed < g_target_left_speed) {
            g_left_speed = g_target_left_speed;
        }
    }

    // 右轮速度渐变
    if (g_right_speed < g_target_right_speed) {
        g_right_speed += g_acceleration;
        if (g_right_speed > g_target_right_speed) {
            g_right_speed = g_target_right_speed;
        }
    } else if (g_right_speed > g_target_right_speed) {
        g_right_speed -= g_acceleration;
        if (g_right_speed < g_target_right_speed) {
            g_right_speed = g_target_right_speed;
        }
    }

    // 更新PWM输出（同时处理方向）
    Motor_SetSpeed(g_left_speed, g_right_speed);
}

// ==================== PID控制（预留实现） ====================
// 简介：初始化PID参数
// 参数：pid - PID结构体指针, Kp/Ki/Kd - PID系数
void Motor_PID_Init(Motor_PID_t *pid, float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->target = 0;
    pid->error = 0;
    pid->last_error = 0;
    pid->integral = 0;
    pid->output = 0;
}

// 简介：单轮速度PID计算
// 参数：pid - PID结构体指针, current_value - 当前实际值
// 返回：PID计算输出值
float Motor_PID_Calculate(Motor_PID_t *pid, float current_value)
{
    // 计算偏差
    pid->error = pid->target - current_value;

    // 比例项
    float p_term = pid->Kp * pid->error;

    // 积分项（带积分限幅，防止积分饱和）
    pid->integral += pid->error;
    if (pid->integral > 100) pid->integral = 100;
    if (pid->integral < -100) pid->integral = -100;
    float i_term = pid->Ki * pid->integral;

    // 微分项
    float d_term = pid->Kd * (pid->error - pid->last_error);
    pid->last_error = pid->error;

    // 计算输出
    pid->output = p_term + i_term + d_term;

    return pid->output;
}

// 简介：速度环PID控制（预留，需配合编码器使用）
// 参数：left_target - 左轮目标速度, right_target - 右轮目标速度
void Motor_Speed_PID_Control(int16_t left_target, int16_t right_target)
{
    // 注意：此函数需要实际的速度反馈（编码器）才能工作
    // 目前为预留接口，代码仅供参考

    // 示例代码结构：
    // static Motor_PID_t left_pid, right_pid;
    // float left_output, right_output;
    //
    // // 设置目标值
    // left_pid.target = left_target;
    // right_pid.target = right_target;
    //
    // // 获取当前速度（需要编码器反馈）
    // float left_current = Motor_GetLeftSpeed();
    // float right_current = Motor_GetRightSpeed();
    //
    // // PID计算
    // left_output = Motor_PID_Calculate(&left_pid, left_current);
    // right_output = Motor_PID_Calculate(&right_pid, right_current);
    //
    // // 输出到电机
    // Motor_SetSpeed((int16_t)left_output, (int16_t)right_output);
}

// ==================== 内部函数实现 ====================
// 简介：设置左轮电机方向
// 参数：direction - (0=正转, 1=反转)
static void Motor_SetLeftDirection(uint8_t direction)
{
    // 左轮方向控制 (AIN1/AIN2)
    if (direction == 0) {
        // 正转：AIN1=0, AIN2=1
        DL_GPIO_clearPins(Motor_AIN1_PORT, Motor_AIN1_PIN);
        DL_GPIO_setPins(Motor_AIN2_PORT, Motor_AIN2_PIN);
    } else {
        // 反转：AIN1=1, AIN2=0
        DL_GPIO_setPins(Motor_AIN1_PORT, Motor_AIN1_PIN);
        DL_GPIO_clearPins(Motor_AIN2_PORT, Motor_AIN2_PIN);
    }
}

// 简介：设置右轮电机方向
// 参数：direction - (0=正转, 1=反转)
static void Motor_SetRightDirection(uint8_t direction)
{
    // 右轮方向控制 (BIN1/BIN2)
    if (direction == 0) {
        // 正转：BIN1=0, BIN2=1
        DL_GPIO_clearPins(Motor_BIN1_PORT, Motor_BIN1_PIN);
        DL_GPIO_setPins(Motor_BIN2_PORT, Motor_BIN2_PIN);
    } else {
        // 反转：BIN1=1, BIN2=0
        DL_GPIO_setPins(Motor_BIN1_PORT, Motor_BIN1_PIN);
        DL_GPIO_clearPins(Motor_BIN2_PORT, Motor_BIN2_PIN);
    }
}

// 简介：设置PWM输出
// 参数：channel - 通道(0=左轮, 1=右轮), speed - 速度值(-100~100)
static void Motor_SetPWM(uint8_t channel, int16_t speed)
{
    uint16_t compare_value;
    uint16_t real_speed;

    // 将速度值(-100~100)转换为PWM占空比(0~PWM_PERIOD)
    if (speed < 0) {
        speed = -speed;  // 取绝对值
    }

    real_speed = 100 - speed;
    if(real_speed < 5)
    {
        real_speed = 5;// 最小速度为20,避免过小的输入导致电机满速运行
    }
    // 计算PWM占空比
    compare_value = (uint16_t)((real_speed * PWM_PERIOD) / 100);

    // 设置对应通道的PWM
    if (channel == 0) {
        // 左轮PWM - 使用TIMA1 CC0 (PB4)
        DL_TimerA_setCaptureCompareValue(PWM_0_INST, compare_value, DL_TIMER_CC_0_INDEX);
        // 确保Timer正在运行
        if (!DL_TimerA_isRunning(PWM_0_INST)) {
            DL_TimerA_startCounter(PWM_0_INST);
        }
    } else {
        // 右轮PWM - 使用TIMA1 CC1 (PB5)
        DL_TimerA_setCaptureCompareValue(PWM_0_INST, compare_value, DL_TIMER_CC_1_INDEX);
        if (!DL_TimerA_isRunning(PWM_0_INST)) {
            DL_TimerA_startCounter(PWM_0_INST);
        }
    }
}

/* ==================== 编码器解析实现 ==================== */

/* 左编码器状态（QEI硬件计数）
 * 注意: QEI计数器是有限位数，会溢出，需要处理 */
static Motor_Encoder_t g_left_encoder = {0, 0, 0, 1};

/* 右编码器状态（CAPTURE软件计数） */
static Motor_Encoder_t g_right_encoder = {0, 0, 0, 1};

/* 上次采样位置（用于计算差值） */
static int32_t g_left_last_pos = 0;
static int32_t g_right_last_pos = 0;

/* 左编码器原始累加器：直接累加QEI硬件计数值，避免整数除法截断
 * 读取位置时再除以2（2X→1X补偿），确保慢速转动不丢脉冲 */
static int32_t g_left_raw_accum = 0;

/* QEI计数器位数（用于溢出计算） */
#define QEI_COUNTER_BITS     16
#define QEI_COUNTER_MAX      (1 << QEI_COUNTER_BITS)  // 65536
#define QEI_HALF_MAX         (QEI_COUNTER_MAX / 2)    // 32768

/* QEI左电机中断处理 - 溢出和方向事件
 * 说明: QEI计数器溢出时修正累计位置，方向改变时记录方向
 * 中断向量: TIMG8_IRQHandler (QEI_0使用TIMG8) */
void TIMG8_IRQHandler(void)
{
    /* 获取待处理中断源 */
    DL_TIMER_IIDX intr = DL_Timer_getPendingInterrupt(QEI_0_INST);

    switch (intr) {
        case DL_TIMER_IIDX_ZERO:
            /* 计数器归零事件（溢出）
             * 从0跳到最大值，或者从最大值跳到0
             * 根据当前方向调整累加器（原始计数值） */
            if (g_left_encoder.direction == 1) {
                g_left_raw_accum += QEI_COUNTER_MAX;
            } else {
                g_left_raw_accum -= QEI_COUNTER_MAX;
            }
            break;

        case DL_TIMER_IIDX_DIR_CHANGE:
            /* 方向改变事件 - 读取QEI方向寄存器更新方向状态 */
            if (DL_Timer_getQEIDirection(QEI_0_INST) == DL_TIMER_QEI_DIR_UP) {
                g_left_encoder.direction = 1;
            } else {
                g_left_encoder.direction = -1;
            }
            break;

        default:
            break;
    }
}

/* 右电机GPIO中断处理 - A相边沿触发计数
 * 说明: 每检测到A相(PB10)边沿，根据B相(PB11)电平判断方向并更新位置
 * 注意: 由于sysconfig配置为上升沿触发，此处只处理A上升沿的情况
 * 中断向量: GROUP1_IRQHandler (GPIOB的所有GPIO中断合并到GROUP1) */
void GROUP1_IRQHandler(void)
{
    /* 获取待处理中断源 - 哪个引脚触发了中断 */
    DL_GPIO_IIDX intr = DL_GPIO_getPendingInterrupt(GPIOB);

    /* 判断是否是PB10(GB1)的中断（我们只配置了PB10中断） */
    if (intr == DL_GPIO_IIDX_DIO10) {
        /* 调试用：中断触发次数+1 */
        g_capture_interrupt_count++;

        /* 读取B相(PB11)电平状态来判断方向
         * 注意: DL_GPIO_readPins返回的是掩码值，需要判断是否等于引脚掩码 */
        uint8_t b_level = (DL_GPIO_readPins(Motor_GB2_PORT, Motor_GB2_PIN) == Motor_GB2_PIN) ? 1 : 0;
        //printf("b_level: %d\r\n",b_level);
        /* 编码器A/B相差90度：
         *   A超前B 90度 = 正转（计数增加）
         *   B超前A 90度 = 反转（计数减少）
         *
         * A上升沿时：
         *   B为高 -> B超前A 90度 -> 反转（计数减少）
         *   B为低 -> A超前B 90度 -> 正转（计数增加） */
        if (b_level) {
            /* B为高 -> 反转 */
            g_right_encoder.position++;
            g_right_encoder.direction = 1;
        } else {
            /* B为低 -> 正转 */
            g_right_encoder.position--;
            g_right_encoder.direction = -1;
        }

        /* 清除PB10的中断标志 */
        DL_GPIO_clearInterruptStatus(Motor_GB1_PORT, Motor_GB1_PIN);
    }
}

/* 简介: 初始化编码器
 * 说明: 启用QEI和CAPTURE的中断 */
void Motor_Encoder_Init(void)
{
    /* 初始化编码器状态 */
    g_left_encoder.position = 0;
    g_left_encoder.speed = 0;
    g_left_encoder.direction = 1;

    g_right_encoder.position = 0;
    g_right_encoder.speed = 0;
    g_right_encoder.direction = 1;

    g_left_last_pos = 0;
    g_right_last_pos = 0;

    /* 启用QEI中断 - 零事件(溢出)和方向改变 */
    DL_Timer_enableInterrupt(QEI_0_INST,
        DL_TIMER_INTERRUPT_ZERO_EVENT | DL_TIMER_INTERRUPT_DC_EVENT);

    /* 使能GPIOB中断 - PB10(GB1)的GPIO外部中断
     * 注意: sysconfig只设置了优先级和配置，但没有使能NVIC中断
     *       必须手动使能，否则GPIO中断无法触发 */
    NVIC_EnableIRQ(GPIOB_INT_IRQn);
}

/* 简介: 更新编码器速度
 * 调用频率: ENCODER_SAMPLE_MS毫秒一次
 *
 * 说明:
 * - 左电机: 读取QEI计数器当前值，计算与上次的差值得到增量
 * - 右电机: 读取软件累计position计算差值
 * - 方向只在非静止时更新
 *
 * 注意: QEI中断的ZERO事件会在溢出时修正g_left_encoder.position，
 *       此处计算差值时需注意与中断处理的时间点关系，避免重复累加 */
void Motor_Encoder_UpdateSpeed(void)
{
    int32_t current_pos;
    int32_t delta_pos;

    /* ========== 左电机 (QEI) ========== */
    /* 读取QEI计数器当前值（硬件计数器，从0到65535循环） */
    uint32_t qei_count = DL_Timer_getTimerCount(QEI_0_INST);

    /* 计算与上次的位置差（处理QEI计数器的模溢出）
     * 注意：此差值是采样周期内的增量，不包含之前的中断修正 */
    int32_t qei_delta = (int32_t)qei_count - (int32_t)g_left_last_pos;

    /* 检测模溢出: 差值的绝对值超过半量程说明发生了溢出 */
    if (qei_delta > (int32_t)QEI_HALF_MAX) {
        qei_delta -= QEI_COUNTER_MAX;
    } else if (qei_delta < -(int32_t)QEI_HALF_MAX) {
        qei_delta += QEI_COUNTER_MAX;
    }

    g_left_last_pos = (int32_t)qei_count;

    /* 累加原始QEI计数值（4X模式，每转52次）
     * 不在此处做除法，避免慢速时整数截断丢失脉冲 */
    g_left_raw_accum += qei_delta;

    /* 位置 = 累加器 / 4（4X→1X补偿，每转13次，与右轮一致） */
    g_left_encoder.position = g_left_raw_accum / 4;

    /* 计算速度（脉冲/秒）：先乘后除，避免截断
     * 原始增量 * 200 / 4 = 原始增量 * 50 */
    g_left_encoder.speed = qei_delta * (1000 / ENCODER_SAMPLE_MS) / 4;

    /* 方向判断: 只在非静止时更新 */
    if (qei_delta > 0) {
        g_left_encoder.direction = 1;
    } else if (qei_delta < 0) {
        g_left_encoder.direction = -1;
    }

    /* ========== 右电机 (CAPTURE软件计数) ========== */
    /* 右电机位置在CAPTURE中断中累加，此处直接读取差值计算速度 */
    current_pos = g_right_encoder.position;
    delta_pos = current_pos - g_right_last_pos;
    g_right_last_pos = current_pos;

    /* 计算速度（脉冲/秒） */
    g_right_encoder.speed = delta_pos * (1000 / ENCODER_SAMPLE_MS);

    /* 方向判断: 只在非静止时更新 */
    if (delta_pos > 0) {
        g_right_encoder.direction = 1;
    } else if (delta_pos < 0) {
        g_right_encoder.direction = -1;
    }
}

/* 简介: 获取左编码器RPM转速
 * 返回: 转速(转/分钟)
 * 计算公式: RPM = 脉冲/秒 / PPR * 60 */
int32_t Motor_GetLeftEncoderRPM(void)
{
    /* 使用位移优化: RPM = speed * 60 / PPR
     * 如果PPR是2的幂次方，可用右移代替除法 */
    #if (ENCODER_PPR == 16)
        return (g_left_encoder.speed * 60) >> 4;
    #elif (ENCODER_PPR == 8)
        return (g_left_encoder.speed * 60) >> 3;
    #elif (ENCODER_PPR == 4)
        return (g_left_encoder.speed * 60) >> 2;
    #elif (ENCODER_PPR == 2)
        return (g_left_encoder.speed * 60) >> 1;
    #else
        return (g_left_encoder.speed * 60) / ENCODER_PPR;
    #endif
}

/* 简介: 获取右编码器RPM转速
 * 返回: 转速(转/分钟) */
int32_t Motor_GetRightEncoderRPM(void)
{
    #if (ENCODER_PPR == 16)
        return (g_right_encoder.speed * 60) >> 4;
    #elif (ENCODER_PPR == 8)
        return (g_right_encoder.speed * 60) >> 3;
    #elif (ENCODER_PPR == 4)
        return (g_right_encoder.speed * 60) >> 2;
    #elif (ENCODER_PPR == 2)
        return (g_right_encoder.speed * 60) >> 1;
    #else
        return (g_right_encoder.speed * 60) / ENCODER_PPR;
    #endif
}

/* ==================== 编码器数据访问接口（补全缺失函数） ==================== */

/* 简介: 获取左编码器当前速度（脉冲/秒）
 * 返回: 左轮速度（正数=正转，负数=反转）
 * 说明: 速度值在Motor_Encoder_UpdateSpeed()调用后更新，
 *       单位是脉冲/秒，可直接用于PID控制 */
int32_t Motor_GetLeftEncoderSpeed(void)
{
    return g_left_encoder.speed;
}

/* 简介: 获取右编码器当前速度（脉冲/秒）
 * 返回: 右轮速度（正数=正转，负数=反转）
 * 说明: 速度值在Motor_Encoder_UpdateSpeed()调用后更新，
 *       单位是脉冲/秒，可直接用于PID控制 */
int32_t Motor_GetRightEncoderSpeed(void)
{
    return g_right_encoder.speed;
}

/* 简介: 获取左编码器累计位置（脉冲计数）
 * 返回: 左轮累计位置（正数=正转累计，负数=反转累计）
 * 说明: 这是一个带符号的累计值，从0开始，正转增加，反转减少。
 *       可用于计算行驶距离（需除以ENCODER_PPR得到转数） */
int32_t Motor_GetLeftEncoderPosition(void)
{
    return g_left_encoder.position;
}

/* 简介: 获取右编码器累计位置（脉冲计数）
 * 返回: 右轮累计位置（正数=正转累计，负数=反转累计）
 * 说明: 这是一个带符号的累计值，从0开始，正转增加，反转减少。
 *       可用于计算行驶距离（需除以ENCODER_PPR得到转数） */
int32_t Motor_GetRightEncoderPosition(void)
{
    return g_right_encoder.position;
}

/* 简介: 重置左编码器计数
 * 说明: 将左编码器的位置、速度清零，同时重置上次采样位置。
 *       适用于需要校准或重新计数的场景 */
void Motor_ResetLeftEncoder(void)
{
    g_left_raw_accum = 0;
    g_left_encoder.position = 0;
    g_left_encoder.speed = 0;
    g_left_encoder.direction = 1;
    g_left_last_pos = 0;
}

/* 简介: 重置右编码器计数
 * 说明: 将右编码器的位置、速度清零，同时重置上次采样位置。
 *       适用于需要校准或重新计数的场景 */
void Motor_ResetRightEncoder(void)
{
    g_right_encoder.position = 0;
    g_right_encoder.speed = 0;
    g_right_encoder.direction = 1;
    g_right_last_pos = 0;
}

/* 简介: 调试用：获取CAPTURE中断触发次数（诊断用）
 * 返回: TIMG6_IRQHandler被触发的次数
 * 说明: 用于诊断编码器中断是否正常工作，调试完成后可移除 */
uint32_t Motor_GetCaptureInterruptCount(void)
{
    return g_capture_interrupt_count;
}
