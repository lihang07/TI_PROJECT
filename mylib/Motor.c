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
static uint16_t g_acceleration = 5;

// PWM周期值（对应syscfg中的timerCount=3，实际PWM周期+1）
#define PWM_PERIOD  3

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
    if (left_speed >= 0) {
        Motor_SetLeftDirection(0);  // 左轮正转
    } else {
        Motor_SetLeftDirection(1);   // 左轮反转
    }

    if (right_speed >= 0) {
        Motor_SetRightDirection(0);  // 右轮正转
    } else {
        Motor_SetRightDirection(1);  // 右轮反转
    }

    // 设置PWM
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

// ==================== 转向控制（差速控制 - 循迹核心） ====================
// 简介：左转（降低左轮速度，右轮正常）
// 参数：speed - 基础速度(-100~100)
void Motor_TurnLeft(int16_t speed)
{
    int16_t left_speed = (int16_t)(speed * 0.3f);   // 左轮减速到30%
    int16_t right_speed = speed;                    // 右轮全速

    Motor_SetSpeed(left_speed, right_speed);
}

// 简介：右转（降低右轮速度，左轮正常）
// 参数：speed - 基础速度(-100~100)
void Motor_TurnRight(int16_t speed)
{
    int16_t left_speed = speed;                     // 左轮全速
    int16_t right_speed = (int16_t)(speed * 0.3f);  // 右轮减速到30%

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

    // 将速度值(-100~100)转换为PWM占空比(0~PWM_PERIOD)
    if (speed < 0) {
        speed = -speed;  // 取绝对值
    }

    // 计算PWM占空比
    compare_value = (uint16_t)((speed * PWM_PERIOD) / 100);

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