#include "ti_msp_dl_config.h"
#include "mylib/find.h"
#include "mylib/Motor.h"

/* ==================== 内部变量 ==================== */

/* 传感器位置权重表（从 G1 到 G8） */
static const int8_t ir_weight[IR_NUM] = {
    IR_WEIGHT_G1,   /* G1: -7 */
    IR_WEIGHT_G2,   /* G2: -5 */
    IR_WEIGHT_G3,   /* G3: -3 */
    IR_WEIGHT_G4,   /* G4: -1 */
    IR_WEIGHT_G5,   /* G5:  1 */
    IR_WEIGHT_G6,   /* G6:  3 */
    IR_WEIGHT_G7,   /* G7:  5 */
    IR_WEIGHT_G8    /* G8:  7 */
};

/* 上一次有效位置（用于黑线丢失时保持） */
static int16_t g_last_position = 0;

/* ==================== 第1层：传感器数据读取 ==================== */

/*
 * 简介：读取8路循迹传感器数据到数组
 * 参数：ir - 存储传感器数据的数组（长度至少为 IR_NUM）
 * 说明：ir[i] = 1 表示检测到黑线，ir[i] = 0 表示检测到白线
 *       注意：传感器检测到黑线时输出高电平，与 ir[i] 逻辑一致
 */
void IR_Read(uint8_t *ir)
{
    /* DL_GPIO_readPins 读取指定端口的指定引脚电平 */
    ir[0] = (DL_GPIO_readPins(XG_G1_PORT, XG_G1_PIN) ? 1 : 0);
    ir[1] = (DL_GPIO_readPins(XG_G2_PORT, XG_G2_PIN) ? 1 : 0);
    ir[2] = (DL_GPIO_readPins(XG_G3_PORT, XG_G3_PIN) ? 1 : 0);
    ir[3] = (DL_GPIO_readPins(XG_G4_PORT, XG_G4_PIN) ? 1 : 0);
    ir[4] = (DL_GPIO_readPins(XG_G5_PORT, XG_G5_PIN) ? 1 : 0);
    ir[5] = (DL_GPIO_readPins(XG_G6_PORT, XG_G6_PIN) ? 1 : 0);
    ir[6] = (DL_GPIO_readPins(XG_G7_PORT, XG_G7_PIN) ? 1 : 0);
    ir[7] = (DL_GPIO_readPins(XG_G8_PORT, XG_G8_PIN) ? 1 : 0);
}

/* ==================== 第1层：位置计算 ==================== */

/*
 * 简介：计算黑线位置（加权平均法）
 * 参数：ir - 传感器数据数组
 * 返回：黑线位置值，范围 -7 ~ +7
 *       负数=线偏左，正数=线偏右，0=线在正中间
 * 算法：position = Σ(weight[i] × ir[i]) / Σ(ir[i])
 *       当所有传感器都没有检测到黑线时，保持上一次的有效位置
 * 示例：若 G3、G4 检测到黑线 → position = (-3×1 + -1×1) / 2 = -2
 */
float IR_GetPosition(uint8_t *ir)
{
    int16_t weighted_sum = 0;   /* 加权和 */
    uint8_t active_count = 0;   /* 检测到黑线的传感器数量 */

    uint8_t i;
    for (i = 0; i < IR_NUM; i++) {
        if (ir[i] == 1) {
            weighted_sum += ir_weight[i];
            active_count++;
        }
    }

    /* 如果没有任何传感器检测到黑线，保持上一次的位置 */
    if (active_count == 0) {
        return g_last_position;
    }

    /* 计算加权平均位置 */
    float position = (float)weighted_sum / (float)active_count;

    /* 保存有效位置 */
    g_last_position = position;

    return position;
}

/*
 * 简介：获取偏离中心线的偏差值
 * 参数：ir - 传感器数据数组
 * 返回：偏差值（同 IR_GetPosition）
 * 说明：直接调用 IR_GetPosition，封装为语义更清晰的接口
 *       负数=需要左转修正，正数=需要右转修正
 */
float IR_GetError(uint8_t *ir)
{
    return IR_GetPosition(ir);
}

/*
 * 简介：获取检测到黑线的传感器数量
 * 参数：ir - 传感器数据数组
 * 返回：检测到黑线的传感器个数（0~8）
 */
uint8_t IR_GetSensorCount(uint8_t *ir)
{
    uint8_t count = 0;
    uint8_t i;

    for (i = 0; i < IR_NUM; i++) {
        if (ir[i] == 1) {
            count++;
        }
    }

    return count;
}

/* ==================== 第2层：线状态检测 ==================== */

/*
 * 简介：检查是否所有传感器都检测到黑线
 * 参数：ir - 传感器数据数组
 * 返回：1=全黑，0=非全黑
 * 说明：通常出现在十字路口中心或到达终点标记
 */
uint8_t IR_IsAllBlack(uint8_t *ir)
{
    uint8_t i;
    for (i = 0; i < IR_NUM; i++) {
        if (ir[i] == 0) {
            return 0;   /* 有一个不黑就不是全黑 */
        }
    }
    return 1;
}

/*
 * 简介：检查是否所有传感器都检测不到黑线
 * 参数：ir - 传感器数据数组
 * 返回：1=全白，0=非全白
 * 说明：通常出现在完全脱离轨道时
 */
uint8_t IR_IsAllWhite(uint8_t *ir)
{
    uint8_t i;
    for (i = 0; i < IR_NUM; i++) {
        if (ir[i] == 1) {
            return 0;   /* 有一个黑就不是全白 */
        }
    }
    return 1;
}

/*
 * 简介：检查是否丢失黑线
 * 参数：ir - 传感器数据数组
 * 返回：1=黑线丢失，0=黑线正常
 * 说明：没有任何传感器检测到黑线即为丢失
 */
uint8_t IR_IsLineLost(uint8_t *ir)
{
    return IR_IsAllWhite(ir);
}

/*
 * 简介：检查是否处于交叉路口
 * 参数：ir - 传感器数据数组
 * 返回：1=处于交叉路口，0=正常路段
 * 说明：当检测到黑线的传感器数量 >= 4 时判定为交叉路口
 *       （可根据实际赛道宽度调整阈值 IR_JUNCTION_THRESHOLD）
 */
#define IR_JUNCTION_THRESHOLD  4

uint8_t IR_IsAtJunction(uint8_t *ir)
{
    uint8_t count = IR_GetSensorCount(ir);
    return (count >= IR_JUNCTION_THRESHOLD) ? 1 : 0;
}

/*
 * 简介：获取当前循迹状态（综合判断）
 * 参数：ir - 传感器数据数组
 * 返回：当前循迹状态枚举值
 * 说明：按优先级顺序判断：全黑 > 全白 > 交叉路口 > 正常循迹
 */
IR_Status_t IR_GetStatus(uint8_t *ir)
{
    if (IR_IsAllBlack(ir)) {
        return IR_STATUS_ALL_BLACK;
    }

    if (IR_IsAllWhite(ir)) {
        return IR_STATUS_ALL_WHITE;
    }

    if (IR_IsAtJunction(ir)) {
        return IR_STATUS_AT_JUNCTION;
    }

    return IR_STATUS_ON_LINE;
}

/* ==================== 第3层：PID循迹控制 ==================== */

/*
 * 简介：初始化循迹PID参数
 * 参数：pid  - PID结构体指针
 *       Kp   - 比例系数（建议初始值：2.0 ~ 5.0）
 *       Ki   - 积分系数（建议初始值：0.0 ~ 0.5）
 *       Kd   - 微分系数（建议初始值：1.0 ~ 3.0）
 * 说明：调用后所有内部状态清零，setpoint 默认为 0（黑线在中间）
 */
void IR_PID_Init(IR_PID_t *pid, float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->setpoint = 0.0f;   /* 目标位置：黑线在中间 */
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}

/*
 * 简介：循迹PID计算
 * 参数：pid              - PID结构体指针
 *       current_position - 当前黑线位置（来自 IR_GetPosition）
 * 返回：PID修正输出值（可直接用于 Motor_TurnSmall 的 error 参数）
 * 说明：输出值范围约为 -100 ~ +100
 *       - 负数：需要左转
 *       - 正数：需要右转
 *       - 0：不需要修正
 */
float IR_PID_Calculate(IR_PID_t *pid, int16_t current_position)
{
    /* 计算位置偏差 */
    pid->error = pid->setpoint - (float)current_position;

    /* 比例项：P = Kp × error */
    float p_term = pid->Kp * pid->error;

    /* 积分项：I = Ki × ∫error（带积分限幅，防止积分饱和） */
    pid->integral += pid->error;
    if (pid->integral > 50.0f) {
        pid->integral = 50.0f;
    } else if (pid->integral < -50.0f) {
        pid->integral = -50.0f;
    }
    float i_term = pid->Ki * pid->integral;

    /* 微分项：D = Kd × d(error)/dt */
    float d_term = pid->Kd * (pid->error - pid->last_error);
    pid->last_error = pid->error;

    /* 计算总输出 */
    pid->output = p_term + i_term + d_term;

    return pid->output;
}

/*
 * 简介：循迹主控函数（一站式循迹控制）
 * 参数：pid        - PID结构体指针
 *       base_speed - 基础速度(0~100)，值越大越快
 * 返回：当前循迹状态
 * 说明：此函数完成以下步骤：
 *       1. 读取8路传感器数据
 *       2. 判断当前循迹状态
 *       3. 若正常循迹：计算位置 → PID修正 → 差速转向
 *       4. 若黑线丢失：保持上次方向缓慢前进尝试找回
 *       5. 若交叉路口：直行通过
 *       6. 若全黑：停止
 *       7. 若全白：停止
 */
IR_Status_t IR_Track(IR_PID_t *pid, int16_t base_speed)
{
    uint8_t ir[IR_NUM];

    /* 步骤1：读取传感器数据 */
    IR_Read(ir);

    /* 步骤2：判断当前状态 */
    IR_Status_t status = IR_GetStatus(ir);

    switch (status) {
    case IR_STATUS_ON_LINE:
    {
        /* 正常循迹：计算位置 → PID修正 → 差速转向 */
        int16_t position = IR_GetPosition(ir);
        float correction = IR_PID_Calculate(pid, position);

        /* 根据PID输出调整左右轮速度 */
        int16_t left_speed  = base_speed - (int16_t)correction;
        int16_t right_speed = base_speed + (int16_t)correction;

        /* 限幅处理 */
        if (left_speed > 100)  left_speed = 100;
        if (left_speed < -100) left_speed = -100;
        if (right_speed > 100)  right_speed = 100;
        if (right_speed < -100) right_speed = -100;

        Motor_SetSpeed(left_speed, right_speed);
        break;
    }

    case IR_STATUS_LINE_LOST:
    case IR_STATUS_ALL_WHITE:
    {
        /* 黑线丢失：降低速度，保持上一次方向缓慢前进 */
        int16_t search_speed = base_speed / 3;   /* 降低到1/3速度 */
        if (g_last_position < 0) {
            /* 上次线偏左，缓慢左转寻找 */
            Motor_SetSpeed(-search_speed, search_speed);
        } else if (g_last_position > 0) {
            /* 上次线偏右，缓慢右转寻找 */
            Motor_SetSpeed(search_speed, -search_speed);
        } else {
            /* 无历史位置，缓慢前进 */
            Motor_SetSpeed(search_speed, search_speed);
        }
        break;
    }

    case IR_STATUS_AT_JUNCTION:
    {
        /* 交叉路口：直行通过 */
        Motor_SetSpeed(base_speed, base_speed);
        break;
    }

    case IR_STATUS_ALL_BLACK:
    {
        /* 全黑：停止（可能到达终点或出错） */
        Motor_Stop();
        break;
    }

    default:
        break;
    }

    return status;
}