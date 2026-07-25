#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "ti_msp_dl_config.h"

// ==================== 速度参数 ====================
// 速度范围：-100 ~ 100 (负数表示后退/反转)
#define MOTOR_SPEED_MAX      100
#define MOTOR_SPEED_MIN      -100
#define MOTOR_SPEED_DEFAULT  50      // 默认速度
#define MOTOR_MAX_SPEED_PPS  2300.0f // cmd=+/-100 约等于 +/-2300 pulse/s

// ==================== PID参数结构体（预留） ====================
typedef struct {
    float Kp;         // 比例系数
    float Ki;         // 积分系数
    float Kd;         // 微分系数
    float target;     // 目标值
    float error;      // 当前误差
    float last_error; // 上一次误差
    float integral;   // 积分累计
    float output;     // PID输出
} Motor_PID_t;

// ==================== 基础控制 ====================
// 电机使能/禁用（STBY引脚）
void Motor_Enable(void);     // STBY = 1, 电机使能
void Motor_Disable(void);    // STBY = 0, 电机禁用

// 停止与刹车
void Motor_Stop(void);       // 快速停止（切断供电，滑行）
void Motor_Brake(void);      // 刹车减速（PWM减为0，缓慢停止）

// ==================== 基础运动（固定速度） ====================
void Motor_Forward(void);     // 前进（双轮同速）
void Motor_Backward(void);    // 后退（双轮同速）

// ==================== 速度控制（核心PWM调速） ====================
// 设置双轮独立速度（范围 -100 ~ 100）
void Motor_SetSpeed(int16_t left_speed, int16_t right_speed);

// Set independent wheel speeds in encoder pulses per second.
void Motor_SetSpeedPps(float left_pps, float right_pps);

// 设置单轮速度
void Motor_SetLeftSpeed(int16_t speed);
void Motor_SetRightSpeed(int16_t speed);

// 获取当前速度
int16_t Motor_GetLeftSpeed(void);
int16_t Motor_GetRightSpeed(void);

// 将电机命令值(-100~100)换算为估算编码器速度(pulse/s)
float Motor_CmdToSpeedPps(int16_t cmd);

// ==================== 转向控制（差速控制 - 循迹核心） ====================
// 差速转向：两轮速度不同实现转向
void Motor_TurnLeft(int16_t speed);    // 左转：降低左轮速度
void Motor_TurnRight(int16_t speed);   // 右转：降低右轮速度

// 原地旋转
void Motor_Spin(int16_t direction);    // direction: -1=原地左转, 1=原地右转

// 小幅度转向修正（循迹中使用）
void Motor_TurnSmall(int16_t error);   // 根据偏差error微调转向

// ==================== 速度曲线（加减速） ====================
// 设置加速度（值越大加速越快）
void Motor_SetAcceleration(uint16_t accel);

// 速度斜坡更新（需在循环中调用，实现平滑加减速）
void Motor_SpeedRamp(void);

// ==================== 编码器参数（需根据实际PPR设置） ====================
// 编码器线数（PPR = Pulse Per Revolution，每转脉冲数）
// 注意：这个值需要根据你的编码器规格书设置
#define ENCODER_PPR           13       // 默认11线PPR，可根据实际修改

// 速度采样周期（毫秒）- 影响速度更新频率
#define ENCODER_SAMPLE_MS      10      // TIMG7 is configured for a 10 ms period

// ==================== 编码器数据结构 ====================
/* 编码器状态结构体
 * 使用volatile确保多线程/中断访问时的数据一致性 */
typedef struct {
    volatile int32_t position;       /* 累计位置（脉冲计数） */
    volatile int32_t last_position;  /* 上次采样位置（用于计算差值） */
    volatile int32_t speed;          /* 当前速度（脉冲/秒） */
    volatile int16_t direction;      /* 方向：1=正转, -1=反转 */
} Motor_Encoder_t;

// ==================== 编码器测速接口 ====================

// 初始化编码器（QEI + CAPTURE）  重要！！！！

// 说明: 初始化左电机QEI和右电机CAPTURE，配置中断
void Motor_Encoder_Init(void);

// 获取左轮编码器速度（脉冲/秒，正负表示方向）,满速为2200
int32_t Motor_GetLeftEncoderSpeed(void);

// 获取右轮编码器速度（脉冲/秒，正负表示方向）,满速为2200
int32_t Motor_GetRightEncoderSpeed(void);

//转换编码器速度为PWM值
float Motor_SpeedToPWM(int32_t speed);

// 获取左轮编码器位置（累计脉冲数）
int32_t Motor_GetLeftEncoderPosition(void);

// 获取右轮编码器位置（累计脉冲数）
int32_t Motor_GetRightEncoderPosition(void);

// 获取左轮编码器RPM转速（转/分钟，正负表示方向）
int32_t Motor_GetLeftEncoderRPM(void);

// 获取右轮编码器RPM转速（转/分钟，正负表示方向）
int32_t Motor_GetRightEncoderRPM(void);

// 重置左编码器计数
void Motor_ResetLeftEncoder(void);

// 重置右编码器计数
void Motor_ResetRightEncoder(void);

// 更新编码器速度（在定时器中调用，周期=ENCODER_SAMPLE_MS）
// 说明: 根据位置差分计算速度，需定期调用
void Motor_Encoder_UpdateSpeed(void);

// 调试用：获取CAPTURE中断触发次数（诊断用）
// 返回: TIMG6_IRQHandler被触发的次数
uint32_t Motor_GetCaptureInterruptCount(void);

// ==================== 速度环PID控制（预留） ====================
// 初始化PID参数
void Motor_PID_Init(Motor_PID_t *pid, float Kp, float Ki, float Kd);

// 单轮速度PID计算
float Motor_PID_Calculate(Motor_PID_t *pid, float current_value);

// 速度环PID控制（预留）
void Motor_Speed_PID_Control(int16_t left_target, int16_t right_target);

#endif
