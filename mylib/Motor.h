#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "ti_msp_dl_config.h"

// ==================== 速度参数 ====================
// 速度范围：-100 ~ 100 (负数表示后退/反转)
#define MOTOR_SPEED_MAX      100
#define MOTOR_SPEED_MIN      -100
#define MOTOR_SPEED_DEFAULT  50      // 默认速度

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

// 设置单轮速度
void Motor_SetLeftSpeed(int16_t speed);
void Motor_SetRightSpeed(int16_t speed);

// 获取当前速度
int16_t Motor_GetLeftSpeed(void);
int16_t Motor_GetRightSpeed(void);

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

// ==================== PID控制（预留接口） ====================
// 初始化PID参数
void Motor_PID_Init(Motor_PID_t *pid, float Kp, float Ki, float Kd);

// 单轮速度PID计算
float Motor_PID_Calculate(Motor_PID_t *pid, float current_value);

// 速度环PID控制（预留）
void Motor_Speed_PID_Control(int16_t left_target, int16_t right_target);

#endif