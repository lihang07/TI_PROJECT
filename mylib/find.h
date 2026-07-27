#ifndef __FIND_H__
#define __FIND_H__

#include "ti_msp_dl_config.h"

/* ==================== 循迹传感器数量 ==================== */
#define IR_NUM 8

/* 循迹模块检测到黑线时的输出电平：高电平填1，低电平填0 */
#define IR_BLACK_LEVEL 1

/* ==================== 传感器位置权重（用于位置计算） ==================== */
/* 8个传感器从左到右排列，权重值供 IR_GetPosition 使用 */
#define IR_WEIGHT_G1  -7
#define IR_WEIGHT_G2  -5
#define IR_WEIGHT_G3  -3
#define IR_WEIGHT_G4  -1
#define IR_WEIGHT_G5   1
#define IR_WEIGHT_G6   3
#define IR_WEIGHT_G7   5
#define IR_WEIGHT_G8   7

/* ==================== 循迹状态枚举 ==================== */
typedef enum {
    IR_STATUS_ON_LINE = 0,      /* 正常循迹中 */
    IR_STATUS_LINE_LOST,        /* 黑线丢失（所有传感器都检测不到） */
    IR_STATUS_AT_JUNCTION,      /* 处于十字路口（多个传感器同时检测到） */
    IR_STATUS_ALL_BLACK,        /* 所有传感器都在黑线上 */
    IR_STATUS_ALL_WHITE         /* 所有传感器都不在黑线上 */
} IR_Status_t;

/* ==================== 循迹PID结构体 ==================== */
typedef struct {
    float Kp;         /* 比例系数 */
    float Ki;         /* 积分系数 */
    float Kd;         /* 微分系数 */
    float setpoint;   /* 目标位置（通常为0，即黑线在中间） */
    float error;      /* 当前偏差 */
    float last_error; /* 上一次偏差 */
    float integral;   /* 积分累计 */
    float output;     /* PID输出 */
} IR_PID_t;

/* ==================== 第1层：传感器数据读取与位置计算 ==================== */

/* 读取所有循迹传感器数据到数组 */
void IR_Read(uint8_t *ir);

/* 计算黑线位置（加权平均法） */
/* 返回值：负数=线偏左，正数=线偏右，0=线在中间 */
/* 范围：-7 ~ +7（对应 G1~G8 的位置权重） */
float IR_GetPosition(uint8_t *ir);

/* 获取偏离中心线的偏差值（直接用于转向修正） */
/* 返回值同 IR_GetPosition，负数表示需要左转，正数表示需要右转 */
float IR_GetError(uint8_t *ir);

/* 获取检测到黑线的传感器数量 */
uint8_t IR_GetSensorCount(uint8_t *ir);

/* ==================== 第2层：线状态检测 ==================== */

/* 检查是否所有传感器都检测到黑线（如十字路口中心） */
uint8_t IR_IsAllBlack(uint8_t *ir);

/* 检查是否所有传感器都检测不到黑线（完全离线） */
uint8_t IR_IsAllWhite(uint8_t *ir);

/* 检查是否丢失黑线（没有任何传感器检测到） */
uint8_t IR_IsLineLost(uint8_t *ir);

/* 检查是否处于交叉路口（多个传感器同时检测到黑线） */
uint8_t IR_IsAtJunction(uint8_t *ir);

/* 获取当前循迹状态（综合判断） */
IR_Status_t IR_GetStatus(uint8_t *ir);

/* ==================== 第3层：PID循迹控制 ==================== */

/* 初始化循迹PID参数 */
void IR_PID_Init(IR_PID_t *pid, float Kp, float Ki, float Kd);

/* 循迹PID计算（输入当前位置，输出修正值） */
float IR_PID_Calculate(IR_PID_t *pid, int16_t current_position);

/* 循迹主控函数（读取传感器 + PID计算 + 电机控制） */
/* base_speed: 基础速度(0~100), 返回当前循迹状态 */
IR_Status_t IR_Track(IR_PID_t *pid, int16_t base_speed);

#endif
