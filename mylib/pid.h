#ifndef __PID_H
#define __PID_H

#include "mylib/delay.h"
#include <stdint.h>
#include "delay.h"
/**
 * @brief PID控制器结构体
 * @note  包含PID参数、状态变量和限幅设置
 */
typedef struct
{
    /* PID控制参数 */
    float kp;           /**< 比例系数 (Proportional gain) */
    float ki;           /**< 积分系数 (Integral gain) */
    float kd;           /**< 微分系数 (Derivative gain) */

    /* 状态变量 */
    float error;        /**< 当前误差 */
    float last_error;   /**< 上一次误差 */
    float integral;     /**< 积分累积值 */
    float output;       /**< 控制器输出 */

    /* 限幅参数 */
    float integral_max; /**< 积分限幅值，防止积分饱和 */
    float output_max;   /**< 输出最大值限制 */
    float output_min;   /**< 输出最小值限制 */
} PID_t;

/**
 * @brief  PID控制器初始化函数
 * @param  pid: PID结构体指针
 * @param  kp: 比例系数 (Proportional gain)
 * @param  ki: 积分系数 (Integral gain)
 * @param  kd: 微分系数 (Derivative gain)
 * @param  integral_max: 积分限幅值，防止积分饱和
 * @param  out_min: 输出最小值限制
 * @param  out_max: 输出最大值限制
 * @retval 无
 * @note   初始化时会将误差、积分项和输出清零
 */
void PID_Init(PID_t *pid, float kp, float ki, float kd,
              float integral_max, float out_min, float out_max);

/**
 * @brief  PID控制器计算函数
 * @param  pid: PID结构体指针
 * @param  target: 目标设定值 (Setpoint)
 * @param  current: 当前实际值 (Process Variable)
 * @retval 计算得到的PID输出值
 * @note   采用位置式PID算法，包含积分限幅和输出限幅
 */
float PID_Calc(PID_t *pid, float target, float current);


float PID_GetError(float target, float current);


#endif /* __PID_H */