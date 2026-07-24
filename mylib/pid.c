#include "ti_msp_dl_config.h"
#include "pid.h"

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
              float integral_max, float out_min, float out_max)
{
    /* 设置PID控制参数 */
    pid->kp = kp;   // 比例系数：响应当前误差
    pid->ki = ki;   // 积分系数：消除稳态误差
    pid->kd = kd;   // 微分系数：预测误差变化趋势
    
    /* 清零状态变量 */
    pid->error = 0;       // 当前误差
    pid->last_error = 0;  // 上一次误差（用于微分项计算）
    pid->integral = 0;    // 积分累积值
    pid->output = 0;      // 控制器输出
    
    /* 设置限幅参数 */
    pid->integral_max = integral_max;  // 积分限幅，防止积分饱和
    pid->output_min = out_min;         // 输出下限
    pid->output_max = out_max;         // 输出上限
}

/**
 * @brief  PID控制器计算函数
 * @param  pid: PID结构体指针
 * @param  target: 目标设定值 (Setpoint)
 * @param  current: 当前实际值 (Process Variable)
 * @retval 计算得到的PID输出值
 * @note   采用位置式PID算法，包含积分限幅和输出限幅
 */
float PID_Calc(PID_t *pid, float target, float current)
{
    /* 1. 计算当前误差：目标值减去当前值 */
    pid->error = target - current;

    /* 2. 积分项累加：误差累积用于消除稳态误差 */
    pid->integral += pid->error;

    /* 3. 积分限幅：防止积分饱和（Anti-windup） */
    if (pid->integral > pid->integral_max)
        pid->integral = pid->integral_max;
    else if (pid->integral < -pid->integral_max)
        pid->integral = -pid->integral_max;

    /* 4. PID公式计算输出：
     *    比例项：kp * error (当前误差)
     *    积分项：ki * integral (误差累积)
     *    微分项：kd * (error - last_error) (误差变化率)
     */
    pid->output =
        pid->kp * pid->error +                           /* 比例控制 */
        pid->ki * pid->integral +                        /* 积分控制 */
        pid->kd * (pid->error - pid->last_error);        /* 微分控制 */

    /* 5. 输出限幅：确保输出在执行器有效范围内 */
    if (pid->output > pid->output_max)
        pid->output = pid->output_max;
    else if (pid->output < pid->output_min)
        pid->output = pid->output_min;

    /* 6. 保存当前误差，供下次微分项计算使用 */
    pid->last_error = pid->error;

    /* 返回计算结果 */
    return pid->output;
}
