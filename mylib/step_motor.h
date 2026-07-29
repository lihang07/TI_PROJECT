#ifndef STEP_MOTOR_H
#define STEP_MOTOR_H

#include <stdint.h>

/**
 * @brief 初始化 2 号步进电机。
 *
 * 配置默认方向、清除历史状态并使能对应定时器中断。
 */
void step_motor_init(void);

/**
 * @brief 设置 2 号步进电机方向。
 * @param direction 0 表示低电平，非 0 表示高电平。
 */
void step_motor_dir_set(uint8_t direction);

/**
 * @brief 启动 2 号步进电机的 STEP 脉冲输出。
 */
void step_motor_step_set(void);

/**
 * @brief 关闭 2 号步进电机的 STEP 脉冲输出。
 */
void step_motor_step_close(void);

/**
 * @brief 设置 2 号步进电机速度。
 * @param speed 目标角速度，单位为度每秒；传入 0 将立即停止电机。
 */
void step_motor_speed_set(uint32_t speed);

/**
 * @brief 驱动 2 号步进电机转动指定角度。
 * @param angle 目标转角，单位为度。
 */
void step_motor_angle(uint32_t angle);

/**
 * @brief 初始化 1 号步进电机。
 *
 * 配置默认方向、清除历史状态并使能对应定时器中断。
 */
void step_motor1_init(void);

/**
 * @brief 设置 1 号步进电机方向。
 * @param direction 0 表示低电平，非 0 表示高电平。
 */
void step_motor1_dir_set(uint8_t direction);

/**
 * @brief 启动 1 号步进电机的 STEP 脉冲输出。
 */
void step_motor1_step_set(void);

/**
 * @brief 关闭 1 号步进电机的 STEP 脉冲输出。
 */
void step_motor1_step_close(void);

/**
 * @brief 设置 1 号步进电机速度。
 * @param speed 目标角速度，单位为度每秒；传入 0 将立即停止电机。
 */
void step_motor1_speed_set(uint32_t speed);

/**
 * @brief 驱动 1 号步进电机转动指定角度。
 * @param angle 目标转角，单位为度。
 */
void step_motor1_angle(uint32_t angle);

#endif
