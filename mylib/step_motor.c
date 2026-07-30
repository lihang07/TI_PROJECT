// #include "ti_msp_dl_config.h"
// #include "step_motor.h"

// /* 0.05625 度对应一个脉冲，因此每度需要 160 / 9 个脉冲。 */
// #define STEP_MOTOR_STEPS_PER_DEGREE_NUMERATOR 160U
// #define STEP_MOTOR_STEPS_PER_DEGREE_DENOMINATOR 9U

// /* 定时器装载值限制为 16 位最大值，避免超出当前 PWM 配置范围。 */
// #define STEP_MOTOR_MAX_PERIOD 65535U

// /**
//  * @brief 保存一个步进电机的硬件资源和运行状态。
//  */
// typedef struct {
//     GPTIMER_Regs *timer;                  /* 产生 STEP 脉冲的定时器实例。 */
//     GPIO_Regs *direction_port;            /* DIR 引脚所在 GPIO 端口。 */
//     uint32_t direction_pin;               /* DIR 引脚位掩码。 */
//     IRQn_Type interrupt_number;           /* 定时器中断号。 */
//     uint32_t timer_clock_frequency;       /* 定时器工作时钟频率。 */
//     DL_TIMER_CC_INDEX compare_index;      /* STEP 输出使用的比较通道。 */
//     volatile uint32_t *steps_remaining;   /* 当前动作尚未输出的脉冲数量。 */
// } StepMotorConfig;

// /* 2 号电机的剩余脉冲数会在主循环和中断之间共享，因此必须使用 volatile。 */
// static volatile uint32_t g_motor2_steps_remaining = 0U;

// /* 1 号电机的剩余脉冲数会在主循环和中断之间共享，因此必须使用 volatile。 */
// static volatile uint32_t g_motor1_steps_remaining = 0U;

// /* 汇总 2 号电机的硬件配置，避免在多个函数中重复使用寄存器宏。 */
// static const StepMotorConfig g_motor2_config = {
//     .timer                 = PWM2_INST,
//     .direction_port        = STEP_MOTOR_PORT,
//     .direction_pin         = STEP_MOTOR_DIR2_PIN,
//     .interrupt_number      = PWM2_INST_INT_IRQN,
//     .timer_clock_frequency = PWM2_INST_CLK_FREQ,
//     .compare_index         = GPIO_PWM2_C0_IDX,
//     .steps_remaining       = &g_motor2_steps_remaining,
// };

// /* 汇总 1 号电机的硬件配置，STEP 使用 PA14/TIMG12，DIR 使用 PA15。 */
// static const StepMotorConfig g_motor1_config = {
//     .timer                 = PWM1_INST,
//     .direction_port        = STEP_MOTOR1_PORT,
//     .direction_pin         = STEP_MOTOR1_DIR1_PIN,
//     .interrupt_number      = PWM1_INST_INT_IRQN,
//     .timer_clock_frequency = PWM1_INST_CLK_FREQ,
//     .compare_index         = GPIO_PWM1_C0_IDX,
//     .steps_remaining       = &g_motor1_steps_remaining,
// };

// /**
//  * @brief 将目标角度换算为所需的 STEP 脉冲数。
//  * @param angle 目标转角，单位为度。
//  * @return 对应的整数脉冲数。
//  */
// static uint32_t step_motor_steps_from_angle(uint32_t angle)
// {
//     /* 先提升到 64 位再相乘，避免大角度时发生 32 位溢出。 */
//     return (uint32_t) (((uint64_t) angle * STEP_MOTOR_STEPS_PER_DEGREE_NUMERATOR) /
//                        STEP_MOTOR_STEPS_PER_DEGREE_DENOMINATOR);
// }

// /**
//  * @brief 根据目标角速度计算定时器装载值。
//  * @param speed 目标角速度，单位为度每秒。
//  * @param timer_clock_frequency 当前定时器工作时钟频率。
//  * @return 合法的定时器装载值。
//  */
// static uint32_t step_motor_period_from_speed(uint32_t speed,
//                                              uint32_t timer_clock_frequency)
// {
//     uint32_t step_frequency;
//     uint32_t timer_period;

//     /* 先将角速度换算成 STEP 脉冲频率。 */
//     step_frequency = (uint32_t) (((uint64_t) speed * STEP_MOTOR_STEPS_PER_DEGREE_NUMERATOR) /
//                                  STEP_MOTOR_STEPS_PER_DEGREE_DENOMINATOR);

//     /* 很低的非零速度至少保留 1Hz 脉冲，避免除零。 */
//     if (step_frequency == 0U) {
//         step_frequency = 1U;
//     }

//     /* 用定时器时钟频率除以目标脉冲频率得到周期。 */
//     timer_period = timer_clock_frequency / step_frequency;

//     /* 速度过高时，周期不能为 0。 */
//     if (timer_period == 0U) {
//         return 1U;
//     }

//     /* 限制周期不超过当前 PWM 定时器允许的最大值。 */
//     return timer_period > STEP_MOTOR_MAX_PERIOD ? STEP_MOTOR_MAX_PERIOD : timer_period;
// }

// /**
//  * @brief 停止指定电机的 STEP 定时器。
//  * @param motor 待停止的电机配置。
//  */
// static void step_motor_stop_internal(const StepMotorConfig *motor)
// {
//     /* 停止计数器后，PWM 将不再产生新的 STEP 脉冲。 */
//     DL_Timer_stopCounter(motor->timer);
// }

// /**
//  * @brief 清除指定电机的动作并关闭定时器中断。
//  * @param motor 待关闭的电机配置。
//  */
// static void step_motor_close_internal(const StepMotorConfig *motor)
// {
//     /* 先屏蔽中断，避免主循环与中断同时修改剩余脉冲数。 */
//     NVIC_DisableIRQ(motor->interrupt_number);

//     /* 清零后续动作需要输出的脉冲数量。 */
//     *motor->steps_remaining = 0U;

//     /* 立即停止产生 STEP 脉冲的定时器。 */
//     step_motor_stop_internal(motor);

//     /* 清除已挂起的中断，避免下次启动时误进入中断处理函数。 */
//     NVIC_ClearPendingIRQ(motor->interrupt_number);
// }

// /**
//  * @brief 初始化指定电机的运行状态。
//  * @param motor 待初始化的电机配置。
//  */
// static void step_motor_init_internal(const StepMotorConfig *motor)
// {
//     /* 先关闭旧动作并清除中断状态。 */
//     step_motor_close_internal(motor);

//     /* 默认将 DIR 设置为高电平，方向可由外部接口再次修改。 */
//     DL_GPIO_setPins(motor->direction_port, motor->direction_pin);

//     /* 使能中断，为后续按角度停止提供计数服务。 */
//     NVIC_EnableIRQ(motor->interrupt_number);
// }

// /**
//  * @brief 设置指定电机的方向电平。
//  * @param motor 待设置的电机配置。
//  * @param direction 0 输出低电平，非 0 输出高电平。
//  */
// static void step_motor_set_direction_internal(const StepMotorConfig *motor,
//                                               uint8_t direction)
// {
//     /* 方向为 0 时清零 DIR 引脚。 */
//     if (direction == 0U) {
//         DL_GPIO_clearPins(motor->direction_port, motor->direction_pin);
//     } else {
//         /* 方向非 0 时拉高 DIR 引脚。 */
//         DL_GPIO_setPins(motor->direction_port, motor->direction_pin);
//     }
// }

// /**
//  * @brief 启动指定电机的 STEP 脉冲输出。
//  * @param motor 待启动的电机配置。
//  */
// static void step_motor_start_internal(const StepMotorConfig *motor)
// {
//     /* 先清除旧的挂起状态，确保本次动作从新的计数开始。 */
//     NVIC_ClearPendingIRQ(motor->interrupt_number);

//     /* 重新使能定时器中断，以便脉冲数达到目标后自动停止。 */
//     NVIC_EnableIRQ(motor->interrupt_number);

//     /* 启动定时器，PWM 输出开始产生 STEP 脉冲。 */
//     DL_Timer_startCounter(motor->timer);
// }

// /**
//  * @brief 设置指定电机的 STEP 脉冲频率。
//  * @param motor 待设置的电机配置。
//  * @param speed 目标角速度，单位为度每秒。
//  */
// static void step_motor_set_speed_internal(const StepMotorConfig *motor,
//                                           uint32_t speed)
// {
//     uint32_t timer_period;

//     /* 速度为 0 时直接取消当前动作并停止电机。 */
//     if (speed == 0U) {
//         step_motor_close_internal(motor);
//         return;
//     }

//     /* 根据当前电机定时器时钟计算新的 PWM 周期。 */
//     timer_period = step_motor_period_from_speed(speed, motor->timer_clock_frequency);

//     /* 在定时器运行时安全更新装载值，使速度立即生效。 */
//     DL_Timer_setLoadValue(motor->timer, timer_period);

//     /* 将比较值设为周期的一半，输出约 50% 占空比的 STEP 脉冲。 */
//     DL_Timer_setCaptureCompareValue(motor->timer, timer_period / 2U, motor->compare_index);
// }

// /**
//  * @brief 让指定电机转动到目标角度。
//  * @param motor 待控制的电机配置。
//  * @param angle 目标转角，单位为度。
//  */
// static void step_motor_move_angle_internal(const StepMotorConfig *motor,
//                                            uint32_t angle)
// {
//     uint32_t required_steps;

//     /* 先将目标角度换算为需要发送的 STEP 脉冲数。 */
//     required_steps = step_motor_steps_from_angle(angle);

//     /* 关闭旧动作，防止新旧目标的剩余步数相互叠加。 */
//     step_motor_close_internal(motor);

//     /* 0 度动作无需启动定时器。 */
//     if (required_steps == 0U) {
//         return;
//     }

//     /* 在中断保持关闭时写入新的剩余脉冲数。 */
//     *motor->steps_remaining = required_steps;

//     /* 启动定时器并由中断在脉冲计数完成后自动关闭。 */
//     step_motor_start_internal(motor);
// }

// /**
//  * @brief 处理指定电机的定时器零点中断。
//  * @param motor 产生本次中断的电机配置。
//  */
// static void step_motor_handle_interrupt(const StepMotorConfig *motor)
// {
//     DL_TIMER_IIDX pending_interrupt;

//     /* 读取中断源，同时确认本次是否为 PWM 周期结束事件。 */
//     pending_interrupt = DL_Timer_getPendingInterrupt(motor->timer);

//     /* 非零点中断不参与步数统计，直接返回。 */
//     if (pending_interrupt != DL_TIMER_IIDX_ZERO) {
//         return;
//     }

//     /* 每完成一个 PWM 周期，就完成一个 STEP 脉冲。 */
//     if (*motor->steps_remaining > 0U) {
//         (*motor->steps_remaining)--;
//     }

//     /* 脉冲数量归零后立即停止，确保角度不会多走一步。 */
//     if (*motor->steps_remaining == 0U) {
//         step_motor_stop_internal(motor);
//     }
// }

// /**
//  * @brief 初始化 2 号步进电机。
//  */
// void step_motor_init(void)
// {
//     /* 使用统一内部函数初始化 2 号电机。 */
//     step_motor_init_internal(&g_motor2_config);
// }

// /**
//  * @brief 设置 2 号步进电机方向。
//  * @param direction 0 表示低电平，非 0 表示高电平。
//  */
// void step_motor_dir_set(uint8_t direction)
// {
//     /* 将方向请求转交给统一内部函数。 */
//     step_motor_set_direction_internal(&g_motor2_config, direction);
// }

// /**
//  * @brief 启动 2 号步进电机的 STEP 脉冲输出。
//  */
// void step_motor_step_set(void)
// {
//     /* 启动 2 号电机使用的 PWM 定时器。 */
//     step_motor_start_internal(&g_motor2_config);
// }

// /**
//  * @brief 关闭 2 号步进电机的 STEP 脉冲输出。
//  */
// void step_motor_step_close(void)
// {
//     /* 取消 2 号电机动作并停止定时器。 */
//     step_motor_close_internal(&g_motor2_config);
// }

// /**
//  * @brief 设置 2 号步进电机速度。
//  * @param speed 目标角速度，单位为度每秒。
//  */
// void step_motor_speed_set(uint32_t speed)
// {
//     /* 根据 2 号电机的独立时钟配置更新 PWM 周期。 */
//     step_motor_set_speed_internal(&g_motor2_config, speed);
// }

// /**
//  * @brief 驱动 2 号步进电机转动指定角度。
//  * @param angle 目标转角，单位为度。
//  */
// void step_motor_angle(uint32_t angle)
// {
//     /* 为 2 号电机设置新的目标步数并启动输出。 */
//     step_motor_move_angle_internal(&g_motor2_config, angle);
// }

// /**
//  * @brief 处理 2 号步进电机的 TIMG0 中断。
//  */
// void PWM2_INST_IRQHandler(void)
// {
//     /* 按 2 号电机的剩余步数处理本次 PWM 周期中断。 */
//     step_motor_handle_interrupt(&g_motor2_config);
// }

// /**
//  * @brief 初始化 1 号步进电机。
//  */
// void step_motor1_init(void)
// {
//     /* 使用统一内部函数初始化 1 号电机。 */
//     step_motor_init_internal(&g_motor1_config);
// }

// /**
//  * @brief 设置 1 号步进电机方向。
//  * @param direction 0 表示低电平，非 0 表示高电平。
//  */
// void step_motor1_dir_set(uint8_t direction)
// {
//     /* 将方向请求转交给统一内部函数。 */
//     step_motor_set_direction_internal(&g_motor1_config, direction);
// }

// /**
//  * @brief 启动 1 号步进电机的 STEP 脉冲输出。
//  */
// void step_motor1_step_set(void)
// {
//     /* 启动 1 号电机使用的 PWM 定时器。 */
//     step_motor_start_internal(&g_motor1_config);
// }

// /**
//  * @brief 关闭 1 号步进电机的 STEP 脉冲输出。
//  */
// void step_motor1_step_close(void)
// {
//     /* 取消 1 号电机动作并停止定时器。 */
//     step_motor_close_internal(&g_motor1_config);
// }

// /**
//  * @brief 设置 1 号步进电机速度。
//  * @param speed 目标角速度，单位为度每秒。
//  */
// void step_motor1_speed_set(uint32_t speed)
// {
//     /* 根据 1 号电机的独立时钟配置更新 PWM 周期。 */
//     step_motor_set_speed_internal(&g_motor1_config, speed);
// }

// /**
//  * @brief 驱动 1 号步进电机转动指定角度。
//  * @param angle 目标转角，单位为度。
//  */
// void step_motor1_angle(uint32_t angle)
// {
//     /* 为 1 号电机设置新的目标步数并启动输出。 */
//     step_motor_move_angle_internal(&g_motor1_config, angle);
// }

// /**
//  * @brief 处理 1 号步进电机的 TIMG12 中断。
//  */
// void PWM1_INST_IRQHandler(void)
// {
//     /* 按 1 号电机的剩余步数处理本次 PWM 周期中断。 */
//     step_motor_handle_interrupt(&g_motor1_config);
// }
