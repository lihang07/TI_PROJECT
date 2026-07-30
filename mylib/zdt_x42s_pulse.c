#include "mylib/zdt_x42s_pulse.h"

/*
 * Q16 定点数的“1.0”。取 65536 (= 2^16) 的原因：二进制移位方便且能把
 * 一个脉冲细分为 65536 份。该精度远高于本项目的 1 ms 控制需要，避免低速
 * 运行时 rate_hz / 1000 被整除为 0，从而出现“已输出脉冲但软件位置不变”。
 */
#define ZDT_Q16_ONE 65536L

/* 将任意目标截断到软件行程范围内，是防止控制算法异常扩大摆幅的最后保护。 */
static int32_t ZDT_Clamp(const ZDT_X42S_Pulse *motor, int32_t pulse)
{
    if (pulse < motor->minimum_pulse) return motor->minimum_pulse;
    if (pulse > motor->maximum_pulse) return motor->maximum_pulse;
    return pulse;
}

/* 返回有符号脉冲数/速度的绝对值，统一用于频率和制动距离计算。 */
static uint32_t ZDT_AbsI32(int32_t value)
{
    return (value < 0) ? (uint32_t)(-value) : (uint32_t)value;
}

/*
 * 根据 ZDT_X42S_EN_ACTIVE_HIGH 输出 EN。
 * 先由上层传入“逻辑使能”，再翻译为实际电平，避免应用层依赖 X42S 菜单中
 * 的高/低有效配置。EN 误设时的典型现象是：PB14 有脉冲但电机没有动作。
 */
static void ZDT_SetEnable(bool enable)
{
    bool high = enable ? (ZDT_X42S_EN_ACTIVE_HIGH != 0) :
                         (ZDT_X42S_EN_ACTIVE_HIGH == 0);

    if (high) DL_GPIO_setPins(ZDT_X42S_EN_PORT, ZDT_X42S_EN_PIN);
    else DL_GPIO_clearPins(ZDT_X42S_EN_PORT, ZDT_X42S_EN_PIN);
}

/*
 * 根据 ZDT_X42S_DIR_POSITIVE_HIGH 输出 DIR。
 * positive 只表示软件脉冲坐标增大，未必等于“摆杆向上”；实际方向错误时只改
 * 该宏，避免把控制器中的正负号、限幅和目标位置全部反向。
 */
static void ZDT_SetDirection(bool positive)
{
    bool high = positive ? (ZDT_X42S_DIR_POSITIVE_HIGH != 0) :
                           (ZDT_X42S_DIR_POSITIVE_HIGH == 0);

    if (high) DL_GPIO_setPins(ZDT_X42S_DIR_PORT, ZDT_X42S_DIR_PIN);
    else DL_GPIO_clearPins(ZDT_X42S_DIR_PORT, ZDT_X42S_DIR_PIN);
}

/*
 * 设定 STP 脉冲频率。
 *
 * hz=0 是明确的停车命令：停止计数器且把比较值清零，确保 PB14 不再翻转。
 * hz>0 时，period = 定时器时钟 / hz。例如 32 MHz、400 Hz 得到 80000，
 * 即 PB14 每 80000 个定时器时钟周期输出一个脉冲。比较值取 period/2，形成
 * 50% 占空比；选择 50% 的原因是对 X42S 的脉冲宽度有最大余量，同时不会偏向
 * 高/低电平。该函数每毫秒可能更新一次，故先停表再更新寄存器，避免更改周期时
 * 产生一个异常窄脉冲。
 */
static void ZDT_SetStepRate(uint32_t hz)
{
    uint32_t period;

    if (hz == 0U) {
        DL_TimerG_stopCounter(ZDT_X42S_STEP_INST);
        DL_TimerG_setCaptureCompareValue(ZDT_X42S_STEP_INST, 0U,
                                         ZDT_X42S_STEP_CC_INDEX);
        return;
    }

    /*
     * period 至少需为 2，才能同时拥有有效的高、低电平。因此频率上限为
     * timer_clock/2。这个保护是定时器的数学极限，不是 X42S 的推荐运行频率；
     * 实际最大速度仍由 Init 的 maximum_rate_hz 决定。
     */
    if (hz > (ZDT_X42S_STEP_CLK_HZ / 2U)) hz = ZDT_X42S_STEP_CLK_HZ / 2U;
    period = ZDT_X42S_STEP_CLK_HZ / hz;
    if (period < 2U) period = 2U;

    DL_TimerG_stopCounter(ZDT_X42S_STEP_INST);
    DL_TimerG_setLoadValue(ZDT_X42S_STEP_INST, period - 1U);
    DL_TimerG_setCaptureCompareValue(ZDT_X42S_STEP_INST, period / 2U,
                                     ZDT_X42S_STEP_CC_INDEX);
    DL_TimerG_startCounter(ZDT_X42S_STEP_INST);
}

void ZDT_X42S_Pulse_Init(ZDT_X42S_Pulse *motor,
                          int32_t minimum_pulse, int32_t maximum_pulse,
                          uint32_t maximum_rate_hz, uint32_t acceleration_pulse_s2,
                          int32_t zero_pulse)
{
    /* 参数非法时不触碰硬件，避免错误限幅或 0 加速度导致后续除零。 */
    if ((motor == 0) || (minimum_pulse > maximum_pulse) ||
        (maximum_rate_hz == 0U) || (acceleration_pulse_s2 == 0U)) return;

    motor->minimum_pulse = minimum_pulse;
    motor->maximum_pulse = maximum_pulse;
    motor->maximum_rate_hz = maximum_rate_hz;
    motor->acceleration_pulse_s2 = acceleration_pulse_s2;

    /* zero_pulse 也限幅，保证上电时软件坐标不会一开始就在允许行程之外。 */
    motor->command_pulse = ZDT_Clamp(motor, zero_pulse);
    motor->target_pulse = motor->command_pulse;
    motor->fractional_pulse_q16 = 0;
    motor->signed_rate_hz = 0;
    motor->enabled = false;

    /* 默认“无脉冲 + 不使能”：上电初始化不能意外带动摆杆。 */
    ZDT_SetStepRate(0U);
    ZDT_SetEnable(false);
}

void ZDT_X42S_Pulse_Enable(ZDT_X42S_Pulse *motor, bool enable)
{
    if (motor == 0) return;

    /* 切换 EN 前先停脉冲，防止驱动器在半个脉冲中被使能/禁用。 */
    ZDT_X42S_Pulse_Hold(motor);
    ZDT_SetEnable(enable);
    motor->enabled = enable;
}

void ZDT_X42S_Pulse_SetTarget(ZDT_X42S_Pulse *motor, int32_t target_pulse)
{
    if (motor != 0) {
        /* 无论上层给出的值多大，目标都不会越过初始化时定义的软件限幅。 */
        motor->target_pulse = ZDT_Clamp(motor, target_pulse);
    }
}

void ZDT_X42S_Pulse_Hold(ZDT_X42S_Pulse *motor)
{
    if (motor == 0) return;

    /* Hold 只停止 STP，不改变 EN：电机仍可保持力矩，适合摆杆停住时使用。 */
    ZDT_SetStepRate(0U);
    motor->signed_rate_hz = 0;
    motor->fractional_pulse_q16 = 0;
}

void ZDT_X42S_Pulse_SetCurrentPosition(ZDT_X42S_Pulse *motor, int32_t pulse)
{
    if (motor == 0) return;

    /* 重设坐标前先停住，避免软件坐标跳变而电机仍按旧目标运行。 */
    ZDT_X42S_Pulse_Hold(motor);
    motor->command_pulse = ZDT_Clamp(motor, pulse);
    motor->target_pulse = motor->command_pulse;
}

void ZDT_X42S_Pulse_Update1ms(ZDT_X42S_Pulse *motor)
{
    int32_t distance, next_rate;
    uint32_t distance_abs, rate_abs, braking_distance, delta_rate;
    bool positive;

    /* 未使能时绝不改写 PWM，防止上层漏调 Enable 仍然带动机构。 */
    if ((motor == 0) || !motor->enabled) return;

    distance = motor->target_pulse - motor->command_pulse;
    distance_abs = ZDT_AbsI32(distance);
    rate_abs = ZDT_AbsI32(motor->signed_rate_hz);

    /*
     * 控制周期是 1 ms，因此“脉冲/s²”要除以 1000 才是每次调用的频率变化量。
     * 最小强制为 1 Hz/ms：低于 1000 的加速度仍可运行，只是实际离散加速度会
     * 被提升到 1000 脉冲/s²；需要更慢时，应将 Update 周期改为更细而非使用 0。
     */
    delta_rate = motor->acceleration_pulse_s2 / 1000U;
    if (delta_rate == 0U) delta_rate = 1U;

    /*
     * 到达目标或已经向相反方向越过目标时，先减速至 0，再允许下一次换向。
     * 这样不会直接翻转 DIR，降低摆杆和传动机构的反向冲击。
     */
    if ((distance == 0) || ((motor->signed_rate_hz > 0) && (distance < 0)) ||
        ((motor->signed_rate_hz < 0) && (distance > 0))) {
        if (rate_abs <= delta_rate) {
            ZDT_X42S_Pulse_Hold(motor);
            return;
        }
        rate_abs -= delta_rate;
        next_rate = (motor->signed_rate_hz > 0) ?
                        (int32_t)rate_abs : -(int32_t)rate_abs;
    } else {
        positive = (distance > 0);

        /*
         * d = v²/(2a)：按当前速度完全刹停所需的脉冲距离。距离小于等于 d 时
         * 立即减速，否则会超过目标。使用 uint64_t 防止高频率平方时 32 位溢出。
         */
        braking_distance = (uint32_t)(((uint64_t)rate_abs * rate_abs) /
                          ((uint64_t)2U * motor->acceleration_pulse_s2));

        if (distance_abs <= braking_distance) {
            rate_abs = (rate_abs > delta_rate) ? rate_abs - delta_rate : 0U;
        } else if (rate_abs < motor->maximum_rate_hz) {
            rate_abs += delta_rate;
            if (rate_abs > motor->maximum_rate_hz) {
                rate_abs = motor->maximum_rate_hz;
            }
        }
        next_rate = positive ? (int32_t)rate_abs : -(int32_t)rate_abs;
    }

    if (next_rate == 0) {
        ZDT_X42S_Pulse_Hold(motor);
        return;
    }

    positive = (next_rate > 0);
    if ((motor->signed_rate_hz == 0) ||
        ((motor->signed_rate_hz > 0) != positive)) {
        /* 方向只在 STP 已停止时更新，保证 DIR 对脉冲满足建立时间。 */
        ZDT_SetStepRate(0U);
        ZDT_SetDirection(positive);
    }

    motor->signed_rate_hz = next_rate;
    ZDT_SetStepRate(ZDT_AbsI32(next_rate));

    /*
     * 每 1 ms 的理论位移为 next_rate/1000 脉冲。Q16 累积后每跨过 1 个
     * 完整脉冲才更新 command_pulse，使软件位置与已发送的脉冲数一致。
     */
    motor->fractional_pulse_q16 +=
        (int32_t)(((int64_t)next_rate * ZDT_Q16_ONE) / 1000L);

    while (motor->fractional_pulse_q16 >= ZDT_Q16_ONE) {
        ++motor->command_pulse;
        motor->fractional_pulse_q16 -= ZDT_Q16_ONE;
    }
    while (motor->fractional_pulse_q16 <= -ZDT_Q16_ONE) {
        --motor->command_pulse;
        motor->fractional_pulse_q16 += ZDT_Q16_ONE;
    }

    /* 防御性二次限幅：即使积分累积到边界外，也不会记录越界软件位置。 */
    motor->command_pulse = ZDT_Clamp(motor, motor->command_pulse);
}

int32_t ZDT_X42S_Pulse_GetTarget(const ZDT_X42S_Pulse *motor)
{
    return (motor == 0) ? 0 : motor->target_pulse;
}

int32_t ZDT_X42S_Pulse_GetCommand(const ZDT_X42S_Pulse *motor)
{
    return (motor == 0) ? 0 : motor->command_pulse;
}
