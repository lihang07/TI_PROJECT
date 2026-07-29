#include "mylib/zdt_x42s_pulse.h"

#define ZDT_Q16_ONE 65536L

static int32_t ZDT_Clamp(const ZDT_X42S_Pulse *motor, int32_t pulse)
{
    if (pulse < motor->minimum_pulse) return motor->minimum_pulse;
    if (pulse > motor->maximum_pulse) return motor->maximum_pulse;
    return pulse;
}

static uint32_t ZDT_AbsI32(int32_t value)
{
    return (value < 0) ? (uint32_t)(-value) : (uint32_t)value;
}

static void ZDT_SetEnable(bool enable)
{
    bool high = enable ? (ZDT_X42S_EN_ACTIVE_HIGH != 0) : (ZDT_X42S_EN_ACTIVE_HIGH == 0);
    if (high) DL_GPIO_setPins(ZDT_X42S_EN_PORT, ZDT_X42S_EN_PIN);
    else DL_GPIO_clearPins(ZDT_X42S_EN_PORT, ZDT_X42S_EN_PIN);
}

static void ZDT_SetDirection(bool positive)
{
    bool high = positive ? (ZDT_X42S_DIR_POSITIVE_HIGH != 0) :
                           (ZDT_X42S_DIR_POSITIVE_HIGH == 0);
    if (high) DL_GPIO_setPins(ZDT_X42S_DIR_PORT, ZDT_X42S_DIR_PIN);
    else DL_GPIO_clearPins(ZDT_X42S_DIR_PORT, ZDT_X42S_DIR_PIN);
}

static void ZDT_SetStepRate(uint32_t hz)
{
    uint32_t period;

    if (hz == 0U) {
        DL_TimerG_stopCounter(ZDT_X42S_STEP_INST);
        DL_TimerG_setCaptureCompareValue(ZDT_X42S_STEP_INST, 0U,
                                         ZDT_X42S_STEP_CC_INDEX);
        return;
    }

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
    if ((motor == 0) || (minimum_pulse > maximum_pulse) ||
        (maximum_rate_hz == 0U) || (acceleration_pulse_s2 == 0U)) return;

    motor->minimum_pulse = minimum_pulse;
    motor->maximum_pulse = maximum_pulse;
    motor->maximum_rate_hz = maximum_rate_hz;
    motor->acceleration_pulse_s2 = acceleration_pulse_s2;
    motor->command_pulse = ZDT_Clamp(motor, zero_pulse);
    motor->target_pulse = motor->command_pulse;
    motor->fractional_pulse_q16 = 0;
    motor->signed_rate_hz = 0;
    motor->enabled = false;
    ZDT_SetStepRate(0U);
    ZDT_SetEnable(false);
}

void ZDT_X42S_Pulse_Enable(ZDT_X42S_Pulse *motor, bool enable)
{
    if (motor == 0) return;
    ZDT_X42S_Pulse_Hold(motor);
    ZDT_SetEnable(enable);
    motor->enabled = enable;
}

void ZDT_X42S_Pulse_SetTarget(ZDT_X42S_Pulse *motor, int32_t target_pulse)
{
    if (motor != 0) motor->target_pulse = ZDT_Clamp(motor, target_pulse);
}

void ZDT_X42S_Pulse_Hold(ZDT_X42S_Pulse *motor)
{
    if (motor == 0) return;
    ZDT_SetStepRate(0U);
    motor->signed_rate_hz = 0;
    motor->fractional_pulse_q16 = 0;
}

void ZDT_X42S_Pulse_SetCurrentPosition(ZDT_X42S_Pulse *motor, int32_t pulse)
{
    if (motor == 0) return;
    ZDT_X42S_Pulse_Hold(motor);
    motor->command_pulse = ZDT_Clamp(motor, pulse);
    motor->target_pulse = motor->command_pulse;
}

void ZDT_X42S_Pulse_Update1ms(ZDT_X42S_Pulse *motor)
{
    int32_t distance, next_rate;
    uint32_t distance_abs, rate_abs, braking_distance, delta_rate;
    bool positive;
    if ((motor == 0) || !motor->enabled) return;

    distance = motor->target_pulse - motor->command_pulse;
    distance_abs = ZDT_AbsI32(distance);
    rate_abs = ZDT_AbsI32(motor->signed_rate_hz);
    delta_rate = motor->acceleration_pulse_s2 / 1000U;
    if (delta_rate == 0U) delta_rate = 1U;

    if ((distance == 0) || ((motor->signed_rate_hz > 0) && (distance < 0)) ||
        ((motor->signed_rate_hz < 0) && (distance > 0))) {
        if (rate_abs <= delta_rate) { ZDT_X42S_Pulse_Hold(motor); return; }
        rate_abs -= delta_rate;
        next_rate = (motor->signed_rate_hz > 0) ? (int32_t)rate_abs : -(int32_t)rate_abs;
    } else {
        positive = (distance > 0);
        braking_distance = (uint32_t)(((uint64_t)rate_abs * rate_abs) /
                          ((uint64_t)2U * motor->acceleration_pulse_s2));
        if (distance_abs <= braking_distance) rate_abs = (rate_abs > delta_rate) ? rate_abs - delta_rate : 0U;
        else if (rate_abs < motor->maximum_rate_hz) {
            rate_abs += delta_rate;
            if (rate_abs > motor->maximum_rate_hz) rate_abs = motor->maximum_rate_hz;
        }
        next_rate = positive ? (int32_t)rate_abs : -(int32_t)rate_abs;
    }
    if (next_rate == 0) { ZDT_X42S_Pulse_Hold(motor); return; }

    positive = (next_rate > 0);
    if ((motor->signed_rate_hz == 0) || ((motor->signed_rate_hz > 0) != positive)) {
        ZDT_SetStepRate(0U);
        ZDT_SetDirection(positive);
    }
    motor->signed_rate_hz = next_rate;
    ZDT_SetStepRate(ZDT_AbsI32(next_rate));
    motor->fractional_pulse_q16 += (int32_t)(((int64_t)next_rate * ZDT_Q16_ONE) / 1000L);
    while (motor->fractional_pulse_q16 >= ZDT_Q16_ONE) {
        ++motor->command_pulse; motor->fractional_pulse_q16 -= ZDT_Q16_ONE;
    }
    while (motor->fractional_pulse_q16 <= -ZDT_Q16_ONE) {
        --motor->command_pulse; motor->fractional_pulse_q16 += ZDT_Q16_ONE;
    }
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
