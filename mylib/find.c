#include "mylib/find.h"

#include "mylib/Motor.h"
#include "ti_msp_dl_config.h"

/* ==================== 实车调节参数 ==================== */

#define LAP_PULSES                  6000L   /* 一圈平均编码器脉冲数。 */
#define START_SPEED                  36.0f   /* task2原起步速度24的1.5倍。 */
#define HIGH_SPEED                   72.0f   /* task2原高速48的1.5倍。 */
#define LOW_SPEED                    48.0f   /* 第二个弯道适当减速。 */
#define TRACK_KP                     5.0f    /* 偏离黑线时的转向力度。 */
#define TRACK_KD                     7.0f    /* 抑制车头快速摆动。 */
#define TRACK_CENTER_OFFSET          2.0f    /* 实测RAW=0x86在车体中间，补偿数字中心偏差。 */
#define FORWARD_SIGN                 1       /* 正速度后退时改为 -1。 */

/* 每段直线的前、后15%保持恒速，中间70%才以固定斜率变速。 */
#define SPEED_RAMP_START_PERCENT     15L
#define SPEED_RAMP_END_PERCENT       85L

/* ==================== 内部保护参数 ==================== */

#define SENSOR_COUNT                 6U
#define LINE_SENSOR_ADDRESS          0x5CU
#define LINE_SENSOR_STATE_REG        0x05U
#define SOFT_I2C_DELAY_CYCLES        320U
#define SEARCH_SPEED                 18
#define CORRECTION_MAX               24.0f
#define CORRECTION_STEP              2.0f
#define MARK_SENSOR_COUNT            3U
#define MARK_CONFIRM_MS              5U
#define END_LINE_ARM_MS              1000U  /* 起步1秒后才允许把宽黑线当成终点。 */
#define LOST_HOLD_MS                 50U
#define LOST_STOP_MS                 400U

/* H1～H8：从车头前方看过去，依次为左到右。 */
static const int8_t g_sensor_weight[SENSOR_COUNT] = {
    -5, -3, -1, 1, 3, 5
};

/* 循迹模块内部共享的数据；task2只调用接口，不直接访问这些变量。 */
typedef struct {
    RingTrackState_t state;
    float raw_position;
    float filtered_position;
    float last_position;
    float base_speed;
    float correction;
    int16_t left_speed;
    int16_t right_speed;
    uint8_t black_count;
    uint8_t sensor_mask;
    int32_t distance;
    uint16_t lost_ms;
    uint16_t mark_ms;
    uint16_t run_ms;
    bool end_line_armed;
} RingTrackControl_t;

static RingTrackControl_t g_track;

static float limit_float(float value, float minimum, float maximum)
{
    if (value > maximum) return maximum;
    if (value < minimum) return minimum;
    return value;
}

static int32_t abs32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static void i2c_delay(void)
{
    DL_Common_delayCycles(SOFT_I2C_DELAY_CYCLES);
}

static void scl_low(void)
{
    DL_GPIO_clearPins(LINE_I2C_PORT, LINE_I2C_LS_SCL_PIN);
    DL_GPIO_enableOutput(LINE_I2C_PORT, LINE_I2C_LS_SCL_PIN);
}

static void scl_high(void)
{
    DL_GPIO_disableOutput(LINE_I2C_PORT, LINE_I2C_LS_SCL_PIN);
}

static void sda_low(void)
{
    DL_GPIO_clearPins(LINE_I2C_PORT, LINE_I2C_LS_SDA_PIN);
    DL_GPIO_enableOutput(LINE_I2C_PORT, LINE_I2C_LS_SDA_PIN);
}

static void sda_high(void)
{
    DL_GPIO_disableOutput(LINE_I2C_PORT, LINE_I2C_LS_SDA_PIN);
}

static void i2c_start(void)
{
    sda_high(); scl_high(); i2c_delay();
    sda_low(); i2c_delay(); scl_low();
}

static void i2c_stop(void)
{
    sda_low(); i2c_delay(); scl_high(); i2c_delay();
    sda_high(); i2c_delay();
}

static bool i2c_write(uint8_t data)
{
    uint8_t i;
    bool ack;

    for (i = 0U; i < 8U; ++i) {
        if ((data & 0x80U) != 0U) sda_high(); else sda_low();
        i2c_delay(); scl_high(); i2c_delay(); scl_low();
        data <<= 1U;
    }
    sda_high(); i2c_delay(); scl_high(); i2c_delay();
    ack = (DL_GPIO_readPins(LINE_I2C_PORT, LINE_I2C_LS_SDA_PIN) == 0U);
    scl_low();
    return ack;
}

static uint8_t i2c_read(void)
{
    uint8_t i;
    uint8_t data = 0U;

    sda_high();
    for (i = 0U; i < 8U; ++i) {
        data <<= 1U; scl_high(); i2c_delay();
        if (DL_GPIO_readPins(LINE_I2C_PORT, LINE_I2C_LS_SDA_PIN) != 0U) {
            data |= 1U;
        }
        scl_low(); i2c_delay();
    }
    sda_high(); scl_high(); i2c_delay(); scl_low();
    return data;
}

static bool line_sensor_read_state(uint8_t *state)
{
    if (state == 0) return false;

    i2c_start();
    if (!i2c_write((uint8_t)(LINE_SENSOR_ADDRESS << 1U)) ||
        !i2c_write(LINE_SENSOR_STATE_REG)) {
        i2c_stop();
        return false;
    }
    i2c_stop();

    i2c_start();
    if (!i2c_write((uint8_t)((LINE_SENSOR_ADDRESS << 1U) | 1U))) {
        i2c_stop();
        return false;
    }
    *state = i2c_read();
    i2c_stop();
    return true;
}

/* 在一段路程的指定区间内，按固定斜率从from_speed变到to_speed。 */
static float get_ramp_speed(int32_t distance, int32_t segment_start,
                            int32_t segment_end, float from_speed,
                            float to_speed)
{
    int32_t segment_length = segment_end - segment_start;
    int32_t ramp_start = segment_start +
        (segment_length * SPEED_RAMP_START_PERCENT) / 100L;
    int32_t ramp_end = segment_start +
        (segment_length * SPEED_RAMP_END_PERCENT) / 100L;
    float progress;

    if (distance <= ramp_start) return from_speed;
    if (distance >= ramp_end) return to_speed;

    progress = (float)(distance - ramp_start) /
               (float)(ramp_end - ramp_start);
    return from_speed + (to_speed - from_speed) * progress;
}

void RingTrack_Init(void)
{
    g_track.state = RING_TRACK_IDLE;
    g_track.end_line_armed = false;
}

void RingTrack_Start(void)
{
    Motor_ResetLeftEncoder();
    Motor_ResetRightEncoder();

    g_track.state = RING_TRACK_AB;
    g_track.raw_position = 0.0f;
    g_track.filtered_position = 0.0f;
    g_track.last_position = 0.0f;
    g_track.base_speed = START_SPEED;
    g_track.correction = 0.0f;
    g_track.left_speed = 0;
    g_track.right_speed = 0;
    g_track.black_count = 0U;
    g_track.sensor_mask = 0U;
    g_track.distance = 0L;
    g_track.lost_ms = 0U;
    g_track.mark_ms = 0U;
    g_track.run_ms = 0U;
    g_track.end_line_armed = false;
    Motor_Enable();
}

void RingTrack_ReadSensors(void)
{
    uint8_t sensor_state = 0U;
    int16_t weighted_sum = 0;
    int32_t left_distance;
    int32_t right_distance;
    uint8_t i;

    (void)line_sensor_read_state(&sensor_state);
    g_track.black_count = 0U;
    g_track.sensor_mask = (uint8_t)(sensor_state & 0x3FU);
    for (i = 0U; i < SENSOR_COUNT; ++i) {
        if (((sensor_state >> i) & 1U) != 0U) {
            weighted_sum += g_sensor_weight[i];
            ++g_track.black_count;
        }
    }

    if (g_track.black_count != 0U) {
        /*
         * 实测RAW=0x86时，第2、3路（bit1、bit2）正好位于黑线中央。
         * 先按探头组合给出明确动作，再用加权位置处理其他过渡状态。
         */
        if ((g_track.sensor_mask & 0x06U) == 0x06U) {
            g_track.raw_position = 0.0f;       /* 中间两路都有线：保持直行。 */
        } else if ((g_track.sensor_mask & 0x03U) == 0x03U) {
            g_track.raw_position = -3.0f;      /* 左侧翼两路都有线：进入左弯。 */
        } else if ((g_track.sensor_mask & 0x30U) == 0x30U) {
            g_track.raw_position = 4.0f;       /* 右侧翼两路都有线：进入右弯。 */
        } else if ((g_track.sensor_mask & 0x02U) != 0U) {
            g_track.raw_position = -0.7f;      /* 中间右路丢线：轻微向左调整。 */
        } else if ((g_track.sensor_mask & 0x04U) != 0U) {
            g_track.raw_position = 0.7f;       /* 中间左路丢线：轻微向右调整。 */
        } else if ((g_track.sensor_mask & 0x01U) != 0U) {
            g_track.raw_position = -5.0f;      /* 左翼只剩外侧：立即加强左修正。 */
        } else if ((g_track.sensor_mask & 0x10U) != 0U) {
            g_track.raw_position = 3.0f;       /* 右翼只剩内侧：向右微调。 */
        } else if ((g_track.sensor_mask & 0x20U) != 0U) {
            g_track.raw_position = 5.0f;       /* 右翼只剩外侧：立即加强右修正。 */
        } else {
            g_track.raw_position =
                (float)weighted_sum / (float)g_track.black_count +
                TRACK_CENTER_OFFSET;
        }

        /* 重新找到黑线后清零丢线计时，短暂漏检不会继续累积。 */
        g_track.lost_ms = 0U;
    }

    left_distance = abs32(Motor_GetLeftEncoderPosition());
    right_distance = abs32(Motor_GetRightEncoderPosition());
    g_track.distance = (left_distance + right_distance) / 2L;
}

void RingTrack_UpdateSpeedProfile(void)
{
    const int32_t point_b = (LAP_PULSES * 2442L) / 10000L;
    const int32_t point_c = LAP_PULSES / 2L;
    const int32_t point_d = (LAP_PULSES * 7442L) / 10000L;

    if (g_track.distance < point_b) {
        /* AB：前15%保持23.04，中间70%匀速加至46.08。 */
        g_track.state = RING_TRACK_AB;
        g_track.base_speed = get_ramp_speed(
            g_track.distance, 0L, point_b, START_SPEED, HIGH_SPEED);
    } else if (g_track.distance < point_c) {
        /* 第一个弯道BC保持46.08。 */
        g_track.state = RING_TRACK_BC;
        g_track.base_speed = HIGH_SPEED;
    } else if (g_track.distance < point_d) {
        /* CD从46.08平滑减速到30.72，为第二个弯道提前减速。 */
        g_track.state = RING_TRACK_CD;
        g_track.base_speed = get_ramp_speed(
            g_track.distance, point_c, point_d, HIGH_SPEED, LOW_SPEED);
    } else {
        /* 第二个弯道DA保持30.72，稳定驶回A点。 */
        g_track.state = RING_TRACK_DA;
        g_track.base_speed = LOW_SPEED;
    }
}

void RingTrack_SetBaseSpeed(float speed)
{
    g_track.base_speed = limit_float(
        speed, (float)MOTOR_SPEED_MIN, (float)MOTOR_SPEED_MAX);
}

static void calculate_steering(float kp, float kd,
                               float new_data_weight,
                               float correction_step,
                               float center_deadband)
{
    float control_position;
    float target;
    float change;
    float steering_limit;

    /*
     * 直线和弯道使用不同力度，但都提高新读数权重，保证掉一路时立刻响应。
     * 直线只做轻微修正；侧翼探头接管后提高增益和修正速度。
     */
    if (g_track.raw_position > -1.5f &&
        g_track.raw_position < 1.5f) {
        kp *= 0.65f;
        kd *= 0.50f;
        new_data_weight = 0.75f;
        correction_step = 2.0f;
    } else {
        kp *= 1.20f;
        new_data_weight = 0.65f;
        correction_step = 2.5f;
    }

    /* 按指定权重融合新旧位置，权重越小，数字探头跳变造成的抖动越弱。 */
    g_track.filtered_position =
        (1.0f - new_data_weight) * g_track.filtered_position +
        new_data_weight * g_track.raw_position;

    control_position = g_track.filtered_position;
    if (control_position > -center_deadband &&
        control_position < center_deadband) {
        control_position = 0.0f;
    }

    target = kp * control_position +
             kd * (control_position - g_track.last_position);
    g_track.last_position = control_position;

    /*
     * 减速接近0时，转向修正量也必须一起减小；否则基础速度已经很低，
     * 左右轮仍可能因为较大的修正量而一正一反，导致停车前原地转动。
     */
    steering_limit = g_track.base_speed;
    if (steering_limit < 0.0f) steering_limit = -steering_limit;
    if (steering_limit > CORRECTION_MAX) steering_limit = CORRECTION_MAX;

    target = limit_float(target, -steering_limit, steering_limit);
    change = limit_float(target - g_track.correction,
                         -correction_step, correction_step);
    g_track.correction += change;
    g_track.correction = limit_float(
        g_track.correction, -steering_limit, steering_limit);

    /*
     * 以任务给出的base_speed为两轮共同基础速度，再叠加差速修正。
     * 居中时correction为0，两轮都等于基础速度；偏离时才改变轮速差。
     */
    /*
     * 左右轮名称采用“站在车头正前方面对小车”时看到的左右方向。
     * 因此差速方向与车辆驾驶视角相反：位置为负时左轮加速、右轮减速。
     */
    g_track.left_speed = (int16_t)(g_track.base_speed - g_track.correction);
    g_track.right_speed = (int16_t)(g_track.base_speed + g_track.correction);
    if (g_track.left_speed > MOTOR_SPEED_MAX) g_track.left_speed = MOTOR_SPEED_MAX;
    if (g_track.left_speed < MOTOR_SPEED_MIN) g_track.left_speed = MOTOR_SPEED_MIN;
    if (g_track.right_speed > MOTOR_SPEED_MAX) g_track.right_speed = MOTOR_SPEED_MAX;
    if (g_track.right_speed < MOTOR_SPEED_MIN) g_track.right_speed = MOTOR_SPEED_MIN;
}

void RingTrack_CalculateSteering(void)
{
    /* 所有任务共用降低后的微调参数，减小行驶中的左右抖动。 */
    calculate_steering(TRACK_KP, TRACK_KD, 0.35f,
                       CORRECTION_STEP, 0.0f);
}

void RingTrack_OutputMotor(void)
{
    Motor_SetSpeed(FORWARD_SIGN * g_track.left_speed,
                   FORWARD_SIGN * g_track.right_speed);
}

bool RingTrack_IsLineLost(void)
{
    return g_track.black_count == 0U;
}

void RingTrack_HandleLineLost(uint16_t period_ms)
{
    if (g_track.lost_ms < LOST_STOP_MS) g_track.lost_ms += period_ms;

    if (g_track.lost_ms <= LOST_HOLD_MS) {
        RingTrack_OutputMotor();
    } else if (g_track.lost_ms < LOST_STOP_MS) {
        /* 丢线后朝最后看到黑线的方向搜索，轮子左右沿用车头正前方视角。 */
        if (g_track.filtered_position < 0.0f) {
            Motor_SetSpeed(FORWARD_SIGN * SEARCH_SPEED,
                           FORWARD_SIGN * (-SEARCH_SPEED));
        } else {
            Motor_SetSpeed(FORWARD_SIGN * (-SEARCH_SPEED),
                           FORWARD_SIGN * SEARCH_SPEED);
        }
    } else {
        Motor_Brake();
        g_track.state = RING_TRACK_FAULT;
    }
}

bool RingTrack_MarkerDetected(uint8_t minimum_black_count,
                              uint16_t period_ms)
{
    /* 起步阶段可能还压着起点线，先等待1秒再启用终点线检测。 */
    if (!g_track.end_line_armed) {
        if (g_track.run_ms < END_LINE_ARM_MS) {
            g_track.run_ms += period_ms;
        }
        if (g_track.run_ms < END_LINE_ARM_MS) {
            return false;
        }
        g_track.end_line_armed = true;
    }

    /* 终点横线会同时覆盖大部分探头，连续25 ms确认后再停车。 */
    if (g_track.black_count < minimum_black_count) {
        g_track.mark_ms = 0U;
        return false;
    }

    if (g_track.mark_ms < MARK_CONFIRM_MS) g_track.mark_ms += period_ms;
    if (g_track.mark_ms < MARK_CONFIRM_MS) return false;

    return true;
}

bool RingTrack_EndLineDetected(uint16_t period_ms)
{
    return RingTrack_MarkerDetected(MARK_SENSOR_COUNT, period_ms);
}

bool RingTrack_CheckFinish(uint16_t period_ms)
{
    /* 至少三路同时压到启停线，当前5 ms控制周期内立即抱刹。 */
    if (!RingTrack_EndLineDetected(period_ms)) {
        return false;
    }

    Motor_Brake();
    g_track.state = RING_TRACK_FINISHED;
    return true;
}

void RingTrack_Stop(void)
{
    Motor_Brake();
    g_track.state = RING_TRACK_IDLE;
}

RingTrackState_t RingTrack_GetState(void)
{
    return g_track.state;
}
