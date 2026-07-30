#include "mylib/find.h"

#include "mylib/Motor.h"
#include "ti_msp_dl_config.h"

/* ==================== 实车调节参数 ==================== */

#define LAP_PULSES                  6000L   /* 一圈平均编码器脉冲数，实车测量后修改。 */
#define START_SPEED                  24.0f   /* A点起步速度。 */
#define HIGH_SPEED                   48.0f   /* AB加速后的最高速度。 */
#define LOW_SPEED                    36.0f   /* CD减速后的最低速度。 */
#define TRACK_KP                     5.0f    /* 偏离黑线时的转向力度。 */
#define TRACK_KD                     7.0f    /* 抑制车头快速摆动。 */
#define FORWARD_SIGN                 1       /* 正速度后退时改为 -1。 */

/* 每段直线的前、后15%保持恒速，中间70%才以固定斜率变速。 */
#define SPEED_RAMP_START_PERCENT     15L
#define SPEED_RAMP_END_PERCENT       85L

/* ==================== 内部保护参数 ==================== */

#define SENSOR_COUNT                 8U
#define SEARCH_SPEED                 18
#define CORRECTION_MAX               24.0f
#define CORRECTION_STEP              2.0f
#define MARK_SENSOR_COUNT            6U
#define MARK_CONFIRM_MS              25U
#define LOST_HOLD_MS                 50U
#define LOST_STOP_MS                 400U

/* H1～H8：从车头前方看过去，依次为左到右。 */
static const int8_t g_sensor_weight[SENSOR_COUNT] = {
    -7, -5, -3, -1, 1, 3, 5, 7
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
    int32_t distance;
    uint16_t lost_ms;
    uint16_t mark_ms;
    bool left_start_mark;
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

static uint8_t sensor_is_black(uint32_t port_value, uint32_t pin_mask)
{
    uint8_t level = ((port_value & pin_mask) != 0U) ? 1U : 0U;
    return (level == LINE_SENSOR_BLACK_LEVEL) ? 1U : 0U;
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
    g_track.left_start_mark = false;
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
    g_track.distance = 0L;
    g_track.lost_ms = 0U;
    g_track.mark_ms = 0U;
    g_track.left_start_mark = false;
    Motor_Enable();
}

void RingTrack_ReadSensors(void)
{
    const uint8_t black[SENSOR_COUNT] = {
        sensor_is_black(DL_GPIO_readPins(XG_G1_PORT, XG_G1_PIN), XG_G1_PIN),
        sensor_is_black(DL_GPIO_readPins(XG_G2_PORT, XG_G2_PIN), XG_G2_PIN),
        sensor_is_black(DL_GPIO_readPins(XG_G3_PORT, XG_G3_PIN), XG_G3_PIN),
        sensor_is_black(DL_GPIO_readPins(XG_G4_PORT, XG_G4_PIN), XG_G4_PIN),
        sensor_is_black(DL_GPIO_readPins(XG_G5_PORT, XG_G5_PIN), XG_G5_PIN),
        sensor_is_black(DL_GPIO_readPins(XG_G6_PORT, XG_G6_PIN), XG_G6_PIN),
        sensor_is_black(DL_GPIO_readPins(XG_G7_PORT, XG_G7_PIN), XG_G7_PIN),
        sensor_is_black(DL_GPIO_readPins(XG_G8_PORT, XG_G8_PIN), XG_G8_PIN)
    };
    int16_t weighted_sum = 0;
    int32_t left_distance;
    int32_t right_distance;
    uint8_t i;

    g_track.black_count = 0U;
    for (i = 0U; i < SENSOR_COUNT; ++i) {
        if (black[i] != 0U) {
            weighted_sum += g_sensor_weight[i];
            ++g_track.black_count;
        }
    }

    if (g_track.black_count != 0U) {
        g_track.raw_position =
            (float)weighted_sum / (float)g_track.black_count;
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
        /* AB：前15%保持24，中间70%匀速加至48，后15%保持48。 */
        g_track.state = RING_TRACK_AB;
        g_track.base_speed = get_ramp_speed(
            g_track.distance, 0L, point_b, START_SPEED, HIGH_SPEED);
    } else if (g_track.distance < point_c) {
        /* BC弧线全程保持48。 */
        g_track.state = RING_TRACK_BC;
        g_track.base_speed = HIGH_SPEED;
    } else if (g_track.distance < point_d) {
        /* CD：前15%保持48，中间70%匀速降至36，后15%保持36。 */
        g_track.state = RING_TRACK_CD;
        g_track.base_speed = get_ramp_speed(
            g_track.distance, point_c, point_d, HIGH_SPEED, LOW_SPEED);
    } else {
        /* DA弧线全程保持36，稳定驶回A点。 */
        g_track.state = RING_TRACK_DA;
        g_track.base_speed = LOW_SPEED;
    }
}

void RingTrack_CalculateSteering(void)
{
    float target;
    float change;

    /* 新读数只占35%，避免数字量传感器跳变使小车来回摆。 */
    g_track.filtered_position = 0.65f * g_track.filtered_position +
                                0.35f * g_track.raw_position;
    target = TRACK_KP * g_track.filtered_position +
             TRACK_KD * (g_track.filtered_position - g_track.last_position);
    g_track.last_position = g_track.filtered_position;

    target = limit_float(target, -CORRECTION_MAX, CORRECTION_MAX);
    change = limit_float(target - g_track.correction,
                         -CORRECTION_STEP, CORRECTION_STEP);
    g_track.correction += change;

    /* 黑线偏左时左轮慢、右轮快；黑线偏右时相反。 */
    g_track.left_speed = (int16_t)(g_track.base_speed + g_track.correction);
    g_track.right_speed = (int16_t)(g_track.base_speed - g_track.correction);
    if (g_track.left_speed > MOTOR_SPEED_MAX) g_track.left_speed = MOTOR_SPEED_MAX;
    if (g_track.left_speed < MOTOR_SPEED_MIN) g_track.left_speed = MOTOR_SPEED_MIN;
    if (g_track.right_speed > MOTOR_SPEED_MAX) g_track.right_speed = MOTOR_SPEED_MAX;
    if (g_track.right_speed < MOTOR_SPEED_MIN) g_track.right_speed = MOTOR_SPEED_MIN;
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
        if (g_track.filtered_position < 0.0f) {
            Motor_SetSpeed(FORWARD_SIGN * (-SEARCH_SPEED),
                           FORWARD_SIGN * SEARCH_SPEED);
        } else {
            Motor_SetSpeed(FORWARD_SIGN * SEARCH_SPEED,
                           FORWARD_SIGN * (-SEARCH_SPEED));
        }
    } else {
        Motor_Brake();
        g_track.state = RING_TRACK_FAULT;
    }
}

bool RingTrack_CheckFinish(uint16_t period_ms)
{
    if (g_track.black_count < MARK_SENSOR_COUNT) {
        g_track.mark_ms = 0U;
        if (g_track.distance > (LAP_PULSES / 20L)) {
            g_track.left_start_mark = true;
        }
        return false;
    }

    /* 必须已经离开起点，且至少走到预估一圈距离的80%。 */
    if (!g_track.left_start_mark ||
        g_track.distance < ((LAP_PULSES * 8L) / 10L)) {
        return false;
    }

    if (g_track.mark_ms < MARK_CONFIRM_MS) g_track.mark_ms += period_ms;
    if (g_track.mark_ms < MARK_CONFIRM_MS) return false;

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
