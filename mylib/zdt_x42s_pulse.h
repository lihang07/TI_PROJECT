#ifndef ZDT_X42S_PULSE_H
#define ZDT_X42S_PULSE_H

#include "ti_msp_dl_config.h"
#include <stdbool.h>
#include <stdint.h>

/* SysConfig binding.  Override these macros before including this header when
 * migrating the library to another MSPM0 project. */
#ifndef ZDT_X42S_STEP_INST
#define ZDT_X42S_STEP_INST              BJ_INST
#endif
#ifndef ZDT_X42S_STEP_CLK_HZ
#define ZDT_X42S_STEP_CLK_HZ            BJ_INST_CLK_FREQ
#endif
#ifndef ZDT_X42S_STEP_CC_INDEX
#define ZDT_X42S_STEP_CC_INDEX          DL_TIMER_CC_1_INDEX
#endif
#ifndef ZDT_X42S_DIR_PORT
#define ZDT_X42S_DIR_PORT               BJin_DIR_PORT
#endif
#ifndef ZDT_X42S_DIR_PIN
#define ZDT_X42S_DIR_PIN                BJin_DIR_PIN
#endif
#ifndef ZDT_X42S_EN_PORT
#define ZDT_X42S_EN_PORT                BJin_EN_PORT
#endif
#ifndef ZDT_X42S_EN_PIN
#define ZDT_X42S_EN_PIN                 BJin_EN_PIN
#endif

/* X42S manual: EN is high-active by default.  Change to 0 only after the
 * motor menu has been configured for low-active EN. */
#ifndef ZDT_X42S_EN_ACTIVE_HIGH
#define ZDT_X42S_EN_ACTIVE_HIGH         1
#endif

/* Change this if a positive pulse count tilts the pendulum in the unwanted
 * direction. */
#ifndef ZDT_X42S_DIR_POSITIVE_HIGH
#define ZDT_X42S_DIR_POSITIVE_HIGH      1
#endif

typedef struct {
    int32_t minimum_pulse;
    int32_t maximum_pulse;
    uint32_t maximum_rate_hz;
    uint32_t acceleration_pulse_s2;
    int32_t command_pulse;
    int32_t target_pulse;
    int32_t fractional_pulse_q16;
    int32_t signed_rate_hz;
    bool enabled;
} ZDT_X42S_Pulse;

void ZDT_X42S_Pulse_Init(ZDT_X42S_Pulse *motor,
                          int32_t minimum_pulse,
                          int32_t maximum_pulse,
                          uint32_t maximum_rate_hz,
                          uint32_t acceleration_pulse_s2,
                          int32_t zero_pulse);
void ZDT_X42S_Pulse_Enable(ZDT_X42S_Pulse *motor, bool enable);
void ZDT_X42S_Pulse_SetTarget(ZDT_X42S_Pulse *motor, int32_t target_pulse);
void ZDT_X42S_Pulse_Update1ms(ZDT_X42S_Pulse *motor);
void ZDT_X42S_Pulse_Hold(ZDT_X42S_Pulse *motor);
void ZDT_X42S_Pulse_SetCurrentPosition(ZDT_X42S_Pulse *motor, int32_t pulse);
int32_t ZDT_X42S_Pulse_GetTarget(const ZDT_X42S_Pulse *motor);
int32_t ZDT_X42S_Pulse_GetCommand(const ZDT_X42S_Pulse *motor);

#endif
