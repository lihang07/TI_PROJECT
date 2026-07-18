#include "ti_msp_dl_config.h"

#ifndef __DELAY_H
#define __DELAY_H

#include <stdint.h>


//ms延迟
void delay_ms(uint32_t ms);
//us延迟
void delay_us(uint32_t us);

#endif