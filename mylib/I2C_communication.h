#ifndef I2C_COMMUNICATION_H_
#define I2C_COMMUNICATION_H_

/*
 * I2C Communication Layer for MSPM0G3507 (Blocking Mode)
 *
 * Dependencies:
 *   - ti_msp_dl_config.h (must define I2C_1_INST)
 *   - SysConfig I2C_1 as Controller
 */

#include <stdint.h>
#include "ti_msp_dl_config.h"

/* Write count bytes to device register(s).
 * Returns: 0 = OK, -1 = NACK */
int8_t I2C_WriteReg(uint8_t addr, uint8_t reg_addr,
                    const uint8_t *reg_data, uint8_t count);

/* Read count consecutive bytes from device register(s).
 * Returns: 0 = OK, -1 = NACK on TX, -2 = NACK/timeout on RX */
int8_t I2C_ReadReg(uint8_t addr, uint8_t reg_addr,
                   uint8_t *reg_data, uint8_t count);

#endif /* I2C_COMMUNICATION_H_ */
