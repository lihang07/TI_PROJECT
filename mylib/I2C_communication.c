/*
 * I2C Communication Layer for MSPM0G3507 (Blocking Mode)
 *
 * Dependencies:
 *   - ti_msp_dl_config.h (must define I2C_1_INST)
 *   - SysConfig I2C_1 as Controller
 */

#include "I2C_communication.h"
#include <stdio.h>

/* NACK bit in MSTAT register (bit 3) */
#define I2C_MSTAT_NACK_MASK  0x08U
#define I2C_RX_TIMEOUT       100000UL

/*
 * I2C_WriteReg: Write register(s) to I2C device.
 * Returns: 0 = success (ACK), -1 = NACK (device not responding)
 */
int8_t I2C_WriteReg(uint8_t addr, uint8_t reg_addr,
                    const uint8_t *reg_data, uint8_t count)
{
    uint8_t txBuf[9];
    uint8_t totalLen = count + 1;
    uint8_t i;
    uint8_t status;
    uint32_t timeout;

    txBuf[0] = reg_addr;
    for (i = 0; i < count; i++) {
        txBuf[i + 1] = reg_data[i];
    }

    DL_I2C_flushControllerTXFIFO(I2C_1_INST);

    while (!(DL_I2C_getControllerStatus(I2C_1_INST)
             & DL_I2C_CONTROLLER_STATUS_IDLE)) {}

    DL_I2C_fillControllerTXFIFO(I2C_1_INST, txBuf, totalLen);

    DL_I2C_startControllerTransfer(I2C_1_INST, addr,
        DL_I2C_CONTROLLER_DIRECTION_TX, totalLen);

    timeout = I2C_RX_TIMEOUT;
    while (DL_I2C_getControllerStatus(I2C_1_INST)
           & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) {
        if (--timeout == 0) {
            printf("I2C W TX timeout! a=0x%02X r=0x%02X\r\n", addr, reg_addr);
            DL_I2C_flushControllerTXFIFO(I2C_1_INST);
            return -1;
        }
    }

    timeout = I2C_RX_TIMEOUT;
    while (!((status = DL_I2C_getControllerStatus(I2C_1_INST))
             & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--timeout == 0) {
            printf("I2C W IDLE timeout! a=0x%02X r=0x%02X\r\n", addr, reg_addr);
            DL_I2C_flushControllerTXFIFO(I2C_1_INST);
            return -1;
        }
    }

    if (status & I2C_MSTAT_NACK_MASK) {
        printf("I2C W NACK! a=0x%02X r=0x%02X\r\n", addr, reg_addr);
        DL_I2C_flushControllerTXFIFO(I2C_1_INST);
        return -1;
    }

    DL_I2C_flushControllerTXFIFO(I2C_1_INST);
    return 0;
}

/*
 * I2C_ReadReg: Read register(s) from I2C device.
 * Returns: 0 = success, -1 = NACK on TX, -2 = NACK/timeout on RX
 */
int8_t I2C_ReadReg(uint8_t addr, uint8_t reg_addr,
                   uint8_t *reg_data, uint8_t count)
{
    uint8_t i;
    uint8_t status;
    uint32_t timeout;

    DL_I2C_flushControllerTXFIFO(I2C_1_INST);

    while (!(DL_I2C_getControllerStatus(I2C_1_INST)
             & DL_I2C_CONTROLLER_STATUS_IDLE)) {}

    DL_I2C_fillControllerTXFIFO(I2C_1_INST, &reg_addr, 1);

    DL_I2C_startControllerTransfer(I2C_1_INST, addr,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1);

    while (DL_I2C_getControllerStatus(I2C_1_INST)
           & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) {}

    while (!((status = DL_I2C_getControllerStatus(I2C_1_INST))
             & DL_I2C_CONTROLLER_STATUS_IDLE)) {}

    if (status & I2C_MSTAT_NACK_MASK) {
        printf("I2C R NACK(TX)! a=0x%02X r=0x%02X\r\n", addr, reg_addr);
        DL_I2C_flushControllerTXFIFO(I2C_1_INST);
        return -1;
    }

    DL_I2C_flushControllerTXFIFO(I2C_1_INST);

    DL_I2C_startControllerTransfer(I2C_1_INST, addr,
        DL_I2C_CONTROLLER_DIRECTION_RX, count);

    for (i = 0; i < count; i++) {
        timeout = 0;
        while (DL_I2C_isControllerRXFIFOEmpty(I2C_1_INST)) {
            if (DL_I2C_getControllerStatus(I2C_1_INST)
                & DL_I2C_CONTROLLER_STATUS_IDLE) {
                printf("I2C R NACK(RX)! no data\r\n");
                return -2;
            }
            if (++timeout > I2C_RX_TIMEOUT) {
                printf("I2C R TIMEOUT! RX stuck\r\n");
                return -2;
            }
        }
        reg_data[i] = DL_I2C_receiveControllerData(I2C_1_INST);
    }
    return 0;
}