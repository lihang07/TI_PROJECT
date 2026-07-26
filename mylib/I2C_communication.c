/*
 * I2C Communication Layer for MSPM0G3507 (Blocking Mode)
 *
 * Dependencies:
 *   - ti_msp_dl_config.h (must define I2C_1_INST)
 *   - SysConfig I2C_0 as Controller
 */

#include "I2C_communication.h"

/* I2C_WriteReg: 向I2C设备指定寄存器写入数据
 * 参数:
 *   addr      - I2C设备从机地址(7位)
 *   reg_addr  - 寄存器地址
 *   reg_data  - 要写入的数据缓冲区
 *   count     - 写入字节数(最大8字节,因为txBuf大小为9)
 * 实现:
 *   1. 将寄存器地址和数据组合成发送缓冲区
 *   2. 等待I2C总线空闲
 *   3. 填充TX FIFO并启动发送传输
 *   4. 等待传输完成
 */
void I2C_WriteReg(uint8_t addr, uint8_t reg_addr,
                   const uint8_t *reg_data, uint8_t count)
{
    uint8_t txBuf[9];
    uint8_t totalLen = count + 1;
    uint8_t i;

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

    while (DL_I2C_getControllerStatus(I2C_1_INST)
           & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) {}
    while (!(DL_I2C_getControllerStatus(I2C_1_INST)
             & DL_I2C_CONTROLLER_STATUS_IDLE)) {}

    DL_I2C_flushControllerTXFIFO(I2C_1_INST);
}

/* I2C_ReadReg: 从I2C设备指定寄存器读取数据
 * 参数:
 *   addr      - I2C设备从机地址(7位)
 *   reg_addr  - 寄存器地址
 *   reg_data  - 读取数据存放缓冲区
 *   count     - 读取字节数
 * 实现:
 *   1. 先发送寄存器地址(写操作)
 *   2. 切换为接收模式
 *   3. 循环从RX FIFO读取数据
 * 注意: ICM42688的I2C读取需要先写入要读取的寄存器地址,
 *       然后重新启动读取操作,这是一种常见的I2C寄存器读取模式
 */
void I2C_ReadReg(uint8_t addr, uint8_t reg_addr,
                  uint8_t *reg_data, uint8_t count)
{
    uint8_t i;

    DL_I2C_flushControllerTXFIFO(I2C_1_INST);

    while (!(DL_I2C_getControllerStatus(I2C_1_INST)
             & DL_I2C_CONTROLLER_STATUS_IDLE)) {}

    DL_I2C_fillControllerTXFIFO(I2C_1_INST, &reg_addr, 1);

    DL_I2C_startControllerTransfer(
        I2C_1_INST, addr, DL_I2C_CONTROLLER_DIRECTION_TX, 1);

    while (DL_I2C_getControllerStatus(I2C_1_INST)
           & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) {}
    while (!(DL_I2C_getControllerStatus(I2C_1_INST)
             & DL_I2C_CONTROLLER_STATUS_IDLE)) {}

    DL_I2C_flushControllerTXFIFO(I2C_1_INST);

    DL_I2C_startControllerTransfer(
        I2C_1_INST, addr, DL_I2C_CONTROLLER_DIRECTION_RX, count);

    for (i = 0; i < count; i++) {
        while (DL_I2C_isControllerRXFIFOEmpty(I2C_1_INST)) {}
        reg_data[i] = DL_I2C_receiveControllerData(I2C_1_INST);
    }
}