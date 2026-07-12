#include "ti_msp_dl_config.h"
#include "mylib/usart.h"

void UART_SendByte(UART_Regs *uart, uint8_t data) {
    // 使用 DriverLib 的底层函数，判断传入的串口 FIFO 是否满
    while (DL_UART_Main_isTXFIFOFull(uart));
    DL_UART_Main_transmitData(uart, data);
}

void MY_UART_SendString(UART_Regs *uart, char *str) {
    while (*str) {
        UART_SendByte(uart, *str++);
    }
}

void UART_SendBytes(UART_Regs *uart, uint8_t *data, uint16_t length) {
    for (uint16_t i = 0; i < length; i++) {
        // 等待 FIFO 腾出空间
        while (DL_UART_Main_isTXFIFOFull(uart));
        // 发送数组的第 i 个字节
        DL_UART_Main_transmitData(uart, data[i]);
    }
}
