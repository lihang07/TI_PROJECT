#include "ti_msp_dl_config.h"
#include "mylib/Motor.h"
#include "mylib/key.h"
#include "mylib/delay.h"
#include "mylib/usart.h"
#include "mylib/find.h"
#include "mylib/oled.h"
#include "mylib/icm42688.h"
#include "mylib/IMU.h"
#include "mylib/I2C_communication.h"
#include "mylib/pid.h"
#include "mylib/zdt_x42s_pulse.h"
#include <stdio.h>
#include <math.h>

/* X42S pulse-interface bench test.
 * 40 pulses is 4.5 motor degrees with the manual's default 1/16 subdivision.
 * Keep these values small until the linkage direction and travel are verified. */
#define ZDT_TEST_MODE                 1
#define ZDT_TEST_LIMIT_PULSE          80
#define ZDT_TEST_MOVE_PULSE           40
#define ZDT_TEST_MAX_RATE_HZ           400U
#define ZDT_TEST_ACCEL_PULSE_S2       1000U


typedef enum 
{
    status_stop,
    status_task1,
    status_task2,
    status_task3,
    status_task4
}Cartask;

/* 重定向printf到串口 */
int fputc(int ch, FILE *stream)
{
    while (DL_UART_isBusy(UART_XG_INST) == true) {}
    DL_UART_Main_transmitData(UART_XG_INST, ch);
    return ch;
}

int fputs(const char *restrict s, FILE *restrict stream)
{
    uint16_t len = 0;
    while (*s) {
        while (DL_UART_isBusy(UART_XG_INST) == true) {}
        DL_UART_Main_transmitData(UART_XG_INST, *s++);
        len++;
    }
    return len;
}

int puts(const char *_ptr) {
    while (*_ptr) {
        while (DL_UART_isBusy(UART_XG_INST) == true) {}
        DL_UART_Main_transmitData(UART_XG_INST, *_ptr++);
    }
    while (DL_UART_isBusy(UART_XG_INST) == true) {}
    DL_UART_Main_transmitData(UART_XG_INST, '\n');
    return 0;
}

static volatile int Timer_count = 0;
static volatile uint32_t TIMA0_count = 0;
static float g_imu_ypr[3];
static ZDT_X42S_Pulse g_zdt_x42s;

void stop(void);
void task1(void);
void task2(void);
void task3(void);
void task4(void);

//临时函数任务区
void task5(void);
void task6(void);
void task7(void);
void task8(void);


int main(void)
{
    SYSCFG_DL_init();
    


    /* 使能TIMER_count的NVIC中断（sysconfig漏掉了这一步）
     * 否则TIMG7_IRQHandler永远不会被调用 */
    NVIC_EnableIRQ(TIMER_count_INST_INT_IRQN);
    NVIC_EnableIRQ(DL_TIMERG_INTERRUPT_LOAD_EVENT);
    NVIC_EnableIRQ(TIMER_TICK_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

#if ZDT_TEST_MODE
    /* The motor remains disabled after reset.  Key 1 enables it; Keys 2/3
       request small positive/negative moves; Key 4 returns to the zero pulse. */
    ZDT_X42S_Pulse_Init(&g_zdt_x42s,
                         -ZDT_TEST_LIMIT_PULSE,
                         ZDT_TEST_LIMIT_PULSE,
                         ZDT_TEST_MAX_RATE_HZ,
                         ZDT_TEST_ACCEL_PULSE_S2,
                         0);

    while (1) {
        if (key_scan(&key1)) {
            ZDT_X42S_Pulse_Enable(&g_zdt_x42s, !g_zdt_x42s.enabled);
        }
        if (key_scan(&key2)) {
            ZDT_X42S_Pulse_SetTarget(&g_zdt_x42s, ZDT_TEST_MOVE_PULSE);
        }
        if (key_scan(&key3)) {
            ZDT_X42S_Pulse_SetTarget(&g_zdt_x42s, -ZDT_TEST_MOVE_PULSE);
        }
        if (key_scan(&key4)) {
            ZDT_X42S_Pulse_SetTarget(&g_zdt_x42s, 0);
        }
    }
#else
    
    uint8_t key_status = 0;
    Cartask state = status_stop;

 
    while (1) {

       

        key_status = key_read();
        if (key_status == 0) state = status_stop;
        else if (key_status == 1) state = status_task1;
        else if (key_status == 2) state = status_task2;
        else if (key_status == 3) state = status_task3;
        else if (key_status == 4) state = status_task4;
       
        switch(state)
        {
            case(status_stop):
            
                stop();
                break;
            case(status_task1):
                task1();
                break;
            case(status_task2):
                task6();
                break;
            case(status_task3):
               
                task4();
                break;
            case(status_task4):
                task8();
                break;

        }
       



    }
#endif
}


void TIMG7_IRQHandler(void)
{
    DL_TimerG_clearInterruptStatus(TIMER_count_INST, DL_TIMERG_INTERRUPT_LOAD_EVENT);
    Motor_Encoder_UpdateSpeed();
    Timer_count++;

}

void TIMA0_IRQHandler(void)
{
    DL_TimerA_clearInterruptStatus(TIMER_TICK_INST, DL_TIMERA_INTERRUPT_LOAD_EVENT);
    TIMA0_count++;
#if ZDT_TEST_MODE
    ZDT_X42S_Pulse_Update1ms(&g_zdt_x42s);
#endif


}

void TIMER_0_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(TIMER_0_INST)) {
    case DL_TIMER_IIDX_ZERO:
        IMU_getYawPitchRoll((float *)g_imu_ypr);  // 获取当前姿态角
        break;
    default:
        break;
    }
}


void stop(void)
{

}






void task2(void)
{

}


