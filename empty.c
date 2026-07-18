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
#include <stdio.h>
#include <math.h>


#define PI 3.14159
//定义任务状态结构体
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


//参数定义区
    static float kp = 0.8;
    static float kd = 0.14;
    static float ir_last_err = 0;

    static int Timer_count = 0;

    static float base_speed = 50;

float IR_PID_Control(float err);

void stop(void);
void task1(void);
void task2(void);
void task3(void);
void task4(void);


//==========  全局变量定义区 =============//

//任务1区域：




/* ==================== 主函数 ==================== */
int main(void)
{
    SYSCFG_DL_init();

    //初始状态
    Cartask state = status_stop;

    OLED_Init();
    /* 初始化编码器 */
    Motor_Encoder_Init();
    Motor_ResetLeftEncoder();
    Motor_ResetRightEncoder();
    /* 使能TIMER_count的NVIC中断（sysconfig漏掉了这一步）
     * 否则TIMG7_IRQHandler永远不会被调用 */
    NVIC_EnableIRQ(TIMER_count_INST_INT_IRQN);
    NVIC_EnableIRQ(DL_TIMERG_INTERRUPT_LOAD_EVENT);

    uint8_t ir[IR_NUM];

    uint8_t key_status = 0;
   
    while (1) {
      //  printf("right encoder: %d\r\n",Motor_GetRightEncoderPosition());
      //  printf("left encoder : %d\r\n",Motor_GetLeftEncoderPosition());
      //  printf("GB2 :%d\r\n",DL_GPIO_readPins(Motor_GB2_PORT,Motor_GB2_PIN));
      
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
                task2();
                break;
            case(status_task3):
                task3();
                break;
            case(status_task4):
                task4();
                break;

        }
       
    //    IR_Read(ir);//读取循迹
    //    float err = IR_GetError(ir);//计算误差
    //    printf("Error: %f\r\n", err);
       
    //    float correction = IR_PID_Control(err);
    // float k = 0.8;//差速控制
    // float diff = k* correction *fabs(correction);
    
    // if(diff > 40 )diff = 40;
    // if(diff < -40)diff = -40;
    // printf("diff : %f\r\n",diff);
    // float base = 40 -1 *fabs(err);
    // if (base <15) base = 15;

    // int right = base -diff;
    // int left = base +diff;
    // Motor_SetSpeed(left, right);

    //    printf("Left Speed:  %d pulse/s\r\n", Motor_GetLeftEncoderSpeed());
    //     printf("Right Speed: %d pulse/s\r\n", Motor_GetRightEncoderSpeed());
    //    printf("correction : %f\r\n",correction);
    // delay_ms(100);
    


    // if(IR_GetSensorCount(ir) == 0)
    // {
    //     if(Timer_count >2000){
    //     Motor_Disable();
    //     break;
    //     }
    //     Timer_count =0;
    // }
        



    }
}

/* 简介: TIMER_count中断处理函数（50ms定时器）
 * 功能: 每50ms自动调用Motor_Encoder_UpdateSpeed()更新编码器速度和位置
 * 说明: sysconfig中TIMER_count已配置为5ms周期、自动启动 */
void TIMG7_IRQHandler(void)
{
    DL_TimerG_clearInterruptStatus(TIMER_count_INST, DL_TIMERG_INTERRUPT_LOAD_EVENT);
    Motor_Encoder_UpdateSpeed();
    Timer_count++;
}

float IR_PID_Control(float err)
{
        float output = kp * err + kd *(err- ir_last_err);
        ir_last_err = err;
        return output;
}

void stop(void)
{
    Motor_Disable();//关闭电机
   // printf("current_left pos:%d\r\n",Motor_GetLeftEncoderPosition());

    //LED快闪
    DL_GPIO_togglePins(LED_PORT,LED_PIN22_PIN);
    delay_ms(100);
}

//任务一：直线行驶
void task1(void)
{
    //Motor_Encoder_Init();
    static float wheel_c = 0;
    wheel_c = PI * 6.5;//轮子周长
    
    //重置编码器的值
    

    //开启电机
    //获取已经前进的距离 
   
    static float task1_current_wheel_c = 0.0f;
    static float task1_target = 100.0f; 
    static int task1_last_left_pos = 0;
    int current_left_pos = Motor_GetLeftEncoderPosition();
    static float task1_error = 0.0f;
    static float task1_last_error = 0.0f;
    int pos_delta = current_left_pos - task1_last_left_pos;
    static int acc_delta = 0;
    
    if(task1_current_wheel_c >=100.0f){Motor_Disable();}
    else{Motor_Enable();}
    
    printf("current_left pos:%d\r\n",Motor_GetLeftEncoderPosition());
    printf("last_pos: %d\r\n",task1_last_left_pos);
    
    acc_delta += pos_delta;
    printf("acc_delta = %d\r\n",acc_delta);
    if (Timer_count > 40)
    {
        Timer_count = 0;
        task1_current_wheel_c = (current_left_pos/300.0f)*wheel_c;

        acc_delta = 0;
    }
    

        task1_last_left_pos = current_left_pos;
    
    //printf(" chazhi:%d\r\n",current_left_pos- task1_last_left_pos);
    
    printf("current wheel_c:%f\r\n",task1_current_wheel_c);
    
     task1_error = task1_target - task1_current_wheel_c;//获取误差
    
    float task1_kp = 0.8;
    float task1_kd = 0.14;
    //float task1_ki = 0;
    float task1_output = task1_kp * task1_error + task1_kd * (task1_error - 0);
    task1_last_error = task1_error;//更新误差

    if(task1_output >70)task1_output = 70;
    if(task1_output <15)task1_output = 15;
    //输出控制
    Motor_SetSpeed(task1_output,task1_output);
    
    

   
    printf("task1_output : %f\r\n",task1_output);

    delay_ms(50);


}

void task2(void)
{

    
}

void task3(void)
{


}

void task4(void)
{


}

