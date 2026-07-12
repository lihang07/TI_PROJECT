/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"
#include "mylib/Motor.h"
#include "mylib/delay.h"
#include "mylib/key.h"
#include "mylib/find.h"
#include "mylib/usart.h"
int main(void)
{
    
    SYSCFG_DL_init();
 
    uint8_t ir[8] ;
    uint8_t a = 0;
 //   DL_GPIO_initDigitalInput(XG_G8_IOMUX);
//     int i = 0;
//    Key_t key1 = {KEY_PORT,KEY_PIN1_PIN,1};
    while (1) {
    // if(key_scan(&key1)==1)
    // {
    //     if (i == 0 )i= 1;
    //     else if(i == 1) i = 0;
    // }    

    // if(i == 0)
    // {
    //     Motor_forward();
    //     Motor_ON_PWM();
    // }
    // else {
    // Motor_backward();
    // Motor_ON_PWM();
    // }
 
    // if(( DL_GPIO_readPins(XG_G8_PORT,XG_G8_PIN))==0)
    // {
    //     DL_GPIO_setPins(LED_PORT,LED_PIN22_PIN);
    //     delay_ms(100);
    //     DL_GPIO_clearPins(LED_PORT,LED_PIN22_PIN);
    //     delay_ms(100);
        
    // }
 //   UART_SendByte(UART_XG_INST,a);
 //   UART_SendByte(UART_XG_INST,'A');
    IR_Read(ir);
    UART_SendBytes(UART_XG_INST,ir,8);
    delay_ms(100);


    
    }
}
